/*
 * spidev_module.c - Python bindings for Linux SPI access through spidev
 *
 * MIT License
 *
 * Copyright (C) 2009 Volker Thoms <unconnected@gmx.de>
 * Copyright (C) 2012 Stephen Caudle <scaudle@doceme.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Python.h>
#include "structmember.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <time.h>

#define _VERSION_ "3.4-0.2"
#define SPIDEV_MAXPATH 4096

#define BLOCK_SIZE_CONTROL_FILE "/sys/module/spidev/parameters/xabufsiz"
// The xfwr3 function attempts to use large blocks if /sys/module/spidev/parameters/bufsiz setting allows it.
// However where we cannot get a value from that file, we fall back to this safe default.
#define XFER3_DEFAULT_BLOCK_SIZE SPIDEV_MAXPATH
// Largest block size for xfer3 - even if /sys/module/spidev/parameters/bufsiz allows bigger
// blocks, we won't go above this value. As I understand, DMA is not used for anything bigger so why bother.
#define XFER3_MAX_BLOCK_SIZE 65535


#if PY_MAJOR_VERSION < 3
#define PyLong_AS_LONG(val) PyInt_AS_LONG(val)
#define PyLong_AsLong(val) PyInt_AsLong(val)
#endif

// Macros needed for Python 3
#ifndef PyInt_Check
#define PyInt_Check			PyLong_Check
#define PyInt_FromLong	PyLong_FromLong
#define PyInt_AsLong		PyLong_AsLong
#define PyInt_Type			PyLong_Type
#endif

// Maximum block size for xfer3
// Initialised once by get_xfer3_block_size
uint32_t xfer3_block_size = 0;

// Read maximum block size from the /sys/module/spidev/parameters/bufsiz
// In case of any problems reading the number, we fall back to XFER3_DEFAULT_BLOCK_SIZE.
// If number is read ok but it exceeds the XFER3_MAX_BLOCK_SIZE, it will be capped to that value.
// The value is read and cached on the first invocation. Following invocations just return the cached one.
uint32_t get_xfer3_block_size(void) {
	int value;

	// If value was already initialised, just use it
	if (xfer3_block_size != 0) {
		return xfer3_block_size;
	}

	// Start with the default
	xfer3_block_size = XFER3_DEFAULT_BLOCK_SIZE;

	FILE *file = fopen(BLOCK_SIZE_CONTROL_FILE,"r");
	if (file != NULL) {
		if (fscanf(file, "%d", &value) == 1 && value > 0) {
			if (value <= XFER3_MAX_BLOCK_SIZE) {
				xfer3_block_size = value;
			} else {
				xfer3_block_size = XFER3_MAX_BLOCK_SIZE;
			}
		}
		fclose(file);
	}

	return xfer3_block_size;
}

PyDoc_STRVAR(XaSpiDev_module_doc,
	"This module defines an object type that allows SPI transactions\n"
	"on hosts running the Linux kernel. The host kernel must have SPI\n"
	"support and SPI device interface support.\n"
	"All of these can be either built-in to the kernel, or loaded from\n"
	"modules.\n"
	"\n"
	"Because the SPI device interface is opened R/W, users of this\n"
	"module usually must have root permissions.\n");

typedef struct {
	PyObject_HEAD

	int fd;	/* open file descriptor: /dev/spidevX.Y */
	uint8_t mode;	/* current SPI mode */
	uint8_t bits_per_word;	/* current SPI bits per word setting */
	uint32_t max_speed_hz;	/* current SPI max speed setting in Hz */
	uint32_t xa_blocksize;	/* current FIFO Block size in bytes */
} XaSpiDevObject;

static PyObject *
XaSpiDev_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	XaSpiDevObject *self;
	if ((self = (XaSpiDevObject *)type->tp_alloc(type, 0)) == NULL)
		return NULL;

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;
	self->xa_blocksize = 3072;

	Py_INCREF(self);
	return (PyObject *)self;
}

PyDoc_STRVAR(XaSpiDev_close_doc,
	"close()\n\n"
	"Disconnects the object from the interface.\n");

static PyObject *
XaSpiDev_close(XaSpiDevObject *self)
{
	if ((self->fd != -1) && (close(self->fd) == -1)) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;
	self->xa_blocksize = 3072;

	Py_INCREF(Py_None);
	return Py_None;
}

static void
XaSpiDev_dealloc(XaSpiDevObject *self)
{
	PyObject *ref = XaSpiDev_close(self);
	Py_XDECREF(ref);

	Py_TYPE(self)->tp_free((PyObject *)self);
}

static char *wrmsg_list0 = "Empty argument list.";
static char *wrmsg_listmax = "Argument list size exceeds %d bytes.";
static char *wrmsg_val = "Non-Int/Long value in arguments: %x.";
static char *wrmsg_oom = "Out of memory.";


PyDoc_STRVAR(XaSpiDev_write_doc,
	"write([values]) -> None\n\n"
	"Write bytes to SPI device.\n");

static PyObject *
XaSpiDev_writebytes(XaSpiDevObject *self, PyObject *args)
{
	int		status;
	uint16_t	ii, len;
	uint8_t	buf[SPIDEV_MAXPATH];
	PyObject	*obj;
	PyObject	*seq;
	char	wrmsg_text[4096];

	if (!PyArg_ParseTuple(args, "O:write", &obj))
		return NULL;

	seq = PySequence_Fast(obj, "expected a sequence");
	len = PySequence_Fast_GET_SIZE(seq);
	if (!seq || len <= 0) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	if (len > SPIDEV_MAXPATH) {
		snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_listmax, SPIDEV_MAXPATH);
		PyErr_SetString(PyExc_OverflowError, wrmsg_text);
		return NULL;
	}

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, ii);
#if PY_MAJOR_VERSION < 3
		if (PyInt_Check(val)) {
			buf[ii] = (__u8)PyInt_AS_LONG(val);
		} else
#endif
		{
			if (PyLong_Check(val)) {
				buf[ii] = (__u8)PyLong_AS_LONG(val);
			} else {
				snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_val, val);
				PyErr_SetString(PyExc_TypeError, wrmsg_text);
				return NULL;
			}
		}
	}

	Py_DECREF(seq);

	status = write(self->fd, &buf[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short write");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

PyDoc_STRVAR(XaSpiDev_read_doc,
	"read(len) -> [values]\n\n"
	"Read len bytes from SPI device.\n");

static PyObject *
XaSpiDev_readbytes(XaSpiDevObject *self, PyObject *args)
{
	uint8_t	rxbuf[SPIDEV_MAXPATH];
	int		status, len, ii;
	PyObject	*list;

	if (!PyArg_ParseTuple(args, "i:read", &len))
		return NULL;

	/* read at least 1 byte, no more than SPIDEV_MAXPATH */
	if (len < 1)
		len = 1;
	else if ((unsigned)len > sizeof(rxbuf))
		len = sizeof(rxbuf);

	memset(rxbuf, 0, sizeof rxbuf);
	status = read(self->fd, &rxbuf[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short read");
		return NULL;
	}

	list = PyList_New(len);

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PyList_SET_ITEM(list, ii, val);
	}

	return list;
}

static PyObject *
XaSpiDev_writebytes2_buffer(XaSpiDevObject *self, Py_buffer *buffer)
{
	int		status;
	Py_ssize_t	remain, block_size, block_start, spi_max_block;

	spi_max_block = get_xfer3_block_size();

	block_start = 0;
	remain = buffer->len;
	while (block_start < buffer->len) {
		block_size = (remain < spi_max_block) ? remain : spi_max_block;

		Py_BEGIN_ALLOW_THREADS
		status = write(self->fd, buffer->buf + block_start, block_size);
		Py_END_ALLOW_THREADS

		if (status < 0) {
			PyErr_SetFromErrno(PyExc_IOError);
			return NULL;
		}

		if (status != block_size) {
			perror("short write");
			return NULL;
		}

		block_start += block_size;
		remain -= block_size;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *
XaSpiDev_writebytes2_seq_internal(XaSpiDevObject *self, PyObject *seq, Py_ssize_t len, uint8_t *buf, Py_ssize_t bufsize)
{
	int		status;
	Py_ssize_t	ii, jj, remain, block_size;
	char	wrmsg_text[4096];

	remain = len;
	jj = 0;
	while (remain > 0) {
		block_size = (remain < bufsize) ? remain : bufsize;

		for (ii = 0; ii < block_size; ii++, jj++) {
			PyObject *val = PySequence_Fast_GET_ITEM(seq, jj);
#if PY_MAJOR_VERSION < 3
			if (PyInt_Check(val)) {
				buf[ii] = (__u8)PyInt_AS_LONG(val);
			} else
#endif
			{
				if (PyLong_Check(val)) {
					buf[ii] = (__u8)PyLong_AS_LONG(val);
				} else {
					snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_val, val);
					PyErr_SetString(PyExc_TypeError, wrmsg_text);
					return NULL;
				}
			}
		}

		Py_BEGIN_ALLOW_THREADS
		status = write(self->fd, buf, block_size);
		Py_END_ALLOW_THREADS

		if (status < 0) {
			PyErr_SetFromErrno(PyExc_IOError);
			return NULL;
		}

		if (status != block_size) {
			perror("short write");
			return NULL;
		}

		remain -= block_size;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// In writebytes2 we try to avoild doing malloc/free on each tiny block.
// So for any transfer below this size we will use on-stack local buffer instead of allocating one on the heap.
#define SMALL_BUFFER_SIZE 128

static PyObject *
XaSpiDev_writebytes2_seq(XaSpiDevObject *self, PyObject *seq)
{
	Py_ssize_t	len, bufsize, spi_max_block;
	PyObject	*result = NULL;

	len = PySequence_Fast_GET_SIZE(seq);
	if (len <= 0) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	spi_max_block = get_xfer3_block_size();

	bufsize = (len < spi_max_block) ? len : spi_max_block;

	if (bufsize <= SMALL_BUFFER_SIZE) {
		// The data size is very small so we can avoid malloc/free completely
		// by using a small local buffer instead
		uint8_t buf[SMALL_BUFFER_SIZE];
		result = XaSpiDev_writebytes2_seq_internal(self, seq, len, buf, SMALL_BUFFER_SIZE);
	} else {
		// Large data, need to allocate buffer on heap
		uint8_t	*buf;
		Py_BEGIN_ALLOW_THREADS
		buf = malloc(sizeof(__u8) * bufsize);
		Py_END_ALLOW_THREADS

		if (!buf) {
			PyErr_SetString(PyExc_OverflowError, wrmsg_oom);
			return NULL;
		}

		result = XaSpiDev_writebytes2_seq_internal(self, seq, len, buf, bufsize);

		Py_BEGIN_ALLOW_THREADS
		free(buf);
		Py_END_ALLOW_THREADS
	}

	return result;
}

PyDoc_STRVAR(XaSpiDev_writebytes2_doc,
	"writebytes2([values]) -> None\n\n"
	"Write bytes to SPI device.\n"
	"values must be a list or buffer.\n");

static PyObject *
XaSpiDev_writebytes2(XaSpiDevObject *self, PyObject *args)
{
	PyObject	*obj, *seq;;
	PyObject	*result = NULL;

	if (!PyArg_ParseTuple(args, "O:writebytes2", &obj)) {
		return NULL;
	}

	// Try using buffer protocol if object supports it.
	if (PyObject_CheckBuffer(obj) && 1) {
		Py_buffer	buffer;
		if (PyObject_GetBuffer(obj, &buffer, PyBUF_SIMPLE) != -1) {
			result = XaSpiDev_writebytes2_buffer(self, &buffer);
			PyBuffer_Release(&buffer);
			return result;
		}
	}


	// Otherwise, fall back to sequence protocol
	seq = PySequence_Fast(obj, "expected a sequence");
	if (seq == NULL) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	result = XaSpiDev_writebytes2_seq(self, seq);

	Py_DECREF(seq);

	return result;

}


uint32_t xa_get_wrspace(int fd, uint16_t delay_usecs, uint32_t speed_hz, uint8_t bits_per_word, uint8_t mode)
{
	int status;


	uint16_t len;
	struct spi_ioc_transfer xfer;

	Py_BEGIN_ALLOW_THREADS
	memset(&xfer, 0, sizeof(xfer));
	Py_END_ALLOW_THREADS

        uint8_t txbuf[3], rxbuf[3];

        uint32_t cnt;

	txbuf[0] = 0x88; txbuf[1] = 0x00; txbuf[2] = 0x00;
	len = 3;

	Py_BEGIN_ALLOW_THREADS
	xfer.tx_buf = (unsigned long)&txbuf;
	xfer.rx_buf = (unsigned long)&rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz;
	xfer.bits_per_word = bits_per_word;
	
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	Py_END_ALLOW_THREADS

	if (status < 0) {
		return -1;
	}

        cnt = rxbuf[2];

        txbuf[0] = 0x89;

	Py_BEGIN_ALLOW_THREADS
        xfer.tx_buf = (unsigned long)&txbuf;
        xfer.rx_buf = (unsigned long)&rxbuf;
        xfer.len = len;
        xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz;
	xfer.bits_per_word = bits_per_word;

        status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	Py_END_ALLOW_THREADS

        if (status < 0) {
                return -1;
        }

        cnt = cnt + rxbuf[2]*256;


	// in CS_HIGH mode CS isnt pulled to low after transfer
	// reading 0 bytes doesn't really matter but brings CS down
	// tomdean:
	// Stop generating an extra CS except in mode CS_HOGH
	if (mode & SPI_CS_HIGH) status = read(fd, &rxbuf[0], 0);

        return cnt;

}

uint32_t xa_get_rdavail(int fd, uint16_t delay_usecs, uint32_t speed_hz, uint8_t bits_per_word, uint8_t mode)
{
	int status;


	uint16_t len = 3;
	struct spi_ioc_transfer xfer;

	Py_BEGIN_ALLOW_THREADS
	memset(&xfer, 0, sizeof(xfer));
	Py_END_ALLOW_THREADS

        uint8_t txbuf[3], rxbuf[3];

        uint32_t cnt;

	txbuf[0] = 0x8A; txbuf[1] = 0x00; txbuf[2] = 0x00;

	Py_BEGIN_ALLOW_THREADS
	xfer.tx_buf = (unsigned long)&txbuf;
	xfer.rx_buf = (unsigned long)&rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz;
	xfer.bits_per_word = bits_per_word;
	
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	Py_END_ALLOW_THREADS

	if (status < 0) {
		return -1;
	}

        cnt = rxbuf[2];

        txbuf[0] = 0x8B;

	Py_BEGIN_ALLOW_THREADS
        xfer.tx_buf = (unsigned long)&txbuf;
        xfer.rx_buf = (unsigned long)&rxbuf;
        xfer.len = len;
        xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz;
	xfer.bits_per_word = bits_per_word;
	
        status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	Py_END_ALLOW_THREADS

        if (status < 0) {
                return -1;
        }

        cnt = cnt + rxbuf[2]*256;


	// in CS_HIGH mode CS isnt pulled to low after transfer
	// reading 0 bytes doesn't really matter but brings CS down
	// tomdean:
	// Stop generating an extra CS except in mode CS_HOGH
	if (mode & SPI_CS_HIGH) status = read(fd, &rxbuf[0], 0);

        return cnt;

}





PyDoc_STRVAR(XaSpiDev_xa_writebulk_doc,
	"xa_writebulk([values]) -> None\n\n"
	"Write bulk data to XAPIZ3500 SPI device.\n"
	"values must be a list or buffer.\n");

static PyObject *
XaSpiDev_xa_writebulk(XaSpiDevObject *self, PyObject *args)
{
	PyObject	*obj;
	Py_ssize_t	remain, block_size, block_start, spi_max_block;

	int		status;
	uint8_t 	firsttx[2];

	uint8_t		*bufptr;
	uint8_t		*cmdbufptr;

        //clock_t 	time0_t, time1_t, time2_t, time3_t, time4_t;
        //double 	diff_t;

        //time0_t = clock();

 	spi_max_block = self->xa_blocksize;


   	//printf("Debug : Begin\n");
	if (!PyArg_ParseTuple(args, "O:xa_writebulk", &obj)) {
		return NULL;
	}

	// Try using buffer protocol if object supports it.
	if (PyObject_CheckBuffer(obj) && 1) {
		Py_buffer	buffer;

		if (PyObject_GetBuffer(obj, &buffer, PyBUF_SIMPLE) != -1) {

			//Send 1 byte first
			while (xa_get_wrspace(self->fd, 0, self->max_speed_hz, self->bits_per_word, self->mode) < 1); 

			Py_INCREF(obj);
			//form the command with first byte
			bufptr = buffer.buf;
			firsttx[0] = 0x10;
			firsttx[1] = *bufptr;
			Py_DECREF(obj);


			status = write(self->fd, &firsttx, 2);
                	if (status < 0) {
                        	PyErr_SetFromErrno(PyExc_IOError);
                        	return NULL;
                	}

   			//printf("Debug : Write first byte Status :  %d\n", status);

			remain = buffer.len - 1;
			block_start = 1;
			while (block_start < buffer.len) {
				block_size = (remain < spi_max_block) ? remain : spi_max_block;
				//Insert cmd
			        Py_INCREF(obj);
				cmdbufptr = bufptr+block_start-1;
				*cmdbufptr = 0x10;
			        Py_DECREF(obj);

				while (xa_get_wrspace(self->fd, 0, self->max_speed_hz, self->bits_per_word, self->mode) < block_size); 
		                Py_BEGIN_ALLOW_THREADS
                		status = write(self->fd, buffer.buf + block_start-1, block_size+1);
                		Py_END_ALLOW_THREADS

                		if (status < 0) {
                        		PyErr_SetFromErrno(PyExc_IOError);
                        		return NULL;
                		}

                		if (status != block_size+1) {
                        		perror("short write");
                        		return NULL;
                		}

                		block_start += block_size;
                		remain -= block_size;

			}

			PyBuffer_Release(&buffer);

        		//time1_t = clock();
			//diff_t = (double)(time1_t - time0_t ) / CLOCKS_PER_SEC;
   			//printf("Bulk time = %f\n", diff_t);

			Py_INCREF(Py_None);
			return Py_None;
		}
	} 
        return NULL;
}

PyDoc_STRVAR(XaSpiDev_xa_readmeta_doc,
	"xa_readmeta([len]) -> None\n\n"
	"Read meta data from XAPIZ3500 SPI device.\n"
	"len must be a integer.\n");

static PyObject *
XaSpiDev_xa_readmeta(XaSpiDevObject *self)
{
	int		status, ii;
	uint32_t	rdavail_cnt;


	PyObject 	*rxlist;

        uint8_t 	*txbuf, *rxbuf;
        uint32_t 	meta_len;

        struct spi_ioc_transfer xfer;
        Py_BEGIN_ALLOW_THREADS
        memset(&xfer, 0, sizeof(xfer));
        Py_END_ALLOW_THREADS

        uint8_t		tx_lencmd[6];
        uint8_t		rx_lencmd[6];

	tx_lencmd[0] = 0xA0; tx_lencmd[1] = 0x00; tx_lencmd[2] = 0x00; 
	tx_lencmd[3] = 0x00; tx_lencmd[4] = 0x00; tx_lencmd[5] = 0x00; 

   	//printf("Debug0\n");


	rdavail_cnt =  xa_get_rdavail(self->fd, 0, self->max_speed_hz, self->bits_per_word, self->mode);


	if (rdavail_cnt < 4) {
	        rxlist = PyList_New(1);
		PyObject *val = Py_BuildValue("l", (long)tx_lencmd[2]); //Just reusing tx_lencmd[2]. Send 0.
                PyList_SET_ITEM(rxlist, 0, val);
		return rxlist;
	} else {
	//Read 4 bytes to get length of meta
   		//printf("Debug1\n");
	        Py_BEGIN_ALLOW_THREADS
        	xfer.tx_buf = (unsigned long)&tx_lencmd;
        	xfer.rx_buf = (unsigned long)&rx_lencmd;
        	xfer.len = 6;
        	xfer.delay_usecs = 0;
        	xfer.speed_hz = self->max_speed_hz;
        	xfer.bits_per_word = self->bits_per_word;
   		//printf("Debug2\n");

        	status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
        	Py_END_ALLOW_THREADS
        	if (status < 0) {
                	PyErr_SetFromErrno(PyExc_IOError);
                	return NULL;
        	}
		meta_len = ((rx_lencmd[2]) + (256*rx_lencmd[3]));
   		//printf("Debug3\n");
		if (meta_len == 0) {
			rxlist = PyList_New(2);
                	PyList_SET_ITEM(rxlist, 0, PyLong_FromLong(0));
                	PyList_SET_ITEM(rxlist, 1, PyLong_FromLong(0));

			return rxlist;
		}
   		//printf("Debug4\n");
		meta_len = meta_len*16;
   		//printf("Debug5\n");
	}

	//Now fetch the meta data based on meta_len

        Py_BEGIN_ALLOW_THREADS
        txbuf = malloc(sizeof(__u8) * (meta_len+2));
        rxbuf = malloc(sizeof(__u8) * (meta_len+2));
        Py_END_ALLOW_THREADS

   	//printf("Debug6\n");
	*txbuf = 0xA0;
   	//printf("Debug7\n");
	
        xfer.tx_buf = (unsigned long)txbuf;
        xfer.rx_buf = (unsigned long)rxbuf;
        xfer.len = meta_len+2;
        xfer.delay_usecs = 0;
        xfer.speed_hz = self->max_speed_hz;
        xfer.bits_per_word = self->bits_per_word;

	while (xa_get_rdavail(self->fd, 0, self->max_speed_hz, self->bits_per_word, self->mode) < meta_len);


        Py_BEGIN_ALLOW_THREADS
        status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
        Py_END_ALLOW_THREADS
   	//printf("Debug8\n");
        if (status < 0) {
                PyErr_SetFromErrno(PyExc_IOError);
                free(txbuf);
                free(rxbuf);
                return NULL;
        }
	rxlist = PyList_New(meta_len);
        for (ii = 0; ii < meta_len; ii++) {
                PyObject *val = Py_BuildValue("l", (long)rxbuf[ii+2]);
                PyList_SET_ITEM(rxlist, ii, val);
        }
   	//printf("Debug9\n");
        // WA:
        // in CS_HIGH mode CS isnt pulled to low after transfer
        // reading 0 bytes doesn't really matter but brings CS down
        // tomdean:
        // Stop generating an extra CS except in mode CS_HOGH
        if (self->mode & SPI_CS_HIGH) status = read(self->fd, &rxbuf[0], 0);

        Py_BEGIN_ALLOW_THREADS
        free(txbuf);
        free(rxbuf);
        Py_END_ALLOW_THREADS

        return rxlist;
}






PyDoc_STRVAR(XaSpiDev_xfer_doc,
	"xfer([values]) -> [values]\n\n"
	"Perform SPI transaction.\n"
	"CS will be released and reactivated between blocks.\n"
	"delay specifies delay in usec between blocks.\n");

static PyObject *
XaSpiDev_xfer(XaSpiDevObject *self, PyObject *args)
{
	uint16_t ii, len;
	int status;
	uint16_t delay_usecs = 0;
	uint32_t speed_hz = 0;
	uint8_t bits_per_word = 0;
	PyObject *obj;
	PyObject *seq;
#ifdef SPIDEV_SINGLE
	struct spi_ioc_transfer *xferptr;
	memset(&xferptr, 0, sizeof(xferptr));
#else
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
#endif
	uint8_t *txbuf, *rxbuf;
	char	wrmsg_text[4096];

	if (!PyArg_ParseTuple(args, "O|IHB:xfer", &obj, &speed_hz, &delay_usecs, &bits_per_word))
		return NULL;

	seq = PySequence_Fast(obj, "expected a sequence");
	len = PySequence_Fast_GET_SIZE(obj);
	if (!seq || len <= 0) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	if (len > SPIDEV_MAXPATH) {
		snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_listmax, SPIDEV_MAXPATH);
		PyErr_SetString(PyExc_OverflowError, wrmsg_text);
		return NULL;
	}

	txbuf = malloc(sizeof(__u8) * len);
	rxbuf = malloc(sizeof(__u8) * len);

#ifdef SPIDEV_SINGLE
	xferptr = (struct spi_ioc_transfer*) malloc(sizeof(struct spi_ioc_transfer) * len);

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, ii);
#if PY_MAJOR_VERSION < 3
		if (PyInt_Check(val)) {
			txbuf[ii] = (__u8)PyInt_AS_LONG(val);
		} else
#endif
		{
			if (PyLong_Check(val)) {
				txbuf[ii] = (__u8)PyLong_AS_LONG(val);
			} else {
				snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_val, val);
				PyErr_SetString(PyExc_TypeError, wrmsg_text);
				free(xferptr);
				free(txbuf);
				free(rxbuf);
				return NULL;
			}
		}
		xferptr[ii].tx_buf = (unsigned long)&txbuf[ii];
		xferptr[ii].rx_buf = (unsigned long)&rxbuf[ii];
		xferptr[ii].len = 1;
		xferptr[ii].delay_usecs = delay;
		xferptr[ii].speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
		xferptr[ii].bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;
#ifdef SPI_IOC_WR_MODE32
		xferptr[ii].tx_nbits = 0;
#endif
#ifdef SPI_IOC_RD_MODE32
		xferptr[ii].rx_nbits = 0;
#endif
	}

	status = ioctl(self->fd, SPI_IOC_MESSAGE(len), xferptr);
	free(xferptr);
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		free(txbuf);
		free(rxbuf);
		return NULL;
	}
#else
	for (ii = 0; ii < len; ii++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, ii);
#if PY_MAJOR_VERSION < 3
		if (PyInt_Check(val)) {
			txbuf[ii] = (__u8)PyInt_AS_LONG(val);
		} else
#endif
		{
			if (PyLong_Check(val)) {
				txbuf[ii] = (__u8)PyLong_AS_LONG(val);
			} else {
				snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_val, val);
				PyErr_SetString(PyExc_TypeError, wrmsg_text);
				free(txbuf);
				free(rxbuf);
				return NULL;
			}
		}
	}

	if (PyTuple_Check(obj)) {
		Py_DECREF(seq);
		seq = PySequence_List(obj);
	}

	xfer.tx_buf = (unsigned long)txbuf;
	xfer.rx_buf = (unsigned long)rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
	xfer.bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;
#ifdef SPI_IOC_WR_MODE32
	xfer.tx_nbits = 0;
#endif
#ifdef SPI_IOC_RD_MODE32
	xfer.rx_nbits = 0;
#endif

	status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		free(txbuf);
		free(rxbuf);
		return NULL;
	}
#endif

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PySequence_SetItem(seq, ii, val);
	}

	// WA:
	// in CS_HIGH mode CS isn't pulled to low after transfer, but after read
	// reading 0 bytes doesnt matter but brings cs down
	// tomdean:
	// Stop generating an extra CS except in mode CS_HOGH
	if (self->mode & SPI_CS_HIGH) status = read(self->fd, &rxbuf[0], 0);

	free(txbuf);
	free(rxbuf);

	if (PyTuple_Check(obj)) {
		PyObject *old = seq;
		seq = PySequence_Tuple(seq);
		Py_DECREF(old);
	}

	return seq;
}


PyDoc_STRVAR(XaSpiDev_xfer2_doc,
	"xfer2([values]) -> [values]\n\n"
	"Perform SPI transaction.\n"
	"CS will be held active between blocks.\n");

static PyObject *
XaSpiDev_xfer2(XaSpiDevObject *self, PyObject *args)
{
	int status;
	uint16_t delay_usecs = 0;
	uint32_t speed_hz = 0;
	uint8_t bits_per_word = 0;
	uint16_t ii, len;
	PyObject *obj;
	PyObject *seq;
	struct spi_ioc_transfer xfer;
	Py_BEGIN_ALLOW_THREADS
	memset(&xfer, 0, sizeof(xfer));
	Py_END_ALLOW_THREADS
	uint8_t *txbuf, *rxbuf;
	char	wrmsg_text[4096];

	if (!PyArg_ParseTuple(args, "O|IHB:xfer2", &obj, &speed_hz, &delay_usecs, &bits_per_word))
		return NULL;

	seq = PySequence_Fast(obj, "expected a sequence");
	len = PySequence_Fast_GET_SIZE(obj);
	if (!seq || len <= 0) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	if (len > SPIDEV_MAXPATH) {
		snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_listmax, SPIDEV_MAXPATH);
		PyErr_SetString(PyExc_OverflowError, wrmsg_text);
		return NULL;
	}

	Py_BEGIN_ALLOW_THREADS
	txbuf = malloc(sizeof(__u8) * len);
	rxbuf = malloc(sizeof(__u8) * len);
	Py_END_ALLOW_THREADS

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, ii);
#if PY_MAJOR_VERSION < 3
		if (PyInt_Check(val)) {
			txbuf[ii] = (__u8)PyInt_AS_LONG(val);
		} else
#endif
		{
			if (PyLong_Check(val)) {
				txbuf[ii] = (__u8)PyLong_AS_LONG(val);
			} else {
				snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_val, val);
				PyErr_SetString(PyExc_TypeError, wrmsg_text);
				free(txbuf);
				free(rxbuf);
				return NULL;
			}
		}
	}

	if (PyTuple_Check(obj)) {
		Py_DECREF(seq);
		seq = PySequence_List(obj);
	}

	Py_BEGIN_ALLOW_THREADS
	xfer.tx_buf = (unsigned long)txbuf;
	xfer.rx_buf = (unsigned long)rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
	xfer.bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;

	status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
	Py_END_ALLOW_THREADS
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		free(txbuf);
		free(rxbuf);
		return NULL;
	}

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PySequence_SetItem(seq, ii, val);
	}
	// WA:
	// in CS_HIGH mode CS isnt pulled to low after transfer
	// reading 0 bytes doesn't really matter but brings CS down
	// tomdean:
	// Stop generating an extra CS except in mode CS_HOGH
	if (self->mode & SPI_CS_HIGH) status = read(self->fd, &rxbuf[0], 0);

	Py_BEGIN_ALLOW_THREADS
	free(txbuf);
	free(rxbuf);
	Py_END_ALLOW_THREADS


	if (PyTuple_Check(obj)) {
		PyObject *old = seq;
		seq = PySequence_Tuple(seq);
		Py_DECREF(old);
	}

	return seq;
}

PyDoc_STRVAR(XaSpiDev_xfer3_doc,
	"xfer3([values]) -> [values]\n\n"
	"Perform SPI transaction. Accepts input of arbitrary size.\n"
	"Large blocks will be send as multiple transactions\n"
	"CS will be held active between blocks.\n");

static PyObject *
XaSpiDev_xfer3(XaSpiDevObject *self, PyObject *args)
{
	int status;
	uint16_t delay_usecs = 0;
	uint32_t speed_hz = 0;
	uint8_t bits_per_word = 0;
	Py_ssize_t ii, jj, len, block_size, block_start, bufsize;
	PyObject *obj;
	PyObject *seq;
	PyObject *rx_tuple;
	struct spi_ioc_transfer xfer;
	Py_BEGIN_ALLOW_THREADS
	memset(&xfer, 0, sizeof(xfer));
	Py_END_ALLOW_THREADS
	uint8_t *txbuf, *rxbuf;
	char	wrmsg_text[4096];

	if (!PyArg_ParseTuple(args, "O|IHB:xfer3", &obj, &speed_hz, &delay_usecs, &bits_per_word))
		return NULL;

	seq = PySequence_Fast(obj, "expected a sequence");
	if (!seq) {
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	len = PySequence_Fast_GET_SIZE(seq);
	if (len <= 0) {
		Py_DECREF(seq);
		PyErr_SetString(PyExc_TypeError, wrmsg_list0);
		return NULL;
	}

	bufsize = get_xfer3_block_size();
	if (bufsize > len) {
		bufsize = len;
	}

	rx_tuple = PyTuple_New(len);
	if (!rx_tuple) {
		Py_DECREF(seq);
		PyErr_SetString(PyExc_OverflowError, wrmsg_oom);
		return NULL;
	}

	Py_BEGIN_ALLOW_THREADS
	// Allocate tx and rx buffers immediately releasing them if any allocation fails
	if ((txbuf = malloc(sizeof(__u8) * bufsize)) != NULL) {
		if ((rxbuf = malloc(sizeof(__u8) * bufsize)) != NULL) {
			// All good, both buffers allocated
		} else {
			// rxbuf allocation failed while txbuf succeeded
			free(txbuf);
			txbuf = NULL;
		}
	} else {
		// txbuf allocation failed
		rxbuf = NULL;
	}
	Py_END_ALLOW_THREADS
	if (!txbuf || !rxbuf) {
		// Allocation failed. Buffers has been freed already
		Py_DECREF(seq);
		Py_DECREF(rx_tuple);
		PyErr_SetString(PyExc_OverflowError, wrmsg_oom);
		return NULL;
	}


	block_start = 0;
	while (block_start < len) {

		for (ii = 0, jj = block_start; jj < len && ii < bufsize; ii++, jj++) {
			PyObject *val = PySequence_Fast_GET_ITEM(seq, jj);
#if PY_MAJOR_VERSION < 3
			if (PyInt_Check(val)) {
				txbuf[ii] = (__u8)PyInt_AS_LONG(val);
			} else
#endif
			{
				if (PyLong_Check(val)) {
					txbuf[ii] = (__u8)PyLong_AS_LONG(val);
				} else {
					snprintf(wrmsg_text, sizeof (wrmsg_text) - 1, wrmsg_val, val);
					PyErr_SetString(PyExc_TypeError, wrmsg_text);
					free(txbuf);
					free(rxbuf);
					Py_DECREF(rx_tuple);
					Py_DECREF(seq);
					return NULL;
				}
			}
		}

		block_size = ii;

		Py_BEGIN_ALLOW_THREADS
		xfer.tx_buf = (unsigned long)txbuf;
		xfer.rx_buf = (unsigned long)rxbuf;
		xfer.len = block_size;
		xfer.delay_usecs = delay_usecs;
		xfer.speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
		xfer.bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;

		status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
		Py_END_ALLOW_THREADS

		if (status < 0) {
			PyErr_SetFromErrno(PyExc_IOError);
			free(txbuf);
			free(rxbuf);
			Py_DECREF(rx_tuple);
			Py_DECREF(seq);
			return NULL;
		}
		for (ii = 0, jj = block_start; ii < block_size; ii++, jj++) {
			PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
			PyTuple_SetItem(rx_tuple, jj, val);
		}

		block_start += block_size;
	}


	// WA:
	// in CS_HIGH mode CS isnt pulled to low after transfer
	// reading 0 bytes doesn't really matter but brings CS down
	// tomdean:
	// Stop generating an extra CS except in mode CS_HIGH
	if (self->mode & SPI_CS_HIGH) status = read(self->fd, &rxbuf[0], 0);

	Py_BEGIN_ALLOW_THREADS
	free(txbuf);
	free(rxbuf);
	Py_END_ALLOW_THREADS

	Py_DECREF(seq);

	return rx_tuple;
}

static int __xaspidev_set_mode( int fd, __u8 mode) {
	__u8 test;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MODE, &test) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	if (test != mode) {
		return -1;
	}
	return 0;
}

PyDoc_STRVAR(XaSpiDev_fileno_doc,
	"fileno() -> integer \"file descriptor\"\n\n"
	"This is needed for lower-level file interfaces, such as os.read().\n");

static PyObject *
XaSpiDev_fileno(XaSpiDevObject *self)
{
	PyObject *result = Py_BuildValue("i", self->fd);
	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_mode(XaSpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", (self->mode & (SPI_CPHA | SPI_CPOL) ) );
	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_cshigh(XaSpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_CS_HIGH)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_lsbfirst(XaSpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_LSB_FIRST)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_3wire(XaSpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_3WIRE)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_loop(XaSpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_LOOP)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
XaSpiDev_get_no_cs(XaSpiDevObject *self, void *closure)
{
        PyObject *result;

        if (self->mode & SPI_NO_CS)
                result = Py_True;
        else
                result = Py_False;

        Py_INCREF(result);
        return result;
}


static int
XaSpiDev_set_mode(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t mode, tmp;
	int ret;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
#if PY_MAJOR_VERSION < 3
	if (PyInt_Check(val)) {
		mode = PyInt_AS_LONG(val);
	} else
#endif
	{
		if (PyLong_Check(val)) {
			mode = PyLong_AS_LONG(val);
		} else {
			PyErr_SetString(PyExc_TypeError,
				"The mode attribute must be an integer");
			return -1;
		}
	}


	if ( mode > 3 ) {
		PyErr_SetString(PyExc_TypeError,
			"The mode attribute must be an integer"
				 "between 0 and 3.");
		return -1;
	}

	// clean and set CPHA and CPOL bits
	tmp = ( self->mode & ~(SPI_CPHA | SPI_CPOL) ) | mode ;

	ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
	return ret;
}

static int
XaSpiDev_set_cshigh(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;
	int ret;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The cshigh attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_CS_HIGH;
	else
		tmp = self->mode & ~SPI_CS_HIGH;

	ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
	return ret;
}

static int
XaSpiDev_set_lsbfirst(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;
	int ret;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The lsbfirst attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_LSB_FIRST;
	else
		tmp = self->mode & ~SPI_LSB_FIRST;

	ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
	return ret;
}

static int
XaSpiDev_set_3wire(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;
	int ret;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The 3wire attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_3WIRE;
	else
		tmp = self->mode & ~SPI_3WIRE;

	ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
	return ret;
}

static int
XaSpiDev_set_no_cs(XaSpiDevObject *self, PyObject *val, void *closure)
{
        uint8_t tmp;
	int ret;

        if (val == NULL) {
                PyErr_SetString(PyExc_TypeError,
                        "Cannot delete attribute");
                return -1;
        }
        else if (!PyBool_Check(val)) {
                PyErr_SetString(PyExc_TypeError,
                        "The no_cs attribute must be boolean");
                return -1;
        }

        if (val == Py_True)
                tmp = self->mode | SPI_NO_CS;
        else
                tmp = self->mode & ~SPI_NO_CS;

        ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
        return ret;
}


static int
XaSpiDev_set_loop(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;
	int ret;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The loop attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_LOOP;
	else
		tmp = self->mode & ~SPI_LOOP;

	ret = __xaspidev_set_mode(self->fd, tmp);

	if (ret != -1)
		self->mode = tmp;
	return ret;
}

static PyObject *
XaSpiDev_get_bits_per_word(XaSpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->bits_per_word);
	Py_INCREF(result);
	return result;
}

static int
XaSpiDev_set_bits_per_word(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t bits;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
#if PY_MAJOR_VERSION < 3
	if (PyInt_Check(val)) {
		bits = PyInt_AS_LONG(val);
	} else
#endif
	{
		if (PyLong_Check(val)) {
			bits = PyLong_AS_LONG(val);
		} else {
			PyErr_SetString(PyExc_TypeError,
				"The bits_per_word attribute must be an integer");
			return -1;
		}
	}

		if (bits < 8 || bits > 32) {
		PyErr_SetString(PyExc_TypeError,
			"invalid bits_per_word (8 to 32)");
		return -1;
	}

	if (self->bits_per_word != bits) {
		if (ioctl(self->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->bits_per_word = bits;
	}
	return 0;
}

static PyObject *
XaSpiDev_get_xa_blocksize(XaSpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->xa_blocksize);
	Py_INCREF(result);
	return result;
}

static int
XaSpiDev_set_xa_blocksize(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint32_t xa_blocksize;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
#if PY_MAJOR_VERSION < 3
	if (PyInt_Check(val)) {
		xa_blocksize = PyInt_AS_LONG(val);
	} else
#endif
	{
		if (PyLong_Check(val)) {
			xa_blocksize = PyLong_AS_LONG(val);
		} else {
			PyErr_SetString(PyExc_TypeError,
				"The xa_blocksize attribute must be an integer");
			return -1;
		}
	}

	if (self->xa_blocksize != xa_blocksize) {
		if ((xa_blocksize != 2048) && (xa_blocksize != 3072)) {
			PyErr_SetString(PyExc_TypeError,
				"The xa_blocksize attribute supported is 2048 or 3072");
			return -1;
		}
		self->xa_blocksize = xa_blocksize;
	}
	return 0;
}

static PyObject *
XaSpiDev_get_max_speed_hz(XaSpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->max_speed_hz);
	Py_INCREF(result);
	return result;
}

static int
XaSpiDev_set_max_speed_hz(XaSpiDevObject *self, PyObject *val, void *closure)
{
	uint32_t max_speed_hz;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
#if PY_MAJOR_VERSION < 3
	if (PyInt_Check(val)) {
		max_speed_hz = PyInt_AS_LONG(val);
	} else
#endif
	{
		if (PyLong_Check(val)) {
			max_speed_hz = PyLong_AS_LONG(val);
		} else {
			PyErr_SetString(PyExc_TypeError,
				"The max_speed_hz attribute must be an integer");
			return -1;
		}
	}

	if (self->max_speed_hz != max_speed_hz) {
		if (ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_hz) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->max_speed_hz = max_speed_hz;
	}
	return 0;
}

static PyGetSetDef XaSpiDev_getset[] = {
	{"mode", (getter)XaSpiDev_get_mode, (setter)XaSpiDev_set_mode,
			"SPI mode as two bit pattern of \n"
			"Clock Polarity  and Phase [CPOL|CPHA]\n"
			"min: 0b00 = 0 max: 0b11 = 3\n"},
	{"cshigh", (getter)XaSpiDev_get_cshigh, (setter)XaSpiDev_set_cshigh,
			"CS active high\n"},
	{"threewire", (getter)XaSpiDev_get_3wire, (setter)XaSpiDev_set_3wire,
			"SI/SO signals shared\n"},
	{"lsbfirst", (getter)XaSpiDev_get_lsbfirst, (setter)XaSpiDev_set_lsbfirst,
			"LSB first\n"},
	{"loop", (getter)XaSpiDev_get_loop, (setter)XaSpiDev_set_loop,
			"loopback configuration\n"},
	{"xa_blocksize", (getter)XaSpiDev_get_xa_blocksize, (setter)XaSpiDev_set_xa_blocksize,
			"Set the FIFO Block size\n"},
	{"no_cs", (getter)XaSpiDev_get_no_cs, (setter)XaSpiDev_set_no_cs,
			"disable chip select\n"},
	{"bits_per_word", (getter)XaSpiDev_get_bits_per_word, (setter)XaSpiDev_set_bits_per_word,
			"bits per word\n"},
	{"max_speed_hz", (getter)XaSpiDev_get_max_speed_hz, (setter)XaSpiDev_set_max_speed_hz,
			"maximum speed in Hz\n"},
	{NULL},
};

PyDoc_STRVAR(XaSpiDev_open_doc,
	"open(bus, device)\n\n"
	"Connects the object to the specified SPI device.\n"
	"open(X,Y) will open /dev/spidev<X>.<Y>\n");

static PyObject *
XaSpiDev_open(XaSpiDevObject *self, PyObject *args, PyObject *kwds)
{
	int bus, device;
	char path[SPIDEV_MAXPATH];
	uint8_t tmp8;
	uint32_t tmp32;
	static char *kwlist[] = {"bus", "device", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ii:open", kwlist, &bus, &device))
		return NULL;
	if (snprintf(path, SPIDEV_MAXPATH, "/dev/spidev%d.%d", bus, device) >= SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError,
			"Bus and/or device number is invalid.");
		return NULL;
	}
	if ((self->fd = open(path, O_RDWR, 0)) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	if (ioctl(self->fd, SPI_IOC_RD_MODE, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->mode = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_BITS_PER_WORD, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->bits_per_word = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->max_speed_hz = tmp32;

	Py_INCREF(Py_None);
	return Py_None;
}

static int
XaSpiDev_init(XaSpiDevObject *self, PyObject *args, PyObject *kwds)
{
	int bus = -1;
	int client = -1;
	static char *kwlist[] = {"bus", "client", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ii:__init__",
			kwlist, &bus, &client))
		return -1;

	if (bus >= 0) {
		XaSpiDev_open(self, args, kwds);
		if (PyErr_Occurred())
			return -1;
	}

	return 0;
}


PyDoc_STRVAR(XaSpiDevObjectType_doc,
	"XaSpiDev([bus],[client]) -> SPI\n\n"
	"Return a new SPI object that is (optionally) connected to the\n"
	"specified SPI device interface.\n");

static
PyObject *XaSpiDev_enter(PyObject *self, PyObject *args)
{
    if (!PyArg_ParseTuple(args, ""))
        return NULL;

    Py_INCREF(self);
    return self;
}

static
PyObject *XaSpiDev_exit(XaSpiDevObject *self, PyObject *args)
{

    PyObject *exc_type = 0;
    PyObject *exc_value = 0;
    PyObject *traceback = 0;
    if (!PyArg_UnpackTuple(args, "__exit__", 3, 3, &exc_type, &exc_value,
                           &traceback)) {
        return 0;
    }

    XaSpiDev_close(self);
    Py_RETURN_FALSE;
}

static PyMethodDef XaSpiDev_methods[] = {
	{"open", (PyCFunction)XaSpiDev_open, METH_VARARGS | METH_KEYWORDS,
		XaSpiDev_open_doc},
	{"close", (PyCFunction)XaSpiDev_close, METH_NOARGS,
		XaSpiDev_close_doc},
	{"fileno", (PyCFunction)XaSpiDev_fileno, METH_NOARGS,
		XaSpiDev_fileno_doc},
	{"readbytes", (PyCFunction)XaSpiDev_readbytes, METH_VARARGS,
		XaSpiDev_read_doc},
	{"writebytes", (PyCFunction)XaSpiDev_writebytes, METH_VARARGS,
		XaSpiDev_write_doc},
	{"writebytes2", (PyCFunction)XaSpiDev_writebytes2, METH_VARARGS,
		XaSpiDev_writebytes2_doc},
	{"xa_writebulk", (PyCFunction)XaSpiDev_xa_writebulk, METH_VARARGS,
		XaSpiDev_xa_writebulk_doc},
	{"xa_readmeta", (PyCFunction)XaSpiDev_xa_readmeta, METH_VARARGS,
		XaSpiDev_xa_readmeta_doc},
	{"xfer", (PyCFunction)XaSpiDev_xfer, METH_VARARGS,
		XaSpiDev_xfer_doc},
	{"xfer2", (PyCFunction)XaSpiDev_xfer2, METH_VARARGS,
		XaSpiDev_xfer2_doc},
	{"xfer3", (PyCFunction)XaSpiDev_xfer3, METH_VARARGS,
		XaSpiDev_xfer3_doc},
	{"__enter__", (PyCFunction)XaSpiDev_enter, METH_VARARGS,
		NULL},
	{"__exit__", (PyCFunction)XaSpiDev_exit, METH_VARARGS,
		NULL},
	{NULL},
};

static PyTypeObject XaSpiDevObjectType = {
#if PY_MAJOR_VERSION >= 3
	PyVarObject_HEAD_INIT(NULL, 0)
#else
	PyObject_HEAD_INIT(NULL)
	0,				/* ob_size */
#endif
	"XaSpiDev",			/* tp_name */
	sizeof(XaSpiDevObject),		/* tp_basicsize */
	0,				/* tp_itemsize */
	(destructor)XaSpiDev_dealloc,	/* tp_dealloc */
	0,				/* tp_print */
	0,				/* tp_getattr */
	0,				/* tp_setattr */
	0,				/* tp_compare */
	0,				/* tp_repr */
	0,				/* tp_as_number */
	0,				/* tp_as_sequence */
	0,				/* tp_as_mapping */
	0,				/* tp_hash */
	0,				/* tp_call */
	0,				/* tp_str */
	0,				/* tp_getattro */
	0,				/* tp_setattro */
	0,				/* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
	XaSpiDevObjectType_doc,		/* tp_doc */
	0,				/* tp_traverse */
	0,				/* tp_clear */
	0,				/* tp_richcompare */
	0,				/* tp_weaklistoffset */
	0,				/* tp_iter */
	0,				/* tp_iternext */
	XaSpiDev_methods,			/* tp_methods */
	0,				/* tp_members */
	XaSpiDev_getset,			/* tp_getset */
	0,				/* tp_base */
	0,				/* tp_dict */
	0,				/* tp_descr_get */
	0,				/* tp_descr_set */
	0,				/* tp_dictoffset */
	(initproc)XaSpiDev_init,		/* tp_init */
	0,				/* tp_alloc */
	XaSpiDev_new,			/* tp_new */
};

static PyMethodDef XaSpiDev_module_methods[] = {
	{NULL}
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef moduledef = {
	PyModuleDef_HEAD_INIT,
	"xaspidev",
	XaSpiDev_module_doc,
	-1,
	XaSpiDev_module_methods,
	NULL,
	NULL,
	NULL,
	NULL,
};
#else
#ifndef PyMODINIT_FUNC	/* declarations for DLL import/export */
#define PyMODINIT_FUNC void
#endif
#endif

#if PY_MAJOR_VERSION >= 3
PyMODINIT_FUNC
PyInit_xaspidev(void)
#else
void initxaspidev(void)
#endif
{
	PyObject* m;

	if (PyType_Ready(&XaSpiDevObjectType) < 0)
#if PY_MAJOR_VERSION >= 3
		return NULL;
#else
		return;
#endif

#if PY_MAJOR_VERSION >= 3
	m = PyModule_Create(&moduledef);
	PyObject *version = PyUnicode_FromString(_VERSION_);
#else
	m = Py_InitModule3("xaspidev", XaSpiDev_module_methods, XaSpiDev_module_doc);
	PyObject *version = PyString_FromString(_VERSION_);
#endif

	PyObject *dict = PyModule_GetDict(m);
	PyDict_SetItemString(dict, "__version__", version);
	Py_DECREF(version);

	Py_INCREF(&XaSpiDevObjectType);
	PyModule_AddObject(m, "XaSpiDev", (PyObject *)&XaSpiDevObjectType);

#if PY_MAJOR_VERSION >= 3
	return m;
#endif
}
