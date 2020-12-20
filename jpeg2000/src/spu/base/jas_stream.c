/*
 * Copyright (c) 1999-2000 Image Power, Inc. and the University of
 *   British Columbia.
 * Copyright (c) 2001-2003 Michael David Adams.
 * All rights reserved.
 */

/* __START_OF_JASPER_LICENSE__
 * 
 * JasPer License Version 2.0
 * 
 * Copyright (c) 2001-2006 Michael David Adams
 * Copyright (c) 1999-2000 Image Power, Inc.
 * Copyright (c) 1999-2000 The University of British Columbia
 * 
 * All rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person (the
 * "User") obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 * 
 * 1.  The above copyright notices and this permission notice (which
 * includes the disclaimer below) shall be included in all copies or
 * substantial portions of the Software.
 * 
 * 2.  The name of a copyright holder shall not be used to endorse or
 * promote products derived from the Software without specific prior
 * written permission.
 * 
 * THIS DISCLAIMER OF WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS
 * LICENSE.  NO USE OF THE SOFTWARE IS AUTHORIZED HEREUNDER EXCEPT UNDER
 * THIS DISCLAIMER.  THE SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS.  IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, OR ANY SPECIAL
 * INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING
 * FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.  NO ASSURANCES ARE
 * PROVIDED BY THE COPYRIGHT HOLDERS THAT THE SOFTWARE DOES NOT INFRINGE
 * THE PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF ANY OTHER ENTITY.
 * EACH COPYRIGHT HOLDER DISCLAIMS ANY LIABILITY TO THE USER FOR CLAIMS
 * BROUGHT BY ANY OTHER ENTITY BASED ON INFRINGEMENT OF INTELLECTUAL
 * PROPERTY RIGHTS OR OTHERWISE.  AS A CONDITION TO EXERCISING THE RIGHTS
 * GRANTED HEREUNDER, EACH USER HEREBY ASSUMES SOLE RESPONSIBILITY TO SECURE
 * ANY OTHER INTELLECTUAL PROPERTY RIGHTS NEEDED, IF ANY.  THE SOFTWARE
 * IS NOT FAULT-TOLERANT AND IS NOT INTENDED FOR USE IN MISSION-CRITICAL
 * SYSTEMS, SUCH AS THOSE USED IN THE OPERATION OF NUCLEAR FACILITIES,
 * AIRCRAFT NAVIGATION OR COMMUNICATION SYSTEMS, AIR TRAFFIC CONTROL
 * SYSTEMS, DIRECT LIFE SUPPORT MACHINES, OR WEAPONS SYSTEMS, IN WHICH
 * THE FAILURE OF THE SOFTWARE OR SYSTEM COULD LEAD DIRECTLY TO DEATH,
 * PERSONAL INJURY, OR SEVERE PHYSICAL OR ENVIRONMENTAL DAMAGE ("HIGH
 * RISK ACTIVITIES").  THE COPYRIGHT HOLDERS SPECIFICALLY DISCLAIM ANY
 * EXPRESS OR IMPLIED WARRANTY OF FITNESS FOR HIGH RISK ACTIVITIES.
 * 
 * __END_OF_JASPER_LICENSE__
 */

/*
 * I/O Stream Library
 *
 * $Id: jas_stream.c,v 1.4 2007/09/12 23:58:56 lrlemini Exp $
 */

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <spu_assert.h>
#if defined(HAVE_FCNTL_H)
#include <fcntl.h>
#endif
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#if defined(HAVE_UNISTD_H)
#include <unistd.h>
#endif
#if defined(WIN32) || defined(HAVE_IO_H)
#include <io.h>
#endif

#include "jasper/jas_types.h"
#include "jasper/jas_stream.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_debug.h"
/* s.kang end */

/******************************************************************************\
* Local function prototypes.
\******************************************************************************/

static void jas_stream_destroy(jas_stream_t *stream);
static jas_stream_t *jas_stream_create(void);
static void jas_stream_initbuf(jas_stream_t *stream, int bufmode, char *buf,
  int bufsize);

static int mem_read(jas_stream_obj_t *obj, char *buf, int cnt);
static int mem_write(jas_stream_obj_t *obj, char *buf, int cnt);
static long mem_seek(jas_stream_obj_t *obj, long offset, int origin);
static int mem_close(jas_stream_obj_t *obj);

/******************************************************************************\
* Local data.
\******************************************************************************/

static jas_stream_ops_t jas_stream_memops = {
	mem_read,
	mem_write,
	mem_seek,
	mem_close
};

/******************************************************************************\
* Code for opening and closing streams.
\******************************************************************************/

static jas_stream_t *jas_stream_create()
{
	jas_stream_t *stream;

	if (!(stream = jas_malloc(sizeof(jas_stream_t)))) {
		return 0;
	}
	stream->openmode_ = 0;
	stream->bufmode_ = 0;
	stream->flags_ = 0;
	stream->bufbase_ = 0;
	stream->bufstart_ = 0;
	stream->bufsize_ = 0;
	stream->ptr_ = 0;
	stream->cnt_ = 0;
	stream->ops_ = 0;
	stream->obj_ = 0;
	stream->rwcnt_ = 0;
	stream->rwlimit_ = -1;

	return stream;
}

jas_stream_t *jas_stream_memopen(char *buf, int bufsize)
{
	jas_stream_t *stream;
	jas_stream_memobj_t *obj;

	if (!(stream = jas_stream_create())) {
		return 0;
	}

	/* A stream associated with a memory buffer is always opened
	for both reading and writing in binary mode. */
	stream->openmode_ = JAS_STREAM_READ | JAS_STREAM_WRITE | JAS_STREAM_BINARY;

	/* Since the stream data is already resident in memory, buffering
	is not necessary. */
	/* But... It still may be faster to use buffering anyways. */
	jas_stream_initbuf(stream, JAS_STREAM_FULLBUF, 0, 0);

	/* Select the operations for a memory stream. */
	stream->ops_ = &jas_stream_memops;

	/* Allocate memory for the underlying memory stream object. */
	if (!(obj = jas_malloc(sizeof(jas_stream_memobj_t)))) {
		jas_stream_destroy(stream);
		return 0;
	}
	stream->obj_ = (void *) obj;

	/* Initialize a few important members of the memory stream object. */
	obj->myalloc_ = 0;
	obj->buf_ = 0;

	/* If the buffer size specified is nonpositive, then the buffer
	is allocated internally and automatically grown as needed. */
	if (bufsize <= 0) {
		obj->bufsize_ = 1024;
		obj->growable_ = 1;
	} else {
		obj->bufsize_ = bufsize;
		obj->growable_ = 0;
	}
	if (buf) {
		obj->buf_ = (unsigned char *) buf;
	} else {
		obj->buf_ = jas_malloc(obj->bufsize_ * sizeof(char));
		obj->myalloc_ = 1;
	}
	if (!obj->buf_) {
		jas_stream_close(stream);
		return 0;
	}

	if (bufsize > 0 && buf) {
		/* If a buffer was supplied by the caller and its length is positive,
		  make the associated buffer data appear in the stream initially. */
		obj->len_ = bufsize;
	} else {
		/* The stream is initially empty. */
		obj->len_ = 0;
	}
	obj->pos_ = 0;
	
	return stream;
}

static void jas_stream_destroy(jas_stream_t *stream)
{
	/* If the memory for the buffer was allocated with malloc, free
	this memory. */
	if ((stream->bufmode_ & JAS_STREAM_FREEBUF) && stream->bufbase_) {
		jas_free_align(stream->bufbase_);
		stream->bufbase_ = 0;
	}
	jas_free(stream);
}

int jas_stream_close(jas_stream_t *stream)
{
	/* Flush buffer if necessary. */
	jas_stream_flush(stream);

	/* Close the underlying stream object. */
	(*stream->ops_->close_)(stream->obj_);

	jas_stream_destroy(stream);

	return 0;
}

/******************************************************************************\
* Code for reading and writing streams.
\******************************************************************************/

int jas_stream_ungetc(jas_stream_t *stream, int c)
{
	if (!stream->ptr_ || stream->ptr_ == stream->bufbase_) {
		return -1;
	}

	/* Reset the EOF indicator (since we now have at least one character
	  to read). */
	stream->flags_ &= ~JAS_STREAM_EOF;

	--stream->rwcnt_;
	--stream->ptr_;
	++stream->cnt_;
	*stream->ptr_ = c;
	return 0;
}

int jas_stream_read(jas_stream_t *stream, void *buf, int cnt)
{
	int n;
	int c;
	char *bufptr;

	bufptr = buf;

	n = 0;
	while (n < cnt) {
		if ((c = jas_stream_getc(stream)) == EOF) {
			return n;
		}
		*bufptr++ = c;
		++n;
	}

	return n;
}

int jas_stream_write(jas_stream_t *stream, const void *buf, int cnt)
{
	int n;
	const char *bufptr;

	bufptr = buf;

	n = 0;
	while (n < cnt) {
		if (jas_stream_putc(stream, *bufptr) == EOF) {
			return n;
		}
		++bufptr;
		++n;
	}

	return n;
}

/******************************************************************************\
* Code for getting and setting the stream position.
\******************************************************************************/

int jas_stream_isseekable(jas_stream_t *stream)
{
	if (stream->ops_ == &jas_stream_memops) {
		return 1;
	} else {
		return 0;
	}
}

int jas_stream_rewind(jas_stream_t *stream)
{
	return jas_stream_seek(stream, 0, SEEK_SET);
}

long jas_stream_seek(jas_stream_t *stream, long offset, int origin)
{
	long newpos;

	/* The buffer cannot be in use for both reading and writing. */
	spu_assert( ( !( ( stream->bufmode_ & JAS_STREAM_RDBUF ) && ( stream->bufmode_ & JAS_STREAM_WRBUF ) ) ), ( "[jas_stream.c:jas_stream_seek()] assertion failure\n" ) );

	/* Reset the EOF indicator (since we may not be at the EOF anymore). */
	stream->flags_ &= ~JAS_STREAM_EOF;

	if (stream->bufmode_ & JAS_STREAM_RDBUF) {
		if (origin == SEEK_CUR) {
			offset -= stream->cnt_;
		}
	} else if (stream->bufmode_ & JAS_STREAM_WRBUF) {
		if (jas_stream_flush(stream)) {
			return -1;
		}
	}
	stream->cnt_ = 0;
	stream->ptr_ = stream->bufstart_;
	stream->bufmode_ &= ~(JAS_STREAM_RDBUF | JAS_STREAM_WRBUF);

	if ((newpos = (*stream->ops_->seek_)(stream->obj_, offset, origin))
	  < 0) {
		return -1;
	}

	return newpos;
}

long jas_stream_tell(jas_stream_t *stream)
{
	int adjust;
	int offset;

	if (stream->bufmode_ & JAS_STREAM_RDBUF) {
		adjust = -stream->cnt_;
	} else if (stream->bufmode_ & JAS_STREAM_WRBUF) {
		adjust = stream->ptr_ - stream->bufstart_;
	} else {
		adjust = 0;
	}

	if ((offset = (*stream->ops_->seek_)(stream->obj_, 0, SEEK_CUR)) < 0) {
		return -1;
	}

	return offset + adjust;
}

/******************************************************************************\
* Buffer initialization code.
\******************************************************************************/

static void jas_stream_initbuf(jas_stream_t *stream, int bufmode, char *buf,
  int bufsize)
{
	/* If this function is being called, the buffer should not have been
	  initialized yet. */
	spu_assert( ( !stream->bufbase_ ), ( "[jas_stream.c:jas_stream_initbuf()] assertion failure\n" ) );

	if (bufmode != JAS_STREAM_UNBUF) {
		/* The full- or line-buffered mode is being employed. */
		if (!buf) {
			/* The caller has not specified a buffer to employ, so allocate
			  one. */
			if ((stream->bufbase_ = jas_malloc(JAS_STREAM_BUFSIZE +
			  JAS_STREAM_MAXPUTBACK))) {
				stream->bufmode_ |= JAS_STREAM_FREEBUF;
				stream->bufsize_ = JAS_STREAM_BUFSIZE;
			} else {
				/* The buffer allocation has failed.  Resort to unbuffered
				  operation. */
				stream->bufbase_ = stream->tinybuf_;
				stream->bufsize_ = 1;
			}
		} else {
			/* The caller has specified a buffer to employ. */
			/* The buffer must be large enough to accommodate maximum
			  putback. */
			spu_assert( ( bufsize > JAS_STREAM_MAXPUTBACK ), ( "[jas_stream.c:jas_stream_initbuf()] assertion failure\n" ) );
			stream->bufbase_ = JAS_CAST(uchar *, buf);
			stream->bufsize_ = bufsize - JAS_STREAM_MAXPUTBACK;
		}
	} else {
		/* The unbuffered mode is being employed. */
		/* A buffer should not have been supplied by the caller. */
		spu_assert( ( !buf ), ( "[jas_stream.c:jas_stream_initbuf()] assertion failure\n" ) );
		/* Use a trivial one-character buffer. */
		stream->bufbase_ = stream->tinybuf_;
		stream->bufsize_ = 1;
	}
	stream->bufstart_ = &stream->bufbase_[JAS_STREAM_MAXPUTBACK];
	stream->ptr_ = stream->bufstart_;
	stream->cnt_ = 0;
	stream->bufmode_ |= bufmode & JAS_STREAM_BUFMODEMASK;
}

/******************************************************************************\
* Buffer filling and flushing code.
\******************************************************************************/

int jas_stream_flush(jas_stream_t *stream)
{
	if (stream->bufmode_ & JAS_STREAM_RDBUF) {
		return 0;
	}
	return jas_stream_flushbuf(stream, EOF);
}

int jas_stream_fillbuf(jas_stream_t *stream, int getflag)
{
	int c;

	/* The stream must not be in an error or EOF state. */
	if ((stream->flags_ & (JAS_STREAM_ERRMASK)) != 0) {
		return EOF;
	}

	/* The stream must be open for reading. */
	if ((stream->openmode_ & JAS_STREAM_READ) == 0) {
		return EOF;
	}

	/* Make a half-hearted attempt to confirm that the buffer is not
	currently being used for writing.  This check is not intended
	to be foolproof! */
	spu_assert( ( ( stream->bufmode_ & JAS_STREAM_WRBUF) == 0 ), ( "[jas_stream.c:jas_stream_fillbuf()] assertion failure\n" ) );

	spu_assert( ( stream->ptr_ - stream->bufstart_ <= stream->bufsize_ ), ( "[jas_stream.c:jas_stream_fillbuf()] assertion failure\n" ) );

	/* Mark the buffer as being used for reading. */
	stream->bufmode_ |= JAS_STREAM_RDBUF;

	/* Read new data into the buffer. */
	stream->ptr_ = stream->bufstart_;
	if ((stream->cnt_ = (*stream->ops_->read_)(stream->obj_,
	  (char *) stream->bufstart_, stream->bufsize_)) <= 0) {
		if (stream->cnt_ < 0) {
			stream->flags_ |= JAS_STREAM_ERR;
		} else {
			stream->flags_ |= JAS_STREAM_EOF;
		}
		stream->cnt_ = 0;
		return EOF;
	}

	spu_assert( ( stream->cnt_ > 0 ), ( "[jas_stream.c:jas_stream_fillbuf()] assertion failure\n" ) );
	/* Get or peek at the first character in the buffer. */
	c = (getflag) ? jas_stream_getc2(stream) : (*stream->ptr_);

	return c;
}

int jas_stream_flushbuf(jas_stream_t *stream, int c)
{
	int len;
	int n;

	/* The stream should not be in an error or EOF state. */
	if ((stream->flags_ & (JAS_STREAM_ERRMASK)) != 0) {
		return EOF;
	}

	/* The stream must be open for writing. */
	if ((stream->openmode_ & (JAS_STREAM_WRITE | JAS_STREAM_APPEND)) == 0) {
		return EOF;
	}

	/* The buffer should not currently be in use for reading. */
	spu_assert( ( !( stream->bufmode_ & JAS_STREAM_RDBUF ) ), ( "[jas_stream.c:jas_stream_flushbuf()] assertion failure\n" ) );

	/* Note: Do not use the quantity stream->cnt to determine the number
	of characters in the buffer!  Depending on how this function was
	called, the stream->cnt value may be "off-by-one". */
	len = stream->ptr_ - stream->bufstart_;
	if (len > 0) {
		n = (*stream->ops_->write_)(stream->obj_, (char *)
		  stream->bufstart_, len);
		if (n != len) {
			stream->flags_ |= JAS_STREAM_ERR;
			return EOF;
		}
	}
	stream->cnt_ = stream->bufsize_;
	stream->ptr_ = stream->bufstart_;

	stream->bufmode_ |= JAS_STREAM_WRBUF;

	if (c != EOF) {
		spu_assert( ( stream->cnt_ > 0 ), ( "[jas_stream.c:jas_stream_flushbuf()] assertion failure\n" ) );
		return jas_stream_putc2(stream, c);
	}

	return 0;
}

/******************************************************************************\
* Miscellaneous code.
\******************************************************************************/

/* s.kang start */
int jas_stream_get_stream_type( jas_stream_t* p_stream ) {
	spu_assert( ( p_stream != NULL ), ( "[jas_stream.c:jas_stream_get_stream_type()] assertion failure\n" ) );

	if( p_stream->ops_ == &jas_stream_memops ) {
		return JAS_STREAM_MEM;
	}
	else {
		jas_eprintf( "[jas_stream.c:jas_stream_get_stream_type()] unsupported stream type\n" );
		return JAS_STREAM_UNKNOWN;
	}
}
/* s.kang end */

int jas_stream_copy(jas_stream_t *out, jas_stream_t *in, int n)
{
	int all;
	int c;
	int m;

	all = (n < 0) ? 1 : 0;

	m = n;
	while (all || m > 0) {
		if ((c = jas_stream_getc_macro(in)) == EOF) {
			/* The next character of input could not be read. */
			/* Return with an error if an I/O error occured
			  (not including EOF) or if an explicit copy count
			  was specified. */
			return (!all || jas_stream_error(in)) ? (-1) : 0;
		}
		if (jas_stream_putc_macro(out, c) == EOF) {
			return -1;
		}
		--m;
	}
	return 0;
}

long jas_stream_setrwcount(jas_stream_t *stream, long rwcnt)
{
	int old;

	old = stream->rwcnt_;
	stream->rwcnt_ = rwcnt;
	return old;
}

long jas_stream_length(jas_stream_t *stream)
{
	long oldpos;
	long pos;
	if ((oldpos = jas_stream_tell(stream)) < 0) {
		return -1;
	}
	if (jas_stream_seek(stream, 0, SEEK_END) < 0) {
		return -1;
	}
	if ((pos = jas_stream_tell(stream)) < 0) {
		return -1;
	}
	if (jas_stream_seek(stream, oldpos, SEEK_SET) < 0) {
		return -1;
	}
	return pos;
}

/******************************************************************************\
* Memory stream object.
\******************************************************************************/

static int mem_read(jas_stream_obj_t *obj, char *buf, int cnt)
{
	int n;
	jas_stream_memobj_t *m = (jas_stream_memobj_t *)obj;
	n = m->len_ - m->pos_;
	cnt = JAS_MIN(n, cnt);
	memcpy(buf, &m->buf_[m->pos_], cnt);
	m->pos_ += cnt;
	return cnt;
}

static int mem_resize(jas_stream_memobj_t *m, int bufsize)
{
	unsigned char *buf;

	spu_assert( ( m->buf_ ), ( "[jas_stream.c:mem_resize()] assertion failure\n" ) );

	if (!(buf = jas_realloc(m->buf_, bufsize * sizeof(unsigned char)))) {
		return -1;
	}

	m->buf_ = buf;
	m->bufsize_ = bufsize;
	return 0;
}

static int mem_write(jas_stream_obj_t *obj, char *buf, int cnt)
{
	int n;
	int ret;
	jas_stream_memobj_t *m = (jas_stream_memobj_t *)obj;
	long newbufsize;
	long newpos;

	newpos = m->pos_ + cnt;
	if (newpos > m->bufsize_ && m->growable_) {
		newbufsize = m->bufsize_;
		while (newbufsize < newpos) {
			newbufsize <<= 1;
			spu_assert( ( newbufsize >= 0 ), ( "[jas_stream.c:mem_write()] assertion failure\n" ) );
		}
		if (mem_resize(m, newbufsize)) {
			return -1;
		}
	}
	if (m->pos_ > m->len_) {
		/* The current position is beyond the end of the file, so
		  pad the file to the current position with zeros. */
		n = JAS_MIN(m->pos_, m->bufsize_) - m->len_;
		if (n > 0) {
			memset(&m->buf_[m->len_], 0, n);
			m->len_ += n;
		}
		if (m->pos_ != m->len_) {
			/* The buffer is not big enough. */
			return 0;
		}
	}
	n = m->bufsize_ - m->pos_;
	ret = JAS_MIN(n, cnt);
	if (ret > 0) {
		memcpy(&m->buf_[m->pos_], buf, ret);
		m->pos_ += ret;
	}
	if (m->pos_ > m->len_) {
		m->len_ = m->pos_;
	}
	spu_assert( ( ret == cnt ), ( "[jas_stream.c:mem_write()] assertion failure\n" ) );
	return ret;
}

static long mem_seek(jas_stream_obj_t *obj, long offset, int origin)
{
	jas_stream_memobj_t *m = (jas_stream_memobj_t *)obj;
	long newpos;

	switch (origin) {
	case SEEK_SET:
		newpos = offset;
		break;
	case SEEK_END:
		newpos = m->len_ - offset;
		break;
	case SEEK_CUR:
		newpos = m->pos_ + offset;
		break;
	default:
		abort();
		break;
	}
	if (newpos < 0) {
		return -1;
	}
	m->pos_ = newpos;

	return m->pos_;
}

static int mem_close(jas_stream_obj_t *obj)
{
	jas_stream_memobj_t *m = (jas_stream_memobj_t *)obj;

	if (m->myalloc_ && m->buf_) {
		jas_free(m->buf_);
		m->buf_ = 0;
	}
	jas_free(obj);
	return 0;
}

