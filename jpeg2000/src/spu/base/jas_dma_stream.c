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

/* s.kang start */

/*
 * I/O DMA Stream Library
 *
 * $Id: jas_dma_stream.c,v 1.1 2007/09/12 23:58:56 lrlemini Exp $
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
#include "jasper/jas_dma_stream.h"
#include "jasper/jas_dma.h"

#include "jasper-cell.h"

/******************************************************************************\
* Local function prototypes.
\******************************************************************************/

/******************************************************************************\
* Local data.
\******************************************************************************/

/******************************************************************************\
* Code for opening and closing streams.
\******************************************************************************/

jas_dma_stream_t* jas_dma_stream_open( int read, unsigned int ext_bufaddr, int ext_bufsize, int ext_len, unsigned int ext_aux_bufaddr, int ext_aux_bufsize ) {
	jas_dma_stream_t* p_stream;

        /* buffer cannot be internally allocated for dma mode */

        if( ( ext_bufaddr == 0 ) || ( ext_bufsize <= 0 ) ) {
                return NULL;
        }

        /* buffer address and size must be properly aligned */

        if( ( ( ( unsigned int )ext_bufaddr % DMA_MIN_SIZE ) != 0 ) || ( ( ext_bufsize % DMA_MIN_SIZE ) != 0 ) ) {
                return NULL;
        }

	/* auxiliary buffer is only for write mode */

	if( read && ( ( ext_aux_bufaddr != 0 ) || ( ext_aux_bufsize != 0 ) ) ) {
		return NULL;
	}

	p_stream = jas_malloc( sizeof( jas_dma_stream_t ) );
	if( p_stream == NULL ) {
		return NULL;
	}

	/* set open mode */

	if( read ) {
		p_stream->openmode_ = JAS_STREAM_READ;
	}
	else {
		p_stream->openmode_ = JAS_STREAM_WRITE;
	}

	/* set external buffer information */

	p_stream->ext_bufaddr_ = ext_bufaddr;
	p_stream->ext_bufsize_ = ext_bufsize;
	p_stream->ext_len_ = ext_len;
	p_stream->ext_offset_ = 0;
	if( read == 0 ) {/* write mode */
		p_stream->ext_aux_bufaddr_ = ext_aux_bufaddr;
		p_stream->ext_aux_bufsize_ = ext_aux_bufsize;
		p_stream->ext_aux_len_ = 0;
		p_stream->ext_aux_offset_ = 0;
	}

        /* do buffering for efficient DMA access */

	p_stream->p_spu_bufbase_ = jas_malloc_align( JAS_STREAM_BUFSIZE + JAS_STREAM_MAXPUTBACK, DMA_MIN_OFFSET_BITS );
	if( p_stream->p_spu_bufbase_ == NULL ) {
		jas_free( p_stream );
		return NULL;
	}
	p_stream->p_spu_bufstart_ = p_stream->p_spu_bufbase_ + JAS_STREAM_MAXPUTBACK;
	p_stream->spu_bufsize_ = JAS_STREAM_BUFSIZE;
	p_stream->spu_len_ = 0;
	p_stream->spu_offset_ = 0;

        return p_stream;
}

int jas_dma_stream_close( jas_dma_stream_t* p_stream ) {
	jas_dma_stream_flush( p_stream );
	jas_free_align( p_stream->p_spu_bufbase_ );
	jas_free( p_stream );

	return 0;
}

/******************************************************************************\
* Code for reading and writing streams.
\******************************************************************************/

int jas_dma_stream_readc_func( jas_dma_stream_t* p_stream, int offset ) {
	char a_buf[DMA_MIN_SIZE] __attribute__ ( ( aligned( DMA_MIN_SIZE ) ) );

	if( p_stream->openmode_ == JAS_STREAM_READ ) {
		if( offset >= p_stream->ext_offset_ + p_stream->ext_len_ ) {
			return EOF;
		}

		/* get data from PPU for the simplicity of implementation */
		jas_dma_get_from_ppu( ( void* )( ( p_stream->ext_bufaddr_ + offset ) & DMA_MIN_OFFSET_MASK ), a_buf, DMA_MIN_SIZE );
		return a_buf[( p_stream->ext_bufaddr_ + offset ) % DMA_MIN_SIZE];
	}
	else {
		if( offset >= p_stream->ext_aux_len_/* aux buf is only for write mode */ + p_stream->ext_len_ + p_stream->spu_len_ ) {
			return EOF;
		}

		if( offset >= p_stream->ext_aux_len_ + p_stream->ext_len_ ) {/* reside in the spu buffer */
			return p_stream->p_spu_bufstart_[offset -p_stream->ext_aux_len_ - p_stream->ext_len_];
		}
		else if( offset >= p_stream->ext_len_ ) {/* reside in the aux buffer */
			jas_dma_get_from_ppu( ( void* )( ( p_stream->ext_aux_bufaddr_ + offset - p_stream->ext_len_ ) & DMA_MIN_OFFSET_MASK ), a_buf, DMA_MIN_SIZE );
			return a_buf[( p_stream->ext_aux_bufaddr_ + offset - p_stream->ext_len_ ) % DMA_MIN_SIZE];
		}
		else {/* reside in the ext buffer */
			jas_dma_get_from_ppu( ( void* )( ( p_stream->ext_bufaddr_ + offset ) & DMA_MIN_OFFSET_MASK ), a_buf, DMA_MIN_SIZE );
			return a_buf[( p_stream->ext_bufaddr_ + offset ) % DMA_MIN_SIZE];
		}
	}
}

/******************************************************************************\
* Code for getting and setting the stream position.
\******************************************************************************/

/******************************************************************************\
* Buffer initialization code.
\******************************************************************************/

/******************************************************************************\
* Buffer filling and flushing code.
\******************************************************************************/

unsigned char jas_dma_stream_fillbuf( jas_dma_stream_t* p_stream ) {
	int dma_len;

	spu_assert( ( p_stream->openmode_ == JAS_STREAM_READ ), ( "[jas_dma_stream.c:jas_dma_stream_fillbuf()] assertion failure\n" ) );
	spu_assert( ( p_stream->spu_len_ == 0 ), ( "[jas_dma_stream.c:jas_dma_stream_fillbuf()] assertion failure\n" ) );
	if( p_stream->ext_len_ > 0 ) {
		if( p_stream->ext_len_ >= p_stream->spu_bufsize_ ) {
			dma_len = p_stream->spu_bufsize_;
		}
		else {
			dma_len = p_stream->ext_len_;
		}
		jas_dma_get_from_ppu( ( void* )( p_stream->ext_bufaddr_ + p_stream->ext_offset_ ), p_stream->p_spu_bufstart_, dma_len );
		p_stream->ext_offset_ += dma_len;
		p_stream->ext_len_ -= dma_len;
		p_stream->spu_offset_ = 0;
		p_stream->spu_len_ = dma_len;

		p_stream->spu_len_--;
		return p_stream->p_spu_bufstart_[p_stream->spu_offset_++];
	}
	else {
		return ( unsigned char )EOF;
	}
}

int jas_dma_stream_flush( jas_dma_stream_t* p_stream ) {
	if( p_stream->openmode_ & JAS_STREAM_READ ) {
		return 0;
	}
	return jas_dma_stream_flushbuf( p_stream, EOF );
}

int jas_dma_stream_flushbuf( jas_dma_stream_t* p_stream, int c ) {
	int dma_obj_len;/* dma to obj buf */
	int dma_aux_len;/* dma to aux buf */
	int rem_obj_buf;/* remaining obj buf space */
	int rem_aux_buf;/* remaining aux buf space */
	int ret;

	spu_assert( ( p_stream->openmode_ == JAS_STREAM_WRITE ), ( "[jas_dma_stream.c:jas_dma_stream_flushbuf()] assertion failure\n" ) );
	spu_assert( ( ( p_stream->spu_len_ == JAS_STREAM_BUFSIZE ) || ( c == EOF ) ), ( "[jas_dma_stream.c:jas_dma_stream_flushbuf()] assertion failure\n" ) );

	rem_obj_buf = p_stream->ext_bufsize_ - p_stream->ext_len_;
	rem_aux_buf = p_stream->ext_aux_bufsize_ - p_stream->ext_aux_len_;
	ret = 0;

	if( rem_obj_buf >= p_stream->spu_len_ ) {
		dma_obj_len = p_stream->spu_len_;
		dma_aux_len = 0;
	}
	else {
		dma_obj_len = rem_obj_buf;
		dma_aux_len = p_stream->spu_len_ - rem_obj_buf;
		if( dma_aux_len > rem_aux_buf ) {
			jas_eprintf( "[jas_stream.c:jas_dma_stream_flushbuf()] buffer size too small\n" );
			dma_aux_len = rem_aux_buf;
			ret = EOF;
		}
	}

	if( dma_obj_len ) {
		jas_dma_put_to_ppu( p_stream->p_spu_bufstart_, ( void* )( p_stream->ext_bufaddr_ + p_stream->ext_offset_ ), dma_obj_len );
		p_stream->ext_offset_ += dma_obj_len;
		p_stream->ext_len_ += dma_obj_len;
		p_stream->spu_offset_ -= dma_obj_len;
		p_stream->spu_len_ -= dma_obj_len;
	}

	if( dma_aux_len ) {
		jas_dma_put_to_ppu( p_stream->p_spu_bufstart_ + dma_obj_len, ( void* )( p_stream->ext_aux_bufaddr_ + p_stream->ext_aux_offset_ ), dma_aux_len );
		p_stream->ext_aux_offset_ += dma_aux_len;
		p_stream->ext_aux_len_ += dma_aux_len;
		p_stream->spu_offset_ -= dma_aux_len;
		p_stream->spu_len_ -= dma_aux_len;
	}

	if( ( ret != EOF ) && ( c != EOF ) ) {
		p_stream->spu_len_++;
		p_stream->p_spu_bufstart_[p_stream->spu_offset_++] = c;
	}

	return ret;
}

/******************************************************************************\
* Miscellaneous code.
\******************************************************************************/

/******************************************************************************\
* Memory stream object.
\******************************************************************************/

/* s.kang end */

