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
 * I/O DMA Stream Class
 *
 * $Id: jas_dma_stream.h,v 1.1 2007/09/12 23:58:55 lrlemini Exp $
 */

/* s.kang start */

#ifndef JAS_DMA_STREAM_H
#define JAS_DMA_STREAM_H

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <stdio.h>
#if defined(HAVE_FCNTL_H)
#include <fcntl.h>
#endif
#include <string.h>
#if defined(HAVE_UNISTD_H)
#include <unistd.h>
#endif

#include <jasper/jas_config.h>
#include <jasper/jas_stream.h>
#include <jasper/jas_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
* Constants.
\******************************************************************************/

/******************************************************************************\
* Types.
\******************************************************************************/

typedef struct _jas_dma_stream_t {
	int openmode_;

	unsigned int ext_bufaddr_;
	int ext_bufsize_;
	int ext_len_;
	int ext_offset_;

	unsigned int ext_aux_bufaddr_;
	int ext_aux_bufsize_;
	int ext_aux_len_;
	int ext_aux_offset_;

	void* p_spu_bufbase_;
	unsigned char* p_spu_bufstart_;
	int spu_bufsize_;
	int spu_len_;
	int spu_offset_;
} jas_dma_stream_t;

/******************************************************************************\
* Macros/functions for opening and closing streams.
\******************************************************************************/

jas_dma_stream_t* jas_dma_stream_open( int read, unsigned int ext_bufaddr, int ext_bufsize, int ext_len, unsigned int ext_aux_bufaddr, int ext_aux_bufsize );

int jas_dma_stream_close( jas_dma_stream_t* p_stream );

/******************************************************************************\
* Macros/functions for getting/setting the stream state.
\******************************************************************************/

/******************************************************************************\
* Macros/functions for I/O.
\******************************************************************************/

#define jas_dma_stream_getc( p_stream ) \
	( ( ( p_stream )->spu_len_ == 0 ) ?\
	jas_dma_stream_fillbuf( ( p_stream ) ) : ( ( p_stream )->spu_len_--, ( p_stream )->p_spu_bufstart_[( p_stream )->spu_offset_++] ) )

#define jas_dma_stream_putc( p_stream, c ) \
	( ( ( p_stream )->spu_len_ == JAS_STREAM_BUFSIZE ) ?\
	( jas_dma_stream_flushbuf( ( p_stream ), ( c ) ) ) : ( ( p_stream )->spu_len_++, ( ( p_stream )->p_spu_bufstart_[( p_stream )->spu_offset_++] = ( c ) ), 0 ) )

#define jas_dma_stream_readc( p_stream, offset ) \
	jas_dma_stream_readc_func( ( p_stream ), ( offset ) )

int jas_dma_stream_readc_func( jas_dma_stream_t* p_stream, int offset );

/******************************************************************************\
* Macros/functions for getting/setting the stream position.
\******************************************************************************/

#define jas_dma_stream_tell( p_stream ) \
	( ( ( p_stream )->openmode_ == JAS_STREAM_READ ) ?\
	( ( p_stream )->ext_offset_ - ( p_stream )->spu_len_ ) : ( ( p_stream )->ext_aux_len_ + ( p_stream )->ext_len_ + ( p_stream )->spu_len_ ) )

/******************************************************************************\
* Macros/functions for fill/flush.
\******************************************************************************/

unsigned char jas_dma_stream_fillbuf( jas_dma_stream_t* p_stream );

int jas_dma_stream_flush( jas_dma_stream_t* p_stream );

int jas_dma_stream_flushbuf( jas_dma_stream_t* p_stream, int c );

/******************************************************************************\
* Miscellaneous macros/functions.
\******************************************************************************/

/******************************************************************************\
* Internal functions.
\******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
/* s.kang end */

