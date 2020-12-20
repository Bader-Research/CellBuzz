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
 * DMA wrapper Library
 *
 * $Id: jas_dma.c,v 1.3 2007/09/12 23:58:56 lrlemini Exp $
 */

/******************************************************************************\
* Includes.
\******************************************************************************/

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

/* s.kang start */
#include <spu_assert.h>
#include <spu_mfcio.h>
#include <spu_internals.h>
/* s.kang end */

#include "jasper/jas_types.h"
#include "jasper/jas_stream.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_debug.h"
#include "jasper/jas_mutex.h"
#include "jasper/jas_dma.h"
/* s.kang end */

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/* s.kang start */
/******************************************************************************\
* Local function prototypes.
\******************************************************************************/

/******************************************************************************\
* Local data.
\******************************************************************************/

/******************************************************************************\
* Code for DMA wrapper
\******************************************************************************/

void jas_dma_get_from_ppu( void* p_src, void* p_dst, int size )
{
	unsigned char a_tmp[DMA_MIN_SIZE] __attribute__ ( ( aligned( DMA_MIN_SIZE ) ) );
	int offset;

	if( ( ( unsigned int )p_src % DMA_MIN_SIZE ) != ( ( unsigned int )p_dst % DMA_MIN_SIZE ) ) {
		jas_eprintf( "[jas_dma.c:jas_dma_get_from_ppu()] alignment error(%x, %x)\n", ( unsigned int )p_src, ( unsigned int )p_dst );
	}
	spu_assert( ( ( ( unsigned int )p_src % DMA_MIN_SIZE ) == ( ( unsigned int )p_dst % DMA_MIN_SIZE ) ), ( "[jas_dma_stream.c:jas_dma_get_from_ppu()] assertion failure\n" ) );

	if( ( ( unsigned int )p_src % DMA_MIN_SIZE ) != 0 ) {
		offset = ( unsigned int )p_src % DMA_MIN_SIZE;
		mfc_get( a_tmp, ( unsigned long long )( ( unsigned int )p_src & DMA_MIN_OFFSET_MASK ), DMA_MIN_SIZE, 31, 0, 0 );
		mfc_write_tag_mask( 1 << 31 );
		mfc_read_tag_status_all();
		if( size > DMA_MIN_SIZE - offset ) {
			memcpy( p_dst, &a_tmp[offset], DMA_MIN_SIZE - offset );
			p_src += DMA_MIN_SIZE - offset;
			p_dst += DMA_MIN_SIZE - offset;
			size -= DMA_MIN_SIZE - offset;
		}
		else {
			memcpy( p_dst, &a_tmp[offset], size );
			return;
		}
	}

	mfc_get( p_dst, ( unsigned long long )( ( unsigned int )p_src ), size & DMA_MIN_OFFSET_MASK, 31, 0, 0 );
	mfc_write_tag_mask( 1 << 31 );
	mfc_read_tag_status_all();
	p_src += size & DMA_MIN_OFFSET_MASK;
	p_dst += size & DMA_MIN_OFFSET_MASK;
	size -= size & DMA_MIN_OFFSET_MASK;

	if( size != 0 ) {
		mfc_get( a_tmp, ( unsigned long long )( ( unsigned int )p_src ), DMA_MIN_SIZE, 31, 0, 0 );
		mfc_write_tag_mask( 1 << 31 );
		mfc_read_tag_status_all();
		memcpy( p_dst, &a_tmp[0], size );
	}

	return;
}

void jas_dma_put_to_ppu( void* p_src, void* p_dst, int size )
{
	int offset;
	int cpy_size;
	int i;

	spu_assert( ( ( ( unsigned int )p_src % DMA_MIN_SIZE ) == ( ( unsigned int )p_dst % DMA_MIN_SIZE ) ), ( "[jas_dma_stream.c:jas_dma_put_to_ppu()] assertion failure, alignment error\n" ) );
	spu_assert( ( DMA_MIN_SIZE == 16 ), ( "[jas_dma_stream.c:jas_dma_put_to_ppu()] assertion failure, constant DMA_MIN_SIZE != 16\n" ) );

	if( ( ( unsigned int )p_src % DMA_MIN_SIZE ) != 0 ) {
		offset = ( unsigned int )p_src % DMA_MIN_SIZE;
		spu_assert( ( ( offset % 4 ) == 0 ), ( "[jas_dma_stream.c:jas_dma_put_to_ppu()] assertion failure\n" ) );
		if( size > DMA_MIN_SIZE - offset ) {
			cpy_size = DMA_MIN_SIZE - offset;
		}
		else {
			cpy_size = size;
		}
		for( i = 0 ; i < cpy_size ; i += 4 ) {
			mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), 4, 20, 0, 0 );
			p_src += 4;
			p_dst += 4;
			size -= 4;	
		}
	}

	if( size >= DMA_MIN_SIZE ) {
		mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), size & DMA_MIN_OFFSET_MASK, 20, 0, 0 );
		p_src += size & DMA_MIN_OFFSET_MASK;
		p_dst += size & DMA_MIN_OFFSET_MASK;
		size -= size & DMA_MIN_OFFSET_MASK;
	}

	if( size != 0 ) {
		if( size >= 8 ) {
			mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), 8, 20, 0, 0 );
			p_src += 8;
			p_dst += 8;
			size -= 8;
		}
		if( size >= 4 ) {
			mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), 4, 20, 0, 0 );
			p_src += 4;
			p_dst += 4;
			size -= 4;
		}
		if( size >= 2 ) {
			mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), 2, 20, 0, 0 );
			p_src += 2;
			p_dst += 2;
			size -= 2;
		}
		if( size >= 1 ) {
			mfc_put( p_src, ( unsigned long long )( ( unsigned int )p_dst ), 1, 20, 0, 0 );
			p_src += 1;
			p_dst += 1;
			size -= 1;
		}
	}

	mfc_write_tag_mask( 1 << 20 );
	mfc_read_tag_status_all();

	return;
}

