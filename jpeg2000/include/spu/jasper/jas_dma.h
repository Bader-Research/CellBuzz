/*
 * Copyright (c) 1999-2000 Image Power, Inc. and the University of
 *   British Columbia.
 * Copyright (c) 2001-2002 Michael David Adams.
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
 * DMA wrapper
 *
 * $Id: jas_dma.h,v 1.5 2008/08/13 18:46:51 lrlemini Exp $
 */

#ifndef JAS_DMA_H
#define JAS_DMA_H

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <jasper/jas_config.h>
#include <jasper/jas_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
* DMA wrapper functions
\******************************************************************************/

void jas_dma_get_from_ppu( void* p_src, void* p_dst, int size );/* p_src % DMA_MIN_SIZE must be equal to p_dst % DMA_MIN_SIZE */
void jas_dma_put_to_ppu( void* p_src, void* p_dst, int size );/* p_src % DMA_MIN_SIZE must be equal to p_dst % DMA_MIN_SIZE */

/******************************************************************************\
* DMA wrapper macros
\******************************************************************************/

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( addr_src, p_dst, size )\
{\
	unsigned long long addr_src2;\
	void* p_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( addr_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED] assertion failure, addr_src=0x%x\n", ( unsigned int )addr_src ) );\
	spu_assert( ( ( ( unsigned int )( p_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED] assertion failure, p_dst=0x%x\n", ( unsigned int )p_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED] assertion failure, size=0x%x\n", size ) );\
	addr_src2 = ( addr_src );\
	p_dst2 = ( void* )( p_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_get( p_dst2, addr_src2, DMA_MAX_SIZE, 8, 0, 0 );\
		p_dst2 += DMA_MAX_SIZE;\
		addr_src2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_get( p_dst2, addr_src2, size2, 8, 0, 0 );\
	}\
	mfc_write_tag_mask( 1 << 8 );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( p_src, addr_dst, size )\
{\
	void* p_src2;\
	unsigned long long addr_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( p_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED] assertion failure, p_src=0x%x\n", ( unsigned int )p_src ) );\
	spu_assert( ( ( ( unsigned int )( addr_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED] assertion failure, addr_dst=0x%x\n", ( unsigned int )addr_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED] assertion failure, size=0x%x\n", size ) );\
	p_src2 = ( void* )( p_src );\
	addr_dst2 = ( addr_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_put( p_src2, addr_dst2, DMA_MAX_SIZE, 9, 0, 0 );\
		p_src2 += DMA_MAX_SIZE;\
		addr_dst2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_put( p_src2, addr_dst2, size2, 9, 0, 0 );\
	}\
	mfc_write_tag_mask( 1 << 9 );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( addr_src, p_dst, size )\
{\
	unsigned long long addr_src2;\
	void* p_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( addr_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, addr_src=0x%x\n", ( unsigned int )addr_src ) );\
	spu_assert( ( ( ( unsigned int )( p_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, p_dst=0x%x\n", ( unsigned int )p_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	addr_src2 = ( addr_src );\
	p_dst2 = ( void* )( p_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_get( p_dst2, addr_src2, DMA_MAX_SIZE, 10, 0, 0 );\
		p_dst2 += DMA_MAX_SIZE;\
		addr_src2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_get( p_dst2, addr_src2, size2, 10, 0, 0 );\
	}\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM()\
{\
	mfc_write_tag_mask( 1 << 10 );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( p_src, addr_dst, size )\
{\
	void* p_src2;\
	unsigned long long addr_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( p_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, p_src=0x%x\n", ( unsigned int )p_src ) );\
	spu_assert( ( ( ( unsigned int )( addr_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, addr_dst=0x%x\n", ( unsigned int )addr_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	p_src2 = ( void* )( p_src );\
	addr_dst2 = ( addr_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_put( p_src2, addr_dst2, DMA_MAX_SIZE, 11, 0, 0 );\
		p_src2 += DMA_MAX_SIZE;\
		addr_dst2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_put( p_src2, addr_dst2, size2, 11, 0, 0 );\
	}\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM()\
{\
	mfc_write_tag_mask( 1 << 11 );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE( addr_src, p_dst, size, sense )\
{\
	unsigned long long addr_src2;\
	void* p_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( addr_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, addr_src=0x%x\n", ( unsigned int )addr_src ) );\
	spu_assert( ( ( ( unsigned int )( p_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, p_dst=0x%x\n", ( unsigned int )p_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	addr_src2 = ( addr_src );\
	p_dst2 = ( void* )( p_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_get( p_dst2, addr_src2, DMA_MAX_SIZE, 12 + ( sense ), 0, 0 );\
		p_dst2 += DMA_MAX_SIZE;\
		addr_src2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_get( p_dst2, addr_src2, size2, 12 + ( sense ), 0, 0 );\
	}\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_DOUBLE_CONFIRM( sense )\
{\
	mfc_write_tag_mask( 1 << ( 12 + ( sense ) ) );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE( p_src, addr_dst, size, sense )\
{\
	void* p_src2;\
	unsigned long long addr_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( p_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, p_src=0x%x\n", ( unsigned int )p_src ) );\
	spu_assert( ( ( ( unsigned int )( addr_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, addr_dst=0x%x\n", ( unsigned int )addr_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_DOUBLE_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	p_src2 = ( void* )( p_src );\
	addr_dst2 = ( addr_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_put( p_src2, addr_dst2, DMA_MAX_SIZE, 14 + ( sense ), 0, 0 );\
		p_src2 += DMA_MAX_SIZE;\
		addr_dst2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_put( p_src2, addr_dst2, size2, 14 + ( sense ), 0, 0 );\
	}\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_DOUBLE_CONFIRM( sense )\
{\
	mfc_write_tag_mask( 1 << ( 14 + sense ) );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( addr_src, p_dst, size, sense )\
{\
	unsigned long long addr_src2;\
	void* p_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( addr_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, addr_src=0x%x\n", ( unsigned int )addr_src ) );\
	spu_assert( ( ( ( unsigned int )( p_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, p_dst=0x%x\n", ( unsigned int )p_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	addr_src2 = ( addr_src );\
	p_dst2 = ( void* )( p_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_get( p_dst2, addr_src2, DMA_MAX_SIZE, 16 + ( sense ), 0, 0 );\
		p_dst2 += DMA_MAX_SIZE;\
		addr_src2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_get( p_dst2, addr_src2, size2, 16 + ( sense ), 0, 0 );\
	}\
}

#define JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense )\
{\
	mfc_write_tag_mask( 1 << ( 16 + ( sense ) ) );\
	mfc_read_tag_status_all();\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( p_src, addr_dst, size, sense )\
{\
	void* p_src2;\
	unsigned long long addr_dst2;\
	int size2;\
	spu_assert( ( ( ( unsigned int )( p_src ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, p_src=0x%x\n", ( unsigned int )p_src ) );\
	spu_assert( ( ( ( unsigned int )( addr_dst ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, addr_dst=0x%x\n", ( unsigned int )addr_dst ) );\
	spu_assert( ( ( ( size ) % CACHE_LINE_SIZE ) == 0 ), ( "[jas_dma.h:JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE] assertion failure, size=0x%x\n", size ) );\
	p_src2 = ( void* )( p_src );\
	addr_dst2 = ( addr_dst );\
	size2 = ( size );\
	while( size2 > DMA_MAX_SIZE ) {\
		mfc_put( p_src2, addr_dst2, DMA_MAX_SIZE, 24 + ( sense ), 0, 0 );\
		p_src2 += DMA_MAX_SIZE;\
		addr_dst2 += DMA_MAX_SIZE;\
		size2 -= DMA_MAX_SIZE;\
	}\
	if( size2 != 0 ) {\
		mfc_put( p_src2, addr_dst2, size2, 24 + ( sense ), 0, 0 );\
	}\
}

#define JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense )\
{\
	mfc_write_tag_mask( 1 << ( 24 + sense ) );\
	mfc_read_tag_status_all();\
}

#ifdef __cplusplus
}
#endif

#endif
