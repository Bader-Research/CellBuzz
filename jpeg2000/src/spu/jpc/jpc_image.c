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
 * read from and write to component stream
 *
 * $Id: jpc_image.c,v 1.2 2008/08/13 18:46:51 lrlemini Exp $
 */

/* s.kang start */
/******************************************************************************\
* Includes.
\******************************************************************************/

#include <spu_assert.h>

#include "jasper/jas_seq.h"
#include "jasper/jas_dma.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"

#include "jpc_fix.h"

#include "jasper-cell.h"

/******************************************************************************\
* Code.
\******************************************************************************/

void jpc_readcmpt( void* p_cmpt_data, void* p_matrix_data, int cmpt_data_stride, int matrix_stride, int numrows, int numcols, int cps, int prec, int sgnd )
{
	int num_partitions;
	unsigned char* a_in_cmpt_dataptr[2];
	jpc_fix_t* a_out_matrix_dataptr[2];
	unsigned int ppu_cmpt_dataaddr;
	unsigned int ppu_cmpt_datarowaddr;
	unsigned int ppu_matrix_dataaddr;
	unsigned int ppu_matrix_datarowaddr;
	int sense;
	int dma_wr_scheduled;
	jas_seqent_t v;
	int c;
	int i;
	int j;
	int k;

	spu_assert( ( ( numcols % JPC_RW_CMPT_BUF_ITEMS ) == 0 ), ( "[jpc_image.c:jpc_readcmpt()] assertion failure\n" ) );

	num_partitions = numcols / JPC_RW_CMPT_BUF_ITEMS;

	a_in_cmpt_dataptr[0] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * cps, CACHE_LINE_OFFSET_BITS );
	a_in_cmpt_dataptr[1] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * cps, CACHE_LINE_OFFSET_BITS );
	a_out_matrix_dataptr[0] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_matrix_dataptr[1] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( ( a_in_cmpt_dataptr[0] != NULL ) && ( a_in_cmpt_dataptr[1] != NULL ) && ( a_out_matrix_dataptr[0] != NULL ) && ( a_out_matrix_dataptr[1] != NULL ) ), ( "[jpc_image.c:jpc_readcmpt()] assertion failure\n" ) );

	ppu_cmpt_dataaddr = ( unsigned int )p_cmpt_data;
	ppu_cmpt_datarowaddr = ppu_cmpt_dataaddr;
	ppu_matrix_dataaddr = ( unsigned int )p_matrix_data;
	ppu_matrix_datarowaddr = ppu_matrix_dataaddr;
	sense = 0;
	dma_wr_scheduled = 0;
	JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_cmpt_dataaddr, ( void* )a_in_cmpt_dataptr[sense], JPC_RW_CMPT_BUF_ITEMS * cps );
	for( i = 0 ; i < numrows * num_partitions ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		if( i != ( numrows * num_partitions - 1 ) ) {
			if( ( ( i + 1 ) % num_partitions ) == 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_cmpt_datarowaddr + cmpt_data_stride * cps, ( void* )a_in_cmpt_dataptr[( sense + 1 ) & 0x1], JPC_RW_CMPT_BUF_ITEMS * cps );
			}
			else {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_cmpt_dataaddr + JPC_RW_CMPT_BUF_ITEMS * cps, ( void* )a_in_cmpt_dataptr[( sense + 1 ) & 0x1], JPC_RW_CMPT_BUF_ITEMS * cps );
			}
		}
		for( j = 0 ; j < JPC_RW_CMPT_BUF_ITEMS ; j++ ) {
			v = 0;
			for( k = 0 ; k < cps ; k++ ) {
				c = a_in_cmpt_dataptr[sense][j * cps + k];
				v = ( v << 8 ) | ( c & 0xff );
			}
			v &= JAS_ONES( prec );
			a_out_matrix_dataptr[sense][j] = ( ( sgnd && ( v & ( 1 << ( prec - 1 ) ) ) ) ? ( v - ( 1 << prec ) ) : v );
		}
		if( dma_wr_scheduled ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_matrix_dataptr[sense], ( unsigned long long )ppu_matrix_dataaddr, JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ) );
		dma_wr_scheduled = 1;
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			ppu_cmpt_dataaddr = ppu_cmpt_datarowaddr + cmpt_data_stride;
			ppu_cmpt_datarowaddr = ppu_cmpt_dataaddr;
			ppu_matrix_dataaddr = ppu_matrix_datarowaddr + matrix_stride * sizeof( jpc_fix_t );
			ppu_matrix_datarowaddr = ppu_matrix_dataaddr;
		}
		else {
			ppu_cmpt_dataaddr = ppu_cmpt_dataaddr + JPC_RW_CMPT_BUF_ITEMS * cps;
			ppu_matrix_dataaddr = ppu_matrix_dataaddr + JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t );
		}
		sense = ( sense + 1 ) & 0x1;
	}
	JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

	jas_free_align( a_in_cmpt_dataptr[0] );
	jas_free_align( a_in_cmpt_dataptr[1] );
	jas_free_align( a_out_matrix_dataptr[0] );
	jas_free_align( a_out_matrix_dataptr[1] );
}

void jpc_writecmpt( void* p_matrix_data, void* p_cmpt_data, int matrix_stride, int cmpt_data_stride, int numrows, int numcols, int cps, int prec, int sgnd )
{
	int num_partitions;
	jpc_fix_t* a_in_matrix_dataptr[2];
	unsigned char* a_out_cmpt_dataptr[2];
	unsigned int ppu_matrix_dataaddr;
	unsigned int ppu_matrix_datarowaddr;
	unsigned int ppu_cmpt_dataaddr;
	unsigned int ppu_cmpt_datarowaddr;
	int sense;
	int dma_wr_scheduled;
	jpc_fix_t data;
	jas_seqent_t v;
	int c;
	int i;
	int j;
	int k;

	spu_assert( ( ( numcols % JPC_RW_CMPT_BUF_ITEMS ) == 0 ), ( "[jpc_image.c:jpc_writecmpt()] assertion failure\n" ) );

	num_partitions = numcols / JPC_RW_CMPT_BUF_ITEMS;

	a_in_matrix_dataptr[0] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_matrix_dataptr[1] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_cmpt_dataptr[0] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_cmpt_dataptr[1] = jas_malloc_align( JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( ( a_in_matrix_dataptr[0] != NULL ) && ( a_in_matrix_dataptr[1] != NULL ) && ( a_out_cmpt_dataptr[0] != NULL ) && ( a_out_cmpt_dataptr[1] != NULL ) ), ( "[jpc_image.c:jpc_writecmpt()] assertion failure\n" ) );

	ppu_matrix_dataaddr = ( unsigned int )p_matrix_data;
	ppu_matrix_datarowaddr = ppu_matrix_dataaddr;
	ppu_cmpt_dataaddr = ( unsigned int )p_cmpt_data;
	ppu_cmpt_datarowaddr = ppu_cmpt_dataaddr;
	sense = 0;
	dma_wr_scheduled = 0;
	JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_matrix_dataaddr, ( void* )a_in_matrix_dataptr[sense], JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ) );
	for( i = 0 ; i < numrows * num_partitions ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		if( i != ( numrows * num_partitions - 1 ) ) {
			if( ( ( i + 1 ) % num_partitions ) == 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_matrix_datarowaddr + matrix_stride * sizeof( jpc_fix_t ), ( void* )a_in_matrix_dataptr[( sense + 1 ) & 0x1], JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ) );
			}
			else {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_matrix_dataaddr + JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ), ( void* )a_in_matrix_dataptr[( sense + 1 ) & 0x1], JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t ) );
			}
		}
		for( j = 0 ; j < JPC_RW_CMPT_BUF_ITEMS ; j++ ) {
			data = a_in_matrix_dataptr[sense][j];
			v = ( ( sgnd && data < 0 ) ? ( ( 1 << prec ) + data ) : data ) & JAS_ONES( prec );
			for( k = 0 ; k < cps ; k++ ) {
				c = ( v >> ( 8 * ( cps - 1 ) ) ) & 0xff;
				a_out_cmpt_dataptr[sense][j * cps + k] = c;
				v <<= 8;
			}
		}
		if( dma_wr_scheduled ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_cmpt_dataptr[sense], ( unsigned long long )ppu_cmpt_dataaddr, JPC_RW_CMPT_BUF_ITEMS * cps );
		dma_wr_scheduled = 1;
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			ppu_matrix_dataaddr = ppu_matrix_datarowaddr + matrix_stride * sizeof( jpc_fix_t );
			ppu_matrix_datarowaddr = ppu_matrix_dataaddr;
			ppu_cmpt_dataaddr = ppu_cmpt_datarowaddr + cmpt_data_stride;
			ppu_cmpt_datarowaddr = ppu_cmpt_dataaddr;
		}
		else {
			ppu_matrix_dataaddr = ppu_matrix_dataaddr + JPC_RW_CMPT_BUF_ITEMS * sizeof( jpc_fix_t );
			ppu_cmpt_dataaddr = ppu_cmpt_dataaddr + JPC_RW_CMPT_BUF_ITEMS * cps;
		}
		sense = ( sense + 1 ) & 0x1;
	}
	JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

	jas_free_align( a_in_matrix_dataptr[0] );
	jas_free_align( a_in_matrix_dataptr[1] );
	jas_free_align( a_out_cmpt_dataptr[0] );
	jas_free_align( a_out_cmpt_dataptr[1] );
}
/* s.kang end */

