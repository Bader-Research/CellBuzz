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
 * Rounding, Level Shfit, and Clipping
 *
 * $Id: jpc_pre_proc.c,v 1.4 2008/08/13 18:46:51 lrlemini Exp $
 */

/* s.kang start */
/******************************************************************************\
* Includes.
\******************************************************************************/

#include <spu_assert.h>

#include "jasper/jas_seq.h"
/* s.kang start */
#include "jasper/jas_dma.h"
#include "jasper/jas_malloc.h"
/* s.kang end */

#include "jpc_fix.h"
/* s.kang start */
#include "jpc_mct.h"
/* s.kang end */

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
* Code.
\******************************************************************************/

/* s.kang start */
void jpc_enc_pre_proc_with_mct( jpc_fix_t* p_data0, jpc_fix_t* p_data1, jpc_fix_t* p_data2, int stride0, int stride1, int stride2, int num_rows, int numcols, int mctid, int realmode, int adjust0, int adjust1, int adjust2 )
{
	int num_partitions;
	jpc_fix_t* aa_in_data_ptr[MULTI_BUF_DEPTH][3];
	jpc_fix_t* aa_out_data_ptr[MULTI_BUF_DEPTH][3];
	unsigned int a_ppu_nxt_r_addr[3];
	unsigned int a_ppu_nxt_r_row_addr[3];
	unsigned int a_ppu_nxt_w_addr[3];
	unsigned int a_ppu_nxt_w_row_addr[3];
	int sense;
#ifdef SIMD_EN
#ifdef FLOAT_MODE
	vector float* p_v_f_out_data0;
	vector float* p_v_f_out_data1;
	vector float* p_v_f_out_data2;
	vector float v_f_r;
	vector float v_f_g;
	vector float v_f_b;
	vector float v_f_y;
	vector float v_f_u;
	vector float v_f_v;
#else
	int r;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int g;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int b;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int y;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int u;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int v;/* jpc_fix_t * jpc_fix_t is executed in scalar mode */
	int k;
#endif
	vector signed int* p_v_in_data0;
	vector signed int* p_v_in_data1;
	vector signed int* p_v_in_data2;
	vector signed int* p_v_out_data0;
	vector signed int* p_v_out_data1;
	vector signed int* p_v_out_data2;
	vector signed int v_r;
	vector signed int v_g;
	vector signed int v_b;
	vector signed int v_y;
	vector signed int v_u;
	vector signed int v_v;
	vector signed int v_data0;
	vector signed int v_data1;
	vector signed int v_data2;
#else
#ifdef FLOAT_MODE
	float* p_f_out_data0;
	float* p_f_out_data1;
	float* p_f_out_data2;
	int f_r;
	int f_g;
	int f_b;
	int f_y;
	int f_u;
	int f_v;
#endif
	int r;
	int g;
	int b;
	int y;
	int u;
	int v;
	int data0;
	int data1;
	int data2;
#endif
	int i;
	int j;

	spu_assert( ( ( numcols % JPC_PRE_PROC_BUF_ITEMS ) == 0 ), ( "[jpc_pre_proc.c:jpc_enc_pre_proc_with_mct()] assertion failure\n" ) );
	spu_assert( ( ( stride0 == stride1 ) && ( stride1 == stride2 ) ), ( "[jpc_post_proc.c:jpc_post_proc_with_imct()] assertiona failure\n" ) );

	num_partitions = numcols / JPC_PRE_PROC_BUF_ITEMS;

	for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
		for( j = 0 ; j < 3 ; j++ ) {
			aa_in_data_ptr[i][j] = jas_malloc_align( JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
			aa_out_data_ptr[i][j] = jas_malloc_align( JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
			spu_assert( ( ( aa_in_data_ptr[i][j] != NULL ) && ( aa_out_data_ptr[i][j] != NULL ) ), ( "[jpc_post_proc.c:jpc_post_proc_with_imct()] assertion failure\n" ) );
		}
	}

	a_ppu_nxt_r_addr[0] = ( unsigned int )p_data0;
	a_ppu_nxt_r_addr[1] = ( unsigned int )p_data1;
	a_ppu_nxt_r_addr[2] = ( unsigned int )p_data2;
	for( i = 0 ; i < 3 ; i++ ) {
		a_ppu_nxt_r_row_addr[i] = a_ppu_nxt_r_addr[i];
		a_ppu_nxt_w_addr[i] = a_ppu_nxt_r_addr[i];
		a_ppu_nxt_w_row_addr[i] = a_ppu_nxt_r_addr[i];
	}

	for( i = 0 ; i < MULTI_BUF_DEPTH - 1 ; i++ ) {
		if( i >= num_rows * num_partitions ) {
			break;
		}
		for( j = 0 ; j < 3 ; j++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )a_ppu_nxt_r_addr[j], ( void* )aa_in_data_ptr[( i % MULTI_BUF_DEPTH )][j], JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), ( i % MULTI_BUF_DEPTH ) );
		}
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			for( j = 0 ; j < 3 ; j++ ) {
				a_ppu_nxt_r_addr[j] = a_ppu_nxt_r_row_addr[j] + stride0 * sizeof( jpc_fix_t );
				a_ppu_nxt_r_row_addr[j] = a_ppu_nxt_r_addr[j];
			}
		}
		else {
			for( j = 0 ; j < 3 ; j++ ) {
				a_ppu_nxt_r_addr[j] += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
			}
		}
	}

	sense = 0;

	for( i = 0 ; i < num_rows * num_partitions ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		if( i < ( num_rows * num_partitions - ( MULTI_BUF_DEPTH - 1 ) ) ) {
			for( j = 0 ; j < 3 ; j++ ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )a_ppu_nxt_r_addr[j], ( void* )aa_in_data_ptr[( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH][j], JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), ( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH );
			}
			if( ( i + 1 + MULTI_BUF_DEPTH - 1 ) % num_partitions == 0 ) {
				for( j = 0 ; j < 3 ; j++ ) {
					a_ppu_nxt_r_addr[j] = a_ppu_nxt_r_row_addr[j] + stride0 * sizeof( jpc_fix_t );
					a_ppu_nxt_r_row_addr[j] = a_ppu_nxt_r_addr[j];
				}
			}
			else {
				for( j = 0 ; j < 3 ; j++ ) {
					a_ppu_nxt_r_addr[j] += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
				}
			}
		}

#ifdef SIMD_EN
		p_v_in_data0 = ( vector signed int* )aa_in_data_ptr[sense][0];
		p_v_in_data1 = ( vector signed int* )aa_in_data_ptr[sense][1];
		p_v_in_data2 = ( vector signed int* )aa_in_data_ptr[sense][2];
		p_v_out_data0 = ( vector signed int* )aa_out_data_ptr[sense][0];
		p_v_out_data1 = ( vector signed int* )aa_out_data_ptr[sense][1];
		p_v_out_data2 = ( vector signed int* )aa_out_data_ptr[sense][2];
#ifdef FLOAT_MODE
		p_v_f_out_data0 = ( vector float* )p_v_out_data0;
		p_v_f_out_data1 = ( vector float* )p_v_out_data1;
		p_v_f_out_data2 = ( vector float* )p_v_out_data2;
#endif
#else
#ifdef FLOAT_MODE
		p_f_out_data0 = ( float* )aa_out_data_ptr[sense][0];
		p_f_out_data1 = ( float* )aa_out_data_ptr[sense][1];
		p_f_out_data2 = ( float* )aa_out_data_ptr[sense][2];
#endif
#endif

#ifdef SIMD_EN
		for( j = 0 ; j < ( int )( JPC_PRE_PROC_BUF_ITEMS / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; j++ )
#else
		for( j = 0 ; j < JPC_PRE_PROC_BUF_ITEMS ; j++ )
#endif
		{
			/* level shift */
#ifdef SIMD_EN
			v_data0 = p_v_in_data0[j];
			v_data1 = p_v_in_data1[j];
			v_data2 = p_v_in_data2[j];
			v_data0 = spu_sub( v_data0, spu_splats( adjust0 ) );
			v_data1 = spu_sub( v_data1, spu_splats( adjust1 ) );
			v_data2 = spu_sub( v_data2, spu_splats( adjust2 ) );
#else
			data0 = aa_in_data_ptr[sense][0][j];
			data1 = aa_in_data_ptr[sense][1][j];
			data2 = aa_in_data_ptr[sense][2][j];
			data0 -= adjust0;
			data1 -= adjust1;
			data2 -= adjust2;
#endif

			if( mctid == JPC_MCT_RCT ) {
#ifdef SIMD_EN
#ifdef FLOAT_MODE
				if( realmode ) {
					v_f_r = spu_convtf( v_data0, 0 );
					v_f_g = spu_convtf( v_data1, 0 );
					v_f_b = spu_convtf( v_data2, 0 );
					v_f_y = spu_mul( spu_add( v_f_r, spu_add( spu_mul( v_f_g, spu_splats( 2.0f ) ), v_f_b ) ), spu_splats( 0.25f ) );
					v_f_u = spu_sub( v_f_b, v_f_g );
					v_f_v = spu_sub( v_f_r, v_f_g );
					p_v_f_out_data0[j] = v_f_y;
					p_v_f_out_data1[j] = v_f_u;
					p_v_f_out_data2[j] = v_f_v;
				}
				else {
					v_r = v_data0;
					v_g = v_data1;
					v_b = v_data2;
					v_y = spu_rlmaska( spu_add( v_r, spu_add( spu_sl( v_g, 1 ), v_b ) ), -2 );
					v_u = spu_sub( v_b, v_g );
					v_v = spu_sub( v_r, v_g );
					p_v_out_data0[j] = v_y;
					p_v_out_data1[j] = v_u;
					p_v_out_data2[j] = v_v;
				}
#else
				if( realmode ) {
					v_r = spu_sl( v_data0, JPC_FIX_FRACBITS );
					v_g = spu_sl( v_data1, JPC_FIX_FRACBITS );
					v_b = spu_sl( v_data2, JPC_FIX_FRACBITS );
				}
				else {
					v_r = v_data0;
					v_g = v_data1;
					v_b = v_data2;
				}
				v_y = spu_rlmaska( spu_add( v_r, spu_add( spu_sl( v_g, 1 ), v_b ) ), -2 );
				v_u = spu_sub( v_b, v_g );
				v_v = spu_sub( v_r, v_g );
				p_v_out_data0[j] = v_y;
				p_v_out_data1[j] = v_u;
				p_v_out_data2[j] = v_v;
#endif
#else
#ifdef FLOAT_MODE
				if( realmode ) {
					f_r = ( float )data0;
					f_g = ( float )data1;
					f_b = ( float )data2;
					f_y = ( f_r + ( f_g * 2.0f ) + f_b ) * 0.25f;
					f_u = f_b - f_g;
					f_v = f_r - f_g;
					p_f_out_data0[j] = f_y;
					p_f_out_data1[j] = f_u;
					p_f_out_data2[j] = f_v;
				}
				else {
					r = data0;
					g = data1;
					b = data2;
					y = ( r + ( g << 1 ) + b ) >> 2;
					u = b - g;
					v = r - g;
					aa_out_data_ptr[sense][0][j] = y;
					aa_out_data_ptr[sense][1][j] = u;
					aa_out_data_ptr[sense][2][j] = v;
				}
#else
				if( realmode ) {
					r = data0 << JPC_FIX_FRACBITS;
					g = data1 << JPC_FIX_FRACBITS;
					b = data2 << JPC_FIX_FRACBITS;
				}
				else {
					r = data0;
					g = data1;
					b = data2;
				}
				y = ( r + ( g << 1 ) + b ) >> 2;
				u = b - g;
				v = r - g;
				aa_out_data_ptr[sense][0][j] = y;
				aa_out_data_ptr[sense][1][j] = u;
				aa_out_data_ptr[sense][2][j] = v;
#endif
#endif
			}
			else {
				spu_assert( ( mctid == JPC_MCT_ICT ), ( "[jpc_post_proc_with_imct()] assertion failure\n" ) );
				spu_assert( ( realmode ), ( "[jpc_post_pro.c:jpc_post_proc_with_imct()] assertion failure\n" ) );
#ifdef SIMD_EN
#ifdef FLOAT_MODE
				v_f_r = spu_convtf( v_data0, 0 );
				v_f_g = spu_convtf( v_data1, 0 );
				v_f_b = spu_convtf( v_data2, 0 );
				v_f_y = spu_add( spu_mul( spu_splats( 0.299f ), v_f_r ), spu_add( spu_mul( spu_splats( 0.587f ), v_f_g ), spu_mul( spu_splats( 0.114f ), v_f_b ) ) );
				v_f_u = spu_add( spu_mul( spu_splats( -0.16875f ), v_f_r ), spu_add( spu_mul( spu_splats( -0.33126f ), v_f_g ), spu_mul( spu_splats( 0.5f ), v_f_b ) ) );
				v_f_v = spu_add( spu_mul( spu_splats( 0.5f ), v_f_r ), spu_add( spu_mul( spu_splats( -0.41869f ), v_f_g ), spu_mul( spu_splats( -0.08131f ), v_f_b ) ) );
				p_v_f_out_data0[j] = v_f_y;
				p_v_f_out_data1[j] = v_f_u;
				p_v_f_out_data2[j] = v_f_v;
#else
				v_r = spu_sl( v_data0, JPC_FIX_FRACBITS );
				v_g = spu_sl( v_data1, JPC_FIX_FRACBITS );
				v_b = spu_sl( v_data2, JPC_FIX_FRACBITS );
				v_y = spu_splats( ( signed int )0 );
				v_u = spu_splats( ( signed int )0 );
				v_v = spu_splats( ( signed int )0 );
				for( k = 0 ; k < ( int )( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ; k++ ) {
					r = spu_extract( v_r, k );
					g = spu_extract( v_g, k );
					b = spu_extract( v_b, k );
					y = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( 0.299 ), r ), jpc_fix_mul( jpc_dbltofix( 0.587 ), g ), jpc_fix_mul( jpc_dbltofix( 0.114 ), b ) );
					u = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( -0.16875 ), r ), jpc_fix_mul( jpc_dbltofix( -0.33126 ), g ), jpc_fix_mul( jpc_dbltofix( 0.5 ), b ) );
					v = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( 0.5 ), r ), jpc_fix_mul( jpc_dbltofix( -0.41869 ), g ), jpc_fix_mul( jpc_dbltofix( -0.08131 ), b ) );
					v_y = spu_insert( y, v_y, k );
					v_u = spu_insert( u, v_u, k );
					v_v = spu_insert( v, v_v, k );

				}
				p_v_out_data0[j] = v_y;
				p_v_out_data1[j] = v_u;
				p_v_out_data2[j] = v_v;
#endif
#else
#ifdef FLOAT_MODE
				f_r = ( float )data0;
				f_g = ( float )data1;
				f_b = ( float )data2;
				f_y = 0.299f * f_r + 0.587f * f_g + 0.114f * f_b;
				f_u = ( -0.16875f ) * f_r + ( -0.33126f ) * f_g + 0.5f * f_b;
				f_v = 0.5f * f_r + ( -0.41869f ) * f_g + ( -0.08131f ) * f_b;
				p_f_out_data0[j] = f_y;
				p_f_out_data1[j] = f_u;
				p_f_out_data2[j] = f_v;
#else
				r = data0 << JPC_FIX_FRACBITS;
				g = data1 << JPC_FIX_FRACBITS;
				b = data2 << JPC_FIX_FRACBITS;
				y = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( 0.299 ), r ), jpc_fix_mul( jpc_dbltofix( 0.587 ), g ), jpc_fix_mul( jpc_dbltofix( 0.114 ), b ) );
				u = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( -0.16875 ), r ), jpc_fix_mul( jpc_dbltofix( -0.33126 ), g ), jpc_fix_mul( jpc_dbltofix( 0.5 ), b ) );
				v = jpc_fix_add3( jpc_fix_mul( jpc_dbltofix( 0.5 ), r ), jpc_fix_mul( jpc_dbltofix( -0.41869 ), g ), jpc_fix_mul( jpc_dbltofix( -0.08131 ), b ) );
				a_out_data_ptr[sense][0][j] = y;
				a_out_data_ptr[sense][1][j] = u;
				a_out_data_ptr[sense][2][j] = v;
#endif
#endif
			}
		}

		for( j = 0 ; j < 3 ; j++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )aa_out_data_ptr[sense][j], ( unsigned long long )a_ppu_nxt_w_addr[j], JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), sense );
		}
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			for( j = 0 ; j < 3 ; j++ ) {
				a_ppu_nxt_w_addr[j] = a_ppu_nxt_w_row_addr[j] + stride0 * sizeof( jpc_fix_t );
				a_ppu_nxt_w_row_addr[j] = a_ppu_nxt_w_addr[j];
			}
		}
		else {
			for( j = 0 ; j < 3 ; j++ ) {
				a_ppu_nxt_w_addr[j] += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
			}
		}

		sense = ( sense + 1 ) % MULTI_BUF_DEPTH;
	}
	JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + MULTI_BUF_DEPTH -1 ) % MULTI_BUF_DEPTH );

	for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
		for( j = 0 ; j < 3 ; j++ ) {
			jas_free_align( aa_in_data_ptr[i][j] );
			jas_free_align( aa_out_data_ptr[i][j] );
		}
	}
}

void jpc_enc_pre_proc_no_mct( void* p_data, int stride, int num_rows, int numcols, int realmode, int adjust )
{
	int num_partitions;
	jpc_fix_t* a_in_data_ptr[MULTI_BUF_DEPTH];
	jpc_fix_t* a_out_data_ptr[MULTI_BUF_DEPTH];
#ifdef FLOAT_MODE
	float* p_f_out_data;
#endif
	unsigned int ppu_nxt_r_addr;
	unsigned int ppu_nxt_r_row_addr;
	unsigned int ppu_nxt_w_addr;
	unsigned int ppu_nxt_w_row_addr;
	int sense;
	int data;
	int i;
	int j;

	spu_assert( ( ( numcols % JPC_PRE_PROC_BUF_ITEMS ) == 0 ), ( "[jpc_pre_proc.c:jpc_enc_pre_proc_no_mct()] assertion failure\n" ) );

	num_partitions = numcols / JPC_PRE_PROC_BUF_ITEMS;

	for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
		a_in_data_ptr[i] = jas_malloc_align( JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		a_out_data_ptr[i] = jas_malloc_align( JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( ( a_in_data_ptr[i] != NULL ) && ( a_out_data_ptr[i] != NULL ) ), ( "[jpc_post_proc.c:jpc_dec_post_proc_no_imct()] assertion failure\n" ) );
	}

	ppu_nxt_r_addr = ( unsigned int )p_data;
	ppu_nxt_r_row_addr = ppu_nxt_r_addr;
	ppu_nxt_w_addr = ppu_nxt_r_addr;
	ppu_nxt_w_row_addr = ppu_nxt_r_addr;

	for( i = 0 ; i < MULTI_BUF_DEPTH - 1 ; i++ ) {
		if( i >= num_rows * num_partitions ) {
			break;
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( i % MULTI_BUF_DEPTH )], JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), ( i % MULTI_BUF_DEPTH ) );
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
			ppu_nxt_r_row_addr = ppu_nxt_r_addr;
		}
		else {
			ppu_nxt_r_addr += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
		}
	}

	sense = 0;

	for( i = 0 ; i < num_rows * num_partitions ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		if( i < ( num_rows * num_partitions - ( MULTI_BUF_DEPTH - 1 ) ) ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH], JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), ( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH );
			if( ( i + 1 + MULTI_BUF_DEPTH - 1 ) % num_partitions == 0 ) {
				ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
				ppu_nxt_r_row_addr = ppu_nxt_r_addr;
			}
			else {
				ppu_nxt_r_addr += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
			}
		}

#ifdef FLOAT_MODE
		p_f_out_data = ( float* )a_out_data_ptr[sense];
#endif
		for( j = 0 ; j < JPC_PRE_PROC_BUF_ITEMS ; j++ ) {
			data = a_in_data_ptr[sense][j];
			/* level shift */
			data -= adjust;
			/* asl */
			if( realmode ) {
#ifdef FLOAT_MODE
				p_f_out_data[j] = ( float )data;
#else
				data <<= JPC_FIX_FRACBITS;
				a_out_data_ptr[sense][j] = data;
#endif
			}
			else {
				a_out_data_ptr[sense][j] = data;
			}
		}

                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_data_ptr[sense], ( unsigned long long )ppu_nxt_w_addr, JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t ), sense );
                if( ( ( i + 1 ) % num_partitions ) == 0 ) {
                        ppu_nxt_w_addr = ppu_nxt_w_row_addr + stride * sizeof( jpc_fix_t );
                        ppu_nxt_w_row_addr = ppu_nxt_w_addr;
                }
                else {
                        ppu_nxt_w_addr += JPC_POST_PROC_BUF_ITEMS * sizeof( jpc_fix_t );
                }
                sense = ( sense + 1 ) % MULTI_BUF_DEPTH;
	}
        JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + MULTI_BUF_DEPTH -1 ) % MULTI_BUF_DEPTH );

        for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
                jas_free_align( a_in_data_ptr[i] );
                jas_free_align( a_out_data_ptr[i] );
        }
}
/* s.kang end */

