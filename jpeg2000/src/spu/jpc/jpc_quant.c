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
 * $Id: jpc_quant.c,v 1.4 2008/08/13 18:46:52 lrlemini Exp $
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
#include "jasper/jas_math.h"
/* s.kang end */

#include "jpc_fix.h"
/* s.kang start */
#include "jpc_enc.h"
#include "jpc_t1cod.h"
/* s.kang end */

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
* Code.
\******************************************************************************/

/* s.kang start */
jpc_fix_t jpc_max_mag( void* p_data, int stride, int num_rows, int numcols, int realmode )
{
	int num_partitions;
	jpc_fix_t* a_in_data_ptr[MULTI_BUF_DEPTH];
	jpc_fix_t* a_out_data_ptr[MULTI_BUF_DEPTH];
	unsigned int ppu_nxt_r_addr;
	unsigned int ppu_nxt_r_row_addr;
	unsigned int ppu_nxt_w_addr;
	unsigned int ppu_nxt_w_row_addr;
	int sense;
	jpc_fix_t data;
	jpc_fix_t mxmag;
#ifdef FLOAT_MODE
	float* p_f_data;
	float f_mxmag;
	float f_data;
#else
#endif
	int i;
	int j;

	spu_assert( ( ( numcols % JPC_QUANT0_BUF_ITEMS ) == 0 ), ( "[spu_quant.c:jpc_max_mag()] assertion failure\n" ) );

	mxmag = 0;
#ifdef FLOAT_MODE
	f_mxmag = 0.0f;
#endif

	num_partitions = numcols / JPC_QUANT0_BUF_ITEMS;

	for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
		a_in_data_ptr[i] = jas_malloc_align( JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_in_data_ptr[i] != NULL ), ( "[jpc_quant.c:jpc_max_mag()] assertion failure\n" ) );
		if( realmode == 0 ) {
			a_out_data_ptr[i] = jas_malloc_align( JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
			spu_assert( ( a_out_data_ptr[i] != NULL ), ( "[jpc_quant.c:jpc_max_mag()] assertion failure\n" ) );
		}
	}

	ppu_nxt_r_addr = ( unsigned int )p_data;
	ppu_nxt_r_row_addr = ppu_nxt_r_addr;
	ppu_nxt_w_addr = ppu_nxt_r_addr;
	ppu_nxt_w_row_addr = ppu_nxt_r_addr;

	for( i = 0 ; i < MULTI_BUF_DEPTH - 1 ; i++ ) {
		if( i >= num_rows * num_partitions ) {
			break;
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( i % MULTI_BUF_DEPTH )], JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ), ( i % MULTI_BUF_DEPTH ) );
		if( ( ( i + 1 ) % num_partitions ) == 0 ) {
			ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
			ppu_nxt_r_row_addr = ppu_nxt_r_addr;
		}
		else {
			ppu_nxt_r_addr += JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t );
		}
	}

	sense = 0;

	for( i = 0 ; i < num_rows * num_partitions ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
		if( i < ( num_rows * num_partitions - ( MULTI_BUF_DEPTH - 1 ) ) ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH], JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ), ( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH );
			if( ( i + 1 + MULTI_BUF_DEPTH - 1 ) % num_partitions == 0 ) {
				ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
				ppu_nxt_r_row_addr = ppu_nxt_r_addr;
			}
			else {
				ppu_nxt_r_addr += JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t );
			}
		}

#ifdef FLOAT_MODE
		if( realmode ) {
			p_f_data = ( float* )a_in_data_ptr[sense];
			for( j = 0 ; j < JPC_QUANT0_BUF_ITEMS ; j++ ) {
				f_data = p_f_data[j];
				if( fabsf( f_data ) > f_mxmag ) {
					f_mxmag = fabsf( f_data );
				}
			}
		}
		else {
			for( j = 0 ; j < JPC_QUANT0_BUF_ITEMS ; j++ ) {
				data = a_in_data_ptr[sense][j];
				if( abs( data ) > mxmag ) {
					mxmag = abs( data );
				}
				a_out_data_ptr[sense][j] = data << JPC_NUMEXTRABITS;
			}
		}
#else
		for( j = 0 ; j < JPC_QUANT0_BUF_ITEMS ; j++ ) {
			data = a_in_data_ptr[sense][j];
			if( abs( data ) > mxmag ) {
				mxmag = abs( data );
			}
			if( realmode == 0 ) {
				a_out_data_ptr[sense][j] = data << JPC_NUMEXTRABITS;
			}
		}
#endif
		if( realmode == 0 ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_data_ptr[sense], ( unsigned long long )ppu_nxt_w_addr, JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t ), sense );
			if( ( ( i + 1 ) % num_partitions ) == 0 ) {
				ppu_nxt_w_addr = ppu_nxt_w_row_addr + stride * sizeof( jpc_fix_t );
				ppu_nxt_w_row_addr = ppu_nxt_w_addr;
			}
			else {
				ppu_nxt_w_addr += JPC_QUANT0_BUF_ITEMS * sizeof( jpc_fix_t );
			}
		}
		sense = ( sense + 1 ) % MULTI_BUF_DEPTH;
	}
	if( realmode == 0 ) {
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + MULTI_BUF_DEPTH -1 ) % MULTI_BUF_DEPTH );
	}

        for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
                jas_free_align( a_in_data_ptr[i] );
		if( realmode == 0 ) {
                	jas_free_align( a_out_data_ptr[i] );
		}
        }

#ifdef FLOAT_MODE
	if( realmode ) {
		return jpc_dbltofix( ( double )f_mxmag );
	}
	else {
		return mxmag;
	}
#else
	return mxmag;
#endif
}

void jpc_quantize( void* p_data, int stride, int num_rows, int numcols, int absstepsize )
{
	int num_partitions;
	jpc_fix_t* a_in_data_ptr[MULTI_BUF_DEPTH];
	jpc_fix_t* a_out_data_ptr[MULTI_BUF_DEPTH];
#ifdef SIMD_EN
#ifdef FLOAT_MODE
	vector float* p_v_f_in_data;
	vector float v_f_data;
#else
	vector signed int* p_v_in_data;
	vector signed int v_data;
	int k;
#endif
	vector signed int* p_v_out_data;
#else
#ifdef FLOAT_MODE
	float* p_f_data;
	float f_data;
	float f_scale;
#endif
#endif
#ifdef FLOAT_MODE
	float f_one_over_absstepsize;
#else
	jpc_fix_t data;
#endif
        unsigned int ppu_nxt_r_addr;
        unsigned int ppu_nxt_r_row_addr;
        unsigned int ppu_nxt_w_addr;
        unsigned int ppu_nxt_w_row_addr;
	int sense;
	int i;
	int j;

	spu_assert( ( ( numcols % JPC_QUANT1_BUF_ITEMS ) == 0 ), ( "[jpc_quant.c:jpc_quantize()] assertion failure\n" ) );

#ifdef FLOAT_MODE
	f_one_over_absstepsize = ( float )( 1.0 / jpc_fixtodbl( absstepsize ) );
#ifndef SIMD_EN
	f_scale = powf( 2.0f, ( float )JPC_NUMEXTRABITS );
#endif
#endif

	num_partitions = numcols / JPC_QUANT1_BUF_ITEMS;

        for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
                a_in_data_ptr[i] = jas_malloc_align( JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
                a_out_data_ptr[i] = jas_malloc_align( JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
                spu_assert( ( ( a_in_data_ptr[i] != NULL ) && ( a_out_data_ptr[i] != NULL ) ), ( "[jpc_quant.c:jpc_quantize()] assertion failure\n" ) );
        }

        ppu_nxt_r_addr = ( unsigned int )p_data;
        ppu_nxt_r_row_addr = ppu_nxt_r_addr;
        ppu_nxt_w_addr = ppu_nxt_r_addr;
        ppu_nxt_w_row_addr = ppu_nxt_r_addr;

        for( i = 0 ; i < MULTI_BUF_DEPTH - 1 ; i++ ) {
		if( i >= num_rows * num_partitions ) {
			break;
		}
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( i % MULTI_BUF_DEPTH )], JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), ( i % MULTI_BUF_DEPTH ) );
                if( ( ( i + 1 ) % num_partitions ) == 0 ) {
                        ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
                        ppu_nxt_r_row_addr = ppu_nxt_r_addr;
                }
                else {
                        ppu_nxt_r_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
                }
        }

	sense = 0;

	for( i = 0 ; i < num_rows * num_partitions ; i++ ) {
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
                if( i < ( num_rows * num_partitions - ( MULTI_BUF_DEPTH - 1 ) ) ) {
                        JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH], JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), ( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH );
                        if( ( i + 1 + MULTI_BUF_DEPTH - 1 ) % num_partitions == 0 ) {
                                ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
                                ppu_nxt_r_row_addr = ppu_nxt_r_addr;
                        }
                        else {
                                ppu_nxt_r_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
                        }
                }

#ifdef SIMD_EN
#ifdef FLOAT_MODE
		p_v_f_in_data = ( vector float* )a_in_data_ptr[sense];
#else
		p_v_in_data = ( vector signed int* )a_in_data_ptr[sense];
#endif
		p_v_out_data = ( vector signed int* )a_out_data_ptr[sense];
#else
#ifdef FLOAT_MODE
		p_f_data = ( float* )a_in_data_ptr[sense];
#endif
#endif
#ifdef SIMD_EN
		for( j = 0 ; j < ( int )( JPC_QUANT1_BUF_ITEMS / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; j++ )
#else
		for( j = 0 ; j < JPC_QUANT1_BUF_ITEMS ; j++ )
#endif
		{
#ifdef SIMD_EN
#ifdef FLOAT_MODE
			v_f_data = p_v_f_in_data[j];
			v_f_data = spu_mul( v_f_data, spu_splats( f_one_over_absstepsize ) );
			p_v_out_data[j] = spu_convts( v_f_data, JPC_NUMEXTRABITS );
#else
			v_data = p_v_in_data[j];
			for( k = 0 ; k < 4 ; k++ ) {
				data = spu_extract( v_data, k );
				if( data < 0 ) {
					data = jpc_fix_neg( jpc_fix_div( jpc_fix_neg( data ), absstepsize ) );
				}
				else {
					data = jpc_fix_div( data, absstepsize );
				}
				data = ( data >= 0 )? ( data >> ( JPC_FIX_FRACBITS - JPC_NUMEXTRABITS ) ) : ( -( ( -( data ) ) >> ( JPC_FIX_FRACBITS - JPC_NUMEXTRABITS ) ) );
				v_data = spu_insert( data, v_data, k );
			}
			p_v_out_data[j] = v_data;
#endif
#else
#ifdef FLOAT_MODE
			f_data = p_f_data[j];
			f_data *= f_one_over_absstepsize * f_scale;
			a_out_data_ptr[sense][j] = ( int )f_data;
#else
			data = a_in_data_ptr[sense][j];
			if( data < 0 ) {
				data = jpc_fix_neg( jpc_fix_div( jpc_fix_neg( data ), absstepsize ) );
			}
			else {
				data = jpc_fix_div( data, absstepsize );
			}
			data = ( data >= 0 )? ( data >> ( JPC_FIX_FRACBITS - JPC_NUMEXTRABITS ) ) : ( -( ( -( data ) ) >> ( JPC_FIX_FRACBITS - JPC_NUMEXTRABITS ) ) );
			a_out_data_ptr[sense][j] = data;
#endif
#endif
		}

                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_data_ptr[sense], ( unsigned long long )ppu_nxt_w_addr, JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), sense );
                if( ( ( i + 1 ) % num_partitions ) == 0 ) {
                        ppu_nxt_w_addr = ppu_nxt_w_row_addr + stride * sizeof( jpc_fix_t );
                        ppu_nxt_w_row_addr = ppu_nxt_w_addr;
                }
                else {
                        ppu_nxt_w_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
                }
                sense = ( sense + 1 ) % MULTI_BUF_DEPTH;
	}
        JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + MULTI_BUF_DEPTH -1 ) % MULTI_BUF_DEPTH );

        for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
                jas_free_align( a_in_data_ptr[i] );
                jas_free_align( a_out_data_ptr[i] );
        }
}

void jpc_dequantize( void* p_data, int stride, int num_rows, int numcols, int roishift, int bgshift, int numbps, int realmode, int absstepsize )
{
	int num_partitions;
	jpc_fix_t* a_in_data_ptr[MULTI_BUF_DEPTH];
	jpc_fix_t* a_out_data_ptr[MULTI_BUF_DEPTH];
#ifdef FLOAT_MODE
	float* p_f_out_data_ptr;
	float f_absstepsize;
#ifdef SIMD_EN
	vector float v_flt_data;
#else
	float flt_data;
#endif
#endif
#ifdef SIMD_EN
	vector signed int v_data;
#endif
        unsigned int ppu_nxt_r_addr;
        unsigned int ppu_nxt_r_row_addr;
        unsigned int ppu_nxt_w_addr;
        unsigned int ppu_nxt_w_row_addr;
	int sense;
	int thresh;
	jpc_fix_t data;
	jpc_fix_t mag;
	bool warn;
	uint_fast32_t mask;
	int i;
	int j;
#ifndef FLOAT_MODE
#ifdef SIMD_EN
	int k;
#endif
#endif

	spu_assert( ( ( numcols % JPC_QUANT1_BUF_ITEMS ) == 0 ), ( "[jpc_quant.c:jpc_dequantize()] assertion failure\n" ) );

#ifdef FLOAT_MODE
	f_absstepsize = ( float )jpc_fixtodbl( absstepsize );
#endif

	thresh = 1 << roishift;

	num_partitions = numcols / JPC_QUANT1_BUF_ITEMS;

        for( i = 0 ; i < MULTI_BUF_DEPTH ; i++ ) {
                a_in_data_ptr[i] = jas_malloc_align( JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
                a_out_data_ptr[i] = jas_malloc_align( JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
                spu_assert( ( ( a_in_data_ptr[i] != NULL ) && ( a_out_data_ptr[i] != NULL ) ), ( "[jpc_quant.c:jpc_dequantize()] assertion failure\n" ) );
        }

        ppu_nxt_r_addr = ( unsigned int )p_data;
        ppu_nxt_r_row_addr = ppu_nxt_r_addr;
        ppu_nxt_w_addr = ppu_nxt_r_addr;
        ppu_nxt_w_row_addr = ppu_nxt_r_addr;

        for( i = 0 ; i < MULTI_BUF_DEPTH - 1 ; i++ ) {
		if( i >= num_rows * num_partitions ) {
			break;
		}
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( i % MULTI_BUF_DEPTH )], JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), ( i % MULTI_BUF_DEPTH ) );
                if( ( ( i + 1 ) % num_partitions ) == 0 ) {
                        ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
                        ppu_nxt_r_row_addr = ppu_nxt_r_addr;
                }
                else {
                        ppu_nxt_r_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
                }
        }

	sense = 0;

	for( i = 0 ; i < num_rows * num_partitions ; i++ ) {
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
                if( i < ( num_rows * num_partitions - ( MULTI_BUF_DEPTH - 1 ) ) ) {
                        JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_addr, ( void* )a_in_data_ptr[( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH], JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), ( sense + MULTI_BUF_DEPTH - 1 ) % MULTI_BUF_DEPTH );
                        if( ( i + 1 + MULTI_BUF_DEPTH - 1 ) % num_partitions == 0 ) {
                                ppu_nxt_r_addr = ppu_nxt_r_row_addr + stride * sizeof( jpc_fix_t );
                                ppu_nxt_r_row_addr = ppu_nxt_r_addr;
                        }
                        else {
                                ppu_nxt_r_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
                        }
                }

		if( roishift != 0 || bgshift != 0 ) {
			for( j = 0 ; j < JPC_QUANT1_BUF_ITEMS ; j++ ) {
				data = a_in_data_ptr[sense][j];
				mag = JAS_ABS( data );
				if( mag >= thresh ) {
					/* we are dealing with ROI data */
					mag >>= roishift;
					data = ( data < 0 ) ? ( -mag ) : mag;
				}
				else {
					/* We are dealing with non-ROI (i.e., background) data. */
					mag <<= bgshift;
					mask = ( 1 << numbps ) - 1;
					/* Perform a basic sanity check on the sample value. */
					/* Some implementations write garbage in the unused
					  most-significant bit planes introduced by ROI shifting.
					  Here we ensure that any such bits are masked off. */
					if( mag & ( ~mask ) ) {
						if( !warn ) {
							jas_eprintf( "warning: possibly corrupt code stream\n" );
							warn = true;
						}
						mag &= mask;
					}
					data = ( data < 0 ) ? ( -mag ) : mag;
				}
				a_in_data_ptr[sense][j] = data;
			}
		}
#ifdef FLOAT_MODE
		p_f_out_data_ptr = ( float* )a_out_data_ptr[sense];
#endif
#ifdef SIMD_EN
		for( j = 0 ; j < ( int )( JPC_QUANT1_BUF_ITEMS / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; j++ )
#else
		for( j = 0 ; j < JPC_QUANT1_BUF_ITEMS ; j++ )
#endif
		{
#ifdef SIMD_EN
			v_data = ( ( vector signed int* )( a_in_data_ptr[sense] ) )[j];
#else
			data = a_in_data_ptr[sense][j];
#endif
			if( realmode ) {
#ifdef SIMD_EN
#ifdef FLOAT_MODE
				v_flt_data = spu_convtf( v_data, 0 );
				if( absstepsize != jpc_inttofix( 1 ) ) {
					v_flt_data = spu_mul( v_flt_data, spu_splats( f_absstepsize ) );
				}
				( ( vector float* )p_f_out_data_ptr )[j] = v_flt_data;
#else
				v_data = spu_sl( v_data, JPC_FIX_FRACBITS );
				if( absstepsize != jpc_inttofix( 1 ) ) {
					for( k = 0 ; k < 4 ; k++ ) {
						v_data = spu_insert( jpc_fix_mul( spu_extract( v_data, k ), absstepsize ), v_data, k );
					}
				}
				( ( vector signed int* )a_out_data_ptr[sense] )[j] = v_data;
#endif
#else
#ifdef FLOAT_MODE
				flt_data = ( float )data;
				if( absstepsize != jpc_inttofix( 1 ) ) {
					flt_data = flt_data * f_absstepsize;
				}
				p_f_out_data_ptr[j] = flt_data;
#else
				data <<= JPC_FIX_FRACBITS;
				if( absstepsize != jpc_inttofix( 1 ) ) {
					data = jpc_fix_mul( data, absstepsize );
				}
				a_out_data_ptr[sense][j] = data;
#endif
#endif
			}
			else {
#ifdef SIMD_EN
				( ( vector signed int* )a_out_data_ptr[sense] )[j] = v_data;
#else
				a_out_data_ptr[sense][j] = data;
#endif
			}
		}

                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_data_ptr[sense], ( unsigned long long )ppu_nxt_w_addr, JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t ), sense );
                if( ( ( i + 1 ) % num_partitions ) == 0 ) {
                        ppu_nxt_w_addr = ppu_nxt_w_row_addr + stride * sizeof( jpc_fix_t );
                        ppu_nxt_w_row_addr = ppu_nxt_w_addr;
                }
                else {
                        ppu_nxt_w_addr += JPC_QUANT1_BUF_ITEMS * sizeof( jpc_fix_t );
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

