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
 * Quadrature Mirror-Image Filter Bank (QMFB) Library
 *
 * $Id: jpc_qmfb_analyze.c,v 1.6 2008/08/13 18:46:51 lrlemini Exp $
 */

/******************************************************************************\
*
\******************************************************************************/

#define WT_DOSCALE

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <spu_assert.h>
#include "jasper/jas_fix.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_dma.h"
/* s.kang end */

#include "jpc_qmfb.h"
#include "jpc_tsfb.h"
#include "jpc_math.h"

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
*
\******************************************************************************/

/* s.kang start */
#define QMFB_SPLIT_COLGRP_DEPTH 64
/* s.kang end */

/* s.kang start */
#if 0
int jpc_ft_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride);
int jpc_ns_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride);
#endif
/* s.kang end */

/* s.kang start */
#if 1
static void jpc_ft_fwdlift_row( jpc_fix_t* p_in, jpc_fix_t* p_out, int numcols, int parity );
#else
void jpc_ft_fwdlift_row(jpc_fix_t *a, int numcols, int parity);
#endif
/* s.kang end */
/* s.kang start */
#ifdef EN_INTERLEAVING
static void jpc_ft_process_colgrp(jpc_fix_t* p_a, jpc_fix_t* splitbuf, int numrows, int stride, int parity);
#else
static void jpc_ft_fwdlift_colgrp(jpc_fix_t *a, int numrows, int stride, int parity);
#endif
/* s.kang end */

/* s.kang start */
#if 1
static void jpc_ns_fwdlift_row( jpc_fix_t* p_in, jpc_fix_t* p_out, int numcols, int parity );
#else
void jpc_ns_fwdlift_row(jpc_fix_t *a, int numcols, int parity);
#endif
/* s.kang end */
/* s.kang start */
#ifdef EN_INTERLEAVING
static void jpc_ns_process_colgrp(jpc_fix_t* p_a, jpc_fix_t* splitbuf, int numrows, int stride, int parity);
#else
static void jpc_ns_fwdlift_colgrp(jpc_fix_t *a, int numrows, int stride, int parity);
#endif
/* s.kang end */

static void jpc_qmfb_split_row(jpc_fix_t *a, int numcols, int parity);
/* s.kang start */
/* splitbuf size can be too large to be allocated in SPU, allocate in PPU */
#if 1
#ifndef EN_INTERLEAVING
static void jpc_qmfb_split_colgrp( jpc_fix_t* a, jpc_fix_t* splitbuf, int numrows, int stride, int parity );
#endif
#else
void jpc_qmfb_split_colgrp(jpc_fix_t *a, int numrows, int stride, int parity);
#endif
/* s.kang end */

/* s.kang start */
#ifdef SIMD_EN
	static vector unsigned char a_v_si_shuffle_pattern[4] = {
		{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f },
		{ 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13 },
		{ 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 },
		{ 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b }
	};
#endif
/* s.kang end */

/* s.kang start */
#if 0
double jpc_ft_lpenergywts[32] = {
	1.2247448713915889,
	1.6583123951776999,
	2.3184046238739260,
	3.2691742076555053,
	4.6199296531440819,
	6.5323713152269596,
	9.2377452606141937,
	13.0639951297449581,
	18.4752262333915667,
	26.1278968190610392,
	36.9504194305524791,
	52.2557819580462777,
	73.9008347315741645,
	104.5115624560829133,
	147.8016689469569656,
	209.0231247296646018,
	295.6033378293900000,
	418.0462494347059419,
	591.2066756503630813,
	836.0924988714708661,
	/* approximations */
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661,
	836.0924988714708661
};

double jpc_ft_hpenergywts[32] = {
	0.8477912478906585,
	0.9601432184835760,
	1.2593401049756179,
	1.7444107171191079,
	2.4538713036750726,
	3.4656517695088755,
	4.8995276398597856,
	6.9283970402160842,
	9.7980274940131444,
	13.8564306871112652,
	19.5959265076535587,
	27.7128159494245487,
	39.1918369552045860,
	55.4256262207444053,
	78.3836719028959124,
	110.8512517317256822,
	156.7673435548526868,
	221.7025033739244293,
	313.5346870787551552,
	443.4050067351659550,
	/* approximations */
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550,
	443.4050067351659550
};

double jpc_ns_lpenergywts[32] = {
	1.4021081679297411,
	2.0303718560817923,
	2.9011625562785555,
	4.1152851751758002,
	5.8245108637728071,
	8.2387599345725171,
	11.6519546479210838,
	16.4785606470644375,
	23.3042776444606794,
	32.9572515613740435,
	46.6086013487782793,
	65.9145194076860861,
	93.2172084551803977,
	131.8290408510004283,
	186.4344176300625691,
	263.6580819564562148,
	372.8688353500955373,
	527.3161639447193920,
	745.7376707114038936,
	1054.6323278917823245,
	/* approximations follow */
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245,
	1054.6323278917823245
};

double jpc_ns_hpenergywts[32] = {
	1.4425227650161456,
	1.9669426082455688,
	2.8839248082788891,
	4.1475208393432981,
	5.8946497530677817,
	8.3471789178590949,
	11.8086046551047463,
	16.7012780415647804,
	23.6196657032246620,
	33.4034255108592362,
	47.2396388881632632,
	66.8069597416714061,
	94.4793162154500692,
	133.6139330736999113,
	188.9586372358249378,
	267.2278678461869390,
	377.9172750722391356,
	534.4557359047058753,
	755.8345502191498326,
	1068.9114718353569060,
	/* approximations follow */
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060,
	1068.9114718353569060
};

jpc_qmfb2d_t jpc_ft_qmfb2d = {
	jpc_ft_analyze,
	jpc_ft_synthesize,
	jpc_ft_lpenergywts,
	jpc_ft_hpenergywts
};

jpc_qmfb2d_t jpc_ns_qmfb2d = {
	jpc_ns_analyze,
	jpc_ns_synthesize,
	jpc_ns_lpenergywts,
	jpc_ns_hpenergywts
};
#endif
/* s.kang end */

/******************************************************************************\
* generic
\******************************************************************************/

#if 1
static void jpc_qmfb_split_row( jpc_fix_t* a, int numcols, int parity )
{
	int bufsize = JPC_CEILDIVPOW2( numcols, 1 );
	jpc_fix_t* p_buf;
	jpc_fix_t* srcptr;
	jpc_fix_t* dstptr;
	int n;
	int m;
	int hstartcol;
#ifdef SIMD_EN
	vector signed int* p_v_dst;
	vector signed int v_temp0;
	vector signed int v_temp1;
	vector unsigned char v_shuffle = { 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0a, 0x0b, 0x10, 0x11, 0x12, 0x13, 0x18, 0x19, 0x1a, 0x1b };
	int shuffle;
	int n0;
	int i;
#endif

	spu_assert( ( ( ( unsigned int )a % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_qmfb_split_row()] assertion failure\n" ) );

	/* Allocate memory for the split buffer from the heap. */
	if( !( p_buf = jas_malloc_align( bufsize * sizeof( jpc_fix_t ), DMA_MIN_OFFSET_BITS ) ) ) {
		/* We have no choice but to commit suicide. */
		spu_assert( ( 0 ), ( "[jpc_qmfb_analyze.c:jpc_qmfb_split_row()] assertion failure\n" ) );
	}

	if( numcols >= 2 ) {
		hstartcol = ( numcols + 1 - parity ) >> 1;
		m = ( parity ) ? hstartcol : ( numcols - hstartcol );

		/* Save the samples destined for the highpass channel. */
		n = m;
#ifdef SIMD_EN
		p_v_dst = ( vector signed int* )p_buf;
		srcptr = a + ( 1 - parity );
		shuffle = ( ( unsigned int )srcptr >> 2 ) & 0x3;

		for( i = 0 ; i < n - ( n % 4 ) ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector signed int* )( srcptr ) ), *( ( vector signed int* )( srcptr + 4 ) ), a_v_si_shuffle_pattern[shuffle] );
			v_temp1 = spu_shuffle( *( ( vector signed int* )( srcptr + 4 ) ), *( ( vector signed int* )( srcptr + 8 ) ), a_v_si_shuffle_pattern[shuffle] );
			*p_v_dst = spu_shuffle( v_temp0, v_temp1, v_shuffle );
			p_v_dst++;
			srcptr += 8;
		}

		dstptr = ( jpc_fix_t* )p_v_dst;
		for( i = 0 ; i < ( n % 4 ) ; i++ ) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
#else
		dstptr = p_buf;
		srcptr = &a[1 - parity];
		while( n-- > 0 ) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
#endif

		/* Copy the appropriate samples into the lowpass channel. */
		n = numcols - m - ( !parity );
#ifdef SIMD_EN
		dstptr = &a[1 - parity];
		srcptr = &a[2 - parity];
		if( !parity ) {
			n0 = ( n > 3 )? 3 : n;
			while( n0-- > 0 ) {
				*dstptr = *srcptr;
				++dstptr;
				srcptr += 2;
			}
			n -= n0;
		}

		p_v_dst = ( vector signed int* )dstptr;
		shuffle = ( ( unsigned int )srcptr >> 2 ) & 0x3;
		for( i = 0 ; i < n - ( n % 4 ) ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector signed int* )( srcptr ) ), *( ( vector signed int* )( srcptr + 4 ) ), a_v_si_shuffle_pattern[shuffle] );
			v_temp1 = spu_shuffle( *( ( vector signed int* )( srcptr + 4 ) ), *( ( vector signed int* )( srcptr + 8 ) ), a_v_si_shuffle_pattern[shuffle] );
			*p_v_dst = spu_shuffle( v_temp0, v_temp1, v_shuffle );
			p_v_dst++;
			srcptr += 8;
		}

		dstptr = ( jpc_fix_t* )p_v_dst;
		for( i = 0 ; i < ( n % 4 ) ; i++ ) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
#else
		dstptr = &a[1 - parity];
		srcptr = &a[2 - parity];
		while( n-- > 0 ) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
#endif

		/* Copy the saved samples into the highpass channel. */
		dstptr = &a[hstartcol];
		srcptr = p_buf;
		n = m;
		memcpy( dstptr, srcptr, n * sizeof( jpc_fix_t ) );
	}

	jas_free_align( p_buf );
}
#else
void jpc_qmfb_split_row(jpc_fix_t *a, int numcols, int parity)
{

	int bufsize = JPC_CEILDIVPOW2(numcols, 1);
#if !defined(HAVE_VLA)
	jpc_fix_t splitbuf[QMFB_SPLITBUFSIZE];
#else
	jpc_fix_t splitbuf[bufsize];
#endif
	jpc_fix_t *buf = splitbuf;
	register jpc_fix_t *srcptr;
	register jpc_fix_t *dstptr;
	register int n;
	register int m;
	int hstartcol;

#if !defined(HAVE_VLA)
	/* Get a buffer. */
	if (bufsize > QMFB_SPLITBUFSIZE) {
		if (!(buf = jas_malloc(bufsize * sizeof(jpc_fix_t)))) {
			/* We have no choice but to commit suicide in this case. */
			abort();
		}
	}
#endif

	if (numcols >= 2) {
		hstartcol = (numcols + 1 - parity) >> 1;
		m = (parity) ? hstartcol : (numcols - hstartcol);
		/* Save the samples destined for the highpass channel. */
		n = m;
		dstptr = buf;
		srcptr = &a[1 - parity];
		while (n-- > 0) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
		/* Copy the appropriate samples into the lowpass channel. */
		dstptr = &a[1 - parity];
		srcptr = &a[2 - parity];
		n = numcols - m - (!parity);
		while (n-- > 0) {
			*dstptr = *srcptr;
			++dstptr;
			srcptr += 2;
		}
		/* Copy the saved samples into the highpass channel. */
		dstptr = &a[hstartcol];
		srcptr = buf;
		n = m;
		while (n-- > 0) {
			*dstptr = *srcptr;
			++dstptr;
			++srcptr;
		}
	}

#if !defined(HAVE_VLA)
	/* If the split buffer was allocated on the heap, free this memory. */
	if (buf != splitbuf) {
		jas_free(buf);
	}
#endif
}
#endif

/* s.kang start */
#if 1
#ifndef EN_INTERLEAVING
static void jpc_qmfb_split_colgrp( jpc_fix_t* a, jpc_fix_t* splitbuf, int numrows, int stride, int parity )
{
	void* p_buf;
	jpc_fix_t* a_p_buf[QMFB_SPLIT_COLGRP_DEPTH];
	unsigned int ppu_saddr;
	unsigned int ppu_daddr;
	int hstartcol;
	int n;
	int m;
	int i, j;

	spu_assert( ( ( ( unsigned int )a % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_qmfb_split_colgrp()] assertion failure\n" ) );

	hstartcol = ( numrows + 1 - parity ) >> 1;
	m = ( parity ) ? hstartcol : ( numrows - hstartcol );

	p_buf = jas_malloc_align( QMFB_SPLIT_COLGRP_DEPTH * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( p_buf != NULL ), ( "[jpc_qmfb_analyze.c:jpc_qmfb_split_colgrp()] assertion failure\n" ) );
	for( i = 0 ; i < QMFB_SPLIT_COLGRP_DEPTH ; i++ ) {
		a_p_buf[i] = p_buf + i * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}

	if( numrows >= 2 ) {
		/* Save the samples destined for the highpass channel. */

		n = m;
		ppu_saddr = ( unsigned int )a + ( 1 - parity ) * stride * sizeof( jpc_fix_t );
		ppu_daddr = ( unsigned int )splitbuf;

		for( i = 0 ; i < n - ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i += QMFB_SPLIT_COLGRP_DEPTH ) {
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[j], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_saddr += ( stride << 1 ) * sizeof( jpc_fix_t );
			}
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[j], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_daddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		}

		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_saddr += ( stride << 1 ) * sizeof( jpc_fix_t );
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[i], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_daddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		/* Copy the appropriate samples into the lowpass channel. */

		n = numrows - m - (!parity);
		ppu_saddr = ( unsigned int )a + ( 2 - parity ) * stride * sizeof( jpc_fix_t );
		ppu_daddr = ( unsigned int )a + ( 1 - parity ) * stride * sizeof( jpc_fix_t );

		for( i = 0 ; i < n - ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i += QMFB_SPLIT_COLGRP_DEPTH ) {
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[j], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_saddr += ( stride << 1 ) * sizeof( jpc_fix_t );
			}
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[j], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_daddr += stride * sizeof( jpc_fix_t );
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		}

		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_saddr += ( stride << 1 ) * sizeof( jpc_fix_t );
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[i], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_daddr += stride * sizeof( jpc_fix_t );
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		/* Copy the saved samples into the highpass channel. */

		n = m;
		ppu_saddr = ( unsigned int )splitbuf;
		ppu_daddr = ( unsigned int )a + hstartcol * stride * sizeof( jpc_fix_t );

		for( i = 0 ; i < n - ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i += QMFB_SPLIT_COLGRP_DEPTH ) {
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[j], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			}
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[j], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_daddr += stride * sizeof( jpc_fix_t );
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		}

		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_saddr, ( void* )a_p_buf[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[i], ( unsigned long long )ppu_daddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_daddr += stride * sizeof( jpc_fix_t );
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}

	jas_free_align( p_buf );
}
#endif
#else
void jpc_qmfb_split_colgrp(jpc_fix_t *a, int numrows, int stride,
  int parity)
{

	int bufsize = JPC_CEILDIVPOW2(numrows, 1);
#if !defined(HAVE_VLA)
	jpc_fix_t splitbuf[QMFB_SPLITBUFSIZE * JPC_QMFB_COLGRPSIZE];
#else
	jpc_fix_t splitbuf[bufsize * JPC_QMFB_COLGRPSIZE];
#endif
	jpc_fix_t *buf = splitbuf;
	jpc_fix_t *srcptr;
	jpc_fix_t *dstptr;
	register jpc_fix_t *srcptr2;
	register jpc_fix_t *dstptr2;
	register int n;
	register int i;
	int m;
	int hstartcol;

#if !defined(HAVE_VLA)
	/* Get a buffer. */
	if (bufsize > QMFB_SPLITBUFSIZE) {
		if (!(buf = jas_malloc(bufsize * sizeof(jpc_fix_t)))) {
			/* We have no choice but to commit suicide in this case. */
			abort();
		}
	}
#endif

	if (numrows >= 2) {
		hstartcol = (numrows + 1 - parity) >> 1;
		m = (parity) ? hstartcol : (numrows - hstartcol);
		/* Save the samples destined for the highpass channel. */
		n = m;
		dstptr = buf;
		srcptr = &a[(1 - parity) * stride];
		while (n-- > 0) {
			dstptr2 = dstptr;
			srcptr2 = srcptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				*dstptr2 = *srcptr2;
				++dstptr2;
				++srcptr2;
			}
			dstptr += JPC_QMFB_COLGRPSIZE;
			srcptr += stride << 1;
		}
		/* Copy the appropriate samples into the lowpass channel. */
		dstptr = &a[(1 - parity) * stride];
		srcptr = &a[(2 - parity) * stride];
		n = numrows - m - (!parity);
		while (n-- > 0) {
			dstptr2 = dstptr;
			srcptr2 = srcptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				*dstptr2 = *srcptr2;
				++dstptr2;
				++srcptr2;
			}
			dstptr += stride;
			srcptr += stride << 1;
		}
		/* Copy the saved samples into the highpass channel. */
		dstptr = &a[hstartcol * stride];
		srcptr = buf;
		n = m;
		while (n-- > 0) {
			dstptr2 = dstptr;
			srcptr2 = srcptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				*dstptr2 = *srcptr2;
				++dstptr2;
				++srcptr2;
			}
			dstptr += stride;
			srcptr += JPC_QMFB_COLGRPSIZE;
		}
	}

#if !defined(HAVE_VLA)
	/* If the split buffer was allocated on the heap, free this memory. */
	if (buf != splitbuf) {
		jas_free(buf);
	}
#endif

}
#endif
/* s.kang end */

/******************************************************************************\
* 5/3 transform
\******************************************************************************/

/* s.kang start */
#if 1
static void jpc_ft_fwdlift_row( jpc_fix_t* p_in, jpc_fix_t* p_out, int numcols, int parity )
{
#ifdef SIMD_EN
	vector signed int v_0;
	vector signed int v_1;
	vector signed int* p_v_in;
	vector signed int* p_v_out;
	int shuffle_0;
	int shuffle_1;
	int n_head;
	int n_body;
	int n_tail;
	int i;
#endif
	jpc_fix_t* in_lptr;
	jpc_fix_t* out_lptr;
	jpc_fix_t* in_hptr;
	jpc_fix_t* out_hptr;
	int n;
	int llen;

	spu_assert( ( ( ( unsigned int )p_in % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_row()] assertion failure\n" ) );
	spu_assert( ( ( ( unsigned int )p_out % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_row()] assertion failure\n" ) );

#ifdef SIMD_EN
	spu_assert( ( sizeof( jpc_fix_t ) == 4 ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_row()] assertion failure\n" ) );/* SIMDization assumes sizeof( jpc_fix_t ) == 4 */
#endif

	llen = ( numcols + 1 - parity ) >> 1;

	if( numcols > 1 ) {
		/* Apply the first lifting step. */
		in_lptr = p_in;
		in_hptr = p_in + llen;
		out_hptr = p_out + llen;
		if( parity ) {
			out_hptr[0] = in_hptr[0] - in_lptr[0];
			++in_hptr;
			++out_hptr;
		}
		n = numcols - llen - parity - ( parity == ( numcols & 1 ) );
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )in_hptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			out_hptr[0] = in_hptr[0] - ( ( in_lptr[0] + in_lptr[1] ) >> 1 );
			++in_lptr;
			++in_hptr;
			++out_hptr;
		}
		p_v_in = ( vector signed int* )in_hptr;
		p_v_out = ( vector signed int* )out_hptr;
		shuffle_0 = ( ( unsigned int )in_lptr >> 2 ) & 0x3;
		shuffle_1 = ( ( unsigned int )( in_lptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_0 = spu_shuffle( *( ( vector signed int* )in_lptr ), *( ( vector signed int* )( in_lptr + 4 ) ), a_v_si_shuffle_pattern[shuffle_0] );
			v_1 = spu_shuffle( *( ( vector signed int* )( in_lptr + 1 ) ), *( ( vector signed int* )( in_lptr + 5 ) ), a_v_si_shuffle_pattern[shuffle_1] );
			*p_v_out = spu_sub( *p_v_in, spu_rlmaska( spu_add( v_0, v_1 ), -1 ) );
			p_v_in++;
			p_v_out++;
			in_lptr += 4;
		}
		in_hptr = ( jpc_fix_t* )p_v_in;
		out_hptr = ( jpc_fix_t* )p_v_out;
		for( i = 0 ; i < n_tail ; i++ ) {
			out_hptr[0] = in_hptr[0] - ( ( in_lptr[0] + in_lptr[1] ) >> 1 );
			++in_lptr;
			++in_hptr;
			++out_hptr;
		}
#else
		while( n-- > 0 ) {
			out_hptr[0] = in_hptr[0] - ( ( in_lptr[0] + in_lptr[1] ) >> 1 );
			++in_lptr;
			++in_hptr;
			++out_hptr;
		}
#endif
		if( parity == ( numcols & 1 ) ) {
			out_hptr[0] = in_hptr[0] - in_lptr[0];
		}

		/* Apply the second lifting step. */
		in_lptr = p_in;
		out_lptr = p_out;
		in_hptr = p_out + llen;
		if( !parity ) {
			out_lptr[0] =  in_lptr[0] + ( ( in_hptr[0] + 1 ) >> 1 );
			++in_lptr;
			++out_lptr;
		}
		n = llen - ( !parity ) - ( parity != ( numcols & 1 ) );
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )in_lptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			out_lptr[0] = in_lptr[0] + ( ( in_hptr[0] + in_hptr[1] + 2 ) >> 2 );
			++in_lptr;
			++out_lptr;
			++in_hptr;
		}
		p_v_in = ( vector signed int* )in_lptr;
		p_v_out = ( vector signed int* )out_lptr;
		shuffle_0 = ( ( unsigned int )in_hptr >> 2 ) & 0x3;
		shuffle_1 = ( ( unsigned int )( in_hptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_0 = spu_shuffle( *( ( vector signed int* )in_hptr ), *( ( vector signed int* )( in_hptr + 4 ) ), a_v_si_shuffle_pattern[shuffle_0] );
			v_1 = spu_shuffle( *( ( vector signed int* )( in_hptr + 1 ) ), *( ( vector signed int* )( in_hptr + 5 ) ), a_v_si_shuffle_pattern[shuffle_1] );
			*p_v_out = spu_add( *p_v_in, spu_rlmaska( spu_add( spu_add( v_0, v_1 ), 2 ), -2 ) );
			p_v_in++;
			p_v_out++;
			in_hptr += 4;
		}
		in_lptr = ( jpc_fix_t* )p_v_in;
		out_lptr = ( jpc_fix_t* )p_v_out;
		for( i = 0 ; i < n_tail ; i++ ) {
			out_lptr[0] = in_lptr[0] + ( ( in_hptr[0] + in_hptr[1] + 2 ) >> 2 );
			++in_lptr;
			++out_lptr;
			++in_hptr;
		}
#else
		while( n-- > 0 ) {
			out_lptr[0] = in_lptr[0] + ( ( in_hptr[0] + in_hptr[1] + 2 ) >> 2 );
			++in_lptr;
			++out_lptr;
			++in_hptr;
		}
#endif
		if( parity != ( numcols & 1 ) ) {
			out_lptr[0] = in_lptr[0] + ( ( in_hptr[0] + 1 ) >> 1 );
		}
	} else {
		if( parity ) {
			in_lptr = p_in;
			out_lptr = p_out;
			out_lptr[0] = in_lptr[0] << 1;
		}
	}
}
#else
void jpc_ft_fwdlift_row(jpc_fix_t *a, int numcols, int parity)
{

	register jpc_fix_t *lptr;
	register jpc_fix_t *hptr;
	register int n;
	int llen;

	llen = (numcols + 1 - parity) >> 1;

	if (numcols > 1) {

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (parity) {
			hptr[0] -= lptr[0];
			++hptr;
		}
		n = numcols - llen - parity - (parity == (numcols & 1));
		while (n-- > 0) {
			hptr[0] -= (lptr[0] + lptr[1]) >> 1;
			++hptr;
			++lptr;
		}
		if (parity == (numcols & 1)) {
			hptr[0] -= lptr[0];
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (!parity) {
			lptr[0] += (hptr[0] + 1) >> 1;
			++lptr;
		}
		n = llen - (!parity) - (parity != (numcols & 1));
		while (n-- > 0) {
			lptr[0] += (hptr[0] + hptr[1] + 2) >> 2;
			++lptr;
			++hptr;
		}
		if (parity != (numcols & 1)) {
			lptr[0] += (hptr[0] + 1) >> 1;
		}

	} else {

		if (parity) {
			lptr = &a[0];
			lptr[0] <<= 1;
		}

	}

}
#endif

/* s.kang start */
#define MERGE2 1
#if 1
#ifdef EN_INTERLEAVING
static void jpc_ft_process_colgrp( jpc_fix_t* p_a, jpc_fix_t* splitbuf, int numrows, int stride, int parity ) {
	void * p_buf;
	jpc_fix_t* a_p_buf[QMFB_SPLIT_COLGRP_DEPTH];
	jpc_fix_t* a_in_lptr[DWT_BUF_DEPTH];
	jpc_fix_t* a_out_lptr[DWT_BUF_DEPTH];
	jpc_fix_t* a_in_hptr[DWT_BUF_DEPTH];
	jpc_fix_t* a_out_hptr[DWT_BUF_DEPTH];
	vector signed int* p_v_in_low0;
	vector signed int* p_v_in_low1;
	vector signed int* p_v_in_high0;
	vector signed int* p_v_in_high1;
	vector signed int* p_v_out_low0;
	vector signed int* p_v_out_high0;
	unsigned int ppu_nxt_r_laddr;
	unsigned int ppu_nxt_w_laddr;
	unsigned int ppu_nxt_r_haddr;
	unsigned int ppu_nxt_w_haddr;
#ifdef EN_OVERLAP
	unsigned int ppu_nxt_r_saddr;
	unsigned int ppu_nxt_w_saddr;
#endif
	int sense;
	int hstart;
	int n;
#ifdef EN_OVERLAP/* 1st path: interleaved computation loop, 2nd path: copy from splitbuf to original array */
	int n_lh_r;/* total # of read for low and high elements in 1st path */
	int n_h_w;/* total # of write for high elements in 1st path */
	int n_buf_tot;/* total # of scheduled elements read in 2nd path */
	int n_buf_r_scheduled;/* total # of scheduled (and not confirmed) read in 2nd path */
	int n_buf_r_confirmed;/* total # of confirmed (and not write scheduled) read in 2nd paht */
	int n_buf_w_scheduled;/* total # of scheduled (and not confirmed) write in 2nd path */
	int buf_r_idx;
	int buf_w_idx;
	int n_h_thresh;
#endif
	int i;
	int j;

	spu_assert( ( parity == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] parity == 1 case unimplemented\n" ) );
	spu_assert( ( DWT_BUF_DEPTH >= 5 ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] DWT_BUF_DEPTH needs to be larger than 4\n" ) );
	spu_assert( ( ( ( unsigned int )p_a % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( ( unsigned int )splitbuf % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] assertion failure\n" ) );

	hstart = ( numrows + 1 ) >> 1;
#ifdef EN_OVERLAP
	n_lh_r = 0;
	n_h_w = 0;
	n_buf_tot = 0;
	n_buf_r_scheduled = 0;
	n_buf_r_confirmed = 0;
	n_buf_w_scheduled = 0;
	buf_r_idx = 0;
	buf_w_idx = 0;
#endif

	for( i = 0 ; i < DWT_BUF_DEPTH ; i++ ) {
		a_in_lptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_in_lptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] jas_malloc_align failure\n" ) );
		a_out_lptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_out_lptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] jas_malloc_align failure\n" ) );
		a_in_hptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_in_hptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] jas_malloc_align failure\n" ) );
		a_out_hptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_out_hptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] jas_malloc_align failure\n" ) );
	}

#ifdef EN_OVERLAP
	p_buf = jas_malloc_align( QMFB_SPLIT_COLGRP_DEPTH * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( p_buf != NULL ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] assertion failure\n" ) );
	for( i = 0 ; i < QMFB_SPLIT_COLGRP_DEPTH ; i++ ) {
		a_p_buf[i] = p_buf + i * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}
#endif

	if( numrows >= 3 ) {
		ppu_nxt_r_laddr = ( unsigned int )p_a;
		ppu_nxt_w_laddr = ( unsigned int )p_a;
		ppu_nxt_r_haddr = ( unsigned int )( p_a + stride );
		ppu_nxt_w_haddr = ( unsigned int )splitbuf;
#ifdef EN_OVERLAP
		ppu_nxt_r_saddr = ( unsigned int )splitbuf;
		ppu_nxt_w_saddr = ( unsigned int )( p_a + hstart * stride );
#endif

		for( i = 0 ; ( i < ( DWT_BUF_DEPTH - 1 ) ) && ( i < ( ( numrows + 1 ) >> 1 ) ) ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_laddr, ( void* )a_in_lptr[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), i );
			ppu_nxt_r_laddr += 2 * stride * sizeof( jpc_fix_t );
			if( i < ( numrows >> 1 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_in_hptr[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), i );
				ppu_nxt_r_haddr += 2 * stride * sizeof( jpc_fix_t );
			}
		}

		/* head */

		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( 0 );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( 1 );
#ifdef EN_OVERLAP
		n_lh_r += 4;
#endif
		p_v_in_low0 = ( vector signed int* )a_in_lptr[0];
		p_v_in_low1 = ( vector signed int* )a_in_lptr[1];
		p_v_in_high0 = ( vector signed int* )a_in_hptr[0];
		p_v_out_low0 = ( vector signed int* )a_out_lptr[0];
		p_v_out_high0 = ( vector signed int* )a_out_hptr[0];
		for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
			/* 1st lifting */
			*p_v_in_high0 = spu_sub( *p_v_in_high0, spu_rlmaska( spu_add( *p_v_in_low0, *p_v_in_low1 ), -1 ) );
			/* 2nd lifting */
			*p_v_in_low0 = spu_add( *p_v_in_low0, spu_rlmaska( spu_add( *p_v_in_high0, 1 ), -1 ) );
			*p_v_out_low0 = *p_v_in_low0;
			*p_v_out_high0 = *p_v_in_high0;
			p_v_in_low0++;
			p_v_in_low1++;
			p_v_in_high0++;
			p_v_out_low0++;
			p_v_out_high0++;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[0], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), 0 );
		ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[0], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), 0 );
		ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );

		/* body */

		sense = 1;
		n = ( ( numrows + 1 ) >> 1 ) - 2;
#ifdef EN_OVERLAP
		n_h_thresh = n - DWT_BUF_DEPTH;
#endif

		while( n-- ) {
#ifdef EN_OVERLAP
			if( ( n_lh_r >= hstart + 2 ) && ( n_h_w - n_buf_tot >= 2 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
				buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
				buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

				n_buf_tot += 2;
				n_buf_r_scheduled += 2;
			}
#endif
			if( n >= ( DWT_BUF_DEPTH - 3 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_laddr, ( void* )a_in_lptr[( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), ( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH );
				ppu_nxt_r_laddr += 2 * stride * sizeof( jpc_fix_t );
				if( ( n != ( DWT_BUF_DEPTH - 3 ) ) || ( ( numrows & 0x1 ) == 0 ) ) {
					JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_in_hptr[( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), ( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH );
					ppu_nxt_r_haddr += 2 * stride * sizeof( jpc_fix_t );
				}
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
#ifdef EN_OVERLAP
			if( n <= n_h_thresh ) {
				n_h_w++;
			}
#endif
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + 1 ) % DWT_BUF_DEPTH );
#ifdef EN_OVERLAP
			n_lh_r += 2;
#endif

			p_v_in_low0 = ( vector signed int* )a_in_lptr[sense];
			p_v_in_low1 = ( vector signed int* )a_in_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_in_high0 = ( vector signed int* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_in_high1 = ( vector signed int* )a_in_hptr[sense];
			p_v_out_low0 = ( vector signed int* )a_out_lptr[sense];
			p_v_out_high0 = ( vector signed int* )a_out_hptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {	
				/* 1st lifting */
				*p_v_in_high1 = spu_sub( *p_v_in_high1, spu_rlmaska( spu_add( *p_v_in_low0, *p_v_in_low1 ), -1 ) );
				/* 2nd lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_rlmaska( spu_add( spu_add( *p_v_in_high0, *p_v_in_high1 ), 2 ), -2 ) );
				*p_v_out_low0 = *p_v_in_low0;
				*p_v_out_high0 = *p_v_in_high1;
				p_v_in_low0++;
				p_v_in_low1++;
				p_v_in_high0++;
				p_v_in_high1++;
				p_v_out_low0++;
				p_v_out_high0++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) % DWT_BUF_DEPTH;
#ifdef EN_OVERLAP
			if( n_buf_r_scheduled >= QMFB_SPLIT_COLGRP_DEPTH / 4 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
				n_buf_r_confirmed += n_buf_r_scheduled;
				n_buf_r_scheduled = 0;
			}
			if( n_buf_w_scheduled >= QMFB_SPLIT_COLGRP_DEPTH / 4 ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
				n_buf_w_scheduled = 0;
			}
			if( n_buf_r_confirmed >= 2 ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
				buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
				buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

				n_buf_r_confirmed -= 2;
				n_buf_w_scheduled += 2;
			}
#endif
		}

		/* tail */

		if( ( numrows & 0x1 ) == 0 ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
			p_v_in_low0 = ( vector signed int* )a_in_lptr[sense];
			p_v_in_high0 = ( vector signed int* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_in_high1 = ( vector signed int* )a_in_hptr[sense];
			p_v_out_low0 = ( vector signed int* )a_out_lptr[sense];
			p_v_out_high0 = ( vector signed int* )a_out_hptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				/* 1st lifting */
				*p_v_in_high1 = spu_sub( *p_v_in_high1, *p_v_in_low0 );
				/* 2nd lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_rlmaska( spu_add( spu_add( *p_v_in_high0, *p_v_in_high1 ), 2 ), -2 ) );
				*p_v_out_low0 = *p_v_in_low0;
				*p_v_out_high0 = *p_v_in_high1;
				p_v_in_low0++;
				p_v_in_high0++;
				p_v_in_high1++;
				p_v_out_low0++;
				p_v_out_high0++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
		}
		else {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
			p_v_in_low0 = ( vector signed int* )a_in_lptr[sense];
			p_v_in_high0 = ( vector signed int* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_out_low0 = ( vector signed int* )a_out_lptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				/* 1st lifting */
				/* 2nd lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_rlmaska( spu_add( *p_v_in_high0, 1 ), -1 ) );
				*p_v_out_low0 = *p_v_in_low0;
				p_v_in_low0++;
				p_v_in_high0++;
				p_v_out_low0++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
		}

		for( i = 0 ; i < DWT_BUF_DEPTH ; i++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( i );
		}
	}
	else {
		spu_assert( ( 0 ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] numrows < 3 case need to be implemented\n" ) );
	}

	/* Copy the saved samples into the highpass channel. */

#ifdef EN_OVERLAP
	if( n_buf_r_scheduled > 0 ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		n_buf_r_confirmed += n_buf_r_scheduled;
		n_buf_r_scheduled = 0;
	}

	n = n_buf_r_confirmed;
	while( n-- ) {
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
		buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
	}
	n_buf_w_scheduled += n_buf_r_confirmed;
	n_buf_r_confirmed = 0;

	if( n_buf_w_scheduled > 0 ) {
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		n_buf_w_scheduled = 0;
	}

	n = numrows - hstart - n_buf_tot;
	if( n != 0 ) {
		spu_assert( ( n <= QMFB_SPLIT_COLGRP_DEPTH ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] n too large\n" ) );
		for( j = 0 ; j < n ; j++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( j = 0 ; j < n ; j++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
			buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}
	
	jas_free_align( p_buf );
#else
	p_buf = jas_malloc_align( QMFB_SPLIT_COLGRP_DEPTH * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( p_buf != NULL ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] assertion failure\n" ) );
	for( i = 0 ; i < QMFB_SPLIT_COLGRP_DEPTH ; i++ ) {
		a_p_buf[i] = p_buf + i * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}

	n = numrows - hstart;
	ppu_nxt_r_haddr = ( unsigned int )splitbuf;
	ppu_nxt_w_haddr = ( unsigned int )( p_a + hstart * stride );

	for( i = 0 ; i < n - ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i += QMFB_SPLIT_COLGRP_DEPTH ) {
		for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_p_buf[j], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_r_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[j], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_w_haddr += stride * sizeof( jpc_fix_t );
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}

	for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_p_buf[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		ppu_nxt_r_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}
	JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[i], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		ppu_nxt_w_haddr += stride * sizeof( jpc_fix_t );
	}
	JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	jas_free_align( p_buf );
#endif

	for( i = 0 ; i < DWT_BUF_DEPTH ;i++ ) {
		jas_free_align( a_in_lptr[i] );
		jas_free_align( a_out_lptr[i] );
		jas_free_align( a_in_hptr[i] );
		jas_free_align( a_out_hptr[i] );
	}
}
#else
static void jpc_ft_fwdlift_colgrp( jpc_fix_t* a, int numrows, int stride, int parity )
{
	jpc_fix_t* a_in_lptr[2];
	jpc_fix_t* a_out_lptr[2];
	jpc_fix_t* a_in_hptr[2];
	jpc_fix_t* a_out_hptr[2];
	jpc_fix_t* a_in_sptr[2];
	int sense;
	int dma_wr_scheduled;
	unsigned int ppu_laddr;
	unsigned int ppu_haddr;
#ifdef SIMD_EN
	vector signed int* p_v_in_low;
	vector signed int* p_v_out_low;
	vector signed int* p_v_in_high;
	vector signed int* p_v_out_high;
	vector signed int* p_v_in_stride;
#else
	jpc_fix_t* in_lptr2;
	jpc_fix_t* out_lptr2;
	jpc_fix_t* in_hptr2;
	jpc_fix_t* out_hptr2;
	jpc_fix_t* in_sptr2;
#endif
	register int n;
	register int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	a_in_lptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_lptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_lptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_lptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_hptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_hptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_hptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_hptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_sptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_sptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( ( a_in_lptr[0] != NULL ) && ( a_in_lptr[1] != NULL ) && ( a_out_lptr[0] != NULL ) && ( a_out_lptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( a_in_hptr[0] != NULL ) && ( a_in_hptr[1] != NULL ) && ( a_out_hptr[0] != NULL ) && ( a_out_hptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( a_in_sptr[0] != NULL ) && ( a_in_sptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ft_fwdlift_colgrp()] assertion failure\n" ) );

	if (numrows > 1) {
		/* Apply the first lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
		if (parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[0];
			p_v_in_high = ( vector signed int* )a_in_hptr[0];
			p_v_out_high = ( vector signed int* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_sub( *p_v_in_high, *p_v_in_low );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = in_hptr2[0] - in_lptr2[0];
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_haddr += stride * sizeof( jpc_fix_t );
		}

		n = numrows - llen - parity - (parity == (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[sense];
			p_v_in_high = ( vector signed int* )a_in_hptr[sense];
			p_v_out_high = ( vector signed int* )a_out_hptr[sense];
			p_v_in_stride = ( vector signed int* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_sub( *p_v_in_high, spu_rlmaska( spu_add( *p_v_in_low, *p_v_in_stride ), -1 ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
				p_v_in_stride++;
			}
#else
			in_lptr2 = a_in_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			out_hptr2 = a_out_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = in_hptr2[0] - ( ( in_lptr2[0] + in_sptr2[0] ) >> 1 );
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity == (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[0];
			p_v_in_high = ( vector signed int* )a_in_hptr[0];
			p_v_out_high = ( vector signed int* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_sub( *p_v_in_high, *p_v_in_low );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = in_hptr2[0] - in_lptr2[0];
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

		/* Apply the second lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
		if (!parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[0];
			p_v_out_low = ( vector signed int* )a_out_lptr[0];
			p_v_in_high = ( vector signed int* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_rlmaska( spu_add( *p_v_in_high, 1 ), -1 ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = in_lptr2[0] + ( ( in_hptr2[0] + 1 ) >> 1 );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_laddr += stride * sizeof( jpc_fix_t );
		}

		n = llen - (!parity) - (parity != (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[sense];
			p_v_out_low = ( vector signed int* )a_out_lptr[sense];
			p_v_in_high = ( vector signed int* )a_in_hptr[sense];
			p_v_in_stride = ( vector signed int* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_rlmaska( spu_add( spu_add( *p_v_in_high, *p_v_in_stride ), 2 ), -2 ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
				p_v_in_stride++;
			}
#else
			in_lptr2 = a_in_lptr[sense];
			out_lptr2 = a_out_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = in_lptr2[0] + ( ( in_hptr2[0] + in_sptr2[0] + 2) >> 2 );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity != (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[0];
			p_v_out_low = ( vector signed int* )a_out_lptr[0];
			p_v_in_high = ( vector signed int* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_rlmaska( spu_add( *p_v_in_high, 1 ), -1 ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = in_lptr2[0] + ( ( in_hptr2[0] + 1 ) >> 1 );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}
	} else {
		if (parity) {
			ppu_laddr = ( unsigned int )a;
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef SIMD_EN
			p_v_in_low = ( vector signed int* )a_in_lptr[0];
			p_v_out_low = ( vector signed int* )a_out_lptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_sl( *p_v_in_low, 1 );
				p_v_in_low++;
				p_v_out_low++;
			}
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = in_lptr2[0] << 1;
				++in_lptr2;
				++out_lptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

	}

	jas_free_align( a_in_lptr[0] );
	jas_free_align( a_in_lptr[1] );
	jas_free_align( a_out_lptr[0] );
	jas_free_align( a_out_lptr[1] );
	jas_free_align( a_in_hptr[0] );
	jas_free_align( a_in_hptr[1] );
	jas_free_align( a_out_hptr[0] );
	jas_free_align( a_out_hptr[1] );
	jas_free_align( a_in_sptr[0] );
	jas_free_align( a_in_sptr[1] );
}
#endif
#else
void jpc_ft_fwdlift_colgrp(jpc_fix_t *a, int numrows, int stride, int parity)
{

	jpc_fix_t *lptr;
	jpc_fix_t *hptr;
	register jpc_fix_t *lptr2;
	register jpc_fix_t *hptr2;
	register int n;
	register int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	if (numrows > 1) {

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				hptr2[0] -= lptr2[0];
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				hptr2[0] -= (lptr2[0] + lptr2[stride]) >> 1;
				++lptr2;
				++hptr2;
			}
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				hptr2[0] -= lptr2[0];
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] += (hptr2[0] + 1) >> 1;
				++lptr2;
				++hptr2;
			}
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] += (hptr2[0] + hptr2[stride] + 2) >> 2;
				++lptr2;
				++hptr2;
			}
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] += (hptr2[0] + 1) >> 1;
				++lptr2;
				++hptr2;
			}
		}

	} else {

		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] <<= 1;
				++lptr2;
			}
		}

	}

}
#endif
/* s.kang end */

/* s.kang start */
#if 1
int jpc_ft_row_analyze( unsigned int startaddr, int numrows, int numcols, int stride, int colparity ) {
	jpc_fix_t* a_in_ptr[2];
	jpc_fix_t* a_out_ptr[2];
	int en_buf;/* enable double buffering */
	int sense;
	int dma_wr_scheduled;
	int dma_size;
	int i;

	a_in_ptr[0] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_in_ptr[1] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_out_ptr[0] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_out_ptr[1] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	if( a_in_ptr[0] == NULL ) {
		jas_eprintf( "[jpc_qmfb_analyze.c:jpc_ft_row_analyze()] jas_malloc_align failure\n" );
		return -1;
	}
	else if( ( a_in_ptr[1] != NULL ) && ( a_out_ptr[0] != NULL ) && ( a_out_ptr[1] != NULL ) ) {
		en_buf = 1;
	}
	else {
		jas_eprintf( "double buffering disabled for row analyze\n" );
		en_buf = 0;
		if( a_in_ptr[1] ) {
			jas_free_align( a_in_ptr[1] );
		}
		if( a_out_ptr[0] ) {
			jas_free_align( a_out_ptr[0] );
		}
		if( a_out_ptr[1] ) {
			jas_free_align( a_out_ptr[1] );
		}
	}

	dma_size = ( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE - 1 ) & CACHE_LINE_OFFSET_MASK;

	if( en_buf ) {
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )startaddr, ( void* )a_in_ptr[sense], dma_size );
		for( i = 0; i < numrows ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( i != ( numrows - 1 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )startaddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_ptr[( sense + 1 ) & 0x1], dma_size );
			}
			jpc_qmfb_split_row( a_in_ptr[sense], numcols, colparity );
			jpc_ft_fwdlift_row( a_in_ptr[sense], a_out_ptr[sense], numcols, colparity );
			memcpy( a_out_ptr[sense] + numcols, a_in_ptr[sense] + numcols, dma_size - numcols * sizeof( jpc_fix_t ) );
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_ptr[sense], ( unsigned long long )startaddr, dma_size );
			dma_wr_scheduled = 1;
			startaddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}
	else {
		for( i = 0; i < numrows ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )startaddr, ( void* )a_in_ptr[0], dma_size );
			jpc_qmfb_split_row( a_in_ptr[0], numcols, colparity );
			jpc_ft_fwdlift_row( a_in_ptr[0], a_in_ptr[0], numcols, colparity );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_in_ptr[0], ( unsigned long long )startaddr, dma_size );
			startaddr += stride * sizeof( jpc_fix_t );
		}
	}

	if( en_buf ) {
		jas_free_align( a_in_ptr[0] );
		jas_free_align( a_in_ptr[1] );
		jas_free_align( a_out_ptr[0] );
		jas_free_align( a_out_ptr[1] );
	}
	else {
		jas_free_align( a_in_ptr[0] );
	}

	return 0;
}

int jpc_ft_colgrp_analyze( unsigned int startaddr, unsigned int splitbufaddr, int numrows, int numcolgrps, int stride, int rowparity ) {
	int i;

	for( i = 0 ; i < numcolgrps ; i++ ) {
#ifdef EN_INTERLEAVING
		jpc_ft_process_colgrp( ( jpc_fix_t* )startaddr, ( jpc_fix_t* )splitbufaddr, numrows, stride, rowparity );
#else
		jpc_qmfb_split_colgrp( ( jpc_fix_t* )startaddr, ( jpc_fix_t* )splitbufaddr, numrows, stride, rowparity) ;
		jpc_ft_fwdlift_colgrp( ( jpc_fix_t* )startaddr, numrows, stride, rowparity );
#endif
		startaddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}

	return 0;
}
#else
int jpc_ft_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride)
{
	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;
	int i;
	jpc_fix_t *startptr;
	int maxcols;

	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;
	startptr = &a[0];
	for (i = 0; i < maxcols; i += JPC_QMFB_COLGRPSIZE) {
		jpc_qmfb_split_colgrp(startptr, numrows, stride, rowparity);
		jpc_ft_fwdlift_colgrp(startptr, numrows, stride, rowparity);
		startptr += JPC_QMFB_COLGRPSIZE;
	}
	if (maxcols < numcols) {
		jpc_qmfb_split_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
		jpc_ft_fwdlift_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
	}

	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_qmfb_split_row(startptr, numcols, colparity);
		jpc_ft_fwdlift_row(startptr, numcols, colparity);
		startptr += stride;
	}

	return 0;

}
#endif

/******************************************************************************\
* 9/7 transform
\******************************************************************************/

#define ALPHA (-1.586134342059924)
#define BETA (-0.052980118572961)
#define GAMMA (0.882911075530934)
#define DELTA (0.443506852043971)
#define LGAIN (1.0 / 1.23017410558578)
#define HGAIN (1.0 / 1.62578613134411)

/* s.kang start */
#if 1
static void jpc_ns_fwdlift_row( jpc_fix_t* p_in, jpc_fix_t* p_out, int numcols, int parity )
{
	int n;
	int llen;
#ifdef FLOAT_MODE
#ifdef SIMD_EN
	int n_head;
	int n_body;
	int n_tail;
	vector float* v_f_in_lptr;
	vector float* v_f_out_lptr;
	vector float* v_f_in_hptr;
	vector float* v_f_out_hptr;
	vector float v_temp0;
	vector float v_temp1;
	int shuffle0;
	int shuffle1;
	int i;
#endif
	float* f_in_lptr;
	float* f_out_lptr;
	float* f_in_hptr;
	float* f_out_hptr;
	float f_gain1;
	float f_gain2;
#else
	jpc_fix_t* in_lptr;
	jpc_fix_t* out_lptr;
	jpc_fix_t* in_hptr;
	jpc_fix_t* out_hptr;
	jpc_fix_t gain1;
	jpc_fix_t gain2;
#endif

	spu_assert( ( ( ( unsigned int )p_in % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_fwdlift_row()] assertion failure\n" ) );
	spu_assert( ( ( ( unsigned int )p_out % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_fwdlift_row()] assertion failure\n" ) );

	llen = ( numcols + 1 - parity ) >> 1;

	if( numcols > 1 ) {
		/* Apply the first lifting step. */
#ifdef FLOAT_MODE
		f_in_lptr = ( float* )( p_in );
		f_in_hptr = ( float* )( p_in + llen );
		f_gain1 = ( float )ALPHA;
		f_gain2 = ( float )( 2.0 * ALPHA );
#else
		in_lptr = p_in;
		in_hptr = p_in + llen;
		gain1 = jpc_dbltofix( ALPHA );
		gain2 = jpc_dbltofix( 2.0 * ALPHA );
#endif
		if( parity ) {
#ifdef FLOAT_MODE
			f_in_hptr[0] += f_gain2 * f_in_lptr[0];
			f_in_hptr++;
#else
			jpc_fix_pluseq( in_hptr[0], jpc_fix_mul( gain2, in_lptr[0] ) );
			 
			++in_hptr;
#endif
		}
		n = numcols - llen - parity - ( parity == ( numcols & 1 ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_hptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_in_hptr[0] += f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
		v_f_in_hptr = ( vector float* )f_in_hptr;
		shuffle0 = ( ( unsigned int )f_in_lptr >> 2 ) & 0x3;
		shuffle1 = ( ( unsigned int )( f_in_lptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector float* )( f_in_lptr ) ), *( ( vector float* )( f_in_lptr + 4 ) ), a_v_si_shuffle_pattern[shuffle0] );
			v_temp1 = spu_shuffle( *( ( vector float* )( f_in_lptr + 1 ) ), *( ( vector float* )( f_in_lptr + 5 ) ), a_v_si_shuffle_pattern[shuffle1] );
			*v_f_in_hptr = spu_add( *v_f_in_hptr, spu_mul( spu_splats( f_gain1 ), spu_add( v_temp0, v_temp1 ) ) );
			v_f_in_hptr++;
			f_in_lptr += 4;
		}
		f_in_hptr = ( float* )v_f_in_hptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_in_hptr[0] += f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
#else
		while( n-- > 0 ) {
			f_in_hptr[0] += f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			jpc_fix_pluseq( in_hptr[0], jpc_fix_mul( gain1, jpc_fix_add( in_lptr[0], in_lptr[1] ) ) );
			++in_lptr;
			++in_hptr;
		}
#endif
		if( parity == ( numcols & 1 ) ) {
#ifdef FLOAT_MODE
			f_in_hptr[0] += f_gain2 * f_in_lptr[0];
#else
			jpc_fix_pluseq( in_hptr[0], jpc_fix_mul( gain2, in_lptr[0] ) );
#endif
		}

		/* Apply the second lifting step. */
#ifdef FLOAT_MODE
		f_in_lptr = ( float* )p_in;
		f_in_hptr = ( float* )( p_in + llen );
		f_gain1 = ( float )BETA;
		f_gain2 = ( float )( 2.0 * BETA );
#else
		in_lptr = p_in;
		in_hptr = p_in + llen;
		gain1 = jpc_dbltofix( BETA );
		gain2 = jpc_dbltofix( 2.0 * BETA );
#endif
		if( !parity ) {
#ifdef FLOAT_MODE
			f_in_lptr[0] += f_gain2 * f_in_hptr[0];
			f_in_lptr++;
#else
			jpc_fix_pluseq( in_lptr[0], jpc_fix_mul( gain2, in_hptr[0] ) );
			++in_lptr;
#endif
		}
		n = llen - ( !parity ) - ( parity != ( numcols & 1 ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_lptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_in_lptr[0] += f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
		v_f_in_lptr = ( vector float* )f_in_lptr;
		shuffle0 = ( ( unsigned int )f_in_hptr >> 2 ) & 0x3;
		shuffle1 = ( ( unsigned int )( f_in_hptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector float* )( f_in_hptr ) ), *( ( vector float* )( f_in_hptr + 4 ) ), a_v_si_shuffle_pattern[shuffle0] );
			v_temp1 = spu_shuffle( *( ( vector float* )( f_in_hptr + 1 ) ), *( ( vector float* )( f_in_hptr + 5 ) ), a_v_si_shuffle_pattern[shuffle1] );
			*v_f_in_lptr = spu_add( *v_f_in_lptr, spu_mul( spu_splats( f_gain1 ), spu_add( v_temp0, v_temp1 ) ) );
			v_f_in_lptr++;
			f_in_hptr += 4;
		}
		f_in_lptr = ( float* )v_f_in_lptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_in_lptr[0] += f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
#else
		while( n-- > 0 ) {
			f_in_lptr[0] += f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_in_hptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			jpc_fix_pluseq( in_lptr[0], jpc_fix_mul( gain1, jpc_fix_add( in_hptr[0], in_hptr[1] ) ) );
			++in_lptr;
			++in_hptr;
		}
#endif
		if( parity != ( numcols & 1 ) ) {
#ifdef FLOAT_MODE
			f_in_lptr[0] += f_gain2 * f_in_hptr[0];
#else
			jpc_fix_pluseq( in_lptr[0], jpc_fix_mul( gain2, in_hptr[0] ) );
#endif
		}

		/* Apply the third lifting step. */
#ifdef FLOAT_MODE
		f_in_lptr = ( float* )( p_in );
		f_in_hptr = ( float* )( p_in + llen );
		f_out_hptr = ( float* )( p_out + llen );
		f_gain1 = ( float )GAMMA;
		f_gain2 = ( float )( 2.0 * GAMMA );
#else
		in_lptr = p_in;
		in_hptr = p_in + llen;
		out_hptr = p_out + llen;
		gain1 = jpc_dbltofix( GAMMA );
		gain2 = jpc_dbltofix( 2.0 * GAMMA );
#endif
		if( parity ) {
#ifdef FLOAT_MODE
			f_out_hptr[0] = f_in_hptr[0] + f_gain2 * f_in_lptr[0];
			f_in_hptr++;
			f_out_hptr++;
#else
			out_hptr[0] = jpc_fix_plus( in_hptr[0], jpc_fix_mul( gain2, in_lptr[0] ) );
			 
			++in_hptr;
			++out_hptr;
#endif
		}
		n = numcols - llen - parity - ( parity == ( numcols & 1 ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_hptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_out_hptr[0] = f_in_hptr[0] + f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
			f_out_hptr++;
		}
		v_f_in_hptr = ( vector float* )f_in_hptr;
		v_f_out_hptr = ( vector float* )f_out_hptr;
		shuffle0 = ( ( unsigned int )f_in_lptr >> 2 ) & 0x3;
		shuffle1 = ( ( unsigned int )( f_in_lptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector float* )( f_in_lptr ) ), *( ( vector float* )( f_in_lptr + 4 ) ), a_v_si_shuffle_pattern[shuffle0] );
			v_temp1 = spu_shuffle( *( ( vector float* )( f_in_lptr + 1 ) ), *( ( vector float* )( f_in_lptr + 5 ) ), a_v_si_shuffle_pattern[shuffle1] );
			*v_f_out_hptr = spu_add( *v_f_in_hptr, spu_mul( spu_splats( f_gain1 ), spu_add( v_temp0, v_temp1 ) ) );
			v_f_in_hptr++;
			v_f_out_hptr++;
			f_in_lptr += 4;
		}
		f_in_hptr = ( float* )v_f_in_hptr;
		f_out_hptr = ( float* )v_f_out_hptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_out_hptr[0] = f_in_hptr[0] + f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
			f_out_hptr++;
		}
#else
		while( n-- > 0 ) {
			f_out_hptr[0] = f_in_hptr[0] + f_gain1 * ( f_in_lptr[0] + f_in_lptr[1] );
			f_in_lptr++;
			f_in_hptr++;
			f_out_hptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			out_hptr[0] = jpc_fix_plus( in_hptr[0], jpc_fix_mul( gain1, jpc_fix_add( in_lptr[0], in_lptr[1] ) ) );
			++in_lptr;
			++in_hptr;
			++out_hptr;
		}
#endif
		if( parity == ( numcols & 1 ) ) {
#ifdef FLOAT_MODE
			f_out_hptr[0] = f_in_hptr[0] + f_gain2 * f_in_lptr[0];
#else
			out_hptr[0] = jpc_fix_plus( in_hptr[0], jpc_fix_mul( gain2, in_lptr[0] ) );
#endif
		}

		/* Apply the fourth lifting step. */
#ifdef FLOAT_MODE
		f_in_lptr = ( float* )p_in;
		f_out_lptr = ( float* )p_out;
		f_in_hptr = ( float* )( p_out + llen );
		f_gain1 = ( float )DELTA;
		f_gain2 = ( float )( 2.0 * DELTA );
#else
		in_lptr = p_in;
		out_lptr = p_out;
		in_hptr = p_out + llen;
		gain1 = jpc_dbltofix( DELTA );
		gain2 = jpc_dbltofix( 2.0 * DELTA );
#endif
		if( !parity ) {
#ifdef FLOAT_MODE
			f_out_lptr[0] = f_in_lptr[0] + f_gain2 * f_in_hptr[0];
			f_in_lptr++;
			f_out_lptr++;
#else
			out_lptr[0] = jpc_fix_plus( in_lptr[0], jpc_fix_mul( gain2, in_hptr[0] ) );
			++in_lptr;
			++out_lptr;
#endif
		}
		n = llen - ( !parity ) - ( parity != ( numcols & 1 ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_lptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_out_lptr[0] = f_in_lptr[0] + f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_out_lptr++;
			f_in_hptr++;
		}
		v_f_in_lptr = ( vector float* )f_in_lptr;
		v_f_out_lptr = ( vector float* )f_out_lptr;
		shuffle0 = ( ( unsigned int )f_in_hptr >> 2 ) & 0x3;
		shuffle1 = ( ( unsigned int )( f_in_hptr + 1 ) >> 2 ) & 0x3;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_temp0 = spu_shuffle( *( ( vector float* )( f_in_hptr ) ), *( ( vector float* )( f_in_hptr + 4 ) ), a_v_si_shuffle_pattern[shuffle0] );
			v_temp1 = spu_shuffle( *( ( vector float* )( f_in_hptr + 1 ) ), *( ( vector float* )( f_in_hptr + 5 ) ), a_v_si_shuffle_pattern[shuffle1] );
			*v_f_out_lptr = spu_add( *v_f_in_lptr, spu_mul( spu_splats( f_gain1 ), spu_add( v_temp0, v_temp1 ) ) );
			v_f_in_lptr++;
			v_f_out_lptr++;
			f_in_hptr += 4;
		}
		f_in_lptr = ( float* )v_f_in_lptr;
		f_out_lptr = ( float* )v_f_out_lptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_out_lptr[0] = f_in_lptr[0] + f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_out_lptr++;
			f_in_hptr++;
		}
#else
		while( n-- > 0 ) {
			f_out_lptr[0] = f_in_lptr[0] + f_gain1 * ( f_in_hptr[0] + f_in_hptr[1] );
			f_in_lptr++;
			f_out_lptr++;
			f_in_hptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			out_lptr[0] = jpc_fix_plus( in_lptr[0], jpc_fix_mul( gain1, jpc_fix_add( in_hptr[0], in_hptr[1] ) ) );
			++in_lptr;
			++out_lptr;
			++in_hptr;
		}
#endif
		if( parity != ( numcols & 1 ) ) {
#ifdef FLOAT_MODE
			f_out_lptr[0] = f_in_lptr[0] + f_gain2 * f_in_hptr[0];
#else
			out_lptr[0] = jpc_fix_plus( in_lptr[0], jpc_fix_mul( gain2, in_hptr[0] ) );
#endif
		}

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		n = llen;
#ifdef FLOAT_MODE
		f_in_lptr = ( float* )p_out;
		f_out_lptr = ( float* )p_out;
		f_gain1 = ( float )LGAIN;
#else
		in_lptr = p_out;
		out_lptr = p_out;
		gain1 = jpc_dbltofix( LGAIN );
#endif
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_lptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_out_lptr[0] = f_in_lptr[0] * f_gain1;
			f_in_lptr++;
			f_out_lptr++;
		}
		v_f_in_lptr = ( vector float* )f_in_lptr;
		v_f_out_lptr = ( vector float* )f_out_lptr;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_f_out_lptr[0] = spu_mul( v_f_in_lptr[0], spu_splats( f_gain1 ) );
			v_f_in_lptr++;
			v_f_out_lptr++;
		}
		f_in_lptr = ( float* )v_f_in_lptr;
		f_out_lptr = ( float* )v_f_out_lptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_out_lptr[0] = f_in_lptr[0] * f_gain1;
			f_in_lptr++;
			f_out_lptr++;
		}
#else
		while( n-- > 0 ) {
			f_out_lptr[0] = f_in_lptr[0] * f_gain1;
			f_in_lptr++;
			f_out_lptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			out_lptr[0] = jpc_fix_mul( in_lptr[0], gain1 );
			++in_lptr;
			++out_lptr;
		}
#endif
		n = numcols - llen;
#ifdef FLOAT_MODE
		f_in_hptr = ( float* )( p_out + llen );
		f_out_hptr = ( float* )( p_out + llen );
		f_gain1 = ( float )HGAIN;
#else
		in_hptr = p_out + llen;
		out_hptr = p_out + llen;
		gain1 = jpc_dbltofix( HGAIN );
#endif
#ifdef FLOAT_MODE
#ifdef SIMD_EN
		n_head = ( 4 - ( ( ( unsigned int )f_in_hptr >> 2 ) & 0x3 ) ) & 0x3;
		n_body = ( n - n_head ) & 0xfffffffc;
		n_tail = n - n_head - n_body;
		for( i = 0 ; i < n_head ; i++ ) {
			f_out_hptr[0] = f_in_hptr[0] * f_gain1;
			f_in_hptr++;
			f_out_hptr++;
		}
		v_f_in_hptr = ( vector float* )f_in_hptr;
		v_f_out_hptr = ( vector float* )f_out_hptr;
		for( i = 0 ; i < n_body ; i += 4 ) {
			v_f_out_hptr[0] = spu_mul( v_f_in_hptr[0], spu_splats( f_gain1 ) );
			v_f_in_hptr++;
			v_f_out_hptr++;
		}
		f_in_hptr = ( float* )v_f_in_hptr;
		f_out_hptr = ( float* )v_f_out_hptr;
		for( i = 0 ; i < n_tail ; i++ ) {
			f_out_hptr[0] = f_in_hptr[0] * f_gain1;
			f_in_hptr++;
			f_out_hptr++;
		}
#else
		while( n-- > 0 ) {
			f_out_hptr[0] = f_in_hptr[0] * f_gain1;
			f_in_hptr++;
			f_out_hptr++;
		}
#endif
#else
		while( n-- > 0 ) {
			out_hptr[0] = jpc_fix_mul( in_hptr[0], gain1 );
			++in_hptr;
			++out_hptr;
		}
#endif
#endif

	} else {
		/* do nothing */
	}
}
#else
void jpc_ns_fwdlift_row(jpc_fix_t *a, int numcols, int parity)
{

	register jpc_fix_t *lptr;
	register jpc_fix_t *hptr;
	register int n;
	int llen;

	llen = (numcols + 1 - parity) >> 1;

	if (numcols > 1) {

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (parity) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * ALPHA),
			  lptr[0]));
			++hptr;
		}
		n = numcols - llen - parity - (parity == (numcols & 1));
		while (n-- > 0) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(ALPHA),
			  jpc_fix_add(lptr[0], lptr[1])));
			++hptr;
			++lptr;
		}
		if (parity == (numcols & 1)) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * ALPHA),
			  lptr[0]));
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (!parity) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
			  hptr[0]));
			++lptr;
		}
		n = llen - (!parity) - (parity != (numcols & 1));
		while (n-- > 0) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(BETA),
			  jpc_fix_add(hptr[0], hptr[1])));
			++lptr;
			++hptr;
		}
		if (parity != (numcols & 1)) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
			  hptr[0]));
		}

		/* Apply the third lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (parity) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * GAMMA),
			  lptr[0]));
			++hptr;
		}
		n = numcols - llen - parity - (parity == (numcols & 1));
		while (n-- > 0) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(GAMMA),
			  jpc_fix_add(lptr[0], lptr[1])));
			++hptr;
			++lptr;
		}
		if (parity == (numcols & 1)) {
			jpc_fix_pluseq(hptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * GAMMA),
			  lptr[0]));
		}

		/* Apply the fourth lifting step. */
		lptr = &a[0];
		hptr = &a[llen];
		if (!parity) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * DELTA),
			  hptr[0]));
			++lptr;
		}
		n = llen - (!parity) - (parity != (numcols & 1));
		while (n-- > 0) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(DELTA),
			  jpc_fix_add(hptr[0], hptr[1])));
			++lptr;
			++hptr;
		}
		if (parity != (numcols & 1)) {
			jpc_fix_pluseq(lptr[0], jpc_fix_mul(jpc_dbltofix(2.0 * DELTA),
			  hptr[0]));
		}

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		lptr = &a[0];
		n = llen;
		while (n-- > 0) {
			lptr[0] = jpc_fix_mul(lptr[0], jpc_dbltofix(LGAIN));
			++lptr;
		}
		hptr = &a[llen];
		n = numcols - llen;
		while (n-- > 0) {
			hptr[0] = jpc_fix_mul(hptr[0], jpc_dbltofix(HGAIN));
			++hptr;
		}
#endif

	} else {

#if defined(WT_LENONE)
		if (parity) {
			lptr = &a[0];
			lptr[0] <<= 1;
		}
#endif

	}

}
#endif
/* s.kang end  */

/* s.kang start */
#if 1
#ifdef EN_INTERLEAVING
static void jpc_ns_process_colgrp( jpc_fix_t* p_a, jpc_fix_t* splitbuf, int numrows, int stride, int parity ) {
	void * p_buf;
	jpc_fix_t* a_p_buf[QMFB_SPLIT_COLGRP_DEPTH];
	float* a_in_lptr[DWT_BUF_DEPTH];
	float* a_out_lptr[DWT_BUF_DEPTH];
	float* a_in_hptr[DWT_BUF_DEPTH];
	float* a_out_hptr[DWT_BUF_DEPTH];
	vector float* p_v_in_low0;
	vector float* p_v_in_low1;
	vector float* p_v_in_low2;
	vector float* p_v_in_high0;
	vector float* p_v_in_high1;
	vector float* p_v_in_high2;
	vector float* p_v_out_low0;
	vector float* p_v_out_low1;
	vector float* p_v_out_high0;
	vector float* p_v_out_high1;
	unsigned int ppu_nxt_r_laddr;
	unsigned int ppu_nxt_w_laddr;
	unsigned int ppu_nxt_r_haddr;
	unsigned int ppu_nxt_w_haddr;
#ifdef EN_OVERLAP
        unsigned int ppu_nxt_r_saddr;
        unsigned int ppu_nxt_w_saddr;
#endif
	float f_gain_l;
	float f_gain_h;
	float f_gain1_1;
	float f_gain1_2;
	float f_gain2_1;
	float f_gain2_2;
	float f_gain3_1;
	float f_gain3_2;
	float f_gain4_1;
	float f_gain4_2;
	int sense;
	int hstart;
	int n;
#ifdef EN_OVERLAP/* 1st path: interleaved computation loop, 2nd path: copy from splitbuf to original array */
        int n_lh_r;/* total # of read for low and high elements in 1st path */
        int n_h_w;/* total # of write for high elements in 1st path */
        int n_buf_tot;/* total # of scheduled elements read in 2nd path */
        int n_buf_r_scheduled;/* total # of scheduled (and not confirmed) read in 2nd path */
        int n_buf_r_confirmed;/* total # of confirmed (and not write scheduled) read in 2nd paht */
        int n_buf_w_scheduled;/* total # of scheduled (and not confirmed) write in 2nd path */
        int buf_r_idx;
        int buf_w_idx;
        int n_h_thresh;
#endif
	int i;
	int j;

	spu_assert( ( parity == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] parity == 1 case unimplemented\n" ) );
	spu_assert( ( DWT_BUF_DEPTH >= 5 ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] DWT_BUF_DEPTH needs to be larger than 4\n" ) );
	spu_assert( ( ( ( unsigned int )p_a % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( ( unsigned int )splitbuf % DMA_MIN_SIZE ) == 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] assertion failure\n" ) );

	hstart = ( numrows + 1 ) >> 1;
#ifdef EN_OVERLAP
        n_lh_r = 0;
        n_h_w = 0;
        n_buf_tot = 0;
        n_buf_r_scheduled = 0;
        n_buf_r_confirmed = 0;
        n_buf_w_scheduled = 0;
        buf_r_idx = 0;
        buf_w_idx = 0;
#endif

	for( i = 0 ; i < DWT_BUF_DEPTH ; i++ ) {
		a_in_lptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_in_lptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] jas_malloc_align failure\n" ) );
		a_out_lptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_out_lptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] jas_malloc_align failure\n" ) );
		a_in_hptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_in_hptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] jas_malloc_align failure\n" ) );
		a_out_hptr[i] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		spu_assert( ( a_out_hptr[i] ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] jas_malloc_align failure\n" ) );
	}

#ifdef EN_OVERLAP
        p_buf = jas_malloc_align( QMFB_SPLIT_COLGRP_DEPTH * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
        spu_assert( ( p_buf != NULL ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] assertion failure\n" ) );
        for( i = 0 ; i < QMFB_SPLIT_COLGRP_DEPTH ; i++ ) {
                a_p_buf[i] = p_buf + i * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
        }
#endif

	if( numrows >= 5 ) {
		f_gain1_1 = ( float )ALPHA;
		f_gain1_2 = ( float )( 2.0 * ALPHA );
		f_gain2_1 = ( float )BETA;
		f_gain2_2 = ( float )( 2.0 * BETA );
		f_gain3_1 = ( float )GAMMA;
		f_gain3_2 = ( float )( 2.0 * GAMMA );
		f_gain4_1 = ( float )DELTA;
		f_gain4_2 = ( float )( 2.0 * DELTA );
		f_gain_l = ( float )LGAIN;
		f_gain_h = ( float )HGAIN;

		ppu_nxt_r_laddr = ( unsigned int )p_a;
		ppu_nxt_r_haddr = ( unsigned int )( p_a + stride );
		ppu_nxt_w_laddr = ( unsigned int )p_a;
		ppu_nxt_w_haddr = ( unsigned int )splitbuf;
#ifdef EN_OVERLAP
                ppu_nxt_r_saddr = ( unsigned int )splitbuf;
                ppu_nxt_w_saddr = ( unsigned int )( p_a + hstart * stride );
#endif

		for( i = 0 ; ( i < ( DWT_BUF_DEPTH - 1 ) ) && ( i < ( ( numrows + 1 ) >> 1 ) ) ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_laddr, ( void* )a_in_lptr[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), i );
			ppu_nxt_r_laddr += 2 * stride * sizeof( jpc_fix_t );
			if( i < ( numrows >> 1 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_in_hptr[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), i );
				ppu_nxt_r_haddr += 2 * stride * sizeof( jpc_fix_t );
			}
		}

		/* head */

		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( 0 );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( 1 );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( 2 );
#ifdef EN_OVERLAP
                n_lh_r += 6;
#endif
		p_v_in_low0 = ( vector float* )a_in_lptr[0];
		p_v_in_low1 = ( vector float* )a_in_lptr[1];
		p_v_in_low2 = ( vector float* )a_in_lptr[2];
		p_v_in_high0 = ( vector float* )a_in_hptr[0];
		p_v_in_high1 = ( vector float* )a_in_hptr[1];
		p_v_out_low0 = ( vector float* )a_out_lptr[0];
		p_v_out_high0 = ( vector float* )a_out_hptr[0];
		for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
			/* 1st lifting */
			*p_v_in_high0 = spu_add( *p_v_in_high0, spu_mul( spu_splats( f_gain1_1 ), spu_add( *p_v_in_low0, *p_v_in_low1 ) ) );
			*p_v_in_high1 = spu_add( *p_v_in_high1, spu_mul( spu_splats( f_gain1_1 ), spu_add( *p_v_in_low1, *p_v_in_low2 ) ) );
			/* 2nd lifting */
			*p_v_in_low0 = spu_add( *p_v_in_low0, spu_mul( spu_splats( f_gain2_2 ), *p_v_in_high0 ) );
			*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain2_1 ), spu_add( *p_v_in_high0, *p_v_in_high1 ) ) );
			/* 3rd lifting */
			*p_v_in_high0 = spu_add( *p_v_in_high0, spu_mul( spu_splats( f_gain3_1 ), spu_add( *p_v_in_low0, *p_v_in_low1 ) ) );
			/* 4th lifting */
			*p_v_in_low0 = spu_add( *p_v_in_low0, spu_mul( spu_splats( f_gain4_2 ), *p_v_in_high0 ) );
			/* scaling */
#if defined(WT_DOSCALE)
			*p_v_out_low0 = spu_mul( *p_v_in_low0, spu_splats( f_gain_l ) );
			*p_v_out_high0 = spu_mul( *p_v_in_high0, spu_splats( f_gain_h ) );
#else
			*p_v_out_low0 = *p_v_in_low0;
			*p_v_out_high0 = *p_v_in_high0;
#endif
			p_v_in_low0++;
			p_v_in_low1++;
			p_v_in_low2++;
			p_v_in_high0++;
			p_v_in_high1++;
			p_v_out_low0++;
			p_v_out_high0++;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[0], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), 0 );
		ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[0], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), 0 );
		ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );

		/* body */

		sense = 1;
		n = ( ( numrows + 1 ) >> 1 ) - 3;
#ifdef EN_OVERLAP
                n_h_thresh = n - DWT_BUF_DEPTH;
#endif

		while( n-- ) {
#ifdef EN_OVERLAP
                        if( ( n_lh_r >= hstart + 2 ) && ( n_h_w - n_buf_tot >= 2 ) ) {
                                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                                ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
                                buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

                                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                                ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
                                buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

                                n_buf_tot += 2;
                                n_buf_r_scheduled += 2;
                        }
#endif
			if( n >= ( DWT_BUF_DEPTH - 4 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_laddr, ( void* )a_in_lptr[( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), ( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH );
				ppu_nxt_r_laddr += 2 * stride * sizeof( jpc_fix_t );
				if( ( n != ( DWT_BUF_DEPTH - 4 ) ) || ( ( numrows & 0x1 ) == 0 ) ) {
					JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_in_hptr[( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), ( sense + DWT_BUF_DEPTH - 2 ) % DWT_BUF_DEPTH );
					ppu_nxt_r_haddr += 2 * stride * sizeof( jpc_fix_t );
				}
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
#ifdef EN_OVERLAP
                        if( n <= n_h_thresh ) {
                                n_h_w++;
                        }
#endif
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + 2 ) % DWT_BUF_DEPTH );
#ifdef EN_OVERLAP
                        n_lh_r += 2;
#endif

			p_v_in_low0 = ( vector float* )a_in_lptr[sense];
			p_v_in_low1 = ( vector float* )a_in_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_in_low2 = ( vector float* )a_in_lptr[( sense + 2 ) % DWT_BUF_DEPTH];
			p_v_in_high0 = ( vector float* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_in_high1 = ( vector float* )a_in_hptr[sense];
			p_v_in_high2 = ( vector float* )a_in_hptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_out_low0 = ( vector float* )a_out_lptr[sense];
			p_v_out_high0 = ( vector float* )a_out_hptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {	
				/* 1st lifting */
				*p_v_in_high2 = spu_add( *p_v_in_high2, spu_mul( spu_splats( f_gain1_1 ), spu_add( *p_v_in_low1, *p_v_in_low2 ) ) );
				/* 2nd lifting */
				*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain2_1 ), spu_add( *p_v_in_high1, *p_v_in_high2 ) ) );
				/* 3rd lifting */
				*p_v_in_high1 = spu_add( *p_v_in_high1, spu_mul( spu_splats( f_gain3_1 ), spu_add( *p_v_in_low0, *p_v_in_low1 ) ) );
				/* 4th lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_mul( spu_splats( f_gain4_1 ), spu_add( *p_v_in_high0, *p_v_in_high1 ) ) );
				/* scaling */
#if defined(WT_DOSCALE)
				*p_v_out_low0 = spu_mul( *p_v_in_low0, spu_splats( f_gain_l ) );
				*p_v_out_high0 = spu_mul( *p_v_in_high1, spu_splats( f_gain_h ) );
#else
				*p_v_out_low0 = *p_v_in_low0;
				*p_v_out_high0 = *p_v_in_high1;
#endif
				p_v_in_low0++;
				p_v_in_low1++;
				p_v_in_low2++;
				p_v_in_high0++;
				p_v_in_high1++;
				p_v_in_high2++;
				p_v_out_low0++;
				p_v_out_high0++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) % DWT_BUF_DEPTH;
#ifdef EN_OVERLAP
                        if( n_buf_r_scheduled >= QMFB_SPLIT_COLGRP_DEPTH / 4 ) {
                                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
                                n_buf_r_confirmed += n_buf_r_scheduled;
                                n_buf_r_scheduled = 0;
                        }
                        if( n_buf_w_scheduled >= QMFB_SPLIT_COLGRP_DEPTH / 4 ) {
                                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
                                n_buf_w_scheduled = 0;
                        }
                        if( n_buf_r_confirmed >= 2 ) {
                                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                                ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
                                buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

                                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                                ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
                                buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;

                                n_buf_r_confirmed -= 2;
                                n_buf_w_scheduled += 2;
                        }
#endif
		}

		/* tail */

		if( ( numrows & 0x1 ) == 0 ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + 1 ) % DWT_BUF_DEPTH );
			p_v_in_low0 = ( vector float* )a_in_lptr[sense];
			p_v_in_low1 = ( vector float* )a_in_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_in_high0 = ( vector float* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_in_high1 = ( vector float* )a_in_hptr[sense];
			p_v_in_high2 = ( vector float* )a_in_hptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_out_low0 = ( vector float* )a_out_lptr[sense];
			p_v_out_low1 = ( vector float* )a_out_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_out_high0 = ( vector float* )a_out_hptr[sense];
			p_v_out_high1 = ( vector float* )a_out_hptr[( sense + 1 ) % DWT_BUF_DEPTH];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				/* 1st lifting */
				*p_v_in_high2 = spu_add( *p_v_in_high2, spu_mul( spu_splats( f_gain1_2 ), *p_v_in_low1 ) );
				/* 2nd lifting */
				*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain2_1 ), spu_add( *p_v_in_high1, *p_v_in_high2 ) ) );
				/* 3rd lifting */
				*p_v_in_high1 = spu_add( *p_v_in_high1, spu_mul( spu_splats( f_gain3_1 ), spu_add( *p_v_in_low0, *p_v_in_low1 ) ) );
				*p_v_in_high2 = spu_add( *p_v_in_high2, spu_mul( spu_splats( f_gain3_2 ), *p_v_in_low1 ) );
				/* 4th lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_mul( spu_splats( f_gain4_1 ), spu_add( *p_v_in_high0, *p_v_in_high1 ) ) );
				*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain4_1 ), spu_add( *p_v_in_high1, *p_v_in_high2 ) ) );
				/* scaling */
#if defined(WT_DOSCALE)
				*p_v_out_low0 = spu_mul( *p_v_in_low0, spu_splats( f_gain_l ) );
				*p_v_out_low1 = spu_mul( *p_v_in_low1, spu_splats( f_gain_l ) );
				*p_v_out_high0 = spu_mul( *p_v_in_high1, spu_splats( f_gain_h ) );
				*p_v_out_high1 = spu_mul( *p_v_in_high2, spu_splats( f_gain_h ) );
#else
				*p_v_out_low0 = *p_v_in_low0;
				*p_v_out_low1 = *p_v_in_low1;
				*p_v_out_high0 = *p_v_in_high1;
				*p_v_out_high1 = *p_v_in_high2;
#endif
				p_v_in_low0++;
				p_v_in_low1++;
				p_v_in_high0++;
				p_v_in_high1++;
				p_v_in_high2++;
				p_v_out_low0++;
				p_v_out_low1++;
				p_v_out_high0++;
				p_v_out_high1++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) % DWT_BUF_DEPTH;
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
		}
		else {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( sense );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( ( sense + 1 ) % DWT_BUF_DEPTH );
			p_v_in_low0 = ( vector float* )a_in_lptr[sense];
			p_v_in_low1 = ( vector float* )a_in_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_in_high0 = ( vector float* )a_in_hptr[( sense + DWT_BUF_DEPTH - 1 ) % DWT_BUF_DEPTH];
			p_v_in_high1 = ( vector float* )a_in_hptr[sense];
			p_v_out_low0 = ( vector float* )a_out_lptr[sense];
			p_v_out_low1 = ( vector float* )a_out_lptr[( sense + 1 ) % DWT_BUF_DEPTH];
			p_v_out_high0 = ( vector float* )a_out_hptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				/* 1st lifting */
				/* 2nd lifting */
				*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain2_2 ), *p_v_in_high1 ) );
				/* 3rd lifting */
				*p_v_in_high1 = spu_add( *p_v_in_high1, spu_mul( spu_splats( f_gain3_1 ), spu_add( *p_v_in_low0, *p_v_in_low1 ) ) );
				/* 4th lifting */
				*p_v_in_low0 = spu_add( *p_v_in_low0, spu_mul( spu_splats( f_gain4_1 ), spu_add( *p_v_in_high0, *p_v_in_high1 ) ) );
				*p_v_in_low1 = spu_add( *p_v_in_low1, spu_mul( spu_splats( f_gain4_2 ), *p_v_in_high1 ) );
				/* scaling */
#if defined(WT_DOSCALE)
				*p_v_out_low0 = spu_mul( *p_v_in_low0, spu_splats( f_gain_l ) );
				*p_v_out_low1 = spu_mul( *p_v_in_low1, spu_splats( f_gain_l ) );
				*p_v_out_high0 = spu_mul( *p_v_in_high1, spu_splats( f_gain_h ) );
#else
				*p_v_out_low0 = *p_v_in_low0;
				*p_v_out_low1 = *p_v_in_low1;
				*p_v_out_high0 = *p_v_in_high1;
#endif
				p_v_in_low0++;
				p_v_in_low1++;
				p_v_in_high0++;
				p_v_in_high1++;
				p_v_out_low0++;
				p_v_out_low1++;
				p_v_out_high0++;
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_laddr += stride * sizeof( jpc_fix_t );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
			ppu_nxt_w_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) % DWT_BUF_DEPTH;
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_nxt_w_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), sense );
		}

		for( i = 0 ; i < DWT_BUF_DEPTH ; i++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_MULTI_CONFIRM( i );
		}
	}
	else {
		spu_assert( ( 0 ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] numrows < 5 case need to be implemented\n" ) );
	}

	/* Copy the saved samples into the highpass channel. */

#ifdef EN_OVERLAP
        if( n_buf_r_scheduled > 0 ) {
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
                n_buf_r_confirmed += n_buf_r_scheduled;
                n_buf_r_scheduled = 0;
        }

        n = n_buf_r_confirmed;
        while( n-- ) {
                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
                buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
        }
        n_buf_w_scheduled += n_buf_r_confirmed;
        n_buf_r_confirmed = 0;

        if( n_buf_w_scheduled > 0 ) {
                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
                n_buf_w_scheduled = 0;
        }

        n = numrows - hstart - n_buf_tot;
        if( n != 0 ) {
                spu_assert( ( n <= QMFB_SPLIT_COLGRP_DEPTH ), ( "[jpc_qmfb_analyze.c:jpc_ft_process_colgrp()] n too large\n" ) );
                for( j = 0 ; j < n ; j++ ) {
                        JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_saddr, ( void* )a_p_buf[buf_r_idx], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                        ppu_nxt_r_saddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
                        buf_r_idx = ( buf_r_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
                }
                JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
                for( j = 0 ; j < n ; j++ ) {
                        JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[buf_w_idx], ( unsigned long long )ppu_nxt_w_saddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
                        ppu_nxt_w_saddr += stride * sizeof( jpc_fix_t );
                        buf_w_idx = ( buf_w_idx + 1 ) % QMFB_SPLIT_COLGRP_DEPTH;
                }
                JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
        }

        jas_free_align( p_buf );
#else
	p_buf = jas_malloc_align( QMFB_SPLIT_COLGRP_DEPTH * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( p_buf != NULL ), ( "[jpc_qmfb_analyze.c:jpc_ns_process_colgrp()] assertion failure\n" ) );
	for( i = 0 ; i < QMFB_SPLIT_COLGRP_DEPTH ; i++ ) {
		a_p_buf[i] = p_buf + i * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}

	n = numrows - hstart;
	ppu_nxt_r_haddr = ( unsigned int )splitbuf;
	ppu_nxt_w_haddr = ( unsigned int )( p_a + hstart * stride );

	for( i = 0 ; i < n - ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i += QMFB_SPLIT_COLGRP_DEPTH ) {
		for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_p_buf[j], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_r_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
		}
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
		for( j = 0 ; j < QMFB_SPLIT_COLGRP_DEPTH ; j++ ) {
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[j], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_nxt_w_haddr += stride * sizeof( jpc_fix_t );
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}

	for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_nxt_r_haddr, ( void* )a_p_buf[i], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		ppu_nxt_r_haddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}
	JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	for( i = 0 ; i < ( n % QMFB_SPLIT_COLGRP_DEPTH ) ; i++ ) {
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_p_buf[i], ( unsigned long long )ppu_nxt_w_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		ppu_nxt_w_haddr += stride * sizeof( jpc_fix_t );
	}
	JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	jas_free_align( p_buf );
#endif

	for( i = 0 ; i < DWT_BUF_DEPTH ;i++ ) {
		jas_free_align( a_in_lptr[i] );
		jas_free_align( a_out_lptr[i] );
		jas_free_align( a_in_hptr[i] );
		jas_free_align( a_out_hptr[i] );
	}
}
#else
static void jpc_ns_fwdlift_colgrp( jpc_fix_t* a, int numrows, int stride, int parity )
{
	jpc_fix_t* a_in_lptr[2];
	jpc_fix_t* a_out_lptr[2];
	jpc_fix_t* a_in_hptr[2];
	jpc_fix_t* a_out_hptr[2];
	jpc_fix_t* a_in_sptr[2];
	int sense;
	int dma_wr_scheduled;
	unsigned int ppu_laddr;
	unsigned int ppu_haddr;
#ifdef FLOAT_MODE
#ifdef SIMD_EN
	vector float* p_v_in_low;
	vector float* p_v_out_low;
	vector float* p_v_in_high;
	vector float* p_v_out_high;
	vector float* p_v_in_stride;
#else
	float* f_in_lptr2;
	float* f_out_lptr2;
	float* f_in_hptr2;
	float* f_out_hptr2;
	float* f_in_sptr2;
#endif
	float f_gain1;
	float f_gain2;
#else
	jpc_fix_t* in_lptr2;
	jpc_fix_t* out_lptr2;
	jpc_fix_t* in_hptr2;
	jpc_fix_t* out_hptr2;
	jpc_fix_t* in_sptr2;
	jpc_fix_t gain1;
	jpc_fix_t gain2;
#endif
	int n;
	int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	a_in_lptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_lptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_lptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_lptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_hptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_hptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_hptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_out_hptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_sptr[0] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	a_in_sptr[1] = jas_malloc_align( JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
	spu_assert( ( ( a_in_lptr[0] != NULL ) && ( a_in_lptr[1] != NULL ) && ( a_out_lptr[0] != NULL ) && ( a_out_lptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ns_fwdlift_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( a_in_hptr[0] != NULL ) && ( a_in_hptr[1] != NULL ) && ( a_out_hptr[0] != NULL ) && ( a_out_hptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ns_fwdlift_colgrp()] assertion failure\n" ) );
	spu_assert( ( ( a_in_sptr[0] != NULL ) && ( a_in_sptr[1] != NULL ) ), ( "[jpc_qmfb_analyze.c:jpc_ns_fwdlift_colgrp()] assertion failure\n" ) );

	if (numrows > 1) {
		/* Apply the first lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
#ifdef FLOAT_MODE
		f_gain1 = ( float )ALPHA;
		f_gain2 = ( float )( 2.0 * ALPHA );
#else
		gain1 = jpc_dbltofix( ALPHA );
		gain2 = jpc_dbltofix( 2.0 * ALPHA );
#endif
		if (parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			p_v_out_high = ( vector float* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain2 ), *p_v_in_low ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			f_out_hptr2 = ( float* )a_out_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain2 * f_in_lptr2[0];
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain2, in_lptr2[0] ) );
				++in_hptr2;
				++out_hptr2;
				++in_lptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_haddr += stride * sizeof( jpc_fix_t );
		}

		n = numrows - llen - parity - (parity == (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[sense];
			p_v_in_high = ( vector float* )a_in_hptr[sense];
			p_v_out_high = ( vector float* )a_out_hptr[sense];
			p_v_in_stride = ( vector float* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain1 ), spu_add( *p_v_in_low, *p_v_in_stride ) ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
				p_v_in_stride++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[sense];
			f_in_hptr2 = ( float* )a_in_hptr[sense];
			f_out_hptr2 = ( float* )a_out_hptr[sense];
			f_in_sptr2 = ( float* )a_in_sptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain1 * ( f_in_lptr2[0] + f_in_sptr2[0] );
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
				f_in_sptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			out_hptr2 = a_out_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain1, jpc_fix_add( in_lptr2[0], in_sptr2[0] ) ) );
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity == (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			p_v_out_high = ( vector float* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain2 ), *p_v_in_low ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			f_out_hptr2 = ( float* )a_out_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain2 * f_in_lptr2[0];
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain2, in_lptr2[0] ) );
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

		/* Apply the second lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
#ifdef FLOAT_MODE
		f_gain1 = ( float )BETA;
		f_gain2 = ( float )( 2.0 * BETA );
#else
		gain1 = jpc_dbltofix( BETA );
		gain2 = jpc_dbltofix( 2.0 * BETA );
#endif
		if (!parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_out_low = ( vector float* )a_out_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain2 ), *p_v_in_high ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_out_lptr2 = ( float* )a_out_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain2 * f_in_hptr2[0];
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain2, in_hptr2[0] ) );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_laddr += stride * sizeof( jpc_fix_t );
		}

		n = llen - (!parity) - (parity != (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[sense];
			p_v_out_low = ( vector float* )a_out_lptr[sense];
			p_v_in_high = ( vector float* )a_in_hptr[sense];
			p_v_in_stride = ( vector float* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain1 ), spu_add( *p_v_in_high, *p_v_in_stride ) ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
				p_v_in_stride++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[sense];
			f_out_lptr2 = ( float* )a_out_lptr[sense];
			f_in_hptr2 = ( float* )a_in_hptr[sense];
			f_in_sptr2 = ( float* )a_in_sptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain1 * ( f_in_hptr2[0] + f_in_sptr2[0] );
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
				f_in_sptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[sense];
			out_lptr2 = a_out_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain1, jpc_fix_add( in_hptr2[0],  in_sptr2[0] ) ) );

				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity != (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_out_low = ( vector float* )a_out_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain2 ), *p_v_in_high ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_out_lptr2 = ( float* )a_out_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain2 * f_in_hptr2[0];
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain2, in_hptr2[0] ) );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

		/* Apply the third lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
#ifdef FLOAT_MODE
		f_gain1 = ( float )GAMMA;
		f_gain2 = ( float )( 2.0 * GAMMA );
#else
		gain1 = jpc_dbltofix( GAMMA );
		gain2 = jpc_dbltofix( 2.0 * GAMMA );
#endif
		if (parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			p_v_out_high = ( vector float* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain2 ), *p_v_in_low ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			f_out_hptr2 = ( float* )a_out_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain2 * f_in_lptr2[0];
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain2, in_lptr2[0] ) );
				++in_hptr2;
				++out_hptr2;
				++in_lptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_haddr += stride * sizeof( jpc_fix_t );
		}

		n = numrows - llen - parity - (parity == (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[sense];
			p_v_in_high = ( vector float* )a_in_hptr[sense];
			p_v_out_high = ( vector float* )a_out_hptr[sense];
			p_v_in_stride = ( vector float* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain1 ), spu_add( *p_v_in_low, *p_v_in_stride ) ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
				p_v_in_stride++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[sense];
			f_in_hptr2 = ( float* )a_in_hptr[sense];
			f_out_hptr2 = ( float* )a_out_hptr[sense];
			f_in_sptr2 = ( float* )a_in_sptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain1 * ( f_in_lptr2[0] + f_in_sptr2[0] );
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
				f_in_sptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			out_hptr2 = a_out_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain1, jpc_fix_add( in_lptr2[0], in_sptr2[0] ) ) );
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity == (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			p_v_out_high = ( vector float* )a_out_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_add( *p_v_in_high, spu_mul( spu_splats( f_gain2 ), *p_v_in_low ) );
				p_v_in_low++;
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			f_out_hptr2 = ( float* )a_out_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] + f_gain2 * f_in_lptr2[0];
				f_in_lptr2++;
				f_in_hptr2++;
				f_out_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			in_hptr2 = a_in_hptr[0];
			out_hptr2 = a_out_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_plus( in_hptr2[0], jpc_fix_mul( gain2, in_lptr2[0] ) );
				++in_lptr2;
				++in_hptr2;
				++out_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_hptr[0], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

		/* Apply the fourth lifting step. */

		ppu_laddr = ( unsigned int )a;
		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
#ifdef FLOAT_MODE
		f_gain1 = ( float )DELTA;
		f_gain2 = ( float )( 2.0 * DELTA );
#else
		gain1 = jpc_dbltofix( DELTA );
		gain2 = jpc_dbltofix( 2.0 * DELTA );
#endif
		if (!parity) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_out_low = ( vector float* )a_out_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain2 ), *p_v_in_high ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_out_lptr2 = ( float* )a_out_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain2 * f_in_hptr2[0];
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain2, in_hptr2[0] ) );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			ppu_laddr += stride * sizeof( jpc_fix_t );
		}

		n = llen - (!parity) - (parity != (numrows & 1));
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_sptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ) * 2, ( void* )a_in_sptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[sense];
			p_v_out_low = ( vector float* )a_out_lptr[sense];
			p_v_in_high = ( vector float* )a_in_hptr[sense];
			p_v_in_stride = ( vector float* )a_in_sptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain1 ), spu_add( *p_v_in_high, *p_v_in_stride ) ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
				p_v_in_stride++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[sense];
			f_out_lptr2 = ( float* )a_out_lptr[sense];
			f_in_hptr2 = ( float* )a_in_hptr[sense];
			f_in_sptr2 = ( float* )a_in_sptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain1 * ( f_in_hptr2[0] + f_in_sptr2[0] );
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
				f_in_sptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[sense];
			out_lptr2 = a_out_lptr[sense];
			in_hptr2 = a_in_hptr[sense];
			in_sptr2 = a_in_sptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain1, jpc_fix_add( in_hptr2[0],  in_sptr2[0] ) ) );

				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
				++in_sptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		if (parity != (numrows & 1)) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[0], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[0];
			p_v_out_low = ( vector float* )a_out_lptr[0];
			p_v_in_high = ( vector float* )a_in_hptr[0];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector signed int ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_add( *p_v_in_low, spu_mul( spu_splats( f_gain2 ), *p_v_in_high ) );
				p_v_in_low++;
				p_v_out_low++;
				p_v_in_high++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[0];
			f_out_lptr2 = ( float* )a_out_lptr[0];
			f_in_hptr2 = ( float* )a_in_hptr[0];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] + f_gain2 * f_in_hptr2[0];
				f_in_lptr2++;
				f_out_lptr2++;
				f_in_hptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[0];
			out_lptr2 = a_out_lptr[0];
			in_hptr2 = a_in_hptr[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_plus( in_lptr2[0], jpc_fix_mul( gain2, in_hptr2[0] ) );
				++in_lptr2;
				++out_lptr2;
				++in_hptr2;
			}
#endif
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_out_lptr[0], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		}

		/* Apply the scaling step. */

#if defined(WT_DOSCALE)
		ppu_laddr = ( unsigned int )a;
		n = llen;
#ifdef FLOAT_MODE
		f_gain1 = ( float )LGAIN;
#else
		gain1 = jpc_dbltofix( LGAIN );
#endif
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr, ( void* )a_in_lptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_laddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_lptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_low = ( vector float* )a_in_lptr[sense];
			p_v_out_low = ( vector float* )a_out_lptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector float ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_low = spu_mul( *p_v_in_low, spu_splats( f_gain1 ) );
				p_v_in_low++;
				p_v_out_low++;
			}
#else
			f_in_lptr2 = ( float* )a_in_lptr[sense];
			f_out_lptr2 = ( float* )a_out_lptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_lptr2[0] = f_in_lptr2[0] * f_gain1;
				f_in_lptr2++;
				f_out_lptr2++;
			}
#endif
#else
			in_lptr2 = a_in_lptr[sense];
			out_lptr2 = a_out_lptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_lptr2[0] = jpc_fix_mul( in_lptr2[0], gain1 );
				++in_lptr2;
				++out_lptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_lptr[sense], ( unsigned long long )ppu_laddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_laddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();

		ppu_haddr = ( unsigned int )a + llen * stride * sizeof( jpc_fix_t );
		n = numrows - llen;
#ifdef FLOAT_MODE
		f_gain1 = ( float )HGAIN;
#else
		gain1 = jpc_dbltofix( HGAIN );
#endif
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr, ( void* )a_in_hptr[sense], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
		while (n-- > 0) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( n != 0 ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )ppu_haddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_hptr[( sense + 1 ) & 0x1], JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			}
#ifdef FLOAT_MODE
#ifdef SIMD_EN
			p_v_in_high = ( vector float* )a_in_hptr[sense];
			p_v_out_high = ( vector float* )a_out_hptr[sense];
			for( i = 0 ; i < ( int )( JPC_QMFB_COLGRPSIZE / ( sizeof( vector float ) / sizeof( jpc_fix_t ) ) ) ; i++ ) {
				*p_v_out_high = spu_mul( *p_v_in_high, spu_splats( f_gain1 ) );
				p_v_in_high++;
				p_v_out_high++;
			}
#else
			f_in_hptr2 = ( float* )a_in_hptr[sense];
			f_out_hptr2 = ( float* )a_out_hptr[sense];
			for( i = 0 ; i < JPC_QMFB_COLGRPSIZE ; i++ ) {
				f_out_hptr2[0] = f_in_hptr2[0] * f_gain1;
				f_in_hptr2++;
				f_out_hptr2++;
			}
#endif
#else
			in_hptr2 = a_in_hptr[sense];
			out_hptr2 = a_out_hptr[sense];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				out_hptr2[0] = jpc_fix_mul( in_hptr2[0], gain1 );
				++in_hptr2;
				++out_hptr2;
			}
#endif
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_hptr[sense], ( unsigned long long )ppu_haddr, JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) );
			dma_wr_scheduled = 1;
			ppu_haddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
#endif
	} else {
		/* do nothing */
	}

	jas_free_align( a_in_lptr[0] );
	jas_free_align( a_in_lptr[1] );
	jas_free_align( a_out_lptr[0] );
	jas_free_align( a_out_lptr[1] );
	jas_free_align( a_in_hptr[0] );
	jas_free_align( a_in_hptr[1] );
	jas_free_align( a_out_hptr[0] );
	jas_free_align( a_out_hptr[1] );
	jas_free_align( a_in_sptr[0] );
	jas_free_align( a_in_sptr[1] );
}
#endif
#else
void jpc_ns_fwdlift_colgrp(jpc_fix_t *a, int numrows, int stride,
  int parity)
{

	jpc_fix_t *lptr;
	jpc_fix_t *hptr;
	register jpc_fix_t *lptr2;
	register jpc_fix_t *hptr2;
	register int n;
	register int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	if (numrows > 1) {

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * ALPHA),
				  lptr2[0]));
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(ALPHA),
				  jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * ALPHA),
				  lptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
				  hptr2[0]));
				++lptr2;
				++hptr2;
			}
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(BETA),
				  jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
				  hptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the third lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * GAMMA),
				  lptr2[0]));
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(GAMMA),
				  jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * GAMMA),
				  lptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the fourth lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * DELTA),
				  hptr2[0]));
				++lptr2;
				++hptr2;
			}
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(DELTA),
				  jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * DELTA),
				  hptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		lptr = &a[0];
		n = llen;
		while (n-- > 0) {
			lptr2 = lptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] = jpc_fix_mul(lptr2[0], jpc_dbltofix(LGAIN));
				++lptr2;
			}
			lptr += stride;
		}
		hptr = &a[llen * stride];
		n = numrows - llen;
		while (n-- > 0) {
			hptr2 = hptr;
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				hptr2[0] = jpc_fix_mul(hptr2[0], jpc_dbltofix(HGAIN));
				++hptr2;
			}
			hptr += stride;
		}
#endif

	} else {

#if defined(WT_LENONE)
		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < JPC_QMFB_COLGRPSIZE; ++i) {
				lptr2[0] <<= 1;
				++lptr2;
			}
		}
#endif

	}

}
#endif
/* s.kang end */

/* s.kang start */
#if 1
int jpc_ns_row_analyze( unsigned int startaddr, int numrows, int numcols, int stride, int colparity ) {
	jpc_fix_t* a_in_ptr[2];
	jpc_fix_t* a_out_ptr[2];
	int en_buf;/* enable double buffering */
	int sense;
	int dma_wr_scheduled;
	int dma_size;
	int i;

	a_in_ptr[0] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_in_ptr[1] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_out_ptr[0] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	a_out_ptr[1] = jas_malloc_align( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE, CACHE_LINE_OFFSET_BITS );
	if( a_in_ptr[0] == NULL ) {
		jas_eprintf( "[jpc_qmfb_analyze.c:jpc_ns_row_analyze()] jas_malloc_align failure\n" );
		return -1;
	}
	else if( ( a_in_ptr[1] != NULL ) && ( a_out_ptr[0] != NULL ) && ( a_out_ptr[1] != NULL ) ) {
		en_buf = 1;
	}
	else {
		jas_eprintf( "double buffering disabled for row analyze\n" );
		en_buf = 0;
		if( a_in_ptr[1] ) {
			jas_free_align( a_in_ptr[1] );
		}
		if( a_out_ptr[0] ) {
			jas_free_align( a_out_ptr[0] );
		}
		if( a_out_ptr[1] ) {
			jas_free_align( a_out_ptr[1] );
		}
	}

	dma_size = ( numcols * sizeof( jpc_fix_t ) + CACHE_LINE_SIZE - 1 ) & CACHE_LINE_OFFSET_MASK;

	if( en_buf ) {
		sense = 0;
		dma_wr_scheduled = 0;
		JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )startaddr, ( void* )a_in_ptr[sense], dma_size );
		for( i = 0; i < numrows ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			if( i != ( numrows - 1 ) ) {
				JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( unsigned long long )startaddr + stride * sizeof( jpc_fix_t ), ( void* )a_in_ptr[( sense + 1 ) & 0x1], dma_size );
			}
			jpc_qmfb_split_row( a_in_ptr[sense], numcols, colparity );
			jpc_ns_fwdlift_row( a_in_ptr[sense], a_out_ptr[sense], numcols, colparity );
			memcpy( a_out_ptr[sense] + numcols, a_in_ptr[sense] + numcols, dma_size - numcols * sizeof( jpc_fix_t ) );
			if( dma_wr_scheduled ) {
				JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
			}
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_SCHEDULE( ( void* )a_out_ptr[sense], ( unsigned long long )startaddr, dma_size );
			dma_wr_scheduled = 1;
			startaddr += stride * sizeof( jpc_fix_t );
			sense = ( sense + 1 ) & 0x1;
		}
		JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED_CONFIRM();
	}
	else {
		for( i = 0; i < numrows ; i++ ) {
			JAS_DMA_GET_FROM_PPU_CACHE_LINE_ALIGNED( ( unsigned long long )startaddr, ( void* )a_in_ptr[0], dma_size );
			jpc_qmfb_split_row( a_in_ptr[0], numcols, colparity );
			jpc_ns_fwdlift_row( a_in_ptr[0], a_in_ptr[0], numcols, colparity );
			JAS_DMA_PUT_TO_PPU_CACHE_LINE_ALIGNED( ( void* )a_in_ptr[0], ( unsigned long long )startaddr, dma_size );
			startaddr += stride * sizeof( jpc_fix_t );
		}
	}

	if( en_buf ) {
		jas_free_align( a_in_ptr[0] );
		jas_free_align( a_in_ptr[1] );
		jas_free_align( a_out_ptr[0] );
		jas_free_align( a_out_ptr[1] );
	}
	else {
		jas_free_align( a_in_ptr[0] );
	}

	return 0;
}

int jpc_ns_colgrp_analyze( unsigned int startaddr, unsigned int splitbufaddr, int numrows, int numcolgrps, int stride, int rowparity ) {
	int i;

	for( i = 0 ; i < numcolgrps ; i++ ) {
#ifdef EN_INTERLEAVING
		jpc_ns_process_colgrp( ( jpc_fix_t* )startaddr, ( jpc_fix_t* )splitbufaddr, numrows, stride, rowparity );
#else
		jpc_qmfb_split_colgrp( ( jpc_fix_t* )startaddr, ( jpc_fix_t* )splitbufaddr, numrows, stride, rowparity) ;
		jpc_ns_fwdlift_colgrp( ( jpc_fix_t* )startaddr, numrows, stride, rowparity );
#endif
		startaddr += JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t );
	}

	return 0;
}
#else
int jpc_ns_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride)
{

	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;
	int i;
	jpc_fix_t *startptr;
	int maxcols;

	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;
	startptr = &a[0];
	for (i = 0; i < maxcols; i += JPC_QMFB_COLGRPSIZE) {
		jpc_qmfb_split_colgrp(startptr, numrows, stride, rowparity);
		jpc_ns_fwdlift_colgrp(startptr, numrows, stride, rowparity);
		startptr += JPC_QMFB_COLGRPSIZE;
	}
	if (maxcols < numcols) {
		jpc_qmfb_split_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
		jpc_ns_fwdlift_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
	}

	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_qmfb_split_row(startptr, numcols, colparity);
		jpc_ns_fwdlift_row(startptr, numcols, colparity);
		startptr += stride;
	}

	return 0;

}
#endif
/* s.kang end */

