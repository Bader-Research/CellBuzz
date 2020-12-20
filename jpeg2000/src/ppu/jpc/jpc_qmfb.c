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
 * $Id: jpc_qmfb.c,v 1.7 2008/02/06 20:37:15 lrlemini Exp $
 */

/******************************************************************************\
*
\******************************************************************************/

#undef WT_LENONE /* This is not needed due to normalization. */
#define WT_DOSCALE

/******************************************************************************\
* Includes.
\******************************************************************************/

//#define PERFORMANCE_MEASURE
#ifdef PERFORMANCE_MEASURE
//REMOVE
#include <sys/time.h>
#endif

#include <assert.h>
/* s.kang start */
#include <sched.h>
/* s.kang end */

#include "jasper/jas_fix.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_debug.h"
/* s.kang end */

#include "jpc_qmfb.h"
#include "jpc_tsfb.h"
#include "jpc_math.h"
/* s.kang start */
#include "jpc_spu_ctrl.h"

#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
*
\******************************************************************************/

#define QMFB_SPLITBUFSIZE	4096
#define	QMFB_JOINBUFSIZE	4096

int jpc_ft_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride);
int jpc_ft_synthesize(int *a, int xstart, int ystart, int width, int height,
  int stride);

int jpc_ns_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride);
int jpc_ns_synthesize(jpc_fix_t *a, int xstart, int ystart, int width,
  int height, int stride);

void jpc_ft_fwdlift_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity);
void jpc_ft_invlift_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity);

void jpc_ns_fwdlift_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
  int parity);
void jpc_ns_invlift_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
  int parity);

void jpc_qmfb_split_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
  int parity);
void jpc_qmfb_join_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
  int parity);

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

/******************************************************************************\
* generic
\******************************************************************************/

void jpc_qmfb_split_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity)
{

	int bufsize = JPC_CEILDIVPOW2(numrows, 1);
#if !defined(HAVE_VLA)
	jpc_fix_t splitbuf[QMFB_SPLITBUFSIZE * JPC_QMFB_COLGRPSIZE];
#else
	jpc_fix_t splitbuf[bufsize * numcols];
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
			for (i = 0; i < numcols; ++i) {
				*dstptr2 = *srcptr2;
				++dstptr2;
				++srcptr2;
			}
			dstptr += numcols;
			srcptr += stride << 1;
		}
		/* Copy the appropriate samples into the lowpass channel. */
		dstptr = &a[(1 - parity) * stride];
		srcptr = &a[(2 - parity) * stride];
		n = numrows - m - (!parity);
		while (n-- > 0) {
			dstptr2 = dstptr;
			srcptr2 = srcptr;
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
				*dstptr2 = *srcptr2;
				++dstptr2;
				++srcptr2;
			}
			dstptr += stride;
			srcptr += numcols;
		}
	}

#if !defined(HAVE_VLA)
	/* If the split buffer was allocated on the heap, free this memory. */
	if (buf != splitbuf) {
		jas_free(buf);
	}
#endif

}

void jpc_qmfb_join_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity)
{

	int bufsize = JPC_CEILDIVPOW2(numrows, 1);
#if !defined(HAVE_VLA)
	jpc_fix_t joinbuf[QMFB_JOINBUFSIZE * JPC_QMFB_COLGRPSIZE];
#else
	jpc_fix_t joinbuf[bufsize * numcols];
#endif
	jpc_fix_t *buf = joinbuf;
	jpc_fix_t *srcptr;
	jpc_fix_t *dstptr;
	register jpc_fix_t *srcptr2;
	register jpc_fix_t *dstptr2;
	register int n;
	register int i;
	int hstartcol;

#if !defined(HAVE_VLA)
	/* Allocate memory for the join buffer from the heap. */
	if (bufsize > QMFB_JOINBUFSIZE) {
		if (!(buf = jas_malloc(bufsize * numcols * sizeof(jpc_fix_t)))) {
			/* We have no choice but to commit suicide. */
			abort();
		}
	}
#endif

	hstartcol = (numrows + 1 - parity) >> 1;

	/* Save the samples from the lowpass channel. */
	n = hstartcol;
	srcptr = &a[0];
	dstptr = buf;
	while (n-- > 0) {
		dstptr2 = dstptr;
		srcptr2 = srcptr;
		for (i = 0; i < numcols; ++i) {
			*dstptr2 = *srcptr2;
			++dstptr2;
			++srcptr2;
		}
		srcptr += stride;
		dstptr += numcols;
	}
	/* Copy the samples from the highpass channel into place. */
	srcptr = &a[hstartcol * stride];
	dstptr = &a[(1 - parity) * stride];
	n = numrows - hstartcol;
	while (n-- > 0) {
		dstptr2 = dstptr;
		srcptr2 = srcptr;
		for (i = 0; i < numcols; ++i) {
			*dstptr2 = *srcptr2;
			++dstptr2;
			++srcptr2;
		}
		dstptr += 2 * stride;
		srcptr += stride;
	}
	/* Copy the samples from the lowpass channel into place. */
	srcptr = buf;
	dstptr = &a[parity * stride];
	n = hstartcol;
	while (n-- > 0) {
		dstptr2 = dstptr;
		srcptr2 = srcptr;
		for (i = 0; i < numcols; ++i) {
			*dstptr2 = *srcptr2;
			++dstptr2;
			++srcptr2;
		}
		dstptr += 2 * stride;
		srcptr += numcols;
	}

#if !defined(HAVE_VLA)
	/* If the join buffer was allocated on the heap, free this memory. */
	if (buf != joinbuf) {
		jas_free(buf);
	}
#endif

}

/******************************************************************************\
* 5/3 transform
\******************************************************************************/

void jpc_ft_fwdlift_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
				lptr2[0] += (hptr2[0] + 1) >> 1;
				++lptr2;
				++hptr2;
			}
		}

	} else {

		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < numcols; ++i) {
				lptr2[0] <<= 1;
				++lptr2;
			}
		}

	}

}

void jpc_ft_invlift_colres(jpc_fix_t *a, int numrows, int numcols, int stride,
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
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] -= (hptr2[0] + 1) >> 1;
				++lptr2;
				++hptr2;
			}
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] -= (hptr2[0] + hptr2[stride] + 2) >> 2;
				++lptr2;
				++hptr2;
			}
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] -= (hptr2[0] + 1) >> 1;
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] += lptr2[0];
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] += (lptr2[0] + lptr2[stride]) >> 1;
				++lptr2;
				++hptr2;
			}
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] += lptr2[0];
				++lptr2;
				++hptr2;
			}
		}

	} else {

		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < numcols; ++i) {
				lptr2[0] >>= 1;
				++lptr2;
			}
		}

	}

}

int jpc_ft_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride)
{
	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;

	int maxcols;
	jpc_fix_t *startptr;
	int i;
/* s.kang start */
	int j;
/* s.kang end */

/* s.kang start */
	int numrows_per_spu;
	int numrows_rem;
	int numcolgrps_per_spu;
	int numcolgrps_rem;
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
struct timeval tv0;
struct timeval tv1;
struct timeval tv2;
struct timeval tv3;
struct timeval tv4;
#endif

#if 1
#endif

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv0, NULL );
#endif

	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;

/* s.kang start */
#if 1
	numcolgrps_per_spu = ( ( maxcols / JPC_QMFB_COLGRPSIZE ) + g_num_spus - 1 ) / g_num_spus;
	numcolgrps_rem = maxcols / JPC_QMFB_COLGRPSIZE;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_fwd_dwt_colgrp_cmd_data[i].startaddr = ( unsigned int )( &a[numcolgrps_per_spu * JPC_QMFB_COLGRPSIZE * i] );
		a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr = ( unsigned int )jas_malloc_align( JPC_CEILDIVPOW2( numrows, 1 ) * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		if( a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr == 0 ) {
			jas_eprintf( "[jpc_qmfb.c:jpc_ft_analyze()] jas_malloc_align failure\n" );
			for( j = 0 ; j < i ; j++ ) {
				jas_free_align( ( void* )( a_fwd_dwt_colgrp_cmd_data[j].splitbufaddr ) );
			}
			return -1;
		}
		a_fwd_dwt_colgrp_cmd_data[i].numrows = numrows;
		if( numcolgrps_rem > numcolgrps_per_spu ) {
			a_fwd_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_per_spu;
			numcolgrps_rem -= numcolgrps_per_spu;
		}
		else {
			a_fwd_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_rem;
			numcolgrps_rem = 0;
		}
		a_fwd_dwt_colgrp_cmd_data[i].stride = stride;
		a_fwd_dwt_colgrp_cmd_data[i].rowparity = rowparity;
		a_fwd_dwt_colgrp_cmd_data[i].lossless = 1;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_FWD_DWT_COLGRP );
	}

	/* overlap residual part computation with column group part computation in SPUs */

	startptr = &a[0] + maxcols;
	if (maxcols < numcols) {
		jpc_qmfb_split_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
		jpc_ft_fwdlift_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv1, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ft_analyze()] jpc_wait_for_spus() failure\n" );
		exit( 1 );/* we can't free the allocated memory, so exit */
	}

	/* free allocated memory */

	for( i = 0 ; i < g_num_spus ; i++ ) {
		jas_free_align( ( void* )( a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr ) );
	}
#else
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
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv2, NULL );
#endif

/* s.kang start */
#if 1
	numrows_per_spu = ( numrows + g_num_spus - 1 ) / g_num_spus;
	numrows_rem = numrows;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_fwd_dwt_row_cmd_data[i].startaddr = ( unsigned int )( &a[numrows_per_spu * i * stride] );
		if( numrows_rem > numrows_per_spu ) {
			a_fwd_dwt_row_cmd_data[i].numrows = numrows_per_spu;
			numrows_rem -= numrows_per_spu;
		}
		else {
			a_fwd_dwt_row_cmd_data[i].numrows = numrows_rem;
			numrows_rem = 0;
		}
		a_fwd_dwt_row_cmd_data[i].numcols = numcols;
		a_fwd_dwt_row_cmd_data[i].stride = stride;
		a_fwd_dwt_row_cmd_data[i].colparity = colparity;
		a_fwd_dwt_row_cmd_data[i].lossless = 1;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_FWD_DWT_ROW );
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv3, NULL );
#endif
	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ft_analyze()] jpc_wait_for_spus() failure\n" );
		return -1;
	}
#else
	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_qmfb_split_row(startptr, numcols, colparity);
		jpc_ft_fwdlift_row(startptr, numcols, colparity);
		startptr += stride;
	}
#endif
/* s.kang end  */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv4, NULL );
#endif
#ifdef PERFORMANCE_MEASURE
//REMOVE
printf( "colgrp=%ld, %ld\n", ( tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec ) / 1000000, tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec );
printf( "colgrp wait=%ld, %ld\n", ( tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec ) / 1000000, tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec );
printf( "row=%ld, %ld\n", ( tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec ) / 1000000, tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec );
printf( "row wait=%ld, %ld\n", ( tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec ) / 1000000, tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec );
#endif

	return 0;
}

int jpc_ft_synthesize(int *a, int xstart, int ystart, int width, int height,
  int stride)
{
	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;

	int maxcols;
	jpc_fix_t *startptr;
	int i;
/* s.kang start */
	int j;
/* s.kang end */

/* s.kang start */
	int numrows_per_spu;
	int numrows_rem;
	int numcolgrps_per_spu;
	int numcolgrps_rem;
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
struct timeval tv0;
struct timeval tv1;
struct timeval tv2;
struct timeval tv3;
struct timeval tv4;
#endif

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv0, NULL );
#endif

/* s.kang start */
#if 1
	numrows_per_spu = ( numrows + g_num_spus - 1 ) / g_num_spus;
	numrows_rem = numrows;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_inv_dwt_row_cmd_data[i].startaddr = ( unsigned int )( &a[numrows_per_spu * i * stride] );
		if( numrows_rem > numrows_per_spu ) {
			a_inv_dwt_row_cmd_data[i].numrows = numrows_per_spu;
			numrows_rem -= numrows_per_spu;
		}
		else {
			a_inv_dwt_row_cmd_data[i].numrows = numrows_rem;
			numrows_rem = 0;
		}
		a_inv_dwt_row_cmd_data[i].numcols = numcols;
		a_inv_dwt_row_cmd_data[i].stride = stride;
		a_inv_dwt_row_cmd_data[i].colparity = colparity;
		a_inv_dwt_row_cmd_data[i].lossless = 1;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_INV_DWT_ROW );
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv1, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ft_synthesize()] jpc_wait_for_spus() failure\n" );
		return -1;
	}
#else
	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_ft_invlift_row(startptr, numcols, colparity);
		jpc_qmfb_join_row(startptr, numcols, colparity);
		startptr += stride;
	}
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv2, NULL );
#endif
	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;

/* s.kang start */
#if 1
	numcolgrps_per_spu = ( ( maxcols / JPC_QMFB_COLGRPSIZE ) + g_num_spus - 1 ) / g_num_spus;
	numcolgrps_rem = maxcols / JPC_QMFB_COLGRPSIZE;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_inv_dwt_colgrp_cmd_data[i].startaddr = ( unsigned int )( &a[numcolgrps_per_spu * JPC_QMFB_COLGRPSIZE * i] );
		a_inv_dwt_colgrp_cmd_data[i].joinbufaddr = ( unsigned int )jas_malloc_align( JPC_CEILDIVPOW2( numrows, 1 ) * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		if( a_inv_dwt_colgrp_cmd_data[i].joinbufaddr == 0 ) {
			jas_eprintf( "[jpc_qmfb.c:jpc_ft_synthesize()] jas_malloc_align failure\n" );
			for( j = 0 ; j < i ; j++ ) {
				jas_free_align( ( void* )( a_inv_dwt_colgrp_cmd_data[j].joinbufaddr ) );
			}
			return -1;
		}
		a_inv_dwt_colgrp_cmd_data[i].numrows = numrows;
		if( numcolgrps_rem > numcolgrps_per_spu ) {
			a_inv_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_per_spu;
			numcolgrps_rem -= numcolgrps_per_spu;
		}
		else {
			a_inv_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_rem;
			numcolgrps_rem = 0;
		}
		a_inv_dwt_colgrp_cmd_data[i].stride = stride;
		a_inv_dwt_colgrp_cmd_data[i].rowparity = rowparity;
		a_inv_dwt_colgrp_cmd_data[i].lossless = 1;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_INV_DWT_COLGRP );
	}

	/* overlap residual part computation with column group part computation in SPUs */

	startptr = &a[0] + maxcols;
	if (maxcols < numcols) {
		jpc_ft_invlift_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
		jpc_qmfb_join_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv3, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ft_synthesize()] jpc_wait_for_spus() failure\n" );
		exit( 1 );/* we can't free the allocated memory, so exit */
	}

	/* free allocated memory */

	for( i = 0 ; i < g_num_spus ; i++ ) {
		jas_free_align( ( void* )( a_inv_dwt_colgrp_cmd_data[i].joinbufaddr ) );
	}
#else
	startptr = &a[0];
	for (i = 0; i < maxcols; i += JPC_QMFB_COLGRPSIZE) {
		jpc_ft_invlift_colgrp(startptr, numrows, stride, rowparity);
		jpc_qmfb_join_colgrp(startptr, numrows, stride, rowparity);
		startptr += JPC_QMFB_COLGRPSIZE;
	}

	startptr = &a[0] + maxcols;
	if (maxcols < numcols) {
		jpc_ft_invlift_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
		jpc_qmfb_join_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
	}
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv4, NULL );
#endif
#ifdef PERFORMANCE_MEASURE
//REMOVE
printf( "row=%ld, %ld\n", ( tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec ) / 1000000, tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec );
printf( "row wait=%ld, %ld\n", ( tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec ) / 1000000, tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec );
printf( "colgrp=%ld, %ld\n", ( tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec ) / 1000000, tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec );
printf( "colgrp wait=%ld, %ld\n", ( tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec ) / 1000000, tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec );
#endif

	return 0;
}

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
void jpc_ns_fwdlift_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity)
{

	jpc_fix_t *lptr;
	jpc_fix_t *hptr;
#ifdef FLOAT_MODE
	float* f_lptr2;
	float* f_hptr2;
#else
	jpc_fix_t *lptr2;
	jpc_fix_t *hptr2;
#endif
	jpc_fix_t gain1;
	jpc_fix_t gain2;
#ifdef FLOAT_MODE
	float f_gain1;
	float f_gain2;
#endif
	int n;
	int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	if (numrows > 1) {
		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
                gain1 = jpc_dbltofix( ALPHA );
                gain2 = jpc_dbltofix( 2.0 * ALPHA );
#ifdef FLOAT_MODE
                f_gain1 = ( float )ALPHA;
                f_gain2 = ( float )( 2.0 * ALPHA );
#endif
		if (parity) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain2 * f_lptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq( hptr2[0], jpc_fix_mul( gain2, lptr2[0] ) );
				++hptr2;
				++lptr2;
			}
#endif
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain1 * ( f_lptr2[0] + f_lptr2[stride] );
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(gain1, jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain2 * f_lptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq( hptr2[0], jpc_fix_mul( gain2, lptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
                gain1 = jpc_dbltofix( BETA );
                gain2 = jpc_dbltofix( 2.0 * BETA );
#ifdef FLOAT_MODE
                f_gain1 = ( float )BETA;
                f_gain2 = ( float )( 2.0 * BETA );
#endif
		if (!parity) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain2 * f_hptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul( gain2, hptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain1 * ( f_hptr2[0] + f_hptr2[stride] );
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(gain1, jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain2 * f_hptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul( gain2, hptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the third lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
                gain1 = jpc_dbltofix( GAMMA );
                gain2 = jpc_dbltofix( 2.0 * GAMMA );
#ifdef FLOAT_MODE
                f_gain1 = ( float )GAMMA;
                f_gain2 = ( float )( 2.0 * GAMMA );
#endif
		if (parity) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain2 * f_lptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq( hptr2[0], jpc_fix_mul( gain2, lptr2[0] ) );
				++hptr2;
				++lptr2;
			}
#endif
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain1 * ( f_lptr2[0] + f_lptr2[stride] );
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(hptr2[0], jpc_fix_mul(gain1, jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] += f_gain2 * f_lptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq( hptr2[0], jpc_fix_mul( gain2, lptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the fourth lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
                gain1 = jpc_dbltofix( DELTA );
                gain2 = jpc_dbltofix( 2.0 * DELTA );
#ifdef FLOAT_MODE
                f_gain1 = ( float )DELTA;
                f_gain2 = ( float )( 2.0 * DELTA );
#endif
		if (!parity) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain2 * f_hptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul( gain2, hptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain1 * ( f_hptr2[0] + f_hptr2[stride] );
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul(gain1, jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] += f_gain2 * f_hptr2[0];
                                f_lptr2++;
                                f_hptr2++;
                        }
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_pluseq(lptr2[0], jpc_fix_mul( gain2, hptr2[0] ) );
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		lptr = &a[0];
		n = llen;
                gain1 = jpc_dbltofix( LGAIN );
#ifdef FLOAT_MODE
                f_gain1 = ( float )LGAIN;
#endif
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_lptr2 = ( float* )lptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_lptr2[0] *= f_gain1;
                                f_lptr2++;
                        }
#else
			lptr2 = lptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] = jpc_fix_mul(lptr2[0], gain1);
				++lptr2;
			}
#endif
			lptr += stride;
		}
		hptr = &a[llen * stride];
		n = numrows - llen;
                gain1 = jpc_dbltofix( HGAIN );
#ifdef FLOAT_MODE
                f_gain1 = ( float )HGAIN;
#endif
		while (n-- > 0) {
#ifdef FLOAT_MODE
                        f_hptr2 = ( float* )hptr;
                        for( i = 0 ; i < numcols ; i++ ) {
                                f_hptr2[0] *= f_gain1;
                                f_hptr2++;
                        }
#else
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] = jpc_fix_mul(hptr2[0], jpc_dbltofix(HGAIN));
				++hptr2;
			}
#endif
			hptr += stride;
		}
#endif

	} else {

#if defined(WT_LENONE)
		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < numcols; ++i) {
				lptr2[0] <<= 1;
				++lptr2;
			}
		}
#endif

	}

}
#else
void jpc_ns_fwdlift_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity)
{

	jpc_fix_t *lptr;
	jpc_fix_t *hptr;
	jpc_fix_t *lptr2;
	jpc_fix_t *hptr2;
	int n;
	int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	if (numrows > 1) {

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
				lptr2[0] = jpc_fix_mul(lptr2[0], jpc_dbltofix(LGAIN));
				++lptr2;
			}
			lptr += stride;
		}
		hptr = &a[llen * stride];
		n = numrows - llen;
		while (n-- > 0) {
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
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
			for (i = 0; i < numcols; ++i) {
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
void jpc_ns_invlift_colres( jpc_fix_t* a, int numrows, int numcols, int stride, int parity )
{
	jpc_fix_t *lptr;
	jpc_fix_t *hptr;
#ifdef FLOAT_MODE
	float* f_lptr2;
	float* f_hptr2;
#else
	jpc_fix_t *lptr2;
	jpc_fix_t *hptr2;
#endif
	jpc_fix_t gain1;
	jpc_fix_t gain2;
#ifdef FLOAT_MODE
	float f_gain1;
	float f_gain2;
#endif
	int n;
	int i;
	int llen;

	llen = (numrows + 1 - parity) >> 1;

	if (numrows > 1) {

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		lptr = &a[0];
		n = llen;
		gain1 = jpc_dbltofix( 1.0 / LGAIN );
#ifdef FLOAT_MODE
		f_gain1 = ( float )( 1.0 / LGAIN );
#endif
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] * f_gain1;
				f_lptr2++;
			}
#else
			lptr2 = lptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] = jpc_fix_mul(lptr2[0], gain1);
				++lptr2;
			}
#endif
			lptr += stride;
		}
		hptr = &a[llen * stride];
		n = numrows - llen;
		gain1 = jpc_dbltofix( 1.0 / HGAIN );
#ifdef FLOAT_MODE
		f_gain1 = ( float )( 1.0 / HGAIN );
#endif
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] * f_gain1;
				f_hptr2++;
			}
#else
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] = jpc_fix_mul(hptr2[0], gain1);
				++hptr2;
			}
#endif
			hptr += stride;
		}
#endif

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		gain1 = jpc_dbltofix( DELTA );
		gain2 = jpc_dbltofix( 2.0 * DELTA );
#ifdef FLOAT_MODE
		f_gain1 = ( float )DELTA;
		f_gain2 = ( float )( 2.0 * DELTA );
#endif
		if (!parity) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain2 * f_hptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain2, hptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain1 * ( f_hptr2[0] + f_hptr2[stride] );
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain1, jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain2 * f_hptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain2, hptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		gain1 = jpc_dbltofix( GAMMA );
		gain2 = jpc_dbltofix( 2.0 * GAMMA );
#ifdef FLOAT_MODE
		f_gain1 = ( float )GAMMA;
		f_gain2 = ( float )( 2.0 * GAMMA );
#endif
		if (parity) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain2 * f_lptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain2, lptr2[0]));
				++hptr2;
				++lptr2;
			}
#endif
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain1 * ( f_lptr2[0] + f_lptr2[stride] );
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain1, jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain2 * f_lptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain2, lptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the third lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		gain1 = jpc_dbltofix( BETA );
		gain2 = jpc_dbltofix( 2.0 * BETA );
#ifdef FLOAT_MODE
		f_gain1 = ( float )BETA;
		f_gain2 = ( float )( 2.0 * BETA );
#endif
		if (!parity) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain2 * f_hptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain2, hptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain1 * ( f_hptr2[0] + f_hptr2[stride] );
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain1, jpc_fix_add(hptr2[0], hptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			lptr += stride;
			hptr += stride;
		}
		if (parity != (numrows & 1)) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] - f_gain2 * f_hptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(gain2, hptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
		}

		/* Apply the fourth lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		gain1 = jpc_dbltofix( ALPHA );
		gain2 = jpc_dbltofix( 2.0 * ALPHA );
#ifdef FLOAT_MODE
		f_gain1 = ( float )ALPHA;
		f_gain2 = ( float )( 2.0 * ALPHA );
#endif
		if (parity) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain2 * f_lptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain2, lptr2[0]));
				++hptr2;
				++lptr2;
			}
#endif
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain1 * ( f_lptr2[0] + f_lptr2[stride] );
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain1, jpc_fix_add(lptr2[0], lptr2[stride])));
				++lptr2;
				++hptr2;
			}
#endif
			hptr += stride;
			lptr += stride;
		}
		if (parity == (numrows & 1)) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )lptr;
			f_hptr2 = ( float* )hptr;
			for( i = 0 ; i < numcols ; i++ ) {
				f_hptr2[0] = f_hptr2[0] - f_gain2 * f_lptr2[0];
				f_lptr2++;
				f_hptr2++;
			}
#else
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(gain2, lptr2[0]));
				++lptr2;
				++hptr2;
			}
#endif
		}

	} else {
#if defined(WT_LENONE)
		if (parity) {
#ifdef FLOAT_MODE
			f_lptr2 = ( float* )&a[0];
			for( i = 0 ; i < numcols ; i++ ) {
				f_lptr2[0] = f_lptr2[0] / 2.0f;
				f_lptr2++;
			}
#else
			lptr2 = &a[0];
			for (i = 0; i < numcols; ++i) {
				lptr2[0] >>= 1;
				++lptr2;
			}
#endif
		}
#endif

	}
}
#else
void jpc_ns_invlift_colres(jpc_fix_t *a, int numrows, int numcols,
  int stride, int parity)
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

		/* Apply the scaling step. */
#if defined(WT_DOSCALE)
		lptr = &a[0];
		n = llen;
		while (n-- > 0) {
			lptr2 = lptr;
			for (i = 0; i < numcols; ++i) {
				lptr2[0] = jpc_fix_mul(lptr2[0], jpc_dbltofix(1.0 / LGAIN));
				++lptr2;
			}
			lptr += stride;
		}
		hptr = &a[llen * stride];
		n = numrows - llen;
		while (n-- > 0) {
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				hptr2[0] = jpc_fix_mul(hptr2[0], jpc_dbltofix(1.0 / HGAIN));
				++hptr2;
			}
			hptr += stride;
		}
#endif

		/* Apply the first lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  DELTA), hptr2[0]));
				++lptr2;
				++hptr2;
			}
			lptr += stride;
		}
		n = llen - (!parity) - (parity != (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(DELTA),
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
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  DELTA), hptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the second lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  GAMMA), lptr2[0]));
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(GAMMA),
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
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  GAMMA), lptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the third lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (!parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
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
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(BETA),
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
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(lptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 * BETA),
				  hptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

		/* Apply the fourth lifting step. */
		lptr = &a[0];
		hptr = &a[llen * stride];
		if (parity) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  ALPHA), lptr2[0]));
				++hptr2;
				++lptr2;
			}
			hptr += stride;
		}
		n = numrows - llen - parity - (parity == (numrows & 1));
		while (n-- > 0) {
			lptr2 = lptr;
			hptr2 = hptr;
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(ALPHA),
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
			for (i = 0; i < numcols; ++i) {
				jpc_fix_minuseq(hptr2[0], jpc_fix_mul(jpc_dbltofix(2.0 *
				  ALPHA), lptr2[0]));
				++lptr2;
				++hptr2;
			}
		}

	} else {

#if defined(WT_LENONE)
		if (parity) {
			lptr2 = &a[0];
			for (i = 0; i < numcols; ++i) {
				lptr2[0] >>= 1;
				++lptr2;
			}
		}
#endif

	}

}
#endif
/* s.kang end */

int jpc_ns_analyze(jpc_fix_t *a, int xstart, int ystart, int width, int height,
  int stride)
{

	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;

	int maxcols;
	jpc_fix_t *startptr;
	int i;
/* s.kang start */
	int j;
/* s.kang end */

/* s.kang start */
	int numrows_per_spu;
	int numrows_rem;
	int numcolgrps_per_spu;
	int numcolgrps_rem;
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
struct timeval tv0;
struct timeval tv1;
struct timeval tv2;
struct timeval tv3;
struct timeval tv4;
#endif

#if 1
#endif

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv0, NULL );
#endif

	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;

/* s.kang start */
#if 1
	numcolgrps_per_spu = ( ( maxcols / JPC_QMFB_COLGRPSIZE ) + g_num_spus - 1 ) / g_num_spus;
	numcolgrps_rem = maxcols / JPC_QMFB_COLGRPSIZE;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_fwd_dwt_colgrp_cmd_data[i].startaddr = ( unsigned int )( &a[numcolgrps_per_spu * JPC_QMFB_COLGRPSIZE * i] );
		a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr = ( unsigned int )jas_malloc_align( JPC_CEILDIVPOW2( numrows, 1 ) * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		if( a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr == 0 ) {
			jas_eprintf( "[jpc_qmfb.c:jpc_ns_synthesize()] jas_malloc_align failure\n" );
			for( j = 0 ; j < i ; j++ ) {
				jas_free_align( ( void* )( a_fwd_dwt_colgrp_cmd_data[j].splitbufaddr ) );
			}
			return -1;
		}
		a_fwd_dwt_colgrp_cmd_data[i].numrows = numrows;
		if( numcolgrps_rem > numcolgrps_per_spu ) {
			a_fwd_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_per_spu;
			numcolgrps_rem -= numcolgrps_per_spu;
		}
		else {
			a_fwd_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_rem;
			numcolgrps_rem = 0;
		}
		a_fwd_dwt_colgrp_cmd_data[i].stride = stride;
		a_fwd_dwt_colgrp_cmd_data[i].rowparity = rowparity;
		a_fwd_dwt_colgrp_cmd_data[i].lossless = 0;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_FWD_DWT_COLGRP );
	}

	/* overlap residual part computation with column group part computation in SPUs */

	startptr = &a[0] + maxcols;
	if (maxcols < numcols) {
		jpc_qmfb_split_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
		jpc_ns_fwdlift_colres(startptr, numrows, numcols - maxcols, stride, rowparity);
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv1, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ns_analyze()] jpc_wait_for_spus() failure\n" );
		exit( 1 );/* we can't free the allocated memory, so exit */
	}

	/* free allocated memory */

	for( i = 0 ; i < g_num_spus ; i++ ) {
		jas_free_align( ( void* )( a_fwd_dwt_colgrp_cmd_data[i].splitbufaddr ) );
	}
#else
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
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv2, NULL );
#endif

/* s.kang start */
#if 1
	numrows_per_spu = ( numrows + g_num_spus - 1 ) / g_num_spus;
	numrows_rem = numrows;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_fwd_dwt_row_cmd_data[i].startaddr = ( unsigned int )( &a[numrows_per_spu * i * stride] );
		if( numrows_rem > numrows_per_spu ) {
			a_fwd_dwt_row_cmd_data[i].numrows = numrows_per_spu;
			numrows_rem -= numrows_per_spu;
		}
		else {
			a_fwd_dwt_row_cmd_data[i].numrows = numrows_rem;
			numrows_rem = 0;
		}
		a_fwd_dwt_row_cmd_data[i].numcols = numcols;
		a_fwd_dwt_row_cmd_data[i].stride = stride;
		a_fwd_dwt_row_cmd_data[i].colparity = colparity;
		a_fwd_dwt_row_cmd_data[i].lossless = 0;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_FWD_DWT_ROW );
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv3, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ns_analyze()] jpc_wait_for_spus() failure\n" );
		return -1;
	}
#else
	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_qmfb_split_row(startptr, numcols, colparity);
		jpc_ns_fwdlift_row(startptr, numcols, colparity);
		startptr += stride;
	}
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv4, NULL );
#endif
#ifdef PERFORMANCE_MEASURE
//REMOVE
printf( "colgrp=%ld, %ld\n", ( tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec ) / 1000000, tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec );
printf( "colgrp wait=%ld, %ld\n", ( tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec ) / 1000000, tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec );
printf( "row=%ld, %ld\n", ( tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec ) / 1000000, tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec );
printf( "row wait=%ld, %ld\n", ( tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec ) / 1000000, tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec );
#endif

	return 0;

}

int jpc_ns_synthesize(jpc_fix_t *a, int xstart, int ystart, int width,
  int height, int stride)
{

	int numrows = height;
	int numcols = width;
	int rowparity = ystart & 1;
	int colparity = xstart & 1;

	int maxcols;
	jpc_fix_t *startptr;
	int i;
/* s.kang start */
	int j;
/* s.kang end */

/* s.kang start */
	int numrows_per_spu;
	int numrows_rem;
	int numcolgrps_per_spu;
	int numcolgrps_rem;
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
struct timeval tv0;
struct timeval tv1;
struct timeval tv2;
struct timeval tv3;
struct timeval tv4;
#endif

#if 1
#endif

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv0, NULL );
#endif

/* s.kang start */
#if 1
	numrows_per_spu = ( numrows + g_num_spus - 1 ) / g_num_spus;
	numrows_rem = numrows;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_inv_dwt_row_cmd_data[i].startaddr = ( unsigned int )( &a[numrows_per_spu * i * stride] );
		if( numrows_rem > numrows_per_spu ) {
			a_inv_dwt_row_cmd_data[i].numrows = numrows_per_spu;
			numrows_rem -= numrows_per_spu;
		}
		else {
			a_inv_dwt_row_cmd_data[i].numrows = numrows_rem;
			numrows_rem = 0;
		}
		a_inv_dwt_row_cmd_data[i].numcols = numcols;
		a_inv_dwt_row_cmd_data[i].stride = stride;
		a_inv_dwt_row_cmd_data[i].colparity = colparity;
		a_inv_dwt_row_cmd_data[i].lossless = 0;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_INV_DWT_ROW );
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv1, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ns_synthesize()] jpc_wait_for_spus() failure\n" );
		return -1;
	}
#else
	startptr = &a[0];
	for (i = 0; i < numrows; ++i) {
		jpc_ns_invlift_row(startptr, numcols, colparity);
		jpc_qmfb_join_row(startptr, numcols, colparity);
		startptr += stride;
	}
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv2, NULL );
#endif
	maxcols = (numcols / JPC_QMFB_COLGRPSIZE) * JPC_QMFB_COLGRPSIZE;
/* s.kang start */
#if 1
	numcolgrps_per_spu = ( ( maxcols / JPC_QMFB_COLGRPSIZE ) + g_num_spus - 1 ) / g_num_spus;
	numcolgrps_rem = maxcols / JPC_QMFB_COLGRPSIZE;
	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set command data */

		a_status[i].status = PRC_RUNNING;
		a_inv_dwt_colgrp_cmd_data[i].startaddr = ( unsigned int )( &a[numcolgrps_per_spu * JPC_QMFB_COLGRPSIZE * i] );
		a_inv_dwt_colgrp_cmd_data[i].joinbufaddr = ( unsigned int )jas_malloc_align( JPC_CEILDIVPOW2( numrows, 1 ) * JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ), CACHE_LINE_OFFSET_BITS );
		if( a_inv_dwt_colgrp_cmd_data[i].joinbufaddr == 0 ) {
			jas_eprintf( "[jpc_qmfb.c:jpc_ns_synthesize()] jas_malloc_align failure\n" );
			for( j = 0 ; j < i ; j++ ) {
				jas_free_align( ( void* )( a_inv_dwt_colgrp_cmd_data[j].joinbufaddr ) );
			}
			return -1;
		}
		a_inv_dwt_colgrp_cmd_data[i].numrows = numrows;
		if( numcolgrps_rem > numcolgrps_per_spu ) {
			a_inv_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_per_spu;
			numcolgrps_rem -= numcolgrps_per_spu;
		}
		else {
			a_inv_dwt_colgrp_cmd_data[i].numcolgrps = numcolgrps_rem;
			numcolgrps_rem = 0;
		}
		a_inv_dwt_colgrp_cmd_data[i].stride = stride;
		a_inv_dwt_colgrp_cmd_data[i].rowparity = rowparity;
		a_inv_dwt_colgrp_cmd_data[i].lossless = 0;

		/* send command to SPUs */

		jpc_send_cmd2spu( i, CMD_INV_DWT_COLGRP );
	}

	/* overlap residual part computation with column group part computation in SPUs */

	startptr = &a[0] + maxcols;
	if (maxcols < numcols) {
		jpc_ns_invlift_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
		jpc_qmfb_join_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
	}

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv3, NULL );
#endif

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_qmfb.c:jpc_ns_synthesize()] jpc_wait_for_spus() failure\n" );
		exit( 1 );/* we can't free the allocated memory, so exit */
	}

	/* free allocated memory */

	for( i = 0 ; i < g_num_spus ; i++ ) {
		jas_free_align( ( void* )( a_inv_dwt_colgrp_cmd_data[i].joinbufaddr ) );
	}
#else
	startptr = &a[0];
	for (i = 0; i < maxcols; i += JPC_QMFB_COLGRPSIZE) {
		jpc_ns_invlift_colgrp(startptr, numrows, stride, rowparity);
		jpc_qmfb_join_colgrp(startptr, numrows, stride, rowparity);
		startptr += JPC_QMFB_COLGRPSIZE;
	}
	if (maxcols < numcols) {
		jpc_ns_invlift_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
		jpc_qmfb_join_colres(startptr, numrows, numcols - maxcols, stride,
		  rowparity);
	}
#endif
/* s.kang end */

#ifdef PERFORMANCE_MEASURE
//REMOVE
gettimeofday( &tv4, NULL );
#endif
#ifdef PERFORMANCE_MEASURE
//REMOVE
printf( "row=%ld, %ld\n", ( tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec ) / 1000000, tv1.tv_sec * 1000000 + tv1.tv_usec - tv0.tv_sec * 1000000 - tv0.tv_usec );
printf( "row wait=%ld, %ld\n", ( tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec ) / 1000000, tv2.tv_sec * 1000000 + tv2.tv_usec - tv1.tv_sec * 1000000 - tv1.tv_usec );
printf( "colgrp=%ld, %ld\n", ( tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec ) / 1000000, tv3.tv_sec * 1000000 + tv3.tv_usec - tv2.tv_sec * 1000000 - tv2.tv_usec );
printf( "colgrp wait=%ld, %ld\n", ( tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec ) / 1000000, tv4.tv_sec * 1000000 + tv4.tv_usec - tv3.tv_sec * 1000000 - tv3.tv_usec );
#endif

	return 0;

}

