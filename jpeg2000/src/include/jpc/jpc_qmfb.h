/*
 * Copyright (c) 1999-2000 Image Power, Inc. and the University of
 *   British Columbia.
 * Copyright (c) 2001-2004 Michael David Adams.
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
 * Quadrature Mirror-Image Filter Bank (QMFB) Routines
 *
 * $Id: jpc_qmfb.h,v 1.6 2008/06/23 16:01:33 lrlemini Exp $
 */

#ifndef JPC_QMFB_H
#define JPC_QMFB_H

/******************************************************************************\
* Includes.
\******************************************************************************/

#include "jasper/jas_seq.h"

/******************************************************************************\
* Constants.
\******************************************************************************/

/* QMFB IDs. */
#define	JPC_QMFB1D_FT	1	/* 5/3 */
#define	JPC_QMFB1D_NS	2	/* 9/7 */

/* s.kang start */
/* constants, only for jpc_qmfb_analyze.c and jpc_qmfb_synthesize.c */
#define ALPHA (-1.586134342059924)
#define BETA (-0.052980118572961)
#define GAMMA (0.882911075530934)
#define DELTA (0.443506852043971)
#define LGAIN (1.0 / 1.23017410558578)
#define HGAIN (1.0 / 1.62578613134411)

/* constants, only for jpc_qmfb_analyze.c and jpc_qmfb_synthesize.c */
#define QMFB_SPLITBUFSIZE       4096
#define QMFB_JOINBUFSIZE        4096
/* s.kang end */

/******************************************************************************\
* Types.
\******************************************************************************/

/******************************************************************************\
* Functions.
\******************************************************************************/

#if !defined(JPC_QMFB_COLGRPSIZE)
/* The number of columns to group together during the vertical processing
stage of the wavelet transform. */
/* The default value for this parameter is probably not optimal for
any particular platform.  Hopefully, it is not too unreasonable, however. */
/* s.kang start */
/* to make JPC_QMFB_COLGRPSIZE * sizeof( jpc_fix_t ) multiple of cache line size */
/* CAUTION!!! THIS VALUE MUST BE MUTIPLE OF 32, OTHERWISE DMA ERROR WILL OCCUR */
#define JPC_QMFB_COLGRPSIZE	32
/* s.kang end */
#endif

typedef struct {
	int (*analyze)(int *, int, int, int, int, int);
	int (*synthesize)(int *, int, int, int, int, int);
	double *lpenergywts;
	double *hpenergywts;
} jpc_qmfb2d_t;

/* s.kang start */
#ifndef SPU
extern jpc_qmfb2d_t jpc_ft_qmfb2d;
extern jpc_qmfb2d_t jpc_ns_qmfb2d;
#else
int jpc_ft_row_analyze( unsigned int startaddr, int numrows, int numcols, int stride, int colparity );
int jpc_ft_row_synthesize( unsigned int startaddr, int numrows, int numcols, int stride, int colparity );
int jpc_ft_colgrp_analyze( unsigned int startaddr, unsigned int joinbufaddr, int numrows, int numcolgrps, int stride, int rowparity );
int jpc_ft_colgrp_synthesize( unsigned int startaddr, unsigned int joinbufaddr, int numrows, int numcolgrps, int stride, int rowparity );
int jpc_ns_row_analyze( unsigned int startaddr, int numrows, int numcols, int stride, int colparity );
int jpc_ns_row_synthesize( unsigned int startaddr, int numrows, int numcols, int stride, int colparity );
int jpc_ns_colgrp_analyze( unsigned int startaddr, unsigned int joinbufaddr, int numrows, int numcolgrps, int stride, int rowparity );
int jpc_ns_colgrp_synthesize( unsigned int startaddr, unsigned int joinbufaddr, int numrows, int numcolgrps, int stride, int rowparity );
#endif
/* s.kang end */

#endif
