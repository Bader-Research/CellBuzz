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
 * Sequence/Matrix Library
 *
 * $Id: jas_seq.c,v 1.3 2007/09/12 23:58:56 lrlemini Exp $
 */

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <stdlib.h>
#include <spu_assert.h>
#include <math.h>

#include "jasper/jas_seq.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
* Constructors and destructors.
\******************************************************************************/

jas_matrix_t *jas_matrix_create(int numrows, int numcols)
{
	jas_matrix_t *matrix;
	int i;

	if (!(matrix = jas_malloc(sizeof(jas_matrix_t)))) {
		return 0;
	}
	matrix->flags_ = 0;
	matrix->numrows_ = numrows;
	matrix->numcols_ = numcols;
	matrix->rows_ = 0;
	matrix->maxrows_ = numrows;
	matrix->data_ = 0;
	matrix->datasize_ = numrows * numcols;

	if (matrix->maxrows_ > 0) {
/* s.kang start */
/* aling by cache line size */
#if 1
		if (!(matrix->rows_ = jas_malloc_align(matrix->maxrows_ *
		  sizeof(jas_seqent_t *), CACHE_LINE_OFFSET_BITS))) {
#else
		if (!(matrix->rows_ = jas_malloc(matrix->maxrows_ *
		  sizeof(jas_seqent_t *)))) {
#endif
/* s.kang end */
			jas_matrix_destroy(matrix);
			return 0;
		}
	}

	if (matrix->datasize_ > 0) {
/* s.kang start */
/* aling by cache line size */
#if 1
		if( !( matrix->data_ = jas_malloc_align( matrix->datasize_ * sizeof( jas_seqent_t ), CACHE_LINE_OFFSET_BITS ) ) ) {
#else
		if (!(matrix->data_ = jas_malloc(matrix->datasize_ *
		  sizeof(jas_seqent_t)))) {
#endif
/* s.kang end */
			jas_matrix_destroy(matrix);
			return 0;
		}
	}

	for (i = 0; i < numrows; ++i) {
		matrix->rows_[i] = &matrix->data_[i * matrix->numcols_];
	}

/* s.kang start */
/* use memset */
#if 1
	memset( matrix->data_, 0x00, matrix->datasize_ * sizeof( jas_seqent_t ) );	
#else
	for (i = 0; i < matrix->datasize_; ++i) {
		matrix->data_[i] = 0;
	}
#endif
/* s.kang end */

	matrix->xstart_ = 0;
	matrix->ystart_ = 0;
	matrix->xend_ = matrix->numcols_;
	matrix->yend_ = matrix->numrows_;

	return matrix;
}

void jas_matrix_destroy(jas_matrix_t *matrix)
{
	if (matrix->data_) {
		spu_assert( ( (!(matrix->flags_ & JAS_MATRIX_REF) ) ), ( "[jas_seq.c:jas_matrix_destroy()] assertion failure\n" ) );
/* s.kang start */
/* free aligned memory space */
#if 1
		jas_free_align( matrix->data_ );
#else
		jas_free(matrix->data_);
#endif
/* s.kang end */
		matrix->data_ = 0;
	}
	if (matrix->rows_) {
/* s.kang start */
/* free aligned memory space */
#if 1
		jas_free_align(matrix->rows_);
#else
		jas_free(matrix->rows_);
#endif
/* s.kang start */
		matrix->rows_ = 0;
	}
	jas_free(matrix);
}

jas_matrix_t *jas_matrix_copy(jas_matrix_t *x)
{
	jas_matrix_t *y;
	int i;
	int j;
	y = jas_matrix_create(x->numrows_, x->numcols_);
	for (i = 0; i < x->numrows_; ++i) {
		for (j = 0; j < x->numcols_; ++j) {
			*jas_matrix_getref(y, i, j) = jas_matrix_get(x, i, j);
		}
	}
	return y;
}

/******************************************************************************\
* Arithmetic operations.
\******************************************************************************/

int jas_matrix_cmp(jas_matrix_t *mat0, jas_matrix_t *mat1)
{
	int i;
	int j;

	if (mat0->numrows_ != mat1->numrows_ || mat0->numcols_ !=
	  mat1->numcols_) {
		return 1;
	}
	for (i = 0; i < mat0->numrows_; i++) {
		for (j = 0; j < mat0->numcols_; j++) {
			if (jas_matrix_get(mat0, i, j) != jas_matrix_get(mat1, i, j)) {
				return 1;
			}
		}
	}
	return 0;
}

void jas_matrix_divpow2(jas_matrix_t *matrix, int n)
{
	int i;
	int j;
	jas_seqent_t *rowstart;
	int rowstep;
	jas_seqent_t *data;

	rowstep = jas_matrix_rowstep(matrix);
	for (i = matrix->numrows_, rowstart = matrix->rows_[0]; i > 0; --i,
	  rowstart += rowstep) {
		for (j = matrix->numcols_, data = rowstart; j > 0; --j,
		  ++data) {
			*data = (*data >= 0) ? ((*data) >> n) :
			  (-((-(*data)) >> n));
		}
	}
}

void jas_matrix_clip(jas_matrix_t *matrix, jas_seqent_t minval, jas_seqent_t maxval)
{
	int i;
	int j;
	jas_seqent_t v;
	jas_seqent_t *rowstart;
	jas_seqent_t *data;
	int rowstep;

	rowstep = jas_matrix_rowstep(matrix);
	for (i = matrix->numrows_, rowstart = matrix->rows_[0]; i > 0; --i,
	  rowstart += rowstep) {
		data = rowstart;
		for (j = matrix->numcols_, data = rowstart; j > 0; --j,
		  ++data) {
			v = *data;
			if (v < minval) {
				*data = minval;
			} else if (v > maxval) {
				*data = maxval;
			}
		}
	}
}

void jas_matrix_asr(jas_matrix_t *matrix, int n)
{
	int i;
	int j;
	jas_seqent_t *rowstart;
	int rowstep;
	jas_seqent_t *data;

	spu_assert( ( n >= 0 ), ( "[jas_seq.c:jas_matrix_asr()] assertion failure\n" ) );
	rowstep = jas_matrix_rowstep(matrix);
	for (i = matrix->numrows_, rowstart = matrix->rows_[0]; i > 0; --i,
	  rowstart += rowstep) {
		for (j = matrix->numcols_, data = rowstart; j > 0; --j,
		  ++data) {
			*data >>= n;
		}
	}
}

void jas_matrix_asl(jas_matrix_t *matrix, int n)
{
	int i;
	int j;
	jas_seqent_t *rowstart;
	int rowstep;
	jas_seqent_t *data;

	rowstep = jas_matrix_rowstep(matrix);
	for (i = matrix->numrows_, rowstart = matrix->rows_[0]; i > 0; --i,
	  rowstart += rowstep) {
		for (j = matrix->numcols_, data = rowstart; j > 0; --j,
		  ++data) {
			*data <<= n;
		}
	}
}

