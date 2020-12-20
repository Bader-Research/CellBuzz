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
 * JPC Image Library
 *
 * $Id: jpc_image.c,v 1.3 2008/02/06 20:37:15 lrlemini Exp $
 */

/* s.kang start */

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "jasper/jas_image.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
#include "jasper/jas_debug.h"

#include "jpc_spu_ctrl.h"
#include "jpc_image.h"
#include "jpc_fix.h"

#include "jasper-cell.h"

/******************************************************************************\
* Types.
\******************************************************************************/

/******************************************************************************\
* Local prototypes.
\******************************************************************************/

/******************************************************************************\
* Global data.
\******************************************************************************/

/******************************************************************************\
* Component read and write operations.
\******************************************************************************/

/* parallel version of jas_image_readcmpt */
int jpc_image_readcmpt( jas_image_t* image, int cmptno, jas_image_coord_t x, jas_image_coord_t y, jas_image_coord_t width, jas_image_coord_t height, jas_matrix_t* p_data )
{
	jas_image_cmpt_t* p_cmpt;
	unsigned char* p_cmpt_data;
	int cmpt_data_width;
	int numrows_per_spu;
	unsigned char* p_data0;
	jpc_fix_t* p_data1;
	jas_seqent_t v;
	int c;
	int i;
	int j;
	int k;

	if( cmptno < 0 || cmptno >= image->numcmpts_ ) {
		jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] invalid cmptno\n" );
		return -1;
	}

	p_cmpt = image->cmpts_[cmptno];
	if( x >= p_cmpt->width_ || y >= p_cmpt->height_ || x + width > p_cmpt->width_ || y + height > p_cmpt->height_) {
		jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] invalid x, y, width, height\n" );
		return -1;
	}

	if( jas_matrix_numrows( p_data ) != height || jas_matrix_numcols( p_data ) != width ) {
		jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] invalid width, height\n" );
		return -1;
	}

	if( jas_matrix_numrows( p_data ) != height || jas_matrix_numcols( p_data ) != width ) {
		if( jas_matrix_resize_row_cache_line_aligned( p_data, height, width ) ) {
			jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] jas_matrix_resize_row_cache_lien_aligned failure\n" );
			return -1;
		}
	}

	cmpt_data_width = ( width * p_cmpt->cps_ + CACHE_LINE_SIZE - 1 ) & CACHE_LINE_OFFSET_MASK;
	p_cmpt_data = jas_malloc_align( cmpt_data_width * height, CACHE_LINE_OFFSET_BITS );
	if( p_cmpt_data == NULL ) {
		jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] jas_malloc_align failure\n" );
		return -1;
	}

	/* read data from the stream */

	for( i = 0 ; i < height ; i++ ) {
		if( jas_stream_seek( p_cmpt->stream_, ( p_cmpt->width_ * ( y + i ) + x ) * p_cmpt->cps_, SEEK_SET ) < 0 ) {
			jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] jas_stream_seek failure\n" );
			jas_free_align( p_cmpt_data );
			return -1;
		}
		if( jas_stream_read( p_cmpt->stream_, p_cmpt_data + cmpt_data_width * i, width * p_cmpt->cps_ ) != width * p_cmpt->cps_ ) {
			jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] jas_stream_read failure\n" );
			jas_free_align( p_cmpt_data );
			return -1;
		}
	}

	/* distribute work to SPUs */

	numrows_per_spu = height / g_num_spus;
	for( i = 0; i < g_num_spus ; i++ ) {
		a_status[i].status = PRC_RUNNING;
		a_rd_cmpt_cmd_data[i].addr_cmpt_data = ( unsigned int )( p_cmpt_data + cmpt_data_width * numrows_per_spu * i );
		a_rd_cmpt_cmd_data[i].addr_matrix_data = ( unsigned int )jas_matrix_getref( p_data, numrows_per_spu * i, 0 );
		a_rd_cmpt_cmd_data[i].cmpt_data_stride = cmpt_data_width;
		a_rd_cmpt_cmd_data[i].matrix_stride = jas_matrix_rowstep( p_data );
		if( i == g_num_spus - 1 ) {
			a_rd_cmpt_cmd_data[i].numrows = height - numrows_per_spu * ( g_num_spus -1 );
		}
		else {
			a_rd_cmpt_cmd_data[i].numrows = numrows_per_spu;
		}
		a_rd_cmpt_cmd_data[i].numcols = width - ( width % JPC_RW_CMPT_BUF_ITEMS );
		a_rd_cmpt_cmd_data[i].cps = p_cmpt->cps_;
		a_rd_cmpt_cmd_data[i].prec = p_cmpt->prec_;
		a_rd_cmpt_cmd_data[i].sgnd = p_cmpt->sgnd_;

		jpc_send_cmd2spu( i, CMD_RD_CMPT );
	}

	/* process residual part */

	for( i = 0 ; i < height ; i++ ) {
		p_data0 = p_cmpt_data + cmpt_data_width * i;
		p_data1 = jas_matrix_getref( p_data, i, 0 );
		for( j = width - ( width % JPC_RW_CMPT_BUF_ITEMS ) ; j < width ; j++ ) {
			v = 0;
			for( k = 0 ; k < p_cmpt->cps_ ; k++ ) {
				c = p_data0[j * p_cmpt->cps_ + k];
				v = ( v << 8 ) | ( c & 0xff );
			}
			v &= JAS_ONES( p_cmpt->prec_ );
			p_data1[j] = ( ( p_cmpt->sgnd_ && ( v & ( 1 << ( p_cmpt->prec_ - 1 ) ) ) ) ? ( v - ( 1 << p_cmpt->prec_ ) ) : v );
		}
	}

	/* wait for all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "[jpc_image.c:jpc_image_readcmpt()] jpc_wait_for_spus() failure\n" );
		jas_free_align( p_cmpt_data );
		return -1;
	}

	jas_free_align( p_cmpt_data );

	return 0;
}

/* parallel version of jas_image_writecmpt */
int jpc_image_writecmpt( jas_image_t* image, int cmptno, jas_image_coord_t x, jas_image_coord_t y, jas_image_coord_t width, jas_image_coord_t height, jas_matrix_t* p_data )
{
	jas_image_cmpt_t* p_cmpt;
	unsigned char* p_cmpt_data;
	int cmpt_data_width;
	int numrows_per_spu;
	jpc_fix_t* p_data0;
	unsigned char* p_data1;
	jas_seqent_t v;
	int c;
	int i;
	int j;
	int k;

	if( cmptno < 0 || cmptno >= image->numcmpts_ ) {
		return -1;
	}

	p_cmpt = image->cmpts_[cmptno];
	if( x >= p_cmpt->width_ || y >= p_cmpt->height_ || x + width > p_cmpt->width_ || y + height > p_cmpt->height_) {
		return -1;
	}

	if( jas_matrix_numrows( p_data ) != height || jas_matrix_numcols( p_data ) != width ) {
		return -1;
	}

	cmpt_data_width = ( width * p_cmpt->cps_ + CACHE_LINE_SIZE - 1 ) & CACHE_LINE_OFFSET_MASK;
	p_cmpt_data = jas_malloc_align( cmpt_data_width * height, CACHE_LINE_OFFSET_BITS );
	if( p_cmpt_data == NULL ) {
		return -1;
	}

	/* distribute work to SPUs */

	numrows_per_spu = height / g_num_spus;
	for( i = 0; i < g_num_spus ; i++ ) {
		a_status[i].status = PRC_RUNNING;
		a_wr_cmpt_cmd_data[i].addr_matrix_data = ( unsigned int )jas_matrix_getref( p_data, numrows_per_spu * i, 0 );
		a_wr_cmpt_cmd_data[i].addr_cmpt_data = ( unsigned int )( p_cmpt_data + cmpt_data_width * numrows_per_spu * i );
		a_wr_cmpt_cmd_data[i].matrix_stride = jas_matrix_rowstep( p_data );
		a_wr_cmpt_cmd_data[i].cmpt_data_stride = cmpt_data_width;
		if( i == g_num_spus - 1 ) {
			a_wr_cmpt_cmd_data[i].numrows = height - numrows_per_spu * ( g_num_spus -1 );
		}
		else {
			a_wr_cmpt_cmd_data[i].numrows = numrows_per_spu;
		}
		a_wr_cmpt_cmd_data[i].numcols = width - ( width % JPC_RW_CMPT_BUF_ITEMS );
		a_wr_cmpt_cmd_data[i].cps = p_cmpt->cps_;
		a_wr_cmpt_cmd_data[i].prec = p_cmpt->prec_;
		a_wr_cmpt_cmd_data[i].sgnd = p_cmpt->sgnd_;

		jpc_send_cmd2spu( i, CMD_WR_CMPT );
	}

	/* process residual part */

	for( i = 0 ; i < height ; i++ ) {
		p_data0 = jas_matrix_getref( p_data, i, 0 );
		p_data1 = p_cmpt_data + cmpt_data_width * i;
		for( j = width - ( width % JPC_RW_CMPT_BUF_ITEMS ) ; j < width ; j++ ) {
			v = ( ( p_cmpt->sgnd_ && p_data0[j] < 0 ) ? ( ( 1 << p_cmpt->prec_ ) + p_data0[j] ) : p_data0[j] ) & JAS_ONES( p_cmpt->prec_ );
			for( k = 0 ; k < p_cmpt->cps_ ; k++ ) {
				c = ( v >> ( 8 * ( p_cmpt->cps_ - 1 ) ) ) & 0xff;
				p_data1[j * p_cmpt->cps_ + k] = c;
				v <<= 8;
			}
		}
	}

	/* wait for all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_free_align( p_cmpt_data );
		return -1;
	}

	/* write data to the stream */

	for( i = 0 ; i < height ; i++ ) {
		if( jas_stream_seek( p_cmpt->stream_, ( p_cmpt->width_ * ( y + i ) + x ) * p_cmpt->cps_, SEEK_SET ) < 0 ) {
			jas_free_align( p_cmpt_data );
			return -1;
		}
		if( jas_stream_write( p_cmpt->stream_, p_cmpt_data + cmpt_data_width * i, width * p_cmpt->cps_ ) != width * p_cmpt->cps_ ) {
			jas_free_align( p_cmpt_data );
			return -1;
		}
	}
	jas_free_align( p_cmpt_data );

	return 0;
}
/* s.kang end */

