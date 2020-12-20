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
 * Tier 1 Decoder
 *
 * $Id: jpc_t1dec.c,v 1.5 2008/02/06 20:37:15 lrlemini Exp $
 */

/******************************************************************************\
* Includes.
\******************************************************************************/

/* s.kang start */
#include <pthread.h>
#include <sched.h>
/* s.kang end */
#include <stdio.h>
#include <stdlib.h>

#include "jasper/jas_fix.h"
#include "jasper/jas_stream.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_malloc.h"
#include "jasper/jas_debug.h"
/* s.kang end */

#include "jpc_bs.h"
#include "jpc_mqdec.h"
#include "jpc_t1dec.h"
#include "jpc_t1cod.h"
#include "jpc_dec.h"
/* s.kang start */
#include "jpc_spu_ctrl.h"
/* s.kang end */

/* s.kang start */
#include "jasper-cell.h"
/* s.kang end */

/******************************************************************************\
*
\******************************************************************************/

/* s.kang start */
static int jpc_dec_decodecblk( t1_dec_cmd_data_t* p_t1_dec_cmd_data );
/* s.kang end */

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
static int dec_sigpass(jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data);
static int dec_rawsigpass(jpc_bitstream_t *in, int bitpos,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data);
static int dec_refpass(jpc_mqdec_t *mqdec, int bitpos, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data);
static int dec_rawrefpass(jpc_bitstream_t *in, int bitpos,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data);
static int dec_clnpass(jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, int segsymflag, jas_matrix_t *flags, jas_matrix_t *data);
/* s.kang end */

#if defined(DEBUG)
static long t1dec_cnt = 0;
#endif

/* s.kang start */
#if !defined(DEBUG)
#define	JPC_T1D_GETBIT(mqdec, v, passtypename, symtypename) \
	((v) = jpc_mqdec_getbit(mqdec))
#else
#define	JPC_T1D_GETBIT(mqdec, v, passtypename, symtypename) \
{ \
	(v) = jpc_mqdec_getbit(mqdec); \
	if (jas_getdbglevel() >= 100) { \
		jas_eprintf("index = %ld; passtype = %s; symtype = %s; sym = %d\n", t1dec_cnt, passtypename, symtypename, v); \
		++t1dec_cnt; \
	} \
}
#endif
#define	JPC_T1D_GETBITNOSKEW(mqdec, v, passtypename, symtypename) \
	JPC_T1D_GETBIT(mqdec, v, passtypename, symtypename)

#if !defined(DEBUG)
#define	JPC_T1D_RAWGETBIT(bitstream, v, passtypename, symtypename) \
	((v) = jpc_bitstream_getbit(bitstream))
#else
#define	JPC_T1D_RAWGETBIT(bitstream, v, passtypename, symtypename) \
{ \
	(v) = jpc_bitstream_getbit(bitstream); \
	if (jas_getdbglevel() >= 100) { \
		jas_eprintf("index = %ld; passtype = %s; symtype = %s; sym = %d\n", t1dec_cnt, passtypename, symtypename, v); \
		++t1dec_cnt; \
	} \
}
#endif
/* s.kang end */

/* s.kang start */
static volatile unsigned int a_ppu_cmd[MAX_PPU_THREADS];
static pthread_mutex_t a_cond_mutex[MAX_PPU_THREADS];
static pthread_cond_t a_cond_var[MAX_PPU_THREADS];

typedef struct _dec_cblks_arg_t {
	jpc_dec_t* p_dec;
	jpc_dec_tile_t* p_tile;
} dec_cblks_arg_t;

static void* dec_cblks_func( void* p_arg );
static void* tier1_thread_func( void* p_arg );
/* s.kang end */

/******************************************************************************\
* Code.
\******************************************************************************/

/* s.kang start */
#if 1
int jpc_dec_decodecblks(jpc_dec_t *dec, jpc_dec_tile_t *tile) {
	pthread_attr_t attr;
	struct sched_param param;
	pthread_t dec_cblks_thread;
	pthread_t a_tier1_threads[MAX_PPU_THREADS];
	void* p_pthread_ret;
	dec_cblks_arg_t dec_cblks_arg;
	jpc_dec_seg_t* p_seg;
	int i;

	/* status set to PRC_IDLE after threads are spawned, prevent sending command before threads are created */

	for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
		a_status[i].status = PRC_RUNNING;
	}

	/* init scheduling attribute  */

	if( pthread_attr_init( &attr ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_attr_init() failure\n" );
		return -1;
	}

	if( pthread_attr_setschedpolicy( &attr, SCHED_RR ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_attr_setschedpolicy() failure\n" );
		return -1;
	}

	if( pthread_attr_getschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_attr_getshcedparam() failure\n" );
		return -1;
	}

	/* create main code block decoding thread */

	param.sched_priority = ( sched_get_priority_max( SCHED_RR ) + sched_get_priority_min( SCHED_RR ) ) >> 1;
	if( pthread_attr_setschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_attr_setschedparam() failure\n" );
		return -1;
	}

	dec_cblks_arg.p_dec = dec;
	dec_cblks_arg.p_tile = tile;
	if( pthread_create( &dec_cblks_thread, &attr, dec_cblks_func, ( void* )&dec_cblks_arg ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_create() failure\n" );
		return -1;
	}

	/* create tier-1 decoding thread */

	param.sched_priority = sched_get_priority_min( SCHED_RR );
	if( pthread_attr_setschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_attr_setschedparam() failure\n" );
		return -1;
	}

	for( i = 0 ; i < g_num_ppus ; i++ ) {
		pthread_mutex_init( &a_cond_mutex[i], NULL);
		pthread_cond_init ( &a_cond_var[i], NULL);
		if( pthread_create( &a_tier1_threads[i], &attr, tier1_thread_func, ( void* )i ) != 0 ) {
			jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_create() failure\n" );
			return -1;
		}
	}

	/* do work in the thread - to control priority, dynamic priority change fails on the blade */

	/* wait for main code block decoding thread  to finish execution */

	if( pthread_join( dec_cblks_thread, &p_pthread_ret ) != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_join() failure\n" );
		return -1;
	}

	if( p_pthread_ret != NULL ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] dec_cblks_thread error\n" );
		return -1;
	}

	/* wait until all the PPUs finish execution */

	for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
		while( a_status[i].status == PRC_RUNNING ) {};
		pthread_mutex_lock( &a_cond_mutex[i - g_num_spus] );
		a_ppu_cmd[i - g_num_spus] = CMD_PPU_TERM;
		pthread_cond_signal( &a_cond_var[i - g_num_spus] );
		pthread_mutex_unlock( &a_cond_mutex[i - g_num_spus] );
	}

	for( i = 0 ; i < g_num_ppus ; i++ ) {
		if( pthread_join( a_tier1_threads[i], &p_pthread_ret ) != 0 ) {
			jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] pthread_join() failure\n" );
			return -1;
		}

		if( p_pthread_ret != NULL ) {
			jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] tier1_thread error\n" );
			return -1;
		}

		pthread_mutex_destroy( &a_cond_mutex[i] );
		pthread_cond_destroy( &a_cond_var[i] );
	}

	pthread_attr_destroy( &attr );

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "jpc_t1dec.c:jpc_dec_decodecblks()] jpc_wait_for_spus() failure\n" );
		return -1;
	}

	/* free segment data */

	for( i = 0 ; i < g_num_spus + g_num_ppus ; i++ ) {
		if( a_t1_dec_cmd_data[i].p_cblk ) {
			while( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head ) {
				p_seg = ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head;
				jpc_seglist_remove( &( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs ), p_seg );
				jpc_seg_destroy( p_seg );
			}
			a_t1_dec_cmd_data[i].p_cblk = NULL;
		}
	}

	return 0;
}
#else
int jpc_dec_decodecblks(jpc_dec_t *dec, jpc_dec_tile_t *tile)
{
	jpc_dec_tcomp_t *tcomp;
	int compcnt;
	jpc_dec_rlvl_t *rlvl;
	int rlvlcnt;
	jpc_dec_band_t *band;
	int bandcnt;
	jpc_dec_prc_t *prc;
	int prccnt;
	jpc_dec_cblk_t *cblk;
	int cblkcnt;

	for (compcnt = dec->numcomps, tcomp = tile->tcomps; compcnt > 0;
	  --compcnt, ++tcomp) {
		for (rlvlcnt = tcomp->numrlvls, rlvl = tcomp->rlvls;
		  rlvlcnt > 0; --rlvlcnt, ++rlvl) {
			if (!rlvl->bands) {
				continue;
			}
			for (bandcnt = rlvl->numbands, band = rlvl->bands;
			  bandcnt > 0; --bandcnt, ++band) {
				if (!band->data) {
					continue;
				}
				for (prccnt = rlvl->numprcs, prc = band->prcs;
				  prccnt > 0; --prccnt, ++prc) {
					if (!prc->cblks) {
						continue;
					}
					for (cblkcnt = prc->numcblks,
					  cblk = prc->cblks; cblkcnt > 0;
					  --cblkcnt, ++cblk) {
						if (jpc_dec_decodecblk(dec, tile, tcomp,
						  band, cblk, 1, JPC_MAXLYRS)) {
							return -1;
						}
					}
				}

			}
		}
	}

	return 0;
}
#endif
/* s.kang end */

/* s.kang start */
static int jpc_dec_decodecblk( t1_dec_cmd_data_t* p_t1_dec_cmd_data )
{
	jpc_dec_seg_t* p_seg;
	int i;
	int bpno;
	int passtype;
	int ret;
	int filldata;
	int fillmask;
        jpc_mqdec_t* mqdec = 0;
	jpc_bitstream_t* nulldec = 0;
	jas_matrix_t* p_data = 0;
	jas_matrix_t* p_flags = 0;

	/* Note: matrix is assumed to be zeroed */
	if (!(p_flags = jas_matrix_create(p_t1_dec_cmd_data->numrows + 2, p_t1_dec_cmd_data->numcols + 2))) {
		jas_eprintf( "[jpc_t1dec.c:jpd_dec_decodecblk()] jas_matrix_create failure( %d, %d )\n", p_t1_dec_cmd_data->numrows + 2, p_t1_dec_cmd_data->numcols + 2 );
		return -1;
	}

	if( ( p_t1_dec_cmd_data->addr_seg_head % CACHE_LINE_SIZE ) != 0 ) {
		jas_eprintf( "[jpc_t1dec.c:jpc_dec_decodecblk()] segment head(0x%x) alignment error\n", ( unsigned int )( p_t1_dec_cmd_data->addr_seg_head ) );
		return -1;
	}

	p_seg = ( jpc_dec_seg_t* )( p_t1_dec_cmd_data->addr_seg_head );

	p_data = ( jas_matrix_t* )( p_t1_dec_cmd_data->addr_data );
	if( p_data == NULL ) {
		jas_eprintf( "[jpc_t1dec.c:jpc_dec_decodecblk()] jpc_get_cblk_matrix failure\n" );
		return -1;
	}

	while ( ( p_seg != NULL ) && (p_t1_dec_cmd_data->maxlyrs < 0 || p_seg->lyrno < p_t1_dec_cmd_data->maxlyrs)) {
		jas_stream_rewind( p_seg->stream );
		jas_stream_setrwcount( p_seg->stream, 0 );

		if (p_seg->type == JPC_SEG_MQ) {
                        if (!mqdec) {
                                if (!(mqdec = jpc_mqdec_create(JPC_NUMCTXS, 0))) {
					if(mqdec) {
						jpc_mqdec_destroy(mqdec);
					}
					if (nulldec) {
						jpc_bitstream_close(nulldec);
					}
					if (p_flags) {
						jas_matrix_destroy(p_flags);
					}
					jas_eprintf( "[jpc_dec.c:jpc_dec_decodecblk()] jpc_mqdec_create failure\n" );
                                        return -1;
                                }
                                jpc_mqdec_setctxs(mqdec, JPC_NUMCTXS, jpc_mqctxs);
                        }
			jpc_mqdec_setinput( mqdec, p_seg->stream );
			jpc_mqdec_init(mqdec);
		} else {
			assert( p_seg->type == JPC_SEG_RAW );
			if (!nulldec) {
				if( !( nulldec = jpc_bitstream_sopen( p_seg->stream, "r" ) ) ) {
					if(mqdec) {
						jpc_mqdec_destroy(mqdec);
					}
					if (nulldec) {
						jpc_bitstream_close(nulldec);
					}
					if (p_flags) {
						jas_matrix_destroy(p_flags);
					}
					jas_eprintf( "[jpc_dec.c:jpc_dec_decodecblk()] jpc_bitstream_sopen failure\n" );
					return -1;
				}
			}
		}

		for (i = 0; i < p_seg->numpasses; ++i) {
			if (p_t1_dec_cmd_data->numimsbs > p_t1_dec_cmd_data->numbps) {
				if (p_t1_dec_cmd_data->c_roishift <= 0) {
					jas_eprintf("[jpc_dec.c:jpc_dec_decodecblk()] warning: corrupt code stream\n");
				} else {
					if (p_t1_dec_cmd_data->numimsbs < (int)p_t1_dec_cmd_data->c_roishift - p_t1_dec_cmd_data->numbps) {
						jas_eprintf("[jpc_dec.c:jpc_dec_decodecblk()] warning: corrupt code stream\n");
					}
				}
			}
			bpno = p_t1_dec_cmd_data->b_roishift + p_t1_dec_cmd_data->numbps - 1 - (p_t1_dec_cmd_data->numimsbs +
			  (p_seg->passno + i - p_t1_dec_cmd_data->firstpassno + 2) / 3);
if (bpno < 0) {
	goto premature_exit;
}
#if 1
			passtype = (p_seg->passno + i + 2) % 3;
#else
			passtype = JPC_PASSTYPE(p_seg->passno + i + 2);
#endif
			assert( bpno >= 0 && bpno < 31 );
			switch (passtype) {
			case JPC_SIGPASS:
				ret = (p_seg->type == JPC_SEG_MQ) ? dec_sigpass(
				  mqdec, bpno, p_t1_dec_cmd_data->orient,
				  (p_t1_dec_cmd_data->cblkctx & JPC_COX_VSC) != 0,
				  p_flags, p_data) :
				  dec_rawsigpass(nulldec, bpno,
				  (p_t1_dec_cmd_data->cblkctx & JPC_COX_VSC) != 0,
				  p_flags, p_data);
				break;
			case JPC_REFPASS:
				ret = (p_seg->type == JPC_SEG_MQ) ?
				  dec_refpass(mqdec, bpno,
				  (p_t1_dec_cmd_data->cblkctx & JPC_COX_VSC) != 0,
				  p_flags, p_data) :
				  dec_rawrefpass(nulldec, bpno,
				  (p_t1_dec_cmd_data->cblkctx & JPC_COX_VSC) != 0,
				  p_flags, p_data);
				break;
			case JPC_CLNPASS:
				assert( p_seg->type == JPC_SEG_MQ );
				ret = dec_clnpass(mqdec, bpno,
				  p_t1_dec_cmd_data->orient, (p_t1_dec_cmd_data->cblkctx &
				  JPC_COX_VSC) != 0, (p_t1_dec_cmd_data->cblkctx &
				  JPC_COX_SEGSYM) != 0, p_flags,
				  p_data);
				break;
			default:
				jas_eprintf( "[jpc_dec.c:jpc_dec_decodecblk()] invalid passtype\n" );
				ret = -1;
				break;
			}
			/* Do we need to reset after each coding pass? */
			if (p_t1_dec_cmd_data->cblkctx & JPC_COX_RESET) {
				jpc_mqdec_setctxs(mqdec, JPC_NUMCTXS, jpc_mqctxs);
			}

			if (ret) {
				if(mqdec) {
					jpc_mqdec_destroy(mqdec);
				}
				if (nulldec) {
					jpc_bitstream_close(nulldec);
				}
				if (p_flags) {
					jas_matrix_destroy(p_flags);
				}
				jas_eprintf("[jpc_dec.c:jpc_dec_decodecblk()] coding pass failed passtype=%d segtype=%d\n", passtype, p_seg->type);
				return -1;
			}
		}

		if (p_seg->type == JPC_SEG_MQ) {
/* Note: dont destroy mq decoder because context info will be lost */
		} else {
			assert( p_seg->type == JPC_SEG_RAW );
			if (p_t1_dec_cmd_data->cblkctx & JPC_COX_PTERM) {
				fillmask = 0x7f;
				filldata = 0x2a;
			} else {
				fillmask = 0;
				filldata = 0;
			}
			if ((ret = jpc_bitstream_inalign(nulldec, fillmask,
			  filldata)) < 0) {
				if(mqdec) {
					jpc_mqdec_destroy(mqdec);
				}
				if (nulldec) {
					jpc_bitstream_close(nulldec);
				}
				if (p_flags) {
					jas_matrix_destroy(p_flags);
				}
				jas_eprintf( "[jpc_dec.c:jpc_dec_decodecblk()] jpc_bitstream_inalign failure\n" );
				return -1;
			} else if (ret > 0) {
				jas_eprintf("[jpc_dec.c:jpc_dec_decodecblk()] warning: bad termination pattern detected\n");
			}
			jpc_bitstream_close(nulldec);
			nulldec = 0;
		}

		if( ( ( unsigned int )( p_seg->next ) % CACHE_LINE_SIZE ) != 0 ) {
			if(mqdec) {
				jpc_mqdec_destroy(mqdec);
			}
			if (nulldec) {
				jpc_bitstream_close(nulldec);
			}
			if (p_flags) {
				jas_matrix_destroy(p_flags);
			}
			jas_eprintf( "[jpc_t1dec.c:jpc_dec_decodecblk()] next segment(0x%x) alignment error\n", ( unsigned int )( p_seg->next ) );
			return -1;
		}

		p_seg = p_seg->next;
	}

premature_exit:
	if( mqdec ) {
		jpc_mqdec_destroy( mqdec );
	}
	if( nulldec ) {
		jpc_bitstream_close( nulldec );
	}
	if( p_flags ) {
		jas_matrix_destroy( p_flags );
	}

	return 0;
}
/* s.kang end */

/******************************************************************************\
* Code for significance pass.
\******************************************************************************/

/* s.kang start */
#define	jpc_sigpass_step(fp, frowstep, dp, bitpos, oneplushalf, orient, mqdec, vcausalflag) \
{ \
	int f; \
	int v; \
	f = *(fp); \
	if ((f & JPC_OTHSIGMSK) && !(f & (JPC_SIG | JPC_VISIT))) { \
		jpc_mqdec_setcurctx((mqdec), JPC_GETZCCTXNO(f, (orient))); \
		JPC_T1D_GETBIT((mqdec), v, "SIG", "ZC"); \
		if (v) { \
			jpc_mqdec_setcurctx((mqdec), JPC_GETSCCTXNO(f)); \
			JPC_T1D_GETBIT((mqdec), v, "SIG", "SC"); \
			v ^= JPC_GETSPB(f); \
			JPC_UPDATEFLAGS4((fp), (frowstep), v, (vcausalflag)); \
			*(fp) |= JPC_SIG; \
			*(dp) = (v) ? (-(oneplushalf)) : (oneplushalf); \
		} \
		*(fp) |= JPC_VISIT; \
	} \
}

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 1
static int dec_sigpass(register jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data)
#else
static int dec_sigpass(jpc_dec_t *dec, register jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data)
#endif
/* s.kang end */
{
	int i;
	int j;
	int one;
	int half;
	int oneplushalf;
	int vscanlen;
	int width;
	int height;
	jpc_fix_t *fp;
	int frowstep;
	int fstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dp;
	int drowstep;
	int dstripestep;
	jpc_fix_t *dstripestart;
	jpc_fix_t *dvscanstart;
	int k;

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 0
	/* Avoid compiler warning about unused parameters. */
	dec = 0;
#endif
/* s.kang end */

	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << bitpos;
	half = one >> 1;
	oneplushalf = one | half;

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);
	for (i = height; i > 0; i -= 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(i, 4);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {
			fp = fvscanstart;
			dp = dvscanstart;
			k = vscanlen;

			/* Process first sample in vertical scan. */
			jpc_sigpass_step(fp, frowstep, dp, bitpos, oneplushalf,
			  orient, mqdec, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process second sample in vertical scan. */
			jpc_sigpass_step(fp, frowstep, dp, bitpos, oneplushalf,
			  orient, mqdec, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process third sample in vertical scan. */
			jpc_sigpass_step(fp, frowstep, dp, bitpos, oneplushalf,
			  orient, mqdec, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process fourth sample in vertical scan. */
			jpc_sigpass_step(fp, frowstep, dp, bitpos, oneplushalf,
			  orient, mqdec, 0);
		}
	}
	return 0;
}

#define	jpc_rawsigpass_step(fp, frowstep, dp, oneplushalf, in, vcausalflag) \
{ \
	jpc_fix_t f = *(fp); \
	jpc_fix_t v; \
	if ((f & JPC_OTHSIGMSK) && !(f & (JPC_SIG | JPC_VISIT))) { \
		JPC_T1D_RAWGETBIT(in, v, "SIG", "ZC"); \
		if (v < 0) { \
			return -1; \
		} \
		if (v) { \
			JPC_T1D_RAWGETBIT(in, v, "SIG", "SC"); \
			if (v < 0) { \
				return -1; \
			} \
			JPC_UPDATEFLAGS4((fp), (frowstep), v, (vcausalflag)); \
			*(fp) |= JPC_SIG; \
			*(dp) = v ? (-oneplushalf) : (oneplushalf); \
		} \
		*(fp) |= JPC_VISIT; \
	} \
}

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 1
static int dec_rawsigpass(jpc_bitstream_t *in, int bitpos, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data)
#else
static int dec_rawsigpass(jpc_dec_t *dec, jpc_bitstream_t *in, int bitpos, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data)
#endif
/* s.kang end */
{
	int i;
	int j;
	int k;
	int one;
	int half;
	int oneplushalf;
	int vscanlen;
	int width;
	int height;
	jpc_fix_t *fp;
	int frowstep;
	int fstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dp;
	int drowstep;
	int dstripestep;
	jpc_fix_t *dstripestart;
	jpc_fix_t *dvscanstart;


/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 0
	/* Avoid compiler warning about unused parameters. */
	dec = 0;
#endif
/* s.kang end */

	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << bitpos;
	half = one >> 1;
	oneplushalf = one | half;

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);
	for (i = height; i > 0; i -= 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(i, 4);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {
			fp = fvscanstart;
			dp = dvscanstart;
			k = vscanlen;

			/* Process first sample in vertical scan. */
			jpc_rawsigpass_step(fp, frowstep, dp, oneplushalf,
			  in, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process second sample in vertical scan. */
			jpc_rawsigpass_step(fp, frowstep, dp, oneplushalf,
			  in, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process third sample in vertical scan. */
			jpc_rawsigpass_step(fp, frowstep, dp, oneplushalf,
			  in, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process fourth sample in vertical scan. */
			jpc_rawsigpass_step(fp, frowstep, dp, oneplushalf,
			  in, 0);

		}
	}
	return 0;
}

/******************************************************************************\
* Code for refinement pass.
\******************************************************************************/

#define	jpc_refpass_step(fp, dp, poshalf, neghalf, mqdec, vcausalflag) \
{ \
	int v; \
	int t; \
	if (((*(fp)) & (JPC_SIG | JPC_VISIT)) == JPC_SIG) { \
		jpc_mqdec_setcurctx((mqdec), JPC_GETMAGCTXNO(*(fp))); \
		JPC_T1D_GETBITNOSKEW((mqdec), v, "REF", "MR"); \
		t = (v ? (poshalf) : (neghalf)); \
		*(dp) += (*(dp) < 0) ? (-t) : t; \
		*(fp) |= JPC_REFINE; \
	} \
}

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 1
static int dec_refpass(register jpc_mqdec_t *mqdec, int bitpos,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data)
#else
static int dec_refpass(jpc_dec_t *dec, register jpc_mqdec_t *mqdec, int bitpos,
  int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data)
#endif
/* s.kang end */
{
	int i;
	int j;
	int vscanlen;
	int width;
	int height;
	int one;
	int poshalf;
	int neghalf;
	jpc_fix_t *fp;
	int frowstep;
	int fstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dp;
	int drowstep;
	int dstripestep;
	jpc_fix_t *dstripestart;
	jpc_fix_t *dvscanstart;
	int k;

	/* Avoid compiler warning about unused parameters. */
/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 0
	dec = 0;
#endif
/* s.kang end */
	vcausalflag = 0;

	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << bitpos;
	poshalf = one >> 1;
	neghalf = (bitpos > 0) ? (-poshalf) : (-1);

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);
	for (i = height; i > 0; i -= 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(i, 4);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {
			fp = fvscanstart;
			dp = dvscanstart;
			k = vscanlen;

			/* Process first sample in vertical scan. */
			jpc_refpass_step(fp, dp, poshalf, neghalf, mqdec,
			  vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process second sample in vertical scan. */
			jpc_refpass_step(fp, dp, poshalf, neghalf, mqdec, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process third sample in vertical scan. */
			jpc_refpass_step(fp, dp, poshalf, neghalf, mqdec, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process fourth sample in vertical scan. */
			jpc_refpass_step(fp, dp, poshalf, neghalf, mqdec, 0);
		}
	}

	return 0;
}

#define	jpc_rawrefpass_step(fp, dp, poshalf, neghalf, in, vcausalflag) \
{ \
	jpc_fix_t v; \
	jpc_fix_t t; \
	if (((*(fp)) & (JPC_SIG | JPC_VISIT)) == JPC_SIG) { \
		JPC_T1D_RAWGETBIT(in, v, "REF", "MAGREF"); \
		if (v < 0) { \
			return -1; \
		} \
		t = (v ? poshalf : neghalf); \
		*(dp) += (*(dp) < 0) ? (-t) : t; \
		*(fp) |= JPC_REFINE; \
	} \
}

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 1
static int dec_rawrefpass(jpc_bitstream_t *in, int bitpos, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data)
#else
static int dec_rawrefpass(jpc_dec_t *dec, jpc_bitstream_t *in, int bitpos, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data)
#endif
/* s.kang end */
{
	int i;
	int j;
	int k;
	int vscanlen;
	int width;
	int height;
	int one;
	int poshalf;
	int neghalf;
	jpc_fix_t *fp;
	int frowstep;
	int fstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dp;
	int drowstep;
	int dstripestep;
	jpc_fix_t *dstripestart;
	jpc_fix_t *dvscanstart;

	/* Avoid compiler warning about unused parameters. */
/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 0
	dec = 0;
#endif
/* s.kang end */
	vcausalflag = 0;

	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << bitpos;
	poshalf = one >> 1;
	neghalf = (bitpos > 0) ? (-poshalf) : (-1);

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);
	for (i = height; i > 0; i -= 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(i, 4);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {
			fp = fvscanstart;
			dp = dvscanstart;
			k = vscanlen;

			/* Process first sample in vertical scan. */
			jpc_rawrefpass_step(fp, dp, poshalf, neghalf, in,
			  vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process second sample in vertical scan. */
			jpc_rawrefpass_step(fp, dp, poshalf, neghalf, in, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process third sample in vertical scan. */
			jpc_rawrefpass_step(fp, dp, poshalf, neghalf, in, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process fourth sample in vertical scan. */
			jpc_rawrefpass_step(fp, dp, poshalf, neghalf, in, 0);
		}
	}
	return 0;
}

/******************************************************************************\
* Code for cleanup pass.
\******************************************************************************/

#define	jpc_clnpass_step(f, fp, frowstep, dp, oneplushalf, orient, mqdec, flabel, plabel, vcausalflag) \
{ \
	int v; \
flabel \
	if (!((f) & (JPC_SIG | JPC_VISIT))) { \
		jpc_mqdec_setcurctx((mqdec), JPC_GETZCCTXNO((f), (orient))); \
		JPC_T1D_GETBIT((mqdec), v, "CLN", "ZC"); \
		if (v) { \
plabel \
			/* Coefficient is significant. */ \
			jpc_mqdec_setcurctx((mqdec), JPC_GETSCCTXNO(f)); \
			JPC_T1D_GETBIT((mqdec), v, "CLN", "SC"); \
			v ^= JPC_GETSPB(f); \
			*(dp) = (v) ? (-(oneplushalf)) : (oneplushalf); \
			JPC_UPDATEFLAGS4((fp), (frowstep), v, (vcausalflag)); \
			*(fp) |= JPC_SIG; \
		} \
	} \
	/* XXX - Is this correct?  Can aggregation cause some VISIT bits not to be reset?  Check. */ \
	*(fp) &= ~JPC_VISIT; \
}

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 1
static int dec_clnpass(register jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, int segsymflag, jas_matrix_t *flags, jas_matrix_t *data)
#else
static int dec_clnpass(jpc_dec_t *dec, register jpc_mqdec_t *mqdec, int bitpos, int orient,
  int vcausalflag, int segsymflag, jas_matrix_t *flags, jas_matrix_t *data)
#endif
/* s.kang end */
{
	int i;
	int j;
	int k;
	int vscanlen;
	int v;
	int half;
	int runlen;
	int f;
	int width;
	int height;
	int one;
	int oneplushalf;

	jpc_fix_t *fp;
	int frowstep;
	int fstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *fvscanstart;

	jpc_fix_t *dp;
	int drowstep;
	int dstripestep;
	jpc_fix_t *dstripestart;
	jpc_fix_t *dvscanstart;

/* s.kang start */
/* jpc_dec_t* dec unnecessary */
#if 0
	/* Avoid compiler warning about unused parameters. */
	dec = 0;
#endif
/* s.kang end */

	one = 1 << bitpos;
	half = one >> 1;
	oneplushalf = one | half;

	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);

	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);

	for (i = 0; i < height; i += 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(4, height - i);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {
			fp = fvscanstart;
			if (vscanlen >= 4 && (!((*fp) & (JPC_SIG | JPC_VISIT |
			  JPC_OTHSIGMSK))) && (fp += frowstep, !((*fp) & (JPC_SIG |
			  JPC_VISIT | JPC_OTHSIGMSK))) && (fp += frowstep, !((*fp) &
			  (JPC_SIG | JPC_VISIT | JPC_OTHSIGMSK))) && (fp += frowstep,
			  !((*fp) & (JPC_SIG | JPC_VISIT | JPC_OTHSIGMSK)))) {
				jpc_mqdec_setcurctx(mqdec, JPC_AGGCTXNO);
				JPC_T1D_GETBIT(mqdec, v, "CLN", "AGG");
				if (!v) {
					continue;
				}
				jpc_mqdec_setcurctx(mqdec, JPC_UCTXNO);
				JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "RL");
				runlen = v;
				JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "RL");
				runlen = (runlen << 1) | v;
				f = *(fp = fvscanstart + frowstep * runlen);
				dp = dvscanstart + drowstep * runlen;
				k = vscanlen - runlen;
				switch (runlen) {
				case 0:
					goto clnpass_partial0;
					break;
				case 1:
					goto clnpass_partial1;
					break;
				case 2:
					goto clnpass_partial2;
					break;
				case 3:
					goto clnpass_partial3;
					break;
				}
			} else {
				f = *(fp = fvscanstart);
				dp = dvscanstart;
				k = vscanlen;
				goto clnpass_full0;
			}

			/* Process first sample in vertical scan. */
			jpc_clnpass_step(f, fp, frowstep, dp, oneplushalf, orient,
			  mqdec, clnpass_full0:, clnpass_partial0:,
			  vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process second sample in vertical scan. */
			f = *fp;
			jpc_clnpass_step(f, fp, frowstep, dp, oneplushalf, orient,
				mqdec, ;, clnpass_partial1:, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process third sample in vertical scan. */
			f = *fp;
			jpc_clnpass_step(f, fp, frowstep, dp, oneplushalf, orient,
				mqdec, ;, clnpass_partial2:, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			/* Process fourth sample in vertical scan. */
			f = *fp;
			jpc_clnpass_step(f, fp, frowstep, dp, oneplushalf, orient,
				mqdec, ;, clnpass_partial3:, 0);
		}
	}

	if (segsymflag) {
		int segsymval;
		segsymval = 0;
		jpc_mqdec_setcurctx(mqdec, JPC_UCTXNO);
		JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "SEGSYM");
		segsymval = (segsymval << 1) | (v & 1);
		JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "SEGSYM");
		segsymval = (segsymval << 1) | (v & 1);
		JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "SEGSYM");
		segsymval = (segsymval << 1) | (v & 1);
		JPC_T1D_GETBITNOSKEW(mqdec, v, "CLN", "SEGSYM");
		segsymval = (segsymval << 1) | (v & 1);
		if (segsymval != 0xa) {
			jas_eprintf("warning: bad segmentation symbol\n");
		}
	}

	return 0;
}
/* s.kang end */

/* s.kang start */
static void* dec_cblks_func( void* p_arg ) {
	jpc_dec_t* p_dec;
	jpc_dec_tile_t* p_tile;
	jpc_dec_tcomp_t *tcomp;
	int compcnt;
	jpc_dec_rlvl_t *rlvl;
	int rlvlcnt;
	jpc_dec_band_t *band;
	int bandcnt;
	jpc_dec_prc_t *prc;
	int prccnt;
	jpc_dec_cblk_t *cblk;
	int cblkcnt;
	jpc_dec_seg_t* p_seg;
	int prc_index;
	int i;

	p_dec = ( ( dec_cblks_arg_t* )p_arg )->p_dec;
	p_tile = ( ( dec_cblks_arg_t* )p_arg )->p_tile;

	for( compcnt = p_dec->numcomps, tcomp = p_tile->tcomps ; compcnt > 0 ; --compcnt, ++tcomp ) {
		for( rlvlcnt = tcomp->numrlvls, rlvl = tcomp->rlvls ; rlvlcnt > 0 ; --rlvlcnt, ++rlvl ) {
			if( !rlvl->bands ) {
				continue;
			}
			for( bandcnt = rlvl->numbands, band = rlvl->bands ; bandcnt > 0 ; --bandcnt, ++band ) {
				if( !band->data ) {
					continue;
				}
				for( prccnt = rlvl->numprcs, prc = band->prcs ; prccnt > 0 ; --prccnt, ++prc ) {
					if( !prc->cblks ) {
						continue;
					}
					for( cblkcnt = prc->numcblks, cblk = prc->cblks ; cblkcnt > 0 ; --cblkcnt, ++cblk ) {
						while( cblk->t2_decoded == 0 ) {
							sched_yield();
						}

						prc_index = -1;
						while( 1 ) {
							for( i = 0 ; i < g_num_spus ; i++ ) {
								if( a_status[i].status == PRC_ERROR ) {
									if( a_t1_dec_cmd_data[i].p_cblk ) {
										while( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head ) {
											p_seg = ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head;
											jpc_seglist_remove( &( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs ), p_seg );
											jpc_seg_destroy( p_seg );
										}
										a_t1_dec_cmd_data[i].p_cblk = NULL;
									}
									pthread_exit( ( void* )-1 );
								}
								else if( a_status[i].status == PRC_IDLE ) {
									if( a_t1_dec_cmd_data[i].p_cblk ) {
										while( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head ) {
											p_seg = ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head;
											jpc_seglist_remove( &( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs ), p_seg );
											jpc_seg_destroy( p_seg );
										}
										a_t1_dec_cmd_data[i].p_cblk = NULL;
									}
									prc_index = i;
									break;
								}
							}
							for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
								if( a_status[i].status == PRC_ERROR ) {
									if( a_t1_dec_cmd_data[i].p_cblk ) {
										while( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head ) {
											p_seg = ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head;
											jpc_seglist_remove( &( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs ), p_seg );
											jpc_seg_destroy( p_seg );
										}
										a_t1_dec_cmd_data[i].p_cblk = NULL;
									}
									pthread_exit( ( void* )-1 );
								}
								else if( a_status[i].status == PRC_IDLE ) {
									if( a_t1_dec_cmd_data[i].p_cblk ) {
										while( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head ) {
											p_seg = ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs.head;
											jpc_seglist_remove( &( ( ( jpc_dec_cblk_t* )( a_t1_dec_cmd_data[i].p_cblk ) )->segs ), p_seg );
											jpc_seg_destroy( p_seg );
										}
										a_t1_dec_cmd_data[i].p_cblk = NULL;
									} 
									prc_index = i;
									break;
								}
							}
							if( prc_index == -1 ) {
								sched_yield();
							}
							else {
								break;
							}
						}

						a_status[prc_index].status = PRC_RUNNING;
						a_t1_dec_cmd_data[prc_index].p_cblk = cblk;
						a_t1_dec_cmd_data[prc_index].cblkctx = p_tile->cp->ccps[tcomp - p_tile->tcomps].cblkctx;
						a_t1_dec_cmd_data[prc_index].c_roishift = p_tile->cp->ccps[tcomp - p_tile->tcomps].roishift;
						a_t1_dec_cmd_data[prc_index].orient = band->orient;
						a_t1_dec_cmd_data[prc_index].numbps = band->numbps;
						a_t1_dec_cmd_data[prc_index].b_roishift = band->roishift;
						a_t1_dec_cmd_data[prc_index].numimsbs = cblk->numimsbs;
						a_t1_dec_cmd_data[prc_index].firstpassno = cblk->firstpassno;
						a_t1_dec_cmd_data[prc_index].addr_seg_head = ( unsigned int )( cblk->segs.head );
						a_t1_dec_cmd_data[prc_index].numrows = cblk->data->numrows_;
						a_t1_dec_cmd_data[prc_index].numcols = cblk->data->numcols_;
						a_t1_dec_cmd_data[prc_index].addr_rows = ( unsigned int )( cblk->data->rows_ );
						a_t1_dec_cmd_data[prc_index].addr_data = ( unsigned int )( cblk->data );
						a_t1_dec_cmd_data[prc_index].dopartial = 1;
						a_t1_dec_cmd_data[prc_index].maxlyrs = JPC_MAXLYRS;
						if( prc_index < g_num_spus ) {
							jpc_send_cmd2spu( prc_index, CMD_T1_DECODE );
						}
						else {
							pthread_mutex_lock( &a_cond_mutex[prc_index - g_num_spus] );
							a_ppu_cmd[prc_index - g_num_spus] = CMD_T1_DECODE;
							pthread_cond_signal( &a_cond_var[prc_index - g_num_spus] );
							pthread_mutex_unlock( &a_cond_mutex[prc_index - g_num_spus] );
						}
					}
				}
			}
		}
	}

	pthread_exit( NULL );
}

static void* tier1_thread_func( void* p_arg ) {
	int id;

	id = ( int )p_arg;

        while( 1 ) {
		pthread_mutex_lock( &a_cond_mutex[id] );
		a_status[g_num_spus + id].status = PRC_IDLE;
		pthread_cond_wait( &a_cond_var[id], &a_cond_mutex[id] );
		pthread_mutex_unlock( &a_cond_mutex[id] );
		if( a_ppu_cmd[id] == CMD_T1_DECODE ) {
			jpc_dec_decodecblk( &a_t1_dec_cmd_data[g_num_spus + id] );
		}
		else if( a_ppu_cmd[id] == CMD_PPU_TERM ) {
			a_status[g_num_spus + id].status = PRC_DONE;
			break;
		}
		else {
			jas_eprintf( "[jpc_t1dec.c:tier1_thread_func()] invalid command\n" );
			a_status[g_num_spus + id].status = PRC_ERROR;
			pthread_exit( ( void* )-1 );
			break;
		}
        }

	pthread_exit( NULL );
}
/* s.kang end */

