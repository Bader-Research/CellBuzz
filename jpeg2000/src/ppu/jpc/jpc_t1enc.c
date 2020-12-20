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
 * Tier 1 Encoder
 *
 * $Id: jpc_t1enc.c,v 1.4 2008/02/06 20:37:15 lrlemini Exp $
 */

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>

#include "jasper/jas_fix.h"
#include "jasper/jas_malloc.h"
#include "jasper/jas_math.h"
/* s.kang start */
#include "jasper/jas_debug.h"
/* s.kang end */

#include "jpc_t1enc.h"
#include "jpc_t1cod.h"
#include "jpc_enc.h"
#include "jpc_cod.h"
#include "jpc_math.h"
/* s.kang start */
#include "jpc_spu_ctrl.h"
/* s.kang end */

/******************************************************************************\
*
\******************************************************************************/

/* s.kang start */
static int jpc_enc_enccblk( t1_enc_cmd_data_t* p_t1_enc_cmd_data, t1_enc_ret_data_t* p_t1_enc_ret_data );
/* s.kang end */

/* s.kang start */
/* do encoding in SPU side */
static int jpc_encsigpass(jpc_mqenc_t *mqenc, int bitpos, int orient, int,
  jas_matrix_t *flags, jas_matrix_t *data, int term, long *nmsedec);

static int jpc_encrefpass(jpc_mqenc_t *mqenc, int bitpos, int, jas_matrix_t *flags,
  jas_matrix_t *data, int term, long *nmsedec);

static int jpc_encclnpass(jpc_mqenc_t *mqenc, int bitpos, int orient, int,
  int, jas_matrix_t *flags, jas_matrix_t *data, int term, long *nmsedec);

static int jpc_encrawsigpass(jpc_bitstream_t *out, int bitpos, int,
  jas_matrix_t *flags, jas_matrix_t *data, int term, long *nmsedec);

static int jpc_encrawrefpass(jpc_bitstream_t *out, int bitpos, int,
  jas_matrix_t *flags, jas_matrix_t *data, int term, long *nmsedec);
/* s.kang end */

/* s.kang start */
static volatile unsigned int a_ppu_cmd[MAX_PPU_THREADS];
static pthread_mutex_t a_cond_mutex[MAX_PPU_THREADS];
static pthread_cond_t a_cond_var[MAX_PPU_THREADS];

typedef struct _enc_cblks_arg_t {
	jpc_enc_t* p_enc;
	jpc_enc_tile_t* p_tile;
} enc_cblks_arg_t;

static void* enc_cblks_func( void* p_arg );
static void* tier1_thread_func( void* p_arg );
/* s.kang end */

/******************************************************************************\
* Code for encoding code blocks.
\******************************************************************************/

/* Encode all of the code blocks associated with the current tile. */
/* s.kang start */
#if 1
int jpc_enc_enccblks(jpc_enc_t *enc)
{
	pthread_attr_t attr;
	struct sched_param param;
	pthread_t enc_cblks_thread;
	pthread_t a_tier1_threads[MAX_PPU_THREADS];
	void* p_pthread_ret;
	enc_cblks_arg_t enc_cblks_arg;
	int i;

	/* status set to PRC_IDLE after threads are spawned, prevent sending command before threads are created */

	for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
		a_status[i].status = PRC_RUNNING;
	}

	/* init scheduling attribute  */

	if( pthread_attr_init( &attr ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_attr_init() failure\n" );
		return -1;
	}

	if( pthread_attr_setschedpolicy( &attr, SCHED_RR ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_attr_setschedpolicy() failure\n" );
		return -1;
	}

	if( pthread_attr_getschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_attr_getshcedparam() failure\n" );
		return -1;
	}

	/* create main code block encoding thread */

	param.sched_priority = ( sched_get_priority_max( SCHED_RR ) + sched_get_priority_min( SCHED_RR ) ) >> 1;
	if( pthread_attr_setschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_attr_setschedparam() failure\n" );
		return -1;
	}

	enc_cblks_arg.p_enc = enc;
	enc_cblks_arg.p_tile = enc->curtile;
	if( pthread_create( &enc_cblks_thread, &attr, enc_cblks_func, ( void* )&enc_cblks_arg ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_create() failure\n" );
		return -1;
	}

	/* create tier-1 encoding thread */

	param.sched_priority = sched_get_priority_min( SCHED_RR );
	if( pthread_attr_setschedparam( &attr, &param ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_attr_setschedparam() failure\n" );
		return -1;
	}

	for( i = 0 ; i < g_num_ppus ; i++ ) {
		pthread_mutex_init( &a_cond_mutex[i], NULL);
		pthread_cond_init ( &a_cond_var[i], NULL);
		if( pthread_create( &a_tier1_threads[i], &attr, tier1_thread_func, ( void* )i ) != 0 ) {
			jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_create() failure\n" );
			return -1;
		}
	}

	/* do work in the thread - to control priority, dynamic priority change fails on the blade */

	/* wait for main code block encoding thread  to finish execution */

	if( pthread_join( enc_cblks_thread, &p_pthread_ret ) != 0 ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_join() failure\n" );
		return -1;
	}

	if( p_pthread_ret != NULL ) {
		jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] enc_cblks_thread error\n" );
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
			jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] pthread_join() failure\n" );
			return -1;
		}

		if( p_pthread_ret != NULL ) {
			jas_eprintf( "jpc_t1enc.c:jpc_enc_enccblks()] tier1_thread error\n" );
			return -1;
		}

		pthread_mutex_destroy( &a_cond_mutex[i] );
		pthread_cond_destroy( &a_cond_var[i] );
	}

	pthread_attr_destroy( &attr );

	/* wait until all the SPEs finish execution */

	if( jpc_wait_for_spus() != 0 ) {
		jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks()] jpc_wait_for_spus() failure\n" );
		return -1;
	}

	/* copy pass data and update code block stream data */

	for( i = 0 ; i < g_num_spus ; i++ ) {
		if( a_t1_enc_cmd_data[i].p_cblk ) {
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numbps = a_t1_enc_ret_data[i].numbps;
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numimsbs = a_t1_enc_ret_data[i].numimsbs;
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numpasses = a_t1_enc_ret_data[i].numpasses;
			if( a_t1_enc_ret_data[i].numpasses != 0 ) {
				( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = jas_malloc( a_t1_enc_ret_data[i].numpasses * sizeof( jpc_enc_pass_t ) );
				memcpy( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes, ( void* )( a_enc_cb[i].addr_t1_enc_pass_data ), a_t1_enc_ret_data[i].numpasses * sizeof( jpc_enc_pass_t ) );
			}
			else {
				( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = 0;
			}
			if( a_t1_enc_ret_data[i].cblk_stream_size > a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
				if( jas_stream_dummy_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) != a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
					jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_dummy_write() failure\n" );
					return -1;
				}
				if( jas_stream_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_p_aux_buf[i], a_t1_enc_ret_data[i].cblk_stream_size - a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) != a_t1_enc_ret_data[i].cblk_stream_size - a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
					jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_write() failure\n" );
					return -1;
				}
				jas_stream_flush( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream );
			}
			else {
				if( jas_stream_dummy_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_t1_enc_ret_data[i].cblk_stream_size ) != a_t1_enc_ret_data[i].cblk_stream_size ) {
					jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_dummy_write() failure\n" );
					return -1;
				}
			}
			a_t1_enc_cmd_data[i].p_cblk = NULL;
		}
	}

	for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
		if( a_t1_enc_cmd_data[i].p_cblk ) {
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numbps = a_t1_enc_ret_data[i].numbps;
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numimsbs = a_t1_enc_ret_data[i].numimsbs;
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numpasses = a_t1_enc_ret_data[i].numpasses;
			( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = ( jpc_enc_pass_t* )( a_t1_enc_ret_data[i].addr_passes );
			jas_stream_flush( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream );
			a_t1_enc_cmd_data[i].p_cblk = NULL;
		}
	}

	return 0;
}
#else 
int jpc_enc_enccblks(jpc_enc_t *enc)
{
	jpc_enc_tcmpt_t *tcmpt;
	jpc_enc_tcmpt_t *endcomps;
	jpc_enc_rlvl_t *lvl;
	jpc_enc_rlvl_t *endlvls;
	jpc_enc_band_t *band;
	jpc_enc_band_t *endbands;
	jpc_enc_cblk_t *cblk;
	jpc_enc_cblk_t *endcblks;
	int i;
	int j;
	int mx;
	int bmx;
	int v;
	jpc_enc_tile_t *tile;
	uint_fast32_t prcno;
	jpc_enc_prc_t *prc;

	tile = enc->curtile;

	endcomps = &tile->tcmpts[tile->numtcmpts];
	for (tcmpt = tile->tcmpts; tcmpt != endcomps; ++tcmpt) {
		endlvls = &tcmpt->rlvls[tcmpt->numrlvls];
		for (lvl = tcmpt->rlvls; lvl != endlvls; ++lvl) {
			if (!lvl->bands) {
				continue;
			}
			endbands = &lvl->bands[lvl->numbands];
			for (band = lvl->bands; band != endbands; ++band) {
				if (!band->data) {
					continue;
				}
				for (prcno = 0, prc = band->prcs; prcno < ( uint_fast32_t )( lvl->numprcs ) ; ++prcno, ++prc) {
					if (!prc->cblks) {
						continue;
					}
					bmx = 0;
					endcblks = &prc->cblks[prc->numcblks];
					for (cblk = prc->cblks; cblk != endcblks; ++cblk) {
						mx = 0;
						for (i = 0; i < jas_matrix_numrows(cblk->data); ++i) {
							for (j = 0; j < jas_matrix_numcols(cblk->data); ++j) {
								v = abs(jas_matrix_get(cblk->data, i, j));
								if (v > mx) {
									mx = v;
								}
							}
						}
						if (mx > bmx) {
							bmx = mx;
						}
						cblk->numbps = JAS_MAX(jpc_firstone(mx) + 1 - JPC_NUMEXTRABITS, 0);
					}

					for (cblk = prc->cblks; cblk != endcblks; ++cblk) {
						cblk->numimsbs = band->numbps - cblk->numbps;
						assert(cblk->numimsbs >= 0);
					}

					for (cblk = prc->cblks; cblk != endcblks; ++cblk) {
						if (jpc_enc_enccblk(enc, cblk->stream, tcmpt, band, cblk)) {
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
int getthebyte(jas_stream_t *in, long off)
{
	int c;
	long oldpos;
	oldpos = jas_stream_tell(in);
	assert(oldpos >= 0);
	jas_stream_seek(in, off, SEEK_SET);
	c = jas_stream_peekc(in);
	jas_stream_seek(in, oldpos, SEEK_SET);
	return c;
}
/* s.kang end */

/* Encode a single code block. */
/* s.kang start */
static int jpc_enc_enccblk( t1_enc_cmd_data_t* p_t1_enc_cmd_data, t1_enc_ret_data_t* p_t1_enc_ret_data ) {
	jpc_enc_pass_t *pass;
	jpc_enc_pass_t *endpasses;
	int bitpos;
	int n;
	int adjust;
	int ret;
	int passtype;
	int t;
	jpc_bitstream_t *bout;
	jpc_enc_pass_t *termpass;
	int vcausal;
	int segsym;
	int termmode;
	int c;
	int mx;
	int v;
	jpc_mqenc_t *mqenc;
	jas_matrix_t* p_data;
	jas_matrix_t* p_flags;
	jpc_enc_pass_t* p_passes;
	jas_stream_t* p_stream = 0;
	int i;
	int j;

	bout = 0;

	if (!(p_flags = jas_matrix_create(p_t1_enc_cmd_data->numrows + 2, p_t1_enc_cmd_data->numcols + 2))) {
		jas_eprintf( "[jpc_t1enc.c:jpd_enc_enccblk()] jas_matrix_create failure( %d, %d )\n", p_t1_enc_cmd_data->numrows + 2, p_t1_enc_cmd_data->numcols + 2 );
		return -1;
	}

	p_data = ( jas_matrix_t* )( p_t1_enc_cmd_data->addr_data );
	if( p_data == NULL ) {
		jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblk()] jpc_get_cblk_matrix failure\n" );
		if( p_flags ) {
			jas_matrix_destroy( p_flags );
		}
		return -1;
	}

	mx = 0;
	for ( i = 0 ; i < p_t1_enc_cmd_data->numrows ; ++i ) {
		for ( j = 0 ; j < p_t1_enc_cmd_data->numcols ; ++j ) {
			v = abs( p_data->rows_[i][j] );
			if( v > mx ) {
				mx = v;
			}
		}
	}
	p_t1_enc_ret_data->numbps = JAS_MAX(jpc_firstone(mx) + 1 - JPC_NUMEXTRABITS, 0);
	p_t1_enc_ret_data->numimsbs = p_t1_enc_cmd_data->numbps - p_t1_enc_ret_data->numbps;
	assert( p_t1_enc_ret_data->numimsbs >= 0 );

	p_stream = ( jas_stream_t* )( p_t1_enc_cmd_data->addr_stream );
	assert( p_stream );
	mqenc = jpc_mqenc_create( JPC_NUMCTXS, p_stream );
	assert( mqenc );
	jpc_mqenc_setctxs( mqenc, JPC_NUMCTXS, jpc_mqctxs );

	p_t1_enc_ret_data->numpasses = ( p_t1_enc_ret_data->numbps > 0 ) ? ( 3 * p_t1_enc_ret_data->numbps - 2 ) : 0;
	assert( p_t1_enc_ret_data->numpasses <= JPC_MAX_PASSES );
	if( p_t1_enc_ret_data->numpasses > 0 ) {
		p_passes = jas_malloc( p_t1_enc_ret_data->numpasses * sizeof( jpc_enc_pass_t ) );
		assert( p_passes );
	} else {
		p_passes = NULL;
	}
	p_t1_enc_ret_data->addr_passes = ( unsigned int )p_passes;
	endpasses = &( p_passes[p_t1_enc_ret_data->numpasses] );
	for( pass = p_passes ; pass != endpasses ; ++pass ) {
		pass->start = 0;
		pass->end = 0;
		pass->term = JPC_ISTERMINATED( pass - p_passes, 0, p_t1_enc_ret_data->numpasses, ( p_t1_enc_cmd_data->cblksty & JPC_COX_TERMALL ) != 0, ( p_t1_enc_cmd_data->cblksty & JPC_COX_LAZY ) != 0 );
		pass->type = JPC_SEGTYPE( pass - p_passes, 0, ( p_t1_enc_cmd_data->cblksty & JPC_COX_LAZY ) != 0 );
		pass->lyrno = -1;
		if( pass == endpasses - 1 ) {
			assert( pass->term == 1 );
			pass->term = 1;
		}
	}

	bitpos = p_t1_enc_ret_data->numbps - 1;
	pass = p_passes;
	n = p_t1_enc_ret_data->numpasses;
	while (--n >= 0) {

		if (pass->type == JPC_SEG_MQ) {
			/* NOP */
		} else {
			assert( pass->type == JPC_SEG_RAW );
			if (!bout) {
				bout = jpc_bitstream_sopen( p_stream, "w" );
				assert( bout );
			}
		}

		passtype = (pass - p_passes + 2) % 3;
		pass->start = jas_stream_tell(p_stream);
		assert( bitpos >= 0 );
		vcausal = (p_t1_enc_cmd_data->cblksty & JPC_COX_VSC) != 0;
		segsym = (p_t1_enc_cmd_data->cblksty & JPC_COX_SEGSYM) != 0;
		if (pass->term) {
			termmode = ((p_t1_enc_cmd_data->cblksty & JPC_COX_PTERM) ?
			  JPC_MQENC_PTERM : JPC_MQENC_DEFTERM) + 1;
		} else {
			termmode = 0;
		}
		switch (passtype) {
		case JPC_SIGPASS:
			ret = (pass->type == JPC_SEG_MQ) ? jpc_encsigpass(mqenc,
			  bitpos, p_t1_enc_cmd_data->orient, vcausal, p_flags,
			  p_data, termmode, &pass->nmsedec) :
			  jpc_encrawsigpass(bout, bitpos, vcausal, p_flags,
			  p_data, termmode, &pass->nmsedec);
			break;
		case JPC_REFPASS:
			ret = (pass->type == JPC_SEG_MQ) ? jpc_encrefpass(mqenc,
			  bitpos, vcausal, p_flags, p_data, termmode,
			  &pass->nmsedec) : jpc_encrawrefpass(bout, bitpos,
			  vcausal, p_flags, p_data, termmode,
			  &pass->nmsedec);
			break;
		case JPC_CLNPASS:
			assert( pass->type == JPC_SEG_MQ );
			ret = jpc_encclnpass(mqenc, bitpos, p_t1_enc_cmd_data->orient,
			  vcausal, segsym, p_flags, p_data, termmode,
			  &pass->nmsedec);
			break;
		default:
			assert( 0 );
			break;
		}

		if (pass->type == JPC_SEG_MQ) {
			if (pass->term) {
				jpc_mqenc_init(mqenc);
			}
			jpc_mqenc_getstate(mqenc, &pass->mqencstate);
			pass->end = jas_stream_tell(p_stream);
			if (p_t1_enc_cmd_data->cblksty & JPC_COX_RESET) {
				jpc_mqenc_setctxs(mqenc, JPC_NUMCTXS, jpc_mqctxs);
			}
		} else {
			if (pass->term) {
				if (jpc_bitstream_pending(bout)) {
					jpc_bitstream_outalign(bout, 0x2a);
				}
				jpc_bitstream_close(bout);
				bout = 0;
				pass->end = jas_stream_tell(p_stream);
			} else {
				pass->end = jas_stream_tell(p_stream) +
				  jpc_bitstream_pending(bout);
/* NOTE - This will not work.  need to adjust by # of pending output bytes */
			}
		}

		pass->wmsedec = jpc_fixtodbl(p_t1_enc_cmd_data->c_synweight) *
		  jpc_fixtodbl(p_t1_enc_cmd_data->c_synweight) *
		  jpc_fixtodbl(p_t1_enc_cmd_data->b_synweight) *
		  jpc_fixtodbl(p_t1_enc_cmd_data->b_synweight) *
		  jpc_fixtodbl(p_t1_enc_cmd_data->absstepsize) * jpc_fixtodbl(p_t1_enc_cmd_data->absstepsize) *
		  ((double) (1 << bitpos)) * ((double)(1 << bitpos)) *
		  jpc_fixtodbl(pass->nmsedec);
		pass->cumwmsedec = pass->wmsedec;
		if (pass != p_passes) {
			pass->cumwmsedec += pass[-1].cumwmsedec;
		}
		if (passtype == JPC_CLNPASS) {
			--bitpos;
		}
		++pass;
	}

	n = 0;
	endpasses = &( p_passes[p_t1_enc_ret_data->numpasses] );
	for (pass = p_passes; pass != endpasses; ++pass) {
		if (pass->start < n) {
			pass->start = n;
		}
		if (pass->end < n) {
			pass->end = n;
		}
		if (!pass->term) {
			termpass = pass;
			while (termpass - pass < p_t1_enc_ret_data->numpasses &&
			  !termpass->term) {
				++termpass;
			}
			if (pass->type == JPC_SEG_MQ) {
				t = (pass->mqencstate.lastbyte == 0xff) ? 1 : 0;
				if (pass->mqencstate.ctreg >= 5) {
					adjust = 4 + t;
				} else {
					adjust = 5 + t;
				}
				pass->end += adjust;
			}
			if (pass->end > termpass->end) {
				pass->end = termpass->end;
			}
			if ((c = getthebyte(p_stream, pass->end - 1)) == EOF) {
				jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblk()] getthebyte failure, abort() invoked\n" );
				abort();
			}
			if (c == 0xff) {
				++pass->end;
			}
			n = JAS_MAX(n, pass->end);
		} else {
			n = JAS_MAX(n, pass->end);
		}
	}

	p_t1_enc_ret_data->cblk_stream_size = jas_stream_tell( p_stream );

	if( bout ) {
		jpc_bitstream_close( bout );
	}
	if( mqenc ) {
		jpc_mqenc_destroy( mqenc );
	}
	if( p_flags ) {
		jas_matrix_destroy( p_flags );
	}

	return 0;
}

/******************************************************************************\
* Code for significance pass.
\******************************************************************************/

#define	sigpass_step(fp, frowstep, dp, bitpos, one, nmsedec, orient, mqenc, vcausalflag) \
{ \
	int f; \
	int v; \
	f = *(fp); \
	if ((f & JPC_OTHSIGMSK) && !(f & (JPC_SIG | JPC_VISIT))) { \
		v = (abs(*(dp)) & (one)) ? 1 : 0; \
		jpc_mqenc_setcurctx(mqenc, JPC_GETZCCTXNO(f, (orient))); \
		jpc_mqenc_putbit(mqenc, v); \
		if (v) { \
			*(nmsedec) += JPC_GETSIGNMSEDEC(abs(*(dp)), (bitpos) + JPC_NUMEXTRABITS); \
			v = ((*(dp) < 0) ? 1 : 0); \
			jpc_mqenc_setcurctx(mqenc, JPC_GETSCCTXNO(f)); \
			jpc_mqenc_putbit(mqenc, v ^ JPC_GETSPB(f)); \
			JPC_UPDATEFLAGS4(fp, frowstep, v, vcausalflag); \
			*(fp) |= JPC_SIG; \
		} \
		*(fp) |= JPC_VISIT; \
	} \
}

static int jpc_encsigpass(jpc_mqenc_t *mqenc, int bitpos, int orient, int vcausalflag,
  jas_matrix_t *flags, jas_matrix_t *data, int term, long *nmsedec)
{
	int i;
	int j;
	int one;
	int vscanlen;
	int width;
	int height;
	int frowstep;
	int drowstep;
	int fstripestep;
	int dstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *dstripestart;
	jpc_fix_t *fp;
	jpc_fix_t *dp;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dvscanstart;
	int k;

	*nmsedec = 0;
	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << (bitpos + JPC_NUMEXTRABITS);

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

			sigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, orient, mqenc, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			sigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, orient, mqenc, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			sigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, orient, mqenc, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			sigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, orient, mqenc, 0);

		}
	}

	if (term) {
		jpc_mqenc_flush(mqenc, term - 1);
	}

	return jpc_mqenc_error(mqenc) ? (-1) : 0;
}

#define	rawsigpass_step(fp, frowstep, dp, bitpos, one, nmsedec, out, vcausalflag) \
{ \
	jpc_fix_t f = *(fp); \
	jpc_fix_t v; \
	if ((f & JPC_OTHSIGMSK) && !(f & (JPC_SIG | JPC_VISIT))) { \
		v = (abs(*(dp)) & (one)) ? 1 : 0; \
		if ((jpc_bitstream_putbit((out), v)) == EOF) { \
			return -1; \
		} \
		if (v) { \
			*(nmsedec) += JPC_GETSIGNMSEDEC(abs(*(dp)), (bitpos) + JPC_NUMEXTRABITS); \
			v = ((*(dp) < 0) ? 1 : 0); \
			if (jpc_bitstream_putbit(out, v) == EOF) { \
				return -1; \
			} \
			JPC_UPDATEFLAGS4(fp, frowstep, v, vcausalflag); \
			*(fp) |= JPC_SIG; \
		} \
		*(fp) |= JPC_VISIT; \
	} \
}

static int jpc_encrawsigpass(jpc_bitstream_t *out, int bitpos, int vcausalflag, jas_matrix_t *flags,
  jas_matrix_t *data, int term, long *nmsedec)
{
	int i;
	int j;
	int k;
	int one;
	int vscanlen;
	int width;
	int height;
	int frowstep;
	int drowstep;
	int fstripestep;
	int dstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *dstripestart;
	jpc_fix_t *fp;
	jpc_fix_t *dp;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dvscanstart;

	*nmsedec = 0;
	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << (bitpos + JPC_NUMEXTRABITS);

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

			rawsigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, out, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			rawsigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, out, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			rawsigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, out, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

			rawsigpass_step(fp, frowstep, dp, bitpos, one,
			  nmsedec, out, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;

		}
	}

	if (term) {
		jpc_bitstream_outalign(out, 0x2a);
	}

	return 0;
}

/******************************************************************************\
* Code for refinement pass.
\******************************************************************************/

#define	refpass_step(fp, dp, bitpos, one, nmsedec, mqenc, vcausalflag) \
{ \
	int v; \
	if (((*(fp)) & (JPC_SIG | JPC_VISIT)) == JPC_SIG) { \
		(d) = *(dp); \
		*(nmsedec) += JPC_GETREFNMSEDEC(abs(d), (bitpos) + JPC_NUMEXTRABITS); \
		jpc_mqenc_setcurctx((mqenc), JPC_GETMAGCTXNO(*(fp))); \
		v = (abs(d) & (one)) ? 1 : 0; \
		jpc_mqenc_putbit((mqenc), v); \
		*(fp) |= JPC_REFINE; \
	} \
}

/* s.kang start */
/* to avoid warning */
#if 1
static int jpc_encrefpass(jpc_mqenc_t *mqenc, int bitpos, int vcausalflag __attribute__ ( ( __unused__ ) ), jas_matrix_t *flags, jas_matrix_t *data,
  int term, long *nmsedec)
#else
static int jpc_encrefpass(jpc_mqenc_t *mqenc, int bitpos, int vcausalflag, jas_matrix_t *flags, jas_matrix_t *data,
  int term, long *nmsedec)
#endif
/* s.kang end */
{
	int i;
	int j;
	int one;
	int vscanlen;
	int d;
	int width;
	int height;
	int frowstep;
	int drowstep;
	int fstripestep;
	int dstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *dstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dvscanstart;
	jpc_fix_t *dp;
	jpc_fix_t *fp;
int k;

	*nmsedec = 0;
	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << (bitpos + JPC_NUMEXTRABITS);

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

			refpass_step(fp, dp, bitpos, one, nmsedec,
			  mqenc, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			refpass_step(fp, dp, bitpos, one, nmsedec,
			  mqenc, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			refpass_step(fp, dp, bitpos, one, nmsedec,
			  mqenc, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			refpass_step(fp, dp, bitpos, one, nmsedec,
			  mqenc, 0);

		}
	}

	if (term) {
		jpc_mqenc_flush(mqenc, term - 1);
	}

	return jpc_mqenc_error(mqenc) ? (-1) : 0;
}

#define	rawrefpass_step(fp, dp, bitpos, one, nmsedec, out, vcausalflag) \
{ \
	jpc_fix_t d; \
	jpc_fix_t v; \
	if (((*(fp)) & (JPC_SIG | JPC_VISIT)) == JPC_SIG) { \
		d = *(dp); \
		*(nmsedec) += JPC_GETREFNMSEDEC(abs(d), (bitpos) + JPC_NUMEXTRABITS); \
		v = (abs(d) & (one)) ? 1 : 0; \
		if (jpc_bitstream_putbit((out), v) == EOF) { \
			return -1; \
		} \
		*(fp) |= JPC_REFINE; \
	} \
}

/* s.kang start */
/* to avoid warning */
#if 1
static int jpc_encrawrefpass(jpc_bitstream_t *out, int bitpos, int vcausalflag __attribute__ ( ( __unused__ ) ), jas_matrix_t *flags,
  jas_matrix_t *data, int term, long *nmsedec)
#else
static int jpc_encrawrefpass(jpc_bitstream_t *out, int bitpos, int vcausalflag, jas_matrix_t *flags,
  jas_matrix_t *data, int term, long *nmsedec)
#endif
/* s.kang end */
{
	int i;
	int j;
	int k;
	int one;
	int vscanlen;
	int width;
	int height;
	int frowstep;
	int drowstep;
	int fstripestep;
	int dstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *dstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dvscanstart;
	jpc_fix_t *dp;
	jpc_fix_t *fp;

	*nmsedec = 0;
	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << (bitpos + JPC_NUMEXTRABITS);

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

			rawrefpass_step(fp, dp, bitpos, one, nmsedec,
			  out, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			rawrefpass_step(fp, dp, bitpos, one, nmsedec,
			  out, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			rawrefpass_step(fp, dp, bitpos, one, nmsedec,
			  out, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			rawrefpass_step(fp, dp, bitpos, one, nmsedec,
			  out, vcausalflag);

		}
	}

	if (term) {
		jpc_bitstream_outalign(out, 0x2a);
	}

	return 0;
}

/******************************************************************************\
* Code for cleanup pass.
\******************************************************************************/

#define	clnpass_step(fp, frowstep, dp, bitpos, one, orient, nmsedec, mqenc, label1, label2, vcausalflag) \
{ \
	int f; \
	int v; \
label1 \
	f = *(fp); \
	if (!(f & (JPC_SIG | JPC_VISIT))) { \
		jpc_mqenc_setcurctx(mqenc, JPC_GETZCCTXNO(f, (orient))); \
		v = (abs(*(dp)) & (one)) ? 1 : 0; \
		jpc_mqenc_putbit((mqenc), v); \
		if (v) { \
label2 \
			f = *(fp); \
			/* Coefficient is significant. */ \
			*(nmsedec) += JPC_GETSIGNMSEDEC(abs(*(dp)), (bitpos) + JPC_NUMEXTRABITS); \
			jpc_mqenc_setcurctx((mqenc), JPC_GETSCCTXNO(f)); \
			v = ((*(dp) < 0) ? 1 : 0); \
			jpc_mqenc_putbit((mqenc), v ^ JPC_GETSPB(f)); \
			JPC_UPDATEFLAGS4((fp), (frowstep), v, vcausalflag); \
			*(fp) |= JPC_SIG; \
		} \
	} \
	*(fp) &= ~JPC_VISIT; \
}

static int jpc_encclnpass(jpc_mqenc_t *mqenc, int bitpos, int orient, int vcausalflag, int segsymflag, jas_matrix_t *flags,
  jas_matrix_t *data, int term, long *nmsedec)
{
	int i;
	int j;
	int k;
	int vscanlen;
	int v;
	int runlen;
	jpc_fix_t *fp;
	int width;
	int height;
	jpc_fix_t *dp;
	int one;
	int frowstep;
	int drowstep;
	int fstripestep;
	int dstripestep;
	jpc_fix_t *fstripestart;
	jpc_fix_t *dstripestart;
	jpc_fix_t *fvscanstart;
	jpc_fix_t *dvscanstart;

	*nmsedec = 0;
	width = jas_matrix_numcols(data);
	height = jas_matrix_numrows(data);
	frowstep = jas_matrix_rowstep(flags);
	drowstep = jas_matrix_rowstep(data);
	fstripestep = frowstep << 2;
	dstripestep = drowstep << 2;

	one = 1 << (bitpos + JPC_NUMEXTRABITS);

	fstripestart = jas_matrix_getref(flags, 1, 1);
	dstripestart = jas_matrix_getref(data, 0, 0);
	for (i = height; i > 0; i -= 4, fstripestart += fstripestep,
	  dstripestart += dstripestep) {
		fvscanstart = fstripestart;
		dvscanstart = dstripestart;
		vscanlen = JAS_MIN(i, 4);
		for (j = width; j > 0; --j, ++fvscanstart, ++dvscanstart) {

			fp = fvscanstart;
			if (vscanlen >= 4 && !((*fp) & (JPC_SIG | JPC_VISIT |
			  JPC_OTHSIGMSK)) && (fp += frowstep, !((*fp) & (JPC_SIG |
			  JPC_VISIT | JPC_OTHSIGMSK))) && (fp += frowstep, !((*fp) &
			  (JPC_SIG | JPC_VISIT | JPC_OTHSIGMSK))) && (fp += frowstep,
			  !((*fp) & (JPC_SIG | JPC_VISIT | JPC_OTHSIGMSK)))) {
				dp = dvscanstart;
				for (k = 0; k < vscanlen; ++k) {
					v = (abs(*dp) & one) ? 1 : 0;
					if (v) {
						break;
					}
					dp += drowstep;
				}
				runlen = k;
				if (runlen >= 4) {
					jpc_mqenc_setcurctx(mqenc, JPC_AGGCTXNO);
					jpc_mqenc_putbit(mqenc, 0);
					continue;
				}
				jpc_mqenc_setcurctx(mqenc, JPC_AGGCTXNO);
				jpc_mqenc_putbit(mqenc, 1);
				jpc_mqenc_setcurctx(mqenc, JPC_UCTXNO);
				jpc_mqenc_putbit(mqenc, runlen >> 1);
				jpc_mqenc_putbit(mqenc, runlen & 1);
				fp = fvscanstart + frowstep * runlen;
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
				runlen = 0;
				fp = fvscanstart;
				dp = dvscanstart;
				k = vscanlen;
				goto clnpass_full0;
			}
			clnpass_step(fp, frowstep, dp, bitpos, one,
			  orient, nmsedec, mqenc, clnpass_full0:, clnpass_partial0:, vcausalflag);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			clnpass_step(fp, frowstep, dp, bitpos, one,
				orient, nmsedec, mqenc, ;, clnpass_partial1:, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			clnpass_step(fp, frowstep, dp, bitpos, one,
				orient, nmsedec, mqenc, ;, clnpass_partial2:, 0);
			if (--k <= 0) {
				continue;
			}
			fp += frowstep;
			dp += drowstep;
			clnpass_step(fp, frowstep, dp, bitpos, one,
				orient, nmsedec, mqenc, ;, clnpass_partial3:, 0);
		}
	}

	if (segsymflag) {
		jpc_mqenc_setcurctx(mqenc, JPC_UCTXNO);
		jpc_mqenc_putbit(mqenc, 1);
		jpc_mqenc_putbit(mqenc, 0);
		jpc_mqenc_putbit(mqenc, 1);
		jpc_mqenc_putbit(mqenc, 0);
	}

	if (term) {
		jpc_mqenc_flush(mqenc, term - 1);
	}

	return jpc_mqenc_error(mqenc) ? (-1) : 0;
}
/* s.kang end */

/* s.kang start */
static void* enc_cblks_func( void* p_arg ) {
	jpc_enc_t* p_enc;
	jpc_enc_tile_t* p_tile;
	jpc_enc_tcmpt_t *tcmpt;
	jpc_enc_tcmpt_t *endcomps;
	jpc_enc_rlvl_t *lvl;
	jpc_enc_rlvl_t *endlvls;
	jpc_enc_band_t *band;
	jpc_enc_band_t *endbands;
	jpc_enc_cblk_t *cblk;
	jpc_enc_cblk_t *endcblks;
	uint_fast32_t prcno;
	jpc_enc_prc_t *prc;
	int prc_index;
	int i;

	p_enc = ( ( enc_cblks_arg_t* )p_arg )->p_enc;
	p_tile = ( ( enc_cblks_arg_t* )p_arg )->p_tile;

	endcomps = &p_tile->tcmpts[p_tile->numtcmpts];
	for (tcmpt = p_tile->tcmpts; tcmpt != endcomps; ++tcmpt) {
		endlvls = &tcmpt->rlvls[tcmpt->numrlvls];
		for (lvl = tcmpt->rlvls; lvl != endlvls; ++lvl) {
			if (!lvl->bands) {
				continue;
			}
			endbands = &lvl->bands[lvl->numbands];
			for (band = lvl->bands; band != endbands; ++band) {
				if (!band->data) {
					continue;
				}
				for (prcno = 0, prc = band->prcs; prcno < ( uint_fast32_t )( lvl->numprcs ) ; ++prcno, ++prc) {
					if (!prc->cblks) {
						continue;
					}
					endcblks = &prc->cblks[prc->numcblks];
					for (cblk = prc->cblks; cblk != endcblks; ++cblk) {
						prc_index = -1;
						while( 1 ) {
							for( i = 0 ; i < g_num_spus ; i++ ) {
								if( a_status[i].status == PRC_ERROR ) {
									jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] spu returned error\n" );
									a_t1_enc_cmd_data[i].p_cblk = NULL;
									pthread_exit( ( void* )-1 );
								}
								else if( a_status[i].status == PRC_IDLE ) {
									/* copy pass data and update code block stream data */
									if( a_t1_enc_cmd_data[i].p_cblk ) {
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numbps = a_t1_enc_ret_data[i].numbps;
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numimsbs = a_t1_enc_ret_data[i].numimsbs;
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numpasses = a_t1_enc_ret_data[i].numpasses;
										if( a_t1_enc_ret_data[i].numpasses != 0 ) {
											( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = jas_malloc( a_t1_enc_ret_data[i].numpasses * sizeof( jpc_enc_pass_t ) );
											memcpy( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes, ( void* )( a_enc_cb[i].addr_t1_enc_pass_data ), a_t1_enc_ret_data[i].numpasses * sizeof( jpc_enc_pass_t ) );
										}
										else {
											( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = 0;
										}
										if( a_t1_enc_ret_data[i].cblk_stream_size > a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
											if( jas_stream_dummy_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) != a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
												jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_dummy_write() failure\n" );
												pthread_exit( ( void* )-1 );
											}
											if( jas_stream_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_p_aux_buf[i], a_t1_enc_ret_data[i].cblk_stream_size - a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) != a_t1_enc_ret_data[i].cblk_stream_size - a_t1_enc_cmd_data[i].cblk_stream_obj_bufsize ) {
												jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_write() failure\n" );
												pthread_exit( ( void* )-1 );
											}
											jas_stream_flush( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream );
										}
										else {
											if( jas_stream_dummy_write( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream, a_t1_enc_ret_data[i].cblk_stream_size ) != a_t1_enc_ret_data[i].cblk_stream_size ) {
												jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] jas_stream_dummy_write() failure\n" );
												pthread_exit( ( void* )-1 );
											}
										}
										a_t1_enc_cmd_data[i].p_cblk = NULL;
									}
									prc_index = i;
									break;
								}
							}
							for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
								if( a_status[i].status == PRC_ERROR ) {
									jas_eprintf( "[jpc_t1enc.c:jpc_enc_enccblks] ppu returned error\n" );
									a_t1_enc_cmd_data[i].p_cblk = NULL;
									pthread_exit( ( void* )-1 );
								}
								else if( a_status[i].status == PRC_IDLE ) {
									/* copy pass data and flush code block stream data */
									if( a_t1_enc_cmd_data[i].p_cblk ) {
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numbps = a_t1_enc_ret_data[i].numbps;
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numimsbs = a_t1_enc_ret_data[i].numimsbs;
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->numpasses = a_t1_enc_ret_data[i].numpasses;
										( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->passes = ( jpc_enc_pass_t* )( a_t1_enc_ret_data[i].addr_passes );
										jas_stream_flush( ( ( jpc_enc_cblk_t* )( a_t1_enc_cmd_data[i].p_cblk ) )->stream );
										a_t1_enc_cmd_data[i].p_cblk = NULL;
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
						a_t1_enc_cmd_data[prc_index].p_cblk = cblk;

						a_t1_enc_cmd_data[prc_index].cblksty = tcmpt->cblksty;
						a_t1_enc_cmd_data[prc_index].c_synweight = tcmpt->synweight;
						a_t1_enc_cmd_data[prc_index].numbps = band->numbps;
						a_t1_enc_cmd_data[prc_index].orient = band->orient;
						a_t1_enc_cmd_data[prc_index].b_synweight = band->synweight;
						a_t1_enc_cmd_data[prc_index].absstepsize = band->absstepsize;
						a_t1_enc_cmd_data[prc_index].numrows = jas_matrix_numrows( cblk->data );
						a_t1_enc_cmd_data[prc_index].numcols = jas_matrix_numcols( cblk->data );
						a_t1_enc_cmd_data[prc_index].addr_rows = ( unsigned int )( cblk->data->rows_ );
						a_t1_enc_cmd_data[prc_index].addr_data = ( unsigned int )( cblk->data );
						cblk->stream = jas_stream_memopen_dma_support();
						a_t1_enc_cmd_data[prc_index].addr_cblk_stream_obj_buf = ( unsigned int )( ( ( jas_stream_memobj_t* )( cblk->stream->obj_ ) )->buf_ );
						a_t1_enc_cmd_data[prc_index].cblk_stream_obj_bufsize = ( ( ( jas_stream_memobj_t* )( cblk->stream->obj_ ) )->bufsize_ );
						a_t1_enc_cmd_data[prc_index].addr_aux_buf = ( unsigned int )a_p_aux_buf[prc_index];
						a_t1_enc_cmd_data[prc_index].aux_bufsize = DMA_AUX_BUF_SIZE;
						a_t1_enc_cmd_data[prc_index].addr_stream = ( unsigned int )( cblk->stream );
						if( prc_index < g_num_spus ) {
							jpc_send_cmd2spu( prc_index, CMD_T1_ENCODE );
						}
						else {
							pthread_mutex_lock( &a_cond_mutex[prc_index - g_num_spus] );
							a_ppu_cmd[prc_index - g_num_spus] = CMD_T1_ENCODE;
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
		if( a_ppu_cmd[id] == CMD_T1_ENCODE ) {
			jpc_enc_enccblk( &a_t1_enc_cmd_data[g_num_spus + id], &a_t1_enc_ret_data[g_num_spus + id] );
		}
		else if( a_ppu_cmd[id] == CMD_PPU_TERM ) {
			a_status[g_num_spus + id].status = PRC_DONE;
			break;
		}
		else {
			jas_eprintf( "[jpc_t1enc.c:tier1_thread_func()] invalid command\n" );
			a_status[g_num_spus + id].status = PRC_ERROR;
			pthread_exit( ( void* )-1 );
			break;
		}
	}

	pthread_exit( NULL );
}
/* s.kang end */

