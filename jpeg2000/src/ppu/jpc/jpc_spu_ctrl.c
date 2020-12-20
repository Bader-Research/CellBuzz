/*
 * Copyright (c) 1999-2000, Image Power, Inc. and the University of
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
 * spu command data
 *
 * $Id: jpc_spu_ctrl.c,v 1.7 2008/06/23 16:01:33 lrlemini Exp $
 */

/* s.kang start */
/******************************************************************************\
* Includes.
\******************************************************************************/

#include <sched.h>
#include <pthread.h>

#include <libspe2.h>

#include "mutex.h"
#include "mutex_init.h"
#include "mutex_lock.h"
#include "mutex_unlock.h"

#include "jasper/jas_debug.h"
#include "jasper/jas_malloc.h"

#include "jpc_enc.h"
#include "jpc_spu_ctrl.h"
#include "jpc_t1cod.h"

#include "jasper-cell.h"

/******************************************************************************\
* Local function prototypes.
\******************************************************************************/

static void* ppu_pthread_function( void* p_arg );

/******************************************************************************\
* global data
\******************************************************************************/

extern spe_program_handle_t jpc_enc_spu;
extern spe_program_handle_t jpc_dec_spu;

enc_control_block_t a_enc_cb[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
dec_control_block_t a_dec_cb[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
volatile prc_status_t a_status[MAX_SPU_THREADS + MAX_PPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
ppu_pthread_data_t a_pthread_data[MAX_SPU_THREADS];

enc_pre_proc_with_mct_cmd_data_t a_enc_pre_proc_with_mct_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
dec_post_proc_with_imct_cmd_data_t a_dec_post_proc_with_imct_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
enc_pre_proc_no_mct_cmd_data_t a_enc_pre_proc_no_mct_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
dec_post_proc_no_imct_cmd_data_t a_dec_post_proc_no_imct_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

fwd_dwt_row_cmd_data_t a_fwd_dwt_row_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
inv_dwt_row_cmd_data_t a_inv_dwt_row_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
fwd_dwt_colgrp_cmd_data_t a_fwd_dwt_colgrp_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
inv_dwt_colgrp_cmd_data_t a_inv_dwt_colgrp_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

quant_cmd_data_t a_quant_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
dequant_cmd_data_t a_dequant_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
quant_ret_data_t a_quant_ret_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

t1_enc_cmd_data_t a_t1_enc_cmd_data[MAX_SPU_THREADS + MAX_PPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
t1_dec_cmd_data_t a_t1_dec_cmd_data[MAX_SPU_THREADS + MAX_PPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
t1_enc_ret_data_t a_t1_enc_ret_data[MAX_SPU_THREADS + MAX_PPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

rd_cmpt_cmd_data_t a_rd_cmpt_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
wr_cmpt_cmd_data_t a_wr_cmpt_cmd_data[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

void* a_p_aux_buf[MAX_SPU_THREADS];

int g_num_spus = 0;
int g_num_ppus = 0;

/******************************************************************************\
* static data
\******************************************************************************/

static unsigned int mutex_var;

/******************************************************************************\
* functions
\******************************************************************************/

int jpc_create_spu_threads( int enc ) {
	mutex_ea_t mutex_ea;
	int ret;
	int i;

	/* init mutex */
	
	mutex_ea = ( mutex_ea_t )( uintptr_t )( &mutex_var );
	_mutex_init( mutex_ea );

	for( i = 0 ; i < g_num_spus ; i++ ) {
		/* set control block data */

		if( enc ==  1 ) {
			a_enc_cb[i].spu_id = i;
			a_enc_cb[i].addr_prc_status = ( unsigned int )&( a_status[i] );
			a_enc_cb[i].mutex_ea = mutex_ea;
			a_enc_cb[i].addr_enc_pre_proc_with_mct_cmd_data = ( unsigned int )&( a_enc_pre_proc_with_mct_cmd_data[i] );
			a_enc_cb[i].addr_enc_pre_proc_no_mct_cmd_data = ( unsigned int )&( a_enc_pre_proc_no_mct_cmd_data[i] );
			a_enc_cb[i].addr_fwd_dwt_row_cmd_data = ( unsigned int )&( a_fwd_dwt_row_cmd_data[i] );
			a_enc_cb[i].addr_fwd_dwt_colgrp_cmd_data = ( unsigned int )&( a_fwd_dwt_colgrp_cmd_data[i] );
			a_enc_cb[i].addr_quant_cmd_data = ( unsigned int )&( a_quant_cmd_data[i] );
			a_enc_cb[i].addr_quant_ret_data = ( unsigned int )&( a_quant_ret_data[i] );
			a_enc_cb[i].addr_t1_enc_cmd_data = ( unsigned int )&( a_t1_enc_cmd_data[i] );
			a_enc_cb[i].addr_t1_enc_ret_data = ( unsigned int )&( a_t1_enc_ret_data[i] );
			a_enc_cb[i].addr_t1_enc_pass_data = ( unsigned int )jas_malloc_align( JPC_MAX_PASSES * sizeof( jpc_enc_pass_t ), CACHE_LINE_OFFSET_BITS );
			a_enc_cb[i].addr_rd_cmpt_cmd_data = ( unsigned int )&( a_rd_cmpt_cmd_data[i] );
			a_p_aux_buf[i] = jas_malloc_align( DMA_AUX_BUF_SIZE, CACHE_LINE_OFFSET_BITS );
			if( a_p_aux_buf[i] == NULL ) {
				jas_eprintf( "[jpc_spu_ctrl.c:jpc_create_spu_threads()] jas_malloc_align failure\n" );
				return -1;
			}
		}
		else {
			a_dec_cb[i].spu_id = i;
			a_dec_cb[i].addr_prc_status = ( unsigned int )&( a_status[i] );
			a_dec_cb[i].mutex_ea = mutex_ea;

			a_dec_cb[i].addr_t1_dec_cmd_data = ( unsigned int )&( a_t1_dec_cmd_data[i] );
			a_dec_cb[i].addr_inv_dwt_row_cmd_data = ( unsigned int )&( a_inv_dwt_row_cmd_data[i] );
			a_dec_cb[i].addr_inv_dwt_colgrp_cmd_data = ( unsigned int )&( a_inv_dwt_colgrp_cmd_data[i] );
			a_dec_cb[i].addr_dequant_cmd_data = ( unsigned int )&( a_dequant_cmd_data[i] );
			a_dec_cb[i].addr_dec_post_proc_with_imct_cmd_data = ( unsigned int )&( a_dec_post_proc_with_imct_cmd_data[i] );
			a_dec_cb[i].addr_dec_post_proc_no_imct_cmd_data = ( unsigned int )&( a_dec_post_proc_no_imct_cmd_data[i] );
			a_enc_cb[i].addr_t1_enc_pass_data = 0;/* prevent freeing unallocated memory */
			a_dec_cb[i].addr_wr_cmpt_cmd_data = ( unsigned int )&( a_wr_cmpt_cmd_data[i] );
			a_p_aux_buf[i] = NULL;/* prevent freeing unallocated memory */
		}

		/* init other variables */

		if( enc == 1 ) {
			a_t1_enc_cmd_data[i].p_cblk = NULL;
		}
		else {
			a_t1_dec_cmd_data[i].p_cblk = NULL;
		}
		a_status[i].status = PRC_IDLE;

		/* create spu threads */
		a_pthread_data[i].spuid = spe_context_create( 0, NULL );
		if( a_pthread_data[i].spuid == NULL ) {
			jas_eprintf( "[jpc_spu_ctrl.c:jpc_create_spu_threads()] spe_context_create failure\n" );
			return -1;
		}

		if( enc ==  1 ) {
			ret = spe_program_load( a_pthread_data[i].spuid, &jpc_enc_spu );
		}
		else {
			ret = spe_program_load( a_pthread_data[i].spuid, &jpc_dec_spu );
		}
		if( ret != 0 ) {
			jas_eprintf( "[jpc_spu_ctrl.c:jpc_create_spu_threads()] spe_program_load failure\n" );
			return -1;
		}

		if( enc == 1 ) {
			a_pthread_data[i].argp = &( a_enc_cb[i] );
		}
		else {
			a_pthread_data[i].argp = &( a_dec_cb[i] );
		}

		if( pthread_create( &( a_pthread_data[i].pthread ), NULL, &ppu_pthread_function, &( a_pthread_data[i] ) ) != 0 ) {
			jas_eprintf( "[jpc_spu_ctrl.c:jpc_create_spu_threads()] pthread_create failure\n" );
			return -1;
		}
	}
	for( i = g_num_spus ; i < g_num_spus + g_num_ppus ; i++ ) {
		/* init other variables */

		if( enc == 1 ) {
			a_t1_enc_cmd_data[i].p_cblk = NULL;
		}
		else {
			a_t1_dec_cmd_data[i].p_cblk = NULL;
		}
		a_status[i].status = PRC_IDLE;
	}

	return 0;
}

int jpc_destroy_spu_threads() {
	int i;

	for( i = 0 ; i < g_num_spus ; i++ ) {
		jpc_send_cmd2spu( i, CMD_SPU_TERM );

		if( pthread_join( a_pthread_data[i].pthread, NULL ) != 0 ) {
			jas_eprintf( "[jpc_spu_ctrl.c:jpc_destroy_spu_threads()] pthread_join failure\n" );
			return -1;
		}

		if( spe_context_destroy( a_pthread_data[i].spuid ) != 0 ) {
			jas_eprintf( "[jpc_spu_ctrl.c:jpc_destroy_spu_threads()] spe_context_destroy failure\n" );
			return -1;
		}

		if( a_enc_cb[i].addr_t1_enc_pass_data != 0 ) {
			jas_free_align( ( void* )( a_enc_cb[i].addr_t1_enc_pass_data ) );
		}
		if( a_p_aux_buf[i] != NULL ) {
			jas_free_align( a_p_aux_buf[i] );
		}
	}

	return 0;
}

void jpc_send_cmd2spu( int spu_index, unsigned int cmd ) {
	unsigned int mbox_data;

	mbox_data = cmd;
	spe_in_mbox_write( a_pthread_data[spu_index].spuid, &mbox_data, 1, SPE_MBOX_ANY_NONBLOCKING );
}

int jpc_wait_for_spus() {
	int i;

	for( i = 0 ; i < g_num_spus ; i++ ) {
		while( a_status[i].status == PRC_RUNNING ) {
			sched_yield();
		}

		if( a_status[i].status == PRC_IDLE ) {
			/* do nothing */
		}
		else if( a_status[i].status == PRC_ERROR ) {
			return -1;
		}
		else {/* invalid state */
			jas_eprintf( "[jpc_cmd.c:jpc_wait_for_spus()] invalid state\n" );
			return -1;
		}
	}

	__asm__ __volatile__ ("sync" : : : "memory");

	return 0;
}

void set_num_spus() {
	if( SPU_THREADS != -1 ) {
		g_num_spus = SPU_THREADS;
	}
	else{
		g_num_spus = spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 );
	}
	jas_eprintf( "Number of SPU threads=%d\n", g_num_spus );

	return;
}

void set_num_ppus() {
	if( PPU_THREADS != -1 ) {
		g_num_ppus = PPU_THREADS;
	}
	else {
		if( spe_cpu_info_get( SPE_COUNT_PHYSICAL_SPES, -1 ) > MAX_SPU_THREADS_PER_CHIP ) {
			g_num_ppus = 2;
		}
		else {
			g_num_ppus = 1;
		}
	}
	jas_eprintf( "Number of PPU threads=%d\n", g_num_ppus );

	return;
}

/******************************************************************************\
* static functions
\******************************************************************************/

static void* ppu_pthread_function( void* p_arg ) {
        ppu_pthread_data_t* p_data = ( ppu_pthread_data_t* )p_arg;
        unsigned int entry;

        entry = SPE_DEFAULT_ENTRY;

        if( spe_context_run( p_data->spuid, &entry, 0, p_data->argp, NULL, NULL ) < 0 ) {
                jas_eprintf( "[jpc_dec.c:ppu_pthread()] spe_context_run failure\n" );
                exit( 1 );
        }

        pthread_exit( NULL );
}

/* s.kang end */

