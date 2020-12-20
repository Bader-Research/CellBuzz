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
 * spu command data
 *
 * $Id: jpc_spu_ctrl.h,v 1.7 2008/06/23 16:01:33 lrlemini Exp $
 */

/* s.kang start */
#ifndef JPC_CMD_DATA_H
#define JPC_CMD_DATA_H

/******************************************************************************\
* Includes.
\******************************************************************************/

#include <libspe2.h>

#include "jasper-cell.h"

/******************************************************************************\
* Data TYpes.
\******************************************************************************/

/* s.kang start */
typedef struct _ppu_pthread_data_t {
        spe_context_ptr_t spuid;
        pthread_t pthread;
        void* argp;
} ppu_pthread_data_t;
/* s.kang end */

/******************************************************************************\
* Global Variables.
\******************************************************************************/

extern enc_control_block_t a_enc_cb[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
extern dec_control_block_t a_dec_cb[MAX_SPU_THREADS] __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
extern volatile prc_status_t a_status[];
extern ppu_pthread_data_t a_pthread_data[];
extern enc_pre_proc_with_mct_cmd_data_t a_enc_pre_proc_with_mct_cmd_data[];
extern dec_post_proc_with_imct_cmd_data_t a_dec_post_proc_with_imct_cmd_data[];
extern enc_pre_proc_no_mct_cmd_data_t a_enc_pre_proc_no_mct_cmd_data[];
extern dec_post_proc_no_imct_cmd_data_t a_dec_post_proc_no_imct_cmd_data[];
extern fwd_dwt_row_cmd_data_t a_fwd_dwt_row_cmd_data[];
extern fwd_dwt_colgrp_cmd_data_t a_fwd_dwt_colgrp_cmd_data[];
extern inv_dwt_row_cmd_data_t a_inv_dwt_row_cmd_data[];
extern inv_dwt_colgrp_cmd_data_t a_inv_dwt_colgrp_cmd_data[];
extern quant_cmd_data_t a_quant_cmd_data[];
extern dequant_cmd_data_t a_dequant_cmd_data[];
extern quant_ret_data_t a_quant_ret_data[];
extern t1_enc_cmd_data_t a_t1_enc_cmd_data[];
extern t1_dec_cmd_data_t a_t1_dec_cmd_data[];
extern t1_enc_ret_data_t a_t1_enc_ret_data[];
extern rd_cmpt_cmd_data_t a_rd_cmpt_cmd_data[];
extern wr_cmpt_cmd_data_t a_wr_cmpt_cmd_data[];

extern void* a_p_aux_buf[];

extern int g_num_spus;
extern int g_num_ppus;

/******************************************************************************\
* Functions
\******************************************************************************/

int jpc_create_spu_threads( int enc );

int jpc_destroy_spu_threads();

void jpc_send_cmd2spu( int spu_index, unsigned int cmd );

int jpc_wait_for_spus();

void set_num_spus();

void set_num_ppus();

#endif
/* s.kang end */

