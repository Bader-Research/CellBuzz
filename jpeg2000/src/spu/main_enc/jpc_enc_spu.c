/* s.kang start */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <stdlib.h>

#include <spu_mfcio.h>
#include <spu_internals.h>

#include "jasper/jas_debug.h"
#include "jasper/jas_mutex.h"

#include "jpc_enc.h"
#include "jpc_t1cod.h"
#include "jpc_t1enc.h"
#include "jpc_quant.h"
#include "jpc_mct.h"
#include "jpc_pre_proc.h"
#include "jpc_image.h"

#include "jasper-cell.h"

static enc_control_block_t cb __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

int main( unsigned long long speid __attribute__ ( ( __unused__ ) ),
	unsigned long long argp,
	unsigned long long envp __attribute__ ( ( __unused__ ) ) )
{
	prc_status_t prc_status __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	enc_pre_proc_with_mct_cmd_data_t enc_pre_proc_with_mct_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	enc_pre_proc_no_mct_cmd_data_t enc_pre_proc_no_mct_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	fwd_dwt_row_cmd_data_t fwd_dwt_row_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	fwd_dwt_colgrp_cmd_data_t fwd_dwt_colgrp_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	quant_cmd_data_t quant_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	quant_ret_data_t quant_ret_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	t1_enc_cmd_data_t t1_enc_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	rd_cmpt_cmd_data_t rd_cmpt_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

	t1_enc_ret_data_t t1_enc_ret_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

	unsigned int cmd;

	if( ( ( sizeof( enc_control_block_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( prc_status_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( enc_pre_proc_with_mct_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( enc_pre_proc_no_mct_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( fwd_dwt_row_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( fwd_dwt_colgrp_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( quant_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( quant_ret_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( t1_enc_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( t1_enc_ret_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( jpc_enc_pass_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( rd_cmpt_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) ) {
		jas_eprintf( "[jpc_enc_spu.c:main] data type size error\n" );
		exit( 1 );
	}

	mfc_get( &cb, argp, sizeof( enc_control_block_t ), 31, 0, 0 );
	mfc_write_tag_mask( 1 << 31 );
	mfc_read_tag_status_all();

	jas_mutex_init( cb.mutex_ea );

	jpc_initluts();

	while( 1 ) {
		cmd = spu_read_in_mbox();
		if( cmd == CMD_RD_CMPT ) {
			mfc_get( &rd_cmpt_cmd_data, cb.addr_rd_cmpt_cmd_data, sizeof( rd_cmpt_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_readcmpt( ( void* )( rd_cmpt_cmd_data.addr_cmpt_data ), ( void* )( rd_cmpt_cmd_data.addr_matrix_data ), rd_cmpt_cmd_data.cmpt_data_stride, rd_cmpt_cmd_data.matrix_stride, rd_cmpt_cmd_data.numrows, rd_cmpt_cmd_data.numcols, rd_cmpt_cmd_data.cps, rd_cmpt_cmd_data.prec, rd_cmpt_cmd_data.sgnd );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_ENC_PRE_PROC_WITH_MCT ) {
			mfc_get( &enc_pre_proc_with_mct_cmd_data, cb.addr_enc_pre_proc_with_mct_cmd_data, sizeof( enc_pre_proc_with_mct_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_enc_pre_proc_with_mct( ( void* )( enc_pre_proc_with_mct_cmd_data.a_addr_comp_data[0] ), ( void* )( enc_pre_proc_with_mct_cmd_data.a_addr_comp_data[1] ), ( void* )( enc_pre_proc_with_mct_cmd_data.a_addr_comp_data[2] ), enc_pre_proc_with_mct_cmd_data.a_comp_stride[0], enc_pre_proc_with_mct_cmd_data.a_comp_stride[1], enc_pre_proc_with_mct_cmd_data.a_comp_stride[2], enc_pre_proc_with_mct_cmd_data.numrows, enc_pre_proc_with_mct_cmd_data.numcols, enc_pre_proc_with_mct_cmd_data.mctid, enc_pre_proc_with_mct_cmd_data.realmode, enc_pre_proc_with_mct_cmd_data.a_adjust[0], enc_pre_proc_with_mct_cmd_data.a_adjust[1], enc_pre_proc_with_mct_cmd_data.a_adjust[2] );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_ENC_PRE_PROC_NO_MCT ) {
			mfc_get( &enc_pre_proc_no_mct_cmd_data, cb.addr_enc_pre_proc_no_mct_cmd_data, sizeof( enc_pre_proc_no_mct_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_enc_pre_proc_no_mct( ( void* )( enc_pre_proc_no_mct_cmd_data.addr_data ), enc_pre_proc_no_mct_cmd_data.stride, enc_pre_proc_no_mct_cmd_data.numrows, enc_pre_proc_no_mct_cmd_data.numcols, enc_pre_proc_no_mct_cmd_data.realmode, enc_pre_proc_no_mct_cmd_data.adjust );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_FWD_DWT_ROW ) {
			mfc_get( &fwd_dwt_row_cmd_data, cb.addr_fwd_dwt_row_cmd_data, sizeof( fwd_dwt_row_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( fwd_dwt_row_cmd_data.lossless ) {
				if( jpc_ft_row_analyze( fwd_dwt_row_cmd_data.startaddr, fwd_dwt_row_cmd_data.numrows, fwd_dwt_row_cmd_data.numcols, fwd_dwt_row_cmd_data.stride, fwd_dwt_row_cmd_data.colparity ) != 0 ) {
					jas_eprintf( "[jpc_enc_spu.c:main] jpc_ft_row_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
			else {
				if( jpc_ns_row_analyze( fwd_dwt_row_cmd_data.startaddr, fwd_dwt_row_cmd_data.numrows, fwd_dwt_row_cmd_data.numcols, fwd_dwt_row_cmd_data.stride, fwd_dwt_row_cmd_data.colparity ) != 0 ) {
					jas_eprintf( "[jpc_enc_spu.c:main] jpc_ns_row_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
		}
		else if( cmd == CMD_FWD_DWT_COLGRP ) {
			mfc_get( &fwd_dwt_colgrp_cmd_data, cb.addr_fwd_dwt_colgrp_cmd_data, sizeof( fwd_dwt_colgrp_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( fwd_dwt_colgrp_cmd_data.lossless ) {
				if( jpc_ft_colgrp_analyze( fwd_dwt_colgrp_cmd_data.startaddr, fwd_dwt_colgrp_cmd_data.splitbufaddr, fwd_dwt_colgrp_cmd_data.numrows, fwd_dwt_colgrp_cmd_data.numcolgrps, fwd_dwt_colgrp_cmd_data.stride, fwd_dwt_colgrp_cmd_data.rowparity ) != 0 ) {
					jas_eprintf( "[jpc_enc_spu.c:main] jpc_ft_colgrp_analyze failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
			else {
				if( jpc_ns_colgrp_analyze( fwd_dwt_colgrp_cmd_data.startaddr, fwd_dwt_colgrp_cmd_data.splitbufaddr, fwd_dwt_colgrp_cmd_data.numrows, fwd_dwt_colgrp_cmd_data.numcolgrps, fwd_dwt_colgrp_cmd_data.stride, fwd_dwt_colgrp_cmd_data.rowparity ) != 0 ) {
					jas_eprintf( "[jpc_enc_spu.c:main] jpc_ns_colgrp_analyze failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
		}
		else if( cmd == CMD_QUANTIZATION0 ) {
			mfc_get( &quant_cmd_data, cb.addr_quant_cmd_data, sizeof( quant_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			prc_status.status = PRC_IDLE;
			quant_ret_data.mxmag = jpc_max_mag( ( void* )( quant_cmd_data.addr_data ), quant_cmd_data.stride, quant_cmd_data.numrows, quant_cmd_data.numcols, quant_cmd_data.realmode );

			mfc_put( &quant_ret_data, ( unsigned long long )( cb.addr_quant_ret_data ), sizeof( quant_ret_data_t ), 20, 0, 0 );
			mfc_write_tag_mask( 1 << 20 );
			mfc_read_tag_status_all();
		}
		else if( cmd == CMD_QUANTIZATION1 ) {
			mfc_get( &quant_cmd_data, cb.addr_quant_cmd_data, sizeof( quant_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();
		
			jpc_quantize( ( void* )( quant_cmd_data.addr_data ), quant_cmd_data.stride, quant_cmd_data.numrows, quant_cmd_data.numcols, quant_cmd_data.absstepsize );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_T1_ENCODE ) {
			mfc_get( &t1_enc_cmd_data, cb.addr_t1_enc_cmd_data, sizeof( t1_enc_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( jpc_enc_enccblk( &t1_enc_cmd_data, &t1_enc_ret_data, cb.addr_t1_enc_pass_data ) != 0 ) {
				jas_eprintf( "[jpc_enc_spu.c:main()] jpc_enc_enccblk failure\n" );
				prc_status.status = PRC_ERROR;
				break;
			}
			else {
				prc_status.status = PRC_IDLE;
			}

			mfc_put( &t1_enc_ret_data, ( unsigned long long )( cb.addr_t1_enc_ret_data ), sizeof( t1_enc_ret_data_t ), 20, 0, 0 );
			mfc_write_tag_mask( 1 << 20 );
			mfc_read_tag_status_all();
		}
		else if( cmd == CMD_SPU_TERM ) {
			prc_status.status = PRC_DONE;
			break;
		}
		else {
			jas_eprintf( "[jpc_enc_spu.c:main] invalid command\n" );
			prc_status.status = PRC_ERROR;
			break;
		}

		mfc_put( &prc_status, ( unsigned long long )( cb.addr_prc_status ), sizeof( prc_status_t ), 20, 0, 0 );
		mfc_write_tag_mask( 1 << 20 );
		mfc_read_tag_status_all();
	}

	mfc_put( &prc_status, ( unsigned long long )( cb.addr_prc_status ), sizeof( prc_status_t ), 20, 0, 0 );
	mfc_write_tag_mask( 1 << 20 );
	mfc_read_tag_status_all();

	return 0;
}

/* s.kang end */

