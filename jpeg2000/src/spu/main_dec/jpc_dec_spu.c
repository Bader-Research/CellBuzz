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

#include "jpc_dec.h"
#include "jpc_t1cod.h"
#include "jpc_t1dec.h"
#include "jpc_quant.h"
#include "jpc_mct.h"
#include "jpc_image.h"
#include "jpc_post_proc.h"

#include "jasper-cell.h"

static dec_control_block_t cb __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );

int main( unsigned long long speid __attribute__ ( ( __unused__ ) ),
	unsigned long long argp,
	unsigned long long envp __attribute__ ( ( __unused__ ) ) )
{
	prc_status_t prc_status __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	t1_dec_cmd_data_t t1_dec_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	dequant_cmd_data_t dequant_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	inv_dwt_row_cmd_data_t inv_dwt_row_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	inv_dwt_colgrp_cmd_data_t inv_dwt_colgrp_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	dec_post_proc_with_imct_cmd_data_t dec_post_proc_with_imct_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	dec_post_proc_no_imct_cmd_data_t dec_post_proc_no_imct_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	wr_cmpt_cmd_data_t wr_cmpt_cmd_data __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
	unsigned int cmd;

	if( ( ( sizeof( dec_control_block_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( prc_status_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( t1_dec_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( jpc_dec_seg_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( dequant_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( inv_dwt_row_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( inv_dwt_colgrp_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( dec_post_proc_with_imct_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( dec_post_proc_no_imct_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) || ( ( sizeof( wr_cmpt_cmd_data_t ) % CACHE_LINE_SIZE ) != 0 ) ) {
		jas_eprintf( "[jpc_dec_spu.c:main] data type size error\n" );
		exit( 1 );
	}

	mfc_get( &cb, argp, sizeof( dec_control_block_t ), 31, 0, 0 );
	mfc_write_tag_mask( 1 << 31 );
	mfc_read_tag_status_all();

	jas_mutex_init( cb.mutex_ea );

	jpc_initluts();

	while( 1 ) {
		cmd = spu_read_in_mbox();
		if( cmd == CMD_T1_DECODE ) {
			mfc_get( &t1_dec_cmd_data, cb.addr_t1_dec_cmd_data, sizeof( t1_dec_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( jpc_dec_decodecblk( &t1_dec_cmd_data ) != 0 ) {
				jas_eprintf( "[jpc_dec_spu.c:main()] jpc_dec_decodecblk failure\n" );
				prc_status.status = PRC_ERROR;
				break;
			}
			else {
				prc_status.status = PRC_IDLE;
			}
		}
		else if( cmd == CMD_DEQUANTIZATION ) {
			mfc_get( &dequant_cmd_data, cb.addr_dequant_cmd_data, sizeof( dequant_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();
	
			jpc_dequantize( ( void* )( dequant_cmd_data.addr_data ), dequant_cmd_data.stride, dequant_cmd_data.numrows, dequant_cmd_data.numcols, dequant_cmd_data.roishift, dequant_cmd_data.bgshift, dequant_cmd_data.numbps, dequant_cmd_data.realmode, dequant_cmd_data.absstepsize );

			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_INV_DWT_ROW ) {
			mfc_get( &inv_dwt_row_cmd_data, cb.addr_inv_dwt_row_cmd_data, sizeof( inv_dwt_row_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( inv_dwt_row_cmd_data.lossless ) {
				if( jpc_ft_row_synthesize( inv_dwt_row_cmd_data.startaddr, inv_dwt_row_cmd_data.numrows, inv_dwt_row_cmd_data.numcols, inv_dwt_row_cmd_data.stride, inv_dwt_row_cmd_data.colparity ) != 0 ) {
					jas_eprintf( "[jpc_dec_spu.c:main()] jpc_ft_row_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
			else {
				if( jpc_ns_row_synthesize( inv_dwt_row_cmd_data.startaddr, inv_dwt_row_cmd_data.numrows, inv_dwt_row_cmd_data.numcols, inv_dwt_row_cmd_data.stride, inv_dwt_row_cmd_data.colparity ) != 0 ) {
					jas_eprintf( "[jpc_dec_spu.c:main()] jpc_ns_row_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
		}
		else if( cmd == CMD_INV_DWT_COLGRP ) {
			mfc_get( &inv_dwt_colgrp_cmd_data, cb.addr_inv_dwt_colgrp_cmd_data, sizeof( inv_dwt_colgrp_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			if( inv_dwt_colgrp_cmd_data.lossless ) {
				if( jpc_ft_colgrp_synthesize( inv_dwt_colgrp_cmd_data.startaddr, inv_dwt_colgrp_cmd_data.joinbufaddr, inv_dwt_colgrp_cmd_data.numrows, inv_dwt_colgrp_cmd_data.numcolgrps, inv_dwt_colgrp_cmd_data.stride, inv_dwt_colgrp_cmd_data.rowparity ) != 0 ) {
					jas_eprintf( "[jpc_dec_spu.c:main()] jpc_ft_colgrp_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
			else {
				if( jpc_ns_colgrp_synthesize( inv_dwt_colgrp_cmd_data.startaddr, inv_dwt_colgrp_cmd_data.joinbufaddr, inv_dwt_colgrp_cmd_data.numrows, inv_dwt_colgrp_cmd_data.numcolgrps, inv_dwt_colgrp_cmd_data.stride, inv_dwt_colgrp_cmd_data.rowparity ) != 0 ) {
					jas_eprintf( "[jpc_dec_spu.c:main()] jpc_ns_colgrp_synthesize failure\n" );
					prc_status.status = PRC_ERROR;
					break;
				}
				else {
					prc_status.status = PRC_IDLE;
				}
			}
		}
		else if( cmd == CMD_DEC_POST_PROC_WITH_IMCT ) {
			mfc_get( &dec_post_proc_with_imct_cmd_data, cb.addr_dec_post_proc_with_imct_cmd_data, sizeof( dec_post_proc_with_imct_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_dec_post_proc_with_imct( ( void* )( dec_post_proc_with_imct_cmd_data.a_addr_comp_data[0] ), ( void* )( dec_post_proc_with_imct_cmd_data.a_addr_comp_data[1] ), ( void* )( dec_post_proc_with_imct_cmd_data.a_addr_comp_data[2] ), dec_post_proc_with_imct_cmd_data.a_comp_stride[0], dec_post_proc_with_imct_cmd_data.a_comp_stride[1], dec_post_proc_with_imct_cmd_data.a_comp_stride[2], dec_post_proc_with_imct_cmd_data.numrows, dec_post_proc_with_imct_cmd_data.numcols, dec_post_proc_with_imct_cmd_data.mctid, dec_post_proc_with_imct_cmd_data.realmode, dec_post_proc_with_imct_cmd_data.a_adjust[0], dec_post_proc_with_imct_cmd_data.a_adjust[1], dec_post_proc_with_imct_cmd_data.a_adjust[2], dec_post_proc_with_imct_cmd_data.a_mn[0], dec_post_proc_with_imct_cmd_data.a_mx[0], dec_post_proc_with_imct_cmd_data.a_mn[1], dec_post_proc_with_imct_cmd_data.a_mx[1], dec_post_proc_with_imct_cmd_data.a_mn[2], dec_post_proc_with_imct_cmd_data.a_mx[2] );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_DEC_POST_PROC_NO_IMCT ) {
			mfc_get( &dec_post_proc_no_imct_cmd_data, cb.addr_dec_post_proc_no_imct_cmd_data, sizeof( dec_post_proc_no_imct_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_dec_post_proc_no_imct( ( void* )( dec_post_proc_no_imct_cmd_data.addr_data ), dec_post_proc_no_imct_cmd_data.stride, dec_post_proc_no_imct_cmd_data.numrows, dec_post_proc_no_imct_cmd_data.numcols, dec_post_proc_no_imct_cmd_data.realmode, dec_post_proc_no_imct_cmd_data.adjust, dec_post_proc_no_imct_cmd_data.mn, dec_post_proc_no_imct_cmd_data.mx  );
			prc_status.status = PRC_IDLE;
		}
		else if ( cmd == CMD_WR_CMPT ) {
			mfc_get( &wr_cmpt_cmd_data, cb.addr_wr_cmpt_cmd_data, sizeof( wr_cmpt_cmd_data_t ), 31, 0, 0 );
			mfc_write_tag_mask( 1 << 31 );
			mfc_read_tag_status_all();

			jpc_writecmpt( ( void* )( wr_cmpt_cmd_data.addr_matrix_data ), ( void* )( wr_cmpt_cmd_data.addr_cmpt_data ), wr_cmpt_cmd_data.matrix_stride, wr_cmpt_cmd_data.cmpt_data_stride, wr_cmpt_cmd_data.numrows, wr_cmpt_cmd_data.numcols, wr_cmpt_cmd_data.cps, wr_cmpt_cmd_data.prec, wr_cmpt_cmd_data.sgnd );
			prc_status.status = PRC_IDLE;
		}
		else if( cmd == CMD_SPU_TERM ) {
			prc_status.status = PRC_DONE;
			break;
		}
		else {
			jas_eprintf( "[jpc_dec_spu.c:main()] invalid command\n" );
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

