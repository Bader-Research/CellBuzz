/* minigzip.c -- simulate gzip using the zlib compression library
 * Copyright (C) 1995-2005 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/*
 * minigzip is a minimal implementation of the gzip utility. This is
 * only an example of using zlib and isn't meant to replace the
 * full-featured gzip. No attempt is made to deal with file systems
 * limiting names to 14 or 8+3 characters, etc... Error checking is
 * very limited. So use minigzip only for testing; use gzip for the
 * real thing. On MSDOS, use only on file names without extension
 * or in pipe mode.
 */

/* @(#) $Id: minigzip_spu_compress.c,v 1.2 2007/07/05 14:44:36 lrlemini Exp $ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef STDC
#  include <string.h>
#  include <stdlib.h>
#endif

#include <spu_mfcio.h>
#include <spu_internals.h>

#include "zlib.h"

#include "../minigzip.h"

#define BUFLEN      16384/* must be multiple of 16 */

static control_block_t cb __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
static gz_ret_info_t ret_info __attribute__ ( ( aligned( CACHE_LINE_SIZE ) ) );
static char a_buf[BUFLEN];
static char *prog;

static size_t read_data_size;
static unsigned int read_offset;
static size_t write_buf_size;
static unsigned int write_offset;

static void error            OF((const char *msg));
static void gz_compress      OF((void* p_in, void* p_out));
static void file_compress();

/* ===========================================================================
 * Display error message and exit
 */
static void error(msg)
    const char *msg;
{
	unsigned int spe_status __attribute__ ( ( aligned( 16 ) ) );

	fprintf(stderr, "%s: %s\n", prog, msg);
	ret_info.block_no = cb.block_no;
	ret_info.success = 0;
	mfc_put( &ret_info, cb.addr_ret_info, sizeof( gz_ret_info_t ), 20, 0, 0 );

	spe_status = SPE_WAITING;
	mfc_put( &spe_status, cb.addr_spe_status, sizeof( unsigned int ), 20, 0, 0 );

	mfc_write_tag_mask( 1 << 20 );
	mfc_read_tag_status_all(); 

	exit(1);
}

/* ===========================================================================
 * 
 */
static int read_data( void* p_buf, size_t size, size_t* p_offset, void* p_handle __attribute__ ( ( __unused__ ) ) ) {
        unsigned int addr;
        size_t data_len;
        size_t dma_len;

        if( ( size & 0xf ) != 0 ) {
                return -1;
        }

        if( read_data_size == 0 ) {
                return 0;
        }

        addr = cb.addr_read_data + read_offset;
        *p_offset = addr & 0xf;
        addr &= 0xfffffff0;

        if( size > *p_offset + read_data_size ) {
                dma_len = ( *p_offset + read_data_size + 15 ) & 0xfffffff0;
                data_len = read_data_size;
        }
        else {
                dma_len = size;
                data_len = size - *p_offset;
        }

        mfc_get( p_buf, addr, dma_len, 31, 0, 0 );
        mfc_write_tag_mask( 1 << 31 );
        mfc_read_tag_status_all();

        read_data_size -= data_len;
        read_offset += data_len;

        return ( int )data_len;
}

/* ===========================================================================
 * 
 */
static int write_data_dma( void* p_buf, size_t size, void* p_handle __attribute__ ( ( __unused__ ) ) ) {
        unsigned int addr;
        size_t data_len;
        size_t dma_len;

	if( size > write_buf_size ) {
		return -1;
	}

        addr = cb.addr_write_buf + write_offset;
	if( ( addr & 0xf ) != 0 ) {/* address misalignment */
		return -1;
	}

	dma_len = ( size + 15 ) & 0xfffffff0;
	data_len = size;

        mfc_put( p_buf, addr, dma_len, 20, 0, 0 );
        mfc_write_tag_mask( 1 << 20 );
        mfc_read_tag_status_all();

        write_buf_size -= data_len;
        write_offset += data_len;

        return ( int )data_len;
}

/* ===========================================================================
 * 
 */
static int write_data_file( void* p_buf, size_t size, void* p_handle ) {
	FILE* p_file;
	size_t len;

	p_file = ( FILE* )p_handle;

	len = fwrite( p_buf, 1, size, p_file );
	if( len != size ) {
		return -1;
	}

	return ( int )len;
}

/* ===========================================================================
 * Compress input to output then close both files.
 */
static void gz_compress( void* p_in __attribute__ ( ( __unused__ ) ), void* p_out )
{
	gzFile out;
	int len;
	int err;
	size_t offset;

	out = ( gzFile )p_out;

	while( 1 ) {
		len = read_data( a_buf, BUFLEN, &offset, NULL );
		if( len == 0 ) {
			break;
		}
		else if( len < 0 ) {
			error( "read_data error" );
		}

		if( gzwrite( out, &( a_buf[offset] ), ( unsigned )len ) != len ) {
			error( gzerror( out, &err ) );
		}
	}

	return;
}

/* ===========================================================================
 * Compress the given file: create a corresponding .gz file and remove the
 * original.
 */
static void file_compress()
{
	char a_outfile[MAX_FILE_NAME_LEN];
	FILE* p_out;
	gzFile out;

	if( strlen( cb.a_file_name ) + strlen( GZ_SUFFIX ) + 1 + CELL_GZIP_MAX_BLOCK_NO_DIGITS + 1 > MAX_FILE_NAME_LEN ) {
		error( "file name too long" );
	}

	sprintf( a_outfile, "%s%s", cb.a_file_name, GZ_SUFFIX );

	if( cb.cell_gzip == 0 ) {
		p_out = fopen( a_outfile, "ab" );
		if( p_out == NULL ) {
			error( "fopen" );
		}
	}
	else {
		p_out = NULL;
	}

	if( cb.cell_gzip == 1 ) {
		out = gzopen_w( a_outfile, cb.a_outmode, p_out, write_data_dma );
	}
	else {
		out = gzopen_w( a_outfile, cb.a_outmode, p_out, write_data_file );
	}
	if( out == NULL ) {
		error( "gzopen failure" );
	}

	read_data_size = cb.read_data_size;
	read_offset = 0;
	write_buf_size = cb.write_buf_size;
	write_offset = 0;
	gz_compress( NULL, ( void* )out );

	if( gzclose_w( out, cb.last, &( ret_info.crc ), &( ret_info.in_size ), &( ret_info.out_size ) ) != Z_OK ) {
		error( "failed gzclose" );
	}

	if( cb.cell_gzip == 0 ) {
		fclose( p_out );
	}

	return;
}

/* ===========================================================================
 * Usage:  minigzip [-d] [-f] [-h] [-r] [-1 to -9] [files...]
 *   -d : decompress
 *   -f : compress with Z_FILTERED
 *   -h : compress with Z_HUFFMAN_ONLY
 *   -r : compress with Z_RLE
 *   -1 to -9 : compression level
 */
int main( unsigned long long speid __attribute__ ( ( __unused__ ) ),
	unsigned long long argp,
	unsigned long long envp __attribute__ ( ( __unused__ ) ) )
{
	unsigned int spe_status __attribute__ ( ( aligned( 16 ) ) );

	prog = "minigzip";

	while( 1 ) {
		mfc_get( &cb, argp, sizeof( control_block_t ), 31, 0, 0 );
		mfc_write_tag_mask( 1 << 31 );
		mfc_read_tag_status_all(); 

		file_compress();

		ret_info.block_no = cb.block_no;
		ret_info.success = 1;
		mfc_put( &ret_info, cb.addr_ret_info, sizeof( gz_ret_info_t ), 20, 0, 0 );
		mfc_write_tag_mask( 1 << 20 );
		mfc_read_tag_status_all(); 

		spe_status = SPE_WAITING;
		mfc_put( &spe_status, cb.addr_spe_status, sizeof( unsigned int ), 20, 0, 0 );
		mfc_write_tag_mask( 1 << 20 );
		mfc_read_tag_status_all(); 

		spe_status = spu_read_in_mbox();
		if( spe_status == SPE_RUNNABLE ) {
			continue;
		}
		else if( spe_status == SPE_DONE ) {
			break;
		}
		else {
			error( "invalid SPE status\n" );
		}
	}

	return 0;
}

