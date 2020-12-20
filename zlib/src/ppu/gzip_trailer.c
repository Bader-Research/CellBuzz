/* gzip_trailer.c
 */

/* @(#) $Id: gzip_trailer.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

static int const gz_magic[2] = { 0x1f, 0x8b }; /* gzip magic header */

int write_trailer( unsigned char* p_data, unsigned int size, unsigned int crc, unsigned int in_size ) {
	if( size < 8 ) {
		return -1;
	}

	p_data[0] = crc & 0xff;
	p_data[1] = ( crc >> 8 ) & 0xff;
	p_data[2] = ( crc >> 16 ) & 0xff;
	p_data[3] = ( crc >> 24 ) & 0xff;

	p_data[4] = in_size & 0xff;
	p_data[5] = ( in_size >> 8 ) & 0xff;
	p_data[6] = ( in_size >> 16 ) & 0xff;
	p_data[7] = ( in_size >> 24 ) & 0xff;

	return 0;
}

int check_trailer( unsigned char* p_data, unsigned int size, unsigned int crc, unsigned int out_size ) {
	unsigned int _crc;
	unsigned int _out_size;

	if( size < 8 ) {
		return -1;
	}

	_crc = p_data[0];
	_crc += ( p_data[1] << 8 );
	_crc += ( p_data[2] << 16 );
	_crc += ( p_data[3] << 24 );
	if( crc != _crc ) {
		return -1;
	}

	_out_size = p_data[4];
	_out_size += ( p_data[5] << 8 );
	_out_size += ( p_data[6] << 16 );
	_out_size += ( p_data[7] << 24 );
	if( out_size != _out_size ) {
		return -1;
	}

	return 0;
}

