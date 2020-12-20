/* gzip_header.c
 */

/* @(#) $Id: gzip_header.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

#include "zlib.h"

/* gzip flag byte */

#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define HEAD_CRC     0x02 /* bit 1 set: header CRC present */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define RESERVED     0xE0 /* bits 5..7: reserved */

#define OS_CODE 0x03/* Unix */
#define Z_DEFLATED 8

static int const gz_magic[2] = { 0x1f, 0x8b }; /* gzip magic header */

int write_header( unsigned char* p_data, unsigned int size, gzip_header_extra_t* p_header_extra ) {
        int header_offset;
        int len;
	int extra_len;
	int sub_len;
	unsigned int i;

	len = ( int )size;
        if( len < 10 ) {
                return -1;
        }

	header_offset = 0;

	/* check transparent */

	if( p_header_extra->transparent == 1 ) {
		return header_offset;
	}

	/* write gzip magic_header */

	p_data[header_offset] = gz_magic[0];
	p_data[header_offset + 1] = gz_magic[1];
	header_offset += 2;
	len -= 2;

	/* write method and flags */

	p_data[header_offset] = Z_DEFLATED;
	if( p_header_extra->num_cell_gzip_block != 0 ) {
		p_data[header_offset + 1] = EXTRA_FIELD;
	}
	else {
		p_data[header_offset + 1] = 0;
	}
	header_offset += 2;
	len -= 2;

        /* write time, xflags, and OS code */

	p_data[header_offset] = 0;
	p_data[header_offset + 1] = 0;
	p_data[header_offset + 2] = 0;
	p_data[header_offset + 3] = 0;
	p_data[header_offset + 4] = 0;
	p_data[header_offset + 5] = OS_CODE;
	header_offset += 6;
	len -= 6;

	/* write extra field */

	if( p_header_extra->extra_block_info_present != 0 ) {
		extra_len = 2/* subfield id */ + 2/* subfield len */ + 2/* number of full flush blocks */ + ( 4 * p_header_extra->num_cell_gzip_block );
		sub_len = extra_len - 2/* subfield id */ -2/* subfield_len */;
		if( len < 2 + extra_len ) {
			return -1;
		}
		p_data[header_offset] = extra_len & 0xff;
		p_data[header_offset + 1] = ( extra_len >> 8 ) & 0xff;
		header_offset += 2;

		p_data[header_offset] = 'C';
		p_data[header_offset + 1] = 'E';
		header_offset += 2;

		p_data[header_offset] = sub_len & 0xff;
		p_data[header_offset + 1] = ( sub_len >> 8 ) & 0xff;
		header_offset += 2;

		p_data[header_offset] = ( p_header_extra->num_cell_gzip_block ) & 0xff;
		p_data[header_offset + 1] = ( ( p_header_extra->num_cell_gzip_block ) >> 8 ) & 0xff;
		header_offset += 2;

		for( i = 0 ; i < p_header_extra->num_cell_gzip_block ; i++ ) {
			p_data[header_offset] = ( p_header_extra->a_cell_gzip_block_size[i] ) & 0xff;
			p_data[header_offset + 1] = ( ( p_header_extra->a_cell_gzip_block_size[i] ) >> 8 ) & 0xff;
			p_data[header_offset + 2] = ( ( p_header_extra->a_cell_gzip_block_size[i] ) >> 16 ) & 0xff;
			p_data[header_offset + 3] = ( ( p_header_extra->a_cell_gzip_block_size[i] ) >> 24 ) & 0xff;
			header_offset += 4;
		}
	}

	return header_offset;
}

int check_header( unsigned char* p_data, unsigned int size, gzip_header_extra_t* p_header_extra ) {
        int header_offset;
        int method;
        int flags;
        int len;
	int extra_len;
	int sub_len;
	unsigned int i;

	p_header_extra->extra_block_info_present = 0;
	p_header_extra->transparent = 0;

	len = ( int )size;
	if( len < 2 ) {
		p_header_extra->transparent = len;
		return 0;
	}
        if( len < 10 ) {
                return -1;
        }

        header_offset = 0;

        /* check gzip magic header */

        if( p_data[header_offset] != gz_magic[0] || p_data[header_offset + 1] != gz_magic[1] ) {
		p_header_extra->transparent = 1;
		return 0;
        }
        header_offset = 2;
        len -= 2;

        /* check method and flags */

        method = ( int )p_data[header_offset];
        flags = ( int )p_data[header_offset + 1];
        if( method != Z_DEFLATED || ( flags & RESERVED ) != 0 ) {
                return -1;
        }
        header_offset = 4;
        len -= 2;

        /* discard time, xflags, and OS code */

        header_offset = 10;
        len -= 6;

        /* skip extra field */

        if( ( flags & EXTRA_FIELD ) != 0 ) {
		extra_len = p_data[header_offset] + ( p_data[header_offset + 1] << 8 );
		header_offset += 2;
		len -= 2;
		if( len < 0 ) {
			return -1;
		}

		while( extra_len > 0 ) {
			if( ( p_data[header_offset] == 'C' ) && ( p_data[header_offset + 1] == 'E' ) ) {
				p_header_extra->extra_block_info_present = 1;
				header_offset += 2;
				len -= 2;
				extra_len -= 2;

				sub_len = p_data[header_offset] + ( p_data[header_offset + 1] << 8 );
				header_offset += 2;
				len -= 2;
				extra_len -= 2;

				p_header_extra->num_cell_gzip_block = p_data[header_offset] + ( p_data[header_offset + 1 ] << 8 );
				if( sub_len != ( int )( 2 + p_header_extra->num_cell_gzip_block * 4 ) ) {
					return -1;
				}
				header_offset += 2;

				for( i = 0 ; i < p_header_extra->num_cell_gzip_block ; i++ ) {
					p_header_extra->a_cell_gzip_block_size[i] = p_data[header_offset] + ( p_data[header_offset + 1] << 8 ) + ( p_data[header_offset + 2] << 16 ) + ( p_data[header_offset + 3] << 24 );
					header_offset += 4;
				}

				len -= sub_len;
				extra_len -= sub_len;

				if( ( len < 0 ) || ( extra_len < 0 ) ) {
					return -1;
				}
			}
			else {
				header_offset += 2;
				len -= 2;
				extra_len -= 2;

				sub_len = p_data[header_offset] + ( p_data[header_offset + 1] << 8 );
				header_offset += 2;
				len -= 2;
				extra_len -= 2;

				header_offset += sub_len;
				len -= sub_len;
				extra_len -= sub_len;

				if( ( len < 0 ) || ( extra_len < 0 ) ) {
					return -1;
				}
			}
		}
        }

        /* skip original file name */

        if( ( ( flags & ORIG_NAME ) != 0 ) ) {
                while( ( p_data[header_offset++] != 0 ) && ( len != 0 ) ) {
                        len--;
                }
                if( len < 0 ) {
                        return -1;
                }
        }

        /* skip .gz file comment */

        if( ( flags & COMMENT ) != 0 ) {
                while( ( p_data[header_offset++] != 0 ) && ( len != 0 ) ) {
                        len--;
                }
                if( len < 0 ) {
                        return -1;
                }
        }

        /* skip header crc */

        if( ( flags & HEAD_CRC ) != 0 ) {
                header_offset += 2;
                len -= 2;
                if( len < 0 ) {
                        return -1;
                }
        }

	return header_offset;
}

