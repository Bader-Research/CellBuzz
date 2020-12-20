/* gzio.c -- IO on .gz files
 * Copyright (C) 1995-2005 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * Compile this file with -DNO_GZCOMPRESS to avoid the compression code.
 */

/* @(#) $Id: gzio_r.c,v 1.1.1.2 2007/05/23 15:20:08 lrlemini Exp $ */

#include <stdio.h>

#include "zutil.h"

#include "gzio.h"

static int const gz_magic[2] = {0x1f, 0x8b}; /* gzip magic header */
/* s.kang start */
static int ibm_cell_gzip;
static gz_r_data_func r_func;
/* s.kang end */

local gzFile gz_open      OF((const char *path, const char *mode, int transparent, FILE* p_file));
local int    get_byte     OF((gz_stream *s));
local void   check_header OF((gz_stream *s));
local int    destroy      OF((gz_stream *s));
/* s.kang start */
#if 1
local uInt  getInt      OF((gz_stream *s, int* p_eof));
#else
local uLong  getLong      OF((gz_stream *s));
#endif
/* s.kang end */

/* ===========================================================================
     Opens a gzip (.gz) file for reading or writing. The mode parameter
   is as in fopen ("rb" or "wb"). The file is given either by file descriptor
   or path name (if fd == -1).
     gz_open returns NULL if the file could not be opened or if there was
   insufficient memory to allocate the (de)compression state; errno
   can be checked to distinguish the two cases (if errno is zero, the
   zlib error is Z_MEM_ERROR).
*/
local gzFile gz_open (path, mode, transparent, p_file)
    const char *path;
    const char *mode;
    int transparent;
    FILE* p_file;
{
    int err;
    int level = Z_DEFAULT_COMPRESSION; /* compression level */
    int strategy = Z_DEFAULT_STRATEGY; /* compression strategy */
    char *p = (char*)mode;
    gz_stream *s;
    char fmode[80]; /* copy of mode, without the compression level */
    char *m = fmode;

    if (!path || !mode) return Z_NULL;

    s = (gz_stream *)ALLOC(sizeof(gz_stream));
    if (!s) return Z_NULL;

    s->stream.zalloc = (alloc_func)0;
    s->stream.zfree = (free_func)0;
    s->stream.opaque = (voidpf)0;
    s->stream.next_in = s->inbuf = Z_NULL;
    s->stream.next_out = s->outbuf = Z_NULL;
    s->stream.avail_in = s->stream.avail_out = 0;
    s->file = NULL;
    s->z_err = Z_OK;
    s->z_eof = 0;
    s->in = 0;
    s->out = 0;
    s->back = EOF;
    s->crc = crc32(0L, Z_NULL, 0);
    s->msg = NULL;
    s->transparent = transparent;

    s->path = (char*)ALLOC(strlen(path)+1);
    if (s->path == NULL) {
        return destroy(s), (gzFile)Z_NULL;
    }
    strcpy(s->path, path); /* do this early for debugging */

    s->mode = '\0';
    do {
        if (*p == 'r') s->mode = 'r';
        if (*p == 'w' || *p == 'a') s->mode = 'w';
        if (*p >= '0' && *p <= '9') {
            level = *p - '0';
        } else if (*p == 'f') {
          strategy = Z_FILTERED;
        } else if (*p == 'h') {
          strategy = Z_HUFFMAN_ONLY;
        } else if (*p == 'R') {
          strategy = Z_RLE;
        } else {
            *m++ = *p; /* copy the mode */
        }
    } while (*p++ && m != fmode + sizeof(fmode));
    if (s->mode == '\0') return destroy(s), (gzFile)Z_NULL;

    if (s->mode == 'w') {
	err = Z_STREAM_ERROR;
        return destroy(s), (gzFile)Z_NULL;
    }

/* s.kang start */
#if 1
    s->stream.next_in  = s->inbuf = (Byte*)ALLOC(Z_BUFSIZE * 2);/* double buffering */
#else
    s->stream.next_in  = s->inbuf = (Byte*)ALLOC(Z_BUFSIZE);
#endif
/* s.kang end */

    err = inflateInit2(&(s->stream), -MAX_WBITS_DECOMPRESS);
    /* windowBits is passed < 0 to tell that there is no zlib header.
     * Note that in this case inflate *requires* an extra "dummy" byte
     * after the compressed stream in order to complete decompression and
     * return Z_STREAM_END. Here the gzip CRC32 ensures that 4 bytes are
     * present after the compressed stream.
     */
    if (err != Z_OK || s->inbuf == Z_NULL) {
        return destroy(s), (gzFile)Z_NULL;
    }
    
    s->stream.avail_out = Z_BUFSIZE;

    errno = 0;
    s->file = p_file;
    s->start = 0L;

    return (gzFile)s;
}

/* ===========================================================================
     Opens a gzip (.gz) file for reading or writing.
*/
gzFile ZEXPORT gzopen_r (path, mode, p_file, cell_gzip, transparent, func)
    const char *path;
    const char *mode;
    FILE* p_file;
    int cell_gzip;
    int transparent;
    gz_r_data_func func; 
{
    ibm_cell_gzip = cell_gzip;
    r_func = func;
    return gz_open (path, mode, transparent, p_file);
}

/* ===========================================================================
     Read a byte from a gz_stream; update next_in and avail_in. Return EOF
   for end of file.
   IN assertion: the stream s has been sucessfully opened for reading.
*/
local int get_byte(s)
    gz_stream *s;
{
/* s.kang start */
    size_t offset;
    int temp;
/* s.kang end */

/* s.kang start */
    if (s == NULL || s->mode != 'r') {
        s->z_err = Z_STREAM_ERROR;
        return EOF;
    }
/* s.kang end */

    if (s->z_eof) return EOF;
    if (s->stream.avail_in == 0) {
        errno = 0;
/* s.kang start */
#if 1
        temp = r_func(s->inbuf, Z_BUFSIZE, &offset, ( void* )s->file);
        if( temp == -1 ) {
            s->z_err = Z_ERRNO;
            s->z_eof = 1;
            return EOF;
        }

        s->stream.avail_in = temp;
        if (s->stream.avail_in == 0) {
            s->z_eof = 1;
            return EOF;
        }
        s->stream.next_in = s->inbuf + offset;
#else
        s->stream.avail_in = (uInt)fread(s->inbuf, 1, Z_BUFSIZE, s->file);
        if (s->stream.avail_in == 0) {
            s->z_eof = 1;
            if (ferror(s->file)) s->z_err = Z_ERRNO;
            return EOF;
        }
        s->stream.next_in = s->inbuf;
#endif
    }
    s->stream.avail_in--;
    return *(s->stream.next_in)++;
}

/* ===========================================================================
      Check the gzip header of a gz_stream opened for reading. Set the stream
    mode to transparent if the gzip magic header is not present; set s->err
    to Z_DATA_ERROR if the magic header is present but the rest of the header
    is incorrect.
    IN assertion: the stream s has already been created sucessfully;
       s->stream.avail_in is zero for the first time, but may be non-zero
       for concatenated .gz files.
*/
local void check_header(s)
    gz_stream *s;
{
    int method; /* method byte */
    int flags;  /* flags byte */
    uInt len;
    int c;
/* s.kang start */
    size_t offset;
    int temp;
/* s.kang end */

/* s.kang start */
    if (s == NULL || s->mode != 'r') {
        s->z_err = Z_STREAM_ERROR;
        return;
    }
/* s.kang end */

    /* Assure two bytes in the buffer so we can peek ahead -- handle case
       where first byte of header is at the end of the buffer after the last
       gzip segment */
    len = s->stream.avail_in;
    if (len < 2) {
        if (len) s->inbuf[0] = s->stream.next_in[0];
        errno = 0;
/* s.kang start */
#if 1
        temp = r_func(s->inbuf, Z_BUFSIZE, &offset, ( void* )s->file);
        if( temp == -1 ) {
            s->z_err = Z_ERRNO;
            temp = 0;
        }
        len = temp;
        s->stream.avail_in += len;
        s->stream.next_in = s->inbuf + offset;
#else
        len = (uInt)fread(s->inbuf + len, 1, Z_BUFSIZE >> len, s->file);
        if (len == 0 && ferror(s->file)) s->z_err = Z_ERRNO;
        s->stream.avail_in += len;
        s->stream.next_in = s->inbuf;
#endif
        if (s->stream.avail_in < 2) {
            s->transparent = s->stream.avail_in;
            return;
        }
    }

    /* Peek ahead to check the gzip magic header */
    if (s->stream.next_in[0] != gz_magic[0] ||
        s->stream.next_in[1] != gz_magic[1]) {
        s->transparent = 1;
        return;
    }
    s->stream.avail_in -= 2;
    s->stream.next_in += 2;

    /* Check the rest of the gzip header */
    method = get_byte(s);
    flags = get_byte(s);
    if (method != Z_DEFLATED || (flags & RESERVED) != 0) {
        s->z_err = Z_DATA_ERROR;
        return;
    }

    /* Discard time, xflags and OS code: */
    for (len = 0; len < 6; len++) (void)get_byte(s);

    if ((flags & EXTRA_FIELD) != 0) { /* skip the extra field */
        len  =  (uInt)get_byte(s);
        len += ((uInt)get_byte(s))<<8;
        /* len is garbage if EOF but the loop below will quit anyway */
        while (len-- != 0 && get_byte(s) != EOF) ;
    }
    if ((flags & ORIG_NAME) != 0) { /* skip the original file name */
        while ((c = get_byte(s)) != 0 && c != EOF) ;
    }
    if ((flags & COMMENT) != 0) {   /* skip the .gz file comment */
        while ((c = get_byte(s)) != 0 && c != EOF) ;
    }
    if ((flags & HEAD_CRC) != 0) {  /* skip the header crc */
        for (len = 0; len < 2; len++) (void)get_byte(s);
    }
    s->z_err = s->z_eof ? Z_DATA_ERROR : Z_OK;
}

 /* ===========================================================================
 * Cleanup then free the given gz_stream. Return a zlib error code.
   Try freeing in the reverse order of allocations.
 */
local int destroy (s)
    gz_stream *s;
{
    int err = Z_OK;

    if (!s) return Z_STREAM_ERROR;

    TRYFREE(s->msg);

    if (s->stream.state != NULL) {
        if (s->mode == 'r') {
            err = inflateEnd(&(s->stream));
        }
    }
    if (s->z_err < 0) err = s->z_err;

    TRYFREE(s->inbuf);
    TRYFREE(s->outbuf);
    TRYFREE(s->path);
    TRYFREE(s);
    return err;
}

/* s.kang start */
/* ===========================================================================
     Reads the given number of uncompressed bytes from the compressed file.
   gzread returns the number of bytes actually read (0 for end of file).
*/
int ZEXPORT gzread (file, buf, len)
    gzFile file;
    voidp buf;
    unsigned len;
{
    gz_stream *s = (gz_stream*)file;
    Bytef *start = (Bytef*)buf; /* starting point for crc computation */
    Byte  *next_out;/* == stream.next_out but not forced far (for MSDOS) */
    size_t offset;
    int eof;
    int temp;

    if (s == NULL || s->mode != 'r') return Z_STREAM_ERROR;

    if (s->z_err == Z_DATA_ERROR || s->z_err == Z_ERRNO) return -1;
    if (s->z_err == Z_STREAM_END) return 0;  /* EOF */

    next_out = (Byte*)buf;
    s->stream.next_out = (Bytef*)buf;
    s->stream.avail_out = len;

    if (s->stream.avail_out && s->back != EOF) {
        *next_out++ = s->back;
        s->stream.next_out++;
        s->stream.avail_out--;
        s->back = EOF;
        s->out++;
        start++;
        if (s->last) {
            s->z_err = Z_STREAM_END;
            return 1;
        }
    }

    while (s->stream.avail_out != 0) {
        if (s->transparent) {
            /* Copy first the lookahead bytes: */
            uInt n = s->stream.avail_in;
            if (n > s->stream.avail_out) n = s->stream.avail_out;
            if (n > 0) {
                zmemcpy(s->stream.next_out, s->stream.next_in, n);
                next_out += n;
                s->stream.next_out = next_out;
                s->stream.next_in += n;
                s->stream.avail_out -= n;
                s->stream.avail_in  -= n;
            }
            while (s->stream.avail_out > 0) {
		temp = r_func( s->inbuf, Z_BUFSIZE, &offset, ( void* )s->file );
                if( temp == -1 ) {
                    s->z_err = Z_ERRNO;
                    break;
                }
                else if (temp == 0) {
                    break;
                }
                s->stream.avail_in = temp;
                s->stream.next_in = s->inbuf + offset;

                if (temp > (int)(s->stream.avail_out)) temp = s->stream.avail_out;
                if (temp > 0) {
                    zmemcpy(s->stream.next_out, s->stream.next_in, temp);
                    next_out += temp;
                    s->stream.next_out = next_out;
                    s->stream.next_in += temp;
                    s->stream.avail_out -= temp;
                    s->stream.avail_in  -= temp;
                }
            }
            len -= s->stream.avail_out;
            s->in  += len;
            s->out += len;
            if (len == 0) s->z_eof = 1;
            return (int)len;
        }
        if (s->stream.avail_in == 0 && !s->z_eof) {
            errno = 0;
            temp = r_func(s->inbuf, Z_BUFSIZE, &offset, ( void* )s->file);
            if(temp == -1) {
                s->z_err = Z_ERRNO;
                break;
            }
            else if (temp == 0) {
                s->z_eof = 1;
            }
            s->stream.avail_in = temp;
            s->stream.next_in = s->inbuf + offset;
        }
	
        if( s->stream.avail_in != 0 ) {
            s->in += s->stream.avail_in;
       	    s->out += s->stream.avail_out;
            s->z_err = inflate(&(s->stream), Z_NO_FLUSH);
            s->in -= s->stream.avail_in;
            s->out -= s->stream.avail_out;
        }

        if (s->z_err == Z_STREAM_END) {
            /* update CRC */
            s->crc = crc32(s->crc, start, (uInt)(s->stream.next_out - start));
            start = s->stream.next_out;
            if( ibm_cell_gzip == 0 ) {/* for concatenated gzip */
                temp = getInt(s, &eof);
                if( eof == 0 ) {
                    if (temp != ( int )s->crc) {
                        s->z_err = Z_DATA_ERROR;
                    } else {
                        (void)getInt(s, &eof);
                        /* The uncompressed length returned by above getlong() may be
                         * different from s->out in case of concatenated .gz files.
                         * Check for such files:
                         */
                        check_header(s);
                        if (s->z_err == Z_OK) {
                            inflateReset(&(s->stream));
                            s->crc = crc32(0L, Z_NULL, 0);
                        }
                    }
                }
            }
        }
        if (s->z_err != Z_OK || s->z_eof) break;
    }
    s->crc = crc32(s->crc, start, (uInt)(s->stream.next_out - start));

    if (len == s->stream.avail_out &&
        (s->z_err == Z_DATA_ERROR || s->z_err == Z_ERRNO))
        return -1;
    return (int)(len - s->stream.avail_out);
}
/* s.kang end */

/* ===========================================================================
   Reads a long in LSB order from the given gz_stream. Sets z_err in case
   of error.
*/
/* s.kang start */
#if 1
local uInt getInt( s, p_eof )
	gz_stream* s;
	int* p_eof;
{
	uInt x;
	size_t offset;
	int temp;
	int i;

	if( s->z_eof ) {
		*p_eof = 1;
		return 0;
	}

	x = 0;
	for( i = 0 ; i < 4 ; i++ ) {
		if( s->stream.avail_in == 0 ) {
			temp = r_func(s->inbuf, Z_BUFSIZE, &offset, ( void* )s->file);
			if( temp == -1 ) {
				s->z_err = Z_ERRNO;
				s->z_eof = 1;
				*p_eof = 1;
				return 0;
			}
			else if( temp == 0 ) {
				s->z_eof = 1;
				*p_eof = 1;
				return 0;
			}
			else {
				s->stream.avail_in = temp;
				s->stream.next_in = s->inbuf + offset;
			}
		}
		x += *( s->stream.next_in ) << ( i * 8 ); 	
		s->stream.next_in++;
		s->stream.avail_in--;
	}

	*p_eof = 0;
	return x;
}
#else
local uLong getLong (s)
    gz_stream *s;
{
    uLong x = (uLong)get_byte(s);
    int c;

    x += ((uLong)get_byte(s))<<8;
    x += ((uLong)get_byte(s))<<16;
    c = get_byte(s);
    if (c == EOF) s->z_err = Z_DATA_ERROR;
    x += ((uLong)c)<<24;
    return x;
}
#endif
/* s.kang end */

/* ===========================================================================
     Flushes all pending output if necessary, closes the compressed file
   and deallocates all the (de)compression state.
*/
/* s.kang start */
#if 1
int ZEXPORT gzclose_r( gzFile file, unsigned int last, unsigned int* p_crc, unsigned int* p_in_size, unsigned int* p_out_size ) {
	gz_stream* s;
	int ret;

	s = ( gz_stream* )file;

	if( s == NULL ) {
		return Z_STREAM_ERROR;
	}

        if( s->mode != 'r' ) {
                return  Z_STREAM_ERROR;
        }

	*p_crc = s->crc;
	*p_in_size = ( unsigned int )( s->in & 0xffffffff );
	*p_out_size = ( unsigned int )( s->out & 0xffffffff );

	ret = destroy( ( gz_stream* )file );
	if( ( last != 1 ) && ( ret == Z_DATA_ERROR ) ) {
		return Z_OK;
	}
	else {
		return ret;
	}
}
#else
int ZEXPORT gzclose (file)
    gzFile file;
{
    gz_stream *s = (gz_stream*)file;

    if (s == NULL) return Z_STREAM_ERROR;

    if (s->mode == 'w') {
#ifdef NO_GZCOMPRESS
        return Z_STREAM_ERROR;
#else
        if (do_flush (file, Z_FINISH) != Z_OK)
            return destroy((gz_stream*)file);

        putLong (s->file, s->crc);
        putLong (s->file, (uLong)(s->in & 0xffffffff));
#endif
    }
    return destroy((gz_stream*)file);
}
#endif
/* s.kang end */

