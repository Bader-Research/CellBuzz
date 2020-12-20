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
 * $Id: jpc_seg.c,v 1.3 2007/09/12 23:58:57 lrlemini Exp $
 */

/* s.kang start */
/* copied from jpc_dec.c, in SPU side, only the portion of jpc_dec.c is required */
/******************************************************************************\
* Includes.
\******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "jasper/jas_malloc.h"

#include "jpc_dec.h"
#include "jpc_t1cod.h"

/******************************************************************************\
*
\******************************************************************************/

void jpc_seglist_insert(jpc_dec_seglist_t *list, jpc_dec_seg_t *ins, jpc_dec_seg_t *node)
{
        jpc_dec_seg_t *prev;
        jpc_dec_seg_t *next;

        prev = ins;
        node->prev = prev;
        next = prev ? (prev->next) : 0;
        node->prev = prev;
        node->next = next;
        if (prev) {
                prev->next = node;
        } else {
                list->head = node;
        }
        if (next) {
                next->prev = node;
        } else {
                list->tail = node;
        }
}

void jpc_seglist_remove(jpc_dec_seglist_t *list, jpc_dec_seg_t *seg)
{
        jpc_dec_seg_t *prev;
        jpc_dec_seg_t *next;

        prev = seg->prev;
        next = seg->next;
        if (prev) {
                prev->next = next;
        } else {
                list->head = next;
        }
        if (next) {
                next->prev = prev;
        } else {
                list->tail = prev;
        }
        seg->prev = 0;
        seg->next = 0;
}

jpc_dec_seg_t *jpc_seg_alloc()
{
        jpc_dec_seg_t *seg;

/* s.kang start */
/* align for DMA transfer */
#if 1
        if (!(seg = jas_malloc_align(sizeof(jpc_dec_seg_t), CACHE_LINE_OFFSET_BITS))) {
                return 0;
        }
#else
        if (!(seg = jas_malloc(sizeof(jpc_dec_seg_t)))) {
                return 0;
        }
#endif
/* s.kang end */
        seg->prev = 0;
        seg->next = 0;
        seg->passno = -1;
        seg->numpasses = 0;
        seg->maxpasses = 0;
        seg->type = JPC_SEG_INVALID;
        seg->stream = 0;
        seg->cnt = 0;
        seg->complete = 0;
        seg->lyrno = -1;
        return seg;
}

void jpc_seg_destroy(jpc_dec_seg_t *seg)
{
        if (seg->stream) {
                jas_stream_close(seg->stream);
        }
/* s.kang start */
/* align for DMA transfer */
#if 1
        jas_free_align(seg);
#else
        jas_free(seg);
#endif
/* s.kang end */
}

/* s.kang end */

