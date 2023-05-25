/****************************************************************************
 * include/lzf.h
 * http://liblzf.plan9.de/
 *
 *   Copyright (c) 2000-2008 Marc Alexander Lehmann <schmorp@schmorp.de>
 *   This algorithm is believed to be patent-free.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_LZF_H
#define __INCLUDE_LZF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_LZF

#define LZF_VERSION 0x0105 /* 1.5, API version */
#define HLOG        CONFIG_LIBC_LZF_HLOG

#define LZF_TYPE0_HDR      0
#define LZF_TYPE1_HDR      1

#define LZF_TYPE0_HDR_SIZE 5
#define LZF_TYPE1_HDR_SIZE 7

#define LZF_MAX_HDR_SIZE   7
#define LZF_MIN_HDR_SIZE   5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* LZF headers */

struct lzf_header_s         /* Common data header */
{
  uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
  uint8_t lzf_type;         /* LZF_TYPE0_HDR or LZF_TYPE1_HDR */
};

struct lzf_type0_header_s   /* Uncompressed data header */
{
  uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
  uint8_t lzf_type;         /* LZF_TYPE0_HDR */
  uint8_t lzf_len[2];       /* Data length (big-endian) */
};

struct lzf_type1_header_s   /* Compressed data header */
{
  uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
  uint8_t lzf_type;         /* LZF_TYPE1_HDR */
  uint8_t lzf_clen[2];      /* Compressed data length (big-endian) */
  uint8_t lzf_ulen[2];      /* Uncompressed data length (big-endian) */
};

/* LZF hash table */

#if LZF_USE_OFFSETS
#  define LZF_HSLOT_BIAS ((const uint8_t *)in_data)
  typedef unsigned int lzf_hslot_t;
#else
#  define LZF_HSLOT_BIAS 0
  typedef const uint8_t *lzf_hslot_t;
#endif

typedef lzf_hslot_t lzf_state_t[1 << HLOG];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lzf_compress
 *
 * Description:
 * Compress in_len bytes stored at the memory block starting at
 *   in_data and write the result to out_data, up to a maximum length
 *   of out_len bytes.
 *
 *   If the output buffer is not large enough or any error occurs return 0,
 *   otherwise return the number of bytes used, which might be considerably
 *   more than in_len (but less than 104% of the original size), so it
 *   makes sense to always use out_len == in_len - 1), to ensure _some_
 *   compression, and store the data uncompressed otherwise (with a flag, of
 *   course.
 *
 *   lzf_compress might use different algorithms on different systems and
 *   even different runs, thus might result in different compressed strings
 *   depending on the phase of the moon or similar factors. However, all
 *   these strings are architecture-independent and will result in the
 *   original data when decompressed using lzf_decompress.
 *
 *   The buffers must not be overlapping.
 *
 *   Compressed format:
 *
 *     000LLLLL <L+1>    ; literal, L+1=1..33 octets
 *     LLLooooo oooooooo ; backref L+1=1..7 octets, o+1=1..4096 offset
 *     111ooooo LLLLLLLL oooooooo ; backref L+8 octets, o+1=1..4096 offset
 *
 ****************************************************************************/

size_t lzf_compress(FAR const void *const in_data,
                    unsigned int in_len, FAR void *out_data,
                    unsigned int out_len, lzf_state_t htab,
                    FAR struct lzf_header_s **reshdr);

/****************************************************************************
 * Name: lzf_decompress
 *
 * Description:
 *   Decompress data compressed with some version of the lzf_compress
 *   function and stored at location in_data and length in_len. The result
 *   will be stored at out_data up to a maximum of out_len characters.
 *
 *   If the output buffer is not large enough to hold the decompressed
 *   data, a 0 is returned and errno is set to E2BIG. Otherwise the number
 *   of decompressed bytes (i.e. the original length of the data) is
 *   returned.
 *
 *   If an error in the compressed data is detected, a zero is returned and
 *   errno is set to EINVAL.
 *
 *   This function is very fast, about as fast as a copying loop.
 *
 ****************************************************************************/

unsigned int lzf_decompress(FAR const void *const in_data,
                            unsigned int in_len, FAR void *out_data,
                            unsigned int out_len);

#endif /* CONFIG_LIBC_LZF */
#endif /* __INCLUDE_LZF_H */
