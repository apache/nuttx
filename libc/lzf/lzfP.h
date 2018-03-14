/****************************************************************************
 * libc/lzf/lzf_c.c
 *
 * Copyright (c) 2000-2007 Marc Alexander Lehmann <schmorp@schmorp.de>
 * 
 * Redistribution and use in source and binary forms, with or without modifica-
 * tion, are permitted provided that the following conditions are met:
 * 
 *   1.  Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 * 
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MER-
 * CHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPE-
 * CIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTH-
 * ERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __LIBC_LZF_LZFP_H
#define __LIBC_LZF_LZFP_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __cplusplus
#  include <cstdint>
#  include <cstring>
#  include <climits>
#  include <cerrno>
using namespace std;
#else
#  include <stdint.h>
#  include <string.h>
#  include <limits.h>
#  include <errno.h>
#endif

#include <lzf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of hashtable is (1 << HLOG) * sizeof (char *).  Decompression is
 * independent of the hash table size the difference between 15 and 14 is
 * very small for small blocks (and 14 is usually a bit faster).  For a
 * low-memory/faster configuration, use HLOG == 13;  For best compression,
 * use 15 or 16 (or more, up to 22).
 */

#ifndef HLOG
#  define HLOG 13
#endif

/* You may choose to pre-set the hash table (might be faster on some modern
 * CPUs and large (>>64k) blocks, and also makes compression deterministic/
 * repeatable when the configuration otherwise is the same).
 */

#ifndef INIT_HTAB
#  define INIT_HTAB 0
#endif

/* Whether to pass the LZF_STATE variable as argument, or allocate it
 * on the stack. For small-stack environments, define this to 1.
 * NOTE: this breaks the prototype in lzf.h.
 */

#ifndef LZF_STATE_ARG
#  define LZF_STATE_ARG 0
#endif

/* Whether to add extra checks for input validity in lzf_decompress
 * and return EINVAL if the input stream has been corrupted. This
 * only shields against overflowing the input buffer and will not
 * detect most corrupted streams.
 * This check is not normally noticeable on modern hardware
 * (<1% slowdown), but might slow down older cpus considerably.
 */

#ifndef CHECK_INPUT
#  define CHECK_INPUT 1
#endif

/* Whether to store pointers or offsets inside the hash table. On
 * 64 bit architectures, pointers take up twice as much space,
 * and might also be slower. Default is to autodetect.
 */

/*#define LZF_USER_OFFSETS autodetect */

#ifndef LZF_USE_OFFSETS
#  define LZF_USE_OFFSETS (UINTPTR_MAX > 0xffffffffU)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if LZF_USE_OFFSETS
# define LZF_HSLOT_BIAS ((const uint8_t *)in_data)
  typedef unsigned int LZF_HSLOT;
#else
# define LZF_HSLOT_BIAS 0
  typedef const uint8_t *LZF_HSLOT;
#endif

typedef LZF_HSLOT LZF_STATE[1 << (HLOG)];

#endif /* __LIBC_LZF_LZFP_H */
