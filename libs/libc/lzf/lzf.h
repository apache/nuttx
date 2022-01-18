/****************************************************************************
 * libs/libc/lzf/lzf.h
 *
 * Copyright (c) 2000-2007
 * Author: Marc Alexander Lehmann <schmorp@schmorp.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
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

#ifndef __LIBS_LIBC_LZF_LZF_H
#define __LIBS_LIBC_LZF_LZF_H

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

/* You may choose to pre-set the hash table (might be faster on some modern
 * CPUs and large (>>64k) blocks, and also makes compression deterministic/
 * repeatable when the configuration otherwise is the same).
 */

#ifndef INIT_HTAB
#  define INIT_HTAB 0
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

/* #define LZF_USER_OFFSETS autodetect */

#ifndef LZF_USE_OFFSETS
#  define LZF_USE_OFFSETS (UINTPTR_MAX > 0xffffffffU)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#endif /* __LIBS_LIBC_LZF_LZF_H */
