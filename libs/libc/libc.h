/****************************************************************************
 * libs/libc/libc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __LIBS_LIBC_LIBC_H
#define __LIBS_LIBC_LIBC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdbool.h>
#  include <stdio.h>
#  include <stdlib.h>
#  include <limits.h>
#  include <semaphore.h>

#  include <nuttx/lib/lib.h>
#  include <nuttx/streams.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This configuration directory is used in environment variable processing
 * when we need to reference the user's home directory.  There are no user
 * directories in NuttX so, by default, this always refers to the root
 * directory.
 */

#ifndef CONFIG_LIBC_HOMEDIR
#  define CONFIG_LIBC_HOMEDIR "/"
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_MEMCHR_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_MEMCHR_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_MEMCHR
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_MEMCMP_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_MEMCMP_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_MEMCMP
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_MEMCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_MEMCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_MEMCPY
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_MEMMOVE_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_MEMMOVE_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_MEMMOVE
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_MEMSET_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_MEMSET_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_MEMSET
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCAT_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCAT_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCAT
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCASECMP_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCASECMP_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCASECMP
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCHR_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCHR_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCHR
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCHRNUL_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCHRNUL_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCHRNUL
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCMP_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCMP_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCMP
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRCPY
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRLCAT_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRLCAT_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRLCAT
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRLEN_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRLEN_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRLEN
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRLCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRLCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRLCPY
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRNCASECMP_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRNCASECMP_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRNCASECMP
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRNCAT_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRNCAT_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRNCAT
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRNLEN_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRNLEN_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRNLEN
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRNCMP_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRNCMP_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRNCMP
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRNCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRNCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRNCPY
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STRRCHR_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STRRCHR_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STRRCHR
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STPCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STPCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STPCPY
#endif

#if ((!defined(CONFIG_LIBC_PREVENT_STPNCPY_USER) && !defined(__KERNEL__))  || \
     (!defined(CONFIG_LIBC_PREVENT_STPNCPY_KERNEL) && defined(__KERNEL__)))
#  define LIBC_BUILD_STPNCPY
#endif

#ifdef CONFIG_MM_KASAN
#  define ARCH_LIBCFUN(x)  arch_##x
#else
#  define ARCH_LIBCFUN(x)  x
#endif

/* Nonzero if either x or y is not aligned on a "libc_data_t" boundary. */

#define UNALIGNED_X(x) \
  (((libc_data_t)(uintptr_t)(x) & (sizeof(libc_data_t) - 1)) != 0)

#define UNALIGNED(x, y) ((UNALIGNED_X(x)) | (UNALIGNED_X(y)))

/* Nonzero if x and y disagree about where a "libc_data_t" boundary falls.
 * A pair that agrees can be walked up to the boundary a byte at a time and
 * handled a word at a time from there, since aligning one aligns the
 * other.  A pair that disagrees cannot, because no single boundary serves
 * both.
 */

#define MISALIGNED(x, y) \
  ((((uintptr_t)(x)) ^ ((uintptr_t)(y))) & (sizeof(libc_data_t) - 1))

#define ALIGNED(x) \
  (((libc_data_t)(uintptr_t)(x) & (sizeof(libc_data_t) - 1)) == 0)

/* How many bytes are copied each iteration of the word copy loop. */

#define LITTLEBLOCKSIZE (sizeof(libc_data_t))
#define BIGBLOCKSIZE    (sizeof(libc_data_t) << 2)

/* Threshold for punting to the byte copier. */

#define TOO_SMALL(len) ((len) < LITTLEBLOCKSIZE)

/* Macros for detecting endchar */

#define DETECTNULL(x) \
     (((x) - 0x0101010101010101LL) & ~(x) & 0x8080808080808080LL)

#define DETECTCHAR(x, mask) (DETECTNULL((x) ^ (mask)))

/* 32-bit helpers for the 4-byte middle path on 64-bit platforms.
 * When libc_data_t is 8 bytes, pointers that are 4-byte aligned
 * but not 8-byte aligned would otherwise fall back to byte-at-a-time.
 */

#define DETECTNULL32(x) \
     (((x) - (uint32_t)0x01010101) & ~(x) & (uint32_t)0x80808080)

#define DETECTCHAR32(x, mask) (DETECTNULL32((x) ^ (mask)))

#define UNALIGNED4(x, y) \
     ((((uintptr_t)(x)) | ((uintptr_t)(y))) & 3)

#define UNALIGNED4_X(x) (((uintptr_t)(x)) & 3)

#define LITTLEBLOCKSIZE4 (sizeof(uint32_t))
#define BIGBLOCKSIZE4    (sizeof(uint32_t) << 2)

#define TOO_SMALL4(len) ((len) < LITTLEBLOCKSIZE4)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef unsigned long long libc_data_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in lib_getfullpath.c */

int lib_getfullpath(int dirfd, FAR const char *path,
                    FAR char *fullpath, size_t fulllen);

/* Defined in lib_fopen.c */

int lib_mode2oflags(FAR const char *mode);

/* Defined in lib_libfwrite.c */

ssize_t lib_fwrite(FAR const void *ptr, size_t count, FAR FILE *stream);
ssize_t lib_fwrite_unlocked(FAR const void *ptr, size_t count,
                            FAR FILE *stream);

/* Defined in lib_libfread_unlocked.c */

ssize_t lib_fread_unlocked(FAR void *ptr, size_t count, FAR FILE *stream);

/* Defined in lib_libgets.c */

FAR char *lib_dgets(FAR char *buf, size_t buflen, int fd,
                    bool keepnl, bool consume);

/* Defined in lib_libfgets.c */

FAR char *lib_fgets(FAR char *buf, size_t buflen, FILE *stream,
                    bool keepnl, bool consume);
FAR char *lib_fgets_unlocked(FAR char *buf, size_t buflen, FILE *stream,
                             bool keepnl, bool consume);

/* Defined in lib_flushall.c */

#ifdef CONFIG_FILE_STREAM
int lib_flushall(FAR struct streamlist *list);
int lib_flushall_unlocked(FAR struct streamlist *list);
#endif

/* Defined in lib_libfflush.c */

ssize_t lib_fflush(FAR FILE *stream);
ssize_t lib_fflush_unlocked(FAR FILE *stream);

/* Defined in lib_rdflush_unlocked.c */

int lib_rdflush_unlocked(FAR FILE *stream);

/* Defined in lib_wrflush_unlocked.c */

int lib_wrflush_unlocked(FAR FILE *stream);

/* Defined in lib_libgetbase.c */

int lib_getbase(FAR const char *nptr, FAR const char **endptr);

/* Defined in lib_skipspace.c */

void lib_skipspace(FAR const char **pptr);

/* Defined in lib_isbasedigit.c */

bool lib_isbasedigit(int ch, int base, FAR int *value);

/* Defined in lib_checkbase.c */

int lib_checkbase(int base, FAR const char **pptr);

/* Defined in lib_parsehostfile.c */

#ifdef CONFIG_NETDB_HOSTFILE
struct hostent;
ssize_t lib_parse_hostfile(FAR FILE *stream, FAR struct hostent *host,
                           FAR char *buf, size_t buflen);
#endif

#ifndef CONFIG_DISABLE_ENVIRON
int lib_restoredir(void);
#endif

/* Defined in lib_cxx_initialize.c */

void lib_cxx_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __LIBS_LIBC_LIBC_H */
