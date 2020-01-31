/****************************************************************************
 * libs/libc/libc.h
 *
 *   Copyright (C) 2007-2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __LIBC_LIBC_H
#define __LIBC_LIBC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>
#include <semaphore.h>

#include <nuttx/streams.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This configuration directory is used in environment variable processing
 * when we need to reference the user's home directory.  There are no user
 * directories in NuttX so, by default, this always refers to the root
 * directory.
 */

#ifndef CONFIG_LIB_HOMEDIR
# define CONFIG_LIB_HOMEDIR "/"
#endif

/* If C std I/O buffering is not supported, then we don't need its semaphore
 * protection.
 */

#ifdef CONFIG_STDIO_DISABLE_BUFFERING
#  define lib_sem_initialize(s)
#  define lib_take_semaphore(s)
#  define lib_give_semaphore(s)
#endif

/* The NuttX C library an be build in two modes: (1) as a standard, C-library
 * that can be used by normal, user-space applications, or (2) as a special,
 * kernel-mode C-library only used within the OS.  If NuttX is not being
 * built as separated kernel- and user-space modules, then only the first
 * mode is supported.
 */

#if (defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)) || \
     defined(CONFIG_BUILD_KERNEL)
#  include <nuttx/kmalloc.h>

   /* Domain-specific allocations */

#  define lib_malloc(s)     kmm_malloc(s)
#  define lib_zalloc(s)     kmm_zalloc(s)
#  define lib_realloc(p,s)  kmm_realloc(p,s)
#  define lib_memalign(p,s) kmm_memalign(p,s)
#  define lib_free(p)       kmm_free(p)

   /* User-accessible allocations */

#  define lib_umalloc(s)    kumm_malloc(s)
#  define lib_uzalloc(s)    kumm_zalloc(s)
#  define lib_urealloc(p,s) kumm_realloc(p,s)
#  define lib_ufree(p)      kumm_free(p)

#else
#  include <stdlib.h>

   /* Domain-specific allocations */

#  define lib_malloc(s)     malloc(s)
#  define lib_zalloc(s)     zalloc(s)
#  define lib_realloc(p,s)  realloc(p,s)
#  define lib_free(p)       free(p)

   /* User-accessible allocations */

#  define lib_umalloc(s)    malloc(s)
#  define lib_uzalloc(s)    zalloc(s)
#  define lib_urealloc(p,s) realloc(p,s)
#  define lib_ufree(p)      free(p)

#endif

#define LIB_BUFLEN_UNKNOWN INT_MAX

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

/* Defined in lib_streamsem.c */

#if CONFIG_NFILE_STREAMS > 0
void  stream_semtake(FAR struct streamlist *list);
void  stream_semgive(FAR struct streamlist *list);
#endif

/* Defined in lib_dtoa.c */

#ifdef CONFIG_LIBC_FLOATINGPOINT
char *__dtoa(double d, int mode, int ndigits, int *decpt, int *sign,
             char **rve);
#endif

/* Defined in lib_fopen.c */

int lib_mode2oflags(FAR const char *mode);

/* Defined in lib_libfwrite.c */

ssize_t lib_fwrite(FAR const void *ptr, size_t count, FAR FILE *stream);

/* Defined in lib_libfread.c */

ssize_t lib_fread(FAR void *ptr, size_t count, FAR FILE *stream);

/* Defined in lib_libfgets.c */

FAR char *lib_fgets(FAR char *buf, size_t buflen, FILE *stream,
                    bool keepnl, bool consume);

/* Defined in lib_libfflush.c */

ssize_t lib_fflush(FAR FILE *stream, bool bforce);

/* Defined in lib_rdflush.c */

int lib_rdflush(FAR FILE *stream);

/* Defined in lib_wrflush.c */

int lib_wrflush(FAR FILE *stream);

/* Defined in lib_sem.c */

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
void lib_sem_initialize(FAR struct file_struct *stream);
void lib_take_semaphore(FAR struct file_struct *stream);
void lib_give_semaphore(FAR struct file_struct *stream);
#endif

/* Defined in lib_libgetbase.c */

int lib_getbase(const char *nptr, const char **endptr);

/* Defined in lib_skipspace.c */

void lib_skipspace(const char **pptr);

/* Defined in lib_isbasedigit.c */

bool lib_isbasedigit(int ch, int base, int *value);

/* Defined in lib_checkbase.c */

int lib_checkbase(int base, const char **pptr);

/* Defined in lib_expi.c */

#ifdef CONFIG_LIBM
float  lib_expif(size_t n);
double lib_expi(size_t n);
#endif

/* Defined in lib_libsqrtapprox.c */

#ifdef CONFIG_LIBM
float lib_sqrtapprox(float x);
#endif

/* Defined in lib_parsehostfile.c */

#ifdef CONFIG_NETDB_HOSTFILE
struct hostent;
ssize_t lib_parse_hostfile(FAR FILE *stream, FAR struct hostent *host,
                           FAR char *buf, size_t buflen);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBC_LIBC_H */
