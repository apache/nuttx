/****************************************************************************
 * libs/libc/libc.h
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

#ifndef __LIBC_LIBC_H
#define __LIBC_LIBC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <semaphore.h>

#include <nuttx/lib/lib.h>
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

#ifdef CONFIG_FILE_STREAM
void  stream_semtake(FAR struct streamlist *list);
void  stream_semgive(FAR struct streamlist *list);
#endif

/* Defined in lib_dtoa.c */

#ifdef CONFIG_LIBC_FLOATINGPOINT
FAR char *__dtoa(double d, int mode, int ndigits, FAR int *decpt,
                 FAR int *sign, FAR char **rve);
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

/* Defined in lib_libgetbase.c */

int lib_getbase(FAR const char *nptr, FAR const char **endptr);

/* Defined in lib_skipspace.c */

void lib_skipspace(FAR const char **pptr);

/* Defined in lib_isbasedigit.c */

bool lib_isbasedigit(int ch, int base, FAR int *value);

/* Defined in lib_checkbase.c */

int lib_checkbase(int base, FAR const char **pptr);

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

#ifndef CONFIG_DISABLE_ENVIRON
int lib_restoredir(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __LIBC_LIBC_H */
