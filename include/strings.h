/****************************************************************************
 * include/strings.h
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

#ifndef __INCLUDE_STRINGS_H
#define __INCLUDE_STRINGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define strcasecmp_l(s1, s2, l)     strcasecmp(s1, s2)
#define strncasecmp_l(s1, s2, n, l) strncasecmp(s1, s2, n)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Compatibility functions
 *
 * Marked LEGACY in Open Group Base Specifications Issue 6/
 * IEEE Std 1003.1-2004
 * Removed from Open Group Base Specifications Issue 7/
 * IEEE Std 1003.1-2008
 */

int bcmp(FAR const void *b1, FAR const void *b2, size_t len);
void bcopy(FAR const void *b1, FAR void *b2, size_t len);
void bzero(FAR void *s, size_t n);

#if CONFIG_FORTIFY_SOURCE > 0
fortify_function(bcopy) void bcopy(FAR const void *b1, FAR void *b2,
                                   size_t len)
{
  fortify_assert(len <= fortify_size(b1, 0) && len <= fortify_size(b2, 0));
  __real_bcopy(b1, b2, len);
}

fortify_function(bzero) void bzero(FAR void *s, size_t n)
{
  fortify_assert(n <= fortify_size(s, 0));
  __real_bzero(s, n);
}
#endif

int ffs(int j);

#ifdef CONFIG_HAVE_BUILTIN_FFS
#  define ffs(j)  __builtin_ffs(j)
#elif defined (CONFIG_HAVE_BUILTIN_CTZ)
#  define ffs(j)  (__builtin_ctz(j) + 1)
#endif

int ffsl(long j);

#ifdef CONFIG_HAVE_BUILTIN_FFSL
#  define ffsl(j) __builtin_ffsl(j)
#elif defined (CONFIG_HAVE_BUILTIN_CTZ)
#  define ffsl(j) (__builtin_ctzl(j) + 1)
#endif

int ffsll(long long j);

#ifdef CONFIG_HAVE_BUILTIN_FFSLL
#  define ffsll(j)  __builtin_ffsll(j)
#elif defined (CONFIG_HAVE_BUILTIN_CTZ)
#  define ffsll(j)  (__builtin_ctzll(j) + 1)
#endif

int fls(int j);

#ifdef CONFIG_HAVE_BUILTIN_CLZ
#  define fls(j)  ((8 * sizeof(int)) - __builtin_clz(j))
#endif

int flsl(long j);

#ifdef CONFIG_HAVE_BUILTIN_CLZ
#  define flsl(j)  ((8 * sizeof(long)) - __builtin_clzl(j))
#endif

int flsll(long long j);

#ifdef CONFIG_HAVE_BUILTIN_CLZ
#  define flsll(j)  ((8 * sizeof(long long)) - __builtin_clzll(j))
#endif

unsigned int popcount(unsigned int j);
unsigned int popcountl(unsigned long j);
unsigned int popcountll(unsigned long long j);

FAR char *index(FAR const char *s, int c);
FAR char *rindex(FAR const char *s, int c);

int strcasecmp(FAR const char *, FAR const char *);
int strncasecmp(FAR const char *, FAR const char *, size_t);

#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_STRINGS_H */
