/****************************************************************************
 * include/strings.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_STRINGS_H
#define __INCLUDE_STRINGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_HAVE_INLINE) && !defined(__cplusplus)
/* Compatibility definitions
 *
 * Marked LEGACY in Open Group Base Specifications Issue 6/IEEE Std 1003.1-2004
 * Removed from Open Group Base Specifications Issue 7/IEEE Std 1003.1-2008
 */

#  define bcmp(b1,b2,len)  memcmp(b1,b2,(size_t)len)
#  define bcopy(b1,b2,len) (void)memmove(b2,b1,len)
#  define bzero(s,n)       (void)memset(s,0,n)
#  define index(s,c)       strchr(s,c)
#  define rindex(s,c)      strrchr(s,c)

#endif /* !CONFIG_HAVE_INLINE && !__cplusplus */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_HAVE_INLINE) || defined(__cplusplus)
/* Compatibility inline functions.
 *
 * Marked LEGACY in Open Group Base Specifications Issue 6/IEEE Std 1003.1-2004
 * Removed from Open Group Base Specifications Issue 7/IEEE Std 1003.1-2008
 */

static inline int bcmp(FAR const void *b1, FAR const void *b2, size_t len)
{
  return memcmp(b1, b2, len);
}

static inline void bcopy(FAR const void *b1, FAR void *b2, size_t len)
{
  (void)memmove(b2, b1, len);
}

static inline void bzero(FAR void *s, size_t len)
{
  (void)memset(s, 0, len);
}

static inline FAR char *index(FAR const char *s, int c)
{
  return strchr(s, c);
}

static inline FAR char *rindex(FAR const char *s, int c)
{
  return strrchr(s, c);
}
#endif /* CONFIG_HAVE_INLINE || __cplusplus */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ffs(int j);
int ffsl(long j);
#ifdef CONFIG_HAVE_LONG_LONG
int ffsll(long long j);
#endif

int fls(int j);
int flsl(long j);
#ifdef CONFIG_HAVE_LONG_LONG
int flsll(long long j);
#endif

int strcasecmp(FAR const char *, FAR const char *);
int strncasecmp(FAR const char *, FAR const char *, size_t);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_STRINGS_H */
