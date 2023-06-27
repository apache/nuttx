/****************************************************************************
 * include/string.h
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

#ifndef __INCLUDE_STRING_H
#define __INCLUDE_STRING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stddef.h>
#include <alloca.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define strcoll_l(s1, s2, l)    strcoll(s1, s2)
#define strdupa(x)              strcpy(alloca(strlen(x) + 1), x)
#define strerror_l(e, l)        strerror(e)
#define strxfrm_l(s1, s2, n, l) strxfrm(s1, s2, n)

#define strndupa(x, l) ({ \
  FAR const char *__old = (x); \
  size_t __len = strnlen(__old, (l)); \
  FAR char *__new = alloca(__len + 1); \
  __new[__len] = '\0'; \
  (FAR char *)memcpy(__new, __old, __len); \
})

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR char  *strdup(FAR const char *s) malloc_like;
FAR char  *strndup(FAR const char *s, size_t size) malloc_like;
FAR char  *strerror(int);
int        strerror_r(int, FAR char *, size_t);
size_t     strlen(FAR const char *);
size_t     strnlen(FAR const char *, size_t);
FAR char  *strcat(FAR char *, FAR const char *);
size_t     strlcat(FAR char *, FAR const char *, size_t);
FAR char  *strncat(FAR char *, FAR const char *, size_t);
int        strcmp(FAR const char *, FAR const char *);
int        strncmp(FAR const char *, FAR const char *, size_t);
int        strcoll(FAR const char *, FAR const char *s2);
FAR char  *strcpy(FAR char *dest, FAR const char *src);
FAR char  *stpcpy(FAR char *dest, FAR const char *src);
size_t     strlcpy(FAR char *dst, FAR const char *src, size_t siz);
FAR char  *strncpy(FAR char *, FAR const char *, size_t);
FAR char  *stpncpy(FAR char *, FAR const char *, size_t);
FAR char  *strpbrk(FAR const char *, FAR const char *);
FAR char  *strchr(FAR const char *s, int c);
FAR char  *strchrnul(FAR const char *s, int c);
FAR char  *strrchr(FAR const char *s, int c);
size_t     strspn(FAR const char *, FAR const char *);
size_t     strcspn(FAR const char *, FAR const char *);
FAR char  *strstr(FAR const char *, FAR const char *);
FAR char  *strcasestr(FAR const char *, FAR const char *);
FAR char  *strsep(FAR char **, FAR const char *);
FAR char  *strsignal(int signum);
FAR char  *strtok(FAR char *, FAR const char *);
FAR char  *strtok_r(FAR char *, FAR const char *, FAR char **);
size_t     strxfrm(FAR char *, FAR const char *, size_t n);
int        strverscmp(FAR const char *s1, FAR const char *s2);

FAR void  *memchr(FAR const void *s, int c, size_t n);
FAR void  *memrchr(FAR const void *s, int c, size_t n);
FAR void  *rawmemchr(FAR const void *s, int c);
FAR void  *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n);
int        memcmp(FAR const void *s1, FAR const void *s2, size_t n);
FAR void  *memcpy(FAR void *dest, FAR const void *src, size_t n);
FAR void  *mempcpy(FAR void *dest, FAR const void *src, size_t n);
FAR void  *memmove(FAR void *dest, FAR const void *src, size_t count);
FAR void  *memset(FAR void *s, int c, size_t n);
FAR void  *memmem(FAR const void *haystack, size_t haystacklen,
                  FAR const void *needle, size_t needlelen);

void explicit_bzero(FAR void *s, size_t n);
int timingsafe_bcmp(FAR const void *b1, FAR const void *b2, size_t n);

#if CONFIG_FORTIFY_SOURCE > 0
fortify_function(strcat) FAR char *strcat(FAR char *dest,
                                          FAR const char *src)
{
  fortify_assert(strlen(dest) + strlen(src) + 1 <= fortify_size(dest, 0));
  return __real_strcat(dest, src);
}

fortify_function(strlcat) size_t strlcat(FAR char *dst,
                                         FAR const char *src,
                                         size_t dsize)
{
  fortify_assert(dsize <= fortify_size(dst, 0));
  return __real_strlcat(dst, src, dsize);
}

fortify_function(strncat) FAR char *strncat(FAR char *dest,
                                            FAR const char *src,
                                            size_t n)
{
  fortify_assert(n <= fortify_size(dest, 0));
  return __real_strncat(dest, src, n);
}

fortify_function(strcpy) FAR char *strcpy(FAR char *dest,
                                          FAR const char *src)
{
  fortify_assert(strlen(src) + 1 <= fortify_size(dest, 0));
  return __real_strcpy(dest, src);
}

fortify_function(stpcpy) FAR char *stpcpy(FAR char *dest,
                                          FAR const char *src)
{
  fortify_assert(strlen(src) + 1 <= fortify_size(dest, 0));
  return __real_stpcpy(dest, src);
}

fortify_function(stpncpy) FAR char *stpncpy(FAR char *dest,
                                            FAR const char *src,
                                            size_t n)
{
  fortify_assert(n <= fortify_size(dest, 0));
  return __real_stpncpy(dest, src, n);
}

fortify_function(strlcpy) size_t strlcpy(FAR char *dst,
                                         FAR const char *src,
                                         size_t siz)
{
  fortify_assert(siz <= fortify_size(dst, 0));
  return __real_strlcpy(dst, src, siz);
}

fortify_function(strncpy) FAR char *strncpy(FAR char *dest,
                                            FAR const char *src,
                                            size_t n)
{
  fortify_assert(n <= fortify_size(dest, 0));
  return __real_strncpy(dest, src, n);
}

fortify_function(memmove) FAR void *memmove(FAR void *dest,
                                            FAR const void *src,
                                            size_t count)
{
  fortify_assert(count <= fortify_size(dest, 0) &&
                 count <= fortify_size(src, 0));
  return __real_memmove(dest, src, count);
}

fortify_function(memcpy) FAR void *memcpy(FAR void *dest,
                                          FAR const void *src,
                                          size_t n)
{
  fortify_assert(n <= fortify_size(dest, 0) && n <= fortify_size(src, 0));
  return __real_memcpy(dest, src, n);
}

fortify_function(memset) FAR void *memset(FAR void *s, int c, size_t n)
{
  fortify_assert(n <= fortify_size(s, 0));
  return __real_memset(s, c, n);
}

fortify_function(mempcpy) FAR void *mempcpy(FAR void *dest,
                                            FAR const void *src,
                                            size_t n)
{
  fortify_assert(n <= fortify_size(dest, 0) && n <= fortify_size(src, 0));
  return __real_mempcpy(dest, src, n);
}

fortify_function(explicit_bzero) void explicit_bzero(FAR void *s,
                                                     size_t n)
{
  fortify_assert(n <= fortify_size(s, 0));
  __real_explicit_bzero(s, n);
}
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_STRING_H */
