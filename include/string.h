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

#include <stddef.h>

/* Non-standard support for cases where CHAR_BIT != 8 carried in strings.h
 * only for convenience.  See include/nuttx/b2c.h.
 */

#include <nuttx/b2c.h>

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

FAR char  *strdup(FAR const char *s);
FAR char  *strndup(FAR const char *s, size_t size);
FAR const char *strerror(int);
int        strerror_r(int, FAR char *, size_t);
size_t     strlen(FAR const char *);
size_t     strnlen(FAR const char *, size_t);
FAR char  *strcat(FAR char *, FAR const char *);
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

FAR void  *memchr(FAR const void *s, int c, size_t n);
FAR void  *memrchr(FAR const void *s, int c, size_t n);
FAR void  *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n);
int        memcmp(FAR const void *s1, FAR const void *s2, size_t n);
FAR void  *memcpy(FAR void *dest, FAR const void *src, size_t n);
FAR void  *memmove(FAR void *dest, FAR const void *src, size_t count);
FAR void  *memset(FAR void *s, int c, size_t n);

void explicit_bzero(FAR void *s, size_t n);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_STRING_H */
