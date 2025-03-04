/****************************************************************************
 * libs/libc/machine/arch_libc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "libc.h"

#include <string.h>

/****************************************************************************
 * Public Functions prototypes
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_MEMCHR
FAR void *ARCH_LIBCFUN(memchr)(FAR const void *s, int c, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCPY
FAR void *ARCH_LIBCFUN(memcpy)(FAR void *dest,
                               FAR const void *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCMP
int ARCH_LIBCFUN(memcmp)(FAR const void *s1, FAR const void *s2, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMMOVE
FAR void *ARCH_LIBCFUN(memmove)(FAR void *dest,
                                FAR const void *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMSET
FAR void *ARCH_LIBCFUN(memset)(FAR void *s, int c, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCMP
int ARCH_LIBCFUN(strcmp)(FAR const char *s1, FAR const char *s2);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCPY
FAR char *ARCH_LIBCFUN(strcpy)(FAR char *dest, FAR const char *src);
#endif

#ifdef CONFIG_LIBC_ARCH_STRLEN
size_t ARCH_LIBCFUN(strlen)(FAR const char *s);
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCPY
FAR char *ARCH_LIBCFUN(strncpy)(FAR char *dest,
                                FAR const char *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHR
FAR char *ARCH_LIBCFUN(strchr)(FAR const char *s, int c);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHNUL
FAR char *ARCH_LIBCFUN(strchrnul)(FAR const char *s, int c);
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCMP
int ARCH_LIBCFUN(strncmp)(FAR const char *s1, FAR const char *s2, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRNLEN
size_t ARCH_LIBCFUN(strnlen)(FAR const char *s, size_t maxlen);
#endif

#ifdef CONFIG_LIBC_ARCH_STRRCHR
FAR char *ARCH_LIBCFUN(strrchr)(FAR const char *s, int c);
#endif

#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
extern void __asan_loadN(FAR void *addr, size_t size);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
extern void __asan_storeN(FAR void *addr, size_t size);
#    endif
#  endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_MEMCHR

FAR void *memchr(FAR const void *s, int c, size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, n);
#    endif
#  endif

  return ARCH_LIBCFUN(memchr)(s, c, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCPY
FAR void *memcpy(FAR void *dest, FAR const void *src, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#  endif
  return ARCH_LIBCFUN(memcpy)(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCMP
int memcmp(FAR const void *s1, FAR const void *s2, size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
    __asan_loadN((FAR void *)s1, n);
    __asan_loadN((FAR void *)s2, n);
#   endif
#  endif
  return ARCH_LIBCFUN(memcmp)(s1, s2, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMMOVE
FAR void *memmove(FAR void *dest, FAR const void *src, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#  endif
  return ARCH_LIBCFUN(memmove)(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMSET
FAR void *memset(FAR void *s, int c, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(s, n);
#    endif
#  endif
  return ARCH_LIBCFUN(memset)(s, c, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCMP
int strcmp(FAR const char *s1, FAR const char *s2)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s1, ARCH_LIBCFUN(strlen)(s1) + 1);
  __asan_loadN((FAR void *)s2, ARCH_LIBCFUN(strlen)(s2) + 1);
#    endif
#  endif
  return ARCH_LIBCFUN(strcmp)(s1, s2);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCPY
FAR char *strcpy(FAR char *dest, FAR const char *src)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, ARCH_LIBCFUN(strlen)(src) + 1);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, ARCH_LIBCFUN(strlen)(src) + 1);
#    endif
#  endif
  return ARCH_LIBCFUN(strcpy)(dest, src);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRLEN
size_t strlen(FAR const char *s)
{
  size_t ret = ARCH_LIBCFUN(strlen)(s);
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, ret + 1);
  #  endif
#  endif
  return ret;
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCPY
FAR char *strncpy(FAR char *dest, FAR const char *src, size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#endif
  return ARCH_LIBCFUN(strncpy)(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHR
FAR char *strchr(FAR const char *s, int c)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, ARCH_LIBCFUN(strlen)(s) + 1);
#    endif
#  endif

  return ARCH_LIBCFUN(strchr)(s, c);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHNUL
FAR char *strchrnul(FAR const char *s, int c);
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, ARCH_LIBCFUN(strlen)(s) + 1);
#    endif
#  endif
  return ARCH_LIBCFUN(strchrnul)(s, c);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCMP
int strncmp(FAR const char *s1, FAR const char *s2, size_t n)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  size_t size_s1 = ARCH_LIBCFUN(strnlen)(s1, n);
  size_t size_s2 = ARCH_LIBCFUN(strnlen)(s2, n);
  size_t size;

  size = size_s1 < size_s2 ? size_s1 : size_s2;
  __asan_loadN((FAR void *)s1, size);
  __asan_loadN((FAR void *)s2, size);
#    endif
#  endif
  return ARCH_LIBCFUN(strncmp)(s1, s2, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRNLEN
size_t strnlen(FAR const char *s, size_t maxlen)
{
  size_t ret = ARCH_LIBCFUN(strnlen)(s, maxlen);
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, ret);
#    endif
#  endif

  return ret;
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRRCHR
FAR char *strrchr(FAR const char *s, int c)
{
#  ifdef CONFIG_MM_KASAN_INSTRUMENT
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, ARCH_LIBCFUN(strlen)(s) + 1);
#    endif
#  endif
  return ARCH_LIBCFUN(strrchr)(s, c);
}
#endif
