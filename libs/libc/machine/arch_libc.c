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

#include <string.h>

/****************************************************************************
 * Public Functions prototypes
 ****************************************************************************/

#ifdef CONFIG_LIBC_ARCH_MEMCHR
FAR void *arch_memchr(FAR const void *s, int c, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCPY
FAR void *arch_memcpy(FAR void *dest, FAR const void *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCMP
int arch_memcmp(FAR const void *s1, FAR const void *s2, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMMOVE
FAR void *arch_memmove(FAR void *dest, FAR const void *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_MEMSET
FAR void *arch_memset(FAR void *s, int c, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCMP
int arch_strcmp(FAR const char *s1, FAR const char *s2);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCPY
FAR char *arch_strcpy(FAR char *dest, FAR const char *src);
#endif

#ifdef CONFIG_LIBC_ARCH_STRLEN
size_t arch_strlen(FAR const char *s);
#else
#define arch_strlen(s) strlen(s)
#endif

#ifdef CONFIG_LIBC_ARCHSTRNCPY
FAR char *arch_strncpy(FAR char *dest, FAR const char *src, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHR
FAR char *arch_strchr(FAR const char *s, int c);
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHNUL
FAR char *arch_strchrnul(FAR const char *s, int c);
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCMP
int arch_strncmp(FAR const char *s1, FAR const char *s2, size_t n);
#endif

#ifdef CONFIG_LIBC_ARCH_STRNLEN
size_t arch_strnlen(FAR const char *s, size_t maxlen);
#endif

#ifdef CONFIG_LIBC_ARCH_STRRCHR
FAR char *arch_strrchr(FAR const char *s, int c);
#endif

#  ifdef CONFIG_MM_KASAN
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
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, n);
#    endif
#  endif

  return arch_memchr(s, c, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCPY
FAR void *memcpy(FAR void *dest, FAR const void *src, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#  endif
  return arch_memcpy(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMCMP
int memcmp(FAR const void *s1, FAR const void *s2, size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
    __asan_loadN((FAR void *)s1, n);
    __asan_loadN((FAR void *)s2, n);
#   endif
#  endif
  return arch_memcmp(s1, s2, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMMOVE
FAR void *memmove(FAR void *dest, FAR const void *src, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#  endif
  return arch_memmove(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_MEMSET
FAR void *memset(FAR void *s, int c, FAR size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(s, n);
#    endif
#  endif
  return arch_memset(s, c, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCMP
int strcmp(FAR const char *s1, FAR const char *s2)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s1, arch_strlen(s1) + 1);
  __asan_loadN((FAR void *)s2, arch_strlen(s2) + 1);
#    endif
#  endif
  return arch_strcmp(s1, s2);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCPY
FAR char *strcpy(FAR char *dest, FAR const char *src)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, arch_strlen(src) + 1);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, arch_strlen(src) + 1);
#    endif
#endif
  return arch_strcpy(dest, src);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRLEN
size_t strlen(FAR const char *s)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  size_t ret = arch_strlen(s);

  __asan_loadN((FAR void *)s, ret + 1);
  return ret;
  #  endif
#  else
  return arch_strlen(s);
#  endif
}
#endif

#ifdef CONFIG_LIBC_ARCHSTRNCPY
FAR char *strncpy(FAR char *dest, FAR const char *src, size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_WRITES_CHECK
  __asan_storeN(dest, n);
#    endif
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)src, n);
#    endif
#endif
  return arch_strncpy(dest, src, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHR
FAR char *strchr(FAR const char *s, int c)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, arch_strlen(s) + 1);
#    endif
#  endif

  return arch_strchr(s, c);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRCHNUL
FAR char *strchrnul(FAR const char *s, int c);
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, arch_strlen(s) + 1);
#    endif
#  endif
  return arch_strchrnul(s, c);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRNCMP
int strncmp(FAR const char *s1, FAR const char *s2, size_t n)
{
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s1, n);
  __asan_loadN((FAR void *)s2, n);
#    endif
#  endif
  return arch_strncmp(s1, s2, n);
}
#endif

#ifdef CONFIG_LIBC_ARCH_STRNLEN
size_t strnlen(FAR const char *s, size_t maxlen)
{
  size_t ret = arch_strnlen(s, maxlen);
#  ifdef CONFIG_MM_KASAN
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
#  ifdef CONFIG_MM_KASAN
#    ifndef CONFIG_MM_KASAN_DISABLE_READS_CHECK
  __asan_loadN((FAR void *)s, arch_strlen(s) + 1);
#    endif
#  endif
  return arch_strrchr(s, c);
}
#endif
