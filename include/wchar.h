/****************************************************************************
 * include/wchar.h
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

#ifndef __INCLUDE_WCHAR_H
#define __INCLUDE_WCHAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* "Inclusion of the <wchar.h> header may make visible all symbols from the
 *  headers <ctype.h>, <stdio.h>, <stdarg.h>, <stdlib.h>, <string.h>,
 *  <stddef.h> and <time.h>."
 *
 * Reference: Opengroup.org
 */

#include <stdio.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* <wchar.h> defines the following macro names:
 *
 * WCHAR_MAX
 *   The maximum value representable by an object of type wchar_t.
 *
 * WCHAR_MIN
 *   The minimum value representable by an object of type wchar_t.
 *
 * WEOF
 *   Constant expression of type wint_t that is returned by several WP
 *   functions to indicate end-of-file.
 *
 * NULL
 *   As described in <stddef.h>.
 *
 * Reference: Opengroup.org
 */

#define WEOF ((wint_t)-1)

#define wcsftime_l(s, m, f, t, l)   wcsftime(s, m, f, t)
#define wcscasecmp_l(s1, s2, l)     wcscasecmp(s1, s2)
#define wcsncasecmp_l(s1, s2, n, l) wcsncasecmp(s1, s2, n)
#define wcscoll_l(s1, s2, l)        wcscoll(s1, s2)
#define wcstold_l(s, e, l)          wcstold(s, e)
#define wcstoull_l(s, e, l)         wcstoull(s, e)
#define wcsxfrm_l(s1, s2, n, l)     wcsxfrm(s1, s2, n)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* "The <wchar.h> header defines the following data types:
 *
 * wchar_t
 *   Provided via <stddef.h>.
 *
 * wint_t
 *   An integral type capable of storing any valid value of wchar_t, or WEOF.
 *   Provided via <sys/type.h>
 *
 * wctype_t
 *   A scalar type of a data object that can hold values which represent
 *   locale-specific character classification.  Provided via <sys/type.h>
 *
 * mbstate_t
 *   An object type other than an array type that can hold the conversion
 *   state information necessary to convert between sequences of (possibly
 *   multibyte) characters and wide-characters. If a codeset is being used
 *   such that an mbstate_t needs to preserve more than 2 levels of reserved
 *   state, the results are unspecified.
 */

struct mbstate_s
{
  int __fill[6];
};

typedef struct mbstate_s mbstate_t;

/* FILE
 *   As described in <stdio.h>.
 *
 * size_t
 *   As described in <stddef.h>."
 *
 * Reference: Opengroup.org
 */

/* "The tag tm is declared as naming an incomplete structure type, the
 *  contents of which are described in the header <time.h>."
 *
 * Reference: Opengroup.org
 */

struct tm; /* Forward reference (see <time.h>) */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* "The <wchar.h> header declares the following as functions and may also
 *  define them as macros. Function prototypes must be provided for use with
 *  an ISO C compiler."
 *
 * Reference: Opengroup.org
 */

wint_t            btowc(int);
int               fwprintf(FILE *, FAR const wchar_t *, ...);
int               fwscanf(FILE *, FAR const wchar_t *, ...);
wint_t            fgetwc(FAR FILE *);
wint_t            fgetwc_unlocked(FAR FILE *f);
FAR wchar_t      *fgetws(wchar_t *, int, FILE *);
wint_t            fputwc(wchar_t, FILE *);
wint_t            fputwc_unlocked(wchar_t, FAR FILE *);
int               fputws(FAR const wchar_t *, FILE *);
int               fputws_unlocked(FAR const wchar_t *, FAR FILE *);
int               fwide(FILE *, int);
wint_t            getwc(FAR FILE *);
wint_t            getwchar(void);
int               mbsinit(FAR const mbstate_t *);
size_t            mbrlen(FAR const char *, size_t, FAR mbstate_t *);
size_t            mbrtowc(FAR wchar_t *, FAR const char *, size_t,
                      FAR mbstate_t *);
size_t            mbsnrtowcs(FAR wchar_t *, FAR const char **, size_t,
                      size_t, FAR mbstate_t *);
size_t            mbsrtowcs(FAR wchar_t *, FAR const char **, size_t,
                      FAR mbstate_t *);
wint_t            putwc(wchar_t, FILE *);
wint_t            putwc_unlocked(wchar_t, FAR FILE *);
wint_t            putwchar(wchar_t);
wint_t            putwchar_unlocked(wchar_t);
int               swprintf(FAR wchar_t *, size_t, FAR const wchar_t *, ...);
int               swscanf(FAR const wchar_t *, FAR const wchar_t *, ...);
wint_t            ungetwc(wint_t, FAR FILE *);
wint_t            ungetwc_unlocked(wint_t, FAR FILE *);
int               vfwprintf(FILE *, FAR const wchar_t *, va_list);
int               vfwscanf(FILE *, FAR const wchar_t *, va_list);
int               vwprintf(FAR const wchar_t *, va_list);
int               vwscanf(FAR const wchar_t *, va_list);
int               vswprintf(FAR wchar_t *, size_t, FAR const wchar_t *,
                      va_list);
int               vswscanf(FAR const wchar_t *, FAR const wchar_t *,
                      va_list);
size_t            wcrtomb(FAR char *, wchar_t, FAR mbstate_t *);
FAR wchar_t      *wcscat(FAR wchar_t *, FAR const wchar_t *);
FAR wchar_t      *wcschr(FAR const wchar_t *, wchar_t);
int               wcscmp(FAR const wchar_t *, FAR const wchar_t *);
int               wcscoll(FAR const wchar_t *, FAR const wchar_t *);
FAR wchar_t      *wcscpy(FAR wchar_t *, FAR const wchar_t *);
size_t            wcscspn(FAR const wchar_t *, FAR const wchar_t *);
size_t            wcsftime(FAR wchar_t *, size_t, FAR const wchar_t *,
                      FAR const struct tm *);
size_t            wcslen(FAR const wchar_t *);
size_t            wcslcpy(FAR wchar_t *, FAR const wchar_t *, size_t);
size_t            wcslcat(FAR wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wcsncat(FAR wchar_t *, FAR const wchar_t *, size_t);
int               wcsncmp(FAR const wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wcsncpy(FAR wchar_t *, FAR const wchar_t *, size_t);
size_t            wcsnrtombs(FAR char *, FAR const wchar_t **, size_t,
                      size_t, FAR mbstate_t *);
FAR wchar_t      *wcspbrk(FAR const wchar_t *, FAR const wchar_t *);
FAR wchar_t      *wcsrchr(FAR const wchar_t *, wchar_t);
size_t            wcsrtombs(FAR char *, FAR const wchar_t **, size_t,
                      FAR mbstate_t *);
size_t            wcsspn(FAR const wchar_t *, FAR const wchar_t *);
FAR wchar_t      *wcsstr(FAR const wchar_t *, FAR const wchar_t *);
#ifdef CONFIG_HAVE_DOUBLE
double            wcstod(FAR const wchar_t *, FAR wchar_t **);
#endif
float             wcstof(FAR const wchar_t *, FAR wchar_t **);
FAR wchar_t      *wcstok(FAR wchar_t *, FAR const wchar_t *, FAR wchar_t **);
long int          wcstol(FAR const wchar_t *, FAR wchar_t **, int);
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double       wcstold(FAR const wchar_t *, FAR wchar_t **);
#endif
long long int     wcstoll(FAR const wchar_t *, FAR wchar_t **, int);
unsigned long int wcstoul(FAR const wchar_t *, FAR wchar_t **, int);
unsigned long long int wcstoull(FAR const wchar_t *, FAR wchar_t **, int);
FAR wchar_t      *wcswcs(FAR const wchar_t *, FAR const wchar_t *);
int               wcswidth(FAR const wchar_t *, size_t);
size_t            wcsxfrm(FAR wchar_t *, FAR const wchar_t *, size_t);
int               wctob(wint_t);
int               wcwidth(wchar_t);
FAR wchar_t      *wmemchr(FAR const wchar_t *, wchar_t, size_t);
int               wmemcmp(FAR const wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemcpy(FAR wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemmove(FAR wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemset(FAR wchar_t *, wchar_t, size_t);
int               wprintf(FAR const wchar_t *, ...);
int               wscanf(FAR const wchar_t *, ...);

#if CONFIG_FORTIFY_SOURCE > 0
fortify_function(fgetws) FAR wchar_t *fgetws(FAR wchar_t *s, int n,
                                             FAR FILE *stream)
{
  fortify_assert((size_t)n <= fortify_size(s, 0) / sizeof(wchar_t));
  return __real_fgetws(s, n, stream);
}

fortify_function(mbsnrtowcs) size_t mbsnrtowcs(FAR wchar_t *dst,
                                               FAR const char **src,
                                               size_t nms,
                                               size_t len,
                                               FAR mbstate_t *ps)
{
  fortify_assert(len <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_mbsnrtowcs(dst, src, nms, len, ps);
}

fortify_function(mbsrtowcs) size_t mbsrtowcs(FAR wchar_t *dst,
                                             FAR const char **src,
                                             size_t len,
                                             FAR mbstate_t *ps)
{
  fortify_assert(len <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_mbsrtowcs(dst, src, len, ps);
}

fortify_function(wcrtomb) size_t wcrtomb(FAR char *s, wchar_t wc,
                                         FAR mbstate_t *ps)
{
  size_t ret = __real_wcrtomb(s, wc, ps);

  fortify_assert(s == NULL || ret <= fortify_size(s, 0));
  return ret;
}

fortify_function(wcscpy) FAR wchar_t *wcscpy(FAR wchar_t *dst,
                                             FAR const wchar_t *src)
{
  fortify_assert(wcslen(src) + 1 <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcscpy(dst, src);
}

fortify_function(wcslcpy) size_t wcslcpy(FAR wchar_t *dst,
                                         FAR const wchar_t *src,
                                         size_t siz)
{
  fortify_assert(siz <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcslcpy(dst, src, siz);
}

fortify_function(wcscat) FAR wchar_t *wcscat(FAR wchar_t *dst,
                                             FAR const wchar_t *src)
{
  fortify_assert(wcslen(dst) + wcslen(src) + 1 <=
                 fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcscat(dst, src);
}

fortify_function(wcsncat) FAR wchar_t *wcsncat(FAR wchar_t *dst,
                                               FAR const wchar_t *src,
                                               size_t siz)
{
  fortify_assert(siz <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcsncat(dst, src, siz);
}

fortify_function(wcslcat) size_t wcslcat(FAR wchar_t *dst,
                                         FAR const wchar_t *src,
                                         size_t siz)
{
  fortify_assert(siz <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcslcat(dst, src, siz);
}

fortify_function(wcsncpy) FAR wchar_t *wcsncpy(FAR wchar_t *dst,
                                               FAR const wchar_t *src,
                                               size_t siz)
{
  fortify_assert(siz <= fortify_size(dst, 0) / sizeof(wchar_t));
  return __real_wcsncpy(dst, src, siz);
}

fortify_function(wcsnrtombs) size_t wcsnrtombs(FAR char *dst,
                                               FAR const wchar_t **src,
                                               size_t nwc, size_t len,
                                               FAR mbstate_t *ps)
{
  fortify_assert(len <= fortify_size(dst, 0));
  return __real_wcsnrtombs(dst, src, nwc, len, ps);
}

fortify_function(wcsrtombs) size_t wcsrtombs(FAR char *dst,
                                             FAR const wchar_t **src,
                                             size_t len,
                                             FAR mbstate_t *ps)
{
  fortify_assert(len <= fortify_size(dst, 0));
  return __real_wcsrtombs(dst, src, len, ps);
}

fortify_function(wmemcpy) FAR wchar_t *wmemcpy(FAR wchar_t *d,
                                               FAR const wchar_t *s,
                                               size_t n)
{
  fortify_assert(n <= fortify_size(d, 0) / sizeof(wchar_t));
  return __real_wmemcpy(d, s, n);
}

fortify_function(wmemmove) FAR wchar_t *wmemmove(FAR wchar_t *d,
                                                 FAR const wchar_t *s,
                                                 size_t n)
{
  fortify_assert(n <= fortify_size(d, 0) / sizeof(wchar_t));
  return __real_wmemmove(d, s, n);
}

fortify_function(wmemset) FAR wchar_t *wmemset(FAR wchar_t *s,
                                               wchar_t c,
                                               size_t n)
{
  fortify_assert(n <= fortify_size(s, 0) / sizeof(wchar_t));
  return __real_wmemset(s, c, n);
}

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_WCHAR_H */
