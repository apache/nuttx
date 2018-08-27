/****************************************************************************
 * include/wchar.h
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#define WCHAR_MAX 0xffff
#define WCHAR_MIN 0x0000
#define WEOF      ((wint_t)-1)

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

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
int               iswalnum(wint_t);
int               iswalpha(wint_t);
int               iswcntrl(wint_t);
int               iswdigit(wint_t);
int               iswgraph(wint_t);
int               iswlower(wint_t);
int               iswprint(wint_t);
int               iswpunct(wint_t);
int               iswspace(wint_t);
int               iswupper(wint_t);
int               iswxdigit(wint_t);
int               iswctype(wint_t, wctype_t);
wint_t            fgetwc(FILE *);
FAR wchar_t      *fgetws(wchar_t *, int, FILE *);
wint_t            fputwc(wchar_t, FILE *);
int               fputws(FAR const wchar_t *, FILE *);
int               fwide(FILE *, int);
wint_t            getwc(FILE *);
wint_t            getwchar(void);
int               mbsinit(FAR const mbstate_t *);
size_t            mbrlen(FAR const char *, size_t, FAR mbstate_t *);
size_t            mbrtowc(wchar_t *, FAR const char *, size_t,
                      mbstate_t *);
size_t            mbsnrtowcs(FAR wchar_t *, FAR const char **, size_t,
                      size_t, FAR mbstate_t *);
size_t            mbsrtowcs(wchar_t *, FAR const char **, size_t,
                      FAR mbstate_t *);
wint_t            putwc(wchar_t, FILE *);
wint_t            putwchar(wchar_t);
int               swprintf(FAR wchar_t *, size_t, FAR const wchar_t *, ...);
int               swscanf(FAR const wchar_t *, FAR const wchar_t *, ...);
wint_t            towlower(wint_t);
wint_t            towupper(wint_t);
wint_t            ungetwc(wint_t, FILE *);
int               vfwprintf(FILE *, FAR const wchar_t *, va_list);
int               vwprintf(FAR const wchar_t *, va_list);
int               vswprintf(wchar_t *, size_t, FAR const wchar_t *,
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
size_t            wcsxfrm(wchar_t *, FAR const wchar_t *, size_t);
int               wctob(wint_t);
wctype_t          wctype(FAR const char *);
int               wcwidth(wchar_t);
FAR wchar_t      *wmemchr(FAR const wchar_t *, wchar_t, size_t);
int               wmemcmp(FAR const wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemcpy(FAR wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemmove(FAR wchar_t *, FAR const wchar_t *, size_t);
FAR wchar_t      *wmemset(FAR wchar_t *, wchar_t, size_t);
int               wprintf(FAR const wchar_t *, ...);
int               wscanf(FAR const wchar_t *, ...);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_WCHAR_H */
