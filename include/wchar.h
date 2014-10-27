/****************************************************************************
 * include/wchar.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
/* "The <wchar.h> header defines the following data types through
 *
 * wchar_t
 *   As described in <stddef.h>.
 *
 * wint_t
 *   An integral type capable of storing any valid value of wchar_t, or WEOF.
 *
 * wctype_t
 *   A scalar type of a data object that can hold values which represent
 *   locale-specific character classification.
 *
 * mbstate_t
 *   An object type other than an array type that can hold the conversion
 *   state information necessary to convert between sequences of (possibly
 *   multibyte) characters and wide-characters. If a codeset is being used
 *   such that an mbstate_t needs to preserve more than 2 levels of reserved
 *   state, the results are unspecified.
 *
 * FILE
 *   As described in <stdio.h>.
 *
 * size_t
 *   As described in <stddef.h>."
 *
 * Reference: Opengroup.org
 */

typedef int wint_t;

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

#if 0 /* Not yet implemented */
wint_t            btowc(int);
int               fwprintf(FILE *, const wchar_t *, ...);
int               fwscanf(FILE *, const wchar_t *, ...);
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
wchar_t          *fgetws(wchar_t *, int, FILE *);
wint_t            fputwc(wchar_t, FILE *);
int               fputws(const wchar_t *, FILE *);
int               fwide(FILE *, int);
wint_t            getwc(FILE *);
wint_t            getwchar(void);
int               mbsinit(const mbstate_t *);
size_t            mbrlen(const char *, size_t, mbstate_t *);
size_t            mbrtowc(wchar_t *, const char *, size_t,
                      mbstate_t *);
size_t            mbsrtowcs(wchar_t *, const char **, size_t,
                      mbstate_t *);
wint_t            putwc(wchar_t, FILE *);
wint_t            putwchar(wchar_t);
int               swprintf(wchar_t *, size_t, const wchar_t *, ...);
int               swscanf(const wchar_t *, const wchar_t *, ...);
wint_t            towlower(wint_t);
wint_t            towupper(wint_t);
wint_t            ungetwc(wint_t, FILE *);
int               vfwprintf(FILE *, const wchar_t *, va_list);
int               vwprintf(const wchar_t *, va_list);
int               vswprintf(wchar_t *, size_t, const wchar_t *,
                      va_list);
size_t            wcrtomb(char *, wchar_t, mbstate_t *);
wchar_t          *wcscat(wchar_t *, const wchar_t *);
wchar_t          *wcschr(const wchar_t *, wchar_t);
int               wcscmp(const wchar_t *, const wchar_t *);
int               wcscoll(const wchar_t *, const wchar_t *);
wchar_t          *wcscpy(wchar_t *, const wchar_t *);
size_t            wcscspn(const wchar_t *, const wchar_t *);
size_t            wcsftime(wchar_t *, size_t, const wchar_t *,
                      const struct tm *);
size_t            wcslen(const wchar_t *);
wchar_t          *wcsncat(wchar_t *, const wchar_t *, size_t);
int               wcsncmp(const wchar_t *, const wchar_t *, size_t);
wchar_t          *wcsncpy(wchar_t *, const wchar_t *, size_t);
wchar_t          *wcspbrk(const wchar_t *, const wchar_t *);
wchar_t          *wcsrchr(const wchar_t *, wchar_t);
size_t            wcsrtombs(char *, const wchar_t **, size_t,
                      mbstate_t *);
size_t            wcsspn(const wchar_t *, const wchar_t *);
wchar_t          *wcsstr(const wchar_t *, const wchar_t *);
double            wcstod(const wchar_t *, wchar_t **);
wchar_t          *wcstok(wchar_t *, const wchar_t *, wchar_t **);
long int          wcstol(const wchar_t *, wchar_t **, int);
unsigned long int wcstoul(const wchar_t *, wchar_t **, int);
wchar_t          *wcswcs(const wchar_t *, const wchar_t *);
int               wcswidth(const wchar_t *, size_t);
size_t            wcsxfrm(wchar_t *, const wchar_t *, size_t);
int               wctob(wint_t);
wctype_t          wctype(const char *);
int               wcwidth(wchar_t);
wchar_t          *wmemchr(const wchar_t *, wchar_t, size_t);
int               wmemcmp(const wchar_t *, const wchar_t *, size_t);
wchar_t          *wmemcpy(wchar_t *, const wchar_t *, size_t);
wchar_t          *wmemmove(wchar_t *, const wchar_t *, size_t);
wchar_t          *wmemset(wchar_t *, wchar_t, size_t);
int               wprintf(const wchar_t *, ...);
int               wscanf(const wchar_t *, ...);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_WCHAR_H */
