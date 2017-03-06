/****************************************************************************
 * include/wctype.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_WTYPE_H
#define __INCLUDE_WTYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stddef.h>
#include <wchar.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef WEOF
#  define WEOF ((wint_t)-1)
#endif

/* valid values for wctype_t */

#define WC_ALNUM        1
#define WC_ALPHA        2
#define WC_BLANK        3
#define WC_CNTRL        4
#define WC_DIGIT        5
#define WC_GRAPH        6
#define WC_LOWER        7
#define WC_PRINT        8
#define WC_PUNCT        9
#define WC_SPACE        10
#define WC_UPPER        11
#define WC_XDIGIT       12

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A scalar type that can hold values which represent locale-specific
 * character mappings.
 */

typedef int wctrans_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* "The <wchar.h> header declares the following as functions and may also
 *  define them as macros. Function prototypes must be provided for use with
 *  an ISO C compiler."
 *
 * Reference: Opengroup.org
 */

int               iswalnum(wint_t);
int               iswalpha(wint_t);
int               iswblank(wint_t);
int               iswcntrl(wint_t);
int               iswctype(wint_t, wctype_t);
int               iswdigit(wint_t);
int               iswgraph(wint_t);
int               iswlower(wint_t);
int               iswprint(wint_t);
int               iswpunct(wint_t);
int               iswspace(wint_t);
int               iswupper(wint_t);
int               iswxdigit(wint_t);
int               towctrans(wint_t, wctrans_t);
wint_t            towlower(wint_t);
wint_t            towupper(wint_t);
wctrans_t         wctrans(FAR const char *);
int               iswctype(wint_t, wctype_t);
wctype_t          wctype(FAR const char *);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WTYPE_H */
