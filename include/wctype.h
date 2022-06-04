/****************************************************************************
 * include/wctype.h
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

#ifndef __INCLUDE_WTYPE_H
#define __INCLUDE_WTYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stddef.h>

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

#define iswalnum_l(c, l)     iswalnum(c)
#define iswalpha_l(c, l)     iswalpha(c)
#define iswblank_l(c, l)     iswblank(c)
#define iswcntrl_l(c, l)     iswcntrl(c)
#define iswdigit_l(c, l)     iswdigit(c)
#define iswgraph_l(c, l)     iswgraph(c)
#define iswlower_l(c, l)     iswlower(c)
#define iswprint_l(c, l)     iswprint(c)
#define iswpunct_l(c, l)     iswpunct(c)
#define iswspace_l(c, l)     iswspace(c)
#define iswupper_l(c, l)     iswupper(c)
#define iswxdigit_l(c, l)    iswxdigit(c)
#define iswctype_l(c, d, l)  iswctype(c, d)
#define towlower_l(c, l)     towlower(c)
#define towupper_l(c, l)     towupper(c)
#define towctrans_l(c, d, l) towctrans(c, d)
#define wctrans_l(p, l)      wctrans(p)
#define wctype_l(p, l)       wctype(p)

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
wctype_t          wctype(FAR const char *);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_WTYPE_H */
