/****************************************************************************
 * include/stddef.h
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

#ifndef __INCLUDE_STDDEF_H
#define __INCLUDE_STDDEF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* The <stddef.h> header shall define the following types:
 *
 * ptrdiff_t
 *   Signed integer type of the result of subtracting two pointers.
 *
 * wchar_t
 *   Integer type whose range of values can represent distinct wide-character
 *   codes for all members of the largest character set specified among the
 *   locales supported by the compilation environment: the null character has
 *   the code value 0 and each member of the portable character set has a
 *   code value equal to its value when used as the lone character in an
 *   integer character constant.
 *
 * size_t
 *   Unsigned integer type of the result of the sizeof operator.
 *
 * NOTE: Currently the type definitions of ptrdiff_t, wchar_t, and size_t are
 *     in sys/types.h but should be moved here sometime.
 *
 * The implementation shall support one or more programming environments in
 * which the widths of ptrdiff_t, size_t, and wchar_t are no greater than the
 * width of type long. The names of these programming environments can be
 * obtained using the confstr() function or the getconf utility.
 *
 * Reference: Opengroup.org
 */

/* Type whose alignment is supported in every context and is at least
 * as great as that of any standard type not using alignment specifiers.
 */

typedef struct
{
#if defined(CONFIG_HAVE_LONG_LONG)
  long long max_align_i;
#else
  long max_align_i;
#endif
#if defined(CONFIG_HAVE_LONG_DOUBLE)
  long double max_align_f;
#elif defined(CONFIG_HAVE_DOUBLE)
  double max_align_f;
#elif defined(CONFIG_HAVE_FLOAT)
  float max_align_f;
#endif
} max_align_t;

#endif /* __INCLUDE_STDDEF_H */
