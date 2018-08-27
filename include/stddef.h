/****************************************************************************
 * include/stddef.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_STDDEF_H
#define __INCLUDE_STDDEF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The <stddef.h> header shall define the following macros:
 *
 * NULL
 *   The null pointer constant.
 *
 *     NOTE: Currently the definition of NULL is in sys/types.h but should
 *     be moved here sometime.
 *
 * offsetof(type, member-designator)
 *   Integer constant expression of type size_t, the value of which is the
 *   offset in bytes to the structure member (member-designator), from the
 *   beginning of its structure (type).
 *
 *     NOTE: This version of offsetof() depends on behaviors that could be
 *     undefined for some compilers.  It would be better to use a builtin
 *     function if one exists.
 *
 * Reference: Opengroup.org
 */

#define offsetof(a, b) ((size_t)(&(((a *)(0))->b)))

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

#endif /* __INCLUDE_STDDEF_H */
