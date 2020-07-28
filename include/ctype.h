/****************************************************************************
 * include/ctype.h
 *
 *   Copyright (C) 2007-2009, 2011, 2014, 2016 Gregory Nutt.
 *   All rights reserved.
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

#ifndef __INCLUDE_CTYPE_H
#define __INCLUDE_CTYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The classification flags */

#define _U          0x01      /* upper */
#define _L          0x02      /* lower */
#define _N          0x04      /* digit */
#define _S          0x08      /* space */
#define _P          0x10      /* punct */
#define _C          0x20      /* cntrl */
#define _X          0x40      /* a-f|A-F */
#define _B          0x80      /* blank */

#define __ctype_lookup(c) up_romgetc(__ctype_ + (c))

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

extern const IOBJ char _ctype_[];
extern const IPTR char *const __ctype_;

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isspace
 *
 * Description:
 *   Checks  for  white-space characters.  In the "C" and "POSIX" locales,
 *   these are: space, form-feed ('\f'), newline ('\n'), carriage return
 *   ('\r'), horizontal tab ('\t'), and vertical tab ('\v').
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isspace(int c)
{
  return __ctype_lookup(c) & _S;
}
#else
#  define isspace(c) (__ctype_lookup(c) & _S)
#endif

/****************************************************************************
 * Name: isascii
 *
 * Description:
 *   Checks whether c is a 7-bit unsigned char value that fits into the
 *   ASCII character set.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isascii(int c)
{
  return (unsigned)c <= 0x7f;
}
#else
#  define isascii(c) ((unsigned)(c) <= 0x7f)
#endif

/****************************************************************************
 * Name: isprint
 *
 * Description:
 *   Checks for a printable character (including space)
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isprint(int c)
{
  return __ctype_lookup(c) & (_P | _U | _L | _N | _B);
}
#else
#  define isprint(c) (__ctype_lookup(c) & (_P | _U | _L | _N | _B))
#endif

/****************************************************************************
 * Name: isgraph
 *
 * Description:
 *   Checks for a printable character (excluding space)
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isgraph(int c)
{
  return __ctype_lookup(c) & (_P | _U | _L | _N);
}
#else
#  define isgraph(c) (__ctype_lookup(c) & (_P | _U | _L | _N))
#endif

/****************************************************************************
 * Name: iscntrl
 *
 * Description:
 *   Checks for control character.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int iscntrl(int c)
{
  return __ctype_lookup(c) & _C;
}
#else
#  define iscntrl(c) (__ctype_lookup(c) & _C)
#endif

/****************************************************************************
 * Name: islower
 *
 * Description:
 *   Checks for an lowercase letter.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int islower(int c)
{
  return (__ctype_lookup(c) & (_U | _L)) == _L;
}
#else
#  define islower(c) ((__ctype_lookup(c) & (_U | _L)) == _L)
#endif

/****************************************************************************
 * Name: isupper
 *
 * Description:
 *   Checks for an uppercase letter.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isupper(int c)
{
  return (__ctype_lookup(c) & (_U | _L)) == _U;
}
#else
#  define isupper(c) ((__ctype_lookup(c) & (_U | _L)) == _U)
#endif

/****************************************************************************
 * Name: isalpha
 *
 * Description:
 *   Checks for an alphabetic character
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isalpha(int c)
{
  return __ctype_lookup(c) & (_U | _L);
}
#else
#  define isalpha(c) (__ctype_lookup(c) & (_U | _L))
#endif

/****************************************************************************
 * Name: isblank
 *
 * Description:
 *   Checks for blank characters (space or tab). C++11
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isblank(int c)
{
  return (__ctype_lookup(c) & _B) || c == '\t';
}
#else
int isblank(int c);
#endif

/****************************************************************************
 * Name: isdigit
 *
 * Description:
 *   ANSI standard isdigit implementation.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isdigit(int c)
{
  return __ctype_lookup(c) & _N;
}
#else
#  define isdigit(c) (__ctype_lookup(c) & _N)
#endif

/****************************************************************************
 * Name: isalnum
 *
 * Description:
 *   Checks for an alphanumeric character
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isalnum(int c)
{
  return __ctype_lookup(c) & (_U | _L | _N);
}
#else
#  define isalnum(c) (__ctype_lookup(c) & (_U | _L | _N))
#endif

/****************************************************************************
 * Name: ispunct
 *
 * Description:
 *   Checks for a printable character which is not a space or an
 *   alphanumeric character
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int ispunct(int c)
{
  return __ctype_lookup(c) & _P;
}
#else
#  define ispunct(c) (__ctype_lookup(c) & _P)
#endif

/****************************************************************************
 * Name: isxdigit
 *
 * Description:
 *   isxdigit() checks for a hexadecimal digits, i.e. one of {0-9,a-f,A-F}
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int isxdigit(int c)
{
  return __ctype_lookup(c) & (_X | _N);
}
#else
#  define isxdigit(c) (__ctype_lookup(c) & (_X | _N))
#endif

/****************************************************************************
 * Name: toupper
 *
 * Description:
 *   toupper() converts the letter c to upper case, if possible.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int toupper(int c)
{
  return islower(c) ? (c - 'a' + 'A') : c;
}
#else
int toupper(int c);
#endif

/****************************************************************************
 * Name: tolower
 *
 * Description:
 *   tolower() converts the letter c to lower case, if possible.
 *
 ****************************************************************************/

#ifdef __cplusplus
static inline int tolower(int c)
{
  return isupper(c) ? (c - 'A' + 'a') : c;
}
#else
int tolower(int c);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_CTYPE_H */
