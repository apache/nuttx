/****************************************************************************
 * include/ctype.h
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

#ifndef __INCLUDE_CTYPE_H
#define __INCLUDE_CTYPE_H

/* There is no consistent ctype implementation, just a smattering of
 * functions.  Individually, they are okay, but a more standard, data lookup
 * approach would make more sense if used extensively.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <langinfo.h>

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

/* GNU libstdc++ is expecting ctype.h to define a few macros for
 * locale related functions like C++ streams.
 */

#define _U  01
#define _L  02
#define _N  04
#define _S  010
#define _P  020
#define _C  040
#define _X  0100
#define _B  0200

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' ||
         c == '\f' || c == '\v';
}

static inline int isspace_l(int c, locale_t locale)
{
  return isspace(c);
}
#else
int isspace(int c);
int isspace_l(int c, locale_t locale);
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
  return c >= 0 && c <= 0x7f;
}

static inline int isascii_l(int c, locale_t locale)
{
  return isascii(c);
}
#else
int isascii(int c);
int isascii_l(int c, locale_t locale);
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
  return c >= 0x20 && c < 0x7f;
}

static inline int isprint_l(int c, locale_t locale)
{
  return isprint(c);
}
#else
int isprint(int c);
int isprint_l(int c, locale_t locale);
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
  return c > 0x20 && c < 0x7f;
}

static inline int isgraph_l(int c, locale_t locale)
{
  return isgraph(c);
}
#else
int isgraph(int c);
int isgraph_l(int c, locale_t locale);
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
  return c < 0x20 || c == 0x7f;
}

static inline int iscntrl_l(int c, locale_t locale)
{
  return iscntrl(c);
}
#else
int iscntrl(int c);
int iscntrl_l(int c, locale_t locale);
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
  return c >= 'a' && c <= 'z';
}

static inline int islower_l(int c, locale_t locale)
{
  return islower(c);
}
#else
int islower(int c);
int islower_l(int c, locale_t locale);
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
  return c >= 'A' && c <= 'Z';
}

static inline int isupper_l(int c, locale_t locale)
{
  return isupper(c);
}
#else
int isupper(int c);
int isupper_l(int c, locale_t locale);
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
  return islower(c) || isupper(c);
}

static inline int isalpha_l(int c, locale_t locale)
{
  return isalpha(c);
}
#else
int isalpha(int c);
int isalpha_l(int c, locale_t locale);
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
  return c == ' ' || c == '\t';
}

static inline int isblank_l(int c, locale_t locale)
{
  return isblank(c);
}
#else
int isblank(int c);
int isblank_l(int c, locale_t locale);
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
  return c >= '0' && c <= '9';
}

static inline int isdigit_l(int c, locale_t locale)
{
  return isdigit(c);
}
#else
int isdigit(int c);
int isdigit_l(int c, locale_t locale);
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
  return isalpha(c) || isdigit(c);
}

static inline int isalnum_l(int c, locale_t locale)
{
  return isalnum(c);
}
#else
int isalnum(int c);
int isalnum_l(int c, locale_t locale);
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
  return isgraph(c) && !isalnum(c);
}

static inline int ispunct_l(int c, locale_t locale)
{
  return ispunct(c);
}
#else
int ispunct(int c);
int ispunct_l(int c, locale_t locale);
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
  return (c >= '0' && c <= '9') ||
         (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}

static inline int isxdigit_l(int c, locale_t locale)
{
  return isxdigit(c);
}
#else
int isxdigit(int c);
int isxdigit_l(int c, locale_t locale);
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
  return (c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c;
}

static inline int toupper_l(int c, locale_t locale)
{
  return toupper(c);
}
#else
int toupper(int c);
int toupper_l(int c, locale_t locale);
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
  return (c >= 'A' && c <= 'Z') ? (c - 'A' + 'a') : c;
}

static inline int tolower_l(int c, locale_t locale)
{
  return tolower(c);
}
#else
int tolower(int c);
int tolower_l(int c, locale_t locale);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_CTYPE_H */
