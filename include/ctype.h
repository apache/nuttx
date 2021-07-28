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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define isalnum_l(c, l)  isalnum(c)
#define isalpha_l(c, l)  isalpha(c)
#define isascii_l(c, l)  isascii(c)
#define isblank_l(c, l)  isblank(c)
#define iscntrl_l(c, l)  iscntrl(c)
#define isdigit_l(c, l)  isdigit(c)
#define isgraph_l(c, l)  isgraph(c)
#define islower_l(c, l)  islower(c)
#define isprint_l(c, l)  isprint(c)
#define ispunct_l(c, l)  ispunct(c)
#define isspace_l(c, l)  isspace(c)
#define isupper_l(c, l)  isupper(c)
#define isxdigit_l(c, l) isxdigit(c)
#define tolower_l(c, l)  tolower(c)
#define toupper_l(c, l)  toupper(c)

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
#else
int isspace(int c);
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
#else
int isascii(int c);
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
#else
int isprint(int c);
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
#else
int isgraph(int c);
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
#else
int iscntrl(int c);
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
#else
int islower(int c);
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
#else
int isupper(int c);
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
#else
int isalpha(int c);
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
  return c >= '0' && c <= '9';
}
#else
int isdigit(int c);
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
#else
int isalnum(int c);
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
#else
int ispunct(int c);
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
#else
int isxdigit(int c);
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
  return (c >= 'A' && c <= 'Z') ? (c - 'A' + 'a') : c;
}
#else
int tolower(int c);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_CTYPE_H */
