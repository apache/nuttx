/************************************************************
 * ctype.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __CTYPE_H
#define __CTYPE_H

/* There is no consistent ctype implementation, just a
 * smattering of functions.  Individually, they are okay, but
 * a more standard, data lookup approach would make more sense
 * if used extensively.
 */

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>

/************************************************************
 * Public Type Definitions
 ************************************************************/

/************************************************************
 * Function:  isspace
 *
 * Description:
 * Checks  for  white-space characters.  In the "C" and "POSIX"
 * locales, these are: space, form-feed ('\f'), newline ('\n'),
 * carriage return ('\r'), horizontal tab ('\t'), and vertical
 * tab ('\v').
 *
 ************************************************************/

static inline int isspace(int c)
{
  if (c == ' '  || c == '\t' || c == '\n' || \
      c == '\r' || c == '\f' || c == '\v')
    {
      return TRUE;
    }
  else
    {
      return FALSE;
    }
}

/************************************************************
 * Function:  isdigit
 *
 * Description:
 *    ANSI standard isdigit implementation.
 *
 ************************************************************/

static inline int isdigit(int c)
{
  return (c >= '0' && c <= '9');
}

/************************************************************
 * Function:  isascii
 *
 * Description:
 *  Checks whether c is a 7-bit unsigned char value that
 *  fits into the ASCII character set.
 *
 ************************************************************/

static inline int isascii(int c)
{
  return (c >= 0 && c <= 0177);
}


/************************************************************
 * Function:  isascii
 *
 * Description:
 *   isxdigit() checks for a hexadecimal digits, i.e. one of
 *   {0-9,a-f,A-F}
 *
 ************************************************************/

static inline int isxdigit(int c)
{
  if ((c >= '0' && c <= '9') ||
      (c >= 'a' && c <= 'f') ||
      (c >= 'A' && c <= 'F'))
    {
      return 1;
    }
  else
    {

      return 0;
    }
}

/************************************************************
 * Function:  isascii
 *
 * Description:
 *   toupper() converts the letter c to upper case, if possible.
 *
 ************************************************************/

static inline int toupper(int c)
{
  if (c >= 'a' && c <= 'z')
    {
      return c - 'a' + 'A';
    }
  else
    {
      return c;
    }
}

/************************************************************
 * Function:  isascii
 *
 * Description:
 *   tolower() converts the letter c to lower case, if possible.
 *
 ************************************************************/

static inline int tolower(int c)
{
  if (c >= 'A' && c <= 'Z')
    {
      return c - 'A' + 'a';
    }
  else
    {
      return c;
    }
}

/************************************************************
 * Public Functions
 ************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __CTYPE_H */
