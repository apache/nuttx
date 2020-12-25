/****************************************************************************
 * libs/libc/stdio/legacy_libvsprintf.c
 *
 *   Copyright (C) 2007-2012, 2018-2019 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <wchar.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLAG_SHOWPLUS            0x0001
#define FLAG_ALTFORM             0x0002
#define FLAG_HASDOT              0x0004
#define FLAG_HASASTERISKWIDTH    0x0008
#define FLAG_HASASTERISKTRUNC    0x0010
#define FLAG_LONGPRECISION       0x0020
#define FLAG_LONGLONGPRECISION   0x0040
#define FLAG_NEGATE              0x0080
#define FLAG_NOTRAILINGZERO      0x0100

#define SET_SHOWPLUS(f)          do (f) |= FLAG_SHOWPLUS; while (0)
#define SET_ALTFORM(f)           do (f) |= FLAG_ALTFORM; while (0)
#define SET_HASDOT(f)            do (f) |= FLAG_HASDOT; while (0)
#define SET_HASASTERISKWIDTH(f)  do (f) |= FLAG_HASASTERISKWIDTH; while (0)
#define SET_HASASTERISKTRUNC(f)  do (f) |= FLAG_HASASTERISKTRUNC; while (0)
#define SET_LONGPRECISION(f)     do (f) |= FLAG_LONGPRECISION; while (0)
#define SET_LONGLONGPRECISION(f) do (f) |= FLAG_LONGLONGPRECISION; while (0)
#define SET_NEGATE(f)            do (f) |= FLAG_NEGATE; while (0)
#define SET_NOTRAILINGZERO(f)    do (f) |= FLAG_NOTRAILINGZERO; while (0)

#define CLR_SHOWPLUS(f)          do (f) &= ~FLAG_SHOWPLUS; while (0)
#define CLR_ALTFORM(f)           do (f) &= ~FLAG_ALTFORM; while (0)
#define CLR_HASDOT(f)            do (f) &= ~FLAG_HASDOT; while (0)
#define CLR_HASASTERISKWIDTH(f)  do (f) &= ~FLAG_HASASTERISKWIDTH; while (0)
#define CLR_HASASTERISKTRUNC(f)  do (f) &= ~FLAG_HASASTERISKTRUNC; while (0)
#define CLR_LONGPRECISION(f)     do (f) &= ~FLAG_LONGPRECISION; while (0)
#define CLR_LONGLONGPRECISION(f) do (f) &= ~FLAG_LONGLONGPRECISION; while (0)
#define CLR_NEGATE(f)            do (f) &= ~FLAG_NEGATE; while (0)
#define CLR_SIGNED(f)            do (f) &= ~(FLAG_SHOWPLUS|FLAG_NEGATE); while (0)
#define CLR_NOTRAILINGZERO(f)    do (f) &= ~FLAG_NOTRAILINGZERO; while (0)

#define IS_SHOWPLUS(f)           (((f) & FLAG_SHOWPLUS) != 0)
#define IS_ALTFORM(f)            (((f) & FLAG_ALTFORM) != 0)
#define IS_HASDOT(f)             (((f) & FLAG_HASDOT) != 0)
#define IS_HASASTERISKWIDTH(f)   (((f) & FLAG_HASASTERISKWIDTH) != 0)
#define IS_HASASTERISKTRUNC(f)   (((f) & FLAG_HASASTERISKTRUNC) != 0)
#define IS_LONGPRECISION(f)      (((f) & FLAG_LONGPRECISION) != 0)
#define IS_LONGLONGPRECISION(f)  (((f) & FLAG_LONGLONGPRECISION) != 0)
#define IS_NEGATE(f)             (((f) & FLAG_NEGATE) != 0)
#define IS_SIGNED(f)             (((f) & (FLAG_SHOWPLUS|FLAG_NEGATE)) != 0)
#define IS_NOTRAILINGZERO(f)     (((f) & FLAG_NOTRAILINGZERO) != 0)

/* If CONFIG_ARCH_ROMGETC is defined, then it is assumed that the format
 * string data cannot be accessed by simply de-referencing the format string
 * pointer.  This might be in the case in Harvard architectures where string
 * data might be stored in instruction space or if string data were stored
 * on some media like EEPROM or external serial FLASH. In all of these cases,
 * string data has to be accessed indirectly using the architecture-supplied
 * up_romgetc().  The following mechanisms attempt to make these different
 * access methods indistinguishable in the following code.
 *
 * NOTE: It is assumed that string arguments for %s still reside in memory
 * that can be directly accessed by de-referencing the string pointer.
 */

#ifdef CONFIG_ARCH_ROMGETC
#  define FMT_TOP      ch = up_romgetc(src)         /* Loop initialization */
#  define FMT_BOTTOM   src++, ch = up_romgetc(src)  /* Bottom of a loop */
#  define FMT_CHAR     ch                           /* Access a character */
#  define FMT_NEXT     src++; ch = up_romgetc(src)  /* Advance to the next character */
#  define FMT_PREV     src--; ch = up_romgetc(src)  /* Backup to the previous character */
#else
#  define FMT_TOP                                   /* Loop initialization */
#  define FMT_BOTTOM   src++                        /* Bottom of a loop */
#  define FMT_CHAR     *src                         /* Access a character */
#  define FMT_NEXT     src++                        /* Advance to the next character */
#  define FMT_PREV     src--                        /* Backup to the previous character */
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

enum
{
  FMT_RJUST = 0, /* Default */
  FMT_LJUST,
  FMT_RJUST0,
  FMT_CENTER
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Pointer to ASCII conversion */

#ifdef CONFIG_PTR_IS_NOT_INT
static void ptohex(FAR struct lib_outstream_s *obj, uint16_t flags,
                   FAR void *p);
static int  getsizesize(uint8_t fmt, uint16_t flags, FAR void *p)
#endif /* CONFIG_PTR_IS_NOT_INT */

/* Unsigned int to ASCII conversion */

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n);
static void utohex(FAR struct lib_outstream_s *obj, unsigned int n,
                   uint8_t a);
static void utooct(FAR struct lib_outstream_s *obj, unsigned int n);
static void utobin(FAR struct lib_outstream_s *obj, unsigned int n);
static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                     uint16_t flags, unsigned int lln);

static void fixup(uint8_t fmt, FAR uint16_t *flags, int *n);
static int  getusize(uint8_t fmt, uint16_t flags, unsigned int lln);

/* Unsigned long int to ASCII conversion */

#ifdef CONFIG_LONG_IS_NOT_INT
static void lutodec(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutohex(FAR struct lib_outstream_s *obj, unsigned long ln,
                    uint8_t a);
static void lutooct(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutobin(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                      uint16_t flags, unsigned long ln);
static void lfixup(uint8_t fmt, FAR uint16_t *flags, long *ln);
static int  getlusize(uint8_t fmt, FAR uint16_t flags, unsigned long ln);
#endif

/* Unsigned long long int to ASCII conversions */

#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_LIBC_LONG_LONG)
static void llutodec(FAR struct lib_outstream_s *obj,
                     unsigned long long lln);
static void llutohex(FAR struct lib_outstream_s *obj,
                     unsigned long long lln, uint8_t a);
static void llutooct(FAR struct lib_outstream_s *obj,
                     unsigned long long lln);
static void llutobin(FAR struct lib_outstream_s *obj,
                     unsigned long long lln);
static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint16_t flags, unsigned long long lln);
static void llfixup(uint8_t fmt, FAR uint16_t *flags, FAR long long *lln);
static int  getllusize(uint8_t fmt, FAR uint16_t flags,
                       FAR unsigned long long lln);
#endif

static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t justify, uint16_t flags, int fieldwidth,
                       int valwidth, int trunc);
static void postjustify(FAR struct lib_outstream_s *obj, uint8_t justify,
                        uint16_t flags, int fieldwidth, int valwidth,
                        int trunc);

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_nullstring[] = "(null)";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Include floating point functions */

#ifdef CONFIG_LIBC_FLOATINGPOINT
#  include "stdio/lib_libdtoa.c"
#endif

/****************************************************************************
 * Name: ptohex
 ****************************************************************************/

#ifdef CONFIG_PTR_IS_NOT_INT
static void ptohex(FAR struct lib_outstream_s *obj, uint16_t flags,
                   FAR void *p)
{
  uint8_t bits;
  union
  {
    uint32_t  dw;
    FAR void *p;
  } u;

  /* Check for alternate form */

  if (IS_ALTFORM(flags))
    {
      /* Prefix the number with "0x" */

      obj->put(obj, '0');
      obj->put(obj, 'x');
    }

  u.dw = 0;
  u.p  = p;

  for (bits = CHAR_BIT * sizeof(FAR void *); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((u.dw >> (bits - 4)) & 0xf);
      if (nibble < 10)
        {
          obj->put(obj, nibble + '0');
        }
      else
        {
          obj->put(obj, nibble + 'a' - 10);
        }
    }
}

/****************************************************************************
 * Name: getpsize
 ****************************************************************************/

static int getpsize(uint16_t flags, FAR void *p)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  ptohex(&nulloutstream, flags, p);
  return nulloutstream.nput;
}

#endif /* CONFIG_PTR_IS_NOT_INT */

/****************************************************************************
 * Name: utodec
 ****************************************************************************/

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n)
{
  char buf[16];
  int i = 0;

  do
    {
      buf[i++] = n % 10 + '0';
      n /= 10;
    }
  while (n > 0);

  while (i > 0)
    {
      obj->put(obj, buf[--i]);
    }
}

/****************************************************************************
 * Name: utohex
 ****************************************************************************/

static void utohex(FAR struct lib_outstream_s *obj, unsigned int n,
                   uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned int); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, nibble + '0');
            }
          else
            {
              obj->put(obj, nibble + a - 10);
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: utooct
 ****************************************************************************/

static void utooct(FAR struct lib_outstream_s *obj, unsigned int n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned int); bits > 0; bits -= 3)
    {
      uint8_t tribit = (uint8_t)((n >> (bits - 3)) & 0x7);
      if (tribit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (tribit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: utobin
 ****************************************************************************/

static void utobin(FAR struct lib_outstream_s *obj, unsigned int n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned int); bits > 0; bits -= 1)
    {
      uint8_t unibit = (uint8_t)((n >> (bits - 1)) & 0x1);
      if (unibit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (unibit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: utoascii
 ****************************************************************************/

static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                     uint16_t flags, unsigned int n)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed/unsigned base 10 */
      case 'i':
      case 'u':
        {
          /* Convert the integer value to a string. */

          utodec(obj, n);
        }
        break;

#ifndef CONFIG_PTR_IS_NOT_INT
      case 'p':  /* Hexadecimal */
#endif
      case 'x':
      case 'X':
        {
          /* Convert the unsigned value to a string.
           *
           * NOTE that the alternate form prefix was already applied in
           * prejustify().
           */

          if (fmt == 'X')
            {
              utohex(obj, n, 'A');
            }
          else
            {
              utohex(obj, n, 'a');
            }
        }
        break;

      case 'o':  /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           utooct(obj, n);
         }
         break;

      case 'b': /* Binary */
        {
          /* Convert the unsigned value to a string. */

          utobin(obj, n);
        }
        break;

#ifdef CONFIG_PTR_IS_NOT_INT
      case 'p':
#endif
      default:
        break;
    }
}

/****************************************************************************
 * Name: fixup
 ****************************************************************************/

static void fixup(uint8_t fmt, FAR uint16_t *flags, FAR int *n)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed base 10 */
      case 'i':
        if (*n < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *n = -*n;
          }
        break;

      case 'u':  /* Unsigned base 10 */
        break;

      case 'p':  /* Hexadecimal */
      case 'x':
      case 'X':
      case 'o':  /* Octal */
      case 'b':  /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getusize
 ****************************************************************************/

static int getusize(uint8_t fmt, uint16_t flags, unsigned int n)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  utoascii(&nulloutstream, fmt, flags, n);
  return nulloutstream.nput;
}

/****************************************************************************
 * Name: getdblsize
 ****************************************************************************/

#ifdef CONFIG_LIBC_FLOATINGPOINT
static int getdblsize(uint8_t fmt, int trunc, uint16_t flags, double n)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  lib_dtoa(&nulloutstream, fmt, trunc, flags, n);
  return nulloutstream.nput;
}
#endif

#ifdef CONFIG_LONG_IS_NOT_INT

/****************************************************************************
 * Name: lutodec
 ****************************************************************************/

static void lutodec(FAR struct lib_outstream_s *obj, unsigned long n)
{
  char buf[32];
  int i = 0;

  do
    {
      buf[i++] = n % 10 + '0';
      n /= 10;
    }
  while (n > 0);

  while (i > 0)
    {
      obj->put(obj, buf[--i]);
    }
}

/****************************************************************************
 * Name: lutohex
 ****************************************************************************/

static void lutohex(FAR struct lib_outstream_s *obj, unsigned long n,
                    uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned long); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, nibble + '0');
            }
          else
            {
              obj->put(obj, nibble + a - 10);
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: lutooct
 ****************************************************************************/

static void lutooct(FAR struct lib_outstream_s *obj, unsigned long n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned long); bits > 0; bits -= 3)
    {
      uint8_t tribit = (uint8_t)((n >> (bits - 3)) & 0x7);
      if (tribit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (tribit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: lutobin
 ****************************************************************************/

static void lutobin(FAR struct lib_outstream_s *obj, unsigned long n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned long); bits > 0; bits -= 1)
    {
      uint8_t unibit = (uint8_t)((n >> (bits - 1)) & 0x1);
      if (unibit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (unibit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: lutoascii
 ****************************************************************************/

static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                      uint16_t flags, unsigned long ln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed/unsigned base 10 */
      case 'i':
      case 'u':
        {
          /* Convert the long integer value to a string. */

          lutodec(obj, ln);
        }
        break;

      case 'x':  /* Hexadecimal */
      case 'X':
        {
          /* Convert the unsigned value to a string.
           *
           * NOTE that the alternate form prefix was already applied in
           * prejustify().
           */

          if (fmt == 'X')
            {
              lutohex(obj, ln, 'A');
            }
          else
            {
              lutohex(obj, ln, 'a');
            }
        }
        break;

      case 'o':  /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           lutooct(obj, ln);
         }
         break;

      case 'b':  /* Binary */
        {
          /* Convert the unsigned value to a string. */

          lutobin(obj, ln);
        }
        break;

      case 'p':
      default:
        break;
    }
}

/****************************************************************************
 * Name: lfixup
 ****************************************************************************/

static void lfixup(uint8_t fmt, FAR uint16_t *flags, FAR long *ln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed base 10 */
      case 'i':
        if (*ln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *ln = -*ln;
          }
        break;

      case 'u':  /* Unsigned base 10 */
        break;

      case 'p':  /* Hexadecimal */
      case 'x':
      case 'X':
      case 'o':  /* Octal */
      case 'b':  /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getlusize
 ****************************************************************************/

static int getlusize(uint8_t fmt, uint16_t flags, unsigned long ln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  lutoascii(&nulloutstream, fmt, flags, ln);
  return nulloutstream.nput;
}

#endif /* CONFIG_LONG_IS_NOT_INT */

#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_LIBC_LONG_LONG)
/****************************************************************************
 * Name: llutodec
 ****************************************************************************/

static void llutodec(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  char buf[32];
  int i = 0;

  do
    {
      buf[i++] = n % 10 + '0';
      n /= 10;
    }
  while (n > 0);

  while (i > 0)
    {
      obj->put(obj, buf[--i]);
    }
}

/****************************************************************************
 * Name: llutohex
 ****************************************************************************/

static void llutohex(FAR struct lib_outstream_s *obj, unsigned long long n,
                     uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT*sizeof(unsigned long long); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, (nibble + '0'));
            }
          else
            {
              obj->put(obj, (nibble + a - 10));
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: llutooct
 ****************************************************************************/

static void llutooct(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned long long); bits > 0; bits -= 3)
    {
      uint8_t tribit = (uint8_t)((n >> (bits - 3)) & 0x7);
      if (tribit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (tribit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: llutobin
 ****************************************************************************/

static void llutobin(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = CHAR_BIT * sizeof(unsigned long long); bits > 0; bits -= 1)
    {
      uint8_t unibit = (uint8_t)((n >> (bits - 1)) & 0x1);
      if (unibit || nonzero)
        {
          nonzero = true;
          obj->put(obj, (unibit + '0'));
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: llutoascii
 ****************************************************************************/

static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint16_t flags, unsigned long long lln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed/unsigned base 10 */
      case 'i':
      case 'u':
        {
          /* Convert the long long integer value to a string. */

          llutodec(obj, (unsigned long long)lln);
        }
        break;

      case 'x':  /* Hexadecimal */
      case 'X':
        {
          /* Convert the unsigned value to a string.
           *
           * NOTE that the alternate form prefix was already applied in
           * prejustify().
           */

          if (fmt == 'X')
            {
              llutohex(obj, (unsigned long long)lln, 'A');
            }
          else
            {
              llutohex(obj, (unsigned long long)lln, 'a');
            }
        }
        break;

      case 'o':  /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           llutooct(obj, (unsigned long long)lln);
         }
         break;

      case 'b':  /* Binary */
        {
          /* Convert the unsigned value to a string. */

          llutobin(obj, (unsigned long long)lln);
        }
        break;

      case 'p':
      default:
        break;
    }
}

/****************************************************************************
 * Name: llfixup
 ****************************************************************************/

static void llfixup(uint8_t fmt, FAR uint16_t *flags, FAR long long *lln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':  /* Signed base 10 */
      case 'i':
        if (*lln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *lln = -*lln;
          }
        break;

      case 'u':  /* Unsigned base 10 */
        break;

      case 'p':  /* Hexadecimal */
      case 'x':
      case 'X':
      case 'o':  /* Octal */
      case 'b':  /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getllusize
 ****************************************************************************/

static int getllusize(uint8_t fmt, uint16_t flags, unsigned long long lln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  llutoascii(&nulloutstream, fmt, flags, lln);
  return nulloutstream.nput;
}

#endif /* CONFIG_HAVE_LONG_LONG */

/****************************************************************************
 * Name: prejustify
 ****************************************************************************/

static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t justify, uint16_t flags, int fieldwidth,
                       int valwidth, int trunc)
{
  bool althex = (fmt == 'x' || fmt == 'X' || fmt == 'p' || fmt == 'P')
                && IS_ALTFORM(flags);
  int i;

  /* If there is integer precision, then use FMT_RJUST vs FMT_RJUST0 */

  if (trunc > 0 && justify == FMT_RJUST0)
    {
      /* Force right justification in the case.  Leading zeros application
       * only to "precision" which is implied anyway.
       */

      justify = FMT_RJUST;
    }

  switch (justify)
    {
      default:
      case FMT_RJUST:
        {
          /* Pad with spaces up to the  size of the precision (aka 'trunc')
           * then with zeroes to the size of the value width.
           */

          if (trunc > valwidth)
            {
              int padlen = fieldwidth - trunc;

              if (IS_SIGNED(flags))
                {
                  padlen--;
                }

              if (althex)
                {
                  padlen -= 2;
                }

              for (i = padlen; i > 0; i--)
                {
                  obj->put(obj, ' ');
                }

              if (IS_NEGATE(flags))
                {
                  obj->put(obj, '-');
                }
              else if (IS_SHOWPLUS(flags))
                {
                  obj->put(obj, '+');
                }

              if (althex)
                {
                  obj->put(obj, '0');
                  obj->put(obj, 'x');
                }

              for (i = trunc - valwidth; i > 0; i--)
                {
                  obj->put(obj, '0');
                }
            }
          else
            {
              /* Add a leading minus sign */

              if (IS_SIGNED(flags))
                {
                  valwidth++;
                }

              if (althex)
                {
                  valwidth += 2;
                }

              for (i = fieldwidth - valwidth; i > 0; i--)
                {
                  obj->put(obj, ' ');
                }

              if (IS_NEGATE(flags))
                {
                  obj->put(obj, '-');
                }
              else if (IS_SHOWPLUS(flags))
                {
                  obj->put(obj, '+');
                }

              if (althex)
                {
                  obj->put(obj, '0');
                  obj->put(obj, 'x');
                }
            }
        }
        break;

      case FMT_RJUST0:
        {
          if (IS_NEGATE(flags))
            {
              obj->put(obj, '-');
              valwidth++;
            }
          else if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
              valwidth++;
            }

          if (althex)
            {
              obj->put(obj, '0');
              obj->put(obj, 'x');
              valwidth += 2;
            }

          for (i = fieldwidth - valwidth; i > 0; i--)
            {
              obj->put(obj, '0');
            }
        }
        break;

      case FMT_LJUST:
        {
          if (IS_NEGATE(flags))
           {
             obj->put(obj, '-');
           }
         else if (IS_SHOWPLUS(flags))
           {
             obj->put(obj, '+');
           }

         if (althex)
           {
             obj->put(obj, '0');
             obj->put(obj, 'x');
           }

          /* Pad with zeros up to the size of the value width. */

          for (i = trunc - valwidth; i > 0; i--)
            {
              obj->put(obj, '0');
            }
        }
        break;
    }
}

/****************************************************************************
 * Name: postjustify
 ****************************************************************************/

static void postjustify(FAR struct lib_outstream_s *obj, uint8_t justify,
                        uint16_t flags, int fieldwidth, int valwidth,
                        int trunc)
{
  int i;

  /* Apply field justification to the integer value. */

  switch (justify)
    {
      default:
      case FMT_RJUST:
      case FMT_RJUST0:
        break;

      case FMT_LJUST:
        {
          int width;

          if (IS_SIGNED(flags))
            {
              valwidth++;
            }

          width = valwidth < trunc ? trunc : valwidth;

          for (i = fieldwidth - width; i > 0; i--)
            {
              obj->put(obj, ' ');
            }
        }
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_vsprintf
 *
 * Description:
 *  Stream-oriented implementation that underlies printf family:  printf,
 *  fprint, sprint, etc.
 *
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *obj, FAR const IPTR char *src,
                 va_list ap)
{
  FAR char        *ptmp;
  uint16_t        flags;
  uint8_t         justify;
#ifdef CONFIG_ARCH_ROMGETC
  char            ch;
#endif
  int             width;
  int             trunc;

  for (FMT_TOP; FMT_CHAR; FMT_BOTTOM)
    {
      /* Just copy regular characters */

      if (FMT_CHAR != '%')
        {
          /* Output the character */

          obj->put(obj, FMT_CHAR);

          /* Flush the buffer if a newline is encountered */

          if (FMT_CHAR == '\n')
            {
              /* Should return an error on a failure to flush */

              obj->flush(obj);
            }

          /* Process the next character in the format */

          continue;
        }

      /* We have found a format specifier. Move past it. */

      FMT_NEXT;

      /* Assume defaults */

      flags   = 0;
      justify = FMT_RJUST;
      width   = 0;
      trunc   = 0;

      /* Process each format qualifier. */

      for (; FMT_CHAR; FMT_BOTTOM)
        {
          /* Break out of the loop when the format is known. */

          if (strchr("diuxXpobeEfgGlLsc%", FMT_CHAR))
            {
              break;
            }

          /* Check for left justification. */

          else if (FMT_CHAR == '-')
            {
              justify = FMT_LJUST;
            }

          /* Check for leading zero fill right justification. */

          else if (FMT_CHAR == '0')
            {
              justify = FMT_RJUST0;
            }

#if 0
          /* Center justification. */

          else if (FMT_CHAR == '~')
            {
              justify = FMT_CENTER;
            }
#endif

          else if (FMT_CHAR == '*')
            {
              int value = va_arg(ap, int);
              if (IS_HASDOT(flags))
                {
                  trunc = value;
                  SET_HASASTERISKTRUNC(flags);
                }
              else
                {
                  width = value;
                  SET_HASASTERISKWIDTH(flags);
                }
            }

          /* Check for field width */

          else if (FMT_CHAR >= '1' && FMT_CHAR <= '9')
            {
              /* Accumulate the field width integer. */

              int n = ((int)(FMT_CHAR)) - (int)'0';
              for (; ; )
                {
                  FMT_NEXT;
                  if (FMT_CHAR >= '0' && FMT_CHAR <= '9')
                    {
                      n = 10*n + (((int)(FMT_CHAR)) - (int)'0');
                    }
                  else
                    {
                      break;
                    }
                }

              if (IS_HASDOT(flags))
                {
                  trunc = n;
                }
              else
                {
                  width = n;
                }

              /* Back up to the last digit. */

              FMT_PREV;
            }

          /* Check for a decimal point. */

          else if (FMT_CHAR == '.')
            {
              SET_HASDOT(flags);
            }

          /* Check for leading plus sign. */

          else if (FMT_CHAR == '+')
            {
              SET_SHOWPLUS(flags);
            }

          /* Check for alternate form. */

          else if (FMT_CHAR == '#')
            {
              SET_ALTFORM(flags);
            }
        }

      /* "%%" means that a literal '%' was intended (instead of a format
       * specification).
       */

      if (FMT_CHAR == '%')
        {
          obj->put(obj, '%');
          continue;
        }

      /* Check for the string format. */

      if (FMT_CHAR == 's')
        {
          int swidth;
          int left;

          /* Get the string to output */

          ptmp = va_arg(ap, FAR char *);
          if (!ptmp)
            {
              ptmp = (FAR char *)g_nullstring;
            }

          /* Get the width of the string and perform right-justification
           * operations.
           */

          swidth = (IS_HASDOT(flags) && trunc >= 0)
                      ? strnlen(ptmp, trunc) : strlen(ptmp);
          prejustify(obj, FMT_CHAR, justify, 0, width, swidth, 0);
          left = swidth;

          /* Concatenate the string into the output */

          while (*ptmp)
            {
              if (left-- <= 0)
                {
                  break;
                }

              obj->put(obj, *ptmp);
              ptmp++;
            }

          /* Perform left-justification operations. */

          postjustify(obj, justify, 0, width, swidth, 0);
          continue;
        }

      /* Check for the character output */

      else if (FMT_CHAR == 'c')
        {
          /* Just copy the character into the output. */

          int n = va_arg(ap, int);
          obj->put(obj, n);
          continue;
        }

      /* Check for the long long prefix. */

      if (FMT_CHAR == 'L')
        {
           SET_LONGLONGPRECISION(flags);
           FMT_NEXT;
        }
      else if (FMT_CHAR == 'l')
        {
          SET_LONGPRECISION(flags);
          FMT_NEXT;
          if (FMT_CHAR == 'l')
            {
              SET_LONGLONGPRECISION(flags);
              FMT_NEXT;
            }
        }

      /* Handle integer conversions */

      if (strchr("diuxXpob", FMT_CHAR))
        {
#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_LIBC_LONG_LONG)
          if (IS_LONGLONGPRECISION(flags) && FMT_CHAR != 'p')
            {
              long long lln;
              int lluwidth;

              /* Extract the long long value. */

              lln = va_arg(ap, long long);

              /* Resolve sign-ness and format issues */

              llfixup(FMT_CHAR, &flags, &lln);

              /* Get the width of the output */

              lluwidth = getllusize(FMT_CHAR, flags, lln);

              /* Perform left field justification actions */

              prejustify(obj, FMT_CHAR, justify, flags, width, lluwidth,
                         trunc);

              /* Output the number */

              llutoascii(obj, FMT_CHAR, flags, (unsigned long long)lln);

              /* Perform right field justification actions */

              postjustify(obj, justify, flags, width, lluwidth, trunc);
            }
          else
#endif /* CONFIG_HAVE_LONG_LONG */
#ifdef CONFIG_LONG_IS_NOT_INT
          if (IS_LONGPRECISION(flags) && FMT_CHAR != 'p')
            {
              long ln;
              int luwidth;

              /* Extract the long value. */

              ln = va_arg(ap, long);

              /* Resolve sign-ness and format issues */

              lfixup(FMT_CHAR, &flags, &ln);

              /* Get the width of the output */

              luwidth = getlusize(FMT_CHAR, flags, ln);

              /* Perform left field justification actions */

              prejustify(obj, FMT_CHAR, justify, flags, width, trunc);

              /* Output the number */

              lutoascii(obj, FMT_CHAR, flags, (unsigned long)ln);

              /* Perform right field justification actions */

              postjustify(obj, justify, flags, width, luwidth, trunc);
            }
          else
#endif /* CONFIG_LONG_IS_NOT_INT */
#ifdef CONFIG_PTR_IS_NOT_INT
          if (FMT_CHAR == 'p')
            {
              void *p;
              int pwidth;

              /* Extract the integer value. */

              p = va_arg(ap, void *);

              /* Resolve sign-ness and format issues */

              lfixup(FMT_CHAR, &flags, &ln);

              /* Get the width of the output */

              pwidth = getpsize(FMT_CHAR, flags, p);

              /* Perform left field justification actions */

              prejustify(obj, FMT_CHAR, justify, flags, width, pwidth, 0);

              /* Output the pointer value */

              ptohex(obj, flags, p);

              /* Perform right field justification actions */

              postjustify(obj, justify, flags, width, pwidth, 0);
            }
          else
#endif
            {
              int uwidth;
              int n;

              /* Extract the long long value. */

              n = va_arg(ap, int);

              /* Resolve sign-ness and format issues */

              fixup(FMT_CHAR, &flags, &n);

              /* Get the width of the output */

              uwidth = getusize(FMT_CHAR, flags, n);

              /* Perform left field justification actions */

              prejustify(obj, FMT_CHAR, justify, flags, width, uwidth,
                         trunc);

              /* Output the number */

              utoascii(obj, FMT_CHAR, flags, (unsigned int)n);

              /* Perform right field justification actions */

              postjustify(obj, justify, flags, width, uwidth, trunc);
            }
        }

      /* Handle floating point conversions */

#ifdef CONFIG_LIBC_FLOATINGPOINT
      else if (strchr("eEfgG", FMT_CHAR))
        {
          double dblval = va_arg(ap, double);
          int dblsize;

          if (FMT_CHAR == 'g' || FMT_CHAR == 'G')
            {
              flags |= FLAG_NOTRAILINGZERO;
            }

          /* Get the width of the output */

          dblsize = getdblsize(FMT_CHAR, trunc, flags, dblval);

          /* Perform left field justification actions */

          prejustify(obj, FMT_CHAR, justify, 0, width, dblsize, 0);

          /* Output the number */

          lib_dtoa(obj, FMT_CHAR, trunc, flags, dblval);

          /* Perform right field justification actions */

          postjustify(obj, justify, 0, width, dblsize, 0);
        }
#endif /* CONFIG_LIBC_FLOATINGPOINT */
    }

  return obj->nput;
}
