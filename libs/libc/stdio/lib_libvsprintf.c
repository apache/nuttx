/****************************************************************************
 * libs/libc/stdio/lib_libvsprintf.c
 *
 *   Copyright (C) 2007-2012, 2018 Gregory Nutt. All rights reserved.
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

#define FLAG_SHOWPLUS            0x01
#define FLAG_ALTFORM             0x02
#define FLAG_HASDOT              0x04
#define FLAG_HASASTERISKWIDTH    0x08
#define FLAG_HASASTERISKTRUNC    0x10
#define FLAG_LONGPRECISION       0x20
#define FLAG_LONGLONGPRECISION   0x40
#define FLAG_NEGATE              0x80

#define SET_SHOWPLUS(f)          do (f) |= FLAG_SHOWPLUS; while (0)
#define SET_ALTFORM(f)           do (f) |= FLAG_ALTFORM; while (0)
#define SET_HASDOT(f)            do (f) |= FLAG_HASDOT; while (0)
#define SET_HASASTERISKWIDTH(f)  do (f) |= FLAG_HASASTERISKWIDTH; while (0)
#define SET_HASASTERISKTRUNC(f)  do (f) |= FLAG_HASASTERISKTRUNC; while (0)
#define SET_LONGPRECISION(f)     do (f) |= FLAG_LONGPRECISION; while (0)
#define SET_LONGLONGPRECISION(f) do (f) |= FLAG_LONGLONGPRECISION; while (0)
#define SET_NEGATE(f)            do (f) |= FLAG_NEGATE; while (0)

#define CLR_SHOWPLUS(f)          do (f) &= ~FLAG_SHOWPLUS; while (0)
#define CLR_ALTFORM(f)           do (f) &= ~FLAG_ALTFORM; while (0)
#define CLR_HASDOT(f)            do (f) &= ~FLAG_HASDOT; while (0)
#define CLR_HASASTERISKWIDTH(f)  do (f) &= ~FLAG_HASASTERISKWIDTH; while (0)
#define CLR_HASASTERISKTRUNC(f)  do (f) &= ~FLAG_HASASTERISKTRUNC; while (0)
#define CLR_LONGPRECISION(f)     do (f) &= ~FLAG_LONGPRECISION; while (0)
#define CLR_LONGLONGPRECISION(f) do (f) &= ~FLAG_LONGLONGPRECISION; while (0)
#define CLR_NEGATE(f)            do (f) &= ~FLAG_NEGATE; while (0)
#define CLR_SIGNED(f)            do (f) &= ~(FLAG_SHOWPLUS|FLAG_NEGATE); while (0)

#define IS_SHOWPLUS(f)           (((f) & FLAG_SHOWPLUS) != 0)
#define IS_ALTFORM(f)            (((f) & FLAG_ALTFORM) != 0)
#define IS_HASDOT(f)             (((f) & FLAG_HASDOT) != 0)
#define IS_HASASTERISKWIDTH(f)   (((f) & FLAG_HASASTERISKWIDTH) != 0)
#define IS_HASASTERISKTRUNC(f)   (((f) & FLAG_HASASTERISKTRUNC) != 0)
#define IS_LONGPRECISION(f)      (((f) & FLAG_LONGPRECISION) != 0)
#define IS_LONGLONGPRECISION(f)  (((f) & FLAG_LONGLONGPRECISION) != 0)
#define IS_NEGATE(f)             (((f) & FLAG_NEGATE) != 0)
#define IS_SIGNED(f)             (((f) & (FLAG_SHOWPLUS|FLAG_NEGATE)) != 0)

/* If CONFIG_ARCH_ROMGETC is defined, then it is assumed that the format
 * string data cannot be accessed by simply de-referencing the format string
 * pointer.  This might be in the case in Harvard architectures where string
 * data might be stored in instruction space or if string data were stored
 * on some media like EEPROM or external serial FLASH.  In all of these cases,
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
static void ptohex(FAR struct lib_outstream_s *obj, uint8_t flags,
                   FAR void *p);
static int  getsizesize(uint8_t fmt, uint8_t flags, FAR void *p)
#endif /* CONFIG_PTR_IS_NOT_INT */

/* Unsigned int to ASCII conversion */

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n,
                   int width);
static void utohex(FAR struct lib_outstream_s *obj, unsigned int n,
                   uint8_t a, int width);
static void utooct(FAR struct lib_outstream_s *obj, unsigned int n,
                   int width);
static void utobin(FAR struct lib_outstream_s *obj, unsigned int n,
                   int width);
static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                     uint8_t flags, unsigned int lln, int uwidth);

static void fixup(uint8_t fmt, FAR uint8_t *flags, int *n);
static int  getusize(uint8_t fmt, uint8_t flags, unsigned int lln);

/* Unsigned long int to ASCII conversion */

#ifdef CONFIG_LONG_IS_NOT_INT
static void lutodec(FAR struct lib_outstream_s *obj, unsigned long ln,
                    int width);
static void lutohex(FAR struct lib_outstream_s *obj, unsigned long ln,
                    uint8_t a, int width);
static void lutooct(FAR struct lib_outstream_s *obj, unsigned long ln,
                    int width);
static void lutobin(FAR struct lib_outstream_s *obj, unsigned long ln,
                    int width);
static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                      uint8_t flags, unsigned long ln, int width);

static void lfixup(uint8_t fmt, FAR uint8_t *flags, long *ln);
static int  getlusize(uint8_t fmt, FAR uint8_t flags, unsigned long ln);
#endif

/* Unsigned long long int to ASCII conversions */

#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_LIBC_LONG_LONG)
static void llutodec(FAR struct lib_outstream_s *obj, unsigned long long lln,
                     int width);
static void llutohex(FAR struct lib_outstream_s *obj, unsigned long long lln,
                     uint8_t a, int width);
static void llutooct(FAR struct lib_outstream_s *obj, unsigned long long lln,
                     int width);
static void llutobin(FAR struct lib_outstream_s *obj, unsigned long long lln,
                     int width);
static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, unsigned long long lln, int width);

static void llfixup(uint8_t fmt, FAR uint8_t *flags, FAR long long *lln);
static int  getllusize(uint8_t fmt, FAR uint8_t flags,
                       FAR unsigned long long lln);
#endif

static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, int fieldwidth, int valwidth);
static void postjustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                        uint8_t flags, int fieldwidth, int valwidth);

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
static void ptohex(FAR struct lib_outstream_s *obj, uint8_t flags,
                   FAR void *p)
{
  union
  {
    uint32_t  dw;
    FAR void *p;
  } u;
  uint8_t bits;

  /* Check for alternate form */

  if (IS_ALTFORM(flags))
    {
      /* Prefix the number with "0x" */

      obj->put(obj, '0');
      obj->put(obj, 'x');
    }

  u.dw = 0;
  u.p  = p;

  for (bits = 8*sizeof(void *); bits > 0; bits -= 4)
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

static int getpsize(uint8_t flags, FAR void *p)
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

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n,
                   int width)
{
  if (width > 0)
    {
      unsigned int remainder = n % 10;
      unsigned int dividend  = n / 10;

      if (dividend != 0)
        {
          utodec(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: utohex
 ****************************************************************************/

static void utohex(FAR struct lib_outstream_s *obj, unsigned int n,
                   uint8_t a, int width)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8 * sizeof(unsigned int);
       width > 0 && bits > 0;
       width--, bits -= 4)
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

  if (width > 0 && !nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: utooct
 ****************************************************************************/

static void utooct(FAR struct lib_outstream_s *obj, unsigned int n, int width)
{
  if (width > 0)
    {
      unsigned int remainder = n & 0x7;
      unsigned int dividend = n >> 3;

      if (dividend != 0)
        {
          utooct(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: utobin
 ****************************************************************************/

static void utobin(FAR struct lib_outstream_s *obj, unsigned int n, int width)
{
  if (width > 0)
    {
      unsigned int remainder = n & 1;
      unsigned int dividend  = n >> 1;

      if (dividend != 0)
        {
          utobin(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: utoascii
 ****************************************************************************/

static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                     uint8_t flags, unsigned int n, int width)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
      case 'u':
        /* Signed/unsigned base 10 */
        {
          /* Convert the integer value to a string. */

          utodec(obj, n, width);
        }
        break;

#ifndef CONFIG_PTR_IS_NOT_INT
      case 'p':
#endif
      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              if (width-- > 0)
                {
                  obj->put(obj, '0');
                }

              if (width-- > 0)
                {
                  obj->put(obj, 'x');
                }
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              utohex(obj, n, 'A', width);
            }
          else
            {
              utohex(obj, n, 'a', width);
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

              if (width-- > 0)
                {
                   obj->put(obj, '0');
                }
             }

           /* Convert the unsigned value to a string. */

           utooct(obj, n, width);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          utobin(obj, n, width);
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

static void fixup(uint8_t fmt, FAR uint8_t *flags, FAR int *n)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*n < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *n    = -*n;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getusize
 ****************************************************************************/

static int getusize(uint8_t fmt, uint8_t flags, unsigned int n)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  utoascii(&nulloutstream, fmt, flags, n, INT_MAX);
  return nulloutstream.nput;
}

/****************************************************************************
 * Name: getdblsize
 ****************************************************************************/

#ifdef CONFIG_LIBC_FLOATINGPOINT
static int getdblsize(uint8_t fmt, int trunc, uint8_t flags, double n)
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

static void lutodec(FAR struct lib_outstream_s *obj, unsigned long n, int width)
{
  if (width > 0)
    {
      unsigned int  remainder = n % 10;
      unsigned long dividend  = n / 10;

      if (dividend != 0)
        {
          lutodec(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: lutohex
 ****************************************************************************/

static void lutohex(FAR struct lib_outstream_s *obj, unsigned long n,
                    uint8_t a, int width)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8 * sizeof(unsigned long);
       width > 0 && bits > 0;
       width--, bits -= 4)
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

  if (width > 0 && !nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: lutooct
 ****************************************************************************/

static void lutooct(FAR struct lib_outstream_s *obj, unsigned long n,
                    int width)
{
  if (width > 0)
    {
      unsigned int  remainder = n & 0x7;
      unsigned long dividend  = n >> 3;

      if (dividend != 0)
        {
          lutooct(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: lutobin
 ****************************************************************************/

static void lutobin(FAR struct lib_outstream_s *obj, unsigned long n,
                    int width)
{
  if (width > 0)
    {
      unsigned int  remainder = n & 1;
      unsigned long dividend  = n >> 1;

      if (dividend != 0)
        {
          lutobin(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: lutoascii
 ****************************************************************************/

static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                      uint8_t flags, unsigned long ln, int width)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
      case 'u':
        /* Signed/unsigned base 10 */
        {
          /* Convert the long integer value to a string. */

          lutodec(obj, ln, width);
        }
        break;

      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              if (width-- > 0)
                {
                  obj->put(obj, '0');
                }

              if (width-- > 0)
                {
                  obj->put(obj, 'x');
                }
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              lutohex(obj, ln, 'A', width);
            }
          else
            {
              lutohex(obj, ln, 'a', width);
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

              if (width-- > 0)
                {
                   obj->put(obj, '0');
                }
             }

           /* Convert the unsigned value to a string. */

           lutooct(obj, ln, width);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          lutobin(obj, ln, width);
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

static void lfixup(uint8_t fmt, FAR uint8_t *flags, FAR long *ln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*ln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *ln    = -*ln;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getlusize
 ****************************************************************************/

static int getlusize(uint8_t fmt, uint8_t flags, unsigned long ln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  lutoascii(&nulloutstream, fmt, flags, ln, INT_MAX);
  return nulloutstream.nput;
}

#endif /* CONFIG_LONG_IS_NOT_INT */

#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_LIBC_LONG_LONG)
/****************************************************************************
 * Name: llutodec
 ****************************************************************************/

static void llutodec(FAR struct lib_outstream_s *obj, unsigned long long n,
                     int width)
{
  if (width > 0)
    {
      unsigned int remainder = n % 10;
      unsigned long long dividend = n / 10;

      if (dividend != 0)
        {
          llutodec(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: llutohex
 ****************************************************************************/

static void llutohex(FAR struct lib_outstream_s *obj, unsigned long long n,
                     uint8_t a, int width)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8 * sizeof(unsigned long long);
       width > 0 && bits > 0;
       width--, bits -= 4)
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

  if (width > 0 && !nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: llutooct
 ****************************************************************************/

static void llutooct(FAR struct lib_outstream_s *obj, unsigned long long n,
                     int width)
{
  if (width > 0)
    {
      unsigned int remainder = n & 0x7;
      unsigned long long dividend = n >> 3;

      if (dividend != 0)
        {
          llutooct(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: llutobin
 ****************************************************************************/

static void llutobin(FAR struct lib_outstream_s *obj, unsigned long long n,
                     int width)
{
  if (width > 0)
    {
      unsigned int remainder = n & 1;
      unsigned long long dividend = n >> 1;

      if (dividend != 0)
        {
          llutobin(obj, dividend, width - 1);
        }

      obj->put(obj, (remainder + (unsigned int)'0'));
    }
}

/****************************************************************************
 * Name: llutoascii
 ****************************************************************************/

static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, unsigned long long lln, int width)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
      case 'u':
        /* Signed/unsigned base 10 */
        {
          /* Convert the long long integer value to a string. */

          llutodec(obj, (unsigned long long)lln, width);
        }
        break;

      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              if (width-- > 0)
                {
                  obj->put(obj, '0');
                }

              if (width-- > 0)
                {
                  obj->put(obj, 'x');
                }
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              llutohex(obj, (unsigned long long)lln, 'A', width);
            }
          else
            {
              llutohex(obj, (unsigned long long)lln, 'a', width);
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

              if (width-- > 0)
                {
                  obj->put(obj, '0');
                }
             }

           /* Convert the unsigned value to a string. */

           llutooct(obj, (unsigned long long)lln, width);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          llutobin(obj, (unsigned long long)lln, width);
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

static void llfixup(uint8_t fmt, FAR uint8_t *flags, FAR long long *lln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*lln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *lln    = -*lln;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getllusize
 ****************************************************************************/

static int getllusize(uint8_t fmt, uint8_t flags, unsigned long long lln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);


  llutoascii(&nulloutstream, fmt, flags, lln, INT_MAX);
  return nulloutstream.nput;
}

#endif /* CONFIG_HAVE_LONG_LONG */

/****************************************************************************
 * Name: prejustify
 ****************************************************************************/

static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, int fieldwidth, int valwidth)
{
  int i;

  switch (fmt)
    {
      default:
      case FMT_RJUST:
        if (IS_SIGNED(flags))
          {
            valwidth++;
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
        break;

      case FMT_RJUST0:
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

        for (i = fieldwidth - valwidth; i > 0; i--)
          {
            obj->put(obj, '0');
          }
        break;

      case FMT_LJUST:
         if (IS_NEGATE(flags))
          {
            obj->put(obj, '-');
          }
        else if (IS_SHOWPLUS(flags))
          {
            obj->put(obj, '+');
          }
        break;
    }
}

/****************************************************************************
 * Name: postjustify
 ****************************************************************************/

static void postjustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                        uint8_t flags, int fieldwidth, int valwidth)
{
  int i;

  /* Apply field justification to the integer value. */

  switch (fmt)
    {
      default:
      case FMT_RJUST:
      case FMT_RJUST0:
        break;

      case FMT_LJUST:
        if (IS_SIGNED(flags))
          {
            valwidth++;
          }

        for (i = fieldwidth - valwidth; i > 0; i--)
          {
            obj->put(obj, ' ');
          }
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * libs/libc/stdio/lib_vsprintf
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *obj, FAR const IPTR char *src,
                 va_list ap)
{
  FAR char        *ptmp;
  int             width;
  int             trunc;
  uint8_t         fmt;
  uint8_t         flags;
#ifdef CONFIG_ARCH_ROMGETC
  char            ch;
#endif

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

               (void)obj->flush(obj);
             }

           /* Process the next character in the format */

           continue;
        }

      /* We have found a format specifier. Move past it. */

      FMT_NEXT;

      /* Assume defaults */

      flags = 0;
      fmt   = FMT_RJUST;
      width = 0;
      trunc = 0;

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
              fmt = FMT_LJUST;
            }

          /* Check for leading zero fill right justification. */

          else if (FMT_CHAR == '0')
            {
              fmt = FMT_RJUST0;
            }
#if 0
          /* Center justification. */

          else if (FMT_CHAR == '~')
            {
              fmt = FMT_CENTER;
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
          prejustify(obj, fmt, 0, width, swidth);
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

          postjustify(obj, fmt, 0, width, swidth);
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
              if (trunc > 0 && lluwidth > trunc)
                {
                  lluwidth = trunc;
                }

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, lluwidth);

              /* Output the number */

              llutoascii(obj, FMT_CHAR, flags, (unsigned long long)lln,
                         lluwidth);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, lluwidth);
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
              if (trunc > 0 && luwidth > trunc)
                {
                  luwidth = trunc;
                }

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, luwidth);

              /* Output the number */

              lutoascii(obj, FMT_CHAR, flags, (unsigned long)ln, luwidth);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, luwidth);
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

              prejustify(obj, fmt, flags, width, pwidth);

              /* Output the pointer value */

              ptohex(obj, flags, p);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, pwidth);
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
              if (trunc > 0 &&  uwidth > trunc)
                {
                  uwidth = trunc;
                }

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, uwidth);

              /* Output the number */

              utoascii(obj, FMT_CHAR, flags, (unsigned int)n, uwidth);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, uwidth);
            }
        }

      /* Handle floating point conversions */

#ifdef CONFIG_LIBC_FLOATINGPOINT
      else if (strchr("eEfgG", FMT_CHAR))
        {
          double dblval = va_arg(ap, double);
          int dblsize;

          /* Get the width of the output */

          dblsize = getdblsize(FMT_CHAR, trunc, flags, dblval);

          /* Perform left field justification actions */

          prejustify(obj, fmt, 0, width, dblsize);

          /* Output the number */

          lib_dtoa(obj, FMT_CHAR, trunc, flags, dblval);

          /* Perform right field justification actions */

          postjustify(obj, fmt, 0, width, dblsize);
        }
#endif /* CONFIG_LIBC_FLOATINGPOINT */
    }

  return obj->nput;
}
