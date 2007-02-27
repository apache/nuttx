/************************************************************
 * lib_libvsprintf.c
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

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>

#include "lib_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

enum
{
  FMT_RJUST = 0, /* Default */
  FMT_LJUST,
  FMT_RJUST0,
  FMT_CENTER
};

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static void utodec(char **pp, unsigned int n);
static void _utodec(char **pp, unsigned int n);
static void utohex(char **pp, unsigned int n);
static void _utohex(char **pp, unsigned int n);
static void utooct(char **pp, unsigned int n);
static void _utooct(char **pp, unsigned int n);
static void utobin(char **pp, unsigned int n);
static void _utobin(char **pp, unsigned int n);

#ifdef CONFIG_HAVE_LONG_LONG
static void llutodec(char **pp, unsigned long long n);
static void _llutodec(char **pp, unsigned long long n);
static void llutohex(char **pp, unsigned long long n);
static void _llutohex(char **pp, unsigned long long n);
static void llutooct(char **pp, unsigned long long n);
static void _llutooct(char **pp, unsigned long long n);
static void llutobin(char **pp, unsigned long long n);
static void _llutobin(char **pp, unsigned long long n);
#endif

/************************************************************
 * Global Constant Data
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Constant Data
 ************************************************************/

static const char g_nullstring[] = "(null)";

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

static void utodec(char **pp, unsigned int n)
{
  _utodec(pp, n);
  **pp = 0;
}

static void _utodec(char **pp, unsigned int n)
{
  unsigned int remainder = n % 10;
  unsigned int dividend = n / 10;

  if (dividend)
    {
      _utodec(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}

static void utohex(char **pp, unsigned int n)
{
  _utohex(pp, n);
  **pp = 0;
}

static void _utohex(char **pp, unsigned int n)
{
  unsigned int remainder = n & 0xf;
  unsigned int dividend = n >> 4;

  if (dividend)
    {
      _utohex(pp, dividend);
    }

  if (remainder < 10)
    {
      **pp = (char)(remainder + (unsigned int)'0');
    }
  else
    {
      **pp = (char)(remainder + ((unsigned int)'a' - 10));
    }
  (*pp)++;
}

static void utooct(char **pp, unsigned int n)
{
  _utooct(pp, n);
  **pp = 0;
}

static void _utooct(char **pp, unsigned int n)
{
  unsigned int remainder = n & 0x7;
  unsigned int dividend = n >> 3;

  if (dividend)
    {
      _utooct(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}

static void utobin(char **pp, unsigned int n)
{
  _utobin(pp, n);
  **pp = 0;
}

static void _utobin(char **pp, unsigned int n)
{
  unsigned int remainder = n & 1;
  unsigned int dividend = n >> 1;

  if (dividend)
    {
      _utobin(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}

#ifdef CONFIG_HAVE_LONG_LONG
static void llutodec(char **pp, unsigned long long n)
{
  _llutodec(pp, n);
  **pp = 0;
}

static void _llutodec(char **pp, unsigned long long n)
{
  unsigned int remainder = n % 10;
  unsigned long long dividend = n / 10;

  if (dividend)
    {
      _llutodec(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}

static void llutohex(char **pp, unsigned long long n)
{
  _llutohex(pp, n);
  **pp = 0;
}

static void _llutohex(char **pp, unsigned long long n)
{
  unsigned int remainder = n & 0xf;
  unsigned long long dividend = n >> 4;
  if (dividend)
    {
      _llutohex(pp, dividend);
    }

  if (remainder < 10)
    {
      **pp = (char)(remainder + (unsigned int)'0');
    }
  else
    {
      **pp = (char)(remainder + ((unsigned int)'a' - 10));
    }
  (*pp)++;
}

static void llutooct(char **pp, unsigned long long n)
{
  _llutooct(pp, n);
  **pp = 0;
}

static void _llutooct(char **pp, unsigned long long n)
{
  unsigned int remainder = n & 0x7;
  unsigned long long dividend = n >> 3;

  if (dividend)
    {
      _llutooct(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}

static void llutobin(char **pp, unsigned long long n)
{
  _llutobin(pp, n);
  **pp = 0;
}

static void _llutobin(char **pp, unsigned long long n)
{
  unsigned int remainder = n & 1;
  unsigned long long dividend = n >> 1;

  if (dividend)
    {
      _llutobin(pp, dividend);
    }

  **pp = (char)(remainder + (unsigned int)'0');
  (*pp)++;
}
#endif /* CONFIG_HAVE_LONG_LONG */

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * lib_vsprintf
 ************************************************************/

int lib_vsprintf(struct lib_stream_s *obj, const char *src, va_list ap)
{
  char           *ptmp;
  char            tmp[40];
  const char     *pfmt;
  unsigned int    n;
  int             fmt, width, trunc, tmpwidth;
  int             showplus, altform, longlong;
  int             hasdot, hasasteriskwidth, hasasterisktrunc;

  for (; *src; src++)
    {
      /* Just copy regular characters */

      if (*src != '%')
        {
           obj->put(obj, *src);
        }
      else
        {
          /* We have found a format specifier. Move past it. */

          pfmt = src;
          src++;

          fmt = FMT_RJUST; /* Assume right justification. */
          width = trunc = 0;
          showplus = altform = longlong = 0;
          hasdot = hasasteriskwidth = hasasterisktrunc = 0;

          /* Process each format qualifier. */

          for (; *src; src++)
            {
              /* Break out of the loop when the format is known. */

              if (strchr("diuxXpobeEfgGlLsc%", *src))
                {
                  break;
                }

              /* Check for left justification. */

              else if (*src == '-')
                {
                  fmt = FMT_LJUST;
                }

              /* Check for leading zero fill right justification. */

              else if (*src == '0')
                {
                  fmt = FMT_RJUST0;
                }
#if 0
              /* Center justification. */

              else if (*src == '~')
                {
                  fmt = FMT_CENTER;
                }
#endif

              else if (*src == '*')
                {
                  int value = va_arg(ap, int);
                  if (hasdot)
                    {
                      trunc = value;
                      hasasterisktrunc = 1;
                    }
                  else
                    {
                      width = value;
                      hasasteriskwidth = 1;
                    }
                }

                  /* Check for field width */

              else if (((*src) >= '1') && ((*src) <= '9'))
                {
                  /* Accumulate the field width integer. */

                  n = ((int)(*src)) - (int)'0';
                  for (src++; (((*src) >= '0') && ((*src) <= '9')); src++)
                    {
                      n = 10*n + (((int)(*src)) - (int)'0');
                    }

                  if (hasdot) trunc = n;
                  else width = n;

                  /* Back up to the last digit. */

                  src--;
                }

              /* Check for a decimal point. */

              else if (*src == '.')
                {
                  hasdot = 1;
                }

              /* Check for leading plus sign. */

              else if (*src == '+')
                {
                  showplus = 1;
                }

              /* Check for alternate form. */

              else if (*src == '#')
                {
                  altform = 1;
                }
            }

          /* "%%" means that a literal '%' was intended (instead of a format
           * specification).
           */

          if (*src == '%')
            {
              obj->put(obj, '%');
            }

          /* Check for the string format. */

          else if (*src == 's')
            {
              /* Just concatenate the string into the output */

              ptmp = va_arg(ap, char *);
              if (!ptmp) ptmp = (char*)g_nullstring;

              while(*ptmp)
                {
                  obj->put(obj, *ptmp);
                  ptmp++;
                }
            }

          /* Check for the character output */

          else if (*src == 'c')
            {
              /* Just copy the character into the output. */

              n = va_arg(ap, int);
              obj->put(obj, n);
            }
          else
            {
              /* Check for the long long prefix. */

              if (*src == 'L')
                {
                  longlong = 1;
                  ++src;
                }
              else if (*src == 'l')
                {
                  if (*++src == 'l')
                    {
                      longlong = 1;
                      ++src;
                    }
                }

              /* Get the ascii format into the tmp[] buffer. */

              ptmp = tmp;

             /* Handle integer conversions */

             if (strchr("diuxXpob", *src))
               {
#ifdef CONFIG_HAVE_LONG_LONG
                 if ((longlong) && (*src != 'p'))
                   {
                     /* Extract the long long value. */

                     long long lln = va_arg(ap, long long);

                     /* Perform the integer conversion according to the
                      * format specifier
                      */

                     switch (*src)
                       {
                       case 'd':
                       case 'i':
                         /* Signed base 10 */
                         {
                           /* Check for leading +/- */

                           if (lln < 0)
                             {
                               *ptmp++ = '-';
                               *ptmp   = 0;
                                lln    = -lln;
                             }
                           else if (showplus)
                             {
                               *ptmp++ = '+';
                               *ptmp   = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           llutodec(&ptmp, (unsigned long long)lln);
                         }
                         break;

                       case 'u':
                         /* Unigned base 10 */
                         {
                           /* Check for leading + */

                           if (showplus)
                             {
                               *ptmp++ = '+';
                               *ptmp = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           llutodec(&ptmp, (unsigned long long)lln);
                         }
                         break;

                       case 'p':
                       case 'x':
                       case 'X':
                         /* Hexadecimal */
                         {
                           /* Check for alternate form */

                           if (altform)
                             {
                               /* Prefix the number with "0x" */

                               *ptmp++ = '0';
                               *ptmp++ = 'x';
                               *ptmp   = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           llutohex(&ptmp, (unsigned long long)lln);

                           /* Check for upper case conversion. */

                           if ((*src) == 'X')
                             {
                               for (ptmp = tmp; *ptmp; ptmp++)
                                 {
                                   if (((*ptmp) >= 'a') && ((*ptmp) <= 'z'))
                                     {
                                       /* Convert to upper case. */

                                       *ptmp += (char)((int)'A' - (int)'a');
                                     }
                                 }
                             }
                         }
                         break;

                       case 'o':
                         /* Octal */
                           {
                             /* Check for alternate form */

                             if (altform)
                               {
                                 /* Prefix the number with '0' */

                                 *ptmp++ = '0';
                                 *ptmp   = 0;
                               }

                             /* Convert the unsigned value to a string. */

                             llutooct(&ptmp, (unsigned long long)lln);
                           }
                           break;

                       case 'b':
                         /* Binary */
                         {
                           /* Convert the unsigned value to a string. */

                           llutobin(&ptmp, (unsigned long long)lln);
                         }
                         break;

                       default:
                         break;
                       }
                   }
                 else
                   {
#endif /* CONFIG_HAVE_LONG_LONG */
                     /* Extract the integer value */

                     n = va_arg(ap, int);

                     /* Perform the integer conversion according to the
                      * format specifier
                      */

                     switch (*src)
                       {
                      case 'd':
                       case 'i':
                         /* Signed base 10 */
                         {
                           /* Check for leading +/- */

                           if ((int)n < 0)
                             {
                               *ptmp++ = '-';
                               *ptmp   = 0;
                                n      = -n;
                             }
                           else if (showplus)
                             {
                               *ptmp++ = '+';
                               *ptmp   = 0;
                             }

                        /* Convert the unsigned value to a string. */

                        utodec(&ptmp, (unsigned int)n);
                      }
                         break;

                       case 'u':
                         /* Unigned base 10 */
                         {
                           /* Check for leading + */

                           if (showplus)
                             {
                               *ptmp++ = '+';
                               *ptmp   = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           utodec(&ptmp, (unsigned int)n);
                         }
                         break;

                       case 'x':
                       case 'X':
                       case 'p':
                         /* Hexadecimal */
                         {
                           /* Check for alternate form */

                           if (altform)
                             {
                               /* Prefix the number with "0x" */

                               *ptmp++ = '0';
                               *ptmp++ = 'x';
                               *ptmp   = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           utohex(&ptmp, (unsigned int)n);

                           /* Check for upper case conversion. */

                           if ((*src) == 'X')
                             {
                               for (ptmp = tmp; *ptmp; ptmp++)
                                 {
                                   if (((*ptmp) >= 'a') && ((*ptmp) <= 'z'))
                                     {
                                       /* Convert to upper case. */

                                       *ptmp += (char)((int)'A' - (int)'a');
                                     }
                                 }
                             }
                           else if ((*src) == 'p')
                             {
                               if ((!width)  && (fmt == FMT_RJUST))
                                 {
                                   if (altform) width = 10;
                                   else width = 8;

                                   fmt = FMT_RJUST0;
                                 }
                              }
                         }
                         break;

                       case 'o':
                         /* Octal */
                         {
                           /* Check for alternate form */

                           if (altform)
                             {
                               /* Prefix the number with '0' */

                               *ptmp++ = '0';
                               *ptmp   = 0;
                             }

                           /* Convert the unsigned value to a string. */

                           utooct(&ptmp, (unsigned int)n);
                         }
                         break;

                       case 'b':
                         /* Binary */
                         {
                           /* Convert the unsigned value to a string. */

                           utobin(&ptmp, (unsigned int)n);
                         }
                         break;

                       default:
                         break;
                       }
#ifdef CONFIG_HAVE_LONG_LONG
                }
#endif /* CONFIG_HAVE_LONG_LONG */

                 /* Now, get the "real" field width of the integer value*/

                 tmpwidth = strlen(tmp);
                 if (width <= tmpwidth)
                   {
                     /* Just copy the string. */

                     for (ptmp = tmp; *ptmp; )
                       {
                         obj->put(obj, *ptmp);
                         ptmp++;
                       }
                   }
                 else
                   {
                     /* Apply field justification to the integer value. */

                     switch (fmt)
                       {
                       default:
                       case FMT_RJUST:
                         for (n = width - tmpwidth; n; n--)
                           {
                            obj->put(obj, ' ');
                          }

                         for (ptmp = tmp; *ptmp; )
                           {
                            obj->put(obj, *ptmp);
                            ptmp++;
                           }
                         break;

                       case FMT_RJUST0:
                         ptmp = tmp;
                         if (((*ptmp) == '-') || ((*ptmp) == '+'))
                           {
                             obj->put(obj, *ptmp);
                             ptmp++;
                           }

                         for (n = width - tmpwidth; n; n--)
                           {
                             obj->put(obj, '0');
                           }

                         while (*ptmp)
                           {
                             obj->put(obj, *ptmp);
                             ptmp++;
                           }
                         break;

                       case FMT_LJUST:
                         for (ptmp = tmp; *ptmp; )
                           {
                             obj->put(obj, *ptmp);
                             ptmp++;
                           }

                         for (n = width - tmpwidth; n; n--)
                           {
                             obj->put(obj, ' ');
                           }
                         break;
                       }
                   }
              }

              /* Handle floating point conversions */

              else if (strchr("eEfgG", *src))
                {
                  char tmpfmt[40];
                  const char *psrc;
                  char *pdst;
                  double_t dbl;

                  /* Reconstruct the floating point format. */

                  psrc = pfmt;
                  pdst = tmpfmt;
                  while (psrc <= src) *pdst++ = *psrc++;
                  *pdst = 0;

                  /* Extract the floating point number. */

                  dbl = va_arg(ap, double_t);

                  /* Then let the lib_sprintf do the work. */

                  if (hasasteriskwidth)
                    {
                     if (hasasterisktrunc)
                       {
                         lib_sprintf(obj, tmpfmt, width, trunc, dbl);
                       }
                     else
                       {
                         lib_sprintf(obj, tmpfmt, width, dbl);
                       }
                   }
                  else
                    {
                      if (hasasterisktrunc)
                        {
                          lib_sprintf(obj, tmpfmt, trunc, dbl);
                        }
                      else
                        {
                          lib_sprintf(obj, tmpfmt, dbl);
                        }
                    }
                }
            }
        }
    }

  return obj->nput;
}

