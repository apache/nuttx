/****************************************************************************
 * libs/libc/wctype/lib_iswctype.c
 *
 *    Copyright (c) 2002 Red Hat Incorporated.
 *    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   The name of Red Hat Incorporated may not be used to endorse
 *   or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL RED HAT INCORPORATED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <ctype.h>
#include <wctype.h>
#include <errno.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int iswalnum(wint_t c)
{
  return (iswalpha(c) || iswdigit(c));
}

int iswalpha(wint_t c)
{
  return (c < (wint_t)0x100 ? isalpha(c) : 0);
}

int iswblank(wint_t c)
{
  return (c < (wint_t)0x100 ? isblank(c) : 0);
}

int iswcntrl(wint_t c)
{
  return (c < (wint_t)0x100 ? iscntrl(c) : 0);
}

int iswdigit(wint_t c)
{
  return (c >= (wint_t)'0' && c <= (wint_t)'9');
}

int iswgraph(wint_t c)
{
  return (iswprint(c) && !iswspace(c));
}

int iswlower(wint_t c)
{
  return (towupper(c) != c);
}

int iswprint(wint_t c)
{
  return (c < (wint_t) 0x100 ? isprint(c) : 0);
}

int iswpunct(wint_t c)
{
  return (!iswalnum(c) && iswgraph(c));
}

int iswspace(wint_t c)
{
  return (c < 0x100 ? isspace(c) : 0);
}

int iswupper(wint_t c)
{
  return (towlower(c) != c);
}

int iswxdigit(wint_t c)
{
  return ((c >= (wint_t)'0' && c <= (wint_t)'9') ||
          (c >= (wint_t)'a' && c <= (wint_t)'f') ||
          (c >= (wint_t)'A' && c <= (wint_t)'F'));
}

int iswctype(wint_t c, wctype_t desc)
{
  switch (desc)
    {
    case WC_ALNUM:
      return iswalnum(c);

    case WC_ALPHA:
      return iswalpha(c);

    case WC_BLANK:
      return iswblank(c);

    case WC_CNTRL:
      return iswcntrl(c);

    case WC_DIGIT:
      return iswdigit(c);

    case WC_GRAPH:
      return iswgraph(c);

    case WC_LOWER:
      return iswlower(c);

    case WC_PRINT:
      return iswprint(c);

    case WC_PUNCT:
      return iswpunct(c);

    case WC_SPACE:
      return iswspace(c);

    case WC_UPPER:
      return iswupper(c);

    case WC_XDIGIT:
      return iswxdigit(c);

    default:
      return 0;                 /* eliminate warning */
    }

  /* Otherwise unknown */

  return 0;
}
#endif
