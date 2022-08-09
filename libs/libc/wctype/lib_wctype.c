/****************************************************************************
 * libs/libc/wctype/lib_wctype.c
 *
 *    Copyright (c) 2002 Red Hat Incorporated.
 *    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL RED HAT INCORPORATED BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <wctype.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

wctype_t wctype(FAR const char *c)
{
  switch (*c)
    {
    case 'a':
      if (!strcmp(c, "alnum"))
        {
          return WC_ALNUM;
        }
      else if (!strcmp(c, "alpha"))
        {
          return WC_ALPHA;
        }

      break;

    case 'b':
      if (!strcmp(c, "blank"))
        {
          return WC_BLANK;
        }

      break;

    case 'c':
      if (!strcmp(c, "cntrl"))
        {
          return WC_CNTRL;
        }

      break;

    case 'd':
      if (!strcmp(c, "digit"))
        {
          return WC_DIGIT;
        }

      break;

    case 'g':
      if (!strcmp(c, "graph"))
        {
          return WC_GRAPH;
        }

      break;

    case 'l':
      if (!strcmp(c, "lower"))
        {
          return WC_LOWER;
        }

      break;

    case 'p':
      if (!strcmp(c, "print"))
        {
          return WC_PRINT;
        }
      else if (!strcmp(c, "punct"))
        {
          return WC_PUNCT;
        }

      break;

    case 's':
      if (!strcmp(c, "space"))
        {
          return WC_SPACE;
        }

      break;

    case 'u':
      if (!strcmp(c, "upper"))
        {
          return WC_UPPER;
        }

      break;

    case 'x':
      if (!strcmp(c, "xdigit"))
        {
          return WC_XDIGIT;
        }

      break;
    }

  return 0;
}
