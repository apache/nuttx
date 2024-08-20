/****************************************************************************
 * libs/libc/stdio/lib_libbsprintf.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/streams.h>

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lib_bsprintf(FAR struct lib_outstream_s *s, FAR const IPTR char *fmt,
                 FAR const void *buf)
{
  begin_packed_struct union
    {
      char c;
      short int si;
      int i;
      long l;
#ifdef CONFIG_HAVE_LONG_LONG
      long long ll;
#endif
      intmax_t im;
      size_t sz;
      ptrdiff_t pd;
      uintptr_t p;
#ifdef CONFIG_HAVE_DOUBLE
      float f;
      double d;
#  ifdef CONFIG_HAVE_LONG_DOUBLE
      long double ld;
#  endif
#endif
    }

  end_packed_struct *var;
  FAR const char *prec = NULL;
  FAR const char *data = buf;
  char fmtstr[64];
  bool infmt = false;
  size_t offset = 0;
  size_t ret = 0;
  size_t len = 0;
  char c;

  while ((c = *fmt++) != '\0')
    {
      if (c != '%' && !infmt)
        {
          lib_stream_putc(s, c);
          ret++;
          continue;
        }

      if (!infmt)
        {
          len = 0;
          infmt = true;
          memset(fmtstr, 0, sizeof(fmtstr));
        }

      var = (FAR void *)((char *)buf + offset);
      fmtstr[len++] = c;

      if (c == 'c' || c == 'd' || c == 'i' || c == 'u' ||
          c == 'o' || c == 'x' || c == 'X')
        {
          if (*(fmt - 2) == 'j')
            {
              offset += sizeof(var->im);
              ret += lib_sprintf(s, fmtstr, var->im);
            }
#ifdef CONFIG_HAVE_LONG_LONG
          else if (*(fmt - 2) == 'l' && *(fmt - 3) == 'l')
            {
              offset += sizeof(var->ll);
              ret += lib_sprintf(s, fmtstr, var->ll);
            }
#endif
          else if (*(fmt - 2) == 'l')
            {
              offset += sizeof(var->l);
              ret += lib_sprintf(s, fmtstr, var->l);
            }
          else if (*(fmt - 2) == 'z')
            {
              offset += sizeof(var->sz);
              ret += lib_sprintf(s, fmtstr, var->sz);
            }
          else if (*(fmt - 2) == 't')
            {
              offset += sizeof(var->pd);
              ret += lib_sprintf(s, fmtstr, var->pd);
            }
          else if (*(fmt - 2) == 'h' && *(fmt - 3) == 'h')
            {
              offset += sizeof(var->c);
              ret += lib_sprintf(s, fmtstr, var->c);
            }
          else if (*(fmt - 2) == 'h')
            {
              offset += sizeof(var->si);
              ret += lib_sprintf(s, fmtstr, var->si);
            }
          else
            {
              offset += sizeof(var->i);
              ret += lib_sprintf(s, fmtstr, var->i);
            }

          infmt = false;
        }
      else if (c == 'e' || c == 'f' || c == 'g' || c == 'a' ||
               c == 'A' || c == 'E' || c == 'F' || c == 'G')
        {
#ifdef CONFIG_HAVE_DOUBLE
          if (*(fmt - 2) == 'h')
            {
              offset += sizeof(var->f);
              ret += lib_sprintf(s, fmtstr, var->f);
            }
#  ifdef CONFIG_HAVE_LONG_DOUBLE
          else if (*(fmt - 2) == 'L')
            {
              offset += sizeof(var->ld);
              ret += lib_sprintf(s, fmtstr, var->ld);
            }
#  endif
          else
            {
              offset += sizeof(var->d);
              ret += lib_sprintf(s, fmtstr, var->d);
            }

          infmt = false;
#endif
        }
      else if (c == '*')
        {
          sprintf(fmtstr + len - 1, "%d", var->i);
          len = strlen(fmtstr);
          offset += sizeof(var->i);
        }
      else if (c == 's')
        {
          FAR const char *value = data + offset;

          if (prec != NULL)
            {
              offset += strtol(prec, NULL, 10);
              prec = NULL;
            }
          else
            {
              offset += strlen(value) + 1;
            }

          ret += lib_sprintf(s, fmtstr, value);
          infmt = false;
        }
      else if (c == 'p')
        {
          offset += sizeof(var->p);
          ret += lib_sprintf(s, fmtstr, var->p);
          infmt = false;
        }
      else if (c == '.')
        {
          prec = fmt;
        }
    }

  return ret;
}
