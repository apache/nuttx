/****************************************************************************
 * libs/libc/netdb/lib_dn.c
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

#include <string.h>
#include <resolv.h>

/* RFC 1035 message compression */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* label start offsets of a compressed domain name s */

static int getoffs(FAR short *offs, FAR const unsigned char *base,
                   FAR const unsigned char *s)
{
  int i = 0;

  for (; ; )
    {
      while ((*s & 0xc0) != 0)
        {
          if ((*s & 0xc0) != 0xc0)
            {
              return 0;
            }

          s = base + ((s[0] & 0x3f) << 8 | s[1]);
        }

      if (*s == 0)
        {
          return i;
        }

      if (s - base >= 0x4000)
        {
          return 0;
        }

      offs[i++] = s - base;
      s += *s + 1;
    }
}

/* label lengths of an ascii domain name s */

static int getlens(FAR unsigned char *lens, FAR const char *s, int l)
{
  int i = 0;
  int j = 0;
  int k = 0;

  for (; ; )
    {
      for (; j < l && s[j] != '.'; j++);
      if (j - k - 1 > 62)
        {
          return 0;
        }

      lens[i++] = j - k;
      if (j == l)
        {
          return i;
        }

      k = ++j;
    }
}

/* longest suffix match of an ascii domain with a compressed domain name dn */

static int match(FAR int *offset, FAR const unsigned char *base,
                 FAR const unsigned char *dn, FAR const char *end,
                 FAR const unsigned char *lens, int nlen)
{
  int l;
  int o;
  int m = 0;
  short offs[128];
  int noff = getoffs(offs, base, dn);

  if (noff == 0)
    {
      return 0;
    }

  for (; ; )
    {
      l = lens[--nlen];
      o = offs[--noff];
      end -= l;
      if (l != base[o] || memcmp(base + o + 1, end, l) != 0)
        {
          return m;
        }

      *offset = o;
      m += l;
      if (nlen > 0)
        {
          m++;
        }

      if (nlen == 0 || noff == 0)
        {
          return m;
        }

      end--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dn_comp(FAR const char *src, FAR unsigned char *dst, int space,
            FAR unsigned char **dnptrs, FAR unsigned char **lastdnptr)
{
  int i;
  int j;
  int n;
  int m = 0;
  int offset = 0;
  int bestlen = 0;
  int bestoff = 0;
  unsigned char lens[127];
  FAR unsigned char **p = dnptrs;
  FAR const char *end;
  size_t l = strnlen(src, 255);

  if (l > 0 && src[l - 1] == '.')
    {
      l--;
    }

  if (l > 253 || space <= 0)
    {
      return -1;
    }

  if (l == 0)
    {
      *dst = 0;
      return 1;
    }

  end = src + l;
  n = getlens(lens, src, l);
  if (n == 0)
    {
      return -1;
    }

  if (dnptrs != NULL && *dnptrs != NULL)
    {
      for (p = dnptrs + 1 ; *p != NULL; p++)
        {
          m = match(&offset, *dnptrs, *p, end, lens, n);
          if (m > bestlen)
            {
              bestlen = m;
              bestoff = offset;
              if (m == l)
                {
                  break;
                }
            }
        }
    }

  /* encode unmatched part */

  if (space < l - bestlen + 2 + (bestlen - 1 < l - 1))
    {
      return -1;
    }

  memcpy(dst + 1, src, l - bestlen);
  for (i = j = 0; i < l - bestlen; i += lens[j++] + 1)
    {
      dst[i] = lens[j];
    }

  /* add tail */

  if (bestlen > 0)
    {
      dst[i++] = 0xc0 | bestoff >> 8;
      dst[i++] = bestoff;
    }
  else
    {
      dst[i++] = 0;
    }

  /* save dst pointer */

  if (i > 2 && lastdnptr != NULL && dnptrs != NULL && *dnptrs != NULL)
    {
      while (*p != NULL)
        {
          p++;
        }

      if (p + 1 < lastdnptr)
        {
          *p++ = dst;
          *p = NULL;
        }
    }

  return i;
}

int dn_expand(FAR const unsigned char *base, FAR const unsigned char *end,
              FAR const unsigned char *src, FAR char *dest, int space)
{
  FAR const unsigned char *p = src;
  FAR char *dend;
  FAR char *dbegin = dest;
  int len = -1;
  int i;
  int j;

  if (p == end || space <= 0)
    {
      return -1;
    }

  dend = dest + (space > 254 ? 254 : space);

  /* detect reference loop using an iteration counter */

  for (i = 0; i < end - base; i += 2)
    {
      /* loop invariants: p<end, dest<dend */

      if ((*p & 0xc0) != 0)
        {
          if (p + 1 == end)
            {
              return -1;
            }

          j = ((p[0] & 0x3f) << 8) | p[1];
          if (len < 0)
            {
              len = p + 2 - src;
            }

          if (j >= end - base)
            {
              return -1;
            }

          p = base + j;
        }
      else if (*p)
        {
          if (dest != dbegin)
            {
              *dest++ = '.';
            }

          j = *p++;
          if (j >= end - p || j >= dend - dest)
            {
              return -1;
            }

          while (j--)
            {
              *dest++ = *p++;
            }
        }
      else
        {
          *dest = 0;
          if (len < 0)
            {
              len = p + 1 - src;
            }

          return len;
        }
    }

  return -1;
}
