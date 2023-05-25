/****************************************************************************
 * libs/libc/locale/lib_iconv.c
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

#include <iconv.h>
#include <errno.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <locale.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UTF_32BE    0300
#define UTF_16LE    0301
#define UTF_16BE    0302
#define UTF_32LE    0303
#define UCS2BE      0304
#define UCS2LE      0305
#define WCHAR_T     0306
#define US_ASCII    0307
#define UTF_8       0310
#define UTF_16      0312
#define UTF_32      0313
#define UCS2        0314
#define EUC_JP      0320
#define SHIFT_JIS   0321
#define ISO2022_JP  0322
#define GB18030     0330
#define GBK         0331
#define GB2312      0332
#define BIG5        0340
#define EUC_KR      0350

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stateful_cd
{
  iconv_t base_cd;
  unsigned state;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Definitions of g_charmaps. Each charmap consists of:
 * 1. Empty-string-terminated list of null-terminated aliases.
 * 2. Special type code or number of elided quads of entries.
 * 3. Character table (size determined by field 2), consisting
 *    of 5 bytes for every 4 characters, interpreted as 10-bit
 *    indices into the g_legacy_chars table.
 */

static const unsigned char g_charmaps[] =
{
  "utf8\0char\0\0\310"
  "wchart\0\0\306"
  "ucs2be\0\0\304"
  "ucs2le\0\0\305"
  "utf16be\0\0\302"
  "utf16le\0\0\301"
  "ucs4be\0utf32be\0\0\300"
  "ucs4le\0utf32le\0\0\303"
  "ascii\0usascii\0iso646\0iso646us\0\0\307"
  "utf16\0\0\312"
  "ucs4\0utf32\0\0\313"
  "ucs2\0\0\314"
#ifdef CONFIG_LIBC_LOCALE_JAPANESE
  "eucjp\0\0\320"
  "shiftjis\0sjis\0\0\321"
  "iso2022jp\0\0\322"
#endif
#ifdef CONFIG_LIBC_LOCALE_CHINESE
  "gb18030\0\0\330"
  "gbk\0\0\331"
  "gb2312\0\0\332"
  "big5\0bigfive\0cp950\0big5hkscs\0\0\340"
#endif
#ifdef CONFIG_LIBC_LOCALE_KOREAN
  "euckr\0ksc5601\0ksx1001\0cp949\0\0\350"
#endif
#ifdef CONFIG_LIBC_LOCALE_CODEPAGES
#  include "codepages.h"
#endif
};

/* Table of characters that appear in legacy 8-bit codepages,
 * limited to 1024 slots (10 bit indices). The first 256 entries
 * are elided since those characters are obviously all included.
 */

static const unsigned short g_legacy_chars[] =
{
#include "legacychars.h"
};

#ifdef CONFIG_LIBC_LOCALE_JAPANESE
static const unsigned short g_jis0208[84][94] =
{
#include "jis0208.h"
};

static const unsigned short g_rev_jis[] =
{
#include "revjis.h"
};
#endif

#ifdef CONFIG_LIBC_LOCALE_CHINESE
static const unsigned short g_gb18030[126][190] =
{
#include "gb18030.h"
};

static const unsigned short g_big5[89][157] =
{
#include "big5.h"
};

static const unsigned short g_hkscs[] =
{
#include "hkscs.h"
};
#endif

#ifdef CONFIG_LIBC_LOCALE_KOREAN
static const unsigned short g_ksc[93][94] =
{
#include "ksc.h"
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int fuzzycmp(FAR const unsigned char *a, FAR const unsigned char *b)
{
  for (; *a && *b; a++, b++)
    {
      while (*a && (*a | 32U) - 'a' > 26 && *a - '0' > 10U)
        a++;

      if ((*a | 32U) != *b)
        {
          return 1;
        }
    }

  return *a != *b;
}

static size_t find_charmap(FAR const void *name)
{
  FAR const unsigned char *s;

  if (*(FAR char *)name == '\0')
    {
      /* "utf8" */

      name = g_charmaps;
    }

  for (s = g_charmaps; *s != '\0'; )
    {
      if (!fuzzycmp(name, s))
        {
          for (; *s; s += strlen((FAR const char *)s) + 1);
          return s + 1 - g_charmaps;
        }

      s += strlen((FAR const char *)s) + 1;
      if (*s == '\0')
        {
          if (s[1] > 0200)
            {
              s += 2;
            }
          else
            {
              s += 2 + (64U - s[1]) * 5;
            }
        }
    }

  return -1;
}

static iconv_t combine_to_from(size_t t, size_t f)
{
  return (iconv_t)(f << 16 | t << 1 | 1);
}

static size_t extract_from(iconv_t cd)
{
  return (size_t)cd >> 16;
}

static size_t extract_to(iconv_t cd)
{
  return (size_t)cd >> 1 & 0x7fff;
}

static unsigned get_16(FAR const unsigned char *s, int e)
{
  e &= 1;
  return s[e] << 8 | s[1 - e];
}

static void put_16(FAR unsigned char *s, unsigned c, int e)
{
  e &= 1;
  s[e] = c >> 8;
  s[1 - e] = c;
}

static unsigned get_32(FAR const unsigned char *s, int e)
{
  e &= 3;
  return (s[e] + 0U) << 24 | s[e ^ 1] << 16 | s[e ^ 2] << 8 | s[e ^ 3];
}

static void put_32(FAR unsigned char *s, unsigned c, int e)
{
  e &= 3;
  s[e ^ 0] = c >> 24;
  s[e ^ 1] = c >> 16;
  s[e ^ 2] = c >> 8;
  s[e ^ 3] = c;
}

static unsigned legacy_map(const unsigned char *map, unsigned c)
{
  unsigned x;

  if (c < 4 * map[0 - 1])
    {
      return c;
    }

  x = c - 4 * map[0 - 1];
  x = (map[x * 5 / 4] >> (2 * x % 8)) |
      ((map[x * 5 / 4 + 1] << (8 - 2 * x % 8)) & 1023);
  return x < 256 ? x : g_legacy_chars[x - 256];
}

#ifdef CONFIG_LIBC_LOCALE_JAPANESE
static unsigned uni_to_jis(unsigned c)
{
  unsigned nitems = sizeof(g_rev_jis) / sizeof(*g_rev_jis);
  unsigned d;
  unsigned j;
  unsigned i;
  unsigned b;

  for (; ; )
    {
      i = nitems / 2;
      j = g_rev_jis[b + i];
      d = g_jis0208[j / 256][j % 256];
      if (d == c)
        {
          return j + 0x2121;
        }
      else if (nitems == 1)
        {
          return 0;
        }
      else if (c < d)
        {
          nitems /= 2;
        }
      else
        {
          b += i;
          nitems -= nitems / 2;
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iconv_open
 *
 * Description:
 *   Allocates a conversion descriptor suitable for converting byte
 *   sequences from character encoding "from" to character encoding "to".
 *
 * Input Parameters:
 *   to   - A pointer to character encoding of "to".
 *   from - A pointer to character encoding of "from".
 *
 * Returned Value:
 *   Returns a freshly allocated conversion descriptor.
 *   In case of error, it sets errno and returns (iconv_t) -1.
 *
 ****************************************************************************/

iconv_t iconv_open(FAR const char *to, FAR const char *from)
{
  FAR struct stateful_cd *scd;
  iconv_t cd;
  size_t f;
  size_t t;

  if ((t = find_charmap(to)) == -1 || (f = find_charmap(from)) == -1 ||
      (g_charmaps[t] >= 0330))
    {
      errno = EINVAL;
      return (iconv_t)-1;
    }

  cd = combine_to_from(t, f);
  switch (g_charmaps[f])
    {
      case UTF_16:
      case UTF_32:
      case UCS2:
#ifdef CONFIG_LIBC_LOCALE_JAPANESE
      case ISO2022_JP:
#endif
        {
          scd = lib_malloc(sizeof(*scd));
          if (scd == NULL)
            {
              errno = ENOMEM;
              return (iconv_t)-1;
            }

          scd->base_cd = cd;
          scd->state = 0;
          cd = (iconv_t)scd;
        }
    }

  return cd;
}

/****************************************************************************
 * Name: iconv
 *
 * Description:
 *   Converts a sequence of characters in one character encoding to a
 *   sequence of characters in another character encoding.
 *
 * Input Parameters:
 *   cd   - A conversion descriptor, previously created by a call
 *          to iconv_open.
 *   in   - The address of a variable that points to the first character of
 *          the input sequence.
 *   inb  - The number of bytes in input buffer.
 *   out  - The address of a variable that points to the first byte
 *          available in the output buffer.
 *   outb - The number of bytes available in the output buffer.
 *
 * Returned Value:
 *   Returns the number of characters converted in a nonreversible way
 *   during this call; reversible conversions are not counted, In case of
 *   error, it sets errno and return (size_t) -1.
 *
 ****************************************************************************/

size_t iconv(iconv_t cd, FAR char **in, FAR size_t *inb,
             FAR char **out, FAR size_t *outb)
{
  FAR struct stateful_cd *scd;
  FAR const unsigned char *tomap;
  FAR const unsigned char *map;
  unsigned char totype;
  unsigned char type;
  unsigned from;
  mbstate_t st;
  unsigned to;
  unsigned c;
  unsigned d;
  wchar_t wc;
  size_t k;
  size_t l;
  size_t x;
  int err;

  x = 0;
  scd = NULL;
  if (((size_t)cd & 1) == 0)
    {
      scd = (FAR void *)cd;
      cd = scd->base_cd;
    }

  to = extract_to(cd);
  from = extract_from(cd);
  map = g_charmaps + from + 1;
  tomap = g_charmaps + to + 1;
  type = map[0 - 1];
  totype = tomap[0 - 1];

  if (in == NULL || *in == NULL || *inb == 0)
    {
      return 0;
    }

  for (; *inb; *in += l, *inb -= l)
    {
      c = *(FAR unsigned char *)*in;
      l = 1;

      switch (type)
        {
          case UTF_8:
            {
              if (c < 128)
                {
                  break;
                }

              memset(&st, 0, sizeof(st));
              l = mbrtowc(&wc, *in, *inb, &st);
              if (l == (size_t)-1)
                {
                  goto ilseq;
                }

              if (l == (size_t)-2)
                {
                  goto starved;
                }

              c = wc;
            }
            break;

          case US_ASCII:
            {
              if (c >= 128)
                {
                  goto ilseq;
                }
            }
            break;

          case WCHAR_T:
            {
              l = sizeof(wchar_t);
              if (*inb < l)
                {
                  goto starved;
                }

              c = *(FAR wchar_t *)*in;
              if (c - 0xd800u < 0x800u || c >= 0x110000u)
                {
                  goto ilseq;
                }
            }
            break;

          case UCS2BE:
          case UCS2LE:
          case UTF_16BE:
          case UTF_16LE:
            {
              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              c = get_16((FAR void *)*in, type);
              if ((unsigned)(c - 0xdc00) < 0x400)
                {
                  goto ilseq;
                }

              if ((unsigned)(c - 0xd800) < 0x400)
                {
                if (type - UCS2BE < 2U)
                    {
                      goto ilseq;
                    }

                l = 4;
                if (*inb < 4)
                    {
                      goto starved;
                    }

                d = get_16((FAR void *)(*in + 2), type);
                if ((unsigned)(d - 0xdc00) >= 0x400)
                    {
                      goto ilseq;
                    }

                c = ((c - 0xd7c0) << 10) + (d - 0xdc00);
                }
            }
            break;

          case UCS2:
          case UTF_16:
            {
              l = 0;
              if (!scd->state)
                {
                  if (*inb < 2)
                    goto starved;

                  c = get_16((FAR void *)*in, 0);
                  scd->state = (type == UCS2
                                ? c == 0xfffe ? UCS2LE : UCS2BE
                                : c == 0xfffe ? UTF_16LE : UTF_16BE);
                  if (c == 0xfffe || c == 0xfeff)
                    {
                      l = 2;
                    }
                }

              type = scd->state;
              continue;
            }

          case UTF_32:
            {
              l = 0;
              if (!scd->state)
                {
                if (*inb < 4)
                    {
                      goto starved;
                    }

                c = get_32((FAR void *)*in, 0);
                scd->state = (c == 0xfffe0000 ? UTF_32LE : UTF_32BE);
                if (c == 0xfffe0000 || c == 0xfeff)
                  {
                    l = 4;
                  }
                }

              type = scd->state;
              continue;
            }

#ifdef CONFIG_LIBC_LOCALE_JAPANESE
          case SHIFT_JIS:
            {
              if (c < 128)
                {
                  break;
                }

              if (c - 0xa1 <= 0xdf - 0xa1)
                {
                  c += 0xff61 - 0xa1;
                  break;
                }

              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              d = *((FAR unsigned char *)*in + 1);
              if (c - 129 <= 159 - 129)
                {
                  c -= 129;
                }
              else if (c - 224 <= 239 - 224)
                {
                  c -= 193;
                }
              else
                {
                  goto ilseq;
                }

              c *= 2;
              if (d - 64 <= 158 - 64)
                {
                if (d == 127)
                    {
                      goto ilseq;
                    }

                if (d > 127)
                    {
                      d--;
                    }

                d -= 64;
                }
              else if (d - 159 <= 252 - 159)
                {
                c++;
                d -= 159;
                }

              c = g_jis0208[c][d];
              if (!c)
                {
                  goto ilseq;
                }
            }
            break;

          case EUC_JP:
            {
              if (c < 128)
                {
                  break;
                }

              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              d = *((FAR unsigned char *)*in + 1);
              if (c == 0x8e)
                {
                  c = d;
                  if (c - 0xa1 > 0xdf - 0xa1)
                    {
                      goto ilseq;
                    }

                  c += 0xff61 - 0xa1;
                  break;
                }

              c -= 0xa1;
              d -= 0xa1;
              if (c >= 84 || d >= 94)
                {
                  goto ilseq;
                }

              c = g_jis0208[c][d];
              if (!c)
                {
                  goto ilseq;
                }
            }
            break;

          case ISO2022_JP:
            {
              if (c >= 128)
                {
                  goto ilseq;
                }

              if (c == '\033')
                {
                  l = 3;
                  if (*inb < 3)
                    {
                      goto starved;
                    }

                  c = *((FAR unsigned char *)*in + 1);
                  d = *((FAR unsigned char *)*in + 2);
                  if (c != '(' && c != '$')
                    {
                      goto ilseq;
                    }

                  switch (128 * (c == '$') + d)
                    {
                      case 'B':
                        {
                          scd->state = 0;
                          continue;
                        }

                      case 'J':
                        {
                          scd->state = 1;
                          continue;
                        }

                      case 'I':
                        {
                          scd->state = 4;
                          continue;
                        }

                      case 128 + '@':
                        {
                          scd->state = 2;
                          continue;
                        }

                      case 128 + 'B':
                        {
                          scd->state = 3;
                          continue;
                        }
                    }

                  goto ilseq;
                }

              switch (scd->state)
                {
                  case 1:
                    {
                      if (c == '\\')
                        {
                          c = 0xa5;
                        }

                      if (c == '~')
                        {
                          c = 0x203e;
                        }
                    }
                    break;

                  case 2:
                  case 3:
                    {
                      l = 2;
                      if (*inb < 2)
                        {
                          goto starved;
                        }

                      d = *((FAR unsigned char *)*in + 1);
                      c -= 0x21;
                      d -= 0x21;
                      if (c >= 84 || d >= 94)
                        {
                          goto ilseq;
                        }

                      c = g_jis0208[c][d];
                      if (!c)
                        {
                          goto ilseq;
                        }
                    }
                    break;

                  case 4:
                    {
                      if (c - 0x60 < 0x1f)
                        {
                          goto ilseq;
                        }

                      if (c - 0x21 < 0x5e)
                        {
                          c += 0xff61 - 0x21;
                        }
                    }
                    break;
                }
            }
            break;
#endif

#ifdef CONFIG_LIBC_LOCALE_CHINESE
          case GB2312:
            {
              if (c < 128)
                {
                  break;
                }

              if (c < 0xa1)
                {
                  goto ilseq;
                }
            }

          case GBK:
          case GB18030:
            {
              if (c < 128)
                {
                  break;
                }

              c -= 0x81;
              if (c >= 126)
                {
                  goto ilseq;
                }

              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              d = *((unsigned char *)*in + 1);
              if (d < 0xa1 && type == GB2312)
                {
                  goto ilseq;
                }

              if (d - 0x40 >= 191 || d == 127)
                {
                  if (d - '0' > 9 || type != GB18030)
                    {
                      goto ilseq;
                    }

                  l = 4;
                  if (*inb < 4)
                    {
                      goto starved;
                    }

                  c = (10 * c + d - '0') * 1260;
                  d = *((FAR unsigned char *)*in + 2);
                  if (d - 0x81 > 126)
                    {
                      goto ilseq;
                    }

                  c += 10 * (d - 0x81);
                  d = *((FAR unsigned char *)*in + 3);
                  if (d - '0' > 9)
                    {
                      goto ilseq;
                    }

                  c += d - '0';
                  c += 128;
                  for (d = 0; d <= c; )
                    {
                      int i;

                      k = 0;
                      for (i = 0; i < 126; i++)
                        {
                          int j;

                          for (j = 0; j < 190; j++)
                            {
                              if (g_gb18030[i][j] - d <= c - d)
                                {
                                  k++;
                                }
                            }
                        }

                      d = c + 1;
                      c += k;
                    }

                  break;
                }

              d -= 0x40;
              if (d > 63)
                {
                  d--;
                }

              c = g_gb18030[c][d];
            }
            break;

          case BIG5:
            {
              if (c < 128)
                {
                  break;
                }

              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              d = *((FAR unsigned char *)*in + 1);
              if (d - 0x40 >= 0xff - 0x40 || d - 0x7f < 0xa1 - 0x7f)
                {
                  goto ilseq;
                }

              d -= 0x40;
              if (d > 0x3e)
                {
                  d -= 0x22;
                }

              if (c - 0xa1 >= 0xfa - 0xa1)
                {
                  if (c - 0x87 >= 0xff - 0x87)
                    {
                      goto ilseq;
                    }

                  if (c < 0xa1)
                    {
                      c -= 0x87;
                    }
                  else
                    {
                      c -= 0x87 + (0xfa - 0xa1);
                    }

                  c = (g_hkscs[4867 + (c * 157 + d) / 16] >>
                       (c * 157 + d) % 16) % 2 << 17 | g_hkscs[c * 157 + d];

                  /* A few HKSCS characters map to pairs of UCS
                   * characters. These are mapped to surrogate
                   * range in the hkscs table then hard-coded
                   * here. Ugly, yes.
                   */

                  if (c / 256 == 0xdc)
                    {
                      FAR char *ptmp;
                      size_t out_len;
                      size_t in_len;
                      size_t tmplen;
                      size_t tmpx;
                      union
                        {
                          char c[8];
                          wchar_t wc[2];
                        } tmp;

                      char *in_buf =
                      {
                        "\303\212\314\204"
                        "\303\212\314\214"
                        "\303\252\314\204"
                        "\303\252\314\214"
                        + c % 256
                      };

                      in_len = 4;
                      ptmp = tmp.c;
                      out_len = sizeof(tmp);
                      tmpx = iconv(combine_to_from(to, find_charmap("utf8")),
                                   &in_buf, &in_len, &ptmp, &out_len);
                      tmplen = ptmp - tmp.c;
                      if (tmplen > *outb)
                        {
                          goto toobig;
                        }

                      if (tmpx)
                        {
                          x++;
                        }

                      memcpy(*out, &tmp, tmplen);
                      *out += tmplen;
                      *outb -= tmplen;
                      continue;
                    }

                  if (!c)
                    {
                      goto ilseq;
                    }

                  break;
                }

              c -= 0xa1;
              c = g_big5[c][d] | (c == 0x27 && (d == 0x3a ||
                                  d == 0x3c || d == 0x42)) << 17;
              if (!c)
                {
                  goto ilseq;
                }
            }
            break;
#endif

#ifdef CONFIG_LIBC_LOCALE_KOREAN
          case EUC_KR:
            {
              if (c < 128)
                {
                  break;
                }

              l = 2;
              if (*inb < 2)
                {
                  goto starved;
                }

              d = *((FAR unsigned char *)*in + 1);
              c -= 0xa1;
              d -= 0xa1;
              if (c >= 93 || d >= 94)
                {
                  c += (0xa1 - 0x81);
                  d += 0xa1;
                  if (c >= 93 || ((c >= 0xc6 - 0x81) && (d > 0x52)))
                    {
                      goto ilseq;
                    }

                  if (d - 'A' < 26)
                    {
                      d = d - 'A';
                    }
                  else if (d - 'a' < 26)
                    {
                      d = d - 'a' + 26;
                    }
                  else if (d - 0x81 < 0xff - 0x81)
                    {
                      d = d - 0x81 + 52;
                    }
                  else
                    {
                      goto ilseq;
                    }

                  if (c < 0x20)
                    {
                      c = 178 * c + d;
                    }
                  else
                    {
                      c = 178 * 0x20 + 84 * (c - 0x20) + d;
                    }

                  c += 0xac00;
                  for (d = 0xac00; d <= c; )
                    {
                      int i;

                      k = 0;
                      for (i = 0; i < 93; i++)
                        {
                          int j;

                          for (j = 0; j < 94; j++)
                            {
                              if (g_ksc[i][j] - d <= c - d)
                                {
                                  k++;
                                }
                            }
                        }

                      d = c + 1;
                      c += k;
                    }
                  break;
                }

              c = g_ksc[c][d];
              if (!c)
                {
                  goto ilseq;
                }
            }
            break;
#endif

          default:
            {
              if (!c)
                {
                  break;
                }

              c = legacy_map(map, c);
              if (!c)
                {
                  goto ilseq;
                }
            }
        }

      switch (totype)
        {
          case WCHAR_T:
            {
              if (*outb < sizeof(wchar_t))
                {
                  goto toobig;
                }

              *(FAR wchar_t *)*out = c;
              *out += sizeof(wchar_t);
              *outb -= sizeof(wchar_t);
            }
            break;

          case UTF_8:
            {
              if (*outb < 4)
                {
                  char tmp[4];
                  k = wctomb(tmp, c);
                  if (*outb < k)
                    {
                      goto toobig;
                    }

                  memcpy(*out, tmp, k);
                }
              else
                {
                  k = wctomb(*out, c);
                }

              *out += k;
              *outb -= k;
            }
            break;

          case US_ASCII:
            if (c > 0x7f)
              {
subst:
                x++;
                c = '*';
              }

          default:
            {
              if (*outb < 1)
                {
                  goto toobig;
                }

              if (c < 256 && c == legacy_map(tomap, c))
                {
revout:
                  if (*outb < 1)
                    {
                      goto toobig;
                    }

                  *(*out)++ = c;
                  *outb -= 1;
                  break;
                }

              d = c;
              for (c = 4 * totype; c < 256; c++)
                {
                  if (d == legacy_map(tomap, c))
                    {
                      goto revout;
                    }
                }

              goto subst;
            }

#ifdef CONFIG_LIBC_LOCALE_JAPANESE
          case SHIFT_JIS:
            {
              if (c < 128)
                {
                  goto revout;
                }

              if (c == 0xa5)
                {
                  x++;
                  c = '\\';
                  goto revout;
                }

              if (c == 0x203e)
                {
                  x++;
                  c = '~';
                  goto revout;
                }

              if (c - 0xff61 <= 0xdf - 0xa1)
                {
                  c += 0xa1 - 0xff61;
                  goto revout;
                }

              c = uni_to_jis(c);
              if (!c)
                {
                  goto subst;
                }

              if (*outb < 2)
                {
                  goto toobig;
                }

              d = c % 256;
              c = c / 256;
              *(*out)++ = (c + 1) / 2 + (c < 95 ? 112 : 176);
              *(*out)++ = c % 2 ? d + 31 + d / 96 : d + 126;
              *outb -= 2;
              break;
            }

          case EUC_JP:
            {
              if (c < 128)
                {
                  goto revout;
                }

              if (c - 0xff61 <= 0xdf - 0xa1)
                {
                  c += 0x0e00 + 0x21 - 0xff61;
                }
              else
                {
                  c = uni_to_jis(c);
                }

              if (!c)
                {
                  goto subst;
                }

              if (*outb < 2)
                {
                  goto toobig;
                }

              *(*out)++ = c / 256 + 0x80;
              *(*out)++ = c % 256 + 0x80;
              *outb -= 2;
            }
            break;

          case ISO2022_JP:
            {
              if (c < 128)
                {
                  goto revout;
                }

              if (c - 0xff61 <= 0xdf - 0xa1 || c == 0xa5 || c == 0x203e)
                {
                  if (*outb < 7)
                    {
                      goto toobig;
                    }

                  *(*out)++ = '\033';
                  *(*out)++ = '(';
                  if (c == 0xa5)
                    {
                      *(*out)++ = 'J';
                      *(*out)++ = '\\';
                    }
                  else if (c == 0x203e)
                    {
                      *(*out)++ = 'J';
                      *(*out)++ = '~';
                    }
                  else
                    {
                      *(*out)++ = 'I';
                      *(*out)++ = c - 0xff61 + 0x21;
                    }

                  *(*out)++ = '\033';
                  *(*out)++ = '(';
                  *(*out)++ = 'B';
                  *outb -= 7;
                  break;
                }

              c = uni_to_jis(c);
              if (!c)
                {
                  goto subst;
                }

              if (*outb < 8)
                {
                  goto toobig;
                }

              *(*out)++ = '\033';
              *(*out)++ = '$';
              *(*out)++ = 'B';
              *(*out)++ = c / 256;
              *(*out)++ = c % 256;
              *(*out)++ = '\033';
              *(*out)++ = '(';
              *(*out)++ = 'B';
              *outb -= 8;
            }
            break;
#endif

          case UCS2:
            totype = UCS2BE;
          case UCS2BE:
          case UCS2LE:
          case UTF_16:
          case UTF_16BE:
          case UTF_16LE:
            {
              if (c < 0x10000 || totype - UCS2BE < 2U)
                {
                  if (c >= 0x10000)
                    {
                      c = 0xfffd;
                    }

                  if (*outb < 2)
                    {
                      goto toobig;
                    }

                  put_16((FAR void *)*out, c, totype);
                  *out += 2;
                  *outb -= 2;
                  break;
                }

              if (*outb < 4)
                {
                  goto toobig;
                }

              c -= 0x10000;
              put_16((FAR void *)*out, (c >> 10) | 0xd800, totype);
              put_16((FAR void *)(*out + 2), (c & 0x3ff) | 0xdc00, totype);
              *out += 4;
              *outb -= 4;
            }
            break;

          case UTF_32:
            totype = UTF_32BE;
          case UTF_32BE:
          case UTF_32LE:
            {
              if (*outb < 4)
                {
                  goto toobig;
                }

              put_32((FAR void *)*out, c, totype);
              *out += 4;
              *outb -= 4;
            }
            break;
        }
    }

  return x;

ilseq:
  err = EILSEQ;
  x = -1;
  goto end;
toobig:
  err = E2BIG;
  x = -1;
  goto end;
starved:
  err = EINVAL;
  x = -1;
end:
  errno = err;
  return x;
}

/****************************************************************************
 * Name: iconv_close
 *
 * Description:
 *   Deallocate descriptor for character set conversion.
 *
 * Input Parameters:
 *   cd - A allocated conversion descriptor.
 *
 * Returned Value:
 *   When successful, the iconv_close() function returns 0. In case of
 *   error, it sets errno and returns -1.
 *
 ****************************************************************************/

int iconv_close(iconv_t cd)
{
  if (((size_t)cd & 1) == 0)
    {
      lib_free((FAR void *)cd);
    }

  return 0;
}
