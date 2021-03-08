/****************************************************************************
 * libs/libc/lzf/lzf_d.c
 *
 * Copyright (c) 2000-2010
 *      Author:  Marc Alexander Lehmann <schmorp@schmorp.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MER CHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTH- ERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lzf/lzf.h"

#ifdef CONFIG_LIBC_LZF

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lzf_decompress
 *
 * Description:
 *   Decompress data compressed with some version of the lzf_compress
 *   function and stored at location in_data and length in_len. The result
 *   will be stored at out_data up to a maximum of out_len characters.
 *
 *   If the output buffer is not large enough to hold the decompressed
 *   data, a 0 is returned and errno is set to E2BIG. Otherwise the number
 *   of decompressed bytes (i.e. the original length of the data) is
 *   returned.
 *
 *   If an error in the compressed data is detected, a zero is returned and
 *   errno is set to EINVAL.
 *
 *   This function is very fast, about as fast as a copying loop.
 *
 ****************************************************************************/

unsigned int lzf_decompress (FAR const void *const in_data,
                             unsigned int in_len, FAR void *out_data,
                             unsigned int out_len)
{
  FAR uint8_t const *ip = (const uint8_t *)in_data;
  FAR uint8_t       *op = (uint8_t *)out_data;
  FAR uint8_t const *const in_end  = ip + in_len;
  FAR uint8_t       *const out_end = op + out_len;

  do
    {
      unsigned int ctrl = *ip++;

      if (ctrl < (1 << 5)) /* literal run */
        {
          ctrl++;

          if (op + ctrl > out_end)
            {
              set_errno(E2BIG);
              return 0;
            }

#if CHECK_INPUT
          if (ip + ctrl > in_end)
            {
              set_errno(EINVAL);
              return 0;
            }
#endif

#ifdef lzf_movsb
          lzf_movsb(op, ip, ctrl);
#else
          switch (ctrl)
            {
              case 32:
                *op++ = *ip++;

              case 31:
                *op++ = *ip++;

              case 30:
                *op++ = *ip++;

              case 29:
                *op++ = *ip++;

              case 28:
                *op++ = *ip++;

              case 27:
                *op++ = *ip++;

              case 26:
                *op++ = *ip++;

              case 25:
                *op++ = *ip++;

              case 24:
                *op++ = *ip++;

              case 23:
                *op++ = *ip++;

              case 22:
                *op++ = *ip++;

              case 21:
                *op++ = *ip++;

              case 20:
                *op++ = *ip++;

              case 19:
                *op++ = *ip++;

              case 18:
                *op++ = *ip++;

              case 17:
                *op++ = *ip++;

              case 16:
                *op++ = *ip++;

              case 15:
                *op++ = *ip++;

              case 14:
                *op++ = *ip++;

              case 13:
                *op++ = *ip++;

              case 12:
                *op++ = *ip++;

              case 11:
                *op++ = *ip++;

              case 10:
                *op++ = *ip++;

              case 9:
                *op++ = *ip++;

              case 8:
                *op++ = *ip++;

              case 7:
                *op++ = *ip++;

              case 6:
                *op++ = *ip++;

              case 5:
                *op++ = *ip++;

              case 4:
                *op++ = *ip++;

              case 3:
                *op++ = *ip++;

              case 2:
                *op++ = *ip++;

              case 1:
                *op++ = *ip++;
            }
#endif
        }
      else /* back reference */
        {
          unsigned int len = ctrl >> 5;

          FAR uint8_t *ref = op - ((ctrl & 0x1f) << 8) - 1;

#if CHECK_INPUT
          if (ip >= in_end)
            {
              set_errno(EINVAL);
              return 0;
            }

#endif
          if (len == 7)
            {
              len += *ip++;
#if CHECK_INPUT
              if (ip >= in_end)
                {
                  set_errno(EINVAL);
                  return 0;
                }
#endif
            }

          ref -= *ip++;

          if (op + len + 2 > out_end)
            {
              set_errno(E2BIG);
              return 0;
            }

          if (ref < (uint8_t *)out_data)
            {
              set_errno(EINVAL);
              return 0;
            }

#ifdef lzf_movsb
          len += 2;
          lzf_movsb(op, ref, len);
#else
          switch (len)
            {
              default:
                len += 2;

                if (op >= ref + len)
                  {
                    /* Disjunct areas */

                    memcpy (op, ref, len);
                    op += len;
                  }
                else
                  {
                    /* Overlapping, use octet by octet copying */

                    do
                      {
                        *op++ = *ref++;
                      }
                    while (--len);
                  }

                break;

              case 9:
                *op++ = *ref++;

              case 8:
                *op++ = *ref++;

              case 7:
                *op++ = *ref++;

              case 6:
                *op++ = *ref++;

              case 5:
                *op++ = *ref++;

              case 4:
                *op++ = *ref++;

              case 3:
                *op++ = *ref++;

              case 2:
                *op++ = *ref++;

              case 1:
                *op++ = *ref++;

              case 0:

                /* Two octets more */

                *op++ = *ref++;
                *op++ = *ref++;
            }
#endif
        }
    }
  while (ip < in_end);

  return op - (uint8_t *)out_data;
}

#endif /* CONFIG_LIBC_LZF */
