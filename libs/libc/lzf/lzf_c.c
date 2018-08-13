/****************************************************************************
 * libs/libc/lzf/lzf_c.c
 *
 * Copyright (c) 2000-2010 Marc Alexander Lehmann <schmorp@schmorp.de>
 *
 * Redistribution and use in source and binary forms, with or without modifica-
 * tion, are permitted provided that the following conditions are met:
 *
 *   1.  Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *   2.  Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MER-
 * CHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPE-
 * CIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTH-
 * ERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lzf/lzf.h"

#ifdef CONFIG_LIBC_LZF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HSIZE (1 << HLOG)

/* Don't play with this unless you benchmark!  The data format is not
 * dependent on the hash function. The hash function might seem strange, just
 * believe me, it works ;)
 */

#ifndef FRST
#  define FRST(p)   (((p[0]) << 8) | p[1])
#  define NEXT(v,p) (((v) << 8) | p[2])
#  if defined(CONFIG_LIBC_LZF_FASTEST)
#    define IDX(h)  ((( h             >> (3*8 - HLOG)) - h  ) & (HSIZE - 1))
#  elif defined(CONFIG_LIBC_LZF_FAST)
#    define IDX(h)  ((( h             >> (3*8 - HLOG)) - h*5) & (HSIZE - 1))
#  else
#    define IDX(h)  ((((h ^ (h << 5)) >> (3*8 - HLOG)) - h*5) & (HSIZE - 1))
#  endif
#endif

/* IDX works because it is very similar to a multiplicative hash, e.g.
 * ((h * 57321 >> (3*8 - HLOG)) & (HSIZE - 1))
 * the latter is also quite fast on newer CPUs, and compresses similarly.
 *
 * the next one is also quite good, albeit slow ;)
 * (int)(cos(h & 0xffffff) * 1e6)
 */

#if 0
/* original lzv-like hash function, much worse and thus slower */

#  define FRST(p)   (p[0] << 5) ^ p[1]
#  define NEXT(v,p) ((v) << 5) ^ p[2]
#  define IDX(h)    ((h) & (HSIZE - 1))
#endif

#define MAX_LIT     (1 <<  5)
#define MAX_OFF     (1 << HLOG)
#define MAX_REF     ((1 << 8) + (1 << 3))

#if __GNUC__ >= 3
#  define expect(expr,value) __builtin_expect((expr),(value))
#  define inline             inline
#else
#  define expect(expr,value) (expr)
#  define inline             static
#endif

#define expect_false(expr)   expect((expr) != 0, 0)
#define expect_true(expr)    expect((expr) != 0, 1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lzf_compress
 *
 * Description:
 * Compress in_len bytes stored at the memory block starting at
 *   in_data and write the result to out_data, up to a maximum length
 *   of out_len bytes.
 *
 *   If the output buffer is not large enough or any error occurs return 0,
 *   otherwise return the number of bytes used, which might be considerably
 *   more than in_len (but less than 104% of the original size), so it
 *   makes sense to always use out_len == in_len - 1), to ensure _some_
 *   compression, and store the data uncompressed otherwise (with a flag, of
 *   course.
 *
 *   lzf_compress might use different algorithms on different systems and
 *   even different runs, thus might result in different compressed strings
 *   depending on the phase of the moon or similar factors. However, all
 *   these strings are architecture-independent and will result in the
 *   original data when decompressed using lzf_decompress.
 *
 *   The buffers must not be overlapping.
 *
 *   Compressed format:
 *
 *     000LLLLL <L+1>    ; literal, L+1=1..33 octets
 *     LLLooooo oooooooo ; backref L+1=1..7 octets, o+1=1..4096 offset
 *     111ooooo LLLLLLLL oooooooo ; backref L+8 octets, o+1=1..4096 offset
 *
 ****************************************************************************/

size_t lzf_compress(FAR const void *const in_data,
                    unsigned int in_len, FAR void *out_data,
                    unsigned int out_len, lzf_state_t htab,
                    FAR struct lzf_header_s **reshdr)
{
  FAR const uint8_t *ip = (const uint8_t *)in_data;
  FAR       uint8_t *op = (uint8_t *)out_data;
  FAR const uint8_t *in_end  = ip + in_len;
  FAR       uint8_t *out_end = op + out_len;
  FAR const uint8_t *ref;
  ssize_t cs;
  ssize_t retlen;

  /* off requires a type wide enough to hold a general pointer difference.
   * ISO C doesn't have that (size_t might not be enough and ptrdiff_t only
   * works for differences within a single object). We also assume that no
   * no bit pattern traps. Since the only platform that is both non-POSIX
   * and fails to support both assumptions is windows 64 bit, we make a
   * special workaround for it.
   */

#ifdef CONFIG_HAVE_LONG_LONG
  uint64_t off;  /* Workaround for missing POSIX compliance */
#else
  unsigned long off;
#endif
  unsigned int hval;
  int lit;

  if (!in_len || !out_len)
    {
      cs = 0;
      goto genhdr;
    }

#if INIT_HTAB
  memset(htab, 0, sizeof(htab));
#endif

  lit = 0; /* start run */
  op++;

  hval = FRST(ip);
  while (ip < in_end - 2)
    {
      lzf_hslot_t *hslot;

      hval   = NEXT(hval, ip);
      hslot  = htab + IDX(hval);
      ref    = *hslot + LZF_HSLOT_BIAS;
      *hslot = ip - LZF_HSLOT_BIAS;

      if (1
#if INIT_HTAB
          && ref < ip /* the next test will actually take care of this, but this is faster */
#endif
          && (off = ip - ref - 1) < MAX_OFF
          && ref > (uint8_t *)in_data
          && ref[2] == ip[2]
#ifdef CONFIG_LIBC_LZF_ALIGN
          && ((ref[1] << 8) | ref[0]) == ((ip[1] << 8) | ip[0])
#else
          && *(uint16_t *)ref == *(uint16_t *)ip
#endif
        )
        {
          /* Match found at *ref++ */

          unsigned int len = 2;
          unsigned int maxlen = in_end - ip - len;
          maxlen = maxlen > MAX_REF ? MAX_REF : maxlen;

          /* First a faster conservative test */

          if (expect_false(op + 3 + 1 >= out_end))
            {
              /* Second the exact but rare test */

              if (op - !lit + 3 + 1 >= out_end)
                {
                  cs = 0;
                  goto genhdr;
                }
            }

          op[- lit - 1] = lit - 1; /* Stop run */
          op -= !lit;              /* Undo run if length is zero */

          for (;;)
            {
              if (expect_true(maxlen > 16))
                {
                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != ip[len])
                    {
                      break;
                    }
                }

              do
                {
                  len++;
                }
              while (len < maxlen && ref[len] == ip[len]);

              break;
            }

          len -= 2; /* len is now #octets - 1 */
          ip++;

          if (len < 7)
            {
              *op++ = (off >> 8) + (len << 5);
            }
          else
            {
              *op++ = (off >> 8) + (  7 << 5);
              *op++ = len - 7;
            }

          *op++ = off;

          lit = 0; op++; /* start run */

          ip += len + 1;

          if (expect_false (ip >= in_end - 2))
            {
              break;
            }

#if defined(CONFIG_LIBC_LZF_FASTEST) || defined(CONFIG_LIBC_LZF_FAST)
          --ip;
#  if defined(CONFIG_LIBC_LZF_FAST) && !defined(CONFIG_LIBC_LZF_FASTEST)
          --ip;
#  endif
          hval = FRST(ip);

          hval = NEXT(hval, ip);
          htab[IDX(hval)] = ip - LZF_HSLOT_BIAS;
          ip++;

#  if defined(CONFIG_LIBC_LZF_FAST) && !defined(CONFIG_LIBC_LZF_FASTEST)
          hval = NEXT(hval, ip);
          htab[IDX(hval)] = ip - LZF_HSLOT_BIAS;
          ip++;
#  endif
#else
          ip -= len + 1;

          do
            {
              hval = NEXT(hval, ip);
              htab[IDX(hval)] = ip - LZF_HSLOT_BIAS;
              ip++;
            }
          while (len--);
#endif
        }
      else
        {
          /* One more literal byte we must copy */

          if (expect_false(op >= out_end))
            {
              cs = 0;
              goto genhdr;
            }

          lit++;
          *op++ = *ip++;

          if (expect_false(lit == MAX_LIT))
            {
              op[- lit - 1] = lit - 1; /* Stop run */
              lit = 0;                 /* Start run */
              op++;
            }
        }
    }

  /* At most 3 bytes can be missing here */

  if (op + 3 > out_end)
    {
      cs = 0;
      goto genhdr;
    }

  while (ip < in_end)
    {
      lit++; *op++ = *ip++;

      if (expect_false(lit == MAX_LIT))
        {
          op[- lit - 1] = lit - 1; /* Stop run */
          lit = 0;                 /* Start run */
          op++;
        }
    }

  op[- lit - 1] = lit - 1; /* End run */
  op -= !lit;              /* Undo run if length is zero */

  cs = op - (uint8_t *)out_data;

genhdr:
  if (cs)
    {
      FAR struct lzf_type1_header_s *header;

      /* Write compressed */

      header = (FAR struct lzf_type1_header_s *)
               ((uintptr_t)out_data - LZF_TYPE1_HDR_SIZE);

      header->lzf_magic[0] = 'Z';
      header->lzf_magic[1] = 'V';
      header->lzf_type     = LZF_TYPE1_HDR;
      header->lzf_clen[0]  = cs >> 8;
      header->lzf_clen[1]  = cs & 0xff;
      header->lzf_ulen[0]  = in_len >> 8;
      header->lzf_ulen[1]  = in_len & 0xff;

      *reshdr              = (FAR struct lzf_header_s *)header;
      retlen               = cs + LZF_TYPE1_HDR_SIZE;
    }
  else
    {
      FAR struct lzf_type0_header_s *header;

      /* Write uncompressed */

      header = (FAR struct lzf_type0_header_s *)
               ((uintptr_t)in_data - LZF_TYPE0_HDR_SIZE);

      header->lzf_magic[0] = 'Z';
      header->lzf_magic[1] = 'V';
      header->lzf_type     = LZF_TYPE0_HDR;
      header->lzf_len[0]   = in_len >> 8;
      header->lzf_len[1]   = in_len & 0xff;

      *reshdr              = (FAR struct lzf_header_s *)header;
      retlen               = in_len + LZF_TYPE0_HDR_SIZE;
    }

  return retlen;
}

#endif /* CONFIG_LIBC_LZF */
