
/****************************************************************************
 * libs/libc/misc/lib_ncompress.c
 * File compression ala IEEE Computer, Mar 1992.
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is the file compress24.c extracted from the ncompress-4.2.4 release
 * and adapted for NuttX.  The original code was released into the public
 * domain.   This NuttX version is re-released under the standard NuttX
 * BSD 3-clause license.  The original authors are listed below:
 *
 *   Spencer W. Thomas   (decvax!harpo!utah-cs!utah-gr!thomas)
 *   Jim McKie           (decvax!mcvax!jim)
 *   Steve Davies        (decvax!vax135!petsd!peora!srd)
 *   Ken Turkowski       (decvax!decwrl!turtlevax!ken)
 *   James A. Woods      (decvax!ihnp4!ames!jaw)
 *   Joe Orost           (decvax!vax135!petsd!joe)
 *   Dave Mack           (csu@alembic.acs.com)
 *   Peter Jannesen, Network Communication Systems
 *                       (peter@ncs.nl)
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <signal.h>
#include <errno.h>

#define RECURSIVE  1

#ifdef __STDC__
#  define ARGS(a)    a
#else
#  define ARGS(a)    ()
#endif

#define LARGS(a) ()             /* Relay on include files for library func defs.
                                 */

#ifndef SIG_TYPE
#  define SIG_TYPE void (*)()
#endif

#define MARK(a) { asm(" .globl M.a"); asm("M.a:"); }

#undef min
#define min(a,b) ((a>b) ? b : a)

#ifndef IBUFSIZ
#  define IBUFSIZ BUFSIZ        /* Defailt input buffer size */
#endif
#ifndef OBUFSIZ
#  define OBUFSIZ BUFSIZ        /* Default output buffer size */
#endif

#define MAXPATHLEN 1024         /* MAXPATHLEN - maximum length of a pathname we
                                 * allow */
#define SIZE_INNER_LOOP  256    /* Size of the inter (fast) compress loop */

       /* Defines for third byte of header */
#define MAGIC_1  (char_type)'\037'      /* First byte of compressed file */
#define MAGIC_2  (char_type)'\235'      /* Second byte of compressed file */
#define BIT_MASK 0x1f           /* Mask for 'number of compression bits' */
         /* Masks 0x20 and 0x40 are free.  */
         /* I think 0x20 should mean that there is */
         /* a fourth header byte (for expansion).  */
#define BLOCK_MODE 0x80         /* Block compression if table is full and */
         /* compression rate is dropping flush tables */

   /* the next two codes should not be changed lightly, as they must not */
   /* lie within the contiguous general code space.  */
#define FIRST 257               /* first free entry */
#define CLEAR 256               /* table clear output code */

#define INIT_BITS 9             /* initial number of bits/code */

#ifndef SACREDMEM
 /*
  * SACREDMEM is the amount of physical memory saved for others; compress
  * will hog the rest.
  */
#  define SACREDMEM 0
#endif

#ifndef USERMEM
 /*
  * Set USERMEM to the maximum amount of physical user memory available
  * in bytes.  USERMEM is used to determine the maximum BITS that can be used
  * for compression.
  */
#  define USERMEM  450000       /* default user memory */
#endif

#ifndef BYTEORDER
#  define BYTEORDER 0000
#endif

#ifndef NOALLIGN
#  define NOALLIGN 0
#endif

/*
 * machine variants which require cc -Dmachine:  pdp11, z8000, DOS
 */

#ifdef interdata                /* Perkin-Elmer */
#  define SIGNED_COMPARE_SLOW   /* signed compare is slower than unsigned */
#endif

#ifdef pdp11                    /* PDP11: don't forget to compile with -i */
#  define BITS   12             /* max bits/code for 16-bit machine */
#  define NO_UCHAR              /* also if "unsigned char" functions as signed
                                 * char */
#endif                                 /* pdp11 */

#ifdef z8000                    /* Z8000: */
#  define BITS  12              /* 16-bits processor max 12 bits */
#  undef vax                    /* weird preprocessor */
#endif                                 /* z8000 */

#ifdef DOS                      /* PC/XT/AT (8088) processor */
#  define BITS   16             /* 16-bits processor max 12 bits */
#  if BITS == 16
#    define MAXSEG_64K
#  endif
#  undef BYTEORDER
#  define BYTEORDER  4321
#  undef NOALLIGN
#  define NOALLIGN 1
#endif                                 /* DOS */

#ifndef O_BINARY
#  define O_BINARY 0            /* System has no binary mode */
#endif

#ifdef M_XENIX                  /* Stupid compiler can't handle arrays with */
#  if BITS == 16                /* more than 65535 bytes - so we fake it */
#    define MAXSEG_64K
#  else
#    if BITS > 13               /* Code only handles BITS = 12, 13, or 16 */
#      define BITS 13
#    endif
#  endif
#endif

#ifndef BITS                    /* General processor calculate BITS */
#  if USERMEM >= (800000+SACREDMEM)
#    define FAST
#  else
#    if USERMEM >= (433484+SACREDMEM)
#      define BITS 16
#    else
#      if USERMEM >= (229600+SACREDMEM)
#        define BITS 15
#      else
#        if USERMEM >= (127536+SACREDMEM)
#          define BITS 14
#        else
#          if USERMEM >= (73464+SACREDMEM)
#            define BITS 13
#          else
#            define BITS 12
#          endif
#        endif
#      endif
#    endif
#  endif
#endif                                 /* BITS */

#ifdef FAST
#  define HBITS  17             /* 50% occupancy */
#  define HSIZE    (1<<HBITS)
#  define HMASK    (HSIZE-1)
#  define HPRIME   9941
#  define BITS     16
#  undef MAXSEG_64K
#else
#  if BITS == 16
#    define HSIZE 69001         /* 95% occupancy */
#  endif
#  if BITS == 15
#    define HSIZE 35023         /* 94% occupancy */
#  endif
#  if BITS == 14
#    define HSIZE 18013         /* 91% occupancy */
#  endif
#  if BITS == 13
#    define HSIZE 9001          /* 91% occupancy */
#  endif
#  if BITS <= 12
#    define HSIZE 5003          /* 80% occupancy */
#  endif
#endif

#define CHECK_GAP 10000

typedef long int code_int;

#ifdef SIGNED_COMPARE_SLOW
typedef unsigned long int count_int;
typedef unsigned short int count_short;
typedef unsigned long int cmp_code_int; /* Cast to make compare faster */
#else
typedef long int count_int;
typedef long int cmp_code_int;
#endif

typedef unsigned char char_type;

#define ARGVAL() (*++(*argv) || (--argc && *++argv))

#define MAXCODE(n) (1L << (n))

union bytes
  {
    long word;
    struct
      {
#if BYTEORDER == 4321
        char_type b1;
        char_type b2;
        char_type b3;
        char_type b4;
#else
#  if BYTEORDER == 1234
        char_type b4;
        char_type b3;
        char_type b2;
        char_type b1;
#  else
#    undef BYTEORDER
        int dummy;
#  endif
#endif
      } bytes;
  };
#if BYTEORDER == 4321 && NOALLIGN == 1
#  define output(b,o,c,n) {             \
       *(long *)&((b)[(o)>>3]) |= ((long)(c))<<((o)&0x7);\
       (o) += (n);          \
      }
#else
#  ifdef BYTEORDER
#    define output(b,o,c,n) { char_type *p = &(b)[(o)>>3];    \
       union bytes i;         \
       i.word = ((long)(c))<<((o)&0x7);    \
       p[0] |= i.bytes.b1;        \
       p[1] |= i.bytes.b2;        \
       p[2] |= i.bytes.b3;        \
       (o) += (n);          \
      }
#  else
#    define output(b,o,c,n) { char_type *p = &(b)[(o)>>3];    \
       long   i = ((long)(c))<<((o)&0x7); \
       p[0] |= (char_type)(i);       \
       p[1] |= (char_type)(i>>8);      \
       p[2] |= (char_type)(i>>16);      \
       (o) += (n);          \
      }
#  endif
#endif
#if BYTEORDER == 4321 && NOALLIGN == 1
#  define input(b,o,c,n,m){             \
       (c) = (*(long *)(&(b)[(o)>>3])>>((o)&0x7))&(m); \
       (o) += (n);          \
      }
#else
#  define input(b,o,c,n,m){ char_type   *p = &(b)[(o)>>3];   \
       (c) = ((((long)(p[0]))|((long)(p[1])<<8)|  \
          ((long)(p[2])<<16))>>((o)&0x7))&(m); \
       (o) += (n);          \
      }
#endif

char *progname;                 /* Program name */
int silent = 0;                 /* don't tell me about errors */
int quiet = 1;                  /* don't tell me about compression */
int do_decomp = 0;              /* Decompress mode */
int force = 0;                  /* Force overwrite of files and links */
int nomagic = 0;                /* Use a 3-byte magic number header, */
         /* unless old file */
int block_mode = BLOCK_MODE;    /* Block compress mode -C compatible with 2.0 */
int maxbits = BITS;             /* user settable max # bits/code */
int zcat_flg = 0;               /* Write output on stdout, suppress messages */
int recursive = 0;              /* compress directories */
int exit_code = -1;             /* Exitcode of compress (-1 no file compressed)
                                 */

char_type inbuf[IBUFSIZ + 64];  /* Input buffer */
char_type outbuf[OBUFSIZ + 2048];       /* Output buffer */

struct stat infstat;            /* Input file status */
char *ifname;                   /* Input filename */
int remove_ofname = 0;          /* Remove output file on a error */
char ofname[MAXPATHLEN];        /* Output filename */
int fgnd_flag = 0;              /* Running in background (SIGINT=SIGIGN) */

long bytes_in;                  /* Total number of byte from input */
long bytes_out;                 /* Total number of byte to output */

/*
 * To save much memory, we overlay the table used by compress() with those
 * used by decompress().  The tab_prefix table is the same size and type
 * as the codetab.  The tab_suffix table needs 2**BITS characters.  We
 * get this from the beginning of htab.  The output stack uses the rest
 * of htab, and contains characters.  There is plenty of room for any
 * possible stack (stack used to be 8000 characters).
 */
#ifdef MAXSEG_64K
count_int htab0[8192];
count_int htab1[8192];
count_int htab2[8192];
count_int htab3[8192];
count_int htab4[8192];
count_int htab5[8192];
count_int htab6[8192];
count_int htab7[8192];
count_int htab8[HSIZE - 65536];
count_int *htab[9] =
  { htab0, htab1, htab2, htab3, htab4, htab5, htab6, htab7, htab8 };

unsigned short code0tab[16384];
unsigned short code1tab[16384];
unsigned short code2tab[16384];
unsigned short code3tab[16384];
unsigned short code4tab[16384];
unsigned short *codetab[5] =
  { code0tab, code1tab, code2tab, code3tab, code4tab };

#  define htabof(i)   (htab[(i) >> 13][(i) & 0x1fff])
#  define codetabof(i)  (codetab[(i) >> 14][(i) & 0x3fff])
#  define tab_prefixof(i)  codetabof(i)
#  define tab_suffixof(i)  ((char_type *)htab[(i)>>15])[(i) & 0x7fff]
#  define de_stack   ((char_type *)(&htab2[8191]))
void clear_htab()
{
  memset(htab0, -1, sizeof(htab0));
  memset(htab1, -1, sizeof(htab1));
  memset(htab2, -1, sizeof(htab2));
  memset(htab3, -1, sizeof(htab3));
  memset(htab4, -1, sizeof(htab4));
  memset(htab5, -1, sizeof(htab5));
  memset(htab6, -1, sizeof(htab6));
  memset(htab7, -1, sizeof(htab7));
  memset(htab8, -1, sizeof(htab8));
}

#  define clear_tab_prefixof() memset(code0tab, 0, 256);
#else                                  /* Normal machine */
count_int htab[HSIZE];
unsigned short codetab[HSIZE];

#  define htabof(i)    htab[i]
#  define codetabof(i)   codetab[i]
#  define tab_prefixof(i)   codetabof(i)
#  define tab_suffixof(i)   ((char_type *)(htab))[i]
#  define de_stack    ((char_type *)&(htab[HSIZE-1]))
#  define clear_htab()   memset(htab, -1, sizeof(htab))
#  define clear_tab_prefixof() memset(codetab, 0, 256);

#  ifdef FAST
int primetab[256] =             /* Special secudary hash table.  */
{
  1013, -1061, 1109, -1181, 1231, -1291, 1361, -1429,
  1481, -1531, 1583, -1627, 1699, -1759, 1831, -1889,
  1973, -2017, 2083, -2137, 2213, -2273, 2339, -2383,
  2441, -2531, 2593, -2663, 2707, -2753, 2819, -2887,
  2957, -3023, 3089, -3181, 3251, -3313, 3361, -3449,
  3511, -3557, 3617, -3677, 3739, -3821, 3881, -3931,
  4013, -4079, 4139, -4219, 4271, -4349, 4423, -4493,
  4561, -4639, 4691, -4783, 4831, -4931, 4973, -5023,
  5101, -5179, 5261, -5333, 5413, -5471, 5521, -5591,
  5659, -5737, 5807, -5857, 5923, -6029, 6089, -6151,
  6221, -6287, 6343, -6397, 6491, -6571, 6659, -6709,
  6791, -6857, 6917, -6983, 7043, -7129, 7213, -7297,
  7369, -7477, 7529, -7577, 7643, -7703, 7789, -7873,
  7933, -8017, 8093, -8171, 8237, -8297, 8387, -8461,
  8543, -8627, 8689, -8741, 8819, -8867, 8963, -9029,
  9109, -9181, 9241, -9323, 9397, -9439, 9511, -9613,
  9677, -9743, 9811, -9871, 9941, -10061, 10111, -10177,
  10259, -10321, 10399, -10477, 10567, -10639, 10711, -10789,
  10867, -10949, 11047, -11113, 11173, -11261, 11329, -11423,
  11491, -11587, 11681, -11777, 11827, -11903, 11959, -12041,
  12109, -12197, 12263, -12343, 12413, -12487, 12541, -12611,
  12671, -12757, 12829, -12917, 12979, -13043, 13127, -13187,
  13291, -13367, 13451, -13523, 13619, -13691, 13751, -13829,
  13901, -13967, 14057, -14153, 14249, -14341, 14419, -14489,
  14557, -14633, 14717, -14767, 14831, -14897, 14983, -15083,
  15149, -15233, 15289, -15359, 15427, -15497, 15583, -15649,
  15733, -15791, 15881, -15937, 16057, -16097, 16189, -16267,
  16363, -16447, 16529, -16619, 16691, -16763, 16879, -16937,
  17021, -17093, 17183, -17257, 17341, -17401, 17477, -17551,
  17623, -17713, 17791, -17891, 17957, -18041, 18097, -18169,
  18233, -18307, 18379, -18451, 18523, -18637, 18731, -18803,
  18919, -19031, 19121, -19211, 19273, -19381, 19429, -19477
};
#  endif

/*
 * compress fdin to fdout
 *
 * Algorithm:  use open addressing double hashing (no chaining) on the
 * prefix code / next character combination.  We do a variant of Knuth's
 * algorithm D (vol. 3, sec. 6.4) along with G. Knott's relatively-prime
 * secondary probe.  Here, the modular division first probe is gives way
 * to a faster exclusive-or manipulation.  Also do block compression with
 * an adaptive reset, whereby the code table is cleared when the compression
 * ratio decreases, but after the table fills.  The variable-length output
 * codes are re-sized at this point, and a special CLEAR code is generated
 * for the decompressor.  Late addition:  construct the table according to
 * file size for noticeable speed improvement on small files.  Please direct
 * questions about this implementation to ames!jaw.
 */
void compress(int fdin, int fdout)
{
  long hp;
  int rpos;
  long fc;
  int outbits;
  int rlop;
  int rsize;
  int stcode;
  code_int free_ent;
  int boff;
  int n_bits;
  int ratio;
  long checkpoint;
  code_int extcode;
  union
    {
      long code;
      struct
        {
          char_type c;
          unsigned short ent;
        } e;
    } fcode;

  ratio = 0;
  checkpoint = CHECK_GAP;
  extcode = MAXCODE(n_bits = INIT_BITS) + 1;
  stcode = 1;
  free_ent = FIRST;

  memset(outbuf, 0, sizeof(outbuf));
  bytes_out = 0;
  bytes_in = 0;
  outbuf[0] = MAGIC_1;
  outbuf[1] = MAGIC_2;
  outbuf[2] = (char)(maxbits | block_mode);
  boff = outbits = (3 << 3);
  fcode.code = 0;

  clear_htab();

  while ((rsize = read(fdin, inbuf, IBUFSIZ)) > 0)
    {
      if (bytes_in == 0)
        {
          fcode.e.ent = inbuf[0];
          rpos = 1;
        }
      else
        rpos = 0;

      rlop = 0;

      do
        {
          if (free_ent >= extcode && fcode.e.ent < FIRST)
            {
              if (n_bits < maxbits)
                {
                  boff = outbits = (outbits - 1) + ((n_bits << 3) -
                                                    ((outbits - boff - 1 +
                                                      (n_bits << 3)) %
                                                     (n_bits << 3)));
                  if (++n_bits < maxbits)
                    extcode = MAXCODE(n_bits) + 1;
                  else
                    extcode = MAXCODE(n_bits);
                }
              else
                {
                  extcode = MAXCODE(16) + OBUFSIZ;
                  stcode = 0;
                }
            }

          if (!stcode && bytes_in >= checkpoint && fcode.e.ent < FIRST)
            {
              long int rat;

              checkpoint = bytes_in + CHECK_GAP;

              if (bytes_in > 0x007fffff)
                {               /* shift will overflow */
                  rat = (bytes_out + (outbits >> 3)) >> 8;

                  if (rat == 0) /* Don't divide by zero */
                    rat = 0x7fffffff;
                  else
                    rat = bytes_in / rat;
                }
              else
                rat = (bytes_in << 8) / (bytes_out + (outbits >> 3));   /* 8
                                                                         * fractional
                                                                         * bits
                                                                         */
              if (rat >= ratio)
                ratio = (int)rat;
              else
                {
                  ratio = 0;
                  clear_htab();
                  output(outbuf, outbits, CLEAR, n_bits);
                  boff = outbits = (outbits - 1) + ((n_bits << 3) -
                                                    ((outbits - boff - 1 +
                                                      (n_bits << 3)) %
                                                     (n_bits << 3)));
                  extcode = MAXCODE(n_bits = INIT_BITS) + 1;
                  free_ent = FIRST;
                  stcode = 1;
                }
            }

          if (outbits >= (OBUFSIZ << 3))
            {
              if (write(fdout, outbuf, OBUFSIZ) != OBUFSIZ)
                write_error();

              outbits -= (OBUFSIZ << 3);
              boff = -(((OBUFSIZ << 3) - boff) % (n_bits << 3));
              bytes_out += OBUFSIZ;

              memcpy(outbuf, outbuf + OBUFSIZ, (outbits >> 3) + 1);
              memset(outbuf + (outbits >> 3) + 1, '\0', OBUFSIZ);
            }

          {
            int i;

            i = rsize - rlop;

            if ((code_int) i > extcode - free_ent)
              i = (int)(extcode - free_ent);
            if (i > ((sizeof(outbuf) - 32) * 8 - outbits) / n_bits)
              i = ((sizeof(outbuf) - 32) * 8 - outbits) / n_bits;

            if (!stcode && (long)i > checkpoint - bytes_in)
              i = (int)(checkpoint - bytes_in);

            rlop += i;
            bytes_in += i;
          }

          goto next;
        hfound:fcode.e.ent = codetabof(hp);
        next:if (rpos >= rlop)
            goto endlop;
        next2:fcode.e.c = inbuf[rpos++];
#  ifndef FAST
          {
            code_int i;
            fc = fcode.code;

            hp = (((long)(fcode.e.c)) << (BITS - 8)) ^ (long)(fcode.e.ent);

            if ((i = htabof(hp)) == fc)
              goto hfound;

            if (i != -1)
              {
                long disp;

                disp = (HSIZE - hp) - 1;        /* secondary hash (after G.
                                                 * Knott) */

                do
                  {
                    if ((hp -= disp) < 0)
                      hp += HSIZE;

                    if ((i = htabof(hp)) == fc)
                      goto hfound;
                  }
                while (i != -1);
              }
          }
#  else
          {
            long i;
            long p;
            fc = fcode.code;

            hp = ((((long)(fcode.e.c)) << (HBITS - 8)) ^ (long)(fcode.e.ent));

            if ((i = htabof(hp)) == fc)
              goto hfound;
            if (i == -1)
              goto out;

            p = primetab[fcode.e.c];
          lookup:hp = (hp + p) & HMASK;
            if ((i = htabof(hp)) == fc)
              goto hfound;
            if (i == -1)
              goto out;
            hp = (hp + p) & HMASK;
            if ((i = htabof(hp)) == fc)
              goto hfound;
            if (i == -1)
              goto out;
            hp = (hp + p) & HMASK;
            if ((i = htabof(hp)) == fc)
              goto hfound;
            if (i == -1)
              goto out;
            goto lookup;
          }
        out:;
#  endif
          output(outbuf, outbits, fcode.e.ent, n_bits);

          {
            long fc = fcode.code;
            fcode.e.ent = fcode.e.c;

            if (stcode)
              {
                codetabof(hp) = (unsigned short)free_ent++;
                htabof(hp) = fc;
              }
          }

          goto next;

        endlop:if (fcode.e.ent >= FIRST && rpos < rsize)
            goto next2;

          if (rpos > rlop)
            {
              bytes_in += rpos - rlop;
              rlop = rpos;
            }
        }
      while (rlop < rsize);
    }

  if (rsize < 0)
    read_error();

  if (bytes_in > 0)
    output(outbuf, outbits, fcode.e.ent, n_bits);

  if (write(fdout, outbuf, (outbits + 7) >> 3) != (outbits + 7) >> 3)
    write_error();

  bytes_out += (outbits + 7) >> 3;

  return;
}

/*
 * Decompress stdin to stdout.  This routine adapts to the codes in the
 * file building the "string" table on-the-fly; requiring no table to
 * be stored in the compressed file.  The tables used herein are shared
 * with those of the compress() routine.  See the definitions above.
 */

void decompress(int fdin, int fdout)
     int fdin;
     int fdout;
{
  char_type *stackp;
  code_int code;
  int finchar;
  code_int oldcode;
  code_int incode;
  int inbits;
  int posbits;
  int outpos;
  int insize;
  int bitmask;
  code_int free_ent;
  code_int maxcode;
  code_int maxmaxcode;
  int n_bits;
  int rsize;

  bytes_in = 0;
  bytes_out = 0;
  insize = 0;

  while (insize < 3 && (rsize = read(fdin, inbuf + insize, IBUFSIZ)) > 0)
    insize += rsize;

  if (insize < 3 || inbuf[0] != MAGIC_1 || inbuf[1] != MAGIC_2)
    {
      if (rsize < 0)
        read_error();

      if (insize > 0)
        {
          fprintf(stderr, "%s: not in compressed format\n",
                  (ifname[0] != '\0' ? ifname : "stdin"));
          exit_code = 1;
        }

      return;
    }

  maxbits = inbuf[2] & BIT_MASK;
  block_mode = inbuf[2] & BLOCK_MODE;

  if (maxbits > BITS)
    {
      fprintf(stderr,
              "%s: compressed with %d bits, can only handle %d bits\n",
              (*ifname != '\0' ? ifname : "stdin"), maxbits, BITS);
      exit_code = 4;
      return;
    }

  maxmaxcode = MAXCODE(maxbits);

  bytes_in = insize;
  maxcode = MAXCODE(n_bits = INIT_BITS) - 1;
  bitmask = (1 << n_bits) - 1;
  oldcode = -1;
  finchar = 0;
  outpos = 0;
  posbits = 3 << 3;

  free_ent = ((block_mode) ? FIRST : 256);

  clear_tab_prefixof();         /* As above, initialize the first 256 entries
                                 * in the table. */

  for (code = 255; code >= 0; --code)
    tab_suffixof(code) = (char_type) code;

  do
    {
    resetbuf:;
      {
        int i;
        int e;
        int o;

        o = posbits >> 3;
        e = o <= insize ? insize - o : 0;

        for (i = 0; i < e; ++i)
          inbuf[i] = inbuf[i + o];

        insize = e;
        posbits = 0;
      }

      if (insize < sizeof(inbuf) - IBUFSIZ)
        {
          if ((rsize = read(fdin, inbuf + insize, IBUFSIZ)) < 0)
            read_error();

          insize += rsize;
        }

      inbits = ((rsize > 0) ? (insize - insize % n_bits) << 3 :
                (insize << 3) - (n_bits - 1));

      while (inbits > posbits)
        {
          if (free_ent > maxcode)
            {
              posbits = ((posbits - 1) + ((n_bits << 3) -
                                          (posbits - 1 +
                                           (n_bits << 3)) % (n_bits << 3)));

              ++n_bits;
              if (n_bits == maxbits)
                maxcode = maxmaxcode;
              else
                maxcode = MAXCODE(n_bits) - 1;

              bitmask = (1 << n_bits) - 1;
              goto resetbuf;
            }

          input(inbuf, posbits, code, n_bits, bitmask);

          if (oldcode == -1)
            {
              if (code >= 256)
                {
                  fprintf(stderr, "oldcode:-1 code:%i\n", (int)(code));
                  fprintf(stderr, "uncompress: corrupt input\n");
                  abort_compress();
                }
              outbuf[outpos++] = (char_type) (finchar = (int)(oldcode = code));
              continue;
            }

          if (code == CLEAR && block_mode)
            {
              clear_tab_prefixof();
              free_ent = FIRST - 1;
              posbits = ((posbits - 1) + ((n_bits << 3) -
                                          (posbits - 1 +
                                           (n_bits << 3)) % (n_bits << 3)));
              maxcode = MAXCODE(n_bits = INIT_BITS) - 1;
              bitmask = (1 << n_bits) - 1;
              goto resetbuf;
            }

          incode = code;
          stackp = de_stack;

          if (code >= free_ent) /* Special case for KwKwK string. */
            {
              if (code > free_ent)
                {
                  char_type *p;

                  posbits -= n_bits;
                  p = &inbuf[posbits >> 3];

                  fprintf(stderr,
                          "insize:%d posbits:%d inbuf:%02X %02X %02X %02X %02X (%d)\n",
                          insize, posbits, p[-1], p[0], p[1], p[2], p[3],
                          (posbits & 07));
                  fprintf(stderr, "uncompress: corrupt input\n");
                  abort_compress();
                }

              *--stackp = (char_type) finchar;
              code = oldcode;
            }

          while ((cmp_code_int) code >= (cmp_code_int) 256)
            {                   /* Generate output characters in reverse order */
              *--stackp = tab_suffixof(code);
              code = tab_prefixof(code);
            }

          *--stackp = (char_type) (finchar = tab_suffixof(code));

          /* And put them out in forward order */

          {
            int i;

            if (outpos + (i = (de_stack - stackp)) >= OBUFSIZ)
              {
                do
                  {
                    if (i > OBUFSIZ - outpos)
                      i = OBUFSIZ - outpos;

                    if (i > 0)
                      {
                        memcpy(outbuf + outpos, stackp, i);
                        outpos += i;
                      }

                    if (outpos >= OBUFSIZ)
                      {
                        if (write(fdout, outbuf, outpos) != outpos)
                          write_error();

                        outpos = 0;
                      }
                    stackp += i;
                  }
                while ((i = (de_stack - stackp)) > 0);
              }
            else
              {
                memcpy(outbuf + outpos, stackp, i);
                outpos += i;
              }
          }

          if ((code = free_ent) < maxmaxcode)   /* Generate the new entry. */
            {
              tab_prefixof(code) = (unsigned short)oldcode;
              tab_suffixof(code) = (char_type) finchar;
              free_ent = code + 1;
            }

          oldcode = incode;     /* Remember previous code. */
        }

      bytes_in += rsize;
    }
  while (rsize > 0);

  if (outpos > 0 && write(fdout, outbuf, outpos) != outpos)
    write_error();
}

void read_error()
{
  fprintf(stderr, "\nread error on");
  perror((ifname[0] != '\0') ? ifname : "stdin");
  abort_compress();
}

void write_error()
{
  fprintf(stderr, "\nwrite error on");
  perror((ofname[0] != '\0') ? ofname : "stdout");
  abort_compress();
}

void abort_compress()
{
  if (remove_ofname)
    unlink(ofname);

  exit(1);
}

void prratio(stream, num, den)
     FILE *stream;
     long int num;
     long int den;
{
  int q;

  if (den > 0)
    {
      if (num > 214748L)
        q = (int)(num / (den / 10000L));        /* 2147483647/10000 */
      else
        q = (int)(10000L * num / den);  /* Long calculations, though */
    }
  else
    q = 10000;

  if (q < 0)
    {
      putc('-', stream);
      q = -q;
    }

  fprintf(stream, "%d.%02d%%", q / 100, q % 100);
}
