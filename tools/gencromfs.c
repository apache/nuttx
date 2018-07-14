/****************************************************************************
 * tools/gencromfs.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The function lzf_compress() comes from the file lzf_c.c (wich substantial
 * modification):
 *
 *   Copyright (c) 2000-2010 Marc Alexander Lehmann <schmorp@schmorp.de>
 *
 * Which has a compatible BSD license and included here under the NuttX BSD
 * license:
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

#define _GNU_SOURCE 1
#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HOST_BIGENDIAN or TGT_BIGENDIAN may be defined on the command line to
 * adapt to different host/target combinations.  Otherwise, both are assumed
 * to be the same endian-ness.
 */

#undef HOST_TGTSWAP

#if (defined(HOST_BIGENDIAN) && !defined(TGT_BIGENDIAN)) || \
    (!defined(HOST_BIGENDIAN) && defined(TGT_BIGENDIAN))
#  define HOST_TGTSWAP 1
#endif

/* mkstemp() has been giving me errors on Cygwin */

#undef USE_MKSTEMP

#define TMP_NAMLEN         32         /* Actually only 22 */
#ifdef USE_MKSTEMP
#  define TMP_NAME         "/tmp/gencromfs-XXXXXX"
#else
#  define TMP_NAME         "/tmp/gencromfs-%06u"
#endif

#define NUTTX_IXOTH        (1 << 0)   /* Bits 0-2: Permissions for others: RWX */
#define NUTTX_IWOTH        (1 << 1)
#define NUTTX_IROTH        (1 << 2)

#define NUTTX_IRXOTH       (NUTTX_IROTH | NUTTX_IXOTH)

#define NUTTX_IXGRP        (1 << 3)   /* Bits 3-5: Group permissions: RWX */
#define NUTTX_IWGRP        (1 << 4)
#define NUTTX_IRGRP        (1 << 5)

#define NUTTX_IRXGRP       (NUTTX_IRGRP | NUTTX_IXGRP)

#define NUTTX_IXUSR        (1 << 6)   /* Bits 6-8: Owner permissions: RWX */
#define NUTTX_IWUSR        (1 << 7)
#define NUTTX_IRUSR        (1 << 8)

#define NUTTX_IRXUSR       (NUTTX_IRUSR | NUTTX_IXUSR)

#define NUTTX_IFDIR        (2 << 11)
#define NUTTX_IFREG        (4 << 11)

#define NUTTX_IFLNK        (1 << 15)  /* Bit 15: Symbolic link */

#define DIR_MODEFLAGS      (NUTTX_IFDIR | NUTTX_IRXUSR | NUTTX_IRXGRP | NUTTX_IRXOTH)
#define DIRLINK_MODEFLAGS  (NUTTX_IFLNK | DIR_MODEFLAGS)
#define FILE_MODEFLAGS     (NUTTX_IFREG | NUTTX_IRUSR | NUTTX_IRGRP | NUTTX_IROTH)

#define CROMFS_MAGIC       0x4d4f5243
#define CROMFS_BLOCKSIZE   512

#define LZF_BUFSIZE        512
#define LZF_HLOG           13
#define LZF_HSIZE          (1 << LZF_HLOG)

#define LZF_TYPE0_HDR      0
#define LZF_TYPE1_HDR      1
#define LZF_TYPE0_HDR_SIZE 5
#define LZF_TYPE1_HDR_SIZE 7

#define LZF_FRST(p)        (((p[0]) << 8) | p[1])
#define LZF_NEXT(v,p)      (((v) << 8) | p[2])
#define LZF_NDX(h)         ((((h ^ (h << 5)) >> (3*8 - LZF_HLOG)) - h*5) & (LZF_HSIZE - 1))

#define LZF_MAX_LIT        (1 <<  5)
#define LZF_MAX_OFF        (1 << LZF_HLOG)
#define LZF_MAX_REF        ((1 << 8) + (1 << 3))

#define HEX_PER_BREAK      8
#define HEX_PER_LINE       16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Maximum size of an offset.  This should normally be size_t since this is
 * an in-memory file system.  However, size_t is 32-bits on most 32-bit
 * target machines but 64-bits on 64-host machines.  We restrict offsets to
 * 32-bits for commonality (limiting the size of the CROMFS image to 4Gb)
 *
 * Similarly, the NuttX mode_t is only 16-bits so uint16_t is explicitly used
 * for NuttX file modes.
 */

/* CROMFS structures */

struct cromfs_volume_s
{
  uint32_t cv_magic;      /* Must be first.  Must be CROMFS_MAGIC */
  uint16_t cv_nnodes;     /* Total number of nodes in-use */
  uint16_t cv_nblocks;    /* Total number of data blocks in-use */
  uint32_t cv_root;       /* Offset to the first node in the root file system */
  uint32_t cv_fsize;      /* Size of the compressed file system image */
  uint32_t cv_bsize;      /* Optimal block size for transfers */
};

struct cromfs_node_s
{
  uint16_t cn_mode;       /* File type, attributes, and access mode bits */
  uint16_t cn_pad;        /* Not used */
  uint32_t cn_name;       /* Offset from the beginning of the volume header to the
                           * node name string.  NUL-terminated. */
  uint32_t cn_size;       /* Size of the uncompressed data (in bytes) */
  uint32_t cn_peer;       /* Offset to next node in this directory (for readdir()) */
  union
  {
    uint32_t cn_child;    /* Offset to first node in sub-directory (directories only) */
    uint32_t cn_link;     /* Offset to an arbitrary node (for hard link) */
    uint32_t cn_blocks;   /* Offset to first block of compressed data (for read) */
  } u;
};

/* LZF headers */

struct lzf_header_s       /* Common data header */
{
  uint8_t lzf_magic[2];   /* [0]='Z', [1]='V' */
  uint8_t lzf_type;       /* LZF_TYPE0_HDR or LZF_TYPE1_HDR */
};

struct lzf_type0_header_s /* Uncompressed data header */
{
  uint8_t lzf_magic[2];   /* [0]='Z', [1]='V' */
  uint8_t lzf_type;       /* LZF_TYPE0_HDR */
  uint8_t lzf_len[2];     /* Data length (big-endian) */
};

struct lzf_type1_header_s /* Compressed data header */
{
  uint8_t lzf_magic[2];   /* [0]='Z', [1]='V' */
  uint8_t lzf_type;       /* LZF_TYPE1_HDR */
  uint8_t lzf_clen[2];    /* Compressed data length (big-endian) */
  uint8_t lzf_ulen[2];    /* Uncompressed data length (big-endian) */
};

/* LZF data buffer */

union lzf_result_u
{
  struct
  {
    uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
    uint8_t lzf_type;         /* LZF_TYPE0_HDR or LZF_TYPE1_HDR */
  } cmn;                      /* Common data header */
  struct
  {
    uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
    uint8_t lzf_type;         /* LZF_TYPE0_HDR */
    uint8_t lzf_len[2];       /* Data length (big-endian) */
    uint8_t lzf_buffer[LZF_BUFSIZE];
  } uncompressed;             /* Uncompressed data header */
  struct
  {
    uint8_t lzf_magic[2];     /* [0]='Z', [1]='V' */
    uint8_t lzf_type;         /* LZF_TYPE1_HDR */
    uint8_t lzf_clen[2];      /* Compressed data length (big-endian) */
    uint8_t lzf_ulen[2];      /* Uncompressed data length (big-endian) */
    uint8_t lzf_buffer[LZF_BUFSIZE + 16];
  } compressed;
};

/* LZF hash table */

static uint8_t *g_lzf_hashtab[LZF_HSIZE];

/* Type of the callback from traverse_directory() */

typedef int (*traversal_callback_t)(const char *dirpath, const char *name,
                                    void *arg, bool lastentry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_progname;       /* Name of this program */
static char *g_dirname;        /* Source directory path */
static char *g_outname;        /* Output file path */

static FILE *g_outstream;      /* Main output stream */
static FILE *g_tmpstream;      /* Temporary file output stream */

static const char g_delim[] =
  "*************************************************************************"
  "***********************";

static uint32_t g_offset;        /* Current image offset */
static uint32_t g_diroffset;     /* Offset for '.' */
static uint32_t g_parent_offset; /* Offset for '..' */

static unsigned int g_nnodes;  /* Number of nodes generated */
static unsigned int g_nblocks; /* Number of blocks of data generated */
static unsigned int g_nhex;    /* Number of hex characters on output line */
#ifndef USE_MKSTEMP
static unsigned int g_ntmps;   /* Number temporary files */
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_usage(void);
static void verify_directory(void);
static void verify_outfile(void);
static void init_outfile(void);
static FILE *open_tmpfile(void);
#ifndef USE_MKSTEMP
static void unlink_tmpfiles(void);
#endif
static void append_tmpfile(FILE *dest, FILE *src);
static void dump_hexbuffer(FILE *stream, const void *buffer, unsigned int nbytes);
static void dump_nextline(FILE *stream);
static size_t lzf_compress(const uint8_t *inbuffer, unsigned int inlen,
                           union lzf_result_u *result);
static uint16_t get_mode(mode_t mode);
#ifdef HOST_TGTSWAP
static inline uint16_t tgt_uint16(uint16_t a);
static inline uint32_t tgt_uint32(uint32_t a);
#  define TGT_UINT16(a) tgt_uint16(a)
#  define TGT_UINT32(a) tgt_uint32(a)
#else
#  define TGT_UINT16(a) (a)
#  define TGT_UINT32(a) (a)
#endif
static void gen_dirlink(const char *name, uint32_t tgtoffs, bool dirempty);
static void gen_directory(const char *path, const char *name, mode_t mode,
                          bool lastentry);
static void gen_file(const char *path, const char *name, mode_t mode,
                          bool lastentry);
static int  dir_notempty(const char *dirpath, const char *name,
                         void *arg, bool lastentry);
static int  process_direntry(const char *dirpath, const char *name,
                             void *arg, bool lastentry);
static int  traverse_directory(const char *dirpath,
                               traversal_callback_t callback, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "USAGE: %s <dir-path> <out-file>\n", g_progname);
  exit(1);
}

static void verify_directory(void)
{
  struct stat buf;
  int len;
  int ret;

  /* Trim any trailing '/' characters from the directory path. */

  len = strlen(g_dirname);
  while (len > 1 && g_dirname[len-1] == '/')
    {
      g_dirname[len-1] = '\0';
      len--;
    }

  if (len < 1)
    {
      fprintf(stderr, "ERROR: Source <dir-path> %s is invalid\n",
              g_dirname);
      show_usage();
    }

  /* stat the source directory containing the file system image */

  ret = stat(g_dirname, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode == ENOENT)
        {
          fprintf(stderr, "ERROR: Source <dir-path> %s does not exist\n",
                  g_dirname);
        }
      else
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 g_dirname, strerror(errcode));
        }

      show_usage();
    }

  /* Verify that the source is, indeed, a directory */

  else if (!S_ISDIR(buf.st_mode))
    {
      fprintf(stderr, "ERROR: Source <dir-path> %s is not a directory\n",
              g_dirname);
    }
}

static void verify_outfile(void)
{
  struct stat buf;
  int ret;

  /* stat the destination file */

  ret = stat(g_outname, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode != ENOENT)
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 g_outname, strerror(errcode));
          show_usage();
        }
    }

  /* Something exists at this path.  Verify that the destination is a regular file */

  else if (!S_ISREG(buf.st_mode))
    {
      fprintf(stderr, "ERROR: Destination <out-file> %s exists\n",
              g_outname);
      show_usage();
    }
  else
    {
      printf("Existing file %s will be replaced\n", g_outname);
    }
}

static void init_outfile(void)
{
  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * %s\n", g_outname);
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " *   Copyright (C) 2018 Gregory Nutt. All rights reserved.\n");
  fprintf(g_outstream, " *   Author: Gregory Nutt <gnutt@nuttx.org>\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * Redistribution and use in source and binary forms, with or without\n");
  fprintf(g_outstream, " * modification, are permitted provided that the following conditions\n");
  fprintf(g_outstream, " * are met:\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * 1. Redistributions of source code must retain the above copyright\n");
  fprintf(g_outstream, " *    notice, this list of conditions and the following disclaimer.\n");
  fprintf(g_outstream, " * 2. Redistributions in binary form must reproduce the above copyright\n");
  fprintf(g_outstream, " *    notice, this list of conditions and the following disclaimer in\n");
  fprintf(g_outstream, " *    the documentation and/or other materials provided with the\n");
  fprintf(g_outstream, " *    distribution.\n");
  fprintf(g_outstream, " * 3. Neither the name NuttX nor the names of its contributors may be\n");
  fprintf(g_outstream, " *    used to endorse or promote products derived from this software\n");
  fprintf(g_outstream, " *    without specific prior written permission.\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n");
  fprintf(g_outstream, " * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n");
  fprintf(g_outstream, " * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS\n");
  fprintf(g_outstream, " * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n");
  fprintf(g_outstream, " * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,\n");
  fprintf(g_outstream, " * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,\n");
  fprintf(g_outstream, " * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS\n");
  fprintf(g_outstream, " * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED\n");
  fprintf(g_outstream, " * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT\n");
  fprintf(g_outstream, " * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN\n");
  fprintf(g_outstream, " * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n");
  fprintf(g_outstream, " * POSSIBILITY OF SUCH DAMAGE.\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Included Files\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "#include <nuttx/config.h>\n\n");
  fprintf(g_outstream, "#include <stdint.h>\n\n");

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Private Data\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);
}

static FILE *open_tmpfile(void)
{
  FILE *tmpstream;
#ifdef USE_MKSTEMP
  int fd;

  fd = mkstemp(TMP_NAME);
  if (fd < 0)
    {
      fprintf(stderr, "Failed to create temporary file: %s\n", strerror(errno));
      exit(1);
    }

  tmpstream = fdopen(fd, "w+");
  if (!tmpstream)
    {
      fprintf(stderr, "fdopen for tmp file failed: %s\n", strerror(errno));
      exit(1);
    }

#else
  char tmpname[TMP_NAMLEN];

  snprintf(tmpname, TMP_NAMLEN, TMP_NAME, g_ntmps);
  g_ntmps++;

  tmpstream = fopen(tmpname, "w+");
  if (!tmpstream)
    {
      fprintf(stderr, "fopen for tmp file %s failed: %s\n",
              tmpname, strerror(errno));
      exit(1);
    }
#endif

  return tmpstream;
}

#ifndef USE_MKSTEMP
static void unlink_tmpfiles(void)
{
  char tmpname[TMP_NAMLEN];
  unsigned int i;

  for (i = 0; i < g_ntmps; i++)
    {
      snprintf(tmpname, TMP_NAMLEN, TMP_NAME, i);
      (void)unlink(tmpname);
    }
}
#endif

static void append_tmpfile(FILE *dest, FILE *src)
{
  uint8_t iobuffer[1024];
  size_t nread;

  /* Rewind the source directory to be beginning.  We assume that the dest
   * is already at the end.
   */

  rewind(src);

  /* Then append the source to the destination */

  do
    {
      nread = fread(iobuffer, 1, 1024, src);
      if (nread > 0)
        {
          (void)fwrite(iobuffer, 1, nread, dest);
        }
    }
  while (nread > 0);

  /* We can now close the src temporary file */

  fclose(src);
}

static void dump_hexbuffer(FILE *stream, const void *buffer,
                           unsigned int nbytes)
{
  uint8_t *ptr = (uint8_t *)buffer;

  while (nbytes > 0)
    {
      if (g_nhex == 0 || g_nhex == HEX_PER_BREAK)
        {
          fprintf(stream, " ");
        }

      fprintf(stream, " 0x%02x,", *ptr++);

      if (++g_nhex >= HEX_PER_LINE)
        {
          fprintf(stream, "\n");
          g_nhex = 0;
        }

      nbytes--;
    }
}

static void dump_nextline(FILE *stream)
{
  if (g_nhex > 0)
    {
      fprintf(stream, "\n");
      g_nhex = 0;
    }
}

static size_t lzf_compress(const uint8_t *inbuffer, unsigned int inlen,
                           union lzf_result_u *result)
{
  const uint8_t *inptr  = inbuffer;
        uint8_t *outptr = result->compressed.lzf_buffer;
  const uint8_t *inend  = inptr + inlen;
        uint8_t *outend = outptr + LZF_BUFSIZE;
  const uint8_t *ref;
  uintptr_t off;
  ssize_t cs;
  ssize_t retlen;
  unsigned int hval;
  int lit;

  if (inlen == 0)
    {
      cs = 0;
      goto genhdr;
    }

  memset(g_lzf_hashtab, 0, sizeof(g_lzf_hashtab));
  lit = 0; /* Start run */
  outptr++;

  hval = LZF_FRST(inptr);
  while (inptr < inend - 2)
    {
      uint8_t **hslot;

      hval   = LZF_NEXT(hval, inptr);
      hslot  = &g_lzf_hashtab[LZF_NDX(hval)];
      ref    = *hslot;
      *hslot = (uint8_t *)inptr;

      if (ref < inptr && /* the next test will actually take care of this, but this is faster */
          (off = inptr - ref - 1) < LZF_MAX_OFF &&
          ref > (uint8_t *)inbuffer &&
          ref[2] == inptr[2] &&
          ((ref[1] << 8) | ref[0]) == ((inptr[1] << 8) | inptr[0]))
        {
          /* Match found at *ref++ */

          unsigned int len = 2;
          unsigned int maxlen = inend - inptr - len;
          maxlen = maxlen > LZF_MAX_REF ? LZF_MAX_REF : maxlen;

          /* First a faster conservative test */

          if ((outptr + 3 + 1) >= outend)
            {
              /* Second the exact but rare test */

              if (outptr - !lit + 3 + 1 >= outend)
                {
                  cs = 0;
                  goto genhdr;
                }
            }

          outptr[- lit - 1] = lit - 1; /* Stop run */
          outptr -= !lit;              /* Undo run if length is zero */

          for (;;)
            {
              if (maxlen > 16)
                {
                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }

                  len++;
                  if (ref[len] != inptr[len])
                    {
                      break;
                    }
                }

              do
                {
                  len++;
                }
              while (len < maxlen && ref[len] == inptr[len]);

              break;
            }

          len -= 2; /* len is now #octets - 1 */
          inptr++;

          if (len < 7)
            {
              *outptr++ = (off >> 8) + (len << 5);
            }
          else
            {
              *outptr++ = (off >> 8) + (  7 << 5);
              *outptr++ = len - 7;
            }

          *outptr++ = off;

          lit = 0; outptr++; /* start run */

          inptr += len + 1;

          if (inptr >= inend - 2)
            {
              break;
            }

          inptr -= len + 1;

          do
            {
              hval = LZF_NEXT(hval, inptr);
              g_lzf_hashtab[LZF_NDX(hval)] = (uint8_t *)inptr;
              inptr++;
            }
          while (len--);
        }
      else
        {
          /* One more literal byte we must copy */

          if (outptr >= outend)
            {
              cs = 0;
              goto genhdr;
            }

          lit++;
          *outptr++ = *inptr++;

          if (lit == LZF_MAX_LIT)
            {
              outptr[- lit - 1] = lit - 1; /* Stop run */
              lit = 0;;                /* Start run */
              outptr++;
            }
        }
    }

  /* At most 3 bytes can be missing here */

  if (outptr + 3 > outend)
    {
      cs = 0;
      goto genhdr;
    }

  while (inptr < inend)
    {
      lit++; *outptr++ = *inptr++;

      if (lit == LZF_MAX_LIT)
        {
          outptr[- lit - 1] = lit - 1; /* Stop run */
          lit = 0;                 /* Start run */
          outptr++;
        }
    }

  outptr[- lit - 1] = lit - 1; /* End run */
  outptr -= !lit;              /* Undo run if length is zero */

  cs = outptr - (uint8_t *)result->compressed.lzf_buffer;

genhdr:
  if (cs > 0)
    {
      /* Write compressed header */

      result->compressed.lzf_magic[0]   = 'Z';
      result->compressed.lzf_magic[1]   = 'V';
      result->compressed.lzf_type       = LZF_TYPE1_HDR;
      result->compressed.lzf_clen[0]    = cs >> 8;
      result->compressed.lzf_clen[1]    = cs & 0xff;
      result->compressed.lzf_ulen[0]    = inlen >> 8;
      result->compressed.lzf_ulen[1]    = inlen & 0xff;
      retlen                            = cs + LZF_TYPE1_HDR_SIZE;
    }
  else
    {
      /* Write uncompressed header*/

      result->uncompressed.lzf_magic[0] = 'Z';
      result->uncompressed.lzf_magic[1] = 'V';
      result->uncompressed.lzf_type     = LZF_TYPE0_HDR;
      result->uncompressed.lzf_len[0]   = inlen >> 8;
      result->uncompressed.lzf_len[1]   = inlen & 0xff;

      /* Copy uncompressed data into the result buffer */

      memcpy(result->uncompressed.lzf_buffer, inbuffer, inlen);
      retlen                            = inlen + LZF_TYPE0_HDR_SIZE;
    }

  return retlen;
}

static uint16_t get_mode(mode_t mode)
{
  uint16_t ret = 0;

  /* Convert mode to CROMFS NuttX read-only mode */

  if ((mode & S_IXOTH) != 0)
    {
      ret |= NUTTX_IXOTH;
    }

  if ((mode & S_IROTH) != 0)
    {
      ret |= NUTTX_IROTH;
    }

  if ((mode & S_IXGRP) != 0)
    {
      ret |= NUTTX_IXGRP;
    }

  if ((mode & S_IRGRP) != 0)
    {
      ret |= NUTTX_IRGRP;
    }

  if ((mode & S_IXUSR) != 0)
    {
      ret |= NUTTX_IXUSR;
    }

  if ((mode & S_IRUSR) != 0)
    {
      ret |= NUTTX_IRUSR;
    }

  return ret;
}

#ifdef HOST_TGTSWAP
static inline uint16_t tgt_uint16(uint16_t a)
{
  /* [15:8][7:0] -> [7:0][15:8] */

  return (a >> 8) | (a << 8);
}

static inline uint32_t tgt_uint32(uint32_t a)
{
  /* [31:24][23:16][15:8][7:0] -> [7:0][15:8][23:16][31:24] */

  return (a >> 24) | ((a >> 8) & 0x0000ff00) |
         ((a << 8) & 0x00ff0000) | (a << 24);
}
#endif

static void gen_dirlink(const char *name, uint32_t tgtoffs, bool dirempty)
{
  struct cromfs_node_s node;
  int namlen;

  namlen          = strlen(name) + 1;

  /* Generate the hardlink node */

  dump_nextline(g_tmpstream);
  fprintf(g_tmpstream, "\n  /* Offset %6lu:  Hard link %s */\n\n",
          (unsigned long)g_offset, name);

  node.cn_mode    = TGT_UINT16(DIRLINK_MODEFLAGS);
  node.cn_pad     = 0;

  g_offset       += sizeof(struct cromfs_node_s);
  node.cn_name    = TGT_UINT32(g_offset);
  node.cn_size    = 0;

  g_offset       += namlen;
  node.cn_peer    = TGT_UINT32(dirempty ? 0 : g_offset);
  node.u.cn_link  = TGT_UINT32(tgtoffs);

  dump_hexbuffer(g_tmpstream, &node, sizeof(struct cromfs_node_s));
  dump_hexbuffer(g_tmpstream, name, namlen);

  g_nnodes++;
}

static void gen_directory(const char *path, const char *name, mode_t mode,
                          bool lastentry)
{
  struct cromfs_node_s node;
  uint32_t save_offset        = g_offset;
  uint32_t save_diroffset     = g_diroffset;
  uint32_t save_parent_offset = g_parent_offset;
  FILE *save_tmpstream      = g_tmpstream;
  FILE *subtree_stream;
  int namlen;
  int result;

  namlen          = strlen(name) + 1;

  /* Open a new temporary file */

  subtree_stream  = open_tmpfile();
  g_tmpstream     = subtree_stream;

  /* Update the offset to account for the file node which we have not yet
   * written (we can't, we don't have enough information yet)
   */

  g_offset       += sizeof(struct cromfs_node_s) + namlen;

  /* Update offsets for the subdirectory */

  g_parent_offset = g_diroffset;  /* New offset for '..' */
  g_diroffset     = g_offset;     /* New offset for '.' */

  /* We are going to traverse the new directory twice; the first time just
   * see if the directory is empty.  The second time is the real thing.
   */

  result = traverse_directory(path, dir_notempty, NULL);

  /* Generate the '.' and '..' links for the directory (in the new temporary
   * file).
   */

  gen_dirlink(".", g_diroffset, false);
  gen_dirlink("..", g_parent_offset, result == 0);
  if (result != 0)
    {
      /* Then recurse to generate all of the nodes for the subtree */

      (void)traverse_directory(path, process_direntry, NULL);
    }

  /* When traverse_directory() returns, all of the nodes in the sub-tree under
   * 'name' will have been written to the new tmpfile.  g_offset is correct,
   * but other settings are not.
   *
   * Restore the state.
   */

  g_tmpstream     = save_tmpstream;
  g_diroffset     = save_diroffset;
  g_parent_offset = save_parent_offset;

  /* Generate the directory node */

  dump_nextline(g_tmpstream);
  fprintf(g_tmpstream, "\n  /* Offset %6lu:  Directory %s */\n\n",
          (unsigned long)save_offset, path);

  node.cn_mode    = TGT_UINT16(NUTTX_IFDIR | get_mode(mode));
  node.cn_pad     = 0;

  save_offset    += sizeof(struct cromfs_node_s);
  node.cn_name    = TGT_UINT32(save_offset);
  node.cn_size    = 0;

  save_offset    += namlen;
  node.cn_peer    = TGT_UINT32(lastentry ? 0 : g_offset);
  node.u.cn_child = TGT_UINT32(save_offset);

  dump_hexbuffer(g_tmpstream, &node, sizeof(struct cromfs_node_s));
  dump_hexbuffer(g_tmpstream, name, namlen);

  g_nnodes++;

  /* Now append the sub-tree nodes in the new tmpfile to the previous tmpfile */

  append_tmpfile(g_tmpstream, subtree_stream);
}

static void gen_file(const char *path, const char *name, mode_t mode,
                     bool lastentry)
{
  struct cromfs_node_s node;
  union lzf_result_u result;
  uint32_t nodeoffs = g_offset;
  FILE *save_tmpstream = g_tmpstream;
  FILE *outstream;
  FILE *instream;
  uint8_t iobuffer[LZF_BUFSIZE];
  size_t nread;
  size_t ntotal;
  size_t blklen;
  size_t blktotal;
  unsigned int blkno;
  int namlen;

  namlen      = strlen(name) + 1;

  /* Open a new temporary file */

  outstream   = open_tmpfile();
  g_tmpstream = outstream;
  g_offset    = nodeoffs + sizeof(struct cromfs_node_s) + namlen;

  /* Open the source data file */

  instream    = fopen(path, "r");
  if (!instream)
    {
      fprintf(stderr, "fopen for source file %s failed: %s\n",
              path, strerror(errno));
      exit(1);
    }

  /* Then read data from the file, compress it, and write it to the new
   * temporary file
   */

  blkno       = 0;
  ntotal      = 0;
  blktotal    = 0;

  do
    {
      /* Read the next chunk from the file */

      nread = fread(iobuffer, 1, LZF_BUFSIZE, instream);
      if (nread > 0)
        {
          uint16_t clen;

          /* Compress the chunk */

          blklen = lzf_compress(iobuffer, nread, &result);
          if (result.cmn.lzf_type == LZF_TYPE0_HDR)
            {
              clen = nread;
            }
          else
            {
              clen = (uint16_t)result.compressed.lzf_clen[0] << 8 |
                     (uint16_t)result.compressed.lzf_clen[1];
            }

          dump_nextline(g_tmpstream);
          fprintf(g_tmpstream,
                  "\n  /* Offset %6lu:  "
                  "Block %u blklen=%lu Uncompressed=%lu Compressed=%u */\n\n",
                  (unsigned long)g_offset, blkno, (long)blklen,
                  (long)nread, clen);
          dump_hexbuffer(g_tmpstream, &result, blklen);

          ntotal   += nread;
          blktotal += blklen;
          g_offset += blklen;

          g_nblocks++;
          blkno++;
        }
    }
  while (nread > 0);

  /* Restore the old tmpfile context */

  g_tmpstream        = save_tmpstream;

  /* Now we have enough information to generate the file node */

  dump_nextline(g_tmpstream);

  fprintf(g_tmpstream, "\n  /* Offset %6lu:  File %s:  "
          "Uncompressed=%lu Compressed=%lu */\n\n",
          (unsigned long)nodeoffs, path, (unsigned long)ntotal,
          (unsigned long)blktotal);

  node.cn_mode       = TGT_UINT16(NUTTX_IFREG | get_mode(mode));
  node.cn_pad        = 0;

  nodeoffs          += sizeof(struct cromfs_node_s);
  node.cn_name       = TGT_UINT32(nodeoffs);

  node.cn_size       = TGT_UINT32(ntotal);

  nodeoffs          += namlen;
  node.u.cn_blocks   = TGT_UINT32(nodeoffs);

  nodeoffs          += blktotal;
  node.cn_peer       = TGT_UINT32(lastentry ? 0 : nodeoffs);

  dump_hexbuffer(g_tmpstream, &node, sizeof(struct cromfs_node_s));
  dump_hexbuffer(g_tmpstream, name, namlen);

  g_nnodes++;

  /* Now append the sub-tree nodes in the new tmpfile to the previous tmpfile */

  append_tmpfile(g_tmpstream, outstream);
}

static int dir_notempty(const char *dirpath, const char *name,
                        void *arg, bool lastentry)
{
  struct stat buf;
  char *path;
  int ret;

  asprintf(&path, "%s/%s", dirpath, name);

  /* stat() should not fail for any reason */

  ret = stat(path, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
             path, strerror(errcode));
      exit(1);
    }

  /* The directory is not empty if it contains with a file or a directory
   * entry.  Anything else will be ignored and the directly may be
   * effectively empty.
   */

  free(path);
  return (S_ISREG(buf.st_mode) || S_ISDIR(buf.st_mode));
}

static int process_direntry(const char *dirpath, const char *name,
                            void *arg, bool lastentry)
{
  struct stat buf;
  char *path;
  int ret;

  asprintf(&path, "%s/%s", dirpath, name);

  ret = stat(path, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode == ENOENT)
        {
          fprintf(stderr, "ERROR: Directory entry %s does not exist\n", path);
        }
      else
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 path, strerror(errcode));
        }

      show_usage();
    }

  /* Verify that the source is, indeed, a directory */

  else if (S_ISDIR(buf.st_mode))
    {
      gen_directory(path, name, buf.st_mode, lastentry);
    }
  else if (S_ISREG(buf.st_mode))
    {
      gen_file(path, name, buf.st_mode, lastentry);
    }
  else
    {
      fprintf(stderr, "Omitting entry %s\n", path);
    }

  free(path);
  return 0;
}

static int traverse_directory(const char *dirpath,
                              traversal_callback_t callback, void *arg)
{
  DIR *dirp;
  struct dirent *direntry;
  char name[NAME_MAX + 1];
  int ret = 0;

  /* Open the directory */

  dirp = opendir(dirpath);
  if (dirp == NULL)
    {
      fprintf(stderr, "ERROR: opendir(%s) failed: %s\n",
              dirpath, strerror(errno));
      show_usage();
    }

  /* Visit each entry in the directory */

  direntry = readdir(dirp);
  while (direntry != NULL)
    {
      /* Preserve the name from the directory entry.  The return value
       * from readdir() only persists until the next time that readdir()
       * is called (alternatively, use readdir_r).
       */

      strncpy(name, direntry->d_name, NAME_MAX + 1);

      /* Get the next entry in advance so that we can anticipate the end of
       * the directory.
       */

      direntry = readdir(dirp);

      /* Skip the '.' and '..' hard links */

      if (strcmp(name, ".") != 0 && strcmp(name, "..") != 0)
        {
          /* Process the directory entry */

          ret = callback(dirpath, name, arg, direntry == NULL);
          if (ret != 0)
            {
              break;
            }
        }
    }

  closedir(dirp);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct cromfs_volume_s vol;
  char *ptr;
  int result;

  /* Verify arguments */

  ptr = strrchr(argv[0], '/');
  g_progname = ptr == NULL ? argv[0] : ptr + 1;

  if (argc != 3)
    {
      fprintf(stderr, "Unexpected number of arguments\n");
      show_usage();
    }

  g_dirname  = argv[1];
  g_outname  = argv[2];

  verify_directory();
  verify_outfile();;

  g_outstream = fopen(g_outname, "w");
  if (!g_outstream)
    {
      fprintf(stderr, "open %s failed: %s\n", g_outname, strerror(errno));
      exit(1);
    }

  g_tmpstream = open_tmpfile();

  /* Set up the initial boilerplate at the beginning of each file */

  init_outfile();

  /* Set up some initial offsets */

  g_offset        = sizeof(struct cromfs_volume_s);  /* Current image offset */
  g_diroffset     = sizeof(struct cromfs_volume_s);  /* Offset for '.' */
  g_parent_offset = sizeof(struct cromfs_volume_s);  /* Offset for '..' */

  /* We are going to traverse the new directory twice; the first time just
   * see if the directory is empty.  The second time is the real thing.
   */

  result = traverse_directory(g_dirname, dir_notempty, NULL);

  /* Generate the '.' link for the root directory (it can't have a '..') */

  gen_dirlink(".", g_diroffset, result == 0);
  if (result != 0)
    {
      /* Then traverse each entry in the directory, generating node data for
       * each directory entry encountered.
       */

      (void)traverse_directory(g_dirname, process_direntry, NULL);
    }

  /* Now append the volume header to output file */

  fprintf(g_outstream, "/* CROMFS image */\n\n");
  fprintf(g_outstream, "const uint8_t g_cromfs_image[] =\n");
  fprintf(g_outstream, "{\n");
  fprintf(g_outstream, "\n  /* Offset %6lu:  Volume header */\n\n", 0ul);

  vol.cv_magic    = TGT_UINT32(CROMFS_MAGIC);
  vol.cv_nnodes   = TGT_UINT16(g_nnodes);
  vol.cv_nblocks  = TGT_UINT16(g_nblocks);
  vol.cv_root     = TGT_UINT32(sizeof(struct cromfs_volume_s));
  vol.cv_fsize    = TGT_UINT32(g_offset);
  vol.cv_bsize    = TGT_UINT32(CROMFS_BLOCKSIZE);

  g_nhex          = 0;
  dump_hexbuffer(g_outstream, &vol, sizeof(struct cromfs_volume_s));

  dump_nextline(g_outstream);
  fprintf(g_outstream, "\n  /* Offset %6lu:  Root directory */\n",
          (unsigned long)sizeof(struct cromfs_volume_s));

  /* Finally append the nodes to the output file */

  append_tmpfile(g_outstream,g_tmpstream);
  fprintf(g_outstream, "\n};\n");

  fclose(g_outstream);
#ifndef USE_MKSTEMP
  unlink_tmpfiles();
#endif
  return 0;
}
