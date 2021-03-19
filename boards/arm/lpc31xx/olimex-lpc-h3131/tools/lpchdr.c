/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/tools/lpchdr.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "lpchdr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IO_BUF_SIZE  1024
#define HDR_SIZE     0x80
#define HDR_CRC_SIZE 0x6c

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_infile;
static const char *g_outfile;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
  fprintf(stderr, "%s -o <outfile> <infile>\n", progname);
  exit(exitcode);
}

static void parse_args(int argc, char **argv)
{
  int ch;

  while ((ch = getopt(argc, argv, ":o:")) >= 0)
    {
      switch (ch)
        {
          case 'o':
            g_outfile = optarg;
            break;

          case ':':
            fprintf(stderr, "Missing option argumen\n");
            show_usage(argv[0], 1);

          case '?':
          default:
            fprintf(stderr, "Unrecognized option\n");
            show_usage(argv[0], 1);
        }
    }

  if (optind >= argc)
    {
      fprintf(stderr, "Missing binary input file name\n");
      show_usage(argv[0], 1);
    }

  g_infile = argv[optind];
  optind++;

  if (optind < argc)
    {
      fprintf(stderr, "Garbage at the end of the command line\n");
      show_usage(argv[0], 1);
    }
}

static inline uint32_t infilecrc32(int infd, size_t len, size_t padlen)
{
  off_t offset;
  uint8_t buffer[IO_BUF_SIZE];
  ssize_t nbytes;
  size_t bytesread;
  uint32_t crc;

  offset = lseek(infd, 0, SEEK_SET);
  if (offset == (off_t)-1)
    {
      fprintf(stderr, "lseek failed: %s\n", strerror(errno));
      exit(4);
    }

  crc = 0;
  for (bytesread = 0; bytesread < len; bytesread += nbytes)
    {
      nbytes = read(infd, buffer, IO_BUF_SIZE);
      if (nbytes < 0)
        {
          fprintf(stderr, "read failed: %s\n", strerror(errno));
          exit(4);
        }
      else if (nbytes == 0)
        {
          fprintf(stderr, "Unexpected end-of-file: %s\n", strerror(errno));
          exit(4);
        }
      else
        {
          crc = crc32part(buffer, nbytes, crc);
        }
    }

  /* Add the zero-padding at the end of the binary in the CRC */

  memset(buffer, 0, IO_BUF_SIZE);
  return crc32part(buffer, padlen, crc);
}

static inline void writefile(int infd, int outfd, size_t len, size_t padlen)
{
  off_t offset;
  uint8_t buffer[IO_BUF_SIZE];
  ssize_t nbytesread;
  ssize_t nbyteswritten;
  size_t totalread;

  offset = lseek(infd, 0, SEEK_SET);
  if (offset == (off_t)-1)
    {
      fprintf(stderr, "lseek failed: %s\n", strerror(errno));
      exit(4);
    }

  for (totalread = 0; totalread < len; totalread += nbytesread)
    {
      nbytesread = read(infd, buffer, IO_BUF_SIZE);
      if (nbytesread < 0)
        {
          fprintf(stderr, "read failed: %s\n", strerror(errno));
          exit(4);
        }
      else if (nbytesread == 0)
        {
          fprintf(stderr, "Unexpected end-of-file: %s\n", strerror(errno));
          exit(4);
        }
      else
        {
          nbyteswritten = write(outfd, buffer, nbytesread);
          if (nbyteswritten < 0)
            {
              fprintf(stderr, "write failed: %s\n", strerror(errno));
              exit(4);
            }
          else if (nbyteswritten != nbytesread)
            {
              fprintf(stderr, "Short writes not handled\n");
              exit(4);
            }
        }
    }

  /* Write the zero-padding at the end of the binary */

  memset(buffer, 0, IO_BUF_SIZE);
  nbyteswritten = write(outfd, buffer, padlen);
  if (nbyteswritten < 0)
    {
      fprintf(stderr, "write failed: %s\n", strerror(errno));
      exit(4);
    }
  else if (nbyteswritten != padlen)
    {
      fprintf(stderr, "Short writes not handled\n");
      exit(4);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct lpc31_header_s g_hdr;
  struct stat buf;
  ssize_t nbytes;
  size_t padlen;
  int infd;
  int outfd;
  int ret;

  /* Parse arguments */

  parse_args(argc, argv);

  /* Open files */

  infd = open(g_infile, O_RDONLY);
  if (infd < 0)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n",
              g_infile, strerror(errno));
      exit(2);
    }

  outfd = open(g_outfile, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (outfd < 0)
    {
      fprintf(stderr, "Failed to open %s for writing: %s\n",
              g_outfile, strerror(errno));
      exit(2);
    }

  /* Get the size of the binary file */

  ret = fstat(infd, &buf);
  if (ret < 0)
    {
      fprintf(stderr, "stat of %s failed: %s\n",
              g_infile, strerror(errno));
      exit(3);
    }

  /* Initialize the header */

  memset(&g_hdr, 0, sizeof(struct lpc31_header_s));
  g_hdr.vector          = 0xea00001e;  /* b 0x11029080 */
  g_hdr.magic           = 0x41676d69;
#if 1 /* CRC doesn't seem to be functional */
  g_hdr.imagetype       = 0x0000000a;
#else
  g_hdr.imagetype       = 0x0000000b;
#endif
  g_hdr.imagelength     = (buf.st_size +
                           sizeof(struct lpc31_header_s) + 511) & ~0x1ff;

  /* This is how much we must pad at the end of the binary image. */

  padlen                = g_hdr.imagelength - buf.st_size;

  /* Calculate CRCs */

  g_hdr.execution_crc32 = infilecrc32(infd, buf.st_size, padlen);
  g_hdr.header_crc32    = crc32((const uint8_t *)&g_hdr, HDR_CRC_SIZE);

  /* Write the header */

  nbytes = write(outfd, &g_hdr, HDR_SIZE);
  if (nbytes != 0x80)
    {
      fprintf(stderr, "write of header to of %s failed: %s\n",
              g_outfile, strerror(errno));
      exit(4);
    }

  /* Copy the input file to the output */

  writefile(infd, outfd, buf.st_size, padlen);
  close(infd);
  close(outfd);
  return 0;
}
