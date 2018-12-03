/****************************************************************************
 * tools/detab.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINE_SIZE 1024

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_line[LINE_SIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *instream;
  FILE *outstream;
  const char *infile;
  const char *outfile;
  int tabsize = 8;
  int tabmask = 7;
  int newpos;
  int pos;
  int ret = 1;
  int i;

  if (argc == 3)
    {
      infile  = argv[1];
      outfile = argv[2];
    }
  else if (argc == 4)
    {
      if (strcmp(argv[1], "-4") != 0)
        {
          fprintf(stderr, "ERROR:  Unrecognized option\n");
          fprintf(stderr, "Usage:  %s [-4] <source-file> <out-file>\n", argv[0]);
          return 1;
        }

      tabsize = 4;
      tabmask = 3;
      infile  = argv[2];
      outfile = argv[3];
    }
  else
    {
      fprintf(stderr, "ERROR:  At least two arguments expected\n");
      fprintf(stderr, "Usage:  %s [-4] <source-file> <out-file>\n", argv[0]);
      return 1;
    }

  /* Open the source file read-only */

  instream = fopen(infile, "r");
  if (instream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[1]);
      return 1;
    }

  /* Open the destination file write-only */

  outstream = fopen(outfile, "w");
  if (outstream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[2]);
      goto errout_with_instream;
    }

  while (fgets(g_line, LINE_SIZE, instream) != NULL)
    {
      for (pos = 0, i = 0; i < LINE_SIZE && g_line[i] != '\0'; i++)
        {
          if (g_line[i] == '\t')
            {
              newpos = (pos + tabsize) & ~tabmask;
              for (; pos < newpos; pos++)
                {
                  fputc(' ', outstream);
                }
            }
          else
            {
              fputc(g_line[i], outstream);
              pos++;
            }
        }
    }

  ret = 0;

  fclose(outstream);

errout_with_instream:
  fclose(instream);
  return ret;
}
