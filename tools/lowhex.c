/****************************************************************************
 * tools/lowhex.c
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
#include <stdbool.h>
#include <ctype.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINESIZE 1024

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_line[LINESIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool isdelimiter(int ch)
{
  if (isalnum(ch) || ch == '_')
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

int main(int argc, char **argv)
{
  FILE *instream;
  FILE *outstream;
  bool delimited;
  bool inhex;
  int ret = 1;
  int i;
  int j;

  if (argc != 3)
    {
      fprintf(stderr, "ERROR:  Two arguments expected\n");
      return 1;
    }

  /* Open the source file read-only */

  instream = fopen(argv[1], "r");
  if (instream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[1]);
      return 1;
    }

  /* Open the destination file write-only */

  outstream = fopen(argv[2], "w");
  if (outstream == NULL)
    {
      fprintf(stderr, "ERROR:  Failed to open %s for reading\n", argv[2]);
      goto errout_with_instream;
    }

  /* Process each line in the file */

  while ((fgets(g_line, LINESIZE, instream) != NULL))
    {
      /* Search for a '0x' that preceding some delimiting character */

      delimited = true;
      inhex     = false;

      for (i = 0;
           i < LINESIZE && g_line[i] != '\n' && g_line[i] != '\0';
           i++)
        {
          if (inhex)
            {
              if (isxdigit(g_line[i]))
                {
                  g_line[i] = tolower(g_line[i]);
                }
              else
                {
                  inhex = false;
                }
            }
          else if (delimited && g_line[i] == '0')
            {
              j = i + 1;
              if (j < LINESIZE)
                {
                  if (g_line[j] == 'x' || g_line[j] == 'X')
                    {
                      g_line[j] = tolower(g_line[j]);
                      inhex = true;
                      i = j;
                    }
                }
            }
          else
            {
              delimited = isdelimiter(g_line[i]);
            }
        }

      fputs(g_line, outstream);
    }

  ret = 0;
  fclose(outstream);

errout_with_instream:
  fclose(instream);
  return ret;
}
