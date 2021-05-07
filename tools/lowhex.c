/****************************************************************************
 * tools/lowhex.c
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
