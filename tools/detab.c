/****************************************************************************
 * tools/detab.c
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
          fprintf(stderr, "Usage:  %s [-4] <source-file> <out-file>\n",
                  argv[0]);
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
