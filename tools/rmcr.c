/****************************************************************************
 * tools/rmcr.c
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
#include <string.h>
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
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  FILE *instream;
  FILE *outstream;
  int len;
  int ret = 1;

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
      /* Remove all whitespace (including newline) from the end of the line */

      len = strlen(g_line) - 1;
      while (len >= 0 && isspace(g_line[len]))
        {
          len--;
        }

      /* Put the newline back. len is either -1, or points to the last, non-
       * space character in the line.
       */

      g_line[len + 1] = '\n';
      g_line[len + 2] = '\0';
      fputs(g_line, outstream);
    }

  ret = 0;
  fclose(outstream);

errout_with_instream:
  fclose(instream);
  return ret;
}
