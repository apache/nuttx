/****************************************************************************
 * tools/csvparser.c
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "csvparser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool g_debug;
char g_line[LINESIZE + 1];
char g_parm[MAX_FIELDS][MAX_PARMSIZE];
int g_lineno;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skip_space
 ****************************************************************************/

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/****************************************************************************
 * Name: copy_parm
 ****************************************************************************/

static char *copy_parm(char *src, char *dest)
{
  char *start = src;
  int i;

  /* De-quote the parameter and copy it into the parameter array */

  for (i = 0; i < MAX_PARMSIZE; i++)
    {
      if (*src == '"')
        {
          *dest = '\0';
          return src;
        }
      else if (*src == '\n' || *src == '\0')
        {
          fprintf(stderr, "%d: Unexpected end of line: \"%s\"\n",
                  g_lineno, start);
          exit(4);
        }
      else
        {
          *dest++ = *src++;
        }
    }

  fprintf(stderr, "%d: Parameter too long: \"%s\"\n", g_lineno, start);
  exit(3);
}

/****************************************************************************
 * Name: find_parm
 ****************************************************************************/

static char *find_parm(char *ptr)
{
  char *start = ptr;

  if (*ptr != '"')
    {
      fprintf(stderr, "%d: I'm confused: \"%s\"\n", g_lineno, start);
      exit(5);
    }

  ptr++;

  ptr = skip_space(ptr);
  if (*ptr == '\n' || *ptr == '\0')
    {
      return NULL;
    }
  else if (*ptr != ',')
    {
      fprintf(stderr, "%d: Expected ',': \"%s\"\n", g_lineno, start);
      exit(6);
    }

  ptr++;

  ptr = skip_space(ptr);
  if (*ptr != '"')
    {
      fprintf(stderr, "%d: Expected \": \"%s\"\n", g_lineno, start);
      exit(7);
    }

  ptr++;

  return ptr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_line
 ****************************************************************************/

char *read_line(FILE *stream)
{
  char *ptr;

  for (; ; )
    {
      g_line[LINESIZE] = '\0';
      if (!fgets(g_line, LINESIZE, stream))
        {
          return NULL;
        }
      else
        {
          g_lineno++;
          if (g_debug)
            {
              printf("Line: %s\n", g_line);
            }

          ptr = skip_space(g_line);
          if (*ptr && *ptr != '#' && *ptr != '\n')
            {
              return ptr;
            }
        }
    }
}

/****************************************************************************
 * Name: parse_csvline
 ****************************************************************************/

int parse_csvline(char *ptr)
{
  int nparms;
  int i;

  /* Format "arg1","arg2","arg3",... Spaces will be tolerated outside of the
   * quotes.  Any initial spaces have already been skipped so the first thing
   * should be '"'.
   */

  if (*ptr != '"')
    {
      fprintf(stderr, "%d: Bad line: \"%s\"\n", g_lineno, g_line);
      exit(2);
    }

  ptr++;
  nparms = 0;

  /* Copy each comma-separated value in an array (stripping quotes from each
   * of the values).
   */

  do
    {
      if (nparms >= MAX_FIELDS)
        {
          fprintf(stderr, "%d: Too many Parameters: \"%s\"\n",
                  g_lineno, g_line);
          exit(8);
        }

      ptr = copy_parm(ptr, &g_parm[nparms][0]);
      nparms++;
      ptr = find_parm(ptr);
    }
  while (ptr);

  /* If debug is enabled, show what we got */

  if (g_debug)
    {
      printf("Parameters: %d\n", nparms);
      for (i = 0; i < nparms; i++)
        {
          printf("  Parm%d: \"%s\"\n", i + 1, g_parm[i]);
        }
    }

  return nparms;
}
