/****************************************************************************
 * tools/cmpconfig.c
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
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

#include "cfgparser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <config1> <config2>\n", progname);
  exit(EXIT_FAILURE);
}

static int compare_variables(struct variable_s *list1,
                             struct variable_s *list2)
{
  char *varval1;
  char *varval2;
  int ret = 0;

  while (list1 || list2)
    {
      if (list1 && list1->val)
        {
          varval1 = list1->val;
        }
      else
        {
          varval1 = "<NULL>";
        }

      if (list2 && list2->val)
        {
          varval2 = list2->val;
        }
      else
        {
          varval2 = "<NULL>";
        }

      if (!list1)
        {
          printf("file1:\n");
          printf("file2: %s=%s\n\n", list2->var, varval2);
          list2 = list2->flink;
          ret = EXIT_FAILURE;
        }
      else if (!list2)
        {
          printf("file1: %s=%s\n", list1->var, varval1);
          printf("file2:\n\n");
          list1 = list1->flink;
          ret = EXIT_FAILURE;
        }
      else
        {
          int result;

          result = strcmp(list1->var, list2->var);
          if (result < 0)
            {
              printf("file1: %s=%s\n", list1->var, varval1);
              printf("file2:\n\n");
              list1 = list1->flink;
              ret = EXIT_FAILURE;
            }
          else if (result > 0)
            {
              printf("file1:\n");
              printf("file2: %s=%s\n\n", list2->var, varval2);
              list2 = list2->flink;
              ret = EXIT_FAILURE;
            }
          else /* if (result == 0) */
            {
              result = strcmp(varval1, varval2);
              if (result != 0)
                {
                  printf("file1: %s=%s\n", list1->var, varval1);
                  printf("file2: %s=%s\n\n", list2->var, varval2);
                }

              list1 = list1->flink;
              list2 = list2->flink;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct variable_s *list1 = 0;
  struct variable_s *list2 = 0;
  FILE *stream1;
  FILE *stream2;

  if (argc != 3)
    {
      fprintf(stderr, "Unexpected number of arguments: %d\n\n", argc);
      show_usage(argv[0]);
    }

  stream1 = fopen(argv[1], "r");
  if (!stream1)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n\n",
        argv[1], strerror(errno));
      show_usage(argv[0]);
    }

  stream2 = fopen(argv[2], "r");
  if (!stream2)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n\n",
        argv[2], strerror(errno));
      show_usage(argv[0]);
    }

  parse_file(stream1, &list1);
  parse_file(stream2, &list2);

  fclose(stream1);
  fclose(stream2);

  return compare_variables(list1, list2);
}
