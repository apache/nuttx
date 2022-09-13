/****************************************************************************
 * tools/b16.c
 * Convert b16 fixed precision value to float or vice versa
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "\nUSAGE: %s <b16_t>|<float>\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "  <b16_t>:\n");
  fprintf(stderr, "    A b16 fixed precision value in hexadecimal form:\n");
  fprintf(stderr, "    E.g., 0x00010000\n");
  fprintf(stderr, "    Any value beginning with '0' will be assumed to\n");
  fprintf(stderr, "    be in hexadecimal format\n");
  fprintf(stderr, "  <float>:\n");
  fprintf(stderr, "    A floating point value in standard form:\n");
  fprintf(stderr, "    E.g., 5.1\n");
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  double fvalue;
  unsigned long ulvalue;
  long lvalue;
  const char *str;
  char *endptr;

  /* There must be exactly one argument */

  if (argc != 2)
    {
      fprintf(stderr, "\nExpected a single argument\n");
      show_usage(argv[0]);
    }

  str = argv[1];

  /* If the value begins with a zero, we will assume that it is a hexadecimal
   * representation.
   */

  if (str[0] == '0')
    {
      endptr = NULL;
      ulvalue = strtoul(str, &endptr, 16);
      if (!endptr || *endptr != '\0')
        {
          fprintf(stderr, "\nHexadecimal argument not fully converted\n");
          show_usage(argv[0]);
        }

      if (ulvalue >= 0x80000000)
        {
          lvalue = ~ulvalue + 1;
        }
      else
        {
          lvalue = ulvalue;
        }

      fvalue = ((double)lvalue) / 65536.0;
      printf("0x%08lx -> %10.5f\n", ulvalue, fvalue);
    }
  else
    {
      endptr = NULL;
      fvalue = strtod(str, &endptr);
      if (!endptr || *endptr != '\0')
        {
          fprintf(stderr, "\nFloating point argument not fully converted\n");
          show_usage(argv[0]);
        }

      lvalue = 65536.0 * fvalue;
      printf("%10.5f -> 0x%08lx\n", fvalue, lvalue);
    }

  return 0;
}
