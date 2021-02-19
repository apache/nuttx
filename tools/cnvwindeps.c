/****************************************************************************
 * tools/cnvwindeps.c
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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#ifdef HOST_CYGWIN

#include <sys/cygwin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_LINE 1024
#define MAX_PATH 1024

/****************************************************************************
 * Global Data
 ****************************************************************************/

static unsigned long g_lineno;
static char g_line[MAX_LINE];
static char g_dequoted[MAX_PATH];
static char g_posix[MAX_PATH];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static char *skip_spaces(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

static char *find_spaces(char *ptr)
{
  bool quoted = false;

  while (*ptr)
    {
      if (ptr[0] == '\\' && isspace((int)ptr[1]))
        {
          quoted = true;
          ptr++;
        }
      else if (!quoted && isspace((int)*ptr))
        {
          break;
        }
      else
        {
          quoted = false;
          ptr++;
        }
    }

  return ptr;
}

static bool scour_path(const char *path)
{
  /* KLUDGE: GNU make cannot handle dependencies with spaces in them.
   * There may be addition characters that cause problems too.
   */

  return strchr(path, ' ') != NULL;
}

static bool dequote_path(const char *winpath)
{
  char *dest = g_dequoted;
  const char *src = winpath;
  int len = 0;
  bool quoted = false;

  while (*src && len < MAX_PATH)
    {
      if (src[0] != '\\' || (src[1] != ' ' &&
          src[1] != '(' && src[1] != ')'))
        {
          *dest++ = *src;
          len++;
        }
      else
        {
          quoted = true;
        }

      src++;
    }

  if (*src || len >= MAX_PATH)
    {
      fprintf(stderr, "%lu: Line truncated\n", g_lineno);
      exit(EXIT_FAILURE);
    }

  *dest = '\0';
  return quoted;
}

static bool convert_path(const char *winpath)
{
  ssize_t size;
  ssize_t ret;
  bool quoted;

  quoted = dequote_path(winpath);

  size = cygwin_conv_path(CCP_WIN_A_TO_POSIX | CCP_RELATIVE,
                          g_dequoted, NULL, 0);
  if (size > MAX_PATH)
    {
      fprintf(stderr, "%lu: POSIX path too long: %lu\n",
              g_lineno, (unsigned long)size);
      exit(EXIT_FAILURE);
    }

  ret = cygwin_conv_path(CCP_WIN_A_TO_POSIX | CCP_RELATIVE,
                         g_dequoted, g_posix, MAX_PATH);
  if (ret < 0)
    {
      fprintf(stderr, "%lu: cygwin_conv_path '%s' failed: %s\n",
              g_lineno, g_dequoted, strerror(errno));
      exit(EXIT_FAILURE);
    }

  return quoted;
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <path-to-deps-file>\n", progname);
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *path;
  char *next;
  FILE *stream;
  bool begin;
  bool quoted;
  bool scouring;

  if (argc != 2)
    {
      fprintf(stderr, "Unexpected number of arguments\n");
      show_usage(argv[0]);
    }

  stream = fopen(argv[1], "r");
  if (!stream)
    {
      fprintf(stderr, "open %s failed: %s\n", argv[1], strerror(errno));
      exit(EXIT_FAILURE);
    }

  begin    = true;
  scouring = false;
  g_lineno = 0;

  while (fgets(g_line, MAX_LINE, stream) != NULL)
    {
      g_lineno++;
      next = g_line;

      for (; ; )
        {
          if (begin)
            {
              path = skip_spaces(next);
              if (*path == '#')
                {
                  /* The reset of the line is comment */

                  puts(path);
                  break;
                }

              next = strchr(path, ':');
              if (!next)
                {
                  fprintf(stderr, "%lu: Expected colon\n", g_lineno);
                  exit(EXIT_FAILURE);
                }

              if (*next != '\0')
                {
                  *next++ = '\0';
                }

              scouring = scour_path(path);
              if (!scouring)
                {
                  quoted = convert_path(path);
                  if (quoted)
                    {
                      printf("\"%s\":", g_posix);
                    }
                  else
                    {
                      printf("%s:", g_posix);
                    }
                }

              begin = false;
            }
          else
            {
              path = skip_spaces(next);
              next = find_spaces(path);

              if (path[0] == '\\')
                {
                  break;
                }
              else if (strcmp(path, "") == 0)
                {
                  printf("\n\n");
                  begin    = true;
                  scouring = false;
                  break;
                }
              else
                {
                  if (*next != '\0')
                    {
                      *next++ = '\0';
                    }

                  if (!scouring && !scour_path(path))
                    {
                      quoted = convert_path(path);
                      if (quoted)
                        {
                          printf(" \\\n\t\"%s\"", g_posix);
                        }
                      else
                        {
                          printf(" \\\n\t%s", g_posix);
                        }
                    }
                }
            }
        }
    }

  fclose(stream);
  return 0;
}

#else /* HOST_CYGWIN */

int main(int argc, char **argv, char **envp)
{
  fprintf(stderr, "ERROR: This tool is only available under Cygwin\n");
  return EXIT_FAILURE;
}

#endif /* HOST_CYGWIN */
