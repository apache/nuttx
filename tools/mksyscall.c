/****************************************************************************
 * tools/mksyscall.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LINESIZE      (PATH_MAX > 256 ? PATH_MAX : 256)

#define MAX_FIELDS    16
#define MAX_PARMSIZE  128
#define NAME_INDEX    0
#define HEADER_INDEX  1
#define RETTYPE_INDEX 2
#define PARM1_INDEX   3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_debug;
static char g_line[LINESIZE+1];
static char g_parm[MAX_FIELDS][MAX_PARMSIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static char *skip_space(char *ptr)
{
  while (*ptr && isspace(*ptr)) ptr++;
  return ptr;
}

static char *read_line(FILE *stream)
{
  char *ptr;

  for (;;)
    {
      g_line[LINESIZE] = '\0';
      if (!fgets(g_line, LINESIZE, stream))
        {
          return NULL;
        }
      else
        {
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

static char *copy_parm(char *src, char *dest)
{
  char *start = src;
  int i;

  for (i = 0; i < MAX_PARMSIZE; i++)
    {
      if (*src == '"')
        {
          *dest = '\0';
          return src;
        }
      else if (*src == '\n' || *src == '\0')
        {
          fprintf(stderr, "Unexpected end of line: \"%s\"\n", start);
          exit(4);
        }
      else
        {
          *dest++ = *src++;
        }
    }

  fprintf(stderr, "Parameter too long: \"%s\"\n", start);
  exit(3);
}

static char *find_parm(char *ptr)
{
  char *start = ptr;

  if (*ptr != '"')
    {
      fprintf(stderr, "I'm confused: \"%s\"\n", start);
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
      fprintf(stderr, "Expected ',': \"%s\"\n", start);
      exit(6);
    }
  ptr++;

  ptr = skip_space(ptr);
  if (*ptr != '"')
    {
      fprintf(stderr, "Expected \": \"%s\"\n", start);
      exit(7);
    }
  ptr++;

  return ptr;
}

static int parse_csvline(char *ptr)
{
  int nparms;
  int i;

  /* Format "arg1","arg2","arg3",... Spaces will be tolerated outside of the
   * quotes.  Any initial spaces have already been skipped so the first thing
   * should be '"'.
   */

  if (*ptr != '"')
    {
      fprintf(stderr, "Bad line: \"%s\"\n", g_line);
      exit(2);
    }

  ptr++;
  nparms = 0;

  do
    {
      ptr = copy_parm(ptr, &g_parm[nparms][0]);
      nparms++;
      ptr = find_parm(ptr);
    }
  while (ptr);

  if (g_debug)
    {
      printf("Parameters: %d\n", nparms);
      for (i = 0; i < nparms; i++)
        {
          printf("  Parm%d: \"%s\"\n", i+1, g_parm[i]);
        }
    }
  return nparms;
}

static FILE *open_proxy(void)
{
  char filename[MAX_PARMSIZE+4];
  FILE *stream;

  snprintf(filename, MAX_PARMSIZE+3, "%s.c", g_parm[NAME_INDEX]);
  filename[MAX_PARMSIZE+3] = '\0';

  stream = fopen(filename, "w");
  if (stream == NULL)
    {
      fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
      exit(10);
    }
  return stream;
}

static void generate_proxy(int nparms)
{
  FILE *stream = open_proxy();
  int i;

  fprintf(stream, "/* Auto-generated %s proxy file -- do not edit */\n\n", g_parm[NAME_INDEX]);
  fprintf(stream, "#include <%s>\n", g_parm[HEADER_INDEX]);
  fprintf(stream, "#include <syscall.h>\n\n");

  fprintf(stream, "%s %s(", g_parm[RETTYPE_INDEX], g_parm[NAME_INDEX]);

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nparms; i++)
        {
          if (i > 0)
            {
              fprintf(stream, ", %s parm%d", g_parm[PARM1_INDEX+i], i+1);
            }
          else
            {
              fprintf(stream, "%s parm%d", g_parm[PARM1_INDEX+i], i+1);
            }
        }
    }

  fprintf(stream, ")\n{\n");
  fprintf(stream, "  return (%s)sys_call%d(", g_parm[RETTYPE_INDEX], nparms);
  for (i = 0; i < nparms; i++)
    {
      if (i > 0)
        {
          fprintf(stream, ", (uintptr_t)parm%d", i+1);
        }
      else
        {
          fprintf(stream, "(uintptr_t)parm%d", i+1);
        }
    }
  fprintf(stream, ");\n}\n");
  fclose(stream);
}

static FILE *open_stub(void)
{
  char filename[MAX_PARMSIZE+8];
  FILE *stream;

  snprintf(filename, MAX_PARMSIZE+7, "STUB_%s.c", g_parm[NAME_INDEX]);
  filename[MAX_PARMSIZE+7] = '\0';

  stream = fopen(filename, "w");
  if (stream == NULL)
    {
      fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
      exit(9);
    }
  return stream;
}

static void generate_stub(int nparms)
{
  FILE *stream = open_stub();
  int i;

  fprintf(stream, "/* Auto-generated %s stub file -- do not edit */\n\n", g_parm[0]);
  fprintf(stream, "#include <%s>\n\n", g_parm[HEADER_INDEX]);
  fprintf(stream, "uintptr_t STUB_%s(", g_parm[NAME_INDEX]);

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nparms; i++)
        {
          if (i > 0)
            {
              fprintf(stream, ", uintptr_t parm%d", i+1);
            }
          else
            {
              fprintf(stream, "uintptr_t parm%d", i+1);
            }
        }
    }

  fprintf(stream, ")\n{\n");
  fprintf(stream, "  return (uintptr_t)%s(", g_parm[NAME_INDEX]);
  for (i = 0; i < nparms; i++)
    {
      if (i > 0)
        {
          fprintf(stream, ", (%s)parm%d", g_parm[PARM1_INDEX+i], i+1);
        }
      else
        {
          fprintf(stream, "(%s)parm%d", g_parm[PARM1_INDEX+i], i+1);
        }
    }
  fprintf(stream, ");\n}\n");
  fclose(stream);
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s [-p|s] <CSV file>\n", progname);
  exit(1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *csvpath;
  bool proxies = false;
  FILE *stream;
  char *ptr;
  int ch;

  /* Parse command line options */

  g_debug = false;

  while ((ch = getopt(argc, argv, ":dps")) > 0)
    {
      switch (ch)
        {
          case 'd' :
            g_debug = true;
            break;

          case 'p' :
            proxies = true;
            break;

          case 's' :
            proxies = false;
            break;

          case '?' :
            fprintf(stderr, "Unrecognized option: %c\n", optopt);
            show_usage(argv[0]);

          case ':' :
            fprintf(stderr, "Missing option argument, option: %c\n", optopt);
            show_usage(argv[0]);

           break;
            fprintf(stderr, "Unexpected option: %c\n", ch);
            show_usage(argv[0]);
        }
    }

  if (optind >= argc)
    {
       fprintf(stderr, "Missing <CSV file>\n");
       show_usage(argv[0]);      
    }

  csvpath = argv[optind];
  if (++optind < argc)
    {
       fprintf(stderr, "Unexpected garbage at the end of the line\n");
       show_usage(argv[0]);      
    }    

  /* Open the CSV file */

  stream= fopen(csvpath, "r");
  if (!stream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(3);
    }

  /* Process each line in the CVS file */

  while ((ptr = read_line(stream)) != NULL)
    {
      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < 3)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(8);
        }

      if (proxies)
        {
          generate_proxy(nargs-3);
        }
      else
        {
          generate_stub(nargs-3);
        }
    }

  /* Close the CSV file */

  fclose(stream);
  return 0;
}
