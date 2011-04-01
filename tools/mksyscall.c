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
static bool g_inline;
static char g_line[LINESIZE+1];
static char g_parm[MAX_FIELDS][MAX_PARMSIZE];
static FILE *g_stubstream;

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

static bool is_vararg(const char *type, int index, int nparms)
{
  if (strcmp(type,"...") == 0)
    {
      if (index != (nparms-1))
        {
          fprintf(stderr, "... is not the last in the argument list\n");
          exit(11);
        }
      return true;
    }
  return false;
}

static bool is_union(const char *type)
{
  return (strncmp(type,"union", 5) == 0);
}

static const char *check_funcptr(const char *type)
{
  const char *str = strstr(type,"(*)");
  if (str)
    {
      return str + 2;
    }
  return NULL;
}

static const char *check_array(const char *type)
{
  const char *str = strchr(type, '[');
  if (str)
    {
      return str;
    }
  return NULL;
}

static void print_formalparm(FILE *stream, const char *argtype, int parmno)
{
  const char *part2;
  int len;

  /* Function pointers and array formal parameter types are a little more work */

  if ((part2 = check_funcptr(argtype)) != NULL || (part2 = check_array(argtype)) != NULL)
    {
      len = part2 - argtype;
      (void)fwrite(argtype, 1, len, stream);
      fprintf(stream, "parm%d%s", parmno, part2);
    }
  else
    {
      fprintf(stream, "%s parm%d", argtype, parmno);
    }
}

static void get_formalparmtype(const char *arg, char *formal)
{
  char *ptr = strchr(arg,'|');
  if (ptr)
    {
      /* The formal parm type is a pointer to everything up to the '|' */

      while (*arg != '|')
        {
          *formal++ = *arg++;
        }
      *formal   = '\0';
    }
  else
    {
      strncpy(formal, arg, MAX_PARMSIZE);
    }
}

static void get_actualparmtype(const char *arg, char *actual)
{
  char *ptr = strchr(arg,'|');
  if (ptr)
    {
      ptr++;
      strncpy(actual, ptr, MAX_PARMSIZE);
    }
  else
    {
      strncpy(actual, arg, MAX_PARMSIZE);
    }
}

static FILE *open_proxy(void)
{
  char filename[MAX_PARMSIZE+10];
  FILE *stream;

  snprintf(filename, MAX_PARMSIZE+9, "PROXY_%s.c", g_parm[NAME_INDEX]);
  filename[MAX_PARMSIZE+9] = '\0';

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
  char formal[MAX_PARMSIZE];
  int i;

  /* Generate "up-front" information, include correct header files */

  fprintf(stream, "/* Auto-generated %s proxy file -- do not edit */\n\n", g_parm[NAME_INDEX]);
  fprintf(stream, "#include <%s>\n", g_parm[HEADER_INDEX]);
  fprintf(stream, "#include <syscall.h>\n\n");

  /* Generate the function definition that matches standard function prototype */

  fprintf(stream, "%s %s(", g_parm[RETTYPE_INDEX], g_parm[NAME_INDEX]);

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nparms; i++)
        {
          get_formalparmtype(g_parm[PARM1_INDEX+i], formal);
          if (i > 0)
            {
              fprintf(stream, ", ");
            }
          print_formalparm(stream, formal, i+1);
        }
    }
  fprintf(stream, ")\n{\n");

  /* Generate the system call.  Functions that do not return or return void are special cases */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "  (void)sys_call%d(", nparms);
    }
  else
    {
      fprintf(stream, "  return (%s)sys_call%d(", g_parm[RETTYPE_INDEX], nparms);
    }

  /* Create the parameter list with the matching types.  The first parametr is always the syscall number. */

  fprintf(stream, "(unsigned int)SYS_%s", g_parm[NAME_INDEX]);

  for (i = 0; i < nparms; i++)
    {
      fprintf(stream, ", (uintptr_t)parm%d", i+1);
    }

  /* Handle the tail end of the function. */

  fprintf(stream, ");\n}\n");
  fclose(stream);
}

static FILE *open_stub(void)
{
  if (g_inline)
    {
      if (!g_stubstream)
        {
          g_stubstream = fopen("STUB.h", "w");
          if (g_stubstream == NULL)
            {
              fprintf(stderr, "Failed to open STUB.h: %s\n", strerror(errno));
              exit(9);
            }
        }

      return g_stubstream;
    }
  else
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
}

static void stub_close(FILE *stream)
{
  if (!g_inline)
    {
      fclose(stream);
    }
}

static void generate_stub(int nparms)
{
  FILE *stream = open_stub();
  char formal[MAX_PARMSIZE];
  char actual[MAX_PARMSIZE];
  int i;
  int j;

  /* Generate "up-front" information, include correct header files */

  fprintf(stream, "/* Auto-generated %s stub file -- do not edit */\n\n", g_parm[0]);
  fprintf(stream, "#include <stdint.h>\n");
  fprintf(stream, "#include <%s>\n\n", g_parm[HEADER_INDEX]);

  /* Generate the function definition that matches standard function prototype */

  fprintf(stream, "uintptr_t STUB_%s(", g_parm[NAME_INDEX]);

  /* Generate the formal parameter list.  A function received no parameters is a special case. */

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nparms; i++)
        {
          /* Treat the first argument in the list differently from the others..
           * It does not need a comma before it.
           */

          if (i > 0)
            {
              /* Check for a variable number of arguments */

              if (is_vararg(g_parm[PARM1_INDEX+i], i, nparms))
                {
                  /* Always receive six arguments in this case */

                  for (j = i+1; j <=6; j++)
                    {
                      fprintf(stream, ", uintptr_t parm%d", j);
                    }
                }
              else
                {
                  fprintf(stream, ", uintptr_t parm%d", i+1);
                }
            }
          else
            {
              fprintf(stream, "uintptr_t parm%d", i+1);
            }
        }
    }
  fprintf(stream, ")\n{\n");

  /* Then call the proxied function.  Functions that have no return value are
   * a special case.
   */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "  %s(", g_parm[NAME_INDEX]);
    }
  else
    {
      fprintf(stream, "  return (uintptr_t)%s(", g_parm[NAME_INDEX]);
    }

  /* The pass all of the system call parameters, casting to the correct type
   * as necessary.
   */

  for (i = 0; i < nparms; i++)
    {
      /* Get the formal type of the parameter, and get the type that we
       * actually have to cast to.  For example for a formal type like 'int parm[]'
       * we have to cast the actual parameter to 'int*'.  The worst is a union
       * type like 'union sigval' where we have to cast to (union sigval)((FAR void *)parm)
       * -- Yech.
       */

     get_formalparmtype(g_parm[PARM1_INDEX+i], formal);
     get_actualparmtype(g_parm[PARM1_INDEX+i], actual);

      /* Treat the first argument in the list differently from the others..
       * It does not need a comma before it.
       */

      if (i > 0)
        {
          /* Check for a variable number of arguments */

          if (is_vararg(actual, i, nparms))
            {
              /* Always pass six arguments */

              for (j = i+1; j <=6; j++)
                {
                  fprintf(stream, ", parm%d", j);
                }
            }
          else
            {
              if (is_union(formal))
                {
                  fprintf(stream, ", (%s)((%s)parm%d)", formal, actual, i+1);
                }
              else
                {
                  fprintf(stream, ", (%s)parm%d", actual, i+1);
                }
            }
        }
      else
        {
          if (is_union(formal))
            {
              fprintf(stream, "(%s)((%s)parm%d)", formal, actual, i+1);
            }
          else
            {
              fprintf(stream, "(%s)parm%d",actual, i+1);
            }
        }
    }

  /* Tail end of the function.  The the proxied function has no return
   * value, just return zero (OK).
   */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, ");\n  return 0;\n}\n");
    }
  else
    {
      fprintf(stream, ");\n}\n");
    }
  stub_close(stream);
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s [-p|s|i] <CSV file>\n\n", progname);
  fprintf(stderr, "Where:\n\n");
  fprintf(stderr, "\t-p : Generate proxies\n");
  fprintf(stderr, "\t-s : Generate stubs\n");
  fprintf(stderr, "\t-i : Generate proxies as static inline functions\n");
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
  g_inline = false;

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

          case 'i' :
            g_inline = true;
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
          g_stubstream = NULL;
          generate_stub(nargs-3);
          if (g_stubstream != NULL)
            {
              fclose(g_stubstream);
            }
        }
    }

  /* Close the CSV file */

  fclose(stream);
  return 0;
}
