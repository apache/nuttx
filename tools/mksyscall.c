/****************************************************************************
 * tools/mksyscall.c
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
#include <unistd.h>
#include <errno.h>

#include "csvparser.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_inline;
static FILE *g_stubstream;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool is_vararg(const char *type)
{
  return strcmp(type, "...") == 0;
}

static bool is_union(const char *type)
{
  return strncmp(type, "union ", 6) == 0;
}

static const char *check_funcptr(const char *type)
{
  const char *str = strstr(type, "(*)");
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

  /* Function pointers and array formal parameter types are a little more
   * work
   */

  if ((part2 = check_funcptr(argtype)) != NULL ||
      (part2 = check_array(argtype)) != NULL)
    {
      len = part2 - argtype;
      fwrite(argtype, 1, len, stream);
      fprintf(stream, "parm%d%s", parmno, part2);
    }
  else
    {
      fprintf(stream, "%s parm%d", argtype, parmno);
    }
}

static void get_formalparmtype(const char *arg, char *formal)
{
  /* The formal parm type is a pointer to everything up to the first'|' (or
   * the end of the string if there is no '|' in the type description).
   */

  while (*arg != '|' && *arg != '\0')
    {
      *formal++ = *arg++;
    }

  *formal   = '\0';
}

static void get_actualparmtype(const char *arg, char *actual)
{
  const char *pstart = strchr(arg, '|');
  if (pstart)
    {
      /* The actual parameter type starts after the '|' */

      pstart++;
    }
  else
    {
      /* The actual parameter is the same as the formal parameter
       * at starts at the beginning of the type string.
       */

      pstart = arg;
    }

  /* The actual parm type is a pointer to everything up to the next '|' (or
   * the end of the string if there is no '|' in the type description).
   */

  while (*pstart != '|' && *pstart != '\0')
    {
      *actual++ = *pstart++;
    }

  *actual   = '\0';
}

static void get_fieldname(const char *arg, char *fieldname)
{
  char *pactual = strchr(arg, '|');
  char *pstart;

  if (pactual)
    {
      /* The actual parameter type starts after the '|' */

      pactual++;
      pstart = strchr(pactual, '|');
      if (pstart)
        {
          /* The fieldname is everything past the second '|' to the end of
           * the string
           */

          pstart++;
          strncpy(fieldname, pstart, MAX_PARMSIZE - 1);
          return;
        }
    }

  fprintf(stderr, "%d: Missing union fieldname: %s\n", g_lineno, arg);
  exit(15);
}

static FILE *open_proxy(void)
{
  char filename[MAX_PARMSIZE + 10];
  FILE *stream;

  snprintf(filename, MAX_PARMSIZE + 9, "PROXY_%s.c", g_parm[NAME_INDEX]);
  filename[MAX_PARMSIZE + 9] = '\0';

  stream = fopen(filename, "w");
  if (stream == NULL)
    {
      fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
      exit(10);
    }

  return stream;
}

static void generate_proxy(int nfixed, int nparms)
{
  FILE *stream = open_proxy();
  char formal[MAX_PARMSIZE];
  char actual[MAX_PARMSIZE];
  char fieldname[MAX_PARMSIZE];
  int i = 0;

  /* Generate "up-front" information, include correct header files */

  fprintf(stream, "/* Auto-generated %s proxy file -- do not edit */\n\n",
          g_parm[NAME_INDEX]);
  fprintf(stream, "#include <nuttx/config.h>\n");

  /* Suppress "'noreturn' function does return" warnings. */

  fprintf(stream, "#include <nuttx/compiler.h>\n");
  fprintf(stream, "#undef noreturn_function\n");
  fprintf(stream, "#define noreturn_function\n");

  /* Does this function have a variable number of parameters?  If so then the
   * final parameter type will be encoded as "..."
   */

  if (nfixed != nparms)
    {
      fprintf(stream, "#include <stdarg.h>\n");
    }

  if (g_parm[HEADER_INDEX] && strlen(g_parm[HEADER_INDEX]) > 0)
    {
      fprintf(stream, "#include <%s>\n", g_parm[HEADER_INDEX]);
    }

  fprintf(stream, "#include <syscall.h>\n\n");

  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "#if %s\n\n", g_parm[COND_INDEX]);
    }

  /* Generate the function definition that matches standard function
   * prototype
   */

  fprintf(stream, "%s %s(", g_parm[RETTYPE_INDEX], g_parm[NAME_INDEX]);

  /* Generate the formal parameter list */

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nfixed; i++)
        {
          /* The formal and actual parameter types may be encoded.. extra the
           * formal parameter type.
           */

          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);

          /* Arguments after the first must be separated from the preceding
           * parameter with a comma.
           */

          if (i > 0)
            {
              fprintf(stream, ", ");
            }

          print_formalparm(stream, formal, i + 1);
        }
    }

  /* Handle the end of the formal parameter list */

  if (i < nparms)
    {
      fprintf(stream, ", ...)\n{\n");

      /* Get parm variables .. some from the parameter list and others from
       * the varargs.
       */

      fprintf(stream, "  va_list ap;\n");
      for (; i < nparms; i++)
        {
          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);
          fprintf(stream, "  %s parm%d;\n", formal, i + 1);
        }

      fprintf(stream, "\n  va_start(ap, parm%d);\n", nfixed);

      for (i = nfixed; i < nparms; i++)
        {
          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);
          get_actualparmtype(g_parm[PARM1_INDEX + i], actual);

          if (is_union(formal))
            {
              fprintf(stream, "  parm%d = (%s)va_arg(ap, %s);\n",
                      i + 1, formal, actual);
            }
          else
            {
              fprintf(stream, "  parm%d = va_arg(ap, %s);\n", i + 1, actual);
            }
        }

      fprintf(stream, "  va_end(ap);\n\n");
    }
  else
    {
      fprintf(stream, ")\n{\n");
    }

  /* Generate the system call.  Functions that do not return or return void
   * are special cases.
   */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "  (void)sys_call%d(", nparms);
    }
  else
    {
      fprintf(stream, "  return (%s)sys_call%d(", g_parm[RETTYPE_INDEX],
              nparms);
    }

  /* Create the parameter list with the matching types.  The first parameter
   * is always the syscall number.
   */

  fprintf(stream, "(unsigned int)SYS_%s", g_parm[NAME_INDEX]);

  for (i = 0; i < nparms; i++)
    {
      /* Is the parameter a union member */

      if (is_union(g_parm[PARM1_INDEX + i]))
        {
          /* Then we will have to pick a field name that can be cast to a
           * uintptr_t.  There probably should be some error handling here
           * to catch the case where the fieldname was not supplied.
           */

          get_fieldname(g_parm[PARM1_INDEX + i], fieldname);
          fprintf(stream, ", (uintptr_t)parm%d.%s", i + 1, fieldname);
        }
      else
        {
          fprintf(stream, ", (uintptr_t)parm%d", i + 1);
        }
    }

  /* Handle the tail end of the function. */

  fprintf(stream, ");\n}\n");
  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "\n#endif /* %s */\n", g_parm[COND_INDEX]);
    }

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
              fprintf(stderr, "Failed to open STUB.h: %s\n",
                      strerror(errno));
              exit(9);
            }

          fprintf(g_stubstream, "/* Autogenerated STUB header file */\n\n");
          fprintf(g_stubstream, "#ifndef __STUB_H\n");
          fprintf(g_stubstream, "#define __STUB_H\n\n");
        }

      return g_stubstream;
    }
  else
    {
      char filename[MAX_PARMSIZE + 8];
      FILE *stream;

      snprintf(filename, MAX_PARMSIZE + 7, "STUB_%s.c", g_parm[NAME_INDEX]);
      filename[MAX_PARMSIZE + 7] = '\0';

      stream = fopen(filename, "w");

      if (stream == NULL)
        {
          fprintf(stderr, "Failed to open %s: %s\n", filename,
                  strerror(errno));
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

static void generate_stub(int nfixed, int nparms)
{
  FILE *stream = open_stub();
  char formal[MAX_PARMSIZE];
  char actual[MAX_PARMSIZE];
  int i;

  /* Generate "up-front" information, include correct header files */

  fprintf(stream, "/* Auto-generated %s stub file -- do not edit */\n\n",
          g_parm[0]);
  fprintf(stream, "#include <nuttx/config.h>\n");
  fprintf(stream, "#include <stdint.h>\n");

  if (g_parm[HEADER_INDEX] && strlen(g_parm[HEADER_INDEX]) > 0)
    {
      fprintf(stream, "#include <%s>\n", g_parm[HEADER_INDEX]);
    }

  putc('\n', stream);

  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "#if %s\n\n", g_parm[COND_INDEX]);
    }

  /* Generate the function definition that matches standard function
   * prototype
   */

  if (g_inline)
    {
      fprintf(stream, "static inline ");
    }

  fprintf(stream, "uintptr_t STUB_%s(int nbr", g_parm[NAME_INDEX]);

  /* Generate the formal parameter list */

  for (i = 0; i < nparms; i++)
    {
      fprintf(stream, ", uintptr_t parm%d", i + 1);
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
       * actually have to cast to.  For example for a formal type like
       * 'int parm[]' we have to cast the actual parameter to 'int *'.
       * The worst is a union type like 'union sigval' where we have to
       * cast to (union sigval)((FAR void *)parm)
       * -- Yech.
       */

     get_formalparmtype(g_parm[PARM1_INDEX + i], formal);
     get_actualparmtype(g_parm[PARM1_INDEX + i], actual);

      /* Treat the first argument in the list differently from the others..
       * It does not need a comma before it.
       */

      if (i > 0)
        {
          fprintf(stream, ", ");
        }

      if (is_union(formal))
        {
          fprintf(stream, "(%s)((%s)parm%d)", formal, actual, i + 1);
        }
      else
        {
          fprintf(stream, "(%s)parm%d", actual, i + 1);
        }
    }

  /* Tail end of the function.  If the stubs function has no return
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

  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "\n#endif /* %s */\n", g_parm[COND_INDEX]);
    }

  stub_close(stream);
}

static FILE *open_wrapper(void)
{
  char filename[MAX_PARMSIZE + 8];
  FILE *stream;

  snprintf(filename, MAX_PARMSIZE + 7, "WRAP_%s.c", g_parm[NAME_INDEX]);
  filename[MAX_PARMSIZE + 7] = '\0';

  stream = fopen(filename, "w");

  if (stream == NULL)
    {
      fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
      exit(10);
    }

  return stream;
}

static void generate_wrapper(int nfixed, int nparms)
{
  FILE *stream = open_wrapper();
  char formal[MAX_PARMSIZE];
  char actual[MAX_PARMSIZE];
  char fieldname[MAX_PARMSIZE];
  int i = 0;

  /* Generate "up-front" information, include correct header files */

  fprintf(stream, "/* Auto-generated %s wrap file -- do not edit */\n\n",
          g_parm[NAME_INDEX]);
  fprintf(stream, "#include <nuttx/config.h>\n");
  fprintf(stream, "#include <nuttx/sched_note.h>\n");
  fprintf(stream, "#include <stdint.h>\n");

  /* Suppress "'noreturn' function does return" warnings. */

  fprintf(stream, "#include <nuttx/compiler.h>\n");
  fprintf(stream, "#undef noreturn_function\n");
  fprintf(stream, "#define noreturn_function\n");

  /* CONFIG_LIB_SYSCALL must be defined to get syscall number */

  fprintf(stream, "#undef CONFIG_LIB_SYSCALL\n");
  fprintf(stream, "#define CONFIG_LIB_SYSCALL\n");
  fprintf(stream, "#include <syscall.h>\n");

  /* Does this function have a variable number of parameters?  If so then the
   * final parameter type will be encoded as "..."
   */

  if (nfixed != nparms)
    {
      fprintf(stream, "#include <stdarg.h>\n");
    }

  if (g_parm[HEADER_INDEX] && strlen(g_parm[HEADER_INDEX]) > 0)
    {
      fprintf(stream, "#include <%s>\n", g_parm[HEADER_INDEX]);
    }

  /* Define macros to get wrapper symbol */

  fprintf(stream, "#include <nuttx/arch.h>\n");
  fprintf(stream, "#ifndef UP_WRAPSYM\n");
  fprintf(stream, "#define UP_WRAPSYM(s) __wrap_##s\n");
  fprintf(stream, "#endif\n");
  fprintf(stream, "#ifndef UP_REALSYM\n");
  fprintf(stream, "#define UP_REALSYM(s) __real_##s\n");
  fprintf(stream, "#endif\n\n");

  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "#if %s\n\n", g_parm[COND_INDEX]);
    }

  /* Generate the wrapper function definition that matches standard function
   * prototype
   */

  fprintf(stream, "%s UP_WRAPSYM(%s)(", g_parm[RETTYPE_INDEX],
          g_parm[NAME_INDEX]);

  /* Generate the formal parameter list */

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nfixed; i++)
        {
          /* The formal and actual parameter types may be encoded.. extra the
           * formal parameter type.
           */

          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);

          /* Arguments after the first must be separated from the preceding
           * parameter with a comma.
           */

          if (i > 0)
            {
              fprintf(stream, ", ");
            }

          print_formalparm(stream, formal, i + 1);
        }
    }

  if (i < nparms)
    {
       fprintf(stream, ", ...)\n{\n");
    }
  else
    {
      fprintf(stream, ")\n{\n");
    }

  /* Generate the result variable definition for non-void function */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") != 0)
    {
      fprintf(stream, "  %s result;\n", g_parm[RETTYPE_INDEX]);
    }

  /* Generate the wrapped (real) function prototype definition */

  fprintf(stream, "  %s UP_REALSYM(%s)(", g_parm[RETTYPE_INDEX],
          g_parm[NAME_INDEX]);

  /* Generate the formal parameter list */

  if (nparms <= 0)
    {
      fprintf(stream, "void");
    }
  else
    {
      for (i = 0; i < nfixed; i++)
        {
          /* The formal and actual parameter types may be encoded.. extra the
           * formal parameter type.
           */

          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);

          /* Arguments after the first must be separated from the preceding
           * parameter with a comma.
           */

          if (i > 0)
            {
              fprintf(stream, ", ");
            }

          fprintf(stream, "%s", formal);
        }
    }

  /* Handle the end of the formal parameter list */

  if (i < nparms)
    {
      fprintf(stream, ", ...);\n");

      /* Get parm variables .. some from the parameter list and others from
       * the varargs.
       */

      fprintf(stream, "  va_list ap;\n");
      for (; i < nparms; i++)
        {
          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);
          fprintf(stream, "  %s parm%d;\n", formal, i + 1);
        }

      fprintf(stream, "\n  va_start(ap, parm%d);\n", nfixed);

      for (i = nfixed; i < nparms; i++)
        {
          get_formalparmtype(g_parm[PARM1_INDEX + i], formal);
          get_actualparmtype(g_parm[PARM1_INDEX + i], actual);

          if (is_union(formal))
            {
              fprintf(stream, "  parm%d = (%s)va_arg(ap, %s);\n",
                      i + 1, formal, actual);
            }
          else
            {
              fprintf(stream, "  parm%d = va_arg(ap, %s);\n", i + 1, actual);
            }
        }

      fprintf(stream, "  va_end(ap);\n");
    }
  else
    {
      fprintf(stream, ");\n");
    }

  /* Call system call enter hook function */

  fprintf(stream, "\n  sched_note_syscall_enter(SYS_%s, %d",
          g_parm[NAME_INDEX], nparms);

  for (i = 0; i < nparms; i++)
    {
      /* Is the parameter a union member */

      if (is_union(g_parm[PARM1_INDEX + i]))
        {
          /* Then we will have to pick a field name that can be cast to a
           * uintptr_t.  There probably should be some error handling here<
           * to catch the case where the fieldname was not supplied.
           */

          get_fieldname(g_parm[PARM1_INDEX + i], fieldname);
          fprintf(stream, ", (uintptr_t)parm%d.%s", i + 1, fieldname);
        }
      else
        {
          fprintf(stream, ", (uintptr_t)parm%d", i + 1);
        }
    }

  fprintf(stream, ");\n\n");

  /* Then call the wrapped (real) function.  Functions that have no return
   * value are a special case.
   */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "  UP_REALSYM(%s)(", g_parm[NAME_INDEX]);
    }
  else
    {
      fprintf(stream, "  result = UP_REALSYM(%s)(", g_parm[NAME_INDEX]);
    }

  /* The pass all of the system call parameters */

  for (i = 0; i < nparms; i++)
    {
      /* Treat the first argument in the list differently from the others..
       * It does not need a comma before it.
       */

      if (i > 0)
        {
          fprintf(stream, ", ");
        }

      fprintf(stream, "parm%d", i + 1);
    }

  fprintf(stream, ");\n\n");

  /* Call system call leave hook function */

  fprintf(stream, "  sched_note_syscall_leave(SYS_%s, ", g_parm[NAME_INDEX]);

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "0");
    }
  else
    {
      fprintf(stream, "(uintptr_t)result");
    }

  fprintf(stream, ");\n\n");

  /* Tail end of the function.  If the wrapped (real) function has no return
   * value, do nothing.
   */

  if (strcmp(g_parm[RETTYPE_INDEX], "void") == 0)
    {
      fprintf(stream, "}\n");
    }
  else
    {
      fprintf(stream, "  return result;\n}\n");
    }

  if (g_parm[COND_INDEX][0] != '\0')
    {
      fprintf(stream, "\n#endif /* %s */\n", g_parm[COND_INDEX]);
    }

  fclose(stream);
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s [-p|s|i] <CSV file>\n\n", progname);
  fprintf(stderr, "Where:\n\n");
  fprintf(stderr, "\t-p : Generate proxies\n");
  fprintf(stderr, "\t-s : Generate stubs\n");
  fprintf(stderr, "\t-i : Generate proxies as static inline functions\n");
  fprintf(stderr, "\t-w : Generate wrappers\n");
  fprintf(stderr, "\t-d : Enable debug output\n");
  exit(1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *csvpath;
  bool proxies = false;
  bool wrappers = false;
  FILE *stream;
  char *ptr;
  int ch;
  int i;

  /* Parse command line options */

  g_debug = false;
  g_inline = false;

  while ((ch = getopt(argc, argv, ":dpsw")) > 0)
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

          case 'w' :
            wrappers = true;
            break;

          case '?' :
            fprintf(stderr, "Unrecognized option: %c\n", optopt);
            show_usage(argv[0]);

          case ':' :
            fprintf(stderr, "Missing option argument, option: %c\n", optopt);
            show_usage(argv[0]);

          default:
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

  stream = fopen(csvpath, "r");
  if (!stream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(3);
    }

  /* Process each line in the CVS file */

  while ((ptr = read_line(stream)) != NULL)
    {
      int nfixed;

      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < PARM1_INDEX)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(8);
        }

      /* Assume no variable arguments by default */

      nfixed = nargs - PARM1_INDEX;

      /* Search for an occurrence of "...".  This is followed by the list
       * types in the variable arguments.  The number of types is the
       * maximum number of variable arguments.
       */

      for (i = PARM1_INDEX; i < nargs; i++)
        {
          if (is_vararg(g_parm[i]))
            {
              /* "..." is the last argument? */

              if (i == --nargs)
                {
                  /* Yes, generate the default variable arguments */

                  while (nargs < PARM1_INDEX + 6)
                    {
                      strcpy(g_parm[nargs++], "uintptr_t");
                    }
                }
              else
                {
                  /* Move up one slot to overwrite "..." */

                  memmove(g_parm[i], g_parm[i + 1],
                          sizeof(g_parm[i]) * (nargs - i));
                }

              nfixed = i - PARM1_INDEX;
              break;
            }
        }

      if (proxies)
        {
          generate_proxy(nfixed, nargs - PARM1_INDEX);
        }
      else if (wrappers)
        {
          generate_wrapper(nfixed, nargs - PARM1_INDEX);
        }
      else
        {
          g_stubstream = NULL;
          generate_stub(nfixed, nargs - PARM1_INDEX);
          if (g_stubstream != NULL)
            {
              fprintf(g_stubstream, "\n#endif /* __STUB_H */\n");
              fclose(g_stubstream);
            }
        }
    }

  /* Close the CSV file */

  fclose(stream);
  return 0;
}
