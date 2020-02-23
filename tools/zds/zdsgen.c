/****************************************************************************
 * tools/zds/zdsgen.c
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

#include <sys/stat.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>

#ifdef HOST_CYGWIN
#  include <sys/cygwin.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_BUFFER  (4096)
#define MAX_EXPAND  (2048)

/* MAX_PATH might be defined in stdlib.h */

#if !defined(MAX_PATH)
#  define MAX_PATH  (512)
#endif

/* NAME_MAX is typically defined in limits.h */

#if !defined(NAME_MAX)

  /* FILENAME_MAX might be defined in stdio.h */

#  if defined(FILENAME_MAX)
#    define NAME_MAX FILENAME_MAX
#  else

  /* MAXNAMELEN might be defined in dirent.h */

#    include <dirent.h>
#    if defined(MAXNAMLEN)
#      define NAME_MAX MAXNAMLEN
#    else

  /* Lets not let a silly think like this stop us... just make something up */

#      define NAME_MAX 256
#    endif
#  endif
#endif

/* Name of the host.  The ZDS-II toolchain runs only on Windows.  Therefore,
 * the only options are (1) Windows native, or (2) Cygwin or environments
 * that derive for Cygwin (like MSYS2).
 */

#define WINSEPARATOR '\\'

#if defined(HOST_NATIVE)
#  define SEPARATOR '\\'
#  define HOSTNAME  "Native" /* Windows native */
#elif defined(HOST_CYGWIN)
#  define SEPARATOR '/'
#  define HOSTNAME  "Cywgin" /* Cygwin or MSYS under Windows */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum slashmode_e
{
  MODE_FSLASH  = 0,
  MODE_BSLASH  = 1,
  MODE_DBLBACK = 2
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_exe     = NULL;     /* Executable, either compiler or assembler */
static char *g_flags   = NULL;     /* Assembler or compiler flags */
static char *g_infile  = NULL;     /* The input source file */
static char *g_outfile = NULL;     /* The output object file */
static int   g_debug   = 0;

static char g_command[MAX_BUFFER]; /* The command to be executed */
static char g_path[MAX_PATH];      /* Temporary for path generation */
#ifdef HOST_CYGWIN
static char g_expand[MAX_EXPAND];  /* Temporary for path expansion */
static char g_dequoted[MAX_PATH];  /* Temporary for dequoting paths */
static char g_hostpath[MAX_PATH];  /* Temporary for host path conversions */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* MinGW does not seem to provide strtok_r */

#ifndef HAVE_STRTOK_R
static char *my_strtok_r(char *str, const char *delim, char **saveptr)
{
  char *pbegin;
  char *pend = NULL;

  /* Decide if we are starting a new string or continuing from
   * the point we left off.
   */

  if (str)
    {
      pbegin = str;
    }
  else if (saveptr && *saveptr)
    {
      pbegin = *saveptr;
    }
  else
    {
      return NULL;
    }

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(delim, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      return NULL;
    }

  /* Find the end of the token */

  for (pend = pbegin + 1;
       *pend && strchr(delim, *pend) == NULL;
       pend++);

  /* pend either points to the end of the string or to
   * the first delimiter after the string.
   */

  if (*pend)
    {
      /* Turn the delimiter into a null terminator */

      *pend++ = '\0';
    }

  /* Save the pointer where we left off and return the
   * beginning of the token.
   */

  if (saveptr)
    {
      *saveptr = pend;
    }

  return pbegin;
}

#undef strtok_r
#define strtok_r my_strtok_r
#endif

static void append(char **base, char *str)
{
  char *oldbase;
  char *newbase;
  int alloclen;

  oldbase = *base;
  if (!oldbase)
    {
      newbase = strdup(str);
      if (!newbase)
        {
          fprintf(stderr, "ERROR: Failed to strdup %s\n", str);
          exit(EXIT_FAILURE);
        }
    }
  else
    {
      alloclen = strlen(oldbase) + strlen(str) + sizeof((char) ' ') +
                 sizeof((char) '\0');
      newbase = (char *)malloc(alloclen);
      if (!newbase)
        {
          fprintf(stderr, "ERROR: Failed to allocate %d bytes\n", alloclen);
          exit(EXIT_FAILURE);
        }

      snprintf(newbase, alloclen, "%s %s", oldbase, str);
      free(oldbase);
    }

  *base = newbase;
}

static void show_usage(const char *progname, const char *msg, int exitcode)
{
  if (msg)
    {
      fprintf(stderr, "\n");
      fprintf(stderr, "%s:\n", msg);
    }

  fprintf(stderr, "\n");
  fprintf(stderr, "%s  [--debug] <EXE> -- <FLAGS> -- <infile> <outfile>\n",
          progname);
  fprintf(stderr, "%s  [--help]\n\n", progname);
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  <EXE>\n");
  fprintf(stderr, "    A variable number of arguments that define how to"
                  " execute the code\n");
  fprintf(stderr, "    generation tool (either the compiler or the "
                  "assembler)\n");
  fprintf(stderr, "  <FLAGS>\n");
  fprintf(stderr, "    The compiler compilation flags\n");
  fprintf(stderr, "  <inile>\n");
  fprintf(stderr, "    One or more C files whose dependencies will be "
                  "checked.  Each file is expected\n");
  fprintf(stderr, "    to reside in the current directory\n");
  fprintf(stderr, "  <outfile>\n");
  fprintf(stderr, "    The name of the binary output file to be "
                  "generated\n");
  fprintf(stderr, "  --debug\n");
  fprintf(stderr, "    Enable script debug\n");
  fprintf(stderr, "  --help\n");
  fprintf(stderr, "    Shows this message and exits\n");
  exit(exitcode);
}

static void parse_args(int argc, char **argv)
{
  char *args = NULL;
  int argidx;

  /* Parse arguments */

  for (argidx = 1; argidx < argc; argidx++)
    {
      if (strcmp(argv[argidx], "--") == 0)
        {
          g_exe   = g_flags;
          g_flags = args;
          args    = NULL;
        }
      else if (strcmp(argv[argidx], "--debug") == 0)
        {
          g_debug++;
        }
      else if (strcmp(argv[argidx], "--help") == 0)
        {
          show_usage(argv[0], NULL, EXIT_SUCCESS);
        }
      else if (g_exe == NULL)
        {
          append(&args, argv[argidx]);
        }
      else
        {
          break;
        }
    }

  /* The penultimate argument should be the input source file */

  if (argidx < argc)
    {
      g_infile = argv[argidx];
      argidx++;
    }
  else
    {
      show_usage(argv[0], "ERROR: Missing input source file",
                 EXIT_FAILURE);
    }

  /* The last argument should be the output object file */

  if (argidx < argc)
    {
      g_outfile = argv[argidx];
      argidx++;
    }
  else
    {
      show_usage(argv[0], "ERROR: Missing output object file",
                 EXIT_FAILURE);
    }

  if (g_debug)
    {
      fprintf(stderr, "SELECTIONS\n");
      fprintf(stderr, "  Host Environ   : [%s]\n",
              HOSTNAME);
      fprintf(stderr, "  Tool           : [%s]\n",
              g_exe ? g_exe : "(None)");
      fprintf(stderr, "  Tool flags     : [%s]\n",
              g_flags ? g_flags : "(None)");
      fprintf(stderr, "  Source file    : [%s]\n",
              g_infile ? g_infile : "(None)");
      fprintf(stderr, "  Object file    : [%s]\n",
              g_outfile ? g_outfile : "(None)");
    }

  /* Check for required parameters */

  if (g_exe == NULL)
    {
      show_usage(argv[0], "ERROR: No compiler specified", EXIT_FAILURE);
    }

  if (g_infile == NULL)
    {
      /* Don't report an error -- this happens normally in some configurations */

      printf("# No source files specified\n");
      exit(EXIT_SUCCESS);
    }
}

static const char *do_expand(const char *argument)
{
#ifdef HOST_CYGWIN
  const char *src;
  char *dest;
  int len;

  src  = argument;
  dest = g_expand;
  len  = 0;

  while (*src && len < MAX_EXPAND)
    {
      if (*src == '\\')
        {
          /* Copy backslash */

          *dest++ = *src++;
          if (++len >= MAX_EXPAND)
            {
              break;
            }

          /* Already expanded? */

          if (*src == '\\')
            {
              /* Yes... just copy all consecutive backslashes */

              do
                {
                  *dest++ = *src++;
                  if (++len >= MAX_EXPAND)
                    {
                      break;
                    }
                }
              while (*src == '\\');
            }
          else
            {
              /* No.. expeand */

              *dest++ = '\\';
              if (++len >= MAX_EXPAND)
                {
                  break;
                }
            }
        }
      else
        {
          *dest++ = *src++;
          len++;
        }
    }

  if (*src)
    {
      fprintf(stderr, "ERROR: Truncated during expansion string is "
                      "too long [%lu/%u]\n",
              (unsigned long)strlen(argument), MAX_EXPAND);
      exit(EXIT_FAILURE);
    }

  *dest = '\0';
  return g_expand;
#else
  return argument;
#endif
}

#ifdef HOST_CYGWIN
static bool dequote_path(const char *winpath)
{
  char *dest = g_dequoted;
  const char *src = winpath;
  int len = 0;
  bool quoted = false;

  while (*src && len < MAX_PATH)
    {
      if (src[0] != '\\' || (src[1] != ' ' && src[1] != '(' && src[1] != ')'))
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
      fprintf(stderr, "# ERROR: Path truncated\n");
      exit(EXIT_FAILURE);
    }

  *dest = '\0';
  return quoted;
}
#endif

/* If using Cygwin with a Window's Toolchain, then we have to convert the
 * POSIX path to a Windows or POSIX path.
 */

#ifdef HOST_CYGWIN
static const char *convert_path(const char *path, cygwin_conv_path_t what)
{
  const char *retptr;
  ssize_t size;
  ssize_t ret;
  bool quoted;

  quoted = dequote_path(path);
  if (quoted)
    {
      retptr = g_hostpath;
    }
  else
    {
      retptr = &g_hostpath[1];
    }

  size = cygwin_conv_path(what | CCP_RELATIVE, g_dequoted, NULL, 0);
  if (size > (MAX_PATH - 3))
    {
      fprintf(stderr, "# ERROR: POSIX path too long: %lu\n",
              (unsigned long)size);
      exit(EXIT_FAILURE);
    }

  ret = cygwin_conv_path(what | CCP_RELATIVE, g_dequoted,
                         &g_hostpath[1], MAX_PATH - 3);
  if (ret < 0)
    {
      fprintf(stderr, "# ERROR: cygwin_conv_path '%s' failed: %s\n",
              g_dequoted, strerror(errno));
      exit(EXIT_FAILURE);
    }

  if (quoted)
    {
      size++;
      g_hostpath[0] = '"';
      g_hostpath[size] = '"';
    }

  g_hostpath[size + 1] = '\0';
  return retptr;
}
#endif

static const char *convert_path_windows(const char *path)
{
#ifdef HOST_CYGWIN
  return convert_path(path, CCP_POSIX_TO_WIN_A);
#else
  return path;
#endif
}

static const char *convert_path_posix(const char *path)
{
#ifdef HOST_CYGWIN
  return convert_path(path, CCP_WIN_A_TO_POSIX);
#else
  return path;
#endif
}

static void do_generate(void)
{
  const char *hostpath;
  const char *toolpath;
  const char *expanded;
  int cmdlen;
  int ret;

  /* First remove any existing .obj, lst, and .src files at the path of the
   * new output object file.
   */

#warning Missing logic

  /* Copy the tool path into the command buffer.  NOTE that
   * convert_path_posix() is a no-op in Windows native mode.
   */

  hostpath = convert_path_posix(g_exe);
  cmdlen   = strlen(hostpath);
  if (cmdlen + 1 >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Tool string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, hostpath);
      exit(EXIT_FAILURE);
    }

  strcpy(g_command, hostpath);

  /* Followed by a space */

  if (cmdlen + 2 >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: No room for whitespace [%d/%d]: %d\n",
              cmdlen, MAX_BUFFER, 2);
      exit(EXIT_FAILURE);
    }

  g_command[cmdlen]     = ' ';
  g_command[cmdlen + 1] = '\0';
  cmdlen++;

  /* Copy the tool flags into the command buffer */

  if (g_flags)
    {
      expanded = do_expand(g_flags);
      cmdlen  += strlen(expanded);

      if (cmdlen + 1 >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: CFLAG string is too long [%d/%d]: %s\n",
                  cmdlen, MAX_BUFFER, g_flags);
          exit(EXIT_FAILURE);
        }

      strcat(g_command, expanded);
    }

  /* Add a space */

  if (cmdlen + 2 >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: No room for whitespace [%d/%d]: %d\n",
              cmdlen, MAX_BUFFER, 2);
      exit(EXIT_FAILURE);
    }

  g_command[cmdlen]     = ' ';
  g_command[cmdlen + 1] = '\0';
  cmdlen++;

  /* Add the input source file.NOTE that convert_path_windows() is a no-op
   * in Windows native mode.
   */

  toolpath = convert_path_windows(g_infile);
  expanded = do_expand(toolpath);
  cmdlen  += strlen(expanded);

  if (cmdlen + 1 >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Input source file name is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, expanded);
      exit(EXIT_FAILURE);
    }

  strcat(g_command, expanded);

  /* Generate the output object file */

  if (g_debug)
    {
      fprintf(stderr, "Executing: %s\n", g_command);
    }

  ret = system(g_command);
#ifdef WEXITSTATUS
  if (ret < 0 || WEXITSTATUS(ret) != 0)
    {
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: system failed: %s\n", strerror(errno));
        }
      else
        {
          fprintf(stderr, "ERROR: %s failed: %d\n", g_exe, WEXITSTATUS(ret));
        }

      fprintf(stderr, "       command: %s\n", g_command);
      exit(EXIT_FAILURE);
    }
#else
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: system failed: %s\n", strerror(errno));
      fprintf(stderr, "       command: %s\n", g_command);
      exit(EXIT_FAILURE);
    }
#endif

  /* We don't really know that the command succeeded... Let's
   * assume that it did
   */

  /* Now, check if we have to move or rename the output gerated by the
   * compiler/assembly.  We can determine this by the just checking if
   * the selected object file now exists after we deleted it above.  In
   * that case we need to do nothing.
   *
   * In the case where we are moving the object file to sub-directory
   * (like bin/), then we don't need to do that either.  That will be
   * handled by the MOVEOBJ define.
   *
   * REVISIT:  Do we want to remove the MOVEOBJ macro?  We should do
   * that.
   *
   * Otherwise, rename the output object file.
   */

#warning Missing logic
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  /* Parse command line parameters */

  parse_args(argc, argv);

  /* Generate the object file from the source. */

  do_generate();
  return EXIT_SUCCESS;
}
