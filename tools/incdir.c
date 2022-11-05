/****************************************************************************
 * tools/incdir.c
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
#ifndef CONFIG_WINDOWS_NATIVE
#include <sys/utsname.h>
#endif

#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>

#ifdef HOST_CYGWIN
#  include <sys/cygwin.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum pathtype_e
{
  USER_PATH = 0,
  SYSTEM_PATH
};

enum os_e
{
  OS_UNKNOWN = 0,
  OS_LINUX,
  OS_WINDOWS,
  OS_CYGWIN,
  OS_MSYS,
  OS_WSL,
  OS_MACOS,
  OS_BSD
};

enum compiler_e
{
  COMPILER_UNKNOWN = 0,
  COMPILER_GCC,
  COMPILER_CLANG,
  COMPILER_MINGW,
  COMPILER_SDCC,
  COMPILER_ZDSII
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_advice(const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s [-h] [-w] [-s] <compiler-path> "
                  "<dir1> [<dir2> [<dir3> ...]]\n",
                  progname);
  fprintf(stderr, "Try '%s -h' for more information\n", progname);

  exit(exitcode);
}

static void show_help(const char *progname, int exitcode)
{
  fprintf(stderr, "%s is a tool for flexible generation of include path "
                  "arguments for a\n",
                  progname);
  fprintf(stderr, "variety of different compilers in a variety of "
                  "compilation environments\n");
  fprintf(stderr, "\nUSAGE: %s [-w] [-s] <compiler-path> "
                  "<dir1> [<dir2> [<dir3> ...]]\n",
                  progname);
  fprintf(stderr, "       %s -h\n\n", progname);

  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  <compiler-path>\n");
  fprintf(stderr, "    The full path to your compiler\n");
  fprintf(stderr, "  <dir1> [<dir2> [<dir3> ...]]\n");
  fprintf(stderr, "    A list of include directories\n");
  fprintf(stderr, "  -w\n");
  fprintf(stderr, "    The compiler is a Windows native tool and requires "
                  "Windows\n");
  fprintf(stderr, "    style pathnames like C:\\Program Files\n");
  fprintf(stderr, "  -s\n");
  fprintf(stderr, "    Generate standard, system header file paths instead "
                  "of normal user\n");
  fprintf(stderr, "    header file paths.\n");
  fprintf(stderr, "  -h\n");
  fprintf(stderr, "    Shows this help text and exits.\n");

  exit(exitcode);
}

static enum os_e get_os(char *ccname)
{
#ifdef CONFIG_WINDOWS_NATIVE
  /* Check for MinGW which implies a Windows native environment */

  if (strstr(ccname, "mingw") != NULL)
    {
      return OS_WINDOWS;
    }
#else
  struct utsname buf;
  int ret;
  
  /* Get the context names */

  ret = uname(&buf);
  if (ret < 0)
    {
      int errcode = errno;
      fprintf(stderr, "ERROR: uname failed: %s\n", strerror(errcode));
      exit(EXIT_FAILURE);
    }

  if (strcmp(buf.sysname, "Linux") == 0)
    {
      return OS_LINUX;  /* Or OS_WSL */
    }
  else if (strncmp(buf.sysname, "CYGWIN", 6) == 0)
    {
      return OS_CYGWIN;
    }
  else if (strncmp(buf.sysname, "MINGW", 5) == 0)
    {
      return OS_CYGWIN;
    }
  else if (strncmp(buf.sysname, "MSYS", 4) == 0)
    {
      return OS_CYGWIN;
    }
  else if (strcmp(buf.sysname, "Darwin") == 0)
    {
      return OS_MACOS;
    }
  else if (strcmp(buf.sysname, "FreeBSD") == 0 ||
           strcmp(buf.sysname, "OpenBSD") == 0 ||
           strcmp(buf.sysname, "GNU/kFreeBSD") == 0)
    {
      return OS_BSD;
    }
  else
    {
      fprintf(stderr, "ERROR:  Unknown operating system: %s\n",
              buf.sysname);
    }
#endif
  return OS_UNKNOWN;
}

static enum compiler_e get_compiler(char *ccname, enum os_e os)
{
  /* Let's assume that all GCC compiler paths contain the string gcc or
   * g++ and no non-GCC compiler paths include these substrings.
   *
   * If the compiler is called cc, let's assume that is GCC too.
   */

  if (strstr(ccname, "gcc")     != NULL ||
      strstr(ccname, "g++")     != NULL ||
      strncmp(ccname, "cc.", 3) == 0)
    {
      return COMPILER_GCC;
    }
  else if (strstr(ccname, "clang") != NULL)
    {
      return COMPILER_CLANG;
    }
  else if (strstr(ccname, "sdcc") != NULL)
    {
      return COMPILER_SDCC;
    }
  else if (strstr(ccname, "mingw") != NULL)
    {
      return COMPILER_MINGW;
    }
  else if (strstr(ccname, "ez8cc") != NULL ||
           strstr(ccname, "zneocc") != NULL ||
           strstr(ccname, "ez80cc") != NULL)
    {
      return COMPILER_ZDSII;
    }
  else
    {
      /* Unknown compiler. Assume GCC-compatible */

      return COMPILER_GCC;
    }
}

static int my_asprintf(char **strp, const char *fmt, ...)
{
  va_list ap;
  ssize_t bufsize;
  char *buffer;

  /* Get the size of the buffer */

  va_start(ap, fmt);
  bufsize = vsnprintf(NULL, 0, fmt, ap);
  va_end(ap);

  if (bufsize <= 0)
    {
      fprintf(stderr, "ERROR: vsnprintf() failed.\n");
      exit (EXIT_FAILURE);
    }

  buffer = malloc(bufsize + 1);
  if (buffer == NULL)
    {
      fprintf(stderr, "ERROR: Failed allocated vsnprintf() buffer.\n");
      exit (EXIT_FAILURE);
    }

  va_start(ap, fmt);
  vsnprintf(buffer, bufsize + 1, fmt, ap);
  va_end(ap);

  *strp = buffer;
  return bufsize;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
#ifdef HOST_CYGWIN
  char *convpath = NULL;
  bool wintool = false;
#endif
  enum pathtype_e pathtype = USER_PATH;
  enum os_e os;
  enum compiler_e compiler;
  const char *progname = argv[0];
  const char *cmdarg;
  char *ccname;
  char * const *dirlist;
  size_t respsize = 0;
  char *response = NULL;
  int ndirs;
  int ret;
  int ch;
  int i;

  /* Handle command line options */

  while ((ch = getopt(argc, argv, "wsh")) >= 0)
    {
      switch (ch)
        {
          case 'w':
#ifdef HOST_CYGWIN
          wintool = true;
#endif
          break;

          case 's':
          pathtype = SYSTEM_PATH;
          break;

          case 'h':
            show_help(progname, EXIT_SUCCESS);
        }
    }

  if (optind >= argc)
    {
      fprintf(stderr, "ERROR:  Missing <compiler-path>\n");
      show_advice(progname, EXIT_FAILURE);
    }

  ccname = basename(argv[optind]);
  optind++;

  if (optind >= argc)
    {
      fprintf(stderr, "ERROR:  At least one directory must be supplied\n");
      show_advice(progname, EXIT_FAILURE);
    }

  dirlist = &argv[optind];
  ndirs   = argc - optind;

  /* Most compilers support CFLAG options like '-I<dir>' to add include
   * file header paths.  Some (like the Zilog tools), do not.  This script
   * makes the selection of header file paths compiler independent.
   *
   * Below are all known compiler names (as found in the board/ Make.defs
   * files).  If a new compiler is used that has some unusual syntax, then
   * additional logic needs to be added to this file.
   *
   *   NAME                        Syntax
   *   $(CROSSDEV)gcc              -I<dir1> -I<dir2> -I<dir3> ...
   *   sdcc                        -I<dir2> -I<dir2> -I<dir3> ...
   *   $(ZDSBINDIR)/ez8cc.exe      -usrinc:'<dir1>:<dir2>:<dir3>:...`
   *   $(ZDSBINDIR)/zneocc.exe     -usrinc:'<dir1>:<dir2>:<dir3>:...`
   *   $(ZDSBINDIR)/ez80cc.exe     -usrinc:'<dir1>:<dir2>:<dir3>:...`
   *
   * Furthermore, just to make matters more difficult, with Windows based
   * toolchains, we have to use the full windows-style paths to the header
   * files.
   */

  os = get_os(ccname);
  if (os == OS_UNKNOWN)
    {
      fprintf(stderr, "ERROR:  Operating system not recognized\n");
      show_advice(progname, EXIT_FAILURE);
    }

  compiler = get_compiler(ccname, os);
  if (compiler == COMPILER_UNKNOWN)
    {
      fprintf(stderr, "ERROR:  Compiler not recognized.\n");
      show_advice(progname, EXIT_FAILURE);
    }

  /* Select system or user header file path command line option */

  if (compiler == COMPILER_ZDSII)
    {
      cmdarg = (pathtype == SYSTEM_PATH) ? "-stdinc:" : "-usrinc:";
#ifdef HOST_CYGWIN
      wintool = true;
#endif
    }
  else if (compiler == COMPILER_SDCC)
    {
      cmdarg = "-I";
    }
  else
    {
      cmdarg = (pathtype == SYSTEM_PATH) ? "-isystem" : "-I";
    }

  /* Now process each directory in the directory list */

  for (i = 0; i < ndirs; i++)
    {
      const char *dirname;
      const char *incpath;
      char *saveresp;
      char *segment = NULL;
      size_t segsize;

      dirname = dirlist[i];

#ifdef HOST_CYGWIN
     /* Check if the path needs to be extended for Windows-based tools under
       * Cygwin:
       *
       * wintool == true:  The platform is Cygwin and we are using a windows
       *                   native tool
       */

      if (os == OS_CYGWIN && wintool)
        {
          ssize_t bufsize;

          bufsize = cygwin_conv_path(CCP_POSIX_TO_WIN_A, dirname, NULL, 0);
          convpath = (char *)malloc(bufsize);
          if (convpath == NULL)
            {
              fprintf(stderr, "ERROR:  Failed to allocate buffer.\n");
              exit(EXIT_FAILURE);
            }

          cygwin_conv_path(CCP_POSIX_TO_WIN_A, dirname, convpath,
                           bufsize);
          incpath = convpath;
        }
      else
#endif
        {
          incpath = dirname;
        }

      /* Handle the output using the selected format */

      if (compiler == COMPILER_ZDSII)
        {
          /* FORM:  -stdinc:'dir1;dir2;...;dirN'
           *        -usrinc:'dir1;dir2;...;dirN'
           */

          /* Treat the first directory differently */

          if (response == NULL)
            {
              if (i == ndirs - 1)
                {
                  ret = my_asprintf(&segment, "%s'%s'", cmdarg, incpath);
                }
              else
                {
                  ret = my_asprintf(&segment, "%s'%s", cmdarg, incpath);
                }
            }
          else
            {
              if (i == ndirs - 1)
                {
                  ret = my_asprintf(&segment, ";%s'", incpath);
                }
              else
                {
                  ret = my_asprintf(&segment, ";%s", incpath);
                }
            }
        }
      else
        {
          /* FORM:  -isystem: "dir1" -isystem "dir2" ... -isystem "dirN"
           *        -I: "dir1" -I "dir2" ... -I "dirN"
           */

          /* Treat the first directory differently */

          if (response == NULL)
            {
              ret = my_asprintf(&segment, "%s \"%s\"", cmdarg, incpath);
            }
          else
            {
              ret = my_asprintf(&segment, " %s \"%s\"", cmdarg, incpath);
            }
        }

      if (ret < 0)
        {
          fprintf(stderr, "ERROR: my_asprintf failed.\n");
          exit(EXIT_FAILURE);
        }

      /* Append the new response segment */

      saveresp  = response;
      segsize   = ret;
      respsize += (response == NULL) ? segsize + 1 : segsize;

      response = (char *)malloc(respsize);
      if (response == NULL)
        {
          fprintf(stderr, "ERROR: Failed to allocate response.\n");
          exit(EXIT_FAILURE);
        }

      if (saveresp == NULL)
        {
          strncpy(response, segment, respsize);
        }
      else
        {
          snprintf(response, respsize, "%s%s", saveresp, segment);
        }

      /* Clean up for the next pass */

      if (saveresp != NULL)
        {
          free(saveresp);
        }

      if (segment != NULL)
        {
          free(segment);
          segment = NULL;
        }

#ifdef HOST_CYGWIN
      if (convpath != NULL)
        {
          free(convpath);
          convpath = NULL;
        }
#endif
    }

  fputs(response, stdout);
  free(response);

  return EXIT_SUCCESS;
}
