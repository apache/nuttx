/****************************************************************************
 * tools/mkdeps.c
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

#define _FILE_OFFSET_BITS 64

#include <sys/stat.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>
#include <dirent.h>

#ifdef HOST_CYGWIN
#  include <sys/cygwin.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_BUFFER  (10240)
#define MAX_EXPAND  (2048)
#define MAX_SHQUOTE (4096)

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

#    if defined(MAXNAMLEN)
#      define NAME_MAX MAXNAMLEN
#    else

  /* Lets not let a silly think like this stop us... just make something up */

#      define NAME_MAX 256
#    endif
#  endif
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

static char *g_cc        = NULL;
static char *g_cflags    = NULL;
static char *g_files     = NULL;
static char *g_altpath   = NULL;
static char *g_objpath   = NULL;
static char *g_suffix    = ".o";
static int   g_debug     = 0;
static bool  g_winnative = false;
#ifdef HOST_CYGWIN
static bool  g_winpath   = false;
#endif

static char g_command[MAX_BUFFER];
static char g_path[MAX_PATH];
#ifdef HOST_CYGWIN
static char g_expand[MAX_EXPAND];
static char g_dequoted[MAX_PATH];
static char g_posixpath[MAX_PATH];
#endif
static char g_shquote[MAX_SHQUOTE];

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

static void append(char **base, const char *str)
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
      alloclen = strlen(oldbase) + strlen(str) +
        sizeof((char) ' ') + sizeof((char) '\0');
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
  fprintf(stderr, "%s  [OPTIONS] CC -- CFLAGS -- file [file [file...]]\n",
          progname);
  fprintf(stderr, "\n");
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  CC\n");
  fprintf(stderr, "    A variable number of arguments that define how to\n");
  fprintf(stderr, "    execute the compiler\n");
  fprintf(stderr, "  CFLAGS\n");
  fprintf(stderr, "    The compiler compilation flags\n");
  fprintf(stderr, "  file\n");
  fprintf(stderr, "    One or more C files whose dependencies will be\n");
  fprintf(stderr, "    checked.  Each file is expected\n");
  fprintf(stderr, "    to reside in the current directory unless\n");
  fprintf(stderr, "     --dep-path is provided on the command line\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "And [OPTIONS] include:\n");
  fprintf(stderr, "  --dep-debug\n");
  fprintf(stderr, "    Enable script debug\n");
  fprintf(stderr, "  --dep-path <path>\n");
  fprintf(stderr, "    Do not look in the current directory for the\n");
  fprintf(stderr, "    file. Instead, look in <path> to see\n");
  fprintf(stderr, "    if the file resides there.  --dep-path may be\n");
  fprintf(stderr, "    used multiple times to specify\n");
  fprintf(stderr, "    multiple alternative location\n");
  fprintf(stderr, "  --obj-path <path>\n");
  fprintf(stderr, "    The final objects will not reside in this path\n");
  fprintf(stderr, "    but, rather, at the path provided by\n");
  fprintf(stderr, "    <path>.  if provided multiple time, only the last\n");
  fprintf(stderr, "     --obj-path will be used.\n");
  fprintf(stderr, "  --obj-suffix <suffix>\n");
  fprintf(stderr, "    If an object path is provided, then the extension\n");
  fprintf(stderr, "    will be assumed to be .o.  This\n");
  fprintf(stderr, "    default suffix can be overridden with this\n");
  fprintf(stderr, "    command line option.\n");
  fprintf(stderr, "  --winnative\n");
  fprintf(stderr, "    By default, a POSIX-style environment is assumed\n");
  fprintf(stderr, "    (e.g., Linux, Cygwin, etc.)  This option is\n");
  fprintf(stderr, "    inform the tool that is working in a pure Windows\n");
  fprintf(stderr, "    native environment.\n");
#ifdef HOST_CYGWIN
  fprintf(stderr, "  --winpaths\n");
  fprintf(stderr, "    This option is useful when using a Windows native\n");
  fprintf(stderr, "    toolchain in a POSIX environment (such such as\n");
  fprintf(stderr, "    Cygwin).  In this case, will CC\n");
  fprintf(stderr, "    generates dependency lists using Windows paths\n");
  fprintf(stderr, "    (e.g., C:\\blablah\\blabla).\n");
#endif
  fprintf(stderr, "  --help\n");
  fprintf(stderr, "    Shows this message and exits\n");
  exit(exitcode);
}

/****************************************************************************
 * Name: do_shquote
 *
 * Description:
 *    Escape the given string for use with the shell.
 *
 *    The idea was taken from:
 *    https://netbsd.gw.com/cgi-bin/man-cgi?shquote++NetBSD-current
 *    However, this implementation doesn't try to elide extraneous quotes.
 ****************************************************************************/

static const char *do_shquote(const char *argument)
{
  const char *src;
  char *dest;
  int len;

  src  = argument;
  dest = g_shquote;
  len  = 0;

  if (len < sizeof(g_shquote))
    {
      *dest++ = '\'';
      len++;
    }

  while (*src && len < sizeof(g_shquote))
    {
      if (*src == '\'')
        {
          /* Expand single quote to '\'' */

          if (len + 4 > sizeof(g_shquote))
            {
              break;
            }

          src++;
          memcpy(dest, "\'\\\'\'", 4);
          dest += 4;
          len += 4;
        }
      else
        {
          *dest++ = *src++;
          len++;
        }
    }

  if (*src || len + 2 > sizeof(g_shquote))
    {
      fprintf(stderr,
              "ERROR: Truncated during shquote string is too long"
              "[%zu/%zu]\n", strlen(argument), sizeof(g_shquote));
      exit(EXIT_FAILURE);
    }

  *dest++ = '\'';
  *dest = '\0';
  return g_shquote;
}

static void parse_args(int argc, char **argv)
{
  char *args = NULL;
  int argidx;
  int group = 0;

  /* Always look in the current directory */

  g_altpath = strdup(".");

  /* Accumulate CFLAGS up to "--" */

  for (argidx = 1; argidx < argc; argidx++)
    {
      if (strcmp(argv[argidx], "--") == 0)
        {
          g_cc = g_cflags;
          g_cflags = args;
          args = NULL;
          group++;
        }
      else if (strcmp(argv[argidx], "--dep-debug") == 0)
        {
          g_debug++;
        }
      else if (strcmp(argv[argidx], "--dep-path") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --dep-path",
                         EXIT_FAILURE);
            }

          if (args)
            {
              append(&args, argv[argidx]);
            }
          else
            {
              append(&g_altpath, argv[argidx]);
            }
        }
      else if (strcmp(argv[argidx], "--obj-path") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --obj-path",
                         EXIT_FAILURE);
            }

          g_objpath = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--obj-suffix") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --obj-suffix",
                         EXIT_FAILURE);
            }

          g_suffix = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--winnative") == 0)
        {
          g_winnative = true;
        }
#ifdef HOST_CYGWIN
      else if (strcmp(argv[argidx], "--winpath") == 0)
        {
          g_winpath = true;
        }
#endif
      else if (strcmp(argv[argidx], "--help") == 0)
        {
          show_usage(argv[0], NULL, EXIT_SUCCESS);
        }
      else
        {
          const char *arg = argv[argidx];

          /* This condition means "perform shquote for
           * g_cflags, but not g_cc or g_files".
           *
           * It isn't safe to escape g_cc because, for some reasons,
           * Makefile passes it as a single argument like:
           *
           *    $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS)
           *
           * It isn't safe to escape g_files because
           * do_dependency() uses them as bare filenames as well.
           * (In addition to passing them to system().)
           */

          if (group == 1)
            {
               arg = do_shquote(arg);
            }

          append(&args, arg);
        }
    }

  /* The final thing accumulated is the list of files */

  g_files = args;

  if (g_debug)
    {
      fprintf(stderr, "SELECTIONS\n");
      fprintf(stderr, "  CC             : [%s]\n",
              g_cc ? g_cc : "(None)");
      fprintf(stderr, "  CFLAGS         : [%s]\n",
              g_cflags ? g_cflags : "(None)");
      fprintf(stderr, "  FILES          : [%s]\n",
              g_files ? g_files : "(None)");
      fprintf(stderr, "  PATHS          : [%s]\n",
              g_altpath ? g_altpath : "(None)");
      if (g_objpath)
        {
          fprintf(stderr, "  OBJDIR         : [%s]\n", g_objpath);
          fprintf(stderr, "  SUFFIX         : [%s]\n", g_suffix);
        }
      else
        {
          fprintf(stderr, "  OBJDIR         : (None)\n");
        }

#ifdef HOST_CYGWIN
      fprintf(stderr, "  Windows Paths  : [%s]\n",
              g_winpath ? "TRUE" : "FALSE");
#endif
      fprintf(stderr, "  Windows Native : [%s]\n",
              g_winnative ? "TRUE" : "FALSE");
    }

  /* Check for required parameters */

  if (!g_cc)
    {
      show_usage(argv[0], "ERROR: No compiler specified", EXIT_FAILURE);
    }

  if (!g_files)
    {
      /* Don't report an error --
       * this happens normally in some configurations
       */

      printf("# No files specified for dependency generation\n");
      exit(EXIT_SUCCESS);
    }

#ifdef HOST_CYGWIN
  if (g_winnative && g_winpath)
    {
      show_usage(argv[0],
                 "ERROR: Both --winnative and --winpath makes no sense",
                 EXIT_FAILURE);
    }
#endif
}

static const char *do_expand(const char *argument)
{
#ifdef HOST_CYGWIN
  if (g_winpath)
    {
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
          fprintf(stderr,
                  "ERROR: Truncated during expansion string is too long"
                  "[%zu/%u]\n", strlen(argument), MAX_EXPAND);
          exit(EXIT_FAILURE);
        }

      *dest = '\0';
      return g_expand;
    }
  else
#endif
    {
      return argument;
    }
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
      if (*src != '\\' || (src[1] != ' ' && src[1] != '(' && src[1] != ')'))
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

static const char *convert_path(const char *path)
{
#ifdef HOST_CYGWIN
  if (g_winpath)
    {
      const char *retptr;
      ssize_t size;
      ssize_t ret;
      bool quoted;

      quoted = dequote_path(path);
      if (quoted)
        {
          retptr = g_posixpath;
        }
      else
        {
          retptr = &g_posixpath[1];
        }

      size = cygwin_conv_path(CCP_POSIX_TO_WIN_A | CCP_RELATIVE, g_dequoted,
                             NULL, 0);
      if (size > (MAX_PATH - 3))
        {
          fprintf(stderr, "# ERROR: POSIX path too long: %zd\n", size);
          exit(EXIT_FAILURE);
        }

      ret = cygwin_conv_path(CCP_POSIX_TO_WIN_A | CCP_RELATIVE, g_dequoted,
                             &g_posixpath[1], MAX_PATH - 3);
      if (ret < 0)
        {
          fprintf(stderr, "# ERROR: cygwin_conv_path '%s' failed: %s\n",
                  g_dequoted, strerror(errno));
          exit(EXIT_FAILURE);
        }

      if (quoted)
        {
          size++;
          g_posixpath[0] = '"';
          g_posixpath[size] = '"';
        }

      g_posixpath[size + 1] = '\0';
      return retptr;
    }
  else
#endif
    {
      return path;
    }
}

static void do_dependency(const char *file)
{
  static const char moption[] = " -M ";
  struct stat buf;
  char *alloc;
  char *altpath;
  char *path;
  char *lasts;
  char separator;
  int cmdlen;
  int pathlen;
  int filelen;
  int totallen;
  int ret;

  /* Initialize the separator */

#ifdef HOST_CYGWIN
  separator =  (g_winnative || g_winpath) ? '\\' : '/';
#else
  separator =  g_winnative ? '\\' : '/';
#endif

  /* Copy the compiler into the command buffer */

  cmdlen = strlen(g_cc);
  if (cmdlen >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Compiler string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, g_cc);
      exit(EXIT_FAILURE);
    }

  strcpy(g_command, g_cc);

  /* Copy " -MT " */

  if (g_objpath)
    {
      char tmp[NAME_MAX + 6];
      char *dupname;
      char *objname;
      char *dotptr;
      const char *expanded;

      dupname = strdup(file);
      if (!dupname)
        {
          fprintf(stderr, "ERROR: Failed to dup: %s\n", file);
          exit(EXIT_FAILURE);
        }

      objname = basename(dupname);
      dotptr  = strrchr(objname, '.');
      if (dotptr)
        {
          *dotptr = '\0';
        }

      snprintf(tmp, NAME_MAX + 6, " -MT %s%c%s%s ",
               g_objpath, separator, objname, g_suffix);
      expanded = do_expand(tmp);

      cmdlen += strlen(expanded);
      if (cmdlen >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: Option string is too long [%d/%d]: %s\n",
                  cmdlen, MAX_BUFFER, moption);
          exit(EXIT_FAILURE);
        }

      strcat(g_command, expanded);
      free(dupname);
    }

  /* Copy " -M " */

  cmdlen += strlen(moption);
  if (cmdlen >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Option string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, moption);
      exit(EXIT_FAILURE);
    }

  strcat(g_command, moption);

  /* Copy the CFLAGS into the command buffer */

  if (g_cflags)
    {
      const char *expanded;

      expanded = do_expand(g_cflags);
      cmdlen += strlen(expanded);

      if (cmdlen >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: CFLAG string is too long [%d/%d]: %s\n",
                  cmdlen, MAX_BUFFER, g_cflags);
          exit(EXIT_FAILURE);
        }

      strcat(g_command, expanded);
    }

  /* Add a space */

  g_command[cmdlen] = ' ';
  cmdlen++;
  g_command[cmdlen] = '\0';

  /* Make a copy of g_altpath. We need to do this because at least the
   * version of strtok_r above does modify it.
   */

  alloc = strdup(g_altpath);
  if (!alloc)
    {
      fprintf(stderr, "ERROR: Failed to strdup paths\n");
      exit(EXIT_FAILURE);
    }

  altpath = alloc;

  /* Try each path.  This loop will continue until each path has been tried
   * (failure) or until stat() finds the file
   */

  while ((path = strtok_r(altpath, " ", &lasts)) != NULL)
    {
      const char *expanded;
      const char *converted;

      /* Create a full path to the file */

      pathlen = strlen(path);
      if (pathlen >= MAX_PATH)
        {
          fprintf(stderr, "ERROR: Path is too long [%d/%d]: %s\n",
                  pathlen, MAX_PATH, path);
          exit(EXIT_FAILURE);
        }

      strcpy(g_path, path);

      if (g_path[pathlen] != '\0')
        {
          fprintf(stderr, "ERROR: Missing NUL terminator\n");
          exit(EXIT_FAILURE);
        }

      if (g_path[pathlen - 1] != separator)
        {
          g_path[pathlen] = separator;
          g_path[pathlen + 1] = '\0';
          pathlen++;
        }

      filelen = strlen(file);
      pathlen += filelen;
      if (pathlen >= MAX_PATH)
        {
          fprintf(stderr, "ERROR: Path+file is too long [%d/%d]\n",
                  pathlen, MAX_PATH);
          exit(EXIT_FAILURE);
        }

      strcat(g_path, file);

      /* Check that a file actually exists at this path */

      if (g_debug)
        {
          fprintf(stderr, "Trying path=%s file=%s fullpath=%s\n",
                  path, file, g_path);
        }

      converted = convert_path(g_path);
      ret = stat(converted, &buf);
      if (ret < 0)
        {
          altpath = NULL;
          continue;
        }

      if (!S_ISREG(buf.st_mode))
        {
          fprintf(stderr,
                  "ERROR: File %s exists but is not a regular file\n",
                  g_path);
          exit(EXIT_FAILURE);
        }

      /* Append the expanded path to the command */

      expanded = do_expand(g_path);
      pathlen  = strlen(expanded);
      totallen = cmdlen + pathlen;

      if (totallen >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: Path string is too long [%d/%d]: %s\n",
                  totallen, MAX_BUFFER, g_path);
          exit(EXIT_FAILURE);
        }

      strcat(g_command, expanded);

      /* Okay.. we have everything.  Create the dependency.  One a failure
       * to start the compiler, system() will return -1;  Otherwise, the
       * returned value from the compiler is in WEXITSTATUS(ret).
       */

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
              fprintf(stderr,
                      "ERROR: %s failed: %d\n", g_cc, WEXITSTATUS(ret));
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

      /* We don't really know that the command succeeded...
       * Let's assume that it did
       */

      free(alloc);
      return;
    }

  printf("# ERROR: File \"%s\" not found at any location\n", file);
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *lasts;
  char *files;
  char *file;

  /* Parse command line parameters */

  parse_args(argc, argv);

  /* Then generate dependencies for each path on the command line.  NOTE
   * strtok_r will clobber the files list.  But that is okay because we are
   * only going to traverse it once.
   */

  files = g_files;
  while ((file = strtok_r(files, " ", &lasts)) != NULL)
    {
      /* Check if we need to do path conversions for a Windows-natvive tool
       * being using in a POSIX/Cygwin environment.
       */

      do_dependency(file);
      files = NULL;
    }

  return EXIT_SUCCESS;
}
