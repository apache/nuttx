/****************************************************************************
 * tools/zdsar.c
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
#include <unistd.h>
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

/* Maximum objects per librarian call */

#define MAX_OBJECTS 64

#if defined(HOST_NATIVE)
#  define SEPARATOR '\\'
#  define HOSTNAME  "Native" /* Windows native */
#elif defined(HOST_CYGWIN)
#  define SEPARATOR '/'
#  define HOSTNAME  "Cygwin" /* Cygwin or MSYS under Windows */
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

static char   *g_ar        = NULL;   /* Full path to the librarian program */
static char   *g_arflags   = NULL;   /* Flags to use with the librarian program */
static char   *g_libpath   = NULL;   /* Path to the library */
static char   *g_libname   = NULL;   /* Library file name*/
static char   *g_objects   = NULL;   /* List of object files */
static int     g_debug     = 0;      /* Debug output enabled if >0 */

static char   g_command[MAX_BUFFER]; /* Full librarian command */
static char   g_wd[MAX_PATH];        /* Current working directory */
static char   g_path[MAX_PATH];      /* Buffer for expanding paths */
static char   g_objpath[MAX_PATH];   /* Path to the object files */
#ifdef HOST_CYGWIN
static char   g_expand[MAX_EXPAND];  /* Expanded path */
static char   g_dequoted[MAX_PATH];  /* De-quoted path */
static char   g_posixpath[MAX_PATH]; /* Full POSIX path */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* MinGW does not seem to provide strtok_r */

#ifndef HAVE_STRTOK_R
static char *MY_strtok_r(char *str, const char *delim, char **saveptr)
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
#  define strtok_r MY_strtok_r
#endif

static void append(char **base, char *str)
{
  char *oldbase;
  char *newbase;
  int alloclen;

  oldbase = *base;
  if (oldbase == NULL)
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
              /* No.. expend */

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
      fprintf(stderr, "ERROR: Truncated during expansion string is too long [%lu/%u]\n",
              (unsigned long)strlen(argument), MAX_EXPAND);
      exit(EXIT_FAILURE);
    }

  *dest = '\0';
  return g_expand;
#else
  return argument;
#endif
}

/* Remove backslash quoting from a path */

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
 * POSIX path to a Windows path.
 */

static const char *convert_path(const char *path)
{
#ifdef HOST_CYGWIN
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
      fprintf(stderr, "# ERROR: POSIX path too long: %lu\n",
              (unsigned long)size);
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
#else
  return path;
#endif
}

static void show_usage(const char *progname, const char *msg, int exitcode)
{
  if (msg)
    {
      fprintf(stderr, "\n");
      fprintf(stderr, "%s:\n", msg);
    }

  fprintf(stderr, "\n");
  fprintf(stderr, "%s  [OPTIONS] --ar \"<AR>\" --library \"<LIBRARY>\" obj [obj [obj...]]\n",
          progname);
  fprintf(stderr, "\n");
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  --ar <AR>\n");
  fprintf(stderr, "    A command line string that defines how to execute the ZDS-II librarian\n");
  fprintf(stderr, "  --library \"<LIBRARY>\"\n");
  fprintf(stderr, "    The library into which the object files will be inserted\n");
  fprintf(stderr, "  obj\n");
  fprintf(stderr, "    One or more object files that will be inserted into the archive.  Each expected\n");
  fprintf(stderr, "    to reside in the current directory unless --obj-path is provided on the command line\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "And [OPTIONS] include:\n");
  fprintf(stderr, "  --ar_flags \"<ARFLAGS>\"\n");
  fprintf(stderr, "    Optional librarian flags\n");
  fprintf(stderr, "  --obj-path <path>\n");
  fprintf(stderr, "    Do not look in the current directory for the object files.  Instead, look in <path> to\n");
  fprintf(stderr, "    for the object files.  --obj-path may be used once on the command line\n");
  fprintf(stderr, "  --debug\n");
  fprintf(stderr, "    Enable %s debug output\n", progname);
  fprintf(stderr, "  --help\n");
  fprintf(stderr, "    Shows this message and exits\n");
  exit(exitcode);
}

static void parse_args(int argc, char **argv)
{
  char *library = NULL;
  char *objpath = NULL;
  int argidx;

  /* Accumulate ARFLAGS up to "--" */

  for (argidx = 1; argidx < argc; argidx++)
    {
      if (strcmp(argv[argidx], "--ar") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --ar", EXIT_FAILURE);
            }
          else if (g_ar != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --ar arguments", EXIT_FAILURE);
            }

          g_ar = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--ar_flags") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --ar_flags", EXIT_FAILURE);
            }
          else if (g_arflags != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --ar_flags arguments", EXIT_FAILURE);
            }

          g_arflags = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--library") == 0)
        {
          const char *tmp_path;

          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --library", EXIT_FAILURE);
            }
          else if (library != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --library arguments", EXIT_FAILURE);
            }

          /* Convert the library path a POSIX.  NOTE this is a no-op in Windows
           * native mode.
           */

          tmp_path = convert_path(argv[argidx]);
          library  = strdup(tmp_path);
          if (library == NULL)
            {
              fprintf(stderr, "ERROR: strdup() failed\n");
              exit(EXIT_FAILURE);
            }
        }
      else if (strcmp(argv[argidx], "--obj-path") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --obj-path", EXIT_FAILURE);
            }
          else if (objpath != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --obj-path arguments", EXIT_FAILURE);
            }

          objpath = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--debug") == 0)
        {
          g_debug++;
        }
      else if (strcmp(argv[argidx], "--help") == 0)
        {
          show_usage(argv[0], NULL, EXIT_SUCCESS);
        }
      else if (strncmp(argv[argidx], "--", 2) == 0)
        {
          show_usage(argv[0], "ERROR: Unrecognized option", EXIT_FAILURE);
        }
      else
        {
          break;
        }
    }

  /* Accumulate object files */

  for (; argidx < argc; argidx++)
    {
      append(&g_objects, argv[argidx]);
    }

  if (g_debug)
    {
      fprintf(stderr, "Selections:\n");
      fprintf(stderr, "  CWD            : [%s]\n", g_wd);
      fprintf(stderr, "  AR             : [%s]\n", g_ar ? g_ar : "(None)");
      fprintf(stderr, "  AR Flags       : [%s]\n", g_arflags ? g_arflags : "(None)");
      fprintf(stderr, "  Library        : [%s]\n", library ? library : "(None)");
      fprintf(stderr, "  Object Path    : [%s]\n", objpath ? objpath : "(None");
      fprintf(stderr, "  Object Files   : [%s]\n", g_objects ? g_objects : "(None)");
      fprintf(stderr, "  Host Environ   : [%s]\n\n", HOSTNAME);
    }

  /* Check for required parameters */

  if (g_ar == NULL)
    {
      show_usage(argv[0], "ERROR: No librarian specified", EXIT_FAILURE);
    }

  if (library == NULL)
    {
      show_usage(argv[0], "ERROR: No library specified", EXIT_FAILURE);
    }
  else
    {
      /* Separate the library file name from the path.  The ZDS-II librarian
       * expects the library to be in the current working directory.
       */

      g_libpath = dirname(library);
      g_libname = basename(library);
    }

  if (g_objects == NULL)
    {
      /* Don't report an error -- this happens normally in some configurations */

      printf("No object files specified\n");
      exit(EXIT_SUCCESS);
    }

  g_objpath[0] = '\0';
  if (objpath != NULL)
    {
      const char *hostpath;
      int pathlen;

      /* If the object path relative to the current working directory? */

      /* It is a relative path if the path does not begin with the path
       * segment separator or if in the Windows native case, it begins
       * with a volume specified like C:.
       */

      pathlen = 0;

#ifdef HOST_NATIVE
      if (objpath[0] != SEPARATOR ||
          (isalpha(objpath[0]) && objpath[1] != ':'))
#else
      if (objpath[0] != SEPARATOR)
#endif
        {
          /* Add the default working directory to the path */

          /* Copy the obj_path */

          pathlen = strlen(g_wd);
          if (pathlen >= MAX_PATH)
            {
              fprintf(stderr, "ERROR: Working directory path is "
                              "too long [%d/%d]: %s\n",
                      pathlen, MAX_PATH, g_wd);
              exit(EXIT_FAILURE);
            }

          strcpy(g_path, g_wd);

          /* Append a separator is one is not already present */

          if (g_path[pathlen - 1] != SEPARATOR)
            {
              pathlen++;
              if (pathlen >= MAX_PATH)
                {
                  fprintf(stderr, "ERROR: Object path is too long "
                          "with separator[%d/%d]: %s\n",
                          pathlen, MAX_PATH, g_wd);
                  exit(EXIT_FAILURE);
                }

              g_path[pathlen] = SEPARATOR;
              g_path[pathlen + 1] = '\0';
            }
        }

      /* Add the object file path after the current working directory */

      pathlen += strlen(objpath);
      if (pathlen >= MAX_PATH)
        {
          fprintf(stderr, "ERROR: Path+objpath is too long [%d/%d]\n",
                  pathlen, MAX_PATH);
          exit(EXIT_FAILURE);
        }

      strcat(g_path, objpath);

#ifdef HOST_CYGWIN
      /* Convert the POSIX working directory to a Windows native path */

      hostpath = convert_path(g_path);
      strcpy(g_objpath, hostpath);
#endif
    }

  if (g_debug)
    {
      fprintf(stderr, "Derived:\n");
      fprintf(stderr, "  Abs Object Path: [%s]\n", g_objpath[0] != '\0' ? g_objpath : "(None");
      fprintf(stderr, "  Library Path   : [%s]\n", g_libpath ? g_libpath : "(None)");
      fprintf(stderr, "  Library Name   : [%s]\n\n", g_libname ? g_libname : "(None)");
    }
}

static void do_archive(void)
{
  struct stat buf;
  char *alloc;
  char *objects;
  char *object;
  char *lasts;
  int cmdlen;
  int pathlen;
  int objlen;
  int totallen;
  int nobjects;
  int ret;

  /* Make a copy of g_objects. We need to do this because at least the version
   * of strtok_r above does modify it.
   */

  alloc = strdup(g_objects);
  if (alloc == NULL)
    {
      fprintf(stderr, "ERROR: Failed to strdup object list\n");
      exit(EXIT_FAILURE);
    }

  objects = alloc;

  /* We may have to loop since we limit the number of objects in each call
   * to the librarian.
   */

  for (; ; )
    {
      /* Copy the librarian into the command buffer */

      cmdlen = strlen(g_ar);
      if (cmdlen >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: Librarian string is too long [%d/%d]: %s\n",
                  cmdlen, MAX_BUFFER, g_ar);
          exit(EXIT_FAILURE);
        }

      strcpy(g_command, g_ar);


      /* Add a space */

      g_command[cmdlen] = ' ';
      cmdlen++;
      g_command[cmdlen] = '\0';

      /* Copy the librarian flags into the command buffer */

      if (g_arflags != NULL)
        {
          const char *expanded;

          expanded = do_expand(g_arflags);
          cmdlen  += strlen(expanded);

          if (cmdlen >= MAX_BUFFER)
            {
              fprintf(stderr, "ERROR: CFLAG string is too long [%d/%d]: %s\n",
                      cmdlen, MAX_BUFFER, g_arflags);
              exit(EXIT_FAILURE);
            }

          strcat(g_command, expanded);
        }

      /* Add each object file.  This loop will continue until each path has been
       * tried (failure) or until stat() finds the object file
       */

      nobjects = 0;
      while ((object = strtok_r(objects, " ", &lasts)) != NULL)
        {
          const char *expanded;
          const char *converted;

          /* Set objects to NULL.  This will force strtok_r to move from the
           * the first object in the list.
           */

          objects = NULL;

          /* Add a space */

          g_command[cmdlen] = ' ';
          cmdlen++;
          g_command[cmdlen] = '\0';

          /* Create a full path to the object file */

          g_path[0] = '\0';
          pathlen   = 0;

          /* Add the path to buffer path buffer first */

          if (g_objpath[0] != '\0')
            {
              /* Copy the obj_path */

              pathlen = strlen(g_objpath);
              if (pathlen >= MAX_PATH)
                {
                  fprintf(stderr, "ERROR: Path is too long [%d/%d]: %s\n",
                          pathlen, MAX_PATH, g_objpath);
                  exit(EXIT_FAILURE);
                }

              strcpy(g_path, g_objpath);

              /* Append a separator is one is not already present */

              if (g_path[pathlen - 1] != SEPARATOR)
                {
                  pathlen++;
                  if (pathlen >= MAX_PATH)
                    {
                      fprintf(stderr, "ERROR: Path is too long with "
                              "separator[%d/%d]: %s\n",
                              pathlen, MAX_PATH, g_path);
                      exit(EXIT_FAILURE);
                    }

                  g_path[pathlen] = SEPARATOR;
                  g_path[pathlen + 1] = '\0';
                }
            }

          /* Add the object file name after the path */

          objlen   = strlen(object);
          pathlen += objlen;
          if (pathlen >= MAX_PATH)
            {
              fprintf(stderr, "ERROR: Path+objfile is too long [%d/%d]\n",
                      pathlen, MAX_PATH);
              exit(EXIT_FAILURE);
            }

          strcat(g_path, object);

          /* Check that a object file actually exists at this path */

          converted = convert_path(g_path);
          ret = stat(converted, &buf);
          if (ret < 0)
            {
              continue;
            }

          if (!S_ISREG(buf.st_mode))
            {
              fprintf(stderr, "ERROR: Object %s exists but is not a regular file\n",
                      g_path);
              exit(EXIT_FAILURE);
            }

          /* Expand the path */

          /* Copy the librarian argument of form like:
           *
           * <libname>=+-<objpath>
           */

          pathlen   = 4;  /* For =+- and terminator */

          expanded  = do_expand(g_path);
          pathlen  += strlen(expanded);

          /* Get the full length */

          pathlen  += strlen(g_libname);
          totallen  = cmdlen + pathlen;

          if (totallen >= MAX_BUFFER)
            {
              fprintf(stderr, "ERROR: object argument is too long [%d/%d]: %s=+-%s\n",
                      totallen, MAX_BUFFER, g_libname, expanded);
              exit(EXIT_FAILURE);
            }

          /* Append the next librarian command */

          pathlen = snprintf(&g_command[cmdlen], MAX_BUFFER - cmdlen, "%s=+-%s",
                             g_libname, expanded);
          cmdlen += pathlen;

          /* Terminate early if we have a LOT files in the command line */

          if (++nobjects >= MAX_OBJECTS)
            {
              break;
            }
        }

      /* Okay.. we have everything.  Add the object files to the library.  On
       * a failure to start the compiler, system() will return -1;  Otherwise,
       * the returned value from the compiler is in WEXITSTATUS(ret).
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
              fprintf(stderr, "ERROR: %s failed: %d\n", g_ar, WEXITSTATUS(ret));
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

       /* We don't really know that the command succeeded... Let's assume
        * that it did
        */

       /* Check if we have more objects to process */

       if (object == NULL)
        {
          /* No, we are finished */

          break;
        }
    }

  free(alloc);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *wd;
  int len;
  int ret;

  /* Get the current working directory */

  wd = getcwd(g_wd, MAX_PATH);
  if (wd == NULL)
    {
      fprintf(stderr, "ERROR: getcwd failed: %d\n", errno);
      return EXIT_FAILURE;
    }

  len = strlen(wd);
  if (len >= PATH_MAX)
    {
      fprintf(stderr, "ERROR: Current directory too long: [%s]\n", wd);
      return EXIT_FAILURE;
    }

  strcpy(g_wd, wd);

  /* Parse command line parameters */

  parse_args(argc, argv);

  /* Change to the directory containing the library */

  if (g_libpath != NULL)
    {
      ret = chdir(g_libpath);
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: getcwd failed: %d\n", errno);
          return EXIT_FAILURE;
        }
    }

  /* Then generate dependencies for each path on the command line. */

  do_archive();
  return EXIT_SUCCESS;
}
