/****************************************************************************
 * tools/zds/zdsar.c
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

/* Maximum objects per librarian call.
 *
 * REVISIT:  The librarian is supposed to handle multiple object insertions
 * per call, but my experience was that it was unreliable in that case.  That
 * may have improved, however, and perhaps we can increase MAX_OPBJEXT.. TRY
 * IT!
 */

#define MAX_OBJECTS 1 /* 64 */

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
#  define HOSTNAME  "Cygwin" /* Cygwin or MSYS under Windows */
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_current_wd = NULL;    /* Current working directory */
static char *g_ar         = NULL;    /* Full path to the librarian program */
static char *g_arflags    = NULL;    /* Flags to use with the librarian program */
static char *g_libpath    = NULL;    /* Path to the library */
static char *g_libname    = NULL;    /* Library file name*/
static char *g_objects    = NULL;    /* List of object files */
static int   g_debug      = 0;       /* Debug output enabled if >0 */

static char  g_command[MAX_BUFFER];  /* Full librarian command */
static char  g_initial_wd[MAX_PATH]; /* Initial working directory */
static char  g_path[MAX_PATH];       /* Temporary for path generation */
static char  g_objpath[MAX_PATH];    /* Holds the relative path to the objects */
#ifdef HOST_CYGWIN
static char  g_expand[MAX_EXPAND];   /* Temporary for quoted path */
static char  g_dequoted[MAX_PATH];   /* Temporary for de-quoted path */
static char  g_hostpath[MAX_PATH];   /* Temporary for host path conversions */
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
#  define strtok_r my_strtok_r
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

static const char *quote_backslash(const char *argument)
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

          /* Already quoted? */

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
      fprintf(stderr, "ERROR: Truncated during expansion string "
              "is too long [%lu/%u]\n",
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
      if (src[0] != '\\' ||
         (src[1] != ' ' && src[1] != '(' && src[1] != ')'))
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
  strcpy(g_path, path);
  return g_path;
#endif
}

static const char *convert_path_posix(const char *path)
{
#ifdef HOST_CYGWIN
  return convert_path(path, CCP_WIN_A_TO_POSIX);
#else
  strcpy(g_path, path);
  return g_path;
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
  fprintf(stderr, "%s [OPTIONS] --ar \"<AR>\" --library \"<LIBRARY>\" "
          "obj [obj [obj...]]\n",
          progname);
  fprintf(stderr, "\n");
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  --ar <AR>\n");
  fprintf(stderr, "    A command line string that defines how to execute the "
                  "ZDS-II librarian\n");
  fprintf(stderr, "  --library \"<LIBRARY>\"\n");
  fprintf(stderr, "    The library into which the object files will be "
                  "inserted\n");
  fprintf(stderr, "  obj\n");
  fprintf(stderr, "    One or more object files that will be inserted into "
                  "the archive.  Each expected\n");
  fprintf(stderr, "    to reside in the current directory unless --obj-path "
                  "is provided on the command line\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "And [OPTIONS] include:\n");
  fprintf(stderr, "  --ar_flags \"<ARFLAGS>\"\n");
  fprintf(stderr, "    Optional librarian flags\n");
  fprintf(stderr, "  --obj-path <path>\n");
  fprintf(stderr, "    Do not look in the current directory for the object "
                  "files.  Instead, look in <path> for\n");
  fprintf(stderr, "    the object files.  --obj-path may be used only once "
                  "on the command line\n");
  fprintf(stderr, "  --debug\n");
  fprintf(stderr, "    Enable %s debug output\n", progname);
  fprintf(stderr, "  --help\n");
  fprintf(stderr, "    Shows this message and exits\n");
  exit(exitcode);
}

static void parse_args(int argc, char **argv)
{
  const char *tmp = NULL;
  char *library   = NULL;
  char *objpath   = NULL;
  int pathlen;
  int argidx;

  /* Parse arguments */

  for (argidx = 1; argidx < argc; argidx++)
    {
      if (strcmp(argv[argidx], "--ar") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --ar",
                         EXIT_FAILURE);
            }
          else if (g_ar != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --ar arguments",
                         EXIT_FAILURE);
            }

          g_ar = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--ar_flags") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --ar_flags",
                         EXIT_FAILURE);
            }
          else if (g_arflags != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --ar_flags arguments",
                         EXIT_FAILURE);
            }

          g_arflags = argv[argidx];
        }
      else if (strcmp(argv[argidx], "--library") == 0)
        {
          const char *tmp_path;

          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --library",
                         EXIT_FAILURE);
            }
          else if (library != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --library arguments",
                         EXIT_FAILURE);
            }

          /* Convert the library path a POSIX.  NOTE this is a no-op in Windows
           * native mode.
           */

          tmp_path = convert_path_posix(argv[argidx]);
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
              show_usage(argv[0], "ERROR: Missing argument to --obj-path",
                         EXIT_FAILURE);
            }
          else if (objpath != NULL)
            {
              show_usage(argv[0], "ERROR: Multiple --obj-path arguments",
                         EXIT_FAILURE);
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
      fprintf(stderr, "  CWD            : [%s]\n",
              g_initial_wd);
      fprintf(stderr, "  Host Environ   : [%s]\n",
              HOSTNAME);
      fprintf(stderr, "  AR             : [%s]\n",
              g_ar ? g_ar : "(None)");
      fprintf(stderr, "  AR Flags       : [%s]\n",
              g_arflags ? g_arflags : "(None)");
      fprintf(stderr, "  Library        : [%s]\n",
              library ? library : "(None)");
      fprintf(stderr, "  Object Path    : [%s]\n",
              objpath ? objpath : "(None");
      fprintf(stderr, "  Object Files   : [%s]\n\n",
              g_objects ? g_objects : "(None)");
    }

  /* Check for required parameters */

  if (g_ar == NULL)
    {
      show_usage(argv[0], "ERROR: No librarian specified",
                 EXIT_FAILURE);
    }

  if (library == NULL)
    {
      show_usage(argv[0], "ERROR: No library specified",
                 EXIT_FAILURE);
    }
  else
    {
      /* Separate the library file name from the path.  The ZDS-II librarian
       * expects the library to be in the current working directory.
       */

      g_libname = basename(library); /* Must come first */
      g_libpath = dirname(library);
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

          /* Copy the initial working directory */

          pathlen = strlen(g_initial_wd);
          if (pathlen >= MAX_PATH)
            {
              fprintf(stderr, "ERROR: Working directory path is "
                              "too long [%d/%d]: %s\n",
                      pathlen, MAX_PATH, g_initial_wd);
              exit(EXIT_FAILURE);
            }

          strcpy(g_path, g_initial_wd);

          /* Append a separator is one is not already present */

          if (g_path[pathlen - 1] != SEPARATOR)
            {
              int newlen = pathlen + 1;
              if (newlen >= MAX_PATH)
                {
                  fprintf(stderr, "ERROR: Object path is too long "
                          "with separator[%d/%d]: %s\n",
                          newlen, MAX_PATH, g_initial_wd);
                  exit(EXIT_FAILURE);
                }

              g_path[pathlen]     = SEPARATOR;
              g_path[pathlen + 1] = '\0';
              pathlen             = newlen;
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
    }

  /* The object was in the current working directory.  If a library path
   * is NOT the current working directory, then the library path will now
   * be the current working directory and the path to the objects will be
   * the  working directory when the program was started.
   */

  else if (g_libpath != NULL && strcmp(g_libpath, ".") != 0)
    {
      strcpy(g_path, g_initial_wd);
    }

  /* Convert the absolute objection file path to the native host path form
   * NOTE that convert_path_posix() is a no-op in Windows native mode.
   */

  tmp = convert_path_posix(g_path);
  strcpy(g_path, tmp);

  /* Check for a relative path.  We will CD to g_libpath because the
   * library must be in the current working directory.
   */

  pathlen = strlen(g_libpath);
  if (strncmp(g_path, g_libpath, pathlen) ==  0)
    {
      const char *relpath = &g_path[pathlen];

      /* Skip over leading path segment delimiters.. that should be as
       * least one.
       */

      while (*relpath == SEPARATOR)
        {
          relpath++;
        }

      /* Convert the relative object file path to the Windows path form
       * for the ZDS-II tool tool.  NOTE that convert_path_windows() is
       * a no-op in Windows native mode.
       */

      tmp = convert_path_windows(relpath);
    }
  else
    {
      /* Convert the absolute object file path to the Windows path form
       * for the ZDS-II tool tool.  NOTE that convert_path_windows() is
       * a no-op in Windows native mode.
       */

      tmp = convert_path_windows(g_path);
    }

  /* And save the path in as a native Windows path */

  strcpy(g_objpath, tmp);

  /* Dump some intermediate results */

  if (g_debug)
    {
      fprintf(stderr, "Derived:\n");
      fprintf(stderr, "  Object Path    : [%s]\n",
              g_objpath[0] != '\0' ? g_objpath : "(None");
      fprintf(stderr, "  Library Path   : [%s]\n",
              g_libpath ? g_libpath : "(None)");
      fprintf(stderr, "  Library Name   : [%s]\n\n",
              g_libname ? g_libname : "(None)");
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

  lasts = NULL;
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
          const char *quoted;

          quoted  = quote_backslash(g_arflags);
          cmdlen += strlen(quoted);

          if (cmdlen >= MAX_BUFFER)
            {
              fprintf(stderr, "ERROR: CFLAG string is too long [%d/%d]: %s\n",
                      cmdlen, MAX_BUFFER, g_arflags);
              exit(EXIT_FAILURE);
            }

          strcat(g_command, quoted);
        }

      /* Add each object file.  This loop will continue until each path has been
       * tried (failure) or until stat() finds the object file
       */

      nobjects = 0;
      while ((object = strtok_r(objects, " ", &lasts)) != NULL)
        {
          const char *quoted;
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

              if (g_path[pathlen - 1] != WINSEPARATOR)
                {
                  int newlen = pathlen + 1;
                  if (newlen >= MAX_PATH)
                    {
                      fprintf(stderr, "ERROR: Path is too long with "
                              "separator[%d/%d]: %s\n",
                              newlen, MAX_PATH, g_path);
                      exit(EXIT_FAILURE);
                    }

                  g_path[pathlen]     = WINSEPARATOR;
                  g_path[pathlen + 1] = '\0';
                  pathlen             = newlen;
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

          /* Check that a object file actually exists at this path.  NOTE
           * that convert_path_posix() is a no-op in Windows native mode.
           */

          converted = convert_path_posix(g_path);
          ret = stat(converted, &buf);
          if (ret < 0)
            {
              fprintf(stderr, "WARNING: Stat of object %s failed: %s\n",
                      g_path, strerror(errno));
              continue;
            }

          if (!S_ISREG(buf.st_mode))
            {
              fprintf(stderr, "ERROR: Object %s exists but is not a regular "
                              "file\n",
                      g_path);
              exit(EXIT_FAILURE);
            }

          /* Expand the path */

          /* Copy the librarian argument of form like:
           *
           * <libname>=-+<objpath>
           */

          pathlen   = 4;  /* For =-+ and terminator */

          quoted   = quote_backslash(g_path);
          pathlen += strlen(quoted);

          /* Get the full length */

          pathlen  += strlen(g_libname);
          totallen  = cmdlen + pathlen;

          if (totallen >= MAX_BUFFER)
            {
              fprintf(stderr, "ERROR: object argument is too long [%d/%d]: "
                      "%s=-+%s\n",
                      totallen, MAX_BUFFER, g_libname, quoted);
              exit(EXIT_FAILURE);
            }

          /* Append the next librarian command */

          pathlen = snprintf(&g_command[cmdlen], MAX_BUFFER - cmdlen, "%s=-+%s",
                             g_libname, quoted);
          cmdlen += pathlen;

          /* Terminate early if we have a LOT files in the command line */

          if (++nobjects >= MAX_OBJECTS)
            {
              break;
            }
        }

      /* Handling the final command which may have not objects to insert */

      if (nobjects > 0)
        {
          /* Okay.. we have everything.  Add the object files to the library.
           * On a failure to start the compiler, system() will return -1;
           * Otherwise, the returned value from the compiler is in
           * WEXITSTATUS(ret).
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
                  fprintf(stderr, "ERROR: system failed: %s\n",
                                  strerror(errno));
                }
              else
                {
                  fprintf(stderr, "ERROR: %s failed: %d\n", g_ar,
                          WEXITSTATUS(ret));
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
        }

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
  int ret;

  /* Get the current working directory */

  wd = getcwd(g_initial_wd, MAX_PATH);
  if (wd == NULL)
    {
      fprintf(stderr, "ERROR: getcwd failed: %s\n", strerror(errno));
      return EXIT_FAILURE;
    }

  g_current_wd = g_initial_wd;

  /* Parse command line parameters */

  parse_args(argc, argv);

  /* Change to the directory containing the library */

  if (g_libpath != NULL && strcmp(g_libpath, ".") != 0)
    {
      ret = chdir(g_libpath);
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: getcwd failed: %s\n", strerror(errno));
          return EXIT_FAILURE;
        }

      g_current_wd = g_libpath;
    }

  /* Then generate dependencies for each path on the command line. */

  do_archive();
  return EXIT_SUCCESS;
}
