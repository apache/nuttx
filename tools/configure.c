/****************************************************************************
 * tools/configure.c
 *
 *   Copyright (C) 2012, 2017-2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <dirent.h>
#include <libgen.h>
#include <errno.h>

#include "cfgparser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUFFER_SIZE 1024

#ifdef WIN32
#  define strndup(x, y) strdup(x)
#endif

#define HOST_NOCHANGE  0
#define HOST_LINUX     1
#define HOST_MACOS     2
#define HOST_WINDOWS   3

#define WINDOWS_NATIVE 1
#define WINDOWS_CYGWIN 2
#define WINDOWS_MSYS   3

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode);
static void dumpcfgs(void);
static void debug(const char *fmt, ...);
static void parse_args(int argc, char **argv);
static int run_make(const char *arg);
static bool filecmp(const char *f1, const char *f2);
static bool check_directory(const char *directory);
static void verify_directory(const char *directory);
static bool verify_optiondir(const char *directory);
static bool verify_file(const char *path);
static void find_topdir(void);
typedef void (*config_callback)(const char *boarddir, const char *archname,
                                const char *chipname, const char *boardname,
                                const char *configname, void *data);
static void config_search(const char *boarddir,
                          config_callback callback, void *data);
static void archname_callback(const char *boarddir, const char *archname,
                              const char *chipname, const char *boardname,
                              const char *configname, void *data);
static void find_archname(void);
static void enumerate_callback(const char *boarddir, const char *archname,
                               const char *chipname, const char *boardname,
                               const char *configname, void *data);
static void enumerate_configs(void);
static void check_configdir(void);
static void check_configured(void);
static void read_configfile(void);
static void read_versionfile(void);
static void get_verstring(void);
static bool verify_appdir(const char *appdir);
static void check_appdir(void);
static void check_configuration(void);
static void copy_file(const char *srcpath,
                      const char *destpath, mode_t mode);
static void substitute(char *str, int ch1, int ch2);
static char *double_appdir_backslashes(char *old_appdir);
static void copy_optional(void);
static void enable_feature(const char *destconfig, const char *varname);
static void disable_feature(const char *destconfig, const char *varname);
static void set_host(const char *destconfig);
static void configure(void);
static void refresh(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_WINDOWS_NATIVE
static char        g_delim         = '\\';  /* Delimiter to use when forming paths */
static bool        g_winpaths      = true;  /* True: Windows style paths */
#else
static char        g_delim         = '/';   /* Delimiter to use when forming paths */
static bool        g_winpaths      = false; /* False: POSIX style paths */
#endif
static bool        g_debug         = false; /* Enable debug output */
static bool        g_enforce       = false; /* Enfore distclean */
static bool        g_distclean     = false; /* Distclean if configured */

static const char *g_appdir        = NULL;  /* Relative path to the application directory */
static const char *g_archdir       = NULL;  /* Name of architecture subdirectory */
static const char *g_chipdir       = NULL;  /* Name of chip subdirectory */
static const char *g_boarddir      = NULL;  /* Name of board subdirectory */
static char       *g_configdir     = NULL;  /* Name of configuration subdirectory */

static char       *g_topdir        = NULL;  /* Full path to top-level NuttX build directory */
static char       *g_apppath       = NULL;  /* Full path to the application directory */
static char       *g_configtop     = NULL;  /* Full path to the top-level configuration directory */
static char       *g_configpath    = NULL;  /* Full path to the configuration sub-directory */
static char       *g_scriptspath   = NULL;  /* Full path to the scripts sub-directory */
static char       *g_verstring     = "0.0"; /* Version String */

static char       *g_srcdefconfig  = NULL;  /* Source defconfig file */
static char       *g_srcmakedefs   = NULL;  /* Source Make.defs file */

static char      **g_makeargv      = NULL;  /* Arguments pass to make */

static bool        g_winnative     = false; /* True: Windows native configuration */
static bool        g_oldnative     = false; /* True: Was Windows native configuration */
static bool        g_needapppath   = true;  /* Need to add app path to the .config file */

static uint8_t     g_host          = HOST_NOCHANGE;
static uint8_t     g_windows       = WINDOWS_CYGWIN;

static char        g_buffer[BUFFER_SIZE];   /* Scratch buffer for forming full paths */

static struct variable_s *g_configvars = NULL;
static struct variable_s *g_versionvars = NULL;

/* Optional configuration files */

static const char *g_optfiles[] =
{
  ".gdbinit",
  ".cproject",
  ".project"
};

#define N_OPTFILES (sizeof(g_optfiles) / sizeof(const char *))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s  [-d] [-E] [-e] [-b|f] [-L] [-l|m|c|g|n] "
          "[-a <app-dir>] <board-name>:<config-name> [make-opts]\n",
          progname);
  fprintf(stderr, "\nUSAGE: %s  [-h]\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "  -d:\n");
  fprintf(stderr, "    Enables debug output\n");
  fprintf(stderr, "  -E:\n");
  fprintf(stderr, "    Enforce distclean if already configured\n");
  fprintf(stderr, "  -e:\n");
  fprintf(stderr, "    Performs distclean if configuration changed\n");
  fprintf(stderr, "  -b:\n");
#ifdef CONFIG_WINDOWS_NATIVE
  fprintf(stderr, "    Informs the tool that it should use Windows style\n");
  fprintf(stderr, "    paths like C:\\Program Files instead of POSIX\n");
  fprintf(stderr, "    style paths are used like /usr/local/bin. Windows\n");
  fprintf(stderr, "    style paths are used by default.\n");
#else
  fprintf(stderr, "    Informs the tool that it should use Windows style\n");
  fprintf(stderr, "    paths like C:\\Program Files. By default, POSIX\n");
  fprintf(stderr, "    style paths like /usr/local/bin are used.\n");
#endif
  fprintf(stderr, "  -f:\n");
#ifdef CONFIG_WINDOWS_NATIVE
  fprintf(stderr, "    Informs the tool that it should use POSIX style\n");
  fprintf(stderr, "    paths like /usr/local/bin. By default, Windows\n");
  fprintf(stderr, "    style paths like C:\\Program Files are used.\n");
#else
  fprintf(stderr, "    Informs the tool that it should use POSIX style\n");
  fprintf(stderr, "    paths like /usr/local/bin instead of Windows\n");
  fprintf(stderr, "    style paths like C:\\Program Files are used.\n");
  fprintf(stderr, "    POSIX style paths are used by default.\n");
#endif
  fprintf(stderr, "  [-l|m|c|g|n]\n");
  fprintf(stderr, "    Selects the host environment.\n");
  fprintf(stderr, "    -l Selects the Linux (l) host environment.\n");
  fprintf(stderr, "    -m Selects the macOS (m) host environment.\n");
  fprintf(stderr, "    -c Selects the Windows Cygwin (c) environment.\n");
  fprintf(stderr, "    -g Selects the Windows MinGW/MSYS environment.\n");
  fprintf(stderr, "    -n Selects the Windows native (n) environment.\n");
  fprintf(stderr, "  Default: Use host setup in the defconfig file.\n");
  fprintf(stderr, "  Default Windows: Cygwin.\n");
  fprintf(stderr, "  -L:\n");
  fprintf(stderr, "    Lists all available configurations.\n");
  fprintf(stderr, "  -a <app-dir>:\n");
  fprintf(stderr, "    Informs the configuration tool where the\n");
  fprintf(stderr, "    application build directory.  This is a relative\n");
  fprintf(stderr, "    path from the top-level NuttX build directory.\n");
  fprintf(stderr, "    But default, this tool will look in the usual\n");
  fprintf(stderr, "    places to locate the application directory:\n");
  fprintf(stderr, "    ..%capps or\n", g_delim);
  fprintf(stderr, "    ..%capps-xx.yy where xx.yy is the version number.\n",
          g_delim);
  fprintf(stderr, "  <board-name>:\n");
  fprintf(stderr, "    Identifies the board.  This must correspond to a\n");
  fprintf(stderr, "    board directory under nuttx%cboards%c.\n",
          g_delim, g_delim);
  fprintf(stderr, "  <config-name>:\n");
  fprintf(stderr, "    Identifies the specific configuration for the\n");
  fprintf(stderr, "    selected <board-name>. This must correspond to\n");
  fprintf(stderr, "    a sub-directory under the board directory at\n");
  fprintf(stderr, "    under nuttx%cboards%c<board-name>%cconfigs%c.\n",
          g_delim, g_delim, g_delim, g_delim);
  fprintf(stderr, "  [make-opts]:\n");
  fprintf(stderr, "    Options directly pass to make\n");
  fprintf(stderr, "  -h:\n");
  fprintf(stderr, "    Prints this message and exits.\n");
  exit(exitcode);
}

static void dumpcfgs(void)
{
  find_topdir();
  snprintf(g_buffer, BUFFER_SIZE, "%s%cboards", g_topdir, g_delim);
  verify_directory(g_buffer);
  g_configtop = strdup(g_buffer);
  enumerate_configs();
  free(g_configtop);
  exit(EXIT_SUCCESS);
}

static void debug(const char *fmt, ...)
{
  va_list ap;

  if (g_debug)
    {
      va_start(ap, fmt);
      vprintf(fmt, ap);
      va_end(ap);
    }
}

static void parse_args(int argc, char **argv)
{
  char *ptr;
  int ch;

  /* Parse command line options */

  while ((ch = getopt(argc, argv, "a:bcdEefghLlmnu")) > 0)
    {
      switch (ch)
        {
          case 'a' :
            g_appdir = optarg;
            break;

          case 'b' :
             g_delim = '\\';
             g_winpaths = false;
             break;

          case 'c' :
            g_host    = HOST_WINDOWS;
            g_windows = WINDOWS_CYGWIN;
            break;

          case 'd' :
            g_debug = true;
            break;

          case 'E' :
            g_enforce = true;
            break;

          case 'e' :
            g_distclean = true;
            break;

          case 'f' :
             g_delim = '/';
             g_winpaths = true;
             break;

          case 'g' :
            g_host    = HOST_WINDOWS;
            g_windows = WINDOWS_MSYS;
            break;

          case 'h' :
            show_usage(argv[0], EXIT_SUCCESS);

          case 'L' :
            dumpcfgs();

          case 'l' :
            g_host = HOST_LINUX;
            break;

          case 'm' :
            g_host = HOST_MACOS;
            break;

          case 'n' :
            g_host    = HOST_WINDOWS;
            g_windows = WINDOWS_NATIVE;
            break;

          case '?' :
            fprintf(stderr, "ERROR: Unrecognized option: %c\n", optopt);
            show_usage(argv[0], EXIT_FAILURE);

          case ':' :
            fprintf(stderr, "ERROR: Missing option argument, option: %c\n",
                    optopt);
            show_usage(argv[0], EXIT_FAILURE);

          default:
            fprintf(stderr, "ERROR: Unexpected option: %c\n", ch);
            show_usage(argv[0], EXIT_FAILURE);
        }
    }

  /* There should be exactly one argument following the options */

  if (optind >= argc)
    {
      fprintf(stderr, "ERROR: Missing <board-name>:<config-name>\n");
      show_usage(argv[0], EXIT_FAILURE);
    }

  /* The required option should be the board directory name and the
   * configuration directory name separated by ':', '/' or '\'.  Any are
   * acceptable in this context.
   */

  g_boarddir = argv[optind];
  optind++;

  ptr = strchr(g_boarddir, ':');
  if (ptr == NULL)
    {
      ptr = strchr(g_boarddir, '/');
      if (!ptr)
        {
          ptr = strchr(g_boarddir, '\\');
        }
    }

  if (ptr == NULL)
    {
      fprintf(stderr, "ERROR: Invalid <board-name>:<config-name>\n");
      show_usage(argv[0], EXIT_FAILURE);
    }

  *ptr++ = '\0';
  g_configdir = ptr;

  /* The left arguments will pass to make */

  g_makeargv = &argv[optind];
}

static int run_make(const char *arg)
{
  char **argv;

  snprintf(g_buffer, BUFFER_SIZE, "make %s", arg);

  for (argv = g_makeargv; *argv; argv++)
    {
      strncat(g_buffer, " ", BUFFER_SIZE - 1);
      strncat(g_buffer, *argv, BUFFER_SIZE - 1);
    }

  return system(g_buffer);
}

static bool filecmp(const char *f1, const char *f2)
{
  FILE *stream1;
  FILE *stream2;
  char ch1;
  char ch2;

  stream1 = fopen(f1, "r");
  stream2 = fopen(f2, "r");

  if (stream1 == NULL || stream2 == NULL)
    {
      return false;
    }

  do
    {
      ch1 = fgetc(stream1);
      ch2 = fgetc(stream2);

      if (ch1 != ch2)
        {
          return false;
        }
    }
  while (ch1 != EOF && ch2 != EOF);

  fclose(stream1);
  fclose(stream2);

  return true;
}

static bool check_directory(const char *directory)
{
  struct stat buf;

  if (stat(directory, &buf) < 0)
    {
      debug("stat of %s failed: %s\n", directory, strerror(errno));
      return false;
    }

  if (!S_ISDIR(buf.st_mode))
    {
      debug("%s exists but is not a directory\n", directory);
      return false;
    }

  return true;
}

static void verify_directory(const char *directory)
{
  struct stat buf;

  if (stat(directory, &buf) < 0)
    {
      fprintf(stderr, "ERROR: stat of %s failed: %s\n",
              directory, strerror(errno));
      exit(EXIT_FAILURE);
    }

  if (!S_ISDIR(buf.st_mode))
    {
      fprintf(stderr, "ERROR: %s exists but is not a directory\n",
              directory);
      exit(EXIT_FAILURE);
    }
}

static bool verify_optiondir(const char *directory)
{
  struct stat buf;

  if (stat(directory, &buf) < 0)
    {
      /* It may be okay if the directory does not exist */

      /* It may be okay if the file does not exist */

      int errcode = errno;
      if (errcode == ENOENT)
        {
          debug("verify_optiondir: stat of %s failed: %s\n",
                directory, strerror(errno));
          return false;
        }
      else
        {
          fprintf(stderr, "ERROR: stat of %s failed: %s\n",
                  directory, strerror(errno));
          exit(EXIT_FAILURE);
        }
    }

  if (!S_ISDIR(buf.st_mode))
    {
      fprintf(stderr, "ERROR: %s exists but is not a directory\n",
              directory);
      exit(EXIT_FAILURE);
    }

  return true;
}

static bool verify_file(const char *path)
{
  struct stat buf;

  if (stat(path, &buf) < 0)
    {
      /* It may be okay if the file does not exist */

      int errcode = errno;
      if (errcode == ENOENT)
        {
          debug("verify_file: stat of %s failed: %s\n",
                path, strerror(errno));
          return false;
        }
      else
        {
          fprintf(stderr, "ERROR: stat of %s failed: %s\n",
                  path, strerror(errno));
          exit(EXIT_FAILURE);
        }
    }

  if (!S_ISREG(buf.st_mode))
    {
      fprintf(stderr, "ERROR: %s exists but is not a regular file\n", path);
      exit(EXIT_FAILURE);
    }

  return true;
}

static void find_topdir(void)
{
  char *currdir;

  /* Get and verify the top-level NuttX directory */

  /* First get the current directory.  We expect this to be either
   * the nuttx root directory or the tools subdirectory.
   */

  if (getcwd(g_buffer, BUFFER_SIZE) == NULL)
    {
      fprintf(stderr, "ERROR: getcwd failed: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Assume that we are in the tools sub-directory and the directory above
   * is the nuttx root directory.
   */

  currdir  = strdup(g_buffer);
  g_topdir = strdup(dirname(g_buffer));

  debug("get_topdir: Checking parent directory: %s\n", g_topdir);
  verify_directory(g_topdir);

  /* Check if the current directory is the nuttx root directory.
   * If so, then the tools directory should be a sub-directory.
   */

  snprintf(g_buffer, BUFFER_SIZE, "%s%ctools", currdir, g_delim);
  debug("get_topdir: Checking topdir/tools=%s\n", g_buffer);
  if (check_directory(g_buffer))
    {
      /* There is a tools sub-directory under the current directory.
       * We must have already been in the nuttx root directory.  We
       * will find out for sure in later tests.
       */

      free(g_topdir);
      g_topdir = currdir;
    }
  else
    {
      /* Yes, we are probably in the tools/ sub-directory */

      free(currdir);
      if (chdir(g_topdir) < 0)
        {
          fprintf(stderr, "ERROR: Failed to ch to %s\n", g_topdir);
          exit(EXIT_FAILURE);
        }
    }
}

static void config_search(const char *boarddir,
                          config_callback callback, void *data)
{
  DIR *dir;
  struct dirent *dp;
  struct stat buf;
  char *parent;
  char *child;

  /* Skip over any leading '/' or '\\'.  This happens on the first second
   * call because the starting boarddir is ""
   */

  if (boarddir[0] == g_delim)
    {
      boarddir++;
    }

  /* Get the full directory path and open it */

  snprintf(g_buffer, BUFFER_SIZE, "%s%c%s", g_configtop, g_delim, boarddir);
  dir = opendir(g_buffer);
  if (!dir)
    {
      fprintf(stderr, "ERROR: Could not open %s: %s\n",
              g_buffer, strerror(errno));
      return;
    }

  /* Make a copy of the path to the directory */

  parent = strdup(g_buffer);

  /* Visit each entry in the directory */

  while ((dp = readdir(dir)) != NULL)
    {
      /* Ignore directory entries that start with '.' */

      if (dp->d_name[0] == '.')
        {
          continue;
        }

      /* Get a properly terminated copy of d_name (if d_name is long it may
       * not include a NUL terminator.
       */

      child = strndup(dp->d_name, NAME_MAX);

      /* Get the full path to d_name and stat the file/directory */

      snprintf(g_buffer, BUFFER_SIZE, "%s%c%s", parent, g_delim, child);
      if (stat(g_buffer, &buf) < 0)
        {
          fprintf(stderr, "ERROR: stat of %s failed: %s\n",
                  g_buffer, strerror(errno));
          free(child);
          continue;
        }

      /* If it is a directory, then recurse */

      if (S_ISDIR(buf.st_mode))
        {
          char *tmppath;
          snprintf(g_buffer, BUFFER_SIZE, "%s%c%s",
                   boarddir, g_delim, child);
          tmppath = strdup(g_buffer);
          config_search(tmppath, callback, data);
          free(tmppath);
        }

      /* If it is a regular file named 'defconfig' then we have found a
       * configuration directory.  We could terminate the search in this case
       * because we do not expect sub-directories within configuration
       * directories.
       */

      else if (S_ISREG(buf.st_mode) && strcmp("defconfig", child) == 0)
        {
          char *archname;
          char *chipname;
          char *boardname;
          char *configname;
          char *delim;

          /* Get the board directory near the beginning of the 'boarddir':
           * <archdir>/<chipdir>/<boarddir>/configs/<configdir>
           */

          /* Make a modifiable copy */

          strncpy(g_buffer, boarddir, BUFFER_SIZE - 1);

          /* Save the <archdir> */

          archname = g_buffer;

          delim = strchr(g_buffer, g_delim);
          if (delim == NULL)
            {
              debug("ERROR: delimiter not found in path: %s\n", boarddir);
            }
          else
            {
              /* Save the <chipdir> */

              *delim   = '\0';
              chipname = delim + 1;

              delim = strchr(chipname, g_delim);
              if (delim == NULL)
                {
                  debug("ERROR: delimiter not found in path: %s\n",
                        chipname);
                }
              else
                {
                  /* Save the <boardir> */

                  *delim    = '\0';
                  boardname = delim + 1;

                  delim = strchr(boardname, g_delim);
                  if (delim == NULL)
                    {
                      debug("ERROR: delimiter not found in path: %s\n",
                            boardname);
                    }
                  else
                    {
                      /* Save the <configdir>  */

                      *delim = '\0';
                      delim  = strrchr(delim + 1, g_delim);
                      if (delim == NULL)
                        {
                          debug("ERROR: directory not found in path: %s\n",
                                boardname);
                        }
                      else
                        {
                          configname = delim + 1;
                          callback(boarddir, archname, chipname,
                                   boardname, configname, data);
                        }
                    }
                }
            }
        }

      free(child);
    }

  free(parent);
  closedir(dir);
}

static void archname_callback(const char *boarddir, const char *archname,
                              const char *chipname, const char *boardname,
                              const char *configname, void *data)
{
  if (strcmp(g_boarddir, boardname) == 0 &&
      strcmp(g_configdir, configname) == 0)
    {
      g_archdir = strdup(archname);
      g_chipdir = strdup(chipname);
    }
}

static void find_archname(void)
{
  config_search("", archname_callback, NULL);
  if (g_archdir == NULL || g_chipdir == NULL)
    {
      g_archdir = "unknown";
      g_chipdir = "unknown";
    }
}

static void enumerate_callback(const char *boarddir, const char *archname,
                               const char *chipname, const char *boardname,
                               const char *configname, void *data)
{
  fprintf(stderr, "  %s:%s\n", boardname, configname);
}

static void enumerate_configs(void)
{
  fprintf(stderr, "Options for <board-name>:<config-name> include:\n\n");
  config_search("", enumerate_callback, NULL);
}

static void check_configdir(void)
{
  /* Get the path to the top level configuration directory: boards/ */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cboards", g_topdir, g_delim);
  debug("check_configdir: Checking configtop=%s\n", g_buffer);

  verify_directory(g_buffer);
  g_configtop = strdup(g_buffer);

  /* Get and verify the path to the selected configuration:
   * boards/<archdir>/<chipdir>/<boarddir>/configs/<configdir>
   */

  find_archname();

  snprintf(g_buffer, BUFFER_SIZE, "%s%cboards%c%s%c%s%c%s%cconfigs%c%s",
           g_topdir, g_delim, g_delim, g_archdir, g_delim, g_chipdir,
           g_delim, g_boarddir, g_delim, g_delim, g_configdir);
  debug("check_configdir: Checking configpath=%s\n", g_buffer);

  if (!verify_optiondir(g_buffer))
    {
      fprintf(stderr, "ERROR: No configuration at %s\n", g_buffer);
      fprintf(stderr, "Run tools/configure -L"
                      " to list available configurations.\n");
      exit(EXIT_FAILURE);
    }

  g_configpath = strdup(g_buffer);

  /* Get and verify the path to the scripts directory:
   * boards/<archdir>/<chipdir>/<boarddir>/scripts
   */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cboards%c%s%c%s%c%s%cscripts",
           g_topdir, g_delim, g_delim, g_archdir, g_delim,
           g_chipdir, g_delim, g_boarddir, g_delim);
  debug("check_configdir: Checking scriptspath=%s\n", g_buffer);

  g_scriptspath = NULL;
  if (verify_optiondir(g_buffer))
    {
      g_scriptspath = strdup(g_buffer);
    }
}

static void check_configured(void)
{
  /* If we are already configured then there will be a .config and
   * a Make.defs file in the top-level directory.
   */

  snprintf(g_buffer, BUFFER_SIZE, "%s%c.config", g_topdir, g_delim);
  debug("check_configured: Checking %s\n", g_buffer);

  if (!verify_file(g_buffer))
    {
      return;
    }

  if (g_enforce)
    {
      run_make("distclean");
    }
  else
    {
      char *defcfgpath = NULL;

      snprintf(g_buffer, BUFFER_SIZE, "%s%cdefconfig",
               g_configpath, g_delim);
      defcfgpath = strdup(g_buffer);

      snprintf(g_buffer, BUFFER_SIZE, "%s%cdefconfig",
               g_topdir, g_delim);

      if (filecmp(g_buffer, defcfgpath))
        {
          fprintf(stderr, "No configuration change.\n");
          free(defcfgpath);
          exit(EXIT_SUCCESS);
        }
      else
        {
          free(defcfgpath);
          if (g_distclean)
            {
              run_make("distclean");
            }
          else
            {
              fprintf(stderr, "Already configured!\n");
              fprintf(stderr, "Please 'make distclean' and try again.\n");
              exit(EXIT_FAILURE);
            }
        }
    }
}

static void read_configfile(void)
{
  FILE *stream;

  snprintf(g_buffer, BUFFER_SIZE, "%s%cdefconfig", g_configpath, g_delim);
  stream = fopen(g_buffer, "r");
  if (!stream)
    {
       fprintf(stderr, "ERROR: failed to open %s for reading: %s\n",
               g_buffer, strerror(errno));
       exit(EXIT_FAILURE);
    }

  parse_file(stream, &g_configvars);
  fclose(stream);
}

static void read_versionfile(void)
{
  FILE *stream;

  snprintf(g_buffer, BUFFER_SIZE, "%s%c.version", g_topdir, g_delim);
  stream = fopen(g_buffer, "r");
  if (!stream)
    {
      /* It may not be an error if there is no .version file */

       debug("Failed to open %s for reading: %s\n",
             g_buffer, strerror(errno));
    }
  else
    {
      parse_file(stream, &g_versionvars);
      fclose(stream);
    }
}

static void get_verstring(void)
{
  struct variable_s *var;

  if (g_versionvars)
    {
      var = find_variable("CONFIG_VERSION_STRING", g_versionvars);
      if (var && var->val)
        {
          g_verstring = strdup(var->val);
        }
    }

  debug("get_verstring: Version string=%s\n", g_verstring);
}

static bool verify_appdir(const char *appdir)
{
  /* Does this directory exist? */

  snprintf(g_buffer, BUFFER_SIZE, "%s%c%s", g_topdir, g_delim, appdir);
  debug("verify_appdir: Checking apppath=%s\n", g_buffer);
  if (verify_optiondir(g_buffer))
    {
      /* Yes.. Use this application directory path */

      g_appdir  = strdup(appdir);
      g_apppath = strdup(g_buffer);
      return true;
    }

  debug("verify_appdir: apppath=%s does not exist\n", g_buffer);
  return false;
}

static void check_appdir(void)
{
  char tmp[16];

  /* Get and verify the full path to the application directory */

  /* Was the appdir provided on the command line? */

  debug("check_appdir: Command line appdir=%s\n",
        g_appdir ? g_appdir : "<null>");

  if (!g_appdir)
    {
      /* If no application directory was provided on the command line and we
       * are switching between a windows native host and some other host then
       * ignore any path to the apps/ directory in the defconfig file.  It
       * will most certainly not be in a usable form.
       */

      if (g_winnative == g_oldnative)
        {
          /* No, was the path provided in the configuration? */

          struct variable_s *var =
            find_variable("CONFIG_APPS_DIR", g_configvars);

          if (var != NULL)
            {
              debug("check_appdir: Config file appdir=%s\n",
                    var->val ? var->val : "<null>");

              /* Yes.. does this directory exist? */

              if (var->val && verify_appdir(var->val))
                {
                  /* We are using the CONFIG_APPS_DIR setting already in the
                   * defconfig file.
                   */

                  g_needapppath = false;
                  return;
                }
            }
        }

      /* Now try some canned locations */

      /* Try ../apps-xx.yy where xx.yy is the version string */

      snprintf(tmp, 16, "..%capps-%s", g_delim, g_verstring);
      debug("check_appdir: Try appdir=%s\n", tmp);
      if (verify_appdir(tmp))
        {
          return;
        }

      /* Try ../apps with no version */

      snprintf(tmp, 16, "..%capps", g_delim);
      debug("check_appdir: Try appdir=%s\n", tmp);
      if (verify_appdir(tmp))
        {
          return;
        }

      /* Try ../apps-xx.yy where xx.yy are the NuttX version number */

      fprintf(stderr, "ERROR: Could not find the path to the appdir\n");
      exit(EXIT_FAILURE);
    }
  else if (!verify_appdir(g_appdir))
    {
      fprintf(stderr, "ERROR: Command line path to appdir does not exist\n");
      exit(EXIT_FAILURE);
    }
}

static void check_configuration(void)
{
  struct variable_s *var;

  /* Check if this is a Windows native configuration */

  var = find_variable("CONFIG_WINDOWS_NATIVE", g_configvars);
  if (var && var->val && strcmp("y", var->val) == 0)
    {
      debug("check_configuration: Windows native configuration\n");
      g_oldnative = true;
    }

  /* If we are going to some host other than windows native or to a windows
   * native host, then don't ignore what is in the defconfig file.
   */

  if (g_host == HOST_NOCHANGE)
    {
      /* Use whatever we found in the configuration file */

      g_winnative = g_oldnative;
    }
  else if (g_host == HOST_WINDOWS && g_windows == WINDOWS_NATIVE)
    {
      /* The new configuration is windows native */

      g_winnative = true;
    }

  /* All configurations must provide a defconfig and Make.defs file */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cdefconfig", g_configpath, g_delim);
  debug("check_configuration: Checking %s\n", g_buffer);
  if (!verify_file(g_buffer))
    {
      fprintf(stderr, "ERROR: No configuration in %s\n", g_configpath);
      fprintf(stderr, "       No defconfig file found.\n");
      fprintf(stderr, "Run tools/configure -L"
                      " to list available configurations.\n");
      exit(EXIT_FAILURE);
    }

  g_srcdefconfig = strdup(g_buffer);

  /* Try the Make.defs file */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cMake.defs", g_configpath, g_delim);
  debug("check_configuration: Checking %s\n", g_buffer);
  if (!verify_file(g_buffer))
    {
      /* An alternative location is the scripts/ directory */

      if (g_scriptspath != NULL)
        {
          snprintf(g_buffer, BUFFER_SIZE, "%s%cMake.defs",
                   g_scriptspath, g_delim);
          debug("check_configuration: Checking %s\n", g_buffer);
          if (!verify_file(g_buffer))
            {
              fprintf(stderr, "ERROR: No Make.defs file in %s\n",
                      g_configpath);
              fprintf(stderr, "       No Make.defs file in %s\n",
                      g_scriptspath);
              fprintf(stderr, "Run tools/configure -L"
                              " to list available configurations.\n");
              exit(EXIT_FAILURE);
            }
        }
      else
        {
          fprintf(stderr, "ERROR: No Make.defs file in %s\n", g_configpath);
          fprintf(stderr, "Run tools/configure -L"
                          " to list available configurations.\n");
          exit(EXIT_FAILURE);
        }
    }

  g_srcmakedefs = strdup(g_buffer);
}

static void copy_file(const char *srcpath,
                      const char *destpath, mode_t mode)
{
  int nbytesread;
  int nbyteswritten;
  int rdfd;
  int wrfd;

  /* Open the source file for reading */

  rdfd = open(srcpath, O_RDONLY);
  if (rdfd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s for reading: %s\n",
              srcpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Now open the destination for writing */

  wrfd = open(destpath, O_WRONLY | O_CREAT | O_TRUNC, mode);
  if (wrfd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s for writing: %s\n",
              destpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Now copy the file */

  for (; ; )
    {
      do
        {
          nbytesread = read(rdfd, g_buffer, BUFFER_SIZE);
          if (nbytesread == 0)
            {
              /* End of file */

              close(rdfd);
              close(wrfd);
              return;
            }
          else if (nbytesread < 0)
            {
              /* EINTR is not an error (but will still stop the copy) */

              fprintf(stderr, "ERROR: Read failure: %s\n", strerror(errno));
              exit(EXIT_FAILURE);
            }
        }
      while (nbytesread <= 0);

      do
        {
          nbyteswritten = write(wrfd, g_buffer, nbytesread);
          if (nbyteswritten >= 0)
            {
              nbytesread -= nbyteswritten;
            }
          else
            {
              /* EINTR is not an error (but will still stop the copy) */

              fprintf(stderr, "ERROR: Write failure: %s\n", strerror(errno));
              exit(EXIT_FAILURE);
            }
        }
      while (nbytesread > 0);
    }
}

static void substitute(char *str, int ch1, int ch2)
{
  for (; *str; str++)
    {
      if (*str == ch1)
        {
          *str = ch2;
        }
    }
}

static char *double_appdir_backslashes(char *old_appdir)
{
  char *new_appdir = NULL;
  char *p_old = NULL;
  char *p_new = NULL;
  int oldlen = 0;
  int occurrences = 0;
  int alloclen = 0;

  p_old = old_appdir;
  while ((p_old = strchr(p_old, '\\')) != NULL)
    {
      occurrences++;
      p_old++;
    }

  if (occurrences != 0)
    {
      oldlen = strlen(old_appdir);
      alloclen = oldlen + occurrences + sizeof((char) '\0');
      new_appdir = malloc(alloclen);

      if (new_appdir != NULL)
        {
          p_old = old_appdir;
          p_new = new_appdir;
          while (oldlen)
            {
              if (*p_old != '\\')
                {
                  *p_new++ = *p_old;
                }
                else
                {
                  *p_new++ = '\\';
                  *p_new++ = '\\';
                }

              ++p_old;
              --oldlen;
            }

          *p_new = '\0';
        }
    }
  else
    {
      new_appdir = strdup(old_appdir);
    }

  return new_appdir;
}

static void copy_optional(void)
{
  int i;

  for (i = 0; i < N_OPTFILES; i++)
    {
      snprintf(g_buffer, BUFFER_SIZE, "%s%c%s",
               g_configpath, g_delim, g_optfiles[i]);

      if (verify_file(g_buffer))
        {
          char *optsrc = strdup(g_buffer);

          snprintf(g_buffer, BUFFER_SIZE, "%s%c%s",
                   g_topdir, g_delim, g_optfiles[i]);

          debug("copy_optional: Copying from %s to %s\n", optsrc, g_buffer);
          copy_file(optsrc, g_buffer, 0644);

          free(optsrc);
        }
    }
}

static void enable_feature(const char *destconfig, const char *varname)
{
  int ret;

  snprintf(g_buffer, BUFFER_SIZE,
           "kconfig-tweak --file %s --enable %s",
           destconfig, varname);

  ret = system(g_buffer);

#ifdef WEXITSTATUS
  if (ret < 0 || WEXITSTATUS(ret) != 0)
#else
  if (ret < 0)
#endif
    {
      fprintf(stderr, "ERROR: Failed to enable %s\n", varname);
      fprintf(stderr, "       command: %s\n", g_buffer);
      exit(EXIT_FAILURE);
    }
}

static void disable_feature(const char *destconfig, const char *varname)
{
  int ret;

  snprintf(g_buffer, BUFFER_SIZE,
           "kconfig-tweak --file %s --disable %s",
           destconfig, varname);

  ret = system(g_buffer);

#ifdef WEXITSTATUS
  if (ret < 0 || WEXITSTATUS(ret) != 0)
#else
  if (ret < 0)
#endif
    {
      fprintf(stderr, "ERROR: Failed to disable %s\n", varname);
      fprintf(stderr, "       command: %s\n", g_buffer);
      exit(EXIT_FAILURE);
    }
}

/* Select the host build development environment */

static void set_host(const char *destconfig)
{
  switch (g_host)
    {
      case HOST_LINUX:
        {
          printf("  Select the Linux host\n");

          enable_feature(destconfig, "CONFIG_HOST_LINUX");
          disable_feature(destconfig, "CONFIG_HOST_WINDOWS");
          disable_feature(destconfig, "CONFIG_HOST_MACOS");

          disable_feature(destconfig, "CONFIG_WINDOWS_NATIVE");
          disable_feature(destconfig, "CONFIG_WINDOWS_CYGWIN");
          disable_feature(destconfig, "CONFIG_WINDOWS_MSYS");
          disable_feature(destconfig, "CONFIG_WINDOWS_OTHER");

          enable_feature(destconfig, "CONFIG_SIM_X8664_SYSTEMV");
          disable_feature(destconfig, "CONFIG_SIM_X8664_MICROSOFT");
        }
        break;

      case HOST_MACOS:
        {
          printf("  Select the macOS host\n");

          disable_feature(destconfig, "CONFIG_HOST_LINUX");
          disable_feature(destconfig, "CONFIG_HOST_WINDOWS");
          enable_feature(destconfig, "CONFIG_HOST_MACOS");

          disable_feature(destconfig, "CONFIG_WINDOWS_NATIVE");
          disable_feature(destconfig, "CONFIG_WINDOWS_CYGWIN");
          disable_feature(destconfig, "CONFIG_WINDOWS_MSYS");
          disable_feature(destconfig, "CONFIG_WINDOWS_OTHER");

          enable_feature(destconfig, "CONFIG_SIM_X8664_SYSTEMV");
          disable_feature(destconfig, "CONFIG_SIM_X8664_MICROSOFT");
        }
        break;

      case HOST_WINDOWS:
        {
          enable_feature(destconfig, "CONFIG_HOST_WINDOWS");
          disable_feature(destconfig, "CONFIG_HOST_LINUX");
          disable_feature(destconfig, "CONFIG_HOST_MACOS");

          disable_feature(destconfig, "CONFIG_WINDOWS_OTHER");

          enable_feature(destconfig, "CONFIG_SIM_X8664_MICROSOFT");
          disable_feature(destconfig, "CONFIG_SIM_X8664_SYSTEMV");

          switch (g_windows)
            {
              case WINDOWS_CYGWIN:
                printf("  Select Windows/Cygwin host\n");
                enable_feature(destconfig, "CONFIG_WINDOWS_CYGWIN");
                disable_feature(destconfig, "CONFIG_WINDOWS_MSYS");
                disable_feature(destconfig, "CONFIG_WINDOWS_NATIVE");
                break;

              case WINDOWS_MSYS:
                printf("  Select Windows/MSYS host\n");
                disable_feature(destconfig, "CONFIG_WINDOWS_CYGWIN");
                enable_feature(destconfig, "CONFIG_WINDOWS_MSYS");
                disable_feature(destconfig, "CONFIG_WINDOWS_NATIVE");
                break;

              case WINDOWS_NATIVE:
                printf("  Select Windows native host\n");
                disable_feature(destconfig, "CONFIG_WINDOWS_CYGWIN");
                disable_feature(destconfig, "CONFIG_WINDOWS_MSYS");
                enable_feature(destconfig, "CONFIG_WINDOWS_NATIVE");
                break;

              default:
               fprintf(stderr,
                       "ERROR: Unrecognized  windows configuration: %d\n",
                       g_windows);
               exit(EXIT_FAILURE);
            }
        }
        break;

      case HOST_NOCHANGE:
        break;

      default:
        {
          fprintf(stderr, "ERROR: Unrecognized  host configuration: %d\n",
                  g_host);
          exit(EXIT_FAILURE);
        }
    }
}

static void configure(void)
{
  char *destconfig;

  /* Copy the defconfig to toplevel */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cdefconfig", g_topdir, g_delim);
  copy_file(g_srcdefconfig, g_buffer, 0644);

  /* Copy the defconfig file as .config */

  snprintf(g_buffer, BUFFER_SIZE, "%s%c.config", g_topdir, g_delim);
  destconfig = strdup(g_buffer);
  debug("configure: Copying from %s to %s\n", g_srcdefconfig, destconfig);
  copy_file(g_srcdefconfig, destconfig, 0644);

  /* Copy the Make.defs file as Make.defs */

  snprintf(g_buffer, BUFFER_SIZE, "%s%cMake.defs", g_topdir, g_delim);
  debug("configure: Copying from %s to %s\n", g_srcmakedefs, g_buffer);
  copy_file(g_srcmakedefs, g_buffer, 0644);

  /* Copy optional files */

  copy_optional();

  /* Select the host build development environment */

  set_host(destconfig);

  /* If we did not use the CONFIG_APPS_DIR that was in the defconfig config
   * file, then append the correct application information to the tail of the
   * .config file
   */

  if (g_needapppath)
    {
      FILE *stream;
      char *appdir = strdup(g_appdir);

      /* One complexity is if we are using Windows paths, but the
       * configuration needs POSIX paths (or vice versa).
       */

      if (g_winpaths != g_winnative)
        {
          /* Not the same */

          if (g_winpaths)
            {
              /* Using Windows paths, but the configuration wants POSIX
               * paths.
               */

              substitute(appdir, '\\', '/');
            }
          else
            {
              /* Using POSIX paths, but the configuration wants Windows
               * paths.
               */

              substitute(appdir, '/', '\\');
            }
        }

      /* Looks like prebuilt winnative kconfig-conf interprets "..\apps" as
       * "..apps" (possibly '\a' as escape-sequence) so expand winnative path
       * to double-backslashed variant "..\\apps".
       */

      if (g_winnative)
        {
          char *tmp_appdir = double_appdir_backslashes(appdir);
          if (NULL == tmp_appdir)
            {
              fprintf(stderr,
                      "ERROR: Failed to double appdir backslashes\n");
              exit(EXIT_FAILURE);
            }

          free(appdir);
          appdir = tmp_appdir;
        }

      /* Open the file for appending */

      stream = fopen(destconfig, "a");
      if (!stream)
        {
          fprintf(stderr,
                  "ERROR: Failed to open %s for append mode mode: %s\n",
                  destconfig, strerror(errno));
          exit(EXIT_FAILURE);
        }

      fprintf(stream, "\n# Application configuration\n\n");
      fprintf(stream, "CONFIG_APPS_DIR=\"%s\"\n", appdir);
      fclose(stream);
      free(appdir);
    }

  free(destconfig);
}

static void refresh(void)
{
  int ret;

  printf("  Refreshing...\n");
  fflush(stdout);

  ret = run_make("olddefconfig");
  putchar('\n');

#ifdef WEXITSTATUS
  if (ret < 0 || WEXITSTATUS(ret) != 0)
#else
  if (ret < 0)
#endif
    {
      fprintf(stderr, "ERROR: Failed to refresh configurations\n");
      fprintf(stderr, "       kconfig-conf --olddefconfig Kconfig\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  debug("main: Checking arguments\n");
  parse_args(argc, argv);

  debug("main: Checking NuttX Directories\n");
  find_topdir();
  check_configdir();
  check_configured();

  debug("main: Reading the configuration/version files\n");
  read_configfile();
  read_versionfile();
  get_verstring();

  debug("main: Checking Configuration Directory\n");
  check_configuration();

  debug("main: Checking Application Directories\n");
  check_appdir();
  debug("main: Using apppath=%s\n", g_apppath ? g_apppath : "<null>");

  debug("main: Configuring\n");
  configure();

  debug("main: Refresh configuration\n");
  refresh();
  return EXIT_SUCCESS;
}
