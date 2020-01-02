/****************************************************************************
 * tools/initialconfig.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#define _GNU_SOURCE 1
#include <sys/stat.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_LINE           512
#define MAX_ARCHITECTURES  32
#define MAX_MCUS           64
#define MAX_BOARDS         128

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*direntcb_t)(const char *dirpath, struct dirent *entry,
                          void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_WINDOWS_NATIVE
static char        g_delim       = '\\';      /* Delimiter to use when forming paths */
#else
static char        g_delim       = '/';       /* Delimiter to use when forming paths */
#endif

static const char  g_archdir[]   = "arch";    /* Architecture directory */
static const char  g_configdir[] = "boards";  /* Board configuration directory */

static char       *g_arch[MAX_ARCHITECTURES]; /* List of architecture names */
static int         g_narch;                   /* Number of architecture names */
static char       *g_selected_arch;           /* Selected architecture name */
static char       *g_selected_family;         /* Selected architecture family name */

static char       *g_mcu[MAX_MCUS];           /* List of MCU names */
static int         g_nmcu;                    /* Number of MCU names */
static char       *g_selected_mcu;            /* Selected MCU name */

static char       *g_board[MAX_BOARDS];       /* List of board names */
static int         g_nboard;                  /* Number of board names */
static char       *g_selected_board;          /* Selected board name */

static char        g_line[MAX_LINE + 1];      /* Line read from config file */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skip_space
 *
 * Description:
 *   Skip over any spaces
 *
 ****************************************************************************/

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/****************************************************************************
 * Name: find_name_end
 *
 * Description:
 *   Find the end of a variable string
 *
 ****************************************************************************/

static char *find_name_end(char *ptr)
{
  while (*ptr && (isalnum((int)*ptr) || *ptr == '_')) ptr++;
  return ptr;
}

/****************************************************************************
 * Name: find_value_end
 *
 * Description:
 *   Find the end of a value string
 *
 ****************************************************************************/

static char *find_value_end(char *ptr)
{
  while (*ptr && !isspace((int)*ptr))
    {
      if (*ptr == '"')
        {
           do ptr++; while (*ptr && *ptr != '"');
           if (*ptr) ptr++;
        }
      else
        {
           do ptr++; while (*ptr && !isspace((int)*ptr) && *ptr != '"');
        }
    }

  return ptr;
}

/****************************************************************************
 * Name: read_line
 *
 * Description:
 *   Read the next line from the configuration file
 *
 ****************************************************************************/

static char *read_line(FILE *stream)
{
  char *ptr;

  for (; ; )
    {
      g_line[MAX_LINE] = '\0';
      if (!fgets(g_line, MAX_LINE, stream))
        {
          return NULL;
        }
      else
        {
          ptr = skip_space(g_line);
          if (*ptr && *ptr != '#' && *ptr != '\n')
            {
              return ptr;
            }
        }
    }
}

/****************************************************************************
 * Name: parse_line
 *
 * Description:
 *   Parse the line from the configuration file into a variable name
 *   string and a value string.
 *
 ****************************************************************************/

static void parse_line(char *ptr, char **varname, char **varval)
{
  /* Skip over any leading spaces */

  ptr = skip_space(ptr);

  /* The first no-space is the beginning of the variable name */

  *varname = skip_space(ptr);
  *varval = NULL;

  /* Parse to the end of the variable name */

  ptr = find_name_end(ptr);

  /* An equal sign is expected next, perhaps after some white space */

  if (*ptr && *ptr != '=')
    {
      /* Some else follows the variable name.  Terminate the variable
       * name and skip over any spaces.
       */

      *ptr = '\0';
       ptr = skip_space(ptr + 1);
    }

  /* Verify that the equal sign is present */

  if (*ptr == '=')
    {
      /* Make sure that the variable name is terminated (this was already
       * done if the name was followed by white space.
       */

      *ptr = '\0';

      /* The variable value should follow =, perhaps separated by some
       * white space.
       */

      ptr = skip_space(ptr + 1);
      if (*ptr)
        {
          /* Yes.. a variable follows.  Save the pointer to the start
           * of the variable string.
           */

          *varval = ptr;

          /* Find the end of the variable string and make sure that it
           * is terminated.
           */

          ptr = find_value_end(ptr);
          *ptr = '\0';
        }
    }
}

/****************************************************************************
 * Name: find_variable
 *
 * Description:
 *   Return true if the selected variable exists.  Also return the value of
 *   the variable.
 *
 ****************************************************************************/

static bool find_variable(const char *configpath, const char *varname,
                          char **varvalue)
{
  FILE *stream;
  char *tmpname;
  char *tmpvalue;
  char *ptr;

  stream = fopen(configpath, "r");
  if (!stream)
    {
       fprintf(stderr, "ERROR: failed to open %s for reading: %s\n",
               configpath, strerror(errno));
       exit(EXIT_FAILURE);
    }

  /* Loop until the entire file has been parsed. */

  do
    {
      /* Read the next line from the file */

      ptr = read_line(stream);
      if (ptr)
        {
          /* Parse the line into a variable and a value field */

          tmpname = NULL;
          tmpvalue = NULL;
          parse_line(ptr, &tmpname, &tmpvalue);

          /* Make sure that both a variable name and value name were found. */

          if (tmpname == NULL || tmpvalue == NULL)
            {
              continue;
            }

          /* Check if this the variable name and value we are looking for */

          if (strcmp(varname, tmpname) == 0)
            {
              /* Yes.. return the value of the variable */

              *varvalue = tmpvalue;
              fclose(stream);
              return true;
            }
        }
    }
  while (ptr);

  /* Return failure */

  fclose(stream);
  return false;
}

/****************************************************************************
 * Name: check_variable
 *
 * Description:
 *   Return true if the selected variable exists in the configuration file
 *   and has the specified value.
 *
 ****************************************************************************/

static bool check_variable(const char *configpath, const char *varname,
                           const char *varvalue)
{
  char *tmpvalue;

  if (find_variable(configpath, varname, &tmpvalue))
    {
      /* The variable name exists.  Does it have a value?  Does the value
       * match varvalue?
       */

      if (tmpvalue != NULL && strcmp(varvalue, tmpvalue) == 0)
        {
          /* Yes.. return success */

          return true;
        }
    }

  /* Return failure */

  return false;
}

/****************************************************************************
 * Name: test_filepath
 *
 * Description:
 *   Test if a regular file exists at this path.
 *
 ****************************************************************************/

static bool test_filepath(const char *filepath)
{
  struct stat statbuf;
  int ret;

  ret = stat(filepath, &statbuf);
  if (ret < 0)
    {
      return false;
    }

  return S_ISREG(statbuf.st_mode);
}

/****************************************************************************
 * Name: test_dirpath
 *
 * Description:
 *   Test if a regular file exists at this path.
 *
 ****************************************************************************/

static bool test_dirpath(const char *filepath)
{
  struct stat statbuf;
  int ret;

  ret = stat(filepath, &statbuf);
  if (ret < 0)
    {
      return false;
    }

  return S_ISDIR(statbuf.st_mode);
}

/****************************************************************************
 * Name: foreach_dirent
 *
 * Description:
 *   Given a directory path, call the provided function for each entry in
 *   the directory.
 *
 ****************************************************************************/

static int foreach_dirent(const char *dirpath, direntcb_t cb, void *arg)
{
  DIR *dirp;
  struct dirent *result;
  struct dirent entry;
  int ret;

  dirp = opendir(dirpath);
  if (dirp == NULL)
    {
      fprintf(stderr, "ERROR: Failed to open directory '%s': %s\n",
              dirpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  for (; ; )
    {
      ret = readdir_r(dirp, &entry, &result);
      if (ret != 0)
        {
          fprintf(stderr,
                  "ERROR: Failed to reed directory '%s' entry: %s\n",
                  dirpath, strerror(ret));
           closedir(dirp);
           exit(EXIT_FAILURE);
        }

      if (result == NULL)
        {
          break;
        }

      /* Skip over the . and .. hard links */

      if (strcmp(entry.d_name, ".") == 0 || strcmp(entry.d_name, "..") == 0)
        {
          continue;
        }

      ret = cb(dirpath, &entry, arg);
      if (ret != 0)
        {
          break;
        }
    }

  closedir(dirp);
  return ret;
}

/****************************************************************************
 * Name: enum_architectures
 *
 * Description:
 *   Enumerate all architecture directory names.
 *
 ****************************************************************************/

static int enum_architectures(const char *dirpath, struct dirent *entry,
                              void *arg)
{
  char *archpath;
  char *testpath;

  /* All architecture directories should contain a Kconfig file, an include/
   * directory, and a src/ directory.
   */

  asprintf(&archpath, "%s%c%s", dirpath, g_delim, entry->d_name);
  asprintf(&testpath, "%s%cKconfig", archpath, g_delim);
  if (test_filepath(testpath))
    {
      free(testpath);
      asprintf(&testpath, "%s%cinclude", archpath, g_delim);
      if (test_dirpath(testpath))
        {
          free(testpath);
          asprintf(&testpath, "%s%csrc", archpath, g_delim);
          if (test_dirpath(testpath))
            {
              if (g_narch >= MAX_ARCHITECTURES)
                {
                  fprintf(stderr,
                          "ERROR: Too many architecture directories found\n");
                  exit(EXIT_FAILURE);
                }

              g_arch[g_narch] = strdup(entry->d_name);
              g_narch++;
            }
        }
    }

  free(testpath);
  free(archpath);
  return 0;
}

/****************************************************************************
 * Name: enum_mcus
 *
 * Description:
 *   Enumerate all MCU directory names.
 *
 ****************************************************************************/

static int enum_mcus(const char *dirpath, struct dirent *entry, void *arg)
{
  char *mcupath;
  char *testpath;

  /* All MCU directories should contain a Kconfig and a Make.defs file. */

  asprintf(&mcupath, "%s%c%s", dirpath, g_delim, entry->d_name);
  asprintf(&testpath, "%s%cKconfig", mcupath, g_delim);
  if (test_filepath(testpath))
    {
      free(testpath);
      asprintf(&testpath, "%s%cMake.defs", mcupath, g_delim);
      if (test_filepath(testpath))
        {
          if (g_nmcu >= MAX_MCUS)
            {
              fprintf(stderr,
                      "ERROR: Too many MCU directories found\n");
              exit(EXIT_FAILURE);
            }

          g_mcu[g_nmcu] = strdup(entry->d_name);
          g_nmcu++;
        }
    }

  free(testpath);
  free(mcupath);
  return 0;
}

/****************************************************************************
 * Name: enum_board_configurations
 *
 * Description:
 *   Enumerate all configurations for boards find the configuration
 *   directory for the selected MCU.
 *
 ****************************************************************************/

static int enum_board_configurations(const char *dirpath,
                                     struct dirent *entry, void *arg)
{
  char *configpath;
  char *varvalue;
  int ret = 0;

  /* All board directories should contain a defconfig file. */

  asprintf(&configpath, "%s%c%s%cdefconfig",
           dirpath, g_delim, entry->d_name, g_delim);
  if (test_filepath(configpath))
    {
      /* We don't want all board configurations, we only want the name of
       * the board that includes a configuration with:
       *
       *   CONFIG_ARCH_CHIP="xxxx"
       *
       * Where xxxx is the selected MCU name.
       */

      asprintf(&varvalue, "\"%s\"", g_selected_mcu);
      if (check_variable(configpath, "CONFIG_ARCH_CHIP", varvalue))
        {
          /* Found it... add the board name to the list of boards for the
           * selected MCU.
           */

          if (g_nboard >= MAX_BOARDS)
            {
              fprintf(stderr,
                      "ERROR: Too many board configurations found\n");
              exit(EXIT_FAILURE);
            }

          g_board[g_nboard] = strdup(arg);
          g_nboard++;

          /* If we have not yet extracted the architecture family, then do
           * that here.
           */

          if (g_selected_family == NULL)
            {
              char *family;

              if (find_variable(configpath, "CONFIG_ARCH_FAMILY", &family))
                {
                  g_selected_family = strdup(family);
                }
            }

          /* Stop the enumeration if we find a match.  Continue if not...
           * that is because one board might possible support multiple
           * architectures.
           */

          ret = 1;
        }

      free(varvalue);
    }

  free(configpath);
  return ret;
}

/****************************************************************************
 * Name: enum_boards
 *
 * Description:
 *   Enumerate all boards find the configuration directory for the selected
 *   MCU.
 *
 ****************************************************************************/

static int enum_boards(const char *dirpath, struct dirent *entry, void *arg)
{
  char *boardpath;
  char *testpath;

  /* All board directories should contain a Kconfig file, an include/
   * directory, and a src/ directory.
   */

  asprintf(&boardpath, "%s%c%s", dirpath, g_delim, entry->d_name);
  asprintf(&testpath, "%s%cKconfig", boardpath, g_delim);
  if (test_filepath(testpath))
    {
      free(testpath);
      asprintf(&testpath, "%s%cinclude", boardpath, g_delim);
      if (test_dirpath(testpath))
        {
          free(testpath);
          asprintf(&testpath, "%s%csrc", boardpath, g_delim);
          if (test_dirpath(testpath))
            {
              /* Enumerate the board configurations */

              foreach_dirent(boardpath, enum_board_configurations,
                             entry->d_name);
            }
        }
    }

  free(testpath);
  free(boardpath);
  return 0;
}

/****************************************************************************
 * Name: list_select
 *
 * Description:
 *   Select one value from a list.
 *
 ****************************************************************************/

char *list_select(char **list, unsigned nitems)
{
  char ch;
  int ndx;
  int i;

  /* Show the list */

  for (i = 0, ch = '1'; i < nitems; i++)
    {
      printf("  %c. %s\n", ch, list[i]);
      if (ch == '9')
        {
          ch = 'a';
        }
      else if (ch == 'z')
        {
          ch = 'A';
        }
      else
        {
          ch++;
        }
    }

  for (; ; )
    {
      bool input = false;

      printf("Enter [1");
      if (nitems > 1)
      {
         printf("-%c", nitems >= 9 ? '9' : '0' + nitems);
         if (nitems > 9)
           {
             printf(",a");
             if (nitems > 10)
               {
                 printf("-%c", 'a' + nitems - 10);
                 if (nitems > 35)
                   {
                     printf(",A");
                     if (nitems > 36)
                       {
                         printf("-%c", 'A' + nitems - 36);
                       }
                   }
               }
           }
      }

      printf("]: ");

      do
        {
          ch = getchar();
          if (ch >= '1' && ch <= '9')
            {
              ndx = ch - '1';
            }
          else if (ch >= 'a' && ch <= 'z')
            {
              ndx = ch - 'a' + 9;
            }
          else if (ch >= 'A' && ch <= 'Z')
            {
              ndx = ch - 'A' + 35;
            }
          else if (ch == '\n')
            {
              continue;
            }
          else
            {
              printf("Invalid selection: %c -- Try again\n", ch);
              input = true;
              continue;
            }

          if (ndx < nitems)
            {
              return list[ndx];
            }
          else
            {
              printf("Invalid selection: %c -- Try again\n", ch);
              input = true;
            }
        }
      while (!input);
    }
}

/****************************************************************************
 * Name: create_config
 *
 * Description:
 *   Generate a bogus .config file.  There is only sufficient information
 *   in this bogus .config to estable the correct symbolic links.
 *
 ****************************************************************************/

static void create_config(void)
{
  FILE *stream;

  stream = fopen(".config", "w");
  if (!stream)
    {
       fprintf(stderr, "ERROR: failed to open .config for writing: %s\n",
               strerror(errno));
       exit(EXIT_FAILURE);
    }

  fprintf(stream, "CONFIG_ARCH=\"%s\"\n", g_selected_arch);
  if (g_selected_family != NULL)
    {
      fprintf(stream, "CONFIG_ARCH_FAMILY=%s\n", g_selected_family);
    }

  fprintf(stream, "CONFIG_ARCH_CHIP=\"%s\"\n", g_selected_mcu);
  fprintf(stream, "CONFIG_ARCH_BOARD=\"%s\"\n", g_selected_board);

  fclose(stream);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Program entry point.
 *
 ****************************************************************************/

int main(int argc, char **argv)
{
  char *archpath;

  /* Enumerate all of the architectures */

  g_narch = 0;
  foreach_dirent(g_archdir, enum_architectures, NULL);

  /* Select an architecture */

  printf("Select an architecture:\n");
  g_selected_arch = list_select(g_arch, g_narch);

  /* Enumerate the MCUs for the selected architecture */

  g_nmcu = 0;
  asprintf(&archpath, "%s%c%s%csrc",
           g_archdir, g_delim, g_selected_arch, g_delim);
  foreach_dirent(archpath, enum_mcus, NULL);

  /* Select an MCU */

  printf("Select an MCU for architecture=%s:\n", g_selected_arch);
  g_selected_mcu = list_select(g_mcu, g_nmcu);

  /* Enumerate the boards for the selected MCU */

  g_nboard = 0;
  foreach_dirent(g_configdir, enum_boards, NULL);

  /* Select an board */

  printf("Select a board for MCU=%s:\n", g_selected_mcu);
  g_selected_board = list_select(g_board, g_nboard);

  /* Then output a bogus .config file with enough information to establish
   * the correct symbolic links
   */

  create_config();
  return 0;
}
