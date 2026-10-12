/****************************************************************************
 * binfmt/script.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/config.h>

#include <ctype.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/binfmt/binfmt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include "binfmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCRIPT_HEADER_MAX 128

/* The interpreter of a script can be a script, up to four levels, as in
 * Linux.
 */

#define SCRIPT_DEPTH_MAX  4

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int script_loadbinary(FAR struct binary_s *binp,
                             FAR const char *filename,
                             FAR const struct symtab_s *exports,
                             int nexports);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_script_binfmt =
{
  NULL,                /* next */
  script_loadbinary,   /* load */
  NULL,                /* unload */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: script_setargv
 *
 * Description:
 *   Set binp->argv to: interp, arg (if not NULL), filename, then the
 *   entries after the first of the old list (a script whose interpreter is
 *   a script).  All strings are copied into one allocation.
 *
 ****************************************************************************/

static int script_setargv(FAR struct binary_s *binp, FAR const char *interp,
                          FAR const char *arg, FAR const char *filename)
{
  FAR char **old = binp->argv;
  FAR const char *src[3];
  FAR char **list;
  FAR char *str;
  size_t size = 0;
  int nsrc = 0;
  int nold = 0;
  int nargs;
  int i;

  src[nsrc++] = interp;
  if (arg != NULL)
    {
      src[nsrc++] = arg;
    }

  src[nsrc++] = filename;

  if (old != NULL)
    {
      while (old[nold] != NULL)
        {
          nold++;
        }
    }

  nargs = nsrc + (nold > 1 ? nold - 1 : 0);
  for (i = 0; i < nsrc; i++)
    {
      size += strlen(src[i]) + 1;
    }

  for (i = 1; i < nold; i++)
    {
      size += strlen(old[i]) + 1;
    }

  list = kmm_malloc((nargs + 1) * sizeof(FAR char *) + size);
  if (list == NULL)
    {
      return -ENOMEM;
    }

  str = (FAR char *)&list[nargs + 1];
  for (i = 0; i < nargs; i++)
    {
      FAR const char *s = i < nsrc ? src[i] : old[i - nsrc + 1];

      list[i] = strcpy(str, s);
      str += strlen(s) + 1;
    }

  list[nargs] = NULL;
  binp->argv = list;
  return OK;
}

/****************************************************************************
 * Name: script_loadbinary
 *
 * Description:
 *   Verify that the file starts with "#!" and parse the shebang line to
 *   extract the interpreter path (and optional argument), then delegate
 *   loading to the interpreter via load_module().  Sets binp->argv
 *   so exec_module() runs the interpreter with the optional argument and
 *   the script path before the caller's arguments.
 *
 ****************************************************************************/

static int script_loadbinary(FAR struct binary_s *binp,
                             FAR const char *filename,
                             FAR const struct symtab_s *exports,
                             int nexports)
{
  char header[SCRIPT_HEADER_MAX];
  FAR char **oldargv = binp->argv;
  FAR char *interp;
  FAR char *arg = NULL;
  FAR char *end;
  struct file file;
  int ret;

  binfo("Attempting script load: %s\n", filename);

  ret = file_open(&file, filename, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_read(&file, header, sizeof(header) - 1);
  file_close(&file);

  if (ret < 0)
    {
      return ret;
    }

  header[ret] = '\0';

  /* Check for "#!" magic */

  if (header[0] != '#' || header[1] != '!')
    {
      return -ENOEXEC;
    }

  if (binp->scriptdepth >= SCRIPT_DEPTH_MAX)
    {
      berr("ERROR: Too many script interpreters for %s\n", filename);
      return -ELOOP;
    }

  /* Terminate at newline */

  end = strchr(header + 2, '\n');
  if (end != NULL)
    {
      *end = '\0';
    }

  /* Skip whitespace after "#!" to find interpreter path */

  interp = header + 2;
  while (isspace(*interp))
    {
      interp++;
    }

  if (*interp == '\0')
    {
      berr("ERROR: Empty interpreter in %s\n", filename);
      return -ENOEXEC;
    }

  /* The interpreter ends at the first whitespace.  The rest of the line,
   * without the whitespace around it, is one optional argument, as in
   * Linux.
   */

  end = interp;
  while (*end != '\0' && !isspace(*end))
    {
      end++;
    }

  if (*end != '\0')
    {
      *end++ = '\0';
      while (isspace(*end))
        {
          end++;
        }

      if (*end != '\0')
        {
          arg = end;
          end = arg + strlen(arg);
          while (isspace(*(end - 1)))
            {
              end--;
            }

          *end = '\0';
        }
    }

  binfo("Script: interp='%s' arg='%s' script='%s'\n",
        interp, arg ? arg : "", filename);

  /* The interpreter runs with: interp [arg] filename argv[1] ...
   * filename is the path that was loaded, after the PATH search.
   */

  ret = script_setargv(binp, interp, arg, filename);
  if (ret < 0)
    {
      return ret;
    }

  binp->scriptdepth++;
  ret = load_module(binp, interp, exports, nexports);
  binp->scriptdepth--;
  if (ret < 0)
    {
      kmm_free(binp->argv);
      binp->argv = oldargv;
      return ret;
    }

  kmm_free(oldargv);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: script_initialize
 *
 * Description:
 *   Register the script binary format handler.  Called from
 *   binfmt_initialize() BEFORE the other handlers: register_binfmt() adds
 *   to the head of the list, so the builtin and ELF formats are tried
 *   first.
 *
 ****************************************************************************/

int script_initialize(void)
{
  int ret;

  binfo("Registering Script Loader\n");

  ret = register_binfmt(&g_script_binfmt);
  if (ret < 0)
    {
      berr("ERROR: Failed to register script binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: script_uninitialize
 ****************************************************************************/

void script_uninitialize(void)
{
  unregister_binfmt(&g_script_binfmt);
}
