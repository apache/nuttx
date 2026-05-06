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

#include "binfmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCRIPT_HEADER_MAX 128

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
 * Name: script_loadbinary
 *
 * Description:
 *   Verify that the file starts with "#!" and parse the shebang line to
 *   extract the interpreter path (and optional argument), then delegate
 *   loading to the interpreter via load_module().  Sets binp->filename
 *   so exec_module() launches the interpreter with the script path
 *   preserved in the forwarded argv.
 *
 ****************************************************************************/

static int script_loadbinary(FAR struct binary_s *binp,
                             FAR const char *filename,
                             FAR const struct symtab_s *exports,
                             int nexports)
{
  char header[SCRIPT_HEADER_MAX];
  FAR char *interp;
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

  /* Terminate the interpreter token at the first whitespace.  Any
   * optional argument after the interpreter is ignored -- the
   * interpreter is relaunched with the script path as argv[1], which
   * is enough for sh / lua / python to auto-execute the script.
   */

  end = interp;
  while (*end != '\0' && !isspace(*end))
    {
      end++;
    }

  *end = '\0';

  binfo("Script: interp='%s' script='%s'\n", interp, filename);

  /* Ask exec_module() to keep the original filename as the task name and
   * forward the unchanged argv, so the interpreter receives the script
   * path (the original filename) at argv[1].
   */

  binp->filename = true;

  return load_module(binp, interp, exports, nexports);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: script_initialize
 *
 * Description:
 *   Register the script binary format handler.  Called from
 *   binfmt_initialize() AFTER builtin and ELF handlers so those
 *   formats are tried first.
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
