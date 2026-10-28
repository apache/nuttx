/****************************************************************************
 * libs/libc/unistd/lib_getusershell.c
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

#include <ctype.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <nuttx/lib/lib.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The shells when _PATH_SHELLS cannot be read, as on other systems */

static FAR const char * const g_default_shells[] =
{
  _PATH_BSHELL, "/bin/csh", NULL
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getusershell
 *
 * Description:
 *   Return the next shell of _PATH_SHELLS: on each line, the word that
 *   starts at the first '/'.  A '#' starts a comment.  If the file cannot
 *   be read, the shells are /bin/sh and /bin/csh.  The string stays valid
 *   until the next call.
 *
 * Returned Value:
 *   The next shell, or NULL at the end of the list.
 *
 ****************************************************************************/

FAR char *getusershell(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct usershell_s *us = info->ta_usershell;
  FAR char *shell;
  FAR char *p;

  /* Each task group has its own state */

  if (us == NULL)
    {
      us = lib_zalloc(sizeof(struct usershell_s));
      if (us == NULL)
        {
          return NULL;
        }

      us->us_file = fopen(_PATH_SHELLS, "re");
      info->ta_usershell = us;
    }

  if (us->us_file == NULL)
    {
      shell = (FAR char *)g_default_shells[us->us_next];
      if (shell != NULL)
        {
          us->us_next++;
        }

      return shell;
    }

  while (getline(&us->us_line, &us->us_size, us->us_file) >= 0)
    {
      for (p = us->us_line; *p != '\0' && *p != '#' && *p != '/'; p++)
        {
        }

      if (*p != '/')
        {
          continue;
        }

      shell = p;
      while (*p != '\0' && *p != '#' && !isspace((unsigned char)*p))
        {
          p++;
        }

      *p = '\0';
      return shell;
    }

  return NULL;
}

/****************************************************************************
 * Name: setusershell
 *
 * Description:
 *   Make the next getusershell() return the first shell again.
 *
 ****************************************************************************/

void setusershell(void)
{
  endusershell();
}

/****************************************************************************
 * Name: endusershell
 *
 * Description:
 *   Close _PATH_SHELLS and free the memory of getusershell().
 *
 ****************************************************************************/

void endusershell(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct usershell_s *us = info->ta_usershell;

  if (us == NULL)
    {
      return;
    }

  if (us->us_file != NULL)
    {
      fclose(us->us_file);
    }

  lib_free(us->us_line);
  lib_free(us);
  info->ta_usershell = NULL;
}
