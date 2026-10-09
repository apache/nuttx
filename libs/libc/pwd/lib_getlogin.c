/****************************************************************************
 * libs/libc/pwd/lib_getlogin.c
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

#include <errno.h>
#include <limits.h>
#include <pwd.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of the buffer for the strings of one user database entry */

#define GETLOGIN_PWBUF_SIZE 256

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getlogin_r
 *
 * Description:
 *   Put the login name of the user into buf.  NuttX has no login sessions,
 *   so the login name is the name of the real user ID in the user database.
 *
 * Input Parameters:
 *   buf     - The buffer for the name.
 *   bufsize - The size of buf in bytes.
 *
 * Returned Value:
 *   Zero on success.  ERANGE if the name does not fit in buf, ENXIO if the
 *   user database has no entry for the real user ID.
 *
 ****************************************************************************/

int getlogin_r(FAR char *buf, size_t bufsize)
{
  char pwbuf[GETLOGIN_PWBUF_SIZE];
  FAR struct passwd *result;
  struct passwd pwd;
  int ret;

  ret = getpwuid_r(getuid(), &pwd, pwbuf, sizeof(pwbuf), &result);
  if (ret != 0)
    {
      return ret;
    }

  if (result == NULL)
    {
      return ENXIO;
    }

  if (strlcpy(buf, pwd.pw_name, bufsize) >= bufsize)
    {
      return ERANGE;
    }

  return 0;
}

/****************************************************************************
 * Name: getlogin
 *
 * Description:
 *   Return the login name of the user.  See getlogin_r().  The string is
 *   overwritten by the next call to getlogin().
 *
 * Returned Value:
 *   A pointer to the name, or NULL with errno set if there is no name.
 *
 ****************************************************************************/

FAR char *getlogin(void)
{
  static char name[NAME_MAX + 1];
  int ret;

  ret = getlogin_r(name, sizeof(name));
  if (ret != 0)
    {
      set_errno(ret);
      return NULL;
    }

  return name;
}
