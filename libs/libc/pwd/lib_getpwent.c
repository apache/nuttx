/****************************************************************************
 * libs/libc/pwd/lib_getpwent.c
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

#include <string.h>
#include <pwd.h>
#include <errno.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setpwent
 *
 * Description:
 *   The setpwent() function rewinds to the beginning of the password
 *   database.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void setpwent(void)
{
  g_passwd_index = 0;
}

/****************************************************************************
 * Name: endpwent
 *
 * Description:
 *   The endpwent() function is used to close the password database after
 *   all processing has been performed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void endpwent(void)
{
  g_passwd_index = 0;
}

/****************************************************************************
 * Name: getpwent
 *
 * Description:
 *   The getpwent() function returns a pointer to a structure
 *   containing the broken-out fields of a record from the password
 *   database.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The getpwent() function returns a pointer to a passwd structure,
 *   or NULL if there are no more entries or an error occurred.  If an
 *   error occurs, errno is set to indicate the error.  If one wants
 *   to check errno after the call, it should be set to zero before
 *   the call.
 *
 ****************************************************************************/

FAR struct passwd *getpwent(void)
{
  FAR struct passwd *pwd;
  getpwent_r(&g_passwd, g_passwd_buffer,
             CONFIG_LIBC_PASSWD_LINESIZE, &pwd);
  return pwd;
}

/****************************************************************************
 * Name: getpwent_r
 *
 * Description:
 *   The functions getpwent_r() are the reentrant versions of getpwent().
 *   The former reads the next passwd entry from the stream
 *   initialized by setpwent().
 *
 * Input Parameters:
 *   pwd     - contains the struct passwd that was read from the stream,
 *             if any.
 *   buf     - contains additional strings, if any.
 *   buflen  - specifies the size of buf.
 *   *result - returns a pointer to the struct passwd in *pwd.
 *
 * Returned Value:
 *   On success, return 0, and *pwbufp shall contain a pointer to the result.
 *   On failure, *pwbufp shall contain NULL. return an error an follows:
 *     ENOENT: No more password entries.
 *     ERANGE: Not enough buffer space. Specify a larger buffer and
 *             try again.
 *
 ****************************************************************************/

int getpwent_r(FAR struct passwd *pwd,
               FAR char *buf, size_t buflen,
               FAR struct passwd **result)
{
  int ret;
#ifdef CONFIG_LIBC_PASSWD_FILE
  ret = pwd_findby_index(g_passwd_index, pwd, buf, buflen);
  if (ret != 1)
    {
      *result = NULL;
      return ret < 0 ? ret : ENOENT;
    }

  *result = pwd;
  g_passwd_index++;
  return OK;
#else
  if (g_passwd_index != 0)
    {
      /* The only known user is 'root', if index in not 0. Thus, report
       * back that no match was found.
       */

      *result = NULL;
      return ENOENT;
    }

  ret = getpwbuf_r(ROOT_UID, ROOT_GID, ROOT_NAME, ROOT_NAME, ROOT_DIR,
                   ROOT_SHELL, ROOT_PASSWD, pwd, buf, buflen, result);
  if (ret == 0)
    {
      g_passwd_index++;
    }

  return ret;
#endif
}
