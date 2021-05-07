/****************************************************************************
 * libs/libc/pwd/lib_getpwnamr.c
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

#include <pwd.h>
#include <string.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpwnam_r
 *
 * Description:
 *   The getpwnam_r() function searches the user database for an entry with a
 *   matching name and stores the retrieved passwd structure in the space
 *   pointed to by pwd.
 *
 * Input Parameters:
 *   name - The name to return a passwd structure for.
 *   pwd - Pointer to the space to store the retrieved passwd structure in.
 *   buf - The string fields pointed to by the passwd struct are stored here.
 *   buflen - The length of buf in bytes.
 *   result - Pointer to the resulting passwd struct, or NULL in case of
 *            fail.
 *
 * Returned Value:
 *   On success getpwnam_r returns 0 and sets *result to pwd.  If no match
 *   is found, 0 is returned and *result is set to NULL.  In case of failure
 *   an error number is returned.
 *
 ****************************************************************************/

int getpwnam_r(FAR const char *name, FAR struct passwd *pwd, FAR char *buf,
               size_t buflen, FAR struct passwd **result)
{
#ifdef CONFIG_LIBC_PASSWD_FILE
  int ret;

  ret = pwd_findby_name(name, pwd, buf, buflen);
  if (ret != 1)
    {
      *result = NULL;
      return ret < 0 ? -ret : 0;
    }

  *result = pwd;
  return 0;
#else
  if (strcmp(name, ROOT_NAME))
    {
      /* The only known user is 'root', which has a uid of 0.  Thus, report
       * back that no match was found.
       */

      *result = NULL;
      return 0;
    }

  return getpwbuf_r(ROOT_UID, ROOT_GID, ROOT_NAME, ROOT_DIR, ROOT_SHELL, pwd,
                    buf, buflen, result);
#endif
}
