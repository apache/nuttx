/****************************************************************************
 * libs/libc/pwd/lib_getpwnam.c
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

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpwnam
 *
 * Description:
 *   The getpwnam() function searches the user database for an entry with a
 *   matching name.
 *
 * Input Parameters:
 *   name - The user name to return a passwd structure for.
 *
 * Returned Value:
 *   A pointer to a statically allocated passwd structure, or NULL if no
 *   matching entry is found or an error occurs.  Applications wishing to
 *   check for error situations should set errno to 0 before calling
 *   getpwnam().  If getpwnam() returns a null pointer and errno is set to
 *   non-zero, an error occurred.
 *
 ****************************************************************************/

FAR struct passwd *getpwnam(FAR const char *name)
{
#ifdef CONFIG_LIBC_PASSWD_FILE
  int ret;

  ret = pwd_findby_name(name, &g_passwd, g_passwd_buffer,
                        CONFIG_LIBC_PASSWD_LINESIZE);
  if (ret != 1)
    {
      return NULL;
    }

  return &g_passwd;
#else
  if (strcmp(name, ROOT_NAME))
    {
      return NULL;
    }

  return getpwbuf(ROOT_UID, ROOT_GID, ROOT_NAME, ROOT_NAME, ROOT_DIR,
                  ROOT_SHELL);
#endif
}
