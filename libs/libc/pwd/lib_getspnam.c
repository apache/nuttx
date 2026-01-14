/****************************************************************************
 * libs/libc/pwd/lib_getspnam.c
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
#include <shadow.h>
#include <string.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct spwd *getspnam(FAR const char *name)
{
  FAR struct spwd *result = NULL;

#ifdef CONFIG_LIBC_PASSWD_FILE
  getspnam_r(name, &g_spwd, g_passwd_buffer,
             sizeof(g_passwd_buffer), &result);
#else
  if (strcmp(name, ROOT_NAME) == 0)
    {
      size_t nsize = sizeof(ROOT_NAME);
      size_t psize = sizeof(ROOT_PWDP);
      result = &g_spwd;

      result->sp_namp = g_passwd_buffer;
      result->sp_pwdp = &g_passwd_buffer[nsize];

      strlcpy(result->sp_namp, ROOT_NAME, nsize);
      strlcpy(result->sp_pwdp, ROOT_PWDP, psize);
    }
#endif

  return result;
}
