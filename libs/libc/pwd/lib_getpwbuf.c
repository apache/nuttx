/****************************************************************************
 * libs/libc/pwd/lib_getpwbuf.c
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

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpwbuf
 *
 * Description:
 *   libc/grp internal helper function for getpwgid and getpwnam to allocate
 *   and setup a passwd structure once a matching entry has been found.
 *
 * Input Parameters:
 *   uid   - Value to set the passwd structure's pw_uid field to.
 *   gid   - Value to set the passwd structure's pw_gid field to.
 *   name  - Value to set the passwd structure's pw_name field to.
 *   geocs - Value to set the passwd structure's pw_gecos field to.
 *   dir   - Value to set the passwd structure's pw_dir field to.
 *   shell - Value to set the passwd structure's pw_shell field to.
 *
 * Returned Value:
 *   A pointer to a statically allocated passwd structure, or NULL if an
 *   error occurs, in which case errno is set appropriately.
 *
 ****************************************************************************/

FAR struct passwd *getpwbuf(uid_t uid, gid_t gid, FAR const char *name,
                            FAR const char *gecos, FAR const char *dir,
                            FAR const char *shell)
{
  FAR struct passwd *pwd = NULL;
  int ret = getpwbuf_r(uid, gid, name, gecos, dir, shell, &g_passwd,
                       g_passwd_buffer, sizeof(g_passwd_buffer), &pwd);
  return ret == 0 ? pwd : NULL;
}
