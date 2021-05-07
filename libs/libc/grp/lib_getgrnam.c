/****************************************************************************
 * libs/libc/grp/lib_getgrnam.c
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
#include <grp.h>

#include "grp/lib_grp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrnam
 *
 * Description:
 *   The getgrnam() function searches the group database for an entry with
 *   a matching name.
 *
 * Input Parameters:
 *   name - The group name to return a group structure for
 *
 * Returned Value:
 *   A pointer to a statically allocated group structure, or NULL if no
 *   matching entry is found or an error occurs.  Applications wishing to
 *   check for error situations should set errno to 0 before calling
 *   getgrnam().  If getgrnam() returns a null pointer and errno is set to
 *   non-zero, an error occurred.
 *
 ****************************************************************************/

FAR struct group *getgrnam(FAR const char *name)
{
#ifdef CONFIG_LIBC_GROUP_FILE
  int ret;

  ret = grp_findby_name(name, &g_group, g_group_buffer, GRPBUF_RESERVE_SIZE);
  if (ret != 1)
    {
      return NULL;
    }

  return &g_group;
#else
  if (strcmp(name, "root"))
    {
      return NULL;
    }

  return getgrbuf(ROOT_GID, ROOT_NAME, ROOT_PASSWD);
#endif
}
