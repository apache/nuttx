/****************************************************************************
 * libs/libc/grp/lib_getgrent.c
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

#include <grp.h>

#include "grp/lib_grp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Index of the next entry that getgrent() returns */

static int g_group_index;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setgrent
 *
 * Description:
 *   Rewind the group database, so that the next getgrent() returns the
 *   first entry.
 *
 ****************************************************************************/

void setgrent(void)
{
  g_group_index = 0;
}

/****************************************************************************
 * Name: endgrent
 *
 * Description:
 *   Close the group database.  The next getgrent() returns the first entry.
 *
 ****************************************************************************/

void endgrent(void)
{
  g_group_index = 0;
}

/****************************************************************************
 * Name: getgrent
 *
 * Description:
 *   Return the next entry of the group database.
 *
 * Returned Value:
 *   A pointer to a statically allocated group structure, or NULL if there
 *   are no more entries or an error occurs.
 *
 ****************************************************************************/

FAR struct group *getgrent(void)
{
#ifdef CONFIG_LIBC_GROUP_FILE
  int ret;

  ret = grp_findby_index(g_group_index, &g_group, g_group_buffer,
                         GRPBUF_RESERVE_SIZE);
  if (ret != 1)
    {
      return NULL;
    }

  g_group_index++;
  return &g_group;
#else
  /* The only group is root */

  if (g_group_index != 0)
    {
      return NULL;
    }

  g_group_index++;
  return getgrbuf(ROOT_GID, ROOT_NAME, ROOT_PASSWD);
#endif
}
