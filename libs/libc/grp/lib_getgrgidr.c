/****************************************************************************
 * libs/libc/grp/lib_getgrgidr.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrgid_r
 *
 * Description:
 *   The getgrgid_r() function searches the group database for an entry with
 *   a matching gid and stores the retrieved group structure in the space
 *   pointed to by grp.
 *
 * Input Parameters:
 *   gid - The gid to return a group structure for.
 *   grp - Pointer to the space to store the retrieved group structure in.
 *   buf - The string fields pointed to by the group struct are stored here.
 *   buflen - The length of buf in bytes.
 *   result - Pointer to the resulting group struct, or NULL in case of fail.
 *
 * Returned Value:
 *   On success getgrgid_r returns 0 and sets *result to grp.  If no match
 *   is found, 0 is returned and *result is set to NULL.  In case of failure
 *   an error number is returned.
 *
 ****************************************************************************/

int getgrgid_r(gid_t gid, FAR struct group *grp, FAR char *buf,
               size_t buflen, FAR struct group **result)
{
#ifdef CONFIG_LIBC_GROUP_FILE
  int ret;

  ret = grp_findby_gid(gid, grp, buf, buflen);
  if (ret != 1)
    {
      *result = NULL;
      return ret < 0 ? -ret : 0;
    }

  *result = grp;
  return 0;
#else
  if (gid != ROOT_GID)
    {
      /* The only known group is 'root', which has a gid of 0.  Thus, report
       * back that no match was found.
       */

      *result = NULL;
      return 0;
    }

  return getgrbuf_r(ROOT_GID, ROOT_NAME, ROOT_PASSWD, grp, buf, buflen,
                    result);
#endif
}
