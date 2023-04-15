/****************************************************************************
 * libs/libc/unistd/lib_setegid.c
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

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setegid
 *
 * Description:
 *   The setegid() function sets the effect group ID of the calling task
 *   group to gid.
 *
 * Input Parameters:
 *   gid - Identity to set the various process' group ID attributes to.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setegid(gid_t gid)
{
  /* NuttX only supports the group identity 'root' with a gid value of 0. */

  if (gid == 0)
    {
      return 0;
    }

  /* All other gid values are considered invalid and not supported by the
   * implementation.
   */

  set_errno(EINVAL);
  return -1;
}
