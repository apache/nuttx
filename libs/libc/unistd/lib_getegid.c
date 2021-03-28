/****************************************************************************
 * libs/libc/unistd/lib_getegid.c
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

#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getegid
 *
 * Description:
 *   The getegid() function will the effective group ID of the calling task
 *   group.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The effective group ID of the calling task group.
 *
 ****************************************************************************/

gid_t getegid(void)
{
#ifdef CONFIG_SCHED_USER_IDENTITY
  /* If we have real UID/GID support, then treat the real group as the
   * effective group ID.
   */

  return getgid();
#else
  /* Return group identity 'root' with a gid value of 0. */

  return 0;
#endif
}
