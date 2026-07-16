/****************************************************************************
 * libs/libc/unistd/lib_getresuid.c
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

#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getresuid
 *
 * Description:
 *   The getresuid() function gets the real, effective, and saved set-user
 *   IDs of the calling process.
 *
 * Input Parameters:
 *   ruid - Location to return the real user ID, or NULL.
 *   euid - Location to return the effective user ID, or NULL.
 *   suid - Location to return the saved set-user ID, or NULL.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int getresuid(FAR uid_t *ruid, FAR uid_t *euid, FAR uid_t *suid)
{
  /* NuttX only supports the user identity 'root' with a uid value of 0. */

  if (ruid != NULL)
    {
      *ruid = 0;
    }

  if (euid != NULL)
    {
      *euid = 0;
    }

  if (suid != NULL)
    {
      *suid = 0;
    }

  return 0;
}
