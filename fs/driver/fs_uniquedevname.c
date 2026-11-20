/****************************************************************************
 * fs/driver/fs_uniquedevname.c
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

#include <sys/stat.h>

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include "driver/driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_devno;
static mutex_t g_devno_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unique_devname
 *
 * Description:
 *   Create a unique temporary device name in the /dev/ directory of the
 *   pseudo-file system.  We cannot use mktemp for this because it will
 *   attempt to open() the file.
 *
 * Input Parameters:
 *   prefix - 'c' for a character device, 'b' for a block device
 *   devbuf - Buffer to store the generated device name
 *   len    - Length of the buffer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int unique_devname(char prefix, FAR char *devbuf, size_t len)
{
  struct stat statbuf;
  uint32_t devno;
  int ret;

  /* Loop until we get a unique device name */

  for (; ; )
    {
      /* Get the mutex protecting the path number */

      ret = nxmutex_lock(&g_devno_lock);
      if (ret < 0)
        {
          ferr("ERROR: nxmutex_lock failed: %d\n", ret);
          return ret;
        }

      /* Get the next device number and release the mutex */

      devno = ++g_devno;
      nxmutex_unlock(&g_devno_lock);

      /* Construct the full device number */

      devno &= 0xffffff;
      snprintf(devbuf, len, "/dev/tmp%c%06lx", prefix,
               (unsigned long)devno);

      /* Make sure that file name is not in use */

      ret = nx_stat(devbuf, &statbuf, 1);
      if (ret < 0)
        {
          DEBUGASSERT(ret == -ENOENT);
          return OK;
        }

      /* It is in use, try again */
    }
}
