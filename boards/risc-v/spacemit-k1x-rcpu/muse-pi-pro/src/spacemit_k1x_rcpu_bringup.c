/****************************************************************************
 * boards/risc-v/spacemit-k1x-rcpu/muse-pi-pro/src/spacemit_k1x_rcpu_bringup.c
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

#include <syslog.h>
#include <sys/mount.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "muse_pi_pro.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_bringup
 *
 * Description:
 *   Register the board's devices.  Nothing to register yet.
 *
 ****************************************************************************/

int spacemit_k1x_rcpu_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount procfs at /proc.
   *
   * The shell's ps, free and df commands read their data from procfs, and so
   * does anything else that wants to look at the running system.  NuttX has
   * to be told to mount it: CONFIG_FS_PROCFS only compiles the file system
   * in, it does not put it anywhere.
   *
   * A failure here is reported but not fatal: the system is perfectly usable
   * without /proc, it just cannot answer questions about itself.
   */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  return OK;
}
