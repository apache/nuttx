/****************************************************************************
 * boards/x86/qemu/qemu-i486/src/qemu_bringup.c
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

#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <nuttx/debug.h>
#include <nuttx/video/fb.h>
#include <nuttx/fs/fs.h>

#include "x86_internal.h"
#include "qemu_vga.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 *
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int qemu_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the proc filesystem */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_QEMU_VGA
  ret = qemu_vga();
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize QEMU VGA: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      serr("ERROR: fb_register() failed: %d\n", ret);
    }
#endif

  return ret;
}
