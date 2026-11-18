/****************************************************************************
 * fs/fs_reboot.c
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

#include <nuttx/irq.h>
#include <nuttx/reboot_notifier.h>

#include "fs_reboot.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_reboot_handler
 *
 * Description:
 *   Flush the filesystems before power-off or restart.
 *
 ****************************************************************************/

static int fs_reboot_handler(FAR struct notifier_block *nb,
                             unsigned long action, FAR void *data)
{
  /* sync() takes filesystem locks, which an interrupt handler cannot wait
   * for.
   */

  if ((action == SYS_POWER_OFF || action == SYS_RESTART) &&
      !up_interrupt_context())
    {
      sync();
    }

  return 0;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct notifier_block g_fs_reboot_nb =
{
  fs_reboot_handler
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_reboot_initialize
 *
 * Description:
 *   Register the filesystem reboot notifier.
 *
 ****************************************************************************/

void fs_reboot_initialize(void)
{
  register_reboot_notifier(&g_fs_reboot_nb);
}
