/****************************************************************************
 * boards/arm/rtl8720f/rtl8720f_evb/src/rtl8720f_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/board.h>

#include "rtl8720f_rtl8720f_evb.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK ROM inter-core IPC-semaphore RTOS hooks.  rtos_* come from the NuttX
 * os_wrapper (ameba_os_wrap.c); IPC_* are SDK ROM symbols.
 */

extern void rtos_critical_enter(uint32_t component_id);
extern void rtos_critical_exit(uint32_t component_id);
extern void rtos_time_delay_ms(uint32_t ms);
extern void IPC_patch_function(void (*enter)(uint32_t),
                               void (*leave)(uint32_t), uint32_t lock_id);
extern void IPC_SEMDelayStub(void (*delay)(uint32_t));

/* RTOS_CRITICAL_SEMA in the SDK os_wrapper_critical.h enum.  Our
 * rtos_critical_enter/exit ignore the component id, so the value is only
 * passed through to satisfy the ROM signature.
 */

#define AMEBA_RTOS_CRITICAL_SEMA  9

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8720f_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int rtl8720f_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmp file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTL8720F_FLASH_FS
  /* Mount the persistent data filesystem (littlefs on the SPI NOR VFS1
   * partition) at /data before WiFi, so the WiFi KV store is available.
   */

  /* The SDK flash erase/program path IPC-pauses the NP (km4ns) while the AP
   * writes the shared XIP flash, so the IPC transport must be up first.
   */

  ameba_ipc_initialize();

  ret = ameba_flash_fs_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ameba_flash_fs_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTL8720F_WIFI
  /* Bring up the KM4 IPC transport and start the WHC host WiFi stack. */

  ret = rtl8720f_wifi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rtl8720f_wifi_initialize failed: %d\n", ret);
    }
#endif

  /* Install the inter-core HW IPC-semaphore RTOS hooks LAST -- after all the
   * flash / WHC bring-up above, and just before this (board_late_initialize)
   * path returns and nx_start() hands off to the init task.
   *
   * The shared-flash / WHC paths take the hardware IPC semaphore to
   * serialize against the NP; the ROM semaphore code then calls an RTOS
   * critical-section pair plus a delay callback while a contended waiter
   * waits.  That delay callback rtos_time_delay_ms() blocks, so it is only
   * safe from a real task.  Registering it here means the bring-up flash
   * access above (which runs in this pre-task context) uses the ROM default
   * (busy-spin, no callback), while every runtime IPC-semaphore user that
   * follows runs in a task and yields properly.
   */

  IPC_patch_function(rtos_critical_enter, rtos_critical_exit,
                     AMEBA_RTOS_CRITICAL_SEMA);
  IPC_SEMDelayStub(rtos_time_delay_ms);

  return ret;
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL
int board_app_initialize(uintptr_t arg)
{
#ifndef CONFIG_BOARD_LATE_INITIALIZE
  /* Perform board initialization */

  return rtl8720f_bringup();
#else
  return OK;
#endif
}
#endif /* CONFIG_BOARDCTL */

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  rtl8720f_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
