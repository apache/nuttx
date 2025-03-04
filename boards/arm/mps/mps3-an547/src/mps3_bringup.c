/****************************************************************************
 * boards/arm/mps/mps3-an547/src/mps3_bringup.c
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
#include <sys/mount.h>
#include <syslog.h>

#include <nuttx/lib/modlib.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/ramdisk.h>

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mps3_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

static int mps3_bringup(void)
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

#ifdef CONFIG_FS_ROMFS
  ret = romdisk_register(1, 0x60000000, 4096, 512);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      ret = nx_mount("/dev/ram1", "/pic", "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount romfs at /mnt: %d\n", ret);
        }
    }

#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE

int board_boot_image(const char *path, uint32_t hdr_size)
{
  struct mod_loadinfo_s loadinfo;
  struct module_s mod;
  uintptr_t bss;
  uintptr_t got;
  uintptr_t msp;
  int ret;

  /* Initialize the ELF library to load the program binary. */

  syslog(LOG_INFO, "modlib_init...\n");

  ret = modlib_initialize(path, &loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to modlib_init: %d\n", ret);
      return ret;
    }

  /* Load the program binary */

  syslog(LOG_INFO, "modlib_load...\n");

  ret = modlib_load(&loadinfo);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to modlib_load: %d\n", ret);
      goto errout_with_init;
    }

  syslog(LOG_INFO, "modlib_bind...\n");

  memset(&mod, 0, sizeof(struct module_s));
  ret = modlib_bind(&mod, &loadinfo, NULL, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to modlib_bind: %d\n", ret);
      goto errout_with_load;
    }

  bss = modlib_findsection(&loadinfo, ".bss");
  got = loadinfo.shdr[loadinfo.gotindex].sh_addr;
  msp = loadinfo.shdr[bss].sh_addr + loadinfo.shdr[bss].sh_size +
        CONFIG_IDLETHREAD_STACKSIZE;

  syslog(LOG_INFO, "add-symbol-file ap.elf -s .text 0x%x -s .data"
         " 0x%x\n", loadinfo.textalloc, loadinfo.datastart);
  up_irq_disable();

  /* Disable systick */

  putreg32(0, NVIC_SYSTICK_CTRL);
  putreg32(NVIC_SYSTICK_RELOAD_MASK, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Set got address to r9 */

  __asm__ __volatile__("mov r9, %0"::"r"(got));

  /* set msp to the top of idle stack */

  __asm__ __volatile__("msr msp, %0" : : "r" (msp));

  ((void (*)(void))loadinfo.ehdr.e_entry + loadinfo.textalloc)();

errout_with_load:
  modlib_unload(&loadinfo);
errout_with_init:
  modlib_uninitialize(&loadinfo);
  return ret;
}
#endif

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  mps3_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */

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
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  UNUSED(arg);
#ifndef CONFIG_BOARD_LATE_INITIALIZE

  /* Perform board initialization */

  return mps3_bringup();
#else
  return OK;
#endif
}

