/****************************************************************************
 * boards/risc-v/qemu-rv/rv-virt/src/qemu_rv_appinit.c
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/virtio/virtio-mmio.h>

#include <sys/mount.h>

#include "hardware/qemu_rv_memorymap.h"
#include "qemu_rv_memorymap.h"
#include "qemu_rv_rptun.h"

#include "riscv_internal.h"
#include "romfs.h"

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEMU_VIRTIO_MMIO_BASE    0x10001000
#define QEMU_VIRTIO_MMIO_REGSIZE 0x1000
#ifdef CONFIG_ARCH_USE_S_MODE
#  define QEMU_VIRTIO_MMIO_IRQ   26
#else
#  define QEMU_VIRTIO_MMIO_IRQ   28
#endif
#define QEMU_VIRTIO_MMIO_NUM     8

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_virtio_register_mmio_devices
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
static void qemu_virtio_register_mmio_devices(void)
{
  uintptr_t virtio = (uintptr_t)QEMU_VIRTIO_MMIO_BASE;
  size_t size = QEMU_VIRTIO_MMIO_REGSIZE;
  int irq = QEMU_VIRTIO_MMIO_IRQ;
  int i;

  for (i = 0; i < QEMU_VIRTIO_MMIO_NUM; i++)
    {
      virtio_register_mmio_device((void *)(virtio + size * i), irq + i);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
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
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

#ifdef CONFIG_FS_PROCFS
  mount(NULL, "/proc", "procfs", 0, NULL);
#endif

#ifdef CONFIG_FS_TMPFS
  mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
#endif

#endif

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
#ifndef CONFIG_BOARD_EARLY_INITIALIZE
  qemu_virtio_register_mmio_devices();
#endif
#endif

#ifdef CONFIG_RPTUN
  qemu_rptun_init();
#endif

  return OK;
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  /* Perform board-specific initialization */

#if defined(CONFIG_BUILD_KERNEL) && !defined(CONFIG_RISCV_SEMIHOSTING_HOSTFS)
  /* Create ROM disk for mount in nx_start_application */

  if (NSECTORS(romfs_img_len) > 1)
    {
      int ret = OK;
      ret = romdisk_register(0, romfs_img, NSECTORS(romfs_img_len),
        SECTORSIZE);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register romfs: %d\n", -ret);
        }
    }
#endif /* CONFIG_BUILD_KERNEL && !CONFIG_RISCV_SEMIHOSTING_HOSTFS */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  int ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif
}

void board_early_initialize(void)
{
#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  qemu_virtio_register_mmio_devices();
#endif
}

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
#if defined(CONFIG_BUILD_KERNEL) && ! defined(CONFIG_NUTTSBI)
  riscv_sbi_system_reset(SBI_SRST_TYPE_SHUTDOWN, SBI_SRST_REASON_NONE);
#else
  *(volatile uint32_t *)QEMU_RV_RESET_BASE = QEMU_RV_RESET_DONE;
#endif

  UNUSED(status);
  return 0;
}
#endif
