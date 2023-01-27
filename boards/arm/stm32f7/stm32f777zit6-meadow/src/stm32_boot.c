/****************************************************************************
 * boards/arm/stm32f7/stm32f777zit6-meadow/src/stm32_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

#include "stm32f777zit6-meadow.h"

#ifdef CONFIG_STM32F7_QUADSPI
#  include <nuttx/mtd/mtd.h>
#  include "stm32_qspi.h"

#  ifdef CONFIG_FS_FAT
#    include <sys/mount.h>
#    include <nuttx/fs/fat.h>
#  endif

/* MEADOW FIXME: header clash? */

extern FAR struct qspi_dev_s *stm32f7_qspi_initialize(int intf);
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point. This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void up_netinitialize(void)
{
}

void stm32_boardinitialize(void)
{
#if defined(CONFIG_STM32F7_SPI1) || defined(CONFIG_STM32F7_SPI2) || \
    defined(CONFIG_STM32F7_SPI3) || defined(CONFIG_STM32F7_SPI4) || \
    defined(CONFIG_STM32F7_SPI5)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* This configuration has been used for evaluating the NuttX sporadic
   * scheduler.
   * The following call initializes the sporadic scheduler monitor.
   */

  arch_sporadic_initialize();
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_STM32F7_FMC
  stm32_sdram_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().
 *   board_late_initialize() will be called immediately after up_initialize()
 *   is called and just before the initial application is started.
 *   This additional initialization phase may be used, for example, to
 *   initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#ifdef CONFIG_STM32F7_QUADSPI
  FAR struct qspi_dev_s *qspi;
  FAR struct mtd_dev_s *mtd;
#endif

#ifdef CONFIG_STM32F7_QUADSPI

  struct qspi_meminfo_s meminfo;

  int ret;

  qspi = stm32f7_qspi_initialize(0);
  if (!qspi)
    {
      syslog(LOG_ERR, "ERROR: sam_qspi_initialize muiled\n");
      return;
    }

  mtd = s25fl5_initialize(qspi, true);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: s25fl5_initialize failed\n");
    }

  ret = ftl_initialize(0, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
    }

    meminfo.flags = QSPIMEM_READ | QSPIMEM_QUADIO;
    meminfo.addrlen = 3;
    meminfo.dummies = 6;
    meminfo.cmd = 0xeb; /* S25FL1_FAST_READ_QUADIO; */
    meminfo.addr = 0;
    meminfo.buflen = 0;
    meminfo.buffer = NULL;

    stm32f7_qspi_enter_memorymapped(qspi, &meminfo, 80000000);

    stm32_mpu_uheap((uintptr_t)0x90000000, 0x4000000);
    }

#endif
  stm32_bringup();

#endif
