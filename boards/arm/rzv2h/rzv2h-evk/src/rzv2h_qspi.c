/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_qspi.c
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

#include <stdint.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <arch/barriers.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/syslog/syslog.h>

#include "chip.h"
#include "rzv2h_gpio.h"
#include "rzv2h_qspi.h"
#include "rzv2h-evk.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_MTD_PARTITION) && \
    !defined(CONFIG_MTD_CONFIG) && !defined(CONFIG_MTD_CONFIG_NVS)
#  error "CONFIG_MTD_PARTITION on this board requires CONFIG_MTD_CONFIG " \
         "or CONFIG_MTD_CONFIG_NVS"
#endif

#ifdef CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_4S4D4D
#  error "4S-4D-4D not supported: XSPI0_DS is not connected to the " \
         "flash chip on this board"
#endif

/* Micron N25Q/MT25Q command set, as used by the n25qxxx MTD driver.  These
 * mirror n25qxxx.c's own (private) command macros; they are standard
 * across the Micron N25Q/MT25Q family fitted on this board.
 */

#define N25QXXX_CMD_FAST_READ_QUADIO   0xeb
#define N25QXXX_CMD_READ               0x03
#define N25QXXX_CMD_FAST_READ_DUALIO   0xbb
#define N25QXXX_CMD_FAST_READ          0x0b
#define N25QXXX_CMD_FAST_READ_DTR      0x0d
#define N25QXXX_CMD_ENTER_QPI          0x35
#define N25QXXX_CMD_RESET_QPI          0xf5
#define N25QXXX_CMD_PAGE_PROGRAM       0x02
#define N25QXXX_CMD_WRITE_ENABLE       0x06
#define N25QXXX_CMD_READ_STATUS        0x05

/* Bit position (not mask) within the status register read via
 * N25QXXX_CMD_READ_STATUS -- bit 0 flags write/erase in progress, bit 1
 * flags the write-enable latch.
 */

#define N25QXXX_STATUS_WIP_BIT         0
#define N25QXXX_STATUS_WEL_BIT         1

#define N25QXXX_PAGE_SIZE_BYTES        256
#define N25QXXX_MEMORY_MB              64

/* The datasheet's command table lists "factory default" dummy-clock
 * counts per command (10 for FAST READ 0Bh's Quad SPI column, 8 for
 * DTR FAST READ 0Dh's) -- but per that table's Note 5, those are only
 * shipped defaults: the flash's actual dummy count is whatever is
 * programmed into its volatile/nonvolatile configuration register,
 * uniformly across FAST READ variants.  n25qxxx_initialize() programs the
 * VCR to CONFIG_N25QXXX_DUMMIES (default 6) via n25qxxx_write_volcfg(), so
 * 6 -- not the per-command table default -- is what the flash actually
 * expects here, confirmed on hardware for 0Bh/4S-4S-4S.
 */

#define N25QXXX_QPI_READ_DUMMIES       6

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The n25qxxx MTD driver always issues Quad I/O Fast Read (0xEB, 1S-4S-4S).
 * Depending on the selected read protocol, the xSPI driver re-encodes it
 * onto the wire; 1S-4S-4S needs no translation.  Program, erase, and
 * status/config-register commands are unaffected and always use 1S-1S-1S.
 */

static const struct rzv2h_qspi_flash_s g_qspi_flash =
{
#ifdef CONFIG_RZV2H_EVK_QSPI_CHIP_SELECT_1
  .chip_select      = 1,
#else
  .chip_select      = 0,
#endif
  .memory_mb        = N25QXXX_MEMORY_MB,
  .page_size        = N25QXXX_PAGE_SIZE_BYTES,
  .page_program_cmd = N25QXXX_CMD_PAGE_PROGRAM,
  .write_enable_cmd = N25QXXX_CMD_WRITE_ENABLE,
  .status_cmd       = N25QXXX_CMD_READ_STATUS,
  .read_cmd         = N25QXXX_CMD_READ,
  .write_status_bit = N25QXXX_STATUS_WIP_BIT,
  .write_enable_bit = N25QXXX_STATUS_WEL_BIT,

#if defined(CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_1S1S1S)
  .read_xlate_from  = N25QXXX_CMD_FAST_READ_QUADIO,
  .read_xlate_to    = N25QXXX_CMD_READ,
  .read_protocol    = RZV2H_QSPI_PROTOCOL_1S1S1S,
  .read_dummies     = 0,
#elif defined(CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_1S2S2S)
  .read_xlate_from  = N25QXXX_CMD_FAST_READ_QUADIO,
  .read_xlate_to    = N25QXXX_CMD_FAST_READ_DUALIO,
  .read_protocol    = RZV2H_QSPI_PROTOCOL_1S2S2S,
  .read_dummies     = -1,
#elif defined(CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_4S4S4S)
  .read_xlate_from  = N25QXXX_CMD_FAST_READ_QUADIO,
  .read_xlate_to    = N25QXXX_CMD_FAST_READ,
  .read_protocol    = RZV2H_QSPI_PROTOCOL_4S4S4S,
  .read_dummies     = N25QXXX_QPI_READ_DUMMIES,
  .qpi_enter_cmd    = N25QXXX_CMD_ENTER_QPI,
  .qpi_exit_cmd     = N25QXXX_CMD_RESET_QPI,
#elif defined(CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_4S4D4D)
  .read_xlate_from  = N25QXXX_CMD_FAST_READ_QUADIO,
  .read_xlate_to    = N25QXXX_CMD_FAST_READ_DTR,
  .read_protocol    = RZV2H_QSPI_PROTOCOL_4S4D4D,
  .read_dummies     = N25QXXX_QPI_READ_DUMMIES,
  .qpi_enter_cmd    = N25QXXX_CMD_ENTER_QPI,
  .qpi_exit_cmd     = N25QXXX_CMD_RESET_QPI,
#else /* CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_1S4S4S */
  .read_protocol    = RZV2H_QSPI_PROTOCOL_1S4S4S,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_qspi_partition
 *
 * Description:
 *   Split the raw QSPI MTD device into three regions so that firmware,
 *   parameters/configuration, and mission-data/flight-logs cannot
 *   corrupt one another on the same chip:
 *
 *     - a firmware-image partition (raw MTD, no filesystem);
 *     - a parameter/configuration partition, exposed as /dev/config via
 *       mtdconfig_register();
 *     - the remaining flash, formatted with LittleFS and mounted at
 *       /mnt/qspi for mission data and flight logs.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_PARTITION
static int rzv2h_qspi_partition(struct mtd_dev_s *mtd,
                                       struct mtd_dev_s **data_mtd)
{
  struct mtd_geometry_s geo;
  struct mtd_dev_s *mtd_fw;
  struct mtd_dev_s *mtd_param;
  off_t fw_blocks;
  off_t param_blocks;
  int ret;

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MTDIOC_GEOMETRY failed: %d\n", ret);
      return ret;
    }

  fw_blocks    = (CONFIG_RZV2H_EVK_QSPI_FW_PART_SIZE * 1024) /
                 geo.erasesize;
  param_blocks = (CONFIG_RZV2H_EVK_QSPI_PARAM_PART_SIZE * 1024) /
                 geo.erasesize;

  if (fw_blocks + param_blocks >= geo.neraseblocks)
    {
      syslog(LOG_ERR,
             "ERROR: QSPI firmware/param partitions (%lu+%lu blocks) do "
             "not fit in %lu erase blocks\n",
             (unsigned long)fw_blocks, (unsigned long)param_blocks,
             (unsigned long)geo.neraseblocks);
      return -EINVAL;
    }

  mtd_fw = mtd_partition(mtd, 0, fw_blocks);
  if (mtd_fw == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to create QSPI firmware partition\n");
      return -EIO;
    }

  ret = register_mtddriver("/dev/mtdfw0", mtd_fw, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: register_mtddriver(/dev/mtdfw0) failed: %d\n", ret);
      return ret;
    }

  mtd_param = mtd_partition(mtd, fw_blocks, param_blocks);
  if (mtd_param == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to create QSPI parameter partition\n");
      return -EIO;
    }

  ret = mtdconfig_register(mtd_param);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtdconfig_register() failed: %d\n", ret);
      return ret;
    }

  *data_mtd = mtd_partition(mtd, fw_blocks + param_blocks,
                             geo.neraseblocks - fw_blocks - param_blocks);
  if (*data_mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: failed to create QSPI data partition\n");
      return -EIO;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_qspi_setup
 *
 * Description:
 *   Configure the xSPI0 pins (CKP/CS0N or CS1N/RESET0N/IO0-3, plus DS
 *   when a DDR protocol is selected) for QSPI
 *   operation.  Must run before rzv2h_qspibus_initialize().
 *
 *   R_BSP_XSPIOutputEnableCfg() is required besides the per-pin config:
 *   PFC_OEN.OEN2-5 is a separate group-level output-enable gate for the
 *   xSPI0 pads, defaults disabled, and R_XSPI_QSPI_Open() never touches
 *   it. Without it the pins never drive, reading back as a fixed
 *   all-zero response from the flash.
 *
 ****************************************************************************/

#ifdef CONFIG_RZV2H_QSPI
void rzv2h_qspi_setup(void)
{
  R_BSP_XSPIOutputEnableCfg();
  UP_DSB();

  rzv2h_configgpio(BOARD_QSPI0_CKP_GPIO);
  UP_DSB();
#ifdef CONFIG_RZV2H_EVK_QSPI_CHIP_SELECT_1
  rzv2h_configgpio(BOARD_QSPI0_CS1N_GPIO);
#else
  rzv2h_configgpio(BOARD_QSPI0_CS0N_GPIO);
#endif
  UP_DSB();
  rzv2h_configgpio(BOARD_QSPI0_RESET0N_GPIO);
  UP_DSB();
  rzv2h_configgpio(BOARD_QSPI0_IO0_GPIO);
  UP_DSB();
  rzv2h_configgpio(BOARD_QSPI0_IO1_GPIO);
  UP_DSB();
  rzv2h_configgpio(BOARD_QSPI0_IO2_GPIO);
  UP_DSB();
  rzv2h_configgpio(BOARD_QSPI0_IO3_GPIO);
  UP_DSB();
#ifdef CONFIG_RZV2H_EVK_QSPI_N25QXXX_PROTOCOL_4S4D4D
  rzv2h_configgpio(BOARD_QSPI0_DS_GPIO);
  UP_DSB();
#endif
}
#endif

/****************************************************************************
 * Name: rzv2h_qspi_initialize
 *
 * Description:
 *   Initialize the xSPI bus in QSPI mode, register the attached SPI NOR
 *   MTD device selected via Kconfig, and mount LittleFS if configured.
 *
 *   Fitted part: Micron MT25QU512ABB8E12-0SIT 512Mbit (64MB), supported
 *   by the n25qxxx MTD driver (CONFIG_MTD_N25QXXX), auto-selected via
 *   CONFIG_RZV2H_EVK_QSPI_MT25QU512ABB.  The other CONFIG_MTD_* branches
 *   only support derivative boards fitted with a different QSPI NOR part.
 *
 *   With CONFIG_MTD_PARTITION, the flash is split into firmware/
 *   parameter/data partitions (rzv2h_qspi_partition()); otherwise it's
 *   one flat LittleFS volume.
 *
 ****************************************************************************/

int rzv2h_qspi_initialize(void)
{
  struct qspi_dev_s *qspi;
  struct mtd_dev_s *mtd;
  struct mtd_dev_s *data_mtd;
  char blockdev[32];
  int ret;

  qspi = rzv2h_qspibus_initialize(0, &g_qspi_flash);
  if (qspi == NULL)
    {
      syslog(LOG_ERR, "ERROR: rzv2h_qspibus_initialize() failed\n");
      return -ENODEV;
    }

#if defined(CONFIG_MTD_N25QXXX)
  mtd = n25qxxx_initialize(qspi, true);
#else
  syslog(LOG_ERR,
         "ERROR: no CONFIG_MTD_* QSPI driver selected for xSPI\n");
  return -ENOSYS;
#endif

  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: QSPI initialize() failed\n");
      return -ENODEV;
    }

#ifdef CONFIG_MTD_PARTITION
  ret = rzv2h_qspi_partition(mtd, &data_mtd);
  if (ret < 0)
    {
      return ret;
    }
#else
  data_mtd = mtd;
#endif

  snprintf(blockdev, sizeof(blockdev), "/dev/mtdqspi0");

  ret = register_mtddriver(blockdev, data_mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver() failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0, NULL);
  if (ret < 0)
    {
      ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: nx_mount() failed: %d\n", ret);
          return ret;
        }
    }

#ifdef CONFIG_RZV2H_EVK_QSPI_SYSLOG_FILE
  if (syslog_file_channel(CONFIG_RZV2H_EVK_QSPI_SYSLOG_FILE_PATH) == NULL)
    {
      syslog(LOG_ERR, "ERROR: syslog_file_channel() failed\n");
    }
#endif
#endif

  return OK;
}
