/****************************************************************************
 * boards/arm/stm32h7/nucleo-h7s3l8/src/stm32_mx25uw25645g.c
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

#include <nuttx/mtd/mtd.h>

#include "stm32_xspi.h"
#include "nucleo-h7s3l8.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Must match OCTA_READ/DC settings in drivers/mtd/mx25uw25645g.c */

#define MX25UW_OCTA_READ         0xec13 /* Octal STR read command */
#define MX25UW_READ_DUMMY        6      /* Dummy cycles for 100 MHz XSPI */

/* DEVSIZE encodes 2^(n+1) bytes: 24 -> 32 MB */

#define MX25UW_DEVSIZE           24

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi_flash_initialize
 *
 * Description:
 *   Initialize XSPI2 and the board's MX25UW25645G flash, then enter
 *   memory-mapped octal STR mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_flash_initialize(void)
{
  struct stm32_xspi_config_s config;
  struct qspi_meminfo_s readinfo;
  struct mtd_dev_s *mtd;
  struct qspi_dev_s *qspi;

  config.memtype      = STM32_XSPI_MEMTYPE_MACRONIX;
  config.iomport      = STM32_XSPI_IOM_PORT2;
  config.cs_override  = STM32_XSPI_CS_NCS1;
  config.devsize      = MX25UW_DEVSIZE;
  config.csht         = 2;
  config.prescaler    = 3;
  config.fthres       = 4;
  config.wrapsize     = 0;
  config.maxtran      = 0;
  config.csbound      = 0;
  config.req2ack      = 1;
  config.refresh      = 0;
  config.clock_mode3  = false;
  config.free_running = false;
  config.sample_shift = false;

  readinfo.flags   = QSPIMEM_IOCTAL | QSPIMEM_OCTALIO;
  readinfo.addrlen = 4;
  readinfo.dummies = MX25UW_READ_DUMMY;
  readinfo.cmd     = MX25UW_OCTA_READ;
  readinfo.buflen  = 0;
  readinfo.addr    = 0;
  readinfo.buffer  = NULL;

  qspi = stm32_xspi_initialize(2, &config);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  mtd = mx25uw25645g_initialize(qspi);
  if (mtd == NULL)
    {
      return -ENODEV;
    }

  return stm32_xspi_enter_memorymapped(qspi, &readinfo, NULL);
}
