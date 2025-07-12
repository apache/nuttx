/****************************************************************************
 * boards/z80/ez80/z20x/src/ez80_w25.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>

#include "ez80_spi.h"
#include "z20x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_w25_initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int ez80_w25_initialize(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int ret;

  /* Get the SPI port */

  spi = ez80_spibus_initialize(1);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port %d\n", 0);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port %d to the W25 FLASH driver\n", 0);
      return -ENODEV;
    }

  /* Register the MTD driver */

  ret = register_mtddriver(W25_DEV, mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: register_mtddriver %s failed: %d\n", W25_DEV, ret);
      return ret;
    }

  return OK;
}
