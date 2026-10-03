/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/src/pic32mz_spi.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/spi/spi.h>

#include "pic32mz_gpio.h"
#include "pic32mz_spi.h"

#include "ev49n51a.h"

#ifdef CONFIG_PIC32MZ_SPI1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_spidev_initialize
 *
 * Description:
 *   Configure the SPI chip select GPIOs.  SPI1 itself uses its dedicated
 *   pins (SCK1 RC6, SDI1 RC7, SDO1 RC8), selected by DEVCFG1.HSSPIEN.
 *
 ****************************************************************************/

void pic32mz_spidev_initialize(void)
{
  pic32mz_configgpio(GPIO_SST26_CS);
}

/****************************************************************************
 * Name: pic32mz_spi1select, pic32mz_spi1status and pic32mz_spi1cmddata
 *
 * Description:
 *   Board-specific select, status and cmddata methods of the SPI1
 *   interface (see include/nuttx/spi/spi.h).  The only device on SPI1 is
 *   the SST26VF032B serial flash.
 *
 ****************************************************************************/

void pic32mz_spi1select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %08" PRIx32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_FLASH(0))
    {
      /* CS is active low */

      pic32mz_gpiowrite(GPIO_SST26_CS, !selected);
    }
}

uint8_t pic32mz_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_PIC32MZ_SPI1 */
