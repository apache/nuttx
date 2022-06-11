/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_mfrc522.c
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
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/contactless/mfrc522.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_SPI) && defined(CONFIG_STM32_SPI1) && defined(CONFIG_CL_MFRC522)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFRC522_SPI_PORTNO 1   /* On SPI1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Initialize and register the MFRC522 RFID driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rfid0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_mfrc522initialize(const char *devpath)
{
  struct spi_dev_s *spi;
  int ret;

  spi = stm32_spibus_initialize(MFRC522_SPI_PORTNO);

  if (!spi)
    {
      return -ENODEV;
    }

  /* Then register the MFRC522 */

  ret = mfrc522_register(devpath, spi);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MFRC522\n");
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_MFRC522 */
