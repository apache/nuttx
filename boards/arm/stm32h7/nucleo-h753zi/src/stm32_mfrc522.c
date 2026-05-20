/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_mfrc522.c
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
#include <syslog.h>
#include <stdbool.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/contactless/mfrc522.h>

#include "nucleo-h753zi.h"
#include "stm32_spi.h"

#if defined(CONFIG_SPI) && defined(CONFIG_CL_MFRC522) && \
    defined(CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which SPI port to use based on Kconfig selection */

#ifdef CONFIG_NUCLEO_H753ZI_MFRC522_SPI1
#  define MFRC522_SPI_PORTNO 1
#  ifndef CONFIG_STM32H7_SPI1
#    error "MFRC522 configured for SPI1 but CONFIG_STM32H7_SPI1 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI1_ENABLE
#    error "MFRC522 on SPI1 needs CONFIG_NUCLEO_H753ZI_SPI1_ENABLE"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_SPI2)
#  define MFRC522_SPI_PORTNO 2
#  ifndef CONFIG_STM32H7_SPI2
#    error "MFRC522 configured for SPI2 but CONFIG_STM32H7_SPI2 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI2_ENABLE
#    error "MFRC522 on SPI2 needs CONFIG_NUCLEO_H753ZI_SPI2_ENABLE"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_SPI3)
#  define MFRC522_SPI_PORTNO 3
#  ifndef CONFIG_STM32H7_SPI3
#    error "MFRC522 configured for SPI3 but CONFIG_STM32H7_SPI3 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI3_ENABLE
#    error "MFRC522 on SPI3 needs CONFIG_NUCLEO_H753ZI_SPI3_ENABLE"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_SPI4)
#  define MFRC522_SPI_PORTNO 4
#  ifndef CONFIG_STM32H7_SPI4
#    error "MFRC522 configured for SPI4 but CONFIG_STM32H7_SPI4 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI4_ENABLE
#    error "MFRC522 on SPI4 needs CONFIG_NUCLEO_H753ZI_SPI4_ENABLE"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_SPI5)
#  define MFRC522_SPI_PORTNO 5
#  ifndef CONFIG_STM32H7_SPI5
#    error "MFRC522 configured for SPI5 but CONFIG_STM32H7_SPI5 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI5_ENABLE
#    error "MFRC522 on SPI5 needs CONFIG_NUCLEO_H753ZI_SPI5_ENABLE"
#  endif
#elif defined(CONFIG_NUCLEO_H753ZI_MFRC522_SPI6)
#  define MFRC522_SPI_PORTNO 6
#  ifndef CONFIG_STM32H7_SPI6
#    error "MFRC522 configured for SPI6 but CONFIG_STM32H7_SPI6 not enabled"
#  endif
#  ifndef CONFIG_NUCLEO_H753ZI_SPI6_ENABLE
#    error "MFRC522 on SPI6 needs CONFIG_NUCLEO_H753ZI_SPI6_ENABLE"
#  endif
#else
#  error "No SPI port selected for MFRC522"
#endif

/* Default device path if not specified */

#ifndef CONFIG_NUCLEO_H753ZI_MFRC522_DEVPATH
#  define CONFIG_NUCLEO_H753ZI_MFRC522_DEVPATH "/dev/rfid0"
#endif

/* Default CS pin configuration if not specified */

#ifndef CONFIG_NUCLEO_H753ZI_MFRC522_CS_PIN
#  define CONFIG_NUCLEO_H753ZI_MFRC522_CS_PIN "PF1"
#endif

/* Default active level (most MFRC522 modules are active low) */

#ifndef CONFIG_NUCLEO_H753ZI_MFRC522_CS_ACTIVE_LOW
#  define CONFIG_NUCLEO_H753ZI_MFRC522_CS_ACTIVE_LOW true
#endif

/* Device ID calculation - must match the scheme in stm32_spi.c
 * SPI1: IDs 0-7, SPI2: IDs 8-15, SPI3: IDs 16-23, etc.
 */

#ifndef CONFIG_NUCLEO_H753ZI_MFRC522_DEVID
#  if MFRC522_SPI_PORTNO == 1
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 0
#  elif MFRC522_SPI_PORTNO == 2
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 8
#  elif MFRC522_SPI_PORTNO == 3
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 16
#  elif MFRC522_SPI_PORTNO == 4
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 24
#  elif MFRC522_SPI_PORTNO == 5
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 32
#  elif MFRC522_SPI_PORTNO == 6
#    define CONFIG_NUCLEO_H753ZI_MFRC522_DEVID 40
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Initialize and register the MFRC522 RFID driver.
 *   This function uses the board-specific CS registration system.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rfid0".
 *             If NULL, the default path from Kconfig is used.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_mfrc522initialize(FAR const char *devpath)
{
  FAR struct spi_dev_s *spi;
  FAR const char *path;
  int ret;

  path = devpath ? devpath : MFRC522_DEVPATH;

  syslog(LOG_INFO,
         "Initializing MFRC522 on SPI%d, device ID: %d, path: %s\n",
         MFRC522_SPI_PORTNO, CONFIG_NUCLEO_H753ZI_MFRC522_DEVID, path);

  /* Register the CS device with the SPI system */

  ret = stm32_spi_register_cs_device(
          MFRC522_SPI_PORTNO,
          CONFIG_NUCLEO_H753ZI_MFRC522_DEVID,
          CONFIG_NUCLEO_H753ZI_MFRC522_CS_PIN,
          CONFIG_NUCLEO_H753ZI_MFRC522_CS_ACTIVE_LOW);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register CS device for MFRC522: %d\n", ret);
      return ret;
    }

  /* Initialize the SPI bus */

  spi = stm32_spibus_initialize(MFRC522_SPI_PORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d\n",
             MFRC522_SPI_PORTNO);

      /* Cleanup: unregister the CS device */

      stm32_spi_unregister_cs_device(MFRC522_SPI_PORTNO,
                                     CONFIG_NUCLEO_H753ZI_MFRC522_DEVID);
      return -ENODEV;
    }

  /* Register the MFRC522 driver */

  ret = mfrc522_register(path, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MFRC522 driver: %d\n",
             ret);

      /* Cleanup: unregister the CS device */

      stm32_spi_unregister_cs_device(MFRC522_SPI_PORTNO,
                                     CONFIG_NUCLEO_H753ZI_MFRC522_DEVID);
      return ret;
    }

  syslog(LOG_INFO, "MFRC522 driver registered successfully at %s\n", path);

  /* Note: The MFRC522 driver will use device ID 0 by default for SPI
   * transactions. Our CS registration system will handle CS control
   * based on the registered device ID. If the driver uses a different
   * device ID, you may need to modify the driver or use a different
   * registration approach.
   */

  return OK;
}

/****************************************************************************
 * Name: stm32_mfrc522_get_devid
 *
 * Description:
 *   Get the device ID used by the MFRC522 for CS pin control.
 *   This is useful for debugging or advanced applications.
 *
 * Returned Value:
 *   Device ID used by MFRC522
 *
 ****************************************************************************/

uint32_t stm32_mfrc522_get_devid(void)
{
  return CONFIG_NUCLEO_H753ZI_MFRC522_DEVID;
}

/****************************************************************************
 * Name: stm32_mfrc522_cleanup
 *
 * Description:
 *   Cleanup MFRC522 resources. This function can be called during
 *   shutdown or error recovery.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_mfrc522_cleanup(void)
{
  int ret;

  ret = stm32_spi_unregister_cs_device(MFRC522_SPI_PORTNO,
                                       MFRC522_DEVICE_ID);
  if (ret < 0)
    {
      syslog(LOG_WARNING,
             "WARNING: Failed to unregister MFRC522 CS device: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_CL_MFRC522 &&
        * CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE */
