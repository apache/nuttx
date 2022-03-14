/****************************************************************************
 * boards/arm/stm32/stm32f429i-disco/src/stm32_spi.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32f429i-disco.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3) ||\
    defined(CONFIG_STM32_SPI4) || defined(CONFIG_STM32_SPI5)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI5
FAR struct spi_dev_s *g_spidev5 = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f429i-disco
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32_SPI5
  stm32_configgpio(GPIO_CS_MEMS);    /* MEMS chip select */
  stm32_configgpio(GPIO_CS_LCD);     /* LCD chip select */
  stm32_configgpio(GPIO_LCD_DC);     /* LCD Data/Command select */
  stm32_configgpio(GPIO_LCD_ENABLE); /* LCD enable select */
#endif
#if defined(CONFIG_STM32_SPI4) && defined(CONFIG_MTD_SST25XX)
  stm32_configgpio(GPIO_CS_SST25);   /* SST25 FLASH chip select */
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3/4/5select and stm32_spi1/2/3/4/5status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev,
                      uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev,
                      uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI4
void stm32_spi4select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
#if defined(CONFIG_MTD_SST25XX)
  if (devid == SPIDEV_FLASH(0))
    {
      stm32_gpiowrite(GPIO_CS_SST25, !selected);
    }
#endif
}

uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32_SPI5
void stm32_spi5select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_STM32F429I_DISCO_ILI9341)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_CS_LCD, !selected);
    }
  else
#endif

    {
      stm32_gpiowrite(GPIO_CS_MEMS, !selected);
    }
}

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI4
int stm32_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI5
int stm32_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_STM32F429I_DISCO_ILI9341)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32_gpiowrite(GPIO_LCD_DC, !cmd);

      return OK;
    }
#endif

  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */

/****************************************************************************
 * Name: stm32_spi5initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *   As long as the method stm32_spibus_initialize recognized the initialized
 *   state of the spi device by the spi enable flag of the cr1 register, it
 *   isn't safe to disable the spi device outside of the nuttx spi interface
 *   structure. But this has to be done as long as the nuttx spi interface
 *   doesn't support bidirectional data transfer for multiple devices share
 *   one spi bus. This wrapper does nothing else than store the initialized
 *   state of the spi device after the first initializing and should be used
 *   by each driver who shares the spi5 bus.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI5
FAR struct spi_dev_s *stm32_spi5initialize(void)
{
  if (!g_spidev5)
    {
      g_spidev5 = stm32_spibus_initialize(5);
    }

  return g_spidev5;
}
#endif
#endif /* CONFIG_STM32_SPI1 || ... CONFIG_STM32_SPI5 */
