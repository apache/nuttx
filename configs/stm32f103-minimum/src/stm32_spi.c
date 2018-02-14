/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_spi.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the HY-MiniSTM32 board.
 *
 ************************************************************************************/

void stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_MTD_W25
  (void)stm32_configgpio(FLASH_SPI1_CS);      /* FLASH chip select */
#endif

#ifdef CONFIG_CAN_MCP2515
  (void)stm32_configgpio(GPIO_MCP2515_CS);    /* MCP2515 chip select */
#endif

#ifdef CONFIG_CL_MFRC522
  (void)stm32_configgpio(GPIO_CS_MFRC522);    /* MFRC522 chip select */
#endif

#if defined(CONFIG_SENSORS_MAX6675)
  (void)stm32_configgpio(GPIO_MAX6675_CS);    /* MAX6675 chip select */
#endif

#ifdef CONFIG_LCD_MAX7219
  (void)stm32_configgpio(STM32_LCD_CS);       /* MAX7219 chip select */
#endif

#ifdef CONFIG_LCD_ST7567
  (void)stm32_configgpio(STM32_LCD_CS);       /* ST7567 chip select */
#endif

#ifdef CONFIG_LCD_PCD8544
  (void)stm32_configgpio(STM32_LCD_CS);       /* ST7567 chip select */
#endif

#ifdef CONFIG_WL_NRF24L01
  stm32_configgpio(GPIO_NRF24L01_CS);         /* nRF24L01 chip select */
#endif

#ifdef CONFIG_MMCSD_SPI
  stm32_configgpio(GPIO_SDCARD_CS);           /* SD/MMC Card chip select */
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2select and stm32_spi1/2status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
#if defined(CONFIG_CAN_MCP2515)
  if (devid == SPIDEV_CANBUS(0))
    {
      stm32_gpiowrite(GPIO_MCP2515_CS, !selected);
    }
#endif

#if defined(CONFIG_CL_MFRC522)
  if (devid == SPIDEV_CONTACTLESS(0))
    {
      stm32_gpiowrite(GPIO_CS_MFRC522, !selected);
    }
#endif

#if defined(CONFIG_SENSORS_MAX6675)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX6675_CS, !selected);
    }
#endif

#ifdef CONFIG_LCD_MAX7219
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(STM32_LCD_CS, !selected);
    }
#endif

#ifdef CONFIG_LCD_PCD8544
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(STM32_LCD_CS, !selected);
    }
#endif

#ifdef CONFIG_LCD_ST7567
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(STM32_LCD_CS, !selected);
    }
#endif

#ifdef CONFIG_WL_NRF24L01
  if (devid == SPIDEV_WIRELESS(0))
    {
      stm32_gpiowrite(GPIO_NRF24L01_CS, !selected);
    }
#endif

#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32_gpiowrite(GPIO_SDCARD_CS, !selected);
    }
#endif

#ifdef CONFIG_MTD_W25
  stm32_gpiowrite(FLASH_SPI1_CS, !selected);
#endif
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef CONFIG_WL_NRF24L01
  if (devid == SPIDEV_WIRELESS(0))
    {
       status |= SPI_STATUS_PRESENT;
    }
#endif

#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
       status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
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
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool cmd)
{
#ifdef CONFIG_LCD_ST7567
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      (void)stm32_gpiowrite(STM32_LCD_RS, !cmd);

      return OK;
    }
#endif

#ifdef CONFIG_LCD_PCD8544
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      (void)stm32_gpiowrite(STM32_LCD_CD, !cmd);

      return OK;
    }
#endif


  return -ENODEV;
}
#endif
#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
