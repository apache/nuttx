/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_spi.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"

#include "stm32f4discovery.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
#ifdef CONFIG_ENC28J60
  stm32_configgpio(GPIO_ENC28J60_CS);
  stm32_configgpio(GPIO_ENC28J60_RESET);
  stm32_configgpio(GPIO_ENC28J60_INTR);
#endif

#ifdef CONFIG_STM32_SPI1
  stm32_configgpio(GPIO_CS_MEMS);    /* MEMS chip select */
#endif
#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_SENSORS_MAX31855)
  stm32_configgpio(GPIO_MAX31855_CS); /* MAX31855 chip select */
#endif
#if defined(CONFIG_LCD_MAX7219) || defined(CONFIG_LEDS_MAX7219)
  stm32_configgpio(GPIO_MAX7219_CS);  /* MAX7219 chip select */
#endif
#ifdef CONFIG_LPWAN_SX127X
  stm32_configgpio(GPIO_SX127X_CS);   /* SX127x chip select*/
#endif

#if defined(CONFIG_LCD_ST7567)
  stm32_configgpio(STM32_LCD_CS);     /* ST7567 chip select */
#endif
#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_SENSORS_MAX6675)
  stm32_configgpio(GPIO_MAX6675_CS); /* MAX6675 chip select */
#endif
#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  stm32_configgpio(GPIO_OLED_CS);    /* OLED chip select */
# if defined(CONFIG_LCD_UG2864AMBAG01)
  stm32_configgpio(GPIO_OLED_A0);    /* OLED Command/Data */
# endif
# if defined(CONFIG_LCD_UG2864HSWEG01) || defined(CONFIG_LCD_SSD1351)
  stm32_configgpio(GPIO_OLED_DC);    /* OLED Command/Data */
# endif
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_ENC28J60
  if (devid == SPIDEV_ETHERNET(0))
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_ENC28J60_CS, !selected);
    }
#endif

#ifdef CONFIG_LPWAN_SX127X
  if (devid == SPIDEV_LPWAN(0))
    {
      stm32_gpiowrite(GPIO_SX127X_CS, !selected);
    }
#endif

#ifdef CONFIG_LCD_ST7567
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(STM32_LCD_CS, !selected);
    }
#endif

#if defined(CONFIG_LCD_MAX7219) || defined(CONFIG_LEDS_MAX7219)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_MAX7219_CS, !selected);
    }
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_OLED_CS, !selected);
    }
  else
#endif
    {
      stm32_gpiowrite(GPIO_CS_MEMS, !selected);
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef CONFIG_LPWAN_SX127X
  if (devid == SPIDEV_LPWAN(0))
    {
      status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX31855_CS, !selected);
    }
#endif

#if defined(CONFIG_SENSORS_MAX6675)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX6675_CS, !selected);
    }
#endif

#if defined(CONFIG_MMCSD_SPI)
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32_gpiowrite(GPIO_MMCSD_NSS, !selected);
    }
#endif
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
#if defined(CONFIG_MMCSD_SPI)
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Note: SD_DET is pulled high when there's no SD card present. */

      ret = stm32_gpioread(GPIO_MMCSD_NCD) ? 0 : 1;
    }
#endif

  return ret;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_WL_GS2200M)
  if (devid == SPIDEV_WIRELESS(0))
    {
      stm32_gpiowrite(GPIO_GS2200M_CS, !selected);
    }
#endif
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
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
#ifdef CONFIG_LCD_ST7567
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32_gpiowrite(STM32_LCD_RS, !cmd);

      return OK;
    }
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* "This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       *
       *  A0 = "H": the inputs at D0 to D7 are treated as display data.
       *  A0 = "L": the inputs at D0 to D7 are transferred to the command
       *       registers."
       */

# if defined(CONFIG_LCD_UG2864AMBAG01)
      stm32_gpiowrite(GPIO_OLED_A0, !cmd);
# endif
# if defined(CONFIG_LCD_UG2864HSWEG01) || defined(CONFIG_LCD_SSD1351)
      stm32_gpiowrite(GPIO_OLED_DC, !cmd);
# endif
      return OK;
    }
#endif

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
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
