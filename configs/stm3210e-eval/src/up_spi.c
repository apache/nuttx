/************************************************************************************
 * configs/stm3210e_eval/src/up_spi.c
 * arch/arm/src/board/up_spi.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "stm3210e-internal.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3210E-EVAL board.
 *
 ************************************************************************************/

void weak_function stm32_spiinitialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 was already provided in stm32_rcc.c */

#ifdef CONFIG_STM32_SPI1
  /* Select SPI1 pin mapping */

  uint32 mapr = getreg32(STM32_AFIO_MAPR);
#ifdef CONFIG_STM32_SPI1_REMAP
  mapr |= AFIO_MAPR_SPI1_REMAP;
#else
  mapr &= ~AFIO_MAPR_SPI1_REMAP;
#endif
  putreg32(mapr, STM32_AFIO_MAPR);

  /* Configure SPI1 alternate function pins */

  stm32_configgpio(GPIO_SPI1_SCK);
  stm32_configgpio(GPIO_SPI1_MISO);
  stm32_configgpio(GPIO_SPI1_MOSI);
 
  /* Configure the SPI-based microSD and FLASH CS GPIO */

  stm32_configgpio(GPIO_MMCSD_CS);
  stm32_configgpio(GPIO_FLASH_CS);
#endif

#ifdef CONFIG_STM32_SPI2
  /* Configure SPI1 alternate function pins */

  stm32_configgpio(GPIO_SPI2_SCK);
  stm32_configgpio(GPIO_SPI2_MISO);
  stm32_configgpio(GPIO_SPI2_MOSI);
 
#endif
}

/****************************************************************************
 * The external functions, stm32_spi1/2select and stm32_spi1/2status must be
 * provided by board-specific logic.  These are implementations of the
 * select and status methods SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi.h). All other methods (including up_spiinitialize())
 * are provided by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide stm32_spi1/2select() and stm32_spi1/2status() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. Add a call to up_spiinitialize() in your low level initialization
 *      logic
 *   3. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_MMCSD)
  {
    /* Set the GPIO low to select and high to de-select */

    stm32_gpiowrite(GPIO_MMCSD_CS, !selected);
  }
  else if (devid == SPIDEV_FLASH)
  {
    /* Set the GPIO low to select and high to de-select */

    stm32_gpiowrite(GPIO_FLASH_CS, !selected);
  }
}

ubyte stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, boolean selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

ubyte stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
