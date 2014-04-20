/****************************************************************************
 * configs/nucleo-f401re/src/stm32_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start
 * required subsystems and perform board-specific initialisation.
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_FAT_DMAMEMORY
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif
#endif

/* PX4 LED colour codes */

#define LED_AMBER      1
#define LED_RED        1  /* Some boards have red rather than amber */
#define LED_BLUE       0
#define LED_SAFETY     2

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_FAT_DMAMEMORY
static GRAN_HANDLE dma_allocator;

/* The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */

static uint8_t g_dma_heap[8192] __attribute__((aligned(64)));
#endif

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct sdio_dev_s *sdio;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_FAT_DMAMEMORY
static void dma_alloc_init(void)
{
  dma_allocator =
    gran_initialize(g_dma_heap,
                    sizeof(g_dma_heap),
                    7,  /* 128B granule - must be > alignment (XXX bug?) */
                    6); /* 64B alignment */

  if (dma_allocator == NULL)
    {
      message("[boot] DMA allocator setup FAILED");
    }
}
#else
# define dma_alloc_init()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_FAT_DMAMEMORY
/* DMA-aware allocator stubs for the FAT filesystem. */

void *fat_dma_alloc(size_t size)
{
  return gran_alloc(dma_allocator, size);
}

void fat_dma_free(FAR void *memory, size_t size)
{
  gran_free(dma_allocator, memory, size);
}

#endif

void up_netinitialize(void)
{
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
#ifdef CONFIG_MMCSD
  int ret;
#endif

  /* Configure ADC pins */

  stm32_configgpio(GPIO_ADC1_IN2);   /* BATT_VOLTAGE_SENS */
  stm32_configgpio(GPIO_ADC1_IN3);   /* BATT_CURRENT_SENS */
  stm32_configgpio(GPIO_ADC1_IN4);   /* VDD_5V_SENS */
//stm32_configgpio(GPIO_ADC1_IN10);  /* used by VBUS valid */
//stm32_configgpio(GPIO_ADC1_IN11);  /* unused */
//stm32_configgpio(GPIO_ADC1_IN12);  /* used by MPU6000 CS */
  stm32_configgpio(GPIO_ADC1_IN13);  /* FMU_AUX_ADC_1 */
  stm32_configgpio(GPIO_ADC1_IN14);  /* FMU_AUX_ADC_2 */
  stm32_configgpio(GPIO_ADC1_IN15);  /* PRESSURE_SENS */

  /* Configure power supply control/sense pins */

  stm32_configgpio(GPIO_VDD_5V_PERIPH_EN);
  stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
  stm32_configgpio(GPIO_VDD_BRICK_VALID);
  stm32_configgpio(GPIO_VDD_SERVO_VALID);
  stm32_configgpio(GPIO_VDD_5V_HIPOWER_OC);
  stm32_configgpio(GPIO_VDD_5V_PERIPH_OC);

  /* Configure the DMA allocator */

  dma_alloc_init();

  /* Configure CPU load estimation */

#ifdef CONFIG_SCHED_INSTRUMENTATION
  cpuload_initialize_once();
#endif

  /* Initial LED state */

  led_off(LED_AMBER);

  /* Configure SPI-based devices */

  spi1 = up_spiinitialize(1);
  if (!spi1)
    {
      message("[boot] FAILED to initialize SPI port 1\n");
      board_led_on(LED_AMBER);
      return -ENODEV;
    }

  /* Default SPI1 to 1MHz and de-assert the known chip selects. */

  SPI_SETFREQUENCY(spi1, 10000000);
  SPI_SETBITS(spi1, 8);
  SPI_SETMODE(spi1, SPIDEV_MODE3);
  SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
  SPI_SELECT(spi1, PX4_SPIDEV_ACCEL_MAG, false);
  SPI_SELECT(spi1, PX4_SPIDEV_BARO, false);
  SPI_SELECT(spi1, PX4_SPIDEV_MPU, false);
  up_udelay(20);

  message("[boot] Initialized SPI port 1 (SENSORS)\n");

  /* Get the SPI port for the FRAM */

  spi2 = up_spiinitialize(2);
  if (!spi2)
    {
      message("[boot] FAILED to initialize SPI port 2\n");
      board_led_on(LED_AMBER);
      return -ENODEV;
    }

  /* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
   * and de-assert the known chip selects.
   */

  // XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated

  SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
  SPI_SETBITS(spi2, 8);
  SPI_SETMODE(spi2, SPIDEV_MODE3);
  SPI_SELECT(spi2, SPIDEV_FLASH, false);

  message("[boot] Initialized SPI port 2 (RAMTRON FRAM)\n");

#ifdef CONFIG_MMCSD
  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!sdio)
    {
      message("[boot] Failed to initialize SDIO slot %d\n",
              CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);
  if (ret != OK)
    {
      message("[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot. There is no
   * card detect GPIO.
   */

  sdio_mediachange(sdio, true);

  message("[boot] Initialized SDIO\n");
#endif

  return OK;
}
