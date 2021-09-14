/****************************************************************************
 * boards/arm/sam34/arduino-due/src/sam_touchscreen.c
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
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "hardware/sam3u_pio.h"

#include "arduino-due.h"

/* Include the bit-band skeleton logic */

#include <nuttx/spi/spi_bitbang.c>

/* In order to use the SD card on the ITEAD shield, you must enable the SPI
 * bit-bang driver as well as support for SPI-based ADS7843E/XPT2046
 * touchscreen controller.
 */

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_INPUT_ADS7843E)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAM34_GPIOC_IRQ
#  error "Touchscreen support requires CONFIG_SAM34_GPIOC_IRQ"
#endif

#ifndef CONFIG_ADS7843E_FREQUENCY
#  define CONFIG_ADS7843E_FREQUENCY 500000
#endif

#ifndef CONFIG_ADS7843E_SPIDEV
#  define CONFIG_ADS7843E_SPIDEV 0
#endif

#ifndef CONFIG_ADS7843E_DEVMINOR
#  define CONFIG_ADS7843E_DEVMINOR 0
#endif

/* Definitions for include/nuttx/spi/spi_bitbang.c. */

#define SPI_SETSCK  putreg32(1 << 24, SAM_PIOA_SODR)
#define SPI_CLRSCK  putreg32(1 << 24, SAM_PIOA_CODR)
#define SPI_SETMOSI putreg32(1 << 16, SAM_PIOA_SODR)
#define SPI_CLRMOSI putreg32(1 << 16, SAM_PIOA_CODR)
#define SPI_GETMISO ((getreg32(SAM_PIOC_PDSR) >> 22) & 1)

/* Only mode 0 */

#undef  SPI_BITBANG_DISABLEMODE0
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE2 1
#define SPI_BITBANG_DISABLEMODE3 1

/* Only 8-bit data width */

#undef SPI_BITBANG_VARWIDTH

/* Calibration value for timing loop */

#define SPI_BITBANG_LOOPSPERMSEC CONFIG_BOARD_LOOPSPERMSEC

/* SPI_PERBIT_NSEC is the minimum time to transfer one bit.  This determines
 * the maximum frequency and is also used to calculate delays to achieve
 * other SPI frequencies.
 */

#define SPI_PERBIT_NSEC      100

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Lower-half SPI */

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected);
static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd);
#endif

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 * attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 * enable  - Enable or disable the GPIO interrupt
 * clear   - Acknowledge/clear any pending GPIO interrupt
 * pendown - Return the state of the pen down GPIO input
 */

static int  tsc_attach(struct ads7843e_config_s *state, xcpt_t isr);
static void tsc_enable(struct ads7843e_config_s *state, bool enable);
static void tsc_clear(struct ads7843e_config_s *state);
static bool tsc_busy(struct ads7843e_config_s *state);
static bool tsc_pendown(struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the XPT2046
 * driver.  This structure provides information about the configuration
 * of the XPT2046 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct ads7843e_config_s g_tscinfo =
{
  .frequency = CONFIG_ADS7843E_FREQUENCY,

  .attach    = tsc_attach,
  .enable    = tsc_enable,
  .clear     = tsc_clear,
  .busy      = tsc_busy,
  .pendown   = tsc_pendown,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Select or de-selected the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *   selected - True:select false:de-select
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected)
{
  /* The touchscreen controller is always selected */
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Return status of the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *
 * Returned Value:
 *   An 8-bit, bit-encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   If there were was a CMD/DATA line, this function would manage it
 *
 * Input Parameters:
 *   priv  - An instance of the bit-bang driver structure
 *   devid - The device to use
 *   cmd   - True=MCD false=DATA
 *
 * Returned Value:
 *  OK
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd)
{
  return OK;
}
#endif

/****************************************************************************
 * IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XPT2046 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the XPT2046 interrupt handler to the GPIO interrupt
 *   enable  - Enable or disable the GPIO interrupt
 *   clear   - Acknowledge/clear any pending GPIO interrupt
 *   pendown - Return the state of the pen down GPIO input
 *
 ****************************************************************************/

static int tsc_attach(struct ads7843e_config_s *state, xcpt_t isr)
{
  /* Attach the XPT2046 interrupt */

  iinfo("Attaching %p to IRQ %d\n", isr, SAM_TSC_IRQ);
  return irq_attach(SAM_TSC_IRQ, isr, NULL);
}

static void tsc_enable(struct ads7843e_config_s *state, bool enable)
{
  /* Attach and enable, or detach and disable */

  iinfo("IRQ:%d enable:%d\n", SAM_TSC_IRQ, enable);
  if (enable)
    {
      sam_gpioirqenable(SAM_TSC_IRQ);
    }
  else
    {
      sam_gpioirqdisable(SAM_TSC_IRQ);
    }
}

static void tsc_clear(struct ads7843e_config_s *state)
{
  /* Does nothing */
}

static bool tsc_busy(struct ads7843e_config_s *state)
{
  return false; /* The BUSY signal is not connected */
}

static bool tsc_pendown(struct ads7843e_config_s *state)
{
  /* The /PENIRQ value is active low */

  bool pendown = !sam_gpioread(GPIO_TSC_IRQ);
  iinfo("pendown:%d\n", pendown);
  return pendown;
}

/****************************************************************************
 * Name: sam_tsc_spiinitialize
 *
 * Description:
 *   Initialize the SPI bit-bang driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL reference to the SPI driver on success
 *
 ****************************************************************************/

static struct spi_dev_s *sam_tsc_spiinitialize(void)
{
  /* Configure the SPI bit-bang pins */

  sam_configgpio(GPIO_TSC_SCK);
  sam_configgpio(GPIO_TSC_MISO);
  sam_configgpio(GPIO_TSC_MOSI);

  /* Create the SPI driver instance */

  return spi_create_bitbang(&g_spiops, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_tsc_setup(int minor)
{
  struct spi_dev_s *dev;
  int ret;

  iinfo("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Configure and enable the XPT2046 interrupt pin as an input */

  sam_configgpio(GPIO_TSC_IRQ);

  /* Configure the PIO interrupt */

  sam_gpioirq(SAM_TSC_IRQ);

  /* Get an instance of the SPI interface for the touchscreen chip select */

  dev = sam_tsc_spiinitialize();
  if (!dev)
    {
      ierr("ERROR: Failed to initialize bit bang SPI\n");
      return -ENODEV;
    }

  /* Initialize and register the SPI touschscreen device */

  ret = ads7843e_register(dev, &g_tscinfo, CONFIG_ADS7843E_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device\n");

      /* up_spiuninitialize(dev); */

      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_ARDUINO_ITHEAD_TFT && CONFIG_SPI_BITBANG && CONFIG_INPUT_ADS7843E */
