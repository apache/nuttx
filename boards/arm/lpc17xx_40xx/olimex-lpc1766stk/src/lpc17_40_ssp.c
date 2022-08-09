/****************************************************************************
 * boards/arm/lpc17xx_40xx/olimex-lpc1766stk/src/lpc17_40_ssp.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#ifdef CONFIG_SPI_CALLBACK
#include <nuttx/irq.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "lpc1766stk.h"

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#undef HAVE_SPI_CALLBACK
#ifdef CONFIG_SPI_CALLBACK
#  ifndef CONFIG_LPC17_40_GPIOIRQ
#    warning "CONFIG_LPC17_40_GPIOIRQ is required to support CONFIG_SPI_CALLBACK"
#  else
#    define HAVE_SPI_CALLBACK 1
#  endif
#endif

/* Debug ********************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define ssp_dumpssp0gpio(m) lpc17_40_dumpgpio(LPC1766STK_LCD_CS, m)
#  define ssp_dumpssp1gpio(m) lpc17_40_dumpgpio(LPC1766STK_MMC_CS, m)
#else
#  define ssp_dumpssp0gpio(m)
#  define ssp_dumpssp1gpio(m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes on media change callback */

#ifdef HAVE_SPI_CALLBACK
struct lpc17_40_mediachange_s
{
  spi_mediachange_t callback; /* The media change callback */
  void              *arg;     /* Callback argument */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Registered media change callback */

#ifdef HAVE_SPI_CALLBACK
#ifdef CONFIG_LPC17_40_SSP0
static struct lpc17_40_mediachange_s g_ssp0callback;
#endif
#ifdef CONFIG_LPC17_40_SSP1
static struct lpc17_40_mediachange_s g_ssp1callback;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssp_cdirqsetup
 *
 * Description:
 *   Setup to receive a card detection interrupt
 *
 ****************************************************************************/

#if 0 /* #ifdef HAVE_SPI_CALLBACK */
static void ssp_cdirqsetup(int irq, xcpt_t irqhandler)
{
  irqstate_t flags;

  /* Disable interrupts until we are done */

  flags = enter_critical_section();

  /* Configure the interrupt.  Either attach and enable the new
   * interrupt or disable and detach the old interrupt handler.
   */

  if (irqhandler)
    {
      /* Attach then enable the new interrupt handler */

      irq_attach(irq, irqhandler, NULL);
      up_enable_irq(irq);
    }
  else
    {
      /* Disable then detach the old interrupt handler */

      up_disable_irq(irq);
      irq_detach(irq);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: ssp0/1_cdinterrupt
 *
 * Description:
 *   Handle card detection interrupt
 *
 ****************************************************************************/

#if 0 /* ifdef HAVE_SPI_CALLBACK */
#ifdef CONFIG_LPC17_40_SSP0
static int ssp0_cdinterrupt(int irq, void *context)
{
  /* Invoke the media change callback */

  if (g_ssp0callback.callback)
    {
      g_ssp0callback.callback(g_ssp0callback.arg);
    }

  return OK;
}
#endif

#ifdef CONFIG_LPC17_40_SSP1
static int ssp1_cdinterrupt(int irq, void *context)
{
  /* Invoke the media change callback */

  if (g_ssp1callback.callback)
    {
      g_ssp1callback.callback(g_ssp1callback.arg);
    }

  return OK;
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc1766stk_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC1766-STK.
 *
 ****************************************************************************/

void weak_function lpc1766stk_sspdev_initialize(void)
{
  /* Configure the SSP0 chip select GPIOs.
   * Only the Nokia LCD is connected to SSP0
   */

#ifdef CONFIG_LPC17_40_SSP0
  ssp_dumpssp0gpio("BEFORE SSP0 Initialization");
  lpc17_40_configgpio(LPC1766STK_LCD_CS);
  ssp_dumpssp0gpio("AFTER SSP0 Initialization");
#endif

  /* Configure SSP1 chip select GPIOs.
   * Only the SD/MMC card slot is connected to SSP1
   */

#ifdef CONFIG_LPC17_40_SSP1
  ssp_dumpssp0gpio("BEFORE SSP1 Initialization");
  lpc17_40_configgpio(LPC1766STK_MMC_CS);

  /* Also configure the SD/MMC power GPIO (but leave power off).
   * This really has nothing to do with SSP, but does belong with other
   * SD/MMC GPIO configuration settings.
   */

  lpc17_40_configgpio(LPC1766STK_MMC_PWR);
  ssp_dumpssp0gpio("AFTER SSP1 Initialization");
#endif

#ifdef HAVE_SPI_CALLBACK
  /* If there were any CD detect pins for the LPC1766-STK, this is where
   * they would be configured.
   */
#endif
}

/****************************************************************************
 * Name:  lpc17_40_ssp0/ssp1select and lpc17_40_ssp0/ssp1status
 *
 * Description:
 *   The external functions, lpc17_40_ssp0/ssp1select and
 *   lpc17_40_ssp0/ssp1status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including lpc17_40_sspbus_initialize())
 *   are provided by common LPC17xx/LPC40xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide lpc17_40_ssp0/ssp1select() and lpc17_40_ssp0/ssp1status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to lpc17_40_sspbus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_sspbus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP0
void  lpc17_40_ssp0select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Assert/de-assert the CS pin to the card */

      ssp_dumpssp0gpio("lpc17_40_ssp0select() Entry");
      lpc17_40_gpiowrite(LPC1766STK_LCD_CS, !selected);
      ssp_dumpssp0gpio("lpc17_40_ssp0select() Exit");
    }
}

uint8_t lpc17_40_ssp0status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}
#endif

#ifdef CONFIG_LPC17_40_SSP1
void  lpc17_40_ssp1select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Assert/de-assert the CS pin to the card */

      ssp_dumpssp1gpio("lpc17_40_ssp1select() Entry");
      lpc17_40_gpiowrite(LPC1766STK_MMC_CS, !selected);
      ssp_dumpssp1gpio("lpc17_40_ssp1select() Exit");
    }
}

uint8_t lpc17_40_ssp1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}
#endif

/****************************************************************************
 * Name: lpc17_40_ssp0/1register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s) must
 *   must be implemented.  These functions implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC17_40_SSP0
  /* If there were any CD detect pins on the LPC1766-STK, this is how the
   * would be configured.
   */

int lpc17_40_ssp0register(struct spi_dev_s *dev,
                          spi_mediachange_t callback,
                          void *arg)
{
  /* Save the callback information */

#if 0
  g_ssp0callback.callback = callback;
  g_ssp0callback.arg      = arg;

  /* Setup the interrupt */

  spi_cdirqsetup(LPC1766STK_SPICD_IRQ, ssp0_cdinterrupt);
#endif
  return OK;
}
#endif

#ifdef CONFIG_LPC17_40_SSP1
int lpc17_40_ssp1register(struct spi_dev_s *dev,
                          spi_mediachange_t callback,
                          void *arg)
{
  /* Save the callback information */

#if 0
  g_ssp1callback.callback = callback;
  g_ssp1callback.arg      = arg;

  /* Setup the interrupt */

  spi_cdirqsetup(LPC1766STK_SPICD_IRQ, ssp1_cdinterrupt);
#endif
  return OK;
}
#endif
#endif
#endif /* CONFIG_LPC17_40_SSP0 || CONFIG_LPC17_40_SSP1 */
