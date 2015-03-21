/****************************************************************************
 * configs/olimex-stm32-p107/src/stm32_encx24j600.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/encx24j600.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "p107-internal.h"

#ifdef CONFIG_ENCX24J600

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* ENCX24J600
 *
 * --- ------ -------------- ------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- ------------------------------------------------
 *
 * 54  PB15   PB15-CS_UEXT   ENCX24J600 #CS
 * 78  PC10   PC10-SPI3-SCK  ENCX24J600 SCK
 * 79  PC11   PC11-SPI3-MISO ENCX24J600 MISO
 * 80  PC12   PC12-SPI3-MOSI ENCX24J600 MOSI
 * 95  PB8    PB8            ENCX24J600 #Interrupt
 */

/* ENCX24J600 is on SPI3 */

#ifndef CONFIG_STM32_SPI3
# error "Need CONFIG_STM32_SPI3 in the configuration"
#endif

#ifndef CONFIG_STM32_SPI3_REMAP
# error "SPI should be remapped for UEXT use"
#endif

/* SPI Assumptions **********************************************************/

#define ENCX24J600_SPI_PORTNO 3   /* On SPI1 */
#define ENCX24J600_DEVNO      0   /* Only one ENCX24J600 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct enc_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                   handler;  /* ENCX24J600 interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler);
static void up_enable(FAR const struct enc_lower_s *lower);
static void up_disable(FAR const struct enc_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ENCX24J600 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENCX24J600 GPIO interrupt.
 */

static struct stm32_lower_s g_enclower =
{
  .lower =
  {
    .attach  = up_attach,
    .enable  = up_enable,
    .disable = up_disable
  },
  .handler = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void up_enable(FAR const struct enc_lower_s *lower)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  (void)stm32_gpiosetevent(GPIO_ENCX24J600_INTR, false, true, true, priv->handler);
}

static void up_disable(FAR const struct enc_lower_s *lower)
{
  (void)stm32_gpiosetevent(GPIO_ENCX24J600_INTR, false, true, true, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Assumptions:
   * 1) ENCX24J600 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in boot-up.
   */

  spi = up_spiinitialize(ENCX24J600_SPI_PORTNO);
  if (!spi)
    {
      nlldbg("Failed to initialize SPI port %d\n", ENCX24J600_SPI_PORTNO);
      return;
    }

  /* Bind the SPI port to the ENCX24J600 driver */

  ret = enc_initialize(spi, &g_enclower.lower, ENCX24J600_DEVNO);

  if (ret < 0)
    {
      nlldbg("Failed to bind SPI port %d ENCX24J600 device %d: %d\n",
             ENCX24J600_SPI_PORTNO, ENCX24J600_DEVNO, ret);
      return;
    }

  nllvdbg("Bound SPI port %d to ENCX24J600 device %d\n",
          ENCX24J600_SPI_PORTNO, ENCX24J600_DEVNO);
}

#endif /* CONFIG_ENCX24J600 */
