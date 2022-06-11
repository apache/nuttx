/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p107/src/stm32_encx24j600.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/encx24j600.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_spi.h"

#include "olimex-stm32-p107.h"

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
  void                    *arg;      /* Argument that accompanies the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct enc_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct enc_lower_s *lower);
static void up_disable(const struct enc_lower_s *lower);

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
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(const struct enc_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void up_enable(const struct enc_lower_s *lower)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler != NULL);
  stm32_gpiosetevent(GPIO_ENCX24J600_INTR, false, true, true,
                     priv->handler, priv->arg);
}

/* REVISIT:  Since the interrupt is torn down completely, any interrupts
 * the occur while "disabled" will be lost.
 */

static void up_disable(const struct enc_lower_s *lower)
{
  stm32_gpiosetevent(GPIO_ENCX24J600_INTR, false, true, true,
                     NULL, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Assumptions:
   * 1) ENCX24J600 pins were configured in up_spi.c early in the boot-up
   *    phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in
   *    boot-up.
   */

  spi = stm32_spibus_initialize(ENCX24J600_SPI_PORTNO);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n",
            ENCX24J600_SPI_PORTNO);
      return;
    }

  /* Bind the SPI port to the ENCX24J600 driver */

  ret = enc_initialize(spi, &g_enclower.lower, ENCX24J600_DEVNO);

  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d ENCX24J600 device %d: %d\n",
           ENCX24J600_SPI_PORTNO, ENCX24J600_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to ENCX24J600 device %d\n",
        ENCX24J600_SPI_PORTNO, ENCX24J600_DEVNO);
}

#endif /* CONFIG_ENCX24J600 */
