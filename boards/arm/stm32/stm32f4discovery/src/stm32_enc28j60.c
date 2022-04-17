/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_enc28j60.c
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

/* 2MBit SPI FLASH OR ENC28J60
 *
 * -- ---- ------------ -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * -- ---- ------------ -----------------------------------------------------
 *
 * 29 PA4 PA4-SPI1-NSS  10Mbit ENC28J60, SPI 2M FLASH
 * 30 PA5 PA5-SPI1-SCK  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 31 PA6 PA6-SPI1-MISO 2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 32 PA7 PA7-SPI1-MOSI 2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/enc28j60.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_spi.h"

#include "stm32f4discovery.h"

#ifdef CONFIG_ENC28J60

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* ENC28J60
 *
 * --- ------ -------------- ------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- ------------------------------------------------
 *
 * 29  PA4    PA4-SPI1-NSS   10Mbit ENC28J60, SPI 2M FLASH
 * 30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen,
 *                                    10Mbit ENC28J60, SPI 2M FLASH
 * 31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen,
 *                                    10Mbit ENC28J60, SPI 2M FLASH
 * 32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen,
 *                                    10Mbit ENC28J60, SPI 2M FLASH
 * 98  PE1    PE1-FSMC_NBL1  2.4" TFT + Touchscreen,
 *                                    10Mbit EN28J60 Reset
 * 4   PE5    (no name)      10Mbps ENC28J60 Interrupt
 */

/* ENC28J60 is on SPI1 */

#ifndef CONFIG_STM32_SPI1
# error "Need CONFIG_STM32_SPI1 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define ENC28J60_SPI_PORTNO 1   /* On SPI1 */
#define ENC28J60_DEVNO      0   /* Only one ENC28J60 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct enc_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                   handler;  /* ENC28J60 interrupt handler */
  void                    *arg;      /* Argument that accompanies the interrupt */
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

/* The ENC28J60 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENC28J60 GPIO interrupt.
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

  DEBUGASSERT(priv->handler);
  stm32_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true,
                     priv->handler, priv->arg);
}

/* REVISIT:  Since the interrupt is completely torn down, not just disabled,
 * in interrupt requests that occurs while the interrupt is disabled will be
 * lost.
 */

static void up_disable(const struct enc_lower_s *lower)
{
  stm32_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true,
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
   * 1) ENC28J60 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in
   *    boot-up.
   */

  spi = stm32_spibus_initialize(ENC28J60_SPI_PORTNO);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", ENC28J60_SPI_PORTNO);
      return;
    }

  /* Take ENC28J60 out of reset (active low) */

  stm32_gpiowrite(GPIO_ENC28J60_RESET, true);

  /* Bind the SPI port to the ENC28J60 driver */

  ret = enc_initialize(spi, &g_enclower.lower, ENC28J60_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d ENC28J60 device %d: %d\n",
           ENC28J60_SPI_PORTNO, ENC28J60_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to ENC28J60 device %d\n",
        ENC28J60_SPI_PORTNO, ENC28J60_DEVNO);
}

#endif /* CONFIG_ENC28J60 */
