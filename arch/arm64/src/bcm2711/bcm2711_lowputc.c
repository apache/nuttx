/***************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_lowputc.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "arm64_arch.h"
#include "bcm2711_gpio.h"
#include "hardware/bcm2711_aux.h"
#include <nuttx/config.h>

/***************************************************************************
 * Pre-processor definitions
 ***************************************************************************/

/* Clock starts at 5MHz. */

#define SYSTEM_CLOCK_FREQUENCY 500000000

/* Early serial baud rate. */

#ifndef BCM_EARLYSERIAL_BAUD
#define BCM_EARLYSERIAL_BAUD 115200
#endif // BCM_EARLYSERIAL_BAUD

/* Baud rate calculation */

#define AUX_MU_BAUD(baud) ((SYSTEM_CLOCK_FREQUENCY / (baud * 8)) - 1)

/***************************************************************************
 * Public Functions
 ***************************************************************************/

#ifdef CONFIG_ARCH_EARLY_PRINT

/***************************************************************************
 * Name: arm64_earlyprintinit
 *
 * Description:
 *   Configure BCM2711 Mini UART for polling driven operation.
 *
 ***************************************************************************/

void arm64_earlyprintinit(char ch)
{
  /* Enable Mini UART */

  modreg32(BCM_AUX_ENABLE_MU, BCM_AUX_ENABLE_MU, BCM_AUX_ENABLES);

  /* Disable interrupts. */

  modreg32(0, (BCM_AUX_MU_IER_RXD | BCM_AUX_MU_IER_TXD),
           BCM_AUX_MU_IER_REG);

  /* Disable TX and RX of the UART */

  modreg32(0, BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_REG);
  modreg32(0, BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_REG);

  /* Put the UART in 8 bit mode */

  modreg32(BCM_AUX_MU_LCR_DATA8B, BCM_AUX_MU_LCR_DATA8B,
           BCM_AUX_MU_LCR_REG);

  /* Ensure RTS line is low. */

  modreg32(0, BCM_AUX_MU_MCR_RTS, BCM_AUX_MU_MCR_REG);

  /* Clear the TX and RX FIFOs */

  putreg32(BCM_AUX_MU_IIR_RXCLEAR | BCM_AUX_MU_IIR_TXCLEAR,
           BCM_AUX_MU_IIR_REG);

  /* Set baud rate. */

  putreg32(AUX_MU_BAUD(BCM_EARLYSERIAL_BAUD), BCM_AUX_MU_BAUD_REG);

  /* GPIO 14 and GPIO 15 are used as TX and RX. */

  /* Turn off pull-up/pull-down resistors. */

  bcm2711_gpio_set_pulls(14, false, false);
  bcm2711_gpio_set_pulls(15, false, false);

  /* Use alternative function 5 (UART1). */

  bcm2711_gpio_set_func(14, BCM_GPIO_FUNC5);
  bcm2711_gpio_set_func(15, BCM_GPIO_FUNC5);

  /* Enable TX and RX again. */

  modreg32(BCM_AUX_MU_CNTL_TXENABLE, BCM_AUX_MU_CNTL_TXENABLE,
           BCM_AUX_MU_CNTL_REG);
  modreg32(BCM_AUX_MU_CNTL_RXENABLE, BCM_AUX_MU_CNTL_RXENABLE,
           BCM_AUX_MU_CNTL_REG);
}

/***************************************************************************
 * Name: arm64_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.
 *   This implementation uses the BCM2711's Mini UART with polling
 *   to output bytes.
 *
 ***************************************************************************/

void arm64_lowputc(char ch)
{
  /* Wait until space for one byte is free */

  while (!(getreg32(BCM_AUX_MU_STAT_REG) & BCM_AUX_MU_STAT_SPACEAVAIL))
    ;

  /* Add carriage return when there is a newline */

  if (ch == '\n')
    {
      putreg32('\r', BCM_AUX_MU_IO_REG);

      /* Wait for space again to add new line character */

      while (!(getreg32(BCM_AUX_MU_STAT_REG) & BCM_AUX_MU_STAT_SPACEAVAIL))
        ;
    }

  /* Send one byte */

  putreg32(ch, BCM_AUX_MU_IO_REG);
}

#endif // CONFIG_ARCH_EARLY_PRINT
