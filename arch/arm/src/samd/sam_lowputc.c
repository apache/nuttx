/****************************************************************************
 * arch/arm/src/samd/sam_lowputc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J–SAM–12/2013
 *   2. Atmel sample code.  This code has an ASF license with is compatible
 *      with the NuttX BSD license, but includes the provision that this
 *      code not be used in non-Atmel products.  That sample code was used
 *      only as a reference so I believe that only the NuttX BSD license
 *      applies.
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
#include <assert.h>
#include <errno.h>

#include "up_arch.h"

#include "sam_config.h"
#include "chip/sam_pm.h"
#include "chip/sam_usart.h"
#include "sam_usart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef OK
#  define OK 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wait_synchronization
 *
 * Description:
 *   Wait until the SERCOM USART reports that it is synchronized.
 *
 ****************************************************************************/

#ifdef HAVE_USART
static void
sam_wait_synchronization(const struct sam_usart_config_s * const config)
{
  while ((getreg16(config->base + SAM_USART_STATUS_OFFSET) & USART_STATUS_SYNCBUSY) != 0);
}
#endif

/****************************************************************************
 * Name: sam_gclk_configure
 *
 * Description:
 *   Configure the SERCOM USART source clock.
 *
 ****************************************************************************/

#ifdef HAVE_USART
static inline int
sam_gclk_configure(const struct sam_usart_config_s * const config)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: sam_usart_configure
 *
 * Description:
 *   Configure the SERCOM USART operating mode (as a normal UART).
 *
 ****************************************************************************/

#ifdef HAVE_USART
static inline int
sam_usart_configure(const struct sam_usart_config_s * const config)
{
  uint32_t ctrla;
  uint32_t ctrlb;
  uint32_t baud;

  /* Check if baud is within the valid range. */

  if (config->baud > (config->frequency >> 1))
    {
      return -ERANGE;
    }

  /* Calculate BAUD divider from the source clock frequency and desired baud */

  baud = (config->frequency / (2 * config->baud)) - 1;

  /* Verify that the calculated result is within range */

  if (baud > UINT16_MAX)
    {
      return -ERANGE;
    }

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Set baud val */

  putreg16((uint16_t)baud, config->base + SAM_USART_BAUD_OFFSET);

  /* Configure the USART CTRLA and CTRLB registers */

  ctrla = (USART_CTRLA_MODE_INTUSART  | (uint32_t)config->muxconfig |
           USART_CTRLA_ASYNCH | USART_CTRLA_CPOL_NORMAL |
           USART_CTRLA_LSBFIRST);
  ctrlb = (USART_CTRLB_TXEN | USART_CTRLB_RXEN);

  /* Set the number of stop bits */

  if (config->stopbits2)
    {
      ctrlb |= USART_CTRLB_SBMODE;
    }

  /* Set the USART word size */

  switch (config->bits)
    {
      case 5:
        ctrlb |= USART_CTRLB_CHSIZE_5BITS;
        break;

      case 6:
        ctrlb |= USART_CTRLB_CHSIZE_6BITS;
        break;

      case 7:
        ctrlb |= USART_CTRLB_CHSIZE_7BITS;
        break;

      default:
      case 8:
        break;

      case 9:
        ctrlb |= USART_CTRLB_CHSIZE_9BITS;
        break;
    }

  /* Set parity mode */

  switch (config->parity)
    {
      default:
      case 0: /* None */
        break;

      case 1: /* Odd */
        ctrlb |= USART_CTRLB_PODD;
        /* Fall through */

      case 2: /* Even */
        ctrla |= USART_CTRLA_FORM_PARITY;
        break;
    }

#if 0 /* Not supported */
  /* Set run mode during device sleep */

  if (config->runinstandby)
    {
      /* Enable in sleep mode */

      ctrla |= USART_CTRLA_RUNSTDBY;
    }
#endif

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Write configuration to CTRLB */

  putreg32(ctrlb, SAM_USART5_CTRLB);

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Write configuration to CTRLA */

  putreg32(ctrlb, SAM_USART5_CTRLA);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_pad_configure
 *
 * Description:
 *   Configure the SERCOM USART pads.
 *
 ****************************************************************************/

#ifdef HAVE_USART
static inline int
sam_pad_configure(const struct sam_usart_config_s * const config)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization.
 *
 ****************************************************************************/

void sam_lowsetup(void)
{
#warning Missing logic
}

/****************************************************************************
 * Name: sam_usart_initialize
 *
 * Description:
 *   Set the configuration of a SERCOM for provided USART configuration.
 *   This configures the SERCOM as a USART, but does not configure USART
 *   interrupts or enable the USART.
 *
 *****************************************************************************/

#ifdef HAVE_USART
int sam_usart_initialize(const struct sam_usart_config_s * const config)
{
  uint32_t regval;
  int ret;

  /* Enable clocking to the SERCOM module in PM */

  regval  = getreg32(SAM_PM_APBCMASK);
  regval |= PM_APBCMASK_SERCOM(config->sercom);
  putreg32(regval, SAM_PM_APBCMASK);

  /* Configure the GCCLK for the SERCOM module */

  ret = sam_gclk_configure(config);
  if (ret < 0)
    {
      return ret;
    }

  /* Set configuration according to the board configuration */

  ret = sam_usart_configure(config);
  if(ret < 0)
    {
      return ret;
    }

  /* Configure USART pins */

  return sam_pad_configure(config);
}
#endif

/****************************************************************************
 * Name: sam_lowputc
 *
 * Description:
 *   Output one character to the USART using a simple polling method.
 *
 *****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void sam_lowputc(uint32_t ch)
{
#warning Missing logic
}
#endif
