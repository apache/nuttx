/****************************************************************************
 * arch/arm/src/samdl/sam_lowputc.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#include "sam_gclk.h"
#include "sam_pm.h"
#include "sam_sercom.h"
#include "sam_usart.h"
#include "sam_lowputc.h"

#include <arch/board/board.h>

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

#ifdef SAMDL_HAVE_USART
static void
sam_wait_synchronization(const struct sam_usart_config_s * const config)
{
  while (usart_syncbusy(config));
}
#endif

/****************************************************************************
 * Name: sam_usart_configure
 *
 * Description:
 *   Configure the SERCOM USART operating mode (as a normal UART).
 *
 ****************************************************************************/

#ifdef SAMDL_HAVE_USART
static inline int
sam_usart_configure(const struct sam_usart_config_s * const config)
{
  uint32_t ctrla;
  uint32_t ctrlb;
  uint16_t baud;
  uint64_t tmp;

  /* Calculate BAUD divider from the source clock frequency and desired.
   * baud.  For asynchronous mode, the formula for the baud generation is
   *
   *   Fbaud = (Frefclk / 16) * (1 - (BAUD / 65,536))
   *
   * Or,
   *
   *   BAUD  = 65,536 * (1 - 16 * (Fbaud / Fref))
   *         = 65,536 - 16 * 65,536 * Fbaud / Fref
   *
   * Example: Fref = 48MHz and Fbaud = 9600
   *
   *   BAUD  = 65,326
   *   Fbaud = 9600
   *
   * Example: Fref = 48MHz and Fbaud = 115,200
   *
   *   BAUD  = 63,019
   *   Fbaud = 115,219
   *
   * REVISIT: For the SAML21, only 16x sampling with arithmetic BAUD is
   * supported.
   */

  tmp = (uint64_t)config->baud << 20;
  tmp = (tmp + (config->frequency >> 1)) / config->frequency;

  /* Verify that the calculated result is within range */

  if (tmp < 1 || tmp > UINT16_MAX)
    {
      return -ERANGE;
    }

  baud = 65536 - (uint16_t)tmp;

  /* Disable all USART interrupts */

  putreg8(USART_INT_ALL, config->base + SAM_USART_INTENCLR_OFFSET);

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Set baud divisor */

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

  putreg32(ctrlb, config->base + SAM_USART_CTRLB_OFFSET);

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Write configuration to CTRLA */

  putreg32(ctrla, config->base + SAM_USART_CTRLA_OFFSET);
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

#ifdef SAMDL_HAVE_USART
static inline void
sam_pad_configure(const struct sam_usart_config_s * const config)
{
  /* Configure SERCOM pads */

  if (config->pad0 != 0)
    {
      sam_configport(config->pad0);
    }

  if (config->pad1 != 0)
    {
      sam_configport(config->pad1);
    }

  if (config->pad2 != 0)
    {
      sam_configport(config->pad2);
    }

  if (config->pad3 != 0)
    {
      sam_configport(config->pad3);
    }
}
#endif

/****************************************************************************
 * Name: sam_usart_internal
 *
 * Description:
 *   Set the configuration of a SERCOM for provided USART configuration.
 *   This configures the SERCOM as a USART, but does not configure USART
 *   interrupts or enable the USART.
 *
 *****************************************************************************/

#ifdef SAMDL_HAVE_USART
int sam_usart_internal(const struct sam_usart_config_s * const config)
{
  int ret;

  /* Enable clocking to the SERCOM module */

  sercom_enable(config->sercom);

  /* Configure the GCLKs for the SERCOM module */

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
  sercom_coreclk_configure(config->sercom, config->gclkgen, false);
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
  sam_gclk_chan_enable(config->sercom + GCLK_CHAN_SERCOM0_CORE,
                       config->gclkgen);
#endif
  sercom_slowclk_configure(config->sercom, config->slowgen);

  /* Set USART configuration according to the board configuration */

  ret = sam_usart_configure(config);
  if (ret == OK)
    {
      /* Configure USART pins */

      sam_pad_configure(config);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: sam_usart_enable
 *
 * Description:
 *   Enable the SERCOM USART (without enabling interrupts).
 *
 ****************************************************************************/

#ifdef SAMDL_HAVE_USART
static inline void
sam_usart_enable(const struct sam_usart_config_s * const config)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Wait until synchronization is complete */

  sam_wait_synchronization(config);

  /* Enable USART module */

  regaddr = config->base + SAM_USART_CTRLA_OFFSET;
  regval = getreg32(regaddr);
  regval |= USART_CTRLA_ENABLE;
  putreg32(regval, regaddr);
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
#ifdef HAVE_SERIAL_CONSOLE
  /* Configure and enable the console USART */

  VERIFY(sam_usart_internal(&g_consoleconfig));
  sam_usart_enable(&g_consoleconfig);
#endif
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

#ifdef SAMDL_HAVE_USART
int sam_usart_initialize(const struct sam_usart_config_s * const config)
{
  irqstate_t flags;
  int ret;

  /* Reset the SERCOM so that we know that it is in its initial state */

  flags = irqsave();
  sam_usart_reset(config);

  /* Just invoke the internal implementation, but with interrupts disabled
   * so that the operation is atomic.
   */

  ret = sam_usart_internal(config);
  irqrestore(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: sam_usart_reset
 *
 * Description:
 *   Reset the USART SERCOM.  This restores all SERCOM register to the
 *   initial state and disables the SERCOM.
 *
 *****************************************************************************/

#ifdef SAMDL_HAVE_USART
void sam_usart_reset(const struct sam_usart_config_s * const config)
{
  uintptr_t regaddr = config->base + SAM_USART_CTRLA_OFFSET;
  uint32_t regval;

  /* Reset the SERCOM by setting the SWRST bit in the CTRLA register.  When
   * the reset completes, the SERCOM will registers will be restored to there
   * initial state and the SERCOM will be disabled.
   */

  regval = getreg32(regaddr);
  regval |= USART_CTRLA_SWRST;
  putreg32(regval, regaddr);

  /* Wait for the reset to complete */

  while ((getreg32(regaddr) & USART_CTRLA_SWRST) != 0);
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
  uintptr_t base    = g_consoleconfig.base;
  uintptr_t intflag = base + SAM_USART_INTFLAG_OFFSET;

  /* Wait for the USART to be ready for new TX data */

  while ((getreg8(intflag) & USART_INT_DRE) == 0);

  /* Wait until synchronization is complete */

  sam_wait_synchronization(&g_consoleconfig);

  /* Write data to USART module */

  putreg16((uint16_t)ch, base + SAM_USART_DATA_OFFSET);

  /* Wait until data is sent */

  while ((getreg8(intflag) & USART_INT_TXC) == 0);
}
#endif
