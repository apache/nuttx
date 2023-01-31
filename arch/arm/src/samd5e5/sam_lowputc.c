/****************************************************************************
 * arch/arm/src/samd5e5/sam_lowputc.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/sam_pm.h"
#include "sam_config.h"
#include "sam_gclk.h"
#include "sam_sercom.h"
#include "sam_usart.h"
#include "sam_lowputc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#ifdef SAMD5E5_HAVE_USART
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

#ifdef SAMD5E5_HAVE_USART
static inline int
sam_usart_configure(const struct sam_usart_config_s * const config)
{
  uint32_t ctrla;
  uint32_t ctrlb;
  uint16_t baud;
  uint64_t tmp;

  /* The CTRLA, CTRLB, and BAUD registers are enable protected, meaning that
   * they can only be written when the USART is disabled.  We can just write
   * zero because only the ENABLE and SWRST bits can be modified while the
   * USART is enabled.
   */

  putreg32(0, config->base + SAM_USART_CTRLA_OFFSET);

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

  /* Write configuration to CTRLA */

  sam_wait_synchronization(config);
  putreg32(ctrla, config->base + SAM_USART_CTRLA_OFFSET);

  /* Write configuration to CTRLB */

  sam_wait_synchronization(config);
  putreg32(ctrlb, config->base + SAM_USART_CTRLB_OFFSET);

  /* Enable the USART */

  sam_wait_synchronization(config);
  ctrla |= USART_CTRLA_ENABLE;
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

#ifdef SAMD5E5_HAVE_USART
static inline void
sam_pad_configure(const struct sam_usart_config_s * const config)
{
  /* Configure SERCOM pads */

  if (config->pad0 != 0)
    {
      sam_portconfig(config->pad0);
    }

  if (config->pad1 != 0)
    {
      sam_portconfig(config->pad1);
    }

  if (config->pad2 != 0)
    {
      sam_portconfig(config->pad2);
    }

  if (config->pad3 != 0)
    {
      sam_portconfig(config->pad3);
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
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART
int sam_usart_internal(const struct sam_usart_config_s * const config)
{
  int ret;

  /* Configure the GCLKs for the SERCOM module */

  sercom_coreclk_configure(config->sercom, config->coregen,
                           (bool)config->corelock);
  sercom_slowclk_configure(config->sercom, config->slowgen);

  /* Enable clocking to the SERCOM module */

  sercom_enable(config->sercom);

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
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART
int sam_usart_initialize(const struct sam_usart_config_s * const config)
{
  irqstate_t flags;
  int ret;

  /* Reset the SERCOM so that we know that it is in its initial state */

  flags = enter_critical_section();
  sam_usart_reset(config);

  /* Just invoke the internal implementation, but with interrupts disabled
   * so that the operation is atomic.
   */

  ret = sam_usart_internal(config);
  leave_critical_section(flags);
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
 ****************************************************************************/

#ifdef SAMD5E5_HAVE_USART
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

  while ((getreg32(regaddr) & USART_CTRLA_SWRST) != 0)
    {
    }
}
#endif

/****************************************************************************
 * Name: sam_lowputc
 *
 * Description:
 *   Output one character to the USART using a simple polling method.
 *
 ****************************************************************************/

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
