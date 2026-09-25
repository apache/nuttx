/****************************************************************************
 * arch/arm/src/ra8m1/ra_lowputc.c
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
#include <stdlib.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "ra_lowputc.h"
#include "ra_gpio.h"
#include "hardware/ra8m1_sci.h"
#include "hardware/ra8m1_mstp.h"
#include "hardware/ra8m1_system.h"

/* The board.h file may redefine pin configurations defined in ra_pinmap.h */

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA8M1 only implements SCI_B0-SCI_B4 and SCI_B9 (SCI_B5-8 do not exist). */

/* Is there a serial console?  It could be on SCI0-4 or 9 */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI0_UART)
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI1_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI2_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI3_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI4_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI9_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#else
#ifndef CONFIG_NO_SERIAL_CONSOLE
#warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#endif

#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#undef HAVE_CONSOLE
#endif

#if defined(HAVE_CONSOLE)

/* Select SCI_B parameters for the selected console */

#  if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B0_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB31
#    define RA_CONSOLE_BAUD     CONFIG_SCI0_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI0_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI0_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI0_2STOP
#  elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B1_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB30
#    define RA_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI1_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI1_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#  elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B2_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB29
#    define RA_CONSOLE_BAUD     CONFIG_SCI2_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI2_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI2_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI2_2STOP
#  elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B3_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB28
#    define RA_CONSOLE_BAUD     CONFIG_SCI3_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI3_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI3_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI3_2STOP
#  elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B4_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB27
#    define RA_CONSOLE_BAUD     CONFIG_SCI4_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI4_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI4_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI4_2STOP
#  elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI_B9_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_MSTPB22
#    define RA_CONSOLE_BAUD     CONFIG_SCI9_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI9_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI9_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI9_2STOP
#  else
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#  endif

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef HAVE_CONSOLE
static spinlock_t g_ra_lowputc_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      while ((getreg32(RA_CONSOLE_BASE + R_SCI_B_CSR_OFFSET)
        & R_SCI_B_CSR_TEND) == 0)
        {
        }

      /* Disable interrupts so that the test and the transmission are
       * atomic.
       */

      flags = spin_lock_irqsave(&g_ra_lowputc_lock);
      if ((getreg32(RA_CONSOLE_BASE + R_SCI_B_CSR_OFFSET)
        & R_SCI_B_CSR_TEND)  == R_SCI_B_CSR_TEND)
        {
          /* Send the character */

          putreg32((uint32_t)ch & R_SCI_B_TDR_TDAT_MASK,
                    RA_CONSOLE_BASE + R_SCI_B_TDR_OFFSET);

          spin_unlock_irqrestore(&g_ra_lowputc_lock, flags);
          return;
        }

      spin_unlock_irqrestore(&g_ra_lowputc_lock, flags);
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  arm_lowputc(ch);
#endif
}

/****************************************************************************
 * Name: ra_sci_baud_ccr2
 *
 * Description:
 *   See ra_lowputc.h.  Table 31.7 gives, for asynchronous mode,
 *
 *     N = TCLK / (D x 2^(2n-1) x B) - 1
 *
 *   where N is BRR, n is CCR2.CKS (TCLK divided by 1, 4, 16 or 64) and D
 *   is 64, 32, 16 or 12 depending on BGDM/ABCS/ABCSE.  Try every D and n
 *   and keep the pair with the smallest baud rate error.
 *
 ****************************************************************************/

uint32_t ra_sci_baud_ccr2(uint32_t sciclk_hz, uint32_t baud)
{
  static const uint8_t div_baud[4] =
    {
      12, 16, 32, 64
    };

  int64_t   brr             = 0;
  uint32_t  reg_brr         = 0;
  uint32_t  best_brr        = 0;
  uint32_t  actual_baudrate = 0;
  int64_t   error           = 0;
  int64_t   min_error       = INT64_MAX;
  uint8_t   best_n          = 0;
  uint8_t   best_i          = 0;
  uint32_t  regval          = 0;

  for (uint8_t i = 0; i < 4; i++)
    {
      for (uint8_t n = 0; n < 4; n++)
        {
          uint32_t  div_n       = (n == 0) ? 1 : (1U << (2 * n - 1));
          uint32_t  multiplier  = (n == 0) ? 2UL : 1UL;

          /* The products below reach several billion (for example 64 x 32
           * x 3000000 baud), so they must be 64-bit.
           */

          brr = (int64_t)(((uint64_t)sciclk_hz * 100UL * multiplier) /
                          ((uint64_t)div_baud[i] * div_n * baud)) - 100;

          /* A negative value means this setting can not reach the requested
           * baud rate; use its fastest one (N = 0), which is the closest.
           */

          if (brr < 0)
            {
              brr = 0;
            }

          reg_brr = (uint32_t)((brr + 50) / 100);

          if (reg_brr > 255)
            {
              continue;
            }

          actual_baudrate = (sciclk_hz * multiplier) /
                            (div_baud[i] * div_n * (reg_brr + 1));

          /* Error in thousandths of a percent.  The baud rate difference
           * times 100000 does not fit in 32 bits, so it is 64-bit and
           * signed.
           */

          error = (((int64_t)actual_baudrate - (int64_t)baud) * 100000) /
                  (int64_t)baud;

          /* Store the best values if we find a new minimum error */

          if (llabs(error) < llabs(min_error))
            {
              min_error = error;
              best_n    = n;
              best_i    = i;
              best_brr  = reg_brr;
            }
        }
    }

  switch (best_i)
    {
      case 0:
        regval = R_SCI_B_CCR2_ABCSE;
        break;

      case 1:
        regval = R_SCI_B_CCR2_BGDM | R_SCI_B_CCR2_ABCS;
        break;

      case 2:
        regval = R_SCI_B_CCR2_BGDM;
        break;

      default:
        break;
    }

  regval |= (best_brr << R_SCI_B_CCR2_BRR_SHIFT);
  regval |= ((uint32_t)best_n << R_SCI_B_CCR2_CKS_SHIFT);

  return regval;
}

/****************************************************************************
 * Name: ra_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void ra_lowsetup(void)
{
#ifdef HAVE_CONSOLE
  uint32_t regval;

  /* Configure only the console's own pins.  This must key off which
   * channel is the console (CONFIG_SCIn_SERIAL_CONSOLE), not off which
   * SCI_B drivers are built in (CONFIG_RA_SCIn_UART): several channels
   * may be enabled at once, but ra_lowsetup only ever brings up the one
   * that is the console.
   */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI0_RX);
  ra_configgpio(GPIO_SCI0_TX);
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI1_RX);
  ra_configgpio(GPIO_SCI1_TX);
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI2_RX);
  ra_configgpio(GPIO_SCI2_TX);
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI3_RX);
  ra_configgpio(GPIO_SCI3_TX);
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI4_RX);
  ra_configgpio(GPIO_SCI4_TX);
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI9_RX);
  ra_configgpio(GPIO_SCI9_TX);
#endif

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC1),
           R_SYSTEM_PRCR_S);
  modifyreg32(R_MSTP_MSTPCRB, RA_CONSOLE_MTSP, 0);
  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);

  /* Disable the channel while it is configured */

  regval = 0;
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR0_OFFSET);

  /* Baud rate generator: ra_clockconfig() has already set SCICLK, which
   * is the baud rate generator's input clock (CCR3.BPEN is 0 after
   * reset).  Load the console baud rate, with the remaining CCR2 fields
   * chosen for the smallest error.
   */

  regval = ra_sci_baud_ccr2(RA_SCICLK_FREQUENCY, RA_CONSOLE_BAUD);
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR2_OFFSET);

  regval = (R_SCI_B_CCR0_TE | R_SCI_B_CCR0_RE);
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR0_OFFSET);
#endif /* HAVE_CONSOLE */
}
