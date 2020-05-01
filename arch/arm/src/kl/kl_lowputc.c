/****************************************************************************
 * arch/arm/src/kl/kl_lowputc.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "kl_config.h"
#include "kl_lowputc.h"
#include "kl_gpio.h"

#include "hardware/kl_uart.h"
#include "hardware/kl_sim.h"
#include "hardware/kl_port.h"
#include "hardware/kl_uart.h"
#include "hardware/kl_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KL_UART0_BASE
#  define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KL_UART1_BASE
#  define CONSOLE_FREQ     BOARD_BUSCLK_FREQ
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KL_UART2_BASE
#  define CONSOLE_FREQ     BOARD_BUSCLK_FREQ
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#endif

#define OVER_SAMPLE 16

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

void kl_lowputc(uint32_t ch)
{
#if defined HAVE_UART_DEVICE && defined HAVE_SERIAL_CONSOLE
  /* Wait until the transmit data register is "empty" (TDRE).  This state
   * depends on the TX watermark setting and may not mean that the transmit
   * buffer is truly empty.  It just means that we can now add another
   * characterto the transmit buffer without exceeding the watermark.
   *
   * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
   * (1-deep).  There appears to be no way to know when the FIFO is not
   * full (other than reading the FIFO length and comparing the FIFO count).
   * Hence, the FIFOs are not used in this implementation and, as a result
   * TDRE indeed mean that the single output buffer is available.
   *
   * Performance on UART0 could be improved by enabling the FIFO and by
   * redesigning all of the FIFO status logic.
   */

  while ((getreg8(CONSOLE_BASE+KL_UART_S1_OFFSET) & UART_S1_TDRE) == 0);

  /* Then write the character to the UART data register */

  putreg8((uint8_t)ch, CONSOLE_BASE+KL_UART_D_OFFSET);

#endif
}

/****************************************************************************
 * Name: kl_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/
void kl_lowsetup(void)
{
  uint32_t regval;
  uint8_t regval8;

#if 0
  regval = getreg32(KL_SIM_SOPT2);
  regval |= SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_UART0SRC_MCGCLK ;
  putreg32(regval, KL_SIM_SOPT2);
#endif

  regval = getreg32(KL_SIM_SCGC4);
#ifdef CONFIG_KL_UART0
  regval |= SIM_SCGC4_UART0;
#endif
#ifdef CONFIG_KL_UART1
  regval |= SIM_SCGC4_UART1;
#endif
#ifdef CONFIG_KL_UART2
  regval |= SIM_SCGC4_UART2;
#endif
  putreg32(regval, KL_SIM_SCGC4);

  regval = getreg32(KL_SIM_SOPT2);
  regval &= ~(SIM_SOPT2_UART0SRC_MASK);
  putreg32(regval, KL_SIM_SOPT2);

  regval = getreg32(KL_SIM_SOPT2);
  regval |= SIM_SOPT2_UART0SRC_MCGCLK;
  putreg32(regval, KL_SIM_SOPT2);

  putreg32((PORT_PCR_MUX_ALT2), KL_PORTA_PCR1);
  putreg32((PORT_PCR_MUX_ALT2), KL_PORTA_PCR2);

  /* Disable UART before changing registers */

  putreg8(0, KL_UART0_C2);
  putreg8(0, KL_UART0_C1);
  putreg8(0, KL_UART0_C3);
  putreg8(0, KL_UART0_S2);

  /* Set the baud rate divisor */

  uint16_t divisor = (CONSOLE_FREQ / OVER_SAMPLE) / CONSOLE_BAUD;
  regval8 = OVER_SAMPLE - 1;
  putreg8(regval8, KL_UART0_C4);

  regval8 = (divisor >> 8) & UART_BDH_SBR_MASK;
  putreg8(regval8, KL_UART0_BDH);

  regval8 = (divisor & UART_BDL_SBR_MASK);
  putreg8(regval8, KL_UART0_BDL);

  /* Enable UART before changing registers */

  regval8 = getreg8(KL_UART0_C2);
  regval8 |= (UART_C2_RE | UART_C2_TE);
  putreg8(regval8, KL_UART0_C2);

  /* Configure the console (only) now.  Other UARTs will be configured
   * when the serial driver is opened.
   */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  kl_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_FREQ,
                   CONSOLE_PARITY, CONSOLE_BITS);
#endif
}

/****************************************************************************
 * Name: kl_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartreset(uintptr_t uart_base)
{
  uint8_t regval;

  /* Just disable the transmitter and receiver */

  regval = getreg8(uart_base + KL_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base + KL_UART_C2_OFFSET);
}
#endif

/****************************************************************************
 * Name: kl_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartconfigure(uintptr_t uart_base, uint32_t baud, uint32_t clock,
                      unsigned int parity, unsigned int nbits)
{
  uint32_t     sbr;
  uint32_t     tmp;
  uint8_t      regval;

  /* Disable the transmitter and receiver throughout the reconfiguration */

  regval = getreg8(uart_base+KL_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base+KL_UART_C2_OFFSET);

  /* Configure number of bits, stop bits and parity */

  regval = 0;

  /* Check for odd parity */

  if (parity == 1)
    {
      regval |= (UART_C1_PE | UART_C1_PT); /* Enable + odd parity type */
    }

  /* Check for even parity */

  else if (parity == 2)
    {
      regval |= UART_C1_PE;              /* Enable (even parity default) */
    }

  /* The only other option is no parity */

  else
    {
      DEBUGASSERT(parity == 0);
    }

  /* Check for 9-bit operation */

  if (nbits == 9)
    {
      regval |= UART_C1_M;
    }

  /* The only other option is 8-bit operation */

  else
    {
      DEBUGASSERT(nbits == 8);
    }

  putreg8(regval, uart_base + KL_UART_C1_OFFSET);

  /* Calculate baud settings (truncating) */

  sbr = clock / (baud << 4);
  DEBUGASSERT(sbr < 0x2000);

  /* Save the new baud divisor, retaining other bits in the UARTx_BDH
   * register.
   */

  regval  = getreg8(uart_base + KL_UART_BDH_OFFSET) & UART_BDH_SBR_MASK;
  tmp     = sbr >> 8;
  regval |= (((uint8_t)tmp) << UART_BDH_SBR_SHIFT) & UART_BDH_SBR_MASK;
  putreg8(regval, uart_base + KL_UART_BDH_OFFSET);

  regval  = sbr & 0xff;
  putreg8(regval, uart_base + KL_UART_BDL_OFFSET);

  /* Now we can (re-)enable the transmitter and receiver */

  regval = getreg8(uart_base + KL_UART_C2_OFFSET);
  regval |= (UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base + KL_UART_C2_OFFSET);
}
#endif
