/**************************************************************************
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "kl_config.h"
#include "kl_lowputc.h"
#include "kl_internal.h"
#include "chip/kl_uart.h"
#include "chip/kl_sim.h"
#include "chip/kl_port.h"
#include "chip/kl_uart.h"
#include "chip/kl_pinmux.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/
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

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/* This array maps an encoded FIFO depth (index) to the actual size of the
 * FIFO (indexed value).  NOTE:  That there is no 8th value.
 */

#ifdef CONFIG_KL_UARTFIFOS
static uint8_t g_sizemap[8] = {1, 4, 8, 16, 32, 64, 128, 0};
#endif

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 **************************************************************************/

void kl_lowputc(uint32_t ch)
{
#if defined HAVE_UART_DEVICE && defined HAVE_SERIAL_CONSOLE
#ifdef CONFIG_KL_UARTFIFOS
  /* Wait until there is space in the TX FIFO:  Read the number of bytes
   * currently in the FIFO and compare that to the size of the FIFO.  If
   * there are fewer bytes in the FIFO than the size of the FIFO, then we
   * are able to transmit.
   */

#  error "Missing logic"
#else
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

#endif

 /* Then write the character to the UART data register */

  putreg8((uint8_t)ch, CONSOLE_BASE+KL_UART_D_OFFSET);

#endif
}

/**************************************************************************
 * Name: kl_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/
void kl_lowsetup(void)
{
   uint32_t regval;
   uint8_t regval8;

   //regval = getreg32(KL_SIM_SOPT2);
   //regval |= SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_UART0SRC_MCGCLK ;
   //putreg32(regval, KL_SIM_SOPT2);

   regval = getreg32(KL_SIM_SCGC5);
   regval |= SIM_SCGC5_PORTA;
   putreg32(regval, KL_SIM_SCGC5);

   regval = getreg32(KL_SIM_SCGC4);
   regval |= SIM_SCGC4_UART0;
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

   // Set the baud rate divisor
   #define CORE_CLOCK 48000000
   #define OVER_SAMPLE 16
   uint16_t divisor = (CORE_CLOCK / OVER_SAMPLE) / CONSOLE_BAUD;
   regval8 = OVER_SAMPLE - 1;
   putreg8(regval8, KL_UART0_C4);

   regval8 = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
   putreg8(regval8, KL_UART0_BDH);

   regval8 = (divisor & UARTLP_BDL_SBR_MASK);
   putreg8(regval8, KL_UART0_BDL);

   /* Enable UART before changing registers */
   regval8 = getreg8(KL_UART0_C2);
   regval8 |= (UART_C2_RE | UART_C2_TE);
   putreg8(regval8, KL_UART0_C2);

  /* Configure the console (only) now.  Other UARTs will be configured
   * when the serial driver is opened.
   */

//#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

//  kl_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_FREQ,
//                   CONSOLE_PARITY, CONSOLE_BITS);
//#endif
}

/******************************************************************************
 * Name: kl_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartreset(uintptr_t uart_base)
{
  uint8_t regval;

  /* Just disable the transmitter and receiver */

  regval = getreg8(uart_base+KL_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE|UART_C2_TE);
  putreg8(regval, uart_base+KL_UART_C2_OFFSET);
}
#endif

/******************************************************************************
 * Name: kl_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void kl_uartconfigure(uintptr_t uart_base, uint32_t baud, uint32_t clock,
                      unsigned int parity, unsigned int nbits)
{
  uint32_t     sbr;
  uint32_t     brfa;
  uint32_t     tmp;
  uint8_t      regval;
#ifdef CONFIG_KL_UARTFIFOS
  unsigned int depth;
#endif

  /* Disable the transmitter and receiver throughout the reconfiguration */

  regval = getreg8(uart_base+KL_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE|UART_C2_TE);
  putreg8(regval, uart_base+KL_UART_C2_OFFSET);

  /* Configure number of bits, stop bits and parity */

  regval = 0;

  /* Check for odd parity */

  if (parity == 1)
    {
      regval |= (UART_C1_PE|UART_C1_PT); /* Enable + odd parity type */
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

  putreg8(regval, uart_base+KL_UART_C1_OFFSET);

  /* Calculate baud settings (truncating) */

  sbr = clock / (baud << 4);
  DEBUGASSERT(sbr < 0x2000);

  /* Save the new baud divisor, retaining other bits in the UARTx_BDH
   * register.
   */

  regval  = getreg8(uart_base+KL_UART_BDH_OFFSET) & UART_BDH_SBR_MASK;
  tmp     = sbr >> 8;
  regval |= (((uint8_t)tmp) << UART_BDH_SBR_SHIFT) & UART_BDH_SBR_MASK;
  putreg8(regval, uart_base+KL_UART_BDH_OFFSET);

  regval  = sbr & 0xff;
  putreg8(regval, uart_base+KL_UART_BDL_OFFSET);

  /* Calculate a fractional divider to get closer to the requested baud.
   * The fractional divider, BRFA, is a 5 bit fractional value that is
   * logically added to the SBR:
   *
   *   UART baud rate = clock / (16 × (SBR + BRFD))
   *
   * The BRFA the remainder.  This will be a non-negative value since the SBR
   * was calculated by truncation.
   */

  tmp  = clock - (sbr * (baud << 4));
  brfa = (tmp << 5) / (baud << 4);

  /* Set the BRFA field (retaining other bits in the UARTx_C4 register) */

  regval  = getreg8(uart_base+KL_UART_C4_OFFSET) & UART_C4_BRFA_MASK;
  regval |= ((uint8_t)brfa << UART_C4_BRFA_SHIFT) & UART_C4_BRFA_MASK;
  putreg8(regval, uart_base+KL_UART_C4_OFFSET);

  /* Set the FIFO watermarks.
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

#ifdef CONFIG_KL_UARTFIFOS
  depth = g_sizemap[(regval & UART_PFIFO_RXFIFOSIZE_MASK) >> UART_PFIFO_RXFIFOSIZE_SHIFT];
  if (depth > 1)
    {
      depth = (3 * depth) >> 2;
    }
  putreg8(depth , uart_base+KL_UART_RWFIFO_OFFSET);

  depth = g_sizemap[(regval & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT];
  if (depth > 3)
    {
      depth = (depth >> 2);
    }
  putreg8(depth, uart_base+KL_UART_TWFIFO_OFFSET);

  /* Enable RX and TX FIFOs */

  putreg8(UART_PFIFO_RXFE | UART_PFIFO_TXFE, uart_base+KL_UART_PFIFO_OFFSET);
#else
  /* Otherwise, disable the FIFOs.  Then the FIFOs are disable, the effective
   * FIFO depth is 1.  So set the watermarks as follows:
   *
   * TWFIFO[TXWATER] = 0:  TDRE will be set when the number of queued bytes
   *  (1 in this case) is less than or equal to 0.
   * RWFIFO[RXWATER] = 1:  RDRF will be set when the number of queued bytes
   *  (1 in this case) is greater than or equal to 1.
   *
   * Set the watermarks to one/zero and disable the FIFOs
   */

  putreg8(1, uart_base+KL_UART_RWFIFO_OFFSET);
  putreg8(0, uart_base+KL_UART_TWFIFO_OFFSET);
  putreg8(0, uart_base+KL_UART_PFIFO_OFFSET);
#endif

  /* Now we can (re-)enable the transmitter and receiver */

  regval = getreg8(uart_base+KL_UART_C2_OFFSET);
  regval |= (UART_C2_RE | UART_C2_TE);
  putreg8(regval, uart_base+KL_UART_C2_OFFSET);
}
#endif



