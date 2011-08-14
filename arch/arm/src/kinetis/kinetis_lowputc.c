/**************************************************************************
 * arch/arm/src/kinetis/kinetis_lowputc.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "kinetis_config.h"
#include "kinetis_internal.h"
#include "kinetis_uart.h"
#include "kinetis_sim.h"
#include "kinetis_pinmux.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/
     
/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART0_BASE
#  define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART1_BASE
#  define CONSOLE_FREQ     BOARD_CORECLK_FREQ
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART2_BASE
#  define CONSOLE_FREQ     BOARD_BUS_FREQ
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART3_BASE
#  define CONSOLE_FREQ     BOARD_BUS_FREQ
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART4_BASE
#  define CONSOLE_FREQ     BOARD_BUS_FREQ
#  define CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define CONSOLE_BITS     CONFIG_UART4_BITS
#  define CONSOLE_PARITY   CONFIG_UART4_PARITY
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE     KINETIS_UART5_BASE
#  define CONSOLE_FREQ     BOARD_BUS_FREQ
#  define CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define CONSOLE_BITS     CONFIG_UART5_BITS
#  define CONSOLE_PARITY   CONFIG_UART5_PARITY
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
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

void up_lowputc(char ch)
{
#if defined HAVE_UART_DEVICE && defined HAVE_SERIAL_CONSOLE

  /* Wait until the transmit data register is "empty."  This state depends
   * on the TX watermark setting and does not mean that the transmit buffer
   * is really empty.  It just means that we can now add another character
   * to the transmit buffer
   */

  while ((getreg8(CONSOLE_BASE+KINETIS_UART_S1_OFFSET) & UART_S1_TDRE) == 0);

 /* Then write the character to the UART data register */

  putreg8((uint8_t)ch, CONSOLE_BASE+KINETIS_UART_D_OFFSET);
#endif
}

/**************************************************************************
 * Name: kinetis_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 *   The UART0/1/2/3 peripherals are configured using the following registers:
 *   1. Power: In the PCONP register, set bits PCUART0/1/2/3.
 *      On reset, UART0 and UART 1 are enabled (PCUART0 = 1 and PCUART1 = 1)
 *      and UART2/3 are disabled (PCUART1 = 0 and PCUART3 = 0).
 *   2. Peripheral clock: In the PCLKSEL0 register, select PCLK_UART0 and
 *      PCLK_UART1; in the PCLKSEL1 register, select PCLK_UART2 and PCLK_UART3.
 *   3. Baud rate: In the LCR register, set bit DLAB = 1. This enables access
 *      to registers DLL and DLM for setting the baud rate. Also, if needed,
 *      set the fractional baud rate in the fractional divider 
 *   4. UART FIFO: Use bit FIFO enable (bit 0) in FCR register to
 *      enable FIFO.
 *   5. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *   6. Interrupts: To enable UART interrupts set bit DLAB = 0 in the LCRF
 *      register. This enables access to IER. Interrupts are enabled
 *      in the NVIC using the appropriate Interrupt Set Enable register.
 *   7. DMA: UART transmit and receive functions can operate with the
 *      GPDMA controller.
 *
 **************************************************************************/

void kinetis_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
  uint32_t regval;

  /* Enable peripheral clocking for all enabled UARTs.  Clocking for UARTs
   * 0-3 is enabled in the SCGC4 register.
   */

#if defined(CONFIG_KINETIS_UART0) || defined(CONFIG_KINETIS_UART1) || \
    defined(CONFIG_KINETIS_UART2) || defined(CONFIG_KINETIS_UART3)

   regval = getreg32(KINETIS_SIM_SCGC4);
#  ifdef CONFIG_KINETIS_UART0
   regval |= SIM_SCGC4_UART0;
#  endif
#  ifdef CONFIG_KINETIS_UART1
   regval |= SIM_SCGC4_UART1;
#  endif
#  ifdef CONFIG_KINETIS_UART2
   regval |= SIM_SCGC4_UART2;
#  endif
#  ifdef CONFIG_KINETIS_UART3
   regval |= SIM_SCGC4_UART3;
#  endif
   putreg32(regval, KINETIS_SIM_SCGC4);

#endif

  /* Clocking for UARTs 4-5 is enabled in the SCGC1 register. */

#if defined(CONFIG_KINETIS_UART4) || defined(CONFIG_KINETIS_UART5)

   regval = getreg32(KINETIS_SIM_SCGC4);
#  ifdef CONFIG_KINETIS_UART4
   regval |= SIM_SCGC1_UART4;
#  endif
#  ifdef CONFIG_KINETIS_UART5
   regval |= SIM_SCGC1_UART5;
#  endif
   putreg32(regval, KINETIS_SIM_SCGC4);

#endif

  /* Configure UART pins for the all enabled UARTs */

#ifdef CONFIG_KINETIS_UART0
  kinetis_pinconfig(PIN_UART0_TX);
  kinetis_pinconfig(PIN_UART0_RX);
#endif
#ifdef CONFIG_KINETIS_UART1
  kinetis_pinconfig(PIN_UART1_TX);
  kinetis_pinconfig(PIN_UART1_RX);
#endif
#ifdef CONFIG_KINETIS_UART2
  kinetis_pinconfig(PIN_UART2_TX);
  kinetis_pinconfig(PIN_UART2_RX);
#endif
#ifdef CONFIG_KINETIS_UART3
  kinetis_pinconfig(PIN_UART3_TX);
  kinetis_pinconfig(PIN_UART3_RX);
#endif
#ifdef CONFIG_KINETIS_UART4
  kinetis_pinconfig(PIN_UART4_TX);
  kinetis_pinconfig(PIN_UART4_RX);
#endif
#ifdef CONFIG_KINETIS_UART5
  kinetis_pinconfig(PIN_UART5_TX);
  kinetis_pinconfig(PIN_UART5_RX);
#endif

  /* Configure the console (only) now.  Other UARTs will be configured
   * when the serial driver is opened.
   */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  kinetis_uartconfigure(CONSOLE_BASE, CONSOLE_BAUD, CONSOLE_FREQ,
                        CONSOLE_PARITY, CONSOLE_BITS);
#endif
#endif /* HAVE_UART_DEVICE */
}

/******************************************************************************
 * Name: kinetis_uartreset
 *
 * Description:
 *   Reset a UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartreset(uintptr_t uart_base)
{
  uint8_t regval;

  /* Just disable the transmitter and receiver */

  regval = getreg8(uart_base+KINETIS_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE|UART_C2_TE);
  putreg8(regval, uart_base+KINETIS_UART_C2_OFFSET);
}
#endif

/******************************************************************************
 * Name: kinetis_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void kinetis_uartconfigure(uintptr_t uart_base, uint32_t baud,
                           uint32_t clock, unsigned int parity,
                           unsigned int nbits)
{
  uint32_t sbr;
  uint32_t brfa;
  uint32_t tmp;
  uint8_t  regval;

  /* Disable the transmitter and receiver throughout the reconfiguration */

  regval = getreg8(uart_base+KINETIS_UART_C2_OFFSET);
  regval &= ~(UART_C2_RE|UART_C2_TE);
  putreg8(regval, uart_base+KINETIS_UART_C2_OFFSET);

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

  putreg8(regval, uart_base+KINETIS_UART_C1_OFFSET);

  /* Calculate baud settings (truncating) */

  sbr = clock / (baud << 4);
  DEBUGASSERT(sbr < 0x2000);
        
  /* Save the new baud divisor, retaining other bits in the UARTx_BDH
   * register.
   */

  regval  = getreg8(uart_base+KINETIS_UART_BDH_OFFSET) & UART_BDH_SBR_MASK;
  tmp     = sbr >> 8;
  regval |= (((uint8_t)tmp) << UART_BDH_SBR_SHIFT) & UART_BDH_SBR_MASK;
  putreg8(regval, uart_base+KINETIS_UART_BDH_OFFSET);

  tmp    = sbr & 0xff;
  putreg8(regval, uart_base+KINETIS_UART_BDL_OFFSET);
    
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

  regval  = getreg8(uart_base+KINETIS_UART_C4_OFFSET) & UART_C4_BRFA_MASK;
  regval |= ((uint8_t)brfa << UART_C4_BRFA_SHIFT) & UART_C4_BRFA_MASK;
  putreg8(regval, uart_base+KINETIS_UART_C4_OFFSET);

  /* Now we can re-enable the transmitter and receiver */

  regval = getreg8(uart_base+KINETIS_UART_C2_OFFSET);
  regval |= (UART_C2_RE|UART_C2_TE);
  putreg8(regval, uart_base+KINETIS_UART_C2_OFFSET);
}
#endif



