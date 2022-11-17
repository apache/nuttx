/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_lowconsole.c
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

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "mips_internal.h"
#include "pic32mz_config.h"
#include "hardware/pic32mz_uart.h"
#include "hardware/pic32mz_pps.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART1_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART1_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART2_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART2_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART3_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART3_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART3_2STOP
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART4_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART4_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART4_2STOP
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART5_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART5_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART5_2STOP
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define PIC32MZ_CONSOLE_BASE     PIC32MZ_UART6_K1BASE
#    define PIC32MZ_CONSOLE_BAUD     CONFIG_UART6_BAUD
#    define PIC32MZ_CONSOLE_BITS     CONFIG_UART6_BITS
#    define PIC32MZ_CONSOLE_PARITY   CONFIG_UART6_PARITY
#    define PIC32MZ_CONSOLE_2STOP    CONFIG_UART6_2STOP
#  else
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif

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
 * Name: pic32mz_putreg
 *
 * Description:
 *   Write a value to a UART register
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline void pic32mz_putreg(uintptr_t uart_base, unsigned int offset,
                                      uint32_t value)
{
  putreg32(value, uart_base + offset);
}
#endif

/****************************************************************************
 * Name: pic32mz_getreg
 *
 * Description:
 *   Get a value from a UART register
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline uint32_t pic32mz_getreg(uintptr_t uart_base,
                                          unsigned int offset)
{
  return getreg32(uart_base + offset);
}
#endif

/****************************************************************************
 * Name: pic32mz_uartsetbaud
 *
 * Description:
 *   Configure the UART baud rate.
 *
 *   With BRGH=0
 *     BAUD = PBCLK2 / 16 / (BRG+1)
 *     BRG  = PBCLK2 / 16 / BAUD - 1
 *   With BRGH=1
 *     BAUD = PBCLK2 / 4 / (BRG+1)
 *     BRG  = PBCLK2 / 4 / BAUD - 1
 *
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
static void pic32mz_uartsetbaud(uintptr_t uart_base, uint32_t baudrate)
{
  uint32_t tmp;
  uint32_t brg;
  unsigned int mode;

  /* We want the largest value of BRG divisor possible
   * (for the best accuracy)
   * Subject to BRG <= 65536.
   */

  tmp = BOARD_PBCLK2 / baudrate;

  /* Try BRGH=1 first.  This will select the 4x divisor and will produce the
   * larger BRG divisor, given all other things equal.
   */

  brg  = (tmp + 2) >> 2;
  mode = PIC32MZ_UART_MODESET_OFFSET;

  if (brg > 65536)
    {
      /* Nope, too big.. try BRGH=0 */

      brg  = (tmp + 8) >> 4;
      mode = PIC32MZ_UART_MODECLR_OFFSET;
    }

  DEBUGASSERT(brg <= 65536);

  /* Set the BRG divisor */

  pic32mz_putreg(uart_base, mode, UART_MODE_BRGH);
  pic32mz_putreg(uart_base, PIC32MZ_UART_BRG_OFFSET, brg);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_uartreset
 *
 * Description:
 *   Reset hardware and disable Rx and Tx.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mz_uartreset(uintptr_t uart_base)
{
  /* Doesn't reset the hardware... just shuts it down */

  pic32mz_putreg(uart_base, PIC32MZ_UART_STACLR_OFFSET,
                 UART_STA_UTXEN | UART_STA_URXEN);
  pic32mz_putreg(uart_base, PIC32MZ_UART_MODECLR_OFFSET, UART_MODE_ON);
}
#endif

/****************************************************************************
 * Name: pic32mz_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mz_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                           unsigned int parity, unsigned int nbits,
                           bool stop2)
{
  /* Clear mode and sta bits */

  pic32mz_putreg(uart_base, PIC32MZ_UART_MODECLR_OFFSET,
                 UART_MODE_STSEL    | UART_MODE_PDSEL_MASK |
                 UART_MODE_BRGH     | UART_MODE_RXINV      |
                 UART_MODE_WAKE     | UART_MODE_LPBACK     |
                 UART_MODE_UEN_MASK | UART_MODE_RTSMD      |
                 UART_MODE_IREN     | UART_MODE_SIDL       | UART_MODE_ON);

  /* Configure the FIFOs:
   *
   *   RX: Interrupt at 75% FIFO full (6 of 8 for 8-deep FIFO)
   *   TX: Interrupt on FIFO empty
   *   Invert transmit polarity.
   *
   * NOTE that there are not many options on trigger TX interrupts.
   * The FIFO not full might generate better through-put but with a higher
   * interrupt rate.  FIFO empty should lower the interrupt rate but result
   * in a burstier output.  If you change this, please read the comment for
   * acknowledging the interrupt in pic32mz_serial.c
   */

  pic32mz_putreg(uart_base, PIC32MZ_UART_STACLR_OFFSET,
                 UART_STA_UTXINV | UART_STA_UTXISEL_TXBE |
                 UART_STA_URXISEL_RXB75);

  /* Configure the FIFO interrupts */

  pic32mz_putreg(uart_base, PIC32MZ_UART_STASET_OFFSET,
                 UART_STA_UTXISEL_TXBNF  | UART_STA_URXISEL_RECVD);

  /* Configure word size and parity */

  if (nbits == 9)
    {
      DEBUGASSERT(parity == 0);
      pic32mz_putreg(uart_base, PIC32MZ_UART_MODESET_OFFSET,
                     UART_MODE_PDSEL_9NONE);
    }
  else
    {
      DEBUGASSERT(nbits == 8);
      if (parity == 1)
        {
          pic32mz_putreg(uart_base, PIC32MZ_UART_MODESET_OFFSET,
                         UART_MODE_PDSEL_8ODD);
        }
      else if (parity == 2)
        {
          pic32mz_putreg(uart_base, PIC32MZ_UART_MODESET_OFFSET,
                         UART_MODE_PDSEL_8EVEN);
        }
    }

  /* Configure 1 or 2 stop bits */

  if (stop2)
    {
      pic32mz_putreg(uart_base, PIC32MZ_UART_MODESET_OFFSET,
                     UART_MODE_STSEL);
    }

  /* Set the BRG divisor */

  pic32mz_uartsetbaud(uart_base, baudrate);

  /* Enable the UART */

  pic32mz_putreg(uart_base, PIC32MZ_UART_STASET_OFFSET,
                 UART_STA_UTXEN | UART_STA_URXEN);
  pic32mz_putreg(uart_base, PIC32MZ_UART_MODESET_OFFSET,
                 UART_MODE_ON);
}
#endif

/****************************************************************************
 * Name: pic32mz_consoleinit
 *
 * Description:
 *   Initialize a low-level console for debug output.  This function is
 *   called very early in the initialization sequence to configure the serial
 *   console UART (only).
 *
 ****************************************************************************/

void pic32mz_consoleinit(void)
{
#ifdef HAVE_UART_DEVICE

  /* Setup up pin selection registers for all configured UARTs.  The board.h
   * header file must provide these definitions to select the correct pin
   * configuration for each enabled UARt.
   */

#ifdef CONFIG_PIC32MZ_UART1
  /* Configure UART1 RX (input) and TX (output) pins */

  putreg32(BOARD_U1RX_PPS, PIC32MZ_U1RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U1TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U1TX_PPS));

#ifdef CONFIG_UART1_OFLOWCONTROL
  /* Configure the UART1 CTS input pin */

  putreg32(BOARD_U1CTS_PPS, PIC32MZ_U1CTSR);
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  /* Configure the UART1 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U1RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U1RTS_PPS));

#endif /* CONFIG_UART1_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART1 */

#ifdef CONFIG_PIC32MZ_UART2
  /* Configure UART2 RX (input) and TX (output) pins */

  putreg32(BOARD_U2RX_PPS, PIC32MZ_U2RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U2TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U2TX_PPS));

#ifdef CONFIG_UART2_OFLOWCONTROL
  /* Configure the UART2 CTS input pin */

  putreg32(BOARD_U2CTS_PPS, PIC32MZ_U2CTSR);
#endif
#ifdef CONFIG_UART2_IFLOWCONTROL
  /* Configure the UART2 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U2RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U2RTS_PPS));

#endif /* CONFIG_UART2_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART2 */

#ifdef CONFIG_PIC32MZ_UART3
  /* Configure UART3 RX (input) and TX (output) pins */

  putreg32(BOARD_U3RX_PPS, PIC32MZ_U3RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U3TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U3TX_PPS));

#ifdef CONFIG_UART3_OFLOWCONTROL
  /* Configure the UART3 CTS input pin */

  putreg32(BOARD_U3CTS_PPS, PIC32MZ_U3CTSR);
#endif
#ifdef CONFIG_UART3_IFLOWCONTROL
  /* Configure the UART3 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U3RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U3RTS_PPS));

#endif /* CONFIG_UART3_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART3 */

#ifdef CONFIG_PIC32MZ_UART4
  /* Configure UART4 RX (input) and TX (output) pins */

  putreg32(BOARD_U4RX_PPS, PIC32MZ_U4RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U4TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U4TX_PPS));

#ifdef CONFIG_UART4_OFLOWCONTROL
  /* Configure the UART4 CTS input pin */

  putreg32(BOARD_U4CTS_PPS, PIC32MZ_U4CTSR);
#endif
#ifdef CONFIG_UART4_IFLOWCONTROL
  /* Configure the UART4 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U4RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U4RTS_PPS));

#endif /* CONFIG_UART4_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART4 */

#ifdef CONFIG_PIC32MZ_UART5
  /* Configure UART5 RX (input) and TX (output) pins */

  putreg32(BOARD_U5RX_PPS, PIC32MZ_U5RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U5TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U5TX_PPS));

#ifdef CONFIG_UART5_OFLOWCONTROL
  /* Configure the UART5 CTS input pin */

  putreg32(BOARD_U5CTS_PPS, PIC32MZ_U5CTSR);
#endif
#ifdef CONFIG_UART5_IFLOWCONTROL
  /* Configure the UART5 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U5RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U5RTS_PPS));

#endif /* CONFIG_UART5_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART5 */

#ifdef CONFIG_PIC32MZ_UART6
  /* Configure UART6 RX (input) and TX (output) pins */

  putreg32(BOARD_U6RX_PPS, PIC32MZ_U6RXR);
  putreg32(PPS_OUTPUT_REGVAL(BOARD_U6TX_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U6TX_PPS));

#ifdef CONFIG_UART6_OFLOWCONTROL
  /* Configure the UART6 CTS input pin */

  putreg32(BOARD_U6CTS_PPS, PIC32MZ_U6CTSR);
#endif
#ifdef CONFIG_UART6_IFLOWCONTROL
  /* Configure the UART6 RTS output pin */

  putreg32(PPS_OUTPUT_REGVAL(BOARD_U6RTS_PPS),
           PPS_OUTPUT_REGADDR(BOARD_U6RTS_PPS));

#endif /* CONFIG_UART6_IFLOWCONTROL */
#endif /* CONFIG_PIC32MZ_UART6 */

#ifdef HAVE_SERIAL_CONSOLE
  /* Configure the console uart */

  pic32mz_uartconfigure(PIC32MZ_CONSOLE_BASE, PIC32MZ_CONSOLE_BAUD,
                        PIC32MZ_CONSOLE_PARITY, PIC32MZ_CONSOLE_BITS,
                        PIC32MZ_CONSOLE_2STOP);

#endif /* HAVE_SERIAL_CONSOLE */
#endif /* HAVE_UART_DEVICE */
}

/****************************************************************************
 * Name: mips_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 ****************************************************************************/

void mips_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait for the transmit buffer not full */

  while ((pic32mz_getreg(PIC32MZ_CONSOLE_BASE, PIC32MZ_UART_STA_OFFSET) &
          UART_STA_UTXBF) != 0);

  /* Then write the character to the TX data register */

  pic32mz_putreg(PIC32MZ_CONSOLE_BASE, PIC32MZ_UART_TXREG_OFFSET,
                 (uint32_t)ch);

  while ((pic32mz_getreg(PIC32MZ_CONSOLE_BASE, PIC32MZ_UART_STA_OFFSET) &
          UART_STA_UTRMT) == 0);
#endif
}
