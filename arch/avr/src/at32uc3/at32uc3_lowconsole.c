/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_lowconsole.c
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
#include "at32uc3_config.h"

#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "avr_internal.h"
#include "at32uc3.h"
#include "at32uc3_pm.h"
#include "at32uc3_usart.h"
#include "at32uc3_pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select USART parameters for the selected console */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define AVR32_CONSOLE_BASE     AVR32_USART0_BASE
#  define AVR32_CONSOLE_BAUD     CONFIG_USART0_BAUD
#  define AVR32_CONSOLE_BITS     CONFIG_USART0_BITS
#  define AVR32_CONSOLE_PARITY   CONFIG_USART0_PARITY
#  define AVR32_CONSOLE_2STOP    CONFIG_USART0_2STOP
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define AVR32_CONSOLE_BASE     AVR32_USART1_BASE
#  define AVR32_CONSOLE_BAUD     CONFIG_USART1_BAUD
#  define AVR32_CONSOLE_BITS     CONFIG_USART1_BITS
#  define AVR32_CONSOLE_PARITY   CONFIG_USART1_PARITY
#  define AVR32_CONSOLE_2STOP    CONFIG_USART1_2STOP
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define AVR32_CONSOLE_BASE     AVR32_USART2_BASE
#  define AVR32_CONSOLE_BAUD     CONFIG_USART2_BAUD
#  define AVR32_CONSOLE_BITS     CONFIG_USART2_BITS
#  define AVR32_CONSOLE_PARITY   CONFIG_USART2_PARITY
#  define AVR32_CONSOLE_2STOP    CONFIG_USART2_2STOP
#else
#  error "No CONFIG_USARTn_SERIAL_CONSOLE Setting"
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
 * Name: usart_putreg
 *
 * Description:
 *   Write a value to a USART register
 *
 ****************************************************************************/

#ifdef HAVE_RS232_DEVICE
static inline void usart_putreg(uintptr_t usart_base,
                                unsigned int offset, uint32_t value)
{
  putreg32(value, usart_base + offset);
}
#endif

/****************************************************************************
 * Name: usart_getreg
 *
 * Description:
 *   Get a value from a USART register
 *
 ****************************************************************************/

#ifdef HAVE_RS232_DEVICE
static inline uint32_t usart_getreg(uintptr_t usart_base,
                                    unsigned int offset)
{
  return getreg32(usart_base + offset);
}
#endif

/****************************************************************************
 * Name: usart_setbaudrate
 *
 * Description:
 *   Configure the UART baud rate.
 *
 ****************************************************************************/

#ifdef HAVE_RS232_DEVICE
static void usart_setbaudrate(uintptr_t usart_base, uint32_t baudrate)
{
  uint32_t cd;
  uint32_t mr;

  /* Select 16x or 8x oversampling mode or go to synchronous mode */

  mr = usart_getreg(usart_base, AVR32_USART_MR_OFFSET);
  if (baudrate < AVR32_PBA_CLOCK / 16)
    {
      /* Select 16x oversampling mode and clear the SYNC mode bit */

      mr &= ~(USART_MR_OVER | USART_MR_SYNC);

      /* Calculate the clock divider assuming 16x oversampling */

      cd = (AVR32_PBA_CLOCK + (baudrate << 3)) / (baudrate << 4);
    }
  else if (baudrate < AVR32_PBA_CLOCK / 8)
    {
      /* Select 8x oversampling mode and clear the SYNC mode bit */

      mr &= ~USART_MR_SYNC;
      mr |= USART_MR_OVER;

      /* Calculate the clock divider assuming 16x oversampling */

      cd = (AVR32_PBA_CLOCK + (baudrate << 2)) / (baudrate << 3);
    }
  else
    {
      /* Set the SYNC mode bit */

      mr |= USART_MR_SYNC;

      /* Use the undivided PBA clock */

      cd = AVR32_PBA_CLOCK / baudrate;
    }

  DEBUGASSERT(cd > 0 && cd < 65536);
  usart_putreg(usart_base, AVR32_USART_MR_OFFSET, mr);
  usart_putreg(usart_base, AVR32_USART_BRGR_OFFSET, cd);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usart_reset
 *
 * Description:
 *   Reset a USART.
 *
 ****************************************************************************/

#ifdef HAVE_RS232_DEVICE
void usart_reset(uintptr_t usart_base)
{
  irqstate_t flags;

  /* Disable all USART interrupts */

  flags = enter_critical_section();
  usart_putreg(usart_base, AVR32_USART_IDR_OFFSET, 0xffffffff);
  leave_critical_section(flags);

  /* Reset mode and other registers */

  usart_putreg(usart_base, AVR32_USART_MR_OFFSET, 0);   /* Reset mode register */
  usart_putreg(usart_base, AVR32_USART_RTOR_OFFSET, 0); /* Reset receiver timeout register */
  usart_putreg(usart_base, AVR32_USART_TTGR_OFFSET, 0); /* Reset transmitter timeguard reg */

  /* Disable RX and TX, put USART in reset, disable handshaking signals */

  usart_putreg(usart_base, AVR32_USART_CR_OFFSET,
               USART_CR_RSTRX | USART_CR_RSTTX | USART_CR_RSTSTA |
               USART_CR_RSTIT | USART_CR_RSTNACK | USART_CR_DTRDIS |
               USART_CR_RTSDIS);
}
#endif

/****************************************************************************
 * Name: usart_configure
 *
 * Description:
 *   Configure a USART as a RS-232 UART.
 *
 ****************************************************************************/

#ifdef HAVE_RS232_DEVICE
void usart_configure(uintptr_t usart_base,
                     uint32_t baud, unsigned int parity,
                     unsigned int nbits, bool stop2)
{
  uint32_t regval;

  /* Reset the USART and disable RX and TX */

  usart_reset(usart_base);

  /* Configure STOP bits */

  regval  = USART_MR_MODE_NORMAL | USART_MR_CHMODE_NORMAL;  /* Normal RS-232 mode */
  regval |= stop2 ? USART_MR_NBSTOP_2 : USART_MR_NBSTOP_1;

  /* Configure parity */

  switch (parity)
    {
      case 0:
      default:
        regval |= USART_MR_PAR_NONE;
        break;

      case 1:
        regval |= USART_MR_PAR_ODD;
        break;

      case 2:
        regval |= USART_MR_PAR_EVEN;
        break;
    }

  /* Configure the number of bits per word */

  DEBUGASSERT(nbits >= 5 && nbits <= 9);
  if (nbits == 9)
    {
      regval |= USART_MR_MODE9;
    }
  else
    {
      regval |= USART_MR_CHRL_BITS(nbits);
    }

  usart_putreg(usart_base, AVR32_USART_MR_OFFSET, regval);

  /* Set the baud rate generation register */

  usart_setbaudrate(usart_base, baud);

  /* Enable RX and TX */

  regval = usart_getreg(usart_base, AVR32_USART_CR_OFFSET);
  regval |= (USART_CR_RXEN | USART_CR_TXEN);
  usart_putreg(usart_base, AVR32_USART_CR_OFFSET, regval);
}
#endif

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   uart (only).
 *
 ****************************************************************************/

void up_consoleinit(void)
{
  uint32_t pbamask = 0;
  uint32_t regval;

  /* Setup GPIO pins fand enable module clocking or each configured
   * USART/UART
   */

#ifdef CONFIG_AVR32_USART0_RS232
  /* PINMUX_USART0_RXD and PINMUX_USART0_TXD must be defined in board.h.
   * It must define them be be one of {PINMUX_USART0_RXD_1,
   * PINMUX_USART0_RXD_2} and {PINMUX_USART_0TXD_1, PINMUX_USART0_TXD_2},
   * respectively.
   */

  at32uc3_configgpio(PINMUX_USART0_RXD);
  at32uc3_configgpio(PINMUX_USART0_TXD);

  /* Enable clocking to USART0
   * (This should be the default state after reset)
   */

  pbamask |= PM_PBAMASK_USART0;

#endif
#ifdef CONFIG_AVR32_USART1_RS232
  /* PINMUX_USART1_RXD and PINMUX_USART1_TXD must be defined in board.h.  It
   * must define them be be one of {PINMUX_USART1_RXD_1, PINMUX_USART1_RXD_2,
   * PINMUX_USART1_RXD_3} and {PINMUX_USART1_TXD_1, PINMUX_USART1_TXD_2,
   * PINMUX_USART1_TXD_3}, respectively.
   */

  at32uc3_configgpio(PINMUX_USART1_RXD);
  at32uc3_configgpio(PINMUX_USART1_TXD);

  /* Enable clocking to USART1
   * (This should be the default state after reset)
   */

  pbamask |= PM_PBAMASK_USART1;

#endif
#ifdef CONFIG_AVR32_USART2_RS232
  /* PINMUX_USART2_RXD and PINMUX_USART2_TXD must be defined in board.h.  It
   * must define them be be one of {PINMUX_USART2_RXD_1, PINMUX_USART2_RXD_2}
   * and {PINMUX_USART2_TXD_1, PINMUX_USART2_TXD_2}, respectively.
   */

  at32uc3_configgpio(PINMUX_USART2_RXD);
  at32uc3_configgpio(PINMUX_USART2_TXD);

  /* Enable clocking to USART2
   * (This should be the default state after reset)
   */

  pbamask |= PM_PBAMASK_USART2;

#endif

  /* Enable selected clocks (and disabled unselected clocks) */

  regval = getreg32(AVR32_PM_PBAMASK);
  regval &= ~(PM_PBAMASK_USART0 | PM_PBAMASK_USART1 | PM_PBAMASK_USART2);
  regval |= pbamask;
  putreg32(regval, AVR32_PM_PBAMASK);

  /* Then configure the console here (if it is not going to be configured
   * by avr_earlyserialinit()).
   */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(USE_EARLYSERIALINIT)
  usart_configure(AVR32_CONSOLE_BASE, AVR32_CONSOLE_BAUD,
                  AVR32_CONSOLE_PARITY, AVR32_CONSOLE_BITS,
                 (bool)AVR32_CONSOLE_2STOP);
#endif
}

/****************************************************************************
 * Name: avr_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void avr_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX to become ready */

  while ((usart_getreg(AVR32_CONSOLE_BASE,
                       AVR32_USART_CSR_OFFSET) & USART_CSR_TXRDY) == 0);

  /* Then send the character */

  usart_putreg(AVR32_CONSOLE_BASE, AVR32_USART_THR_OFFSET, (uint32_t)ch);
#endif
}
