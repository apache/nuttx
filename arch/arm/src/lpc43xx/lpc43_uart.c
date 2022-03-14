/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_uart.c
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc43_config.h"
#include "lpc43_pinconfig.h"
#include "lpc43_rgu.h"
#include "lpc43_cgu.h"
#include "lpc43_ccu.h"

#include "lpc43_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
  /* Select UART parameters for the selected console */

#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_BASE     LPC43_USART0_BASE
#    define CONSOLE_BASEFREQ BOARD_USART0_BASEFREQ
#    define CONSOLE_BAUD     CONFIG_USART0_BAUD
#    define CONSOLE_BITS     CONFIG_USART0_BITS
#    define CONSOLE_PARITY   CONFIG_USART0_PARITY
#    define CONSOLE_2STOP    CONFIG_USART0_2STOP
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE     LPC43_UART1_BASE
#    define CONSOLE_BASEFREQ BOARD_UART1_BASEFREQ
#    define CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define CONSOLE_BITS     CONFIG_UART1_BITS
#    define CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_BASE     LPC43_USART2_BASE
#    define CONSOLE_BASEFREQ BOARD_USART2_BASEFREQ
#    define CONSOLE_BAUD     CONFIG_USART2_BAUD
#    define CONSOLE_BITS     CONFIG_USART2_BITS
#    define CONSOLE_PARITY   CONFIG_USART2_PARITY
#    define CONSOLE_2STOP    CONFIG_USART2_2STOP
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_BASE     LPC43_USART3_BASE
#    define CONSOLE_BASEFREQ BOARD_USART3_BASEFREQ
#    define CONSOLE_BAUD     CONFIG_USART3_BAUD
#    define CONSOLE_BITS     CONFIG_USART3_BITS
#    define CONSOLE_PARITY   CONFIG_USART3_PARITY
#    define CONSOLE_2STOP    CONFIG_USART3_2STOP
#  elif defined(HAVE_SERIAL_CONSOLE)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif

  /* Get word length setting for the console */

#  if CONSOLE_BITS == 5
#    define CONSOLE_LCR_WLS UART_LCR_WLS_5BIT
#  elif CONSOLE_BITS == 6
#    define CONSOLE_LCR_WLS UART_LCR_WLS_6BIT
#  elif CONSOLE_BITS == 7
#    define CONSOLE_LCR_WLS UART_LCR_WLS_7BIT
#  elif CONSOLE_BITS == 8
#    define CONSOLE_LCR_WLS UART_LCR_WLS_8BIT
#  elif defined(HAVE_SERIAL_CONSOLE)
#    error "Invalid CONFIG_UARTn_BITS setting for console "
#  endif

  /* Get parity setting for the console */

#  if CONSOLE_PARITY == 0
#    define CONSOLE_LCR_PAR 0
#  elif CONSOLE_PARITY == 1
#    define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#  elif CONSOLE_PARITY == 2
#    define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#  elif CONSOLE_PARITY == 3
#    define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#  elif CONSOLE_PARITY == 4
#    define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#  elif defined(HAVE_SERIAL_CONSOLE)
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#  endif

  /* Get stop-bit setting for the console and USART0/2/3, UART1 */

#  if CONSOLE_2STOP != 0
#    define CONSOLE_LCR_STOP UART_LCR_STOP
#  else
#    define CONSOLE_LCR_STOP 0
#  endif

  /* LCR and FCR values for the console */

#  define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | \
                             CONSOLE_LCR_STOP)
#  define CONSOLE_FCR_VALUE (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                             UART_FCR_RXRST | UART_FCR_FIFOEN)

#endif

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
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE + LPC43_UART_LSR_OFFSET) &
          UART_LSR_THRE) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + LPC43_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: lpc43_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 *   The USART0/2/3 and UART1 peripherals are configured using the following
 *   registers:
 *
 *   1. Baud rate: In the LCR register, set bit DLAB = 1. This enables access
 *      to registers DLL and DLM for setting the baud rate. Also, if needed,
 *      set the fractional baud rate in the fractional divider
 *   2. UART FIFO: Use bit FIFO enable (bit 0) in FCR register to
 *      enable FIFO.
 *   3. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *   4. Interrupts: To enable UART interrupts set bit DLAB = 0 in the LCRF
 *      register. This enables access to IER. Interrupts are enabled
 *      in the NVIC using the appropriate Interrupt Set Enable register.
 *   5. DMA: UART transmit and receive functions can operate with the
 *      GPDMA controller.
 *
 ****************************************************************************/

void lpc43_lowsetup(void)
{
#ifdef HAVE_UART
  /* Enable clocking and  for all console UART and disable power for
   * other UARTs
   */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
  lpc43_usart0_setup();
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  lpc43_uart1_setup();
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
  lpc43_usart2_setup();
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
  lpc43_usart3_setup();
#endif

  /* Configure the console (only) */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* Clear fifos */

  putreg32(UART_FCR_RXRST | UART_FCR_TXRST,
           CONSOLE_BASE + LPC43_UART_FCR_OFFSET);

  /* Set trigger */

  putreg32(UART_FCR_FIFOEN | UART_FCR_RXTRIGGER_8,
           CONSOLE_BASE + LPC43_UART_FCR_OFFSET);

  /* Set up the LCR */

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + LPC43_UART_LCR_OFFSET);

  /* Set the BAUD divisor */

  lpc43_setbaud(CONSOLE_BASE, CONSOLE_BASEFREQ, CONSOLE_BAUD);

  /* Configure the FIFOs */

  putreg32(UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST | UART_FCR_RXRST |
           UART_FCR_FIFOEN, CONSOLE_BASE + LPC43_UART_FCR_OFFSET);
#endif
#endif /* HAVE_UART */
}

/****************************************************************************
 * Name: lpc43_u[s]art0/1/2/3_reset
 *
 * Description:
 *   Reset a UART.  These functions are used by the serial driver when a
 *   UART is closed.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
void lpc43_usart0_reset(void)
{
  putreg32(RGU_CTRL1_USART0_RST, LPC43_RGU_CTRL1);
}
#endif

#ifdef CONFIG_LPC43_UART1
void lpc43_uart1_reset(void)
{
  putreg32(RGU_CTRL1_UART1_RST, LPC43_RGU_CTRL1);
}
#endif

#ifdef CONFIG_LPC43_USART2
void lpc43_usart2_reset(void)
{
  putreg32(RGU_CTRL1_USART2_RST, LPC43_RGU_CTRL1);
}
#endif

#ifdef CONFIG_LPC43_USART3
void lpc43_usart3_reset(void)
{
  putreg32(RGU_CTRL1_USART3_RST, LPC43_RGU_CTRL1);
}
#endif

/****************************************************************************
 * Name: lpc43_usart0_setup, lpc43_uart1_setup, lpc43_usart2_setup, and
 *       lpc43_usart3_setup
 *
 * Description:
 *   Configure the U[S]ART.  This involves:
 *
 *   1. Connecting the input clock to the U[S]ART as specified in the
 *      board.h file,
 *   2. Configuring the U[S]ART pins
 *
 * USART0/2/3 and UART1 clocking and power control:
 *
 *    ----------------------------------- -------------- --------------
 *                                        BASE CLOCK     BRANCH CLOCK
 *    ----------------------------------- -------------- --------------
 *    USART0 clock to register interface  BASE_M4_CLK    CLK_M4_USART0
 *    USART0 peripheral clock (PCLK)      BASE_UART0_CLK CLK_APB0_UART0
 *    UART1 clock to register interface   BASE_M4_CLK    CLK_M4_UART1
 *    UART1 peripheral clock (PCLK)       BASE_UART1_CLK CLK_APB0_UART1
 *    USART2 clock to register interface  BASE_M4_CLK    CLK_M4_USART2
 *    USART2 peripheral clock (PCLK)      BASE_UART2_CLK CLK_APB2_UART2
 *    USART3 clock to register interface  BASE_M4_CLK    CLK_M4_USART3
 *    USART3 peripheral clock (PCLK)      BASE_UART3_CLK CLK_APB2_UART3
 *    ----------------------------------- -------------- --------------
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_USART0
void lpc43_usart0_setup(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Connect USART0 into the clock source specified in board.h */

  flags   = enter_critical_section();

  regval  = getreg32(LPC43_BASE_USART0_CLK);
  regval &= ~BASE_USART0_CLK_CLKSEL_MASK;
  regval |= (BOARD_USART0_CLKSRC | BASE_USART0_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_USART0_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_USART0_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_USART0_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB0_USART0_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB0_USART0_CFG);

  /* Configure I/O pins.  NOTE that multiple pin configuration options must
   * be disambiguated by defining the pin configuration in the board.h
   * header file.
   */

  lpc43_pin_config(PINCONF_U0_TXD);
  lpc43_pin_config(PINCONF_U0_RXD);

  /* If USART RS-485 mode is selected, then configure the DIR pin as well.
   * NOTE, again, that multiple pin configuration options must be
   * disambiguated by defining the pin configuration in the board.h header
   * file.
   */

#ifdef CONFIG_USART0_RS485MODE
  lpc43_pin_config(PINCONF_U0_DIR);

  /* Enable direction output pin */

  regval = getreg32(LPC43_USART0_RS485CTRL);
  regval |= UART_RS485CTRL_DCTRL;
  putreg32(regval, LPC43_USART0_RS485CTRL);

#ifdef CONFIG_USART0_RS485DIROIN

  /* Invert direction control output pin polarity */

  regval = getreg32(LPC43_USART0_RS485CTRL);
  regval |= UART_RS485CTRL_OINV;
  putreg32(regval, LPC43_USART0_RS485CTRL);

#else

  /* Do not invert direction countrol output pin polarity */

  regval = getreg32(LPC43_USART0_RS485CTRL);
  regval &= ~(UART_RS485CTRL_OINV);
  putreg32(regval, LPC43_USART0_RS485CTRL);

#endif /* CONFIG_USART0_RS485DIROIN */
#endif /* CONFIG_USART0_RS485MODE */

  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC43_UART1
void lpc43_uart1_setup(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Connect UART1 into the clock source specified in board.h */

  flags   = enter_critical_section();

  regval  = getreg32(LPC43_BASE_UART1_CLK);
  regval &= ~BASE_UART1_CLK_CLKSEL_MASK;
  regval |= (BOARD_UART1_CLKSRC | BASE_UART1_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_UART1_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_UART1_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_UART1_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB0_UART1_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB0_UART1_CFG);

  /* Configure I/O pins.  NOTE that multiple pin configuration options must
   * be disambiguated by defining the pin configuration in the board.h
   * header file.
   */

  lpc43_pin_config(PINCONF_U1_TXD);
  lpc43_pin_config(PINCONF_U1_RXD);
#ifdef CONFIG_UART1_FLOWCONTROL
  lpc43_pin_config(PINCONF_U1_CTS);
  lpc43_pin_config(PINCONF_U1_DCD);
  lpc43_pin_config(PINCONF_U1_DSR);
  lpc43_pin_config(PINCONF_U1_DTR);
  lpc43_pin_config(PINCONF_U1_RTS);
#ifdef CONFIG_LPC43_UART1_RINGINDICATOR
  lpc43_pin_config(PINCONF_U1_RI);
#endif
#endif

#ifdef CONFIG_UART1_RS485MODE
  lpc43_pin_config(PINCONF_U1_DIR);
#endif

  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC43_USART2
void lpc43_usart2_setup(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Connect USART2 the clock source specified in board.h */

  flags   = enter_critical_section();

  regval  = getreg32(LPC43_BASE_USART2_CLK);
  regval &= ~BASE_USART2_CLK_CLKSEL_MASK;
  regval |= (BOARD_USART2_CLKSRC | BASE_USART2_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_USART2_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_USART2_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_USART2_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB2_USART2_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB2_USART2_CFG);

  /* Configure I/O pins.  NOTE that multiple pin configuration options must
   * be disambiguated by defining the pin configuration in the board.h
   * header file.
   */

  lpc43_pin_config(PINCONF_U2_TXD);
  lpc43_pin_config(PINCONF_U2_RXD);

  /* If USART RS-485 mode is selected, then configure the DIR pin as well.
   * NOTE, again, that multiple pin configuration options must be
   * disambiguated by defining the pin configuration in the board.h header
   * file.
   */

#ifdef CONFIG_USART2_RS485MODE
  lpc43_pin_config(PINCONF_U2_DIR);

  /* Enable direction output pin */

  regval = getreg32(LPC43_USART2_RS485CTRL);
  regval |= UART_RS485CTRL_DCTRL;
  putreg32(regval, LPC43_USART2_RS485CTRL);

#ifdef CONFIG_USART2_RS485DIROIN

  /* Invert direction control output pin polarity */

  regval = getreg32(LPC43_USART2_RS485CTRL);
  regval |= UART_RS485CTRL_OINV;
  putreg32(regval, LPC43_USART2_RS485CTRL);

#else

  /* Do not invert direction countrol output pin polarity */

  regval = getreg32(LPC43_USART2_RS485CTRL);
  regval &= ~(UART_RS485CTRL_OINV);
  putreg32(regval, LPC43_USART2_RS485CTRL);

#endif /* CONFIG_USART2_RS485DIROIN */
#endif /* CONFIG_USART2_RS485MODE */

  leave_critical_section(flags);
};
#endif

#ifdef CONFIG_LPC43_USART3
void lpc43_usart3_setup(void)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Connect USART3 into the clock source specified in board.h */

  flags   = enter_critical_section();

  regval  = getreg32(LPC43_BASE_USART3_CLK);
  regval &= ~BASE_USART3_CLK_CLKSEL_MASK;
  regval |= (BOARD_USART3_CLKSRC | BASE_USART3_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_USART3_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_USART3_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_USART3_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB2_USART3_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB2_USART3_CFG);

  /* Configure I/O pins.  NOTE that multiple pin configuration options must
   * be disambiguated by defining the pin configuration in the board.h
   * header file.
   */

  lpc43_pin_config(PINCONF_U3_TXD);
  lpc43_pin_config(PINCONF_U3_RXD);

  /* If USART RS-485 mode is selected, then configure the DIR pin as well.
   * NOTE, again, that multiple pin configuration options must be
   * disambiguated by defining the pin configuration in the board.h header
   * file.
   */

#ifdef CONFIG_USART3_RS485MODE
  lpc43_pin_config(PINCONF_U3_DIR);

  /* Enable direction output pin */

  regval = getreg32(LPC43_USART3_RS485CTRL);
  regval |= UART_RS485CTRL_DCTRL;
  putreg32(regval, LPC43_USART3_RS485CTRL);

#ifdef CONFIG_USART3_RS485DIROIN

  /* Invert direction control output pin polarity */

  regval = getreg32(LPC43_USART3_RS485CTRL);
  regval |= UART_RS485CTRL_OINV;
  putreg32(regval, LPC43_USART3_RS485CTRL);

#else

  /* Do not invert direction countrol output pin polarity */

  regval = getreg32(LPC43_USART3_RS485CTRL);
  regval &= ~(UART_RS485CTRL_OINV);
  putreg32(regval, LPC43_USART3_RS485CTRL);

#endif /* CONFIG_USART3_RS485DIROIN */
#endif /* CONFIG_USART3_RS485MODE */

  leave_critical_section(flags);
};
#endif

/****************************************************************************
 * Name: lpc43_setbaud
 *
 * Description:
 *   Configure the U[S]ART divisors to accomplish the desired BAUD given the
 *   U[S]ART base frequency.
 *
 *   This computationally intensive algorithm is based on the same logic
 *   used in the NXP sample code.
 *
 ****************************************************************************/

void lpc43_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud)
{
  uint32_t lcr;      /* Line control register value */
  uint32_t dl;       /* Best DLM/DLL full value */
  uint32_t mul;      /* Best FDR MULVALL value */
  uint32_t divadd;   /* Best FDR DIVADDVAL value */
  uint32_t best;     /* Error value associated with best {dl, mul, divadd} */
  uint32_t cdl;      /* Candidate DLM/DLL full value */
  uint32_t cmul;     /* Candidate FDR MULVALL value */
  uint32_t cdivadd;  /* Candidate FDR DIVADDVAL value */
  uint32_t errval;   /* Error value associated with the candidate */

  /* The U[S]ART baud is given by:
   *
   * Fbaud =  Fbase * mul / (mul + divadd) / (16 * dl)
   * dl    =  Fbase * mul / (mul + divadd) / Fbaud / 16
   *       =  Fbase * mul / ((mul + divadd) * Fbaud * 16)
   *       = ((Fbase * mul) >> 4) / ((mul + divadd) * Fbaud)
   *
   * Where the  value of MULVAL and DIVADDVAL comply with:
   *
   *  0 < mul < 16
   *  0 <= divadd < mul
   */

  best   = UINT32_MAX;
  divadd = 0;
  mul    = 0;
  dl     = 0;

  /* Try each multiplier value in the valid range */

  for (cmul = 1 ; cmul < 16; cmul++)
    {
      /* Try each divider value in the valid range */

      for (cdivadd = 0 ; cdivadd < cmul ; cdivadd++)
        {
          /* Candidate:
           *   dl         = ((Fbase * mul) >> 4) / ((mul + cdivadd) * Fbaud)
           *   (dl << 32) = (Fbase << 28) * cmul / ((mul + cdivadd) * Fbaud)
           */

          uint64_t dl64 = ((uint64_t)basefreq << 28) * cmul /
                          ((cmul + cdivadd) * baud);

          /* The lower 32-bits of this value is the error */

          errval = (uint32_t)(dl64 & 0x00000000ffffffffull);

          /* The upper 32-bits is the candidate DL value */

          cdl = (uint32_t)(dl64 >> 32);

          /* Round up */

          if (errval > (1 << 31))
            {
              errval = -errval;
              cdl++;
            }

          /* Check if the resulting candidate DL value is within range */

          if (cdl < 1 || cdl > 65536)
            {
              /* No... try a different divadd value */

              continue;
            }

          /* Is this the best combination that we have seen so far? */

          if (errval < best)
            {
              /* Yes.. then the candidate is out best guess so far */

              best   = errval;
              dl     = cdl;
              divadd = cdivadd;
              mul    = cmul;

              /* If the new best guess is exact (within our precision), then
               * we are finished.
               */

              if (best == 0)
                {
                  break;
                }
            }
        }
    }

  DEBUGASSERT(dl > 0);

  /* Enter DLAB=1 */

  lcr = getreg32(uartbase + LPC43_UART_LCR_OFFSET);
  putreg32(lcr | UART_LCR_DLAB, uartbase + LPC43_UART_LCR_OFFSET);

  /* Save the divider values */

  putreg32(dl >> 8, uartbase + LPC43_UART_DLM_OFFSET);
  putreg32(dl & 0xff, uartbase + LPC43_UART_DLL_OFFSET);

  /* Clear DLAB */

  putreg32(lcr & ~UART_LCR_DLAB, uartbase + LPC43_UART_LCR_OFFSET);

  /* Then save the fractional divider values */

  putreg32((mul << UART_FDR_MULVAL_SHIFT) |
           (divadd << UART_FDR_DIVADDVAL_SHIFT),
            uartbase + LPC43_UART_FDR_OFFSET);
}
