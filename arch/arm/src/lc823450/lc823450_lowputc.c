/****************************************************************************
 * arch/arm/src/lc823450/lc823450_lowputc.c
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "lc823450_serial.h"
#include "lc823450_syscontrol.h"

#include <arch/board/board.h>

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Configuration **********************************************************/

/* Select UART parameters for the selected console */

#define CTL_CLK XT1OSC_CLK

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define LC823450_CONSOLE_BASE     LC823450_UART0_REGBASE
#  define LC823450_CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define LC823450_CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define LC823450_CONSOLE_BITS     CONFIG_UART0_BITS
#  define LC823450_CONSOLE_2STOP    CONFIG_UART0_2STOP
#  define LC823450_UARTCLK_VALUE    (MCLKCNTAPB_UART0_CLKEN | \
                                     MCLKCNTAPB_UART0IF_CLKEN)
#  define LC823450_UARTRST_VALUE    MRSTCNTAPB_UART0_RSTB
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define LC823450_CONSOLE_BASE     LC823450_UART1_REGBASE
#  define LC823450_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define LC823450_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define LC823450_CONSOLE_BITS     CONFIG_UART1_BITS
#  define LC823450_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  define LC823450_UARTCLK_VALUE    (MCLKCNTAPB_UART1_CLKEN | \
                                     MCLKCNTAPB_UART1IF_CLKEN)
#  define LC823450_UARTRST_VALUE    MRSTCNTAPB_UART1_RSTB
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define LC823450_CONSOLE_BASE     LC823450_UART2_REGBASE
#  define LC823450_CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define LC823450_CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define LC823450_CONSOLE_BITS     CONFIG_UART2_BITS
#  define LC823450_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  define LC823450_UARTCLK_VALUE    (MCLKCNTAPB_UART2_CLKEN | \
                                     MCLKCNTAPB_UART2IF_CLKEN)
#  define LC823450_UARTRST_VALUE    MRSTCNTAPB_UART2_RSTB
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* UDIV settings */

#define UART_UDIV_N     16
#define UART_UDIV_VALUE ((UART_UDIV_N - 1) << 0)

/* UBR settings */

#define UART_UBR_VALUE \
  (65536 - (CTL_CLK / (UART_UDIV_N * LC823450_CONSOLE_BAUD)))

/* UMD settings */

#if LC823450_CONSOLE_BITS == 8
#  define UART_UMD_BIT_VALUE UART_UMD_CL
#else
#  define UART_UMD_BIT_VALUE 0
#endif

#if LC823450_CONSOLE_2STOP != 0
#  define UART_UMD_2STOP_VALUE UART_UMD_STL
#else
#  define UART_UMD_2STOP_VALUE 0
#endif

#if LC823450_CONSOLE_PARITY == 1
#  define UART_UMD_PARITY_VALUE UART_UMD_PS0
#elif LC823450_CONSOLE_PARITY == 2
#  define UART_UMD_PARITY_VALUE UART_UMD_PS1
#else
#  define UART_UMD_PARITY_VALUE 0
#endif

#define UART_UMD_VALUE \
  (UART_UMD_BIT_VALUE | UART_UMD_2STOP_VALUE | UART_UMD_PARITY_VALUE)

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

#ifdef CONFIG_DEV_CONSOLE_SWITCH
  if (g_console_disable)
    {
      return;
    }
#endif

  /* Wait until the TX FIFO is empty */

  while (!(getreg32(LC823450_CONSOLE_BASE + UART_USR) & UART_USR_TXEMP))
    ;

  /* Wait until the TX Register is not full */

  while (getreg32(LC823450_CONSOLE_BASE + UART_USR) & UART_USR_TFF)
    ;

  /* Clear SendDone status */

  putreg32(UART_UINT_UARTTF_INT, LC823450_CONSOLE_BASE + UART_UISR);

  /* Then send the character */

  putreg32((uint32_t)ch, LC823450_CONSOLE_BASE + UART_USTF);

  /* Wait SendDone */

  while (!(getreg32(LC823450_CONSOLE_BASE + UART_UISR) & UART_UINT_UARTTF_INT))
    ;
}

/**************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void lc823450_lowsetup(void)
{
  /* Clock & Reset */

  modifyreg32(MCLKCNTAPB, 0, LC823450_UARTCLK_VALUE);
  modifyreg32(MRSTCNTAPB, 0, LC823450_UARTRST_VALUE);

  /* INTC enable */

  modifyreg32(MRSTCNTBASIC, 0, MRSTCNTBASIC_IRQCNT_RSTB);

  /* baud */

  putreg32(UART_UBR_VALUE, LC823450_CONSOLE_BASE + UART_UBR);

  /* parity : bits : 2stop */

  putreg32(UART_UMD_VALUE, LC823450_CONSOLE_BASE + UART_UMD);

  /* Tx FIFO Enable */

  putreg32(UART_USFC_TXFF_EN, LC823450_CONSOLE_BASE + UART_USFC);

  /* Tx Enable */

  putreg32(UART_UCM_TE, LC823450_CONSOLE_BASE + UART_UCM);
}
