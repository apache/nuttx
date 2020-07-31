/****************************************************************************
 * arch/arm/src/lc823450/lc823450_lowputc.c
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

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"

#include "lc823450_serial.h"
#include "lc823450_syscontrol.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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

  while (!(getreg32(LC823450_CONSOLE_BASE + UART_UISR)
           & UART_UINT_UARTTF_INT))
    ;
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

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
