/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_lowputc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "bsp_api.h"
#include "chip.h"
#include "r_sci_b_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI0
#  define RZV2H_CONSOLE_CHANNEL 0
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI0_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI0_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI0_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI0_2STOP
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI1
#  define RZV2H_CONSOLE_CHANNEL 1
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI1_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI1_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI1_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI1_2STOP
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI2
#  define RZV2H_CONSOLE_CHANNEL 2
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI2_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI2_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI2_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI2_2STOP
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI3
#  define RZV2H_CONSOLE_CHANNEL 3
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI3_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI3_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI3_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI3_2STOP
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI4
#  define RZV2H_CONSOLE_CHANNEL 4
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI4_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI4_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI4_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI4_2STOP
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI5
#  define RZV2H_CONSOLE_CHANNEL 5
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI5_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI5_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI5_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI5_2STOP
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI6
#  define RZV2H_CONSOLE_CHANNEL 6
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI6_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI6_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI6_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI6_2STOP
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI7
#  define RZV2H_CONSOLE_CHANNEL 7
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI7_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI7_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI7_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI7_2STOP
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI8
#  define RZV2H_CONSOLE_CHANNEL 8
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI8_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI8_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI8_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI8_2STOP
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#  define RZV2H_CONSOLE_REG R_SCI9
#  define RZV2H_CONSOLE_CHANNEL 9
#  define RZV2H_CONSOLE_BAUD CONFIG_SCI9_BAUD
#  define RZV2H_CONSOLE_BITS CONFIG_SCI9_BITS
#  define RZV2H_CONSOLE_PARITY CONFIG_SCI9_PARITY
#  define RZV2H_CONSOLE_2STOP CONFIG_SCI9_2STOP
#endif

#define RZV2H_SCI_CCR0_RE_MASK         (1u << 0)
#define RZV2H_SCI_CCR0_TE_MASK         (1u << 4)
#define RZV2H_SCI_CCR0_IDSEL_MASK      (1u << 10)
#define RZV2H_SCI_CCR1_SPB2_MASK       (0x00000030u)
#define RZV2H_SCI_CCR1_PARITY_SHIFT    (8u)
#define RZV2H_SCI_CCR1_NFEN_MASK       (1u << 28)
#define RZV2H_SCI_CCR3_CHAR_SHIFT      (8u)
#define RZV2H_SCI_CCR3_LSBF_MASK       (1u << 12)
#define RZV2H_SCI_CCR3_STP_MASK        (1u << 14)
#define RZV2H_SCI_CESR_RIST_MASK       (1u << 0)
#define RZV2H_SCI_CESR_TIST_MASK       (1u << 4)
#define RZV2H_SCI_CFCLR_CLEAR_ALL      (0x9d070010u)
#define RZV2H_SCI_CSR_TDRE_MASK        (1u << 29)
#define RZV2H_SCI_CSR_TEND_MASK        (1u << 30)
#define RZV2H_SCI_FCR_DEFAULT          (0x1f1f0000u)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

extern fsp_err_t rzv2h_fsp_baud_calculate(
  uint32_t baud, bool modulation, uint32_t error,
  sci_b_baud_setting_t *setting)
  __asm__("R_SCI_B_UART_BaudCalculate");

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void rzv2h_lowputc(char ch)
{
#ifdef RZV2H_CONSOLE_REG
  if ((RZV2H_CONSOLE_REG->CCR0 & RZV2H_SCI_CCR0_TE_MASK) == 0)
    {
      return;
    }

  while ((RZV2H_CONSOLE_REG->CSR & RZV2H_SCI_CSR_TDRE_MASK) == 0)
    {
    }

  RZV2H_CONSOLE_REG->TDR_BY = (uint8_t)ch;

  while ((RZV2H_CONSOLE_REG->CSR & RZV2H_SCI_CSR_TEND_MASK) == 0)
    {
    }
#endif
}

/****************************************************************************
 * Name: rzv2h_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void rzv2h_lowsetup(void)
{
#ifdef RZV2H_CONSOLE_REG
  sci_b_baud_setting_t baud_setting;
  uint32_t ccr1;
  uint32_t ccr3;

  rzv2h_serial_setup();

  if (rzv2h_fsp_baud_calculate(RZV2H_CONSOLE_BAUD, false, 50000,
                               &baud_setting) != FSP_SUCCESS)
    {
      return;
    }

  R_BSP_MODULE_START(FSP_IP_SCI, RZV2H_CONSOLE_CHANNEL);

  /* Stop communication while changing the asynchronous-mode settings. */

  RZV2H_CONSOLE_REG->CCR0 = RZV2H_SCI_CCR0_IDSEL_MASK;

  ccr3 = RZV2H_SCI_CCR3_LSBF_MASK;
  ccr3 |= (RZV2H_CONSOLE_BITS == 7 ? UART_DATA_BITS_7 :
           UART_DATA_BITS_8) << RZV2H_SCI_CCR3_CHAR_SHIFT;

  if (RZV2H_CONSOLE_2STOP)
    {
      ccr3 |= RZV2H_SCI_CCR3_STP_MASK;
    }

  RZV2H_CONSOLE_REG->CCR3 = ccr3;
  RZV2H_CONSOLE_REG->CCR2 = baud_setting.baudrate_bits;

  /* Hold TXD high while transmission is disabled. Configure parity and the
   * receive noise filter used by the normal FSP instance.
   */

  ccr1 = RZV2H_SCI_CCR1_SPB2_MASK | RZV2H_SCI_CCR1_NFEN_MASK;
  if (RZV2H_CONSOLE_PARITY == 1)
    {
      ccr1 |= UART_PARITY_ODD << RZV2H_SCI_CCR1_PARITY_SHIFT;
    }
  else if (RZV2H_CONSOLE_PARITY == 2)
    {
      ccr1 |= 1u << RZV2H_SCI_CCR1_PARITY_SHIFT;
    }

  RZV2H_CONSOLE_REG->CCR1 = ccr1;
  RZV2H_CONSOLE_REG->CCR4 = 0;

  if ((BSP_FEATURE_SCI_UART_FIFO_CHANNELS &
       (1u << RZV2H_CONSOLE_CHANNEL)) != 0)
    {
      RZV2H_CONSOLE_REG->FCR = RZV2H_SCI_FCR_DEFAULT;
    }

  RZV2H_CONSOLE_REG->CFCLR = RZV2H_SCI_CFCLR_CLEAR_ALL;

  /* Leave interrupt enables clear until the serial driver takes ownership. */

  RZV2H_CONSOLE_REG->CCR0 = RZV2H_SCI_CCR0_IDSEL_MASK |
                            RZV2H_SCI_CCR0_TE_MASK |
                            RZV2H_SCI_CCR0_RE_MASK;

  while ((RZV2H_CONSOLE_REG->CESR & RZV2H_SCI_CESR_TIST_MASK) == 0 ||
         (RZV2H_CONSOLE_REG->CESR & RZV2H_SCI_CESR_RIST_MASK) == 0)
    {
    }
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Write one character to the serial console with interrupts disabled.
 *
 ****************************************************************************/

void up_putc(int ch)
{
  irqstate_t flags;

  flags = enter_critical_section();
  rzv2h_lowputc((char)ch);
  leave_critical_section(flags);
}
