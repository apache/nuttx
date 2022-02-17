/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_lowputc.c
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

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_arch.h"

#include "hardware/mpfs_memorymap.h"
#include "hardware/mpfs_uart.h"
#include "mpfs.h"
#include "mpfs_clockconfig.h"
#include "mpfs_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define MPFS_CONSOLE_BASE        MPFS_UART0_BASE
#  define MPFS_CONSOLE_BAUD        CONFIG_UART0_BAUD
#  define MPFS_CONSOLE_BITS        CONFIG_UART0_BITS
#  define MPFS_CONSOLE_PARITY      CONFIG_UART0_PARITY
#  define MPFS_CONSOLE_2STOP       CONFIG_UART0_2STOP
#  define MPFS_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART0
#  define MPFS_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART0
#  define HAVE_UART
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define MPFS_CONSOLE_BASE        MPFS_UART1_BASE
#  define MPFS_CONSOLE_BAUD        CONFIG_UART1_BAUD
#  define MPFS_CONSOLE_BITS        CONFIG_UART1_BITS
#  define MPFS_CONSOLE_PARITY      CONFIG_UART1_PARITY
#  define MPFS_CONSOLE_2STOP       CONFIG_UART1_2STOP
#  define MPFS_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART1
#  define MPFS_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART1
#  define HAVE_UART
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define MPFS_CONSOLE_BASE        MPFS_UART2_BASE
#    define MPFS_CONSOLE_BAUD        CONFIG_UART2_BAUD
#    define MPFS_CONSOLE_BITS        CONFIG_UART2_BITS
#    define MPFS_CONSOLE_PARITY      CONFIG_UART2_PARITY
#    define MPFS_CONSOLE_2STOP       CONFIG_UART2_2STOP
#    define MPFS_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART2
#    define MPFS_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART2
#    define HAVE_UART
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define MPFS_CONSOLE_BASE        MPFS_UART3_BASE
#    define MPFS_CONSOLE_BAUD        CONFIG_UART3_BAUD
#    define MPFS_CONSOLE_BITS        CONFIG_UART3_BITS
#    define MPFS_CONSOLE_PARITY      CONFIG_UART3_PARITY
#    define MPFS_CONSOLE_2STOP       CONFIG_UART3_2STOP
#    define MPFS_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART3
#    define MPFS_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART3
#    define HAVE_UART
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define MPFS_CONSOLE_BASE        MPFS_UART4_BASE
#    define MPFS_CONSOLE_BAUD        CONFIG_UART4_BAUD
#    define MPFS_CONSOLE_BITS        CONFIG_UART4_BITS
#    define MPFS_CONSOLE_PARITY      CONFIG_UART4_PARITY
#    define MPFS_CONSOLE_2STOP       CONFIG_UART4_2STOP
#    define MPFS_CONSOLE_CLOCKBIT    SYSREG_SUBBLK_CLOCK_CR_MMUART4
#    define MPFS_CONSOLE_RESETBIT    SYSREG_SOFT_RESET_CR_MMUART4
#    define HAVE_UART
#  elif defined(HAVE_UART)
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(CONFIG_SUPPRESS_UART_CONFIG)

/****************************************************************************
 * Name: config_baud_divisors
 *
 * Description:
 *   Configure the UART baudrate divisors.
 *
 ****************************************************************************/

static void config_baud_divisors(void)
{
  uint32_t baud_value;
  uint32_t baud_value_by_64;
  uint32_t baud_value_by_128;
  uint32_t fractional_baud_value;
  uint64_t pclk_freq;

  pclk_freq = MPFS_MSS_APB_AHB_CLK;

  /* Compute baud value based on requested baud rate and PCLK frequency.
   * The baud value is computed using the following equation:
   *   baud_value = PCLK_Frequency / (baud_rate * 16)
   */

  baud_value_by_128 = (uint32_t)((8UL * pclk_freq) / MPFS_CONSOLE_BAUD);
  baud_value_by_64 = baud_value_by_128 / 2U;
  baud_value = baud_value_by_64 / 64U;
  fractional_baud_value = baud_value_by_64 - (baud_value * 64U);
  fractional_baud_value += (baud_value_by_128 - (baud_value * 128U))
                           - (fractional_baud_value * 2U);

  if (baud_value <= (uint32_t)UINT16_MAX)
    {
      putreg32(baud_value >> 8, MPFS_CONSOLE_BASE + MPFS_UART_DLH_OFFSET);
      putreg32(baud_value & 0xff, MPFS_CONSOLE_BASE + MPFS_UART_DLH_OFFSET);

      if (baud_value > 1)
        {
          /* Enable Fractional baud rate */

          uint8_t mm0 = getreg32(MPFS_CONSOLE_BASE + MPFS_UART_MM0_OFFSET);
          mm0 |= UART_MM0_EFBR;
          putreg32(mm0, MPFS_CONSOLE_BASE + MPFS_UART_MM0_OFFSET);
          putreg32(fractional_baud_value,
                   MPFS_CONSOLE_BASE + MPFS_UART_DFR_OFFSET);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#if defined HAVE_UART && defined HAVE_SERIAL_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(MPFS_CONSOLE_BASE + MPFS_UART_LSR_OFFSET)
          & UART_LSR_THRE) == 0);

  /* Send the character */

  putreg32((uint32_t)ch, MPFS_CONSOLE_BASE + MPFS_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: mpfs_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void mpfs_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t lcr = 0;

  /* reset on */

  modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
            0, MPFS_CONSOLE_RESETBIT);

  /* reset off */

  modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET,
              0, MPFS_CONSOLE_CLOCKBIT);

  /* clock on */

  modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
              MPFS_CONSOLE_RESETBIT, 0);

  switch (MPFS_CONSOLE_BITS)
    {
    case 5:
      lcr |= UART_LCR_DLS_5BITS;
      break;

    case 6:
      lcr |= UART_LCR_DLS_6BITS;
      break;

    case 7:
      lcr |= UART_LCR_DLS_7BITS;
      break;

    case 8:
    default:
      lcr |= UART_LCR_DLS_8BITS;
      break;
    }

#ifdef MPFS_CONSOLE_2STOP
      lcr |= UART_LCR_STOP;
#endif

  if (MPFS_CONSOLE_PARITY == 1)
    {
      lcr |= UART_LCR_PEN;
    }
  else if (MPFS_CONSOLE_PARITY == 2)
    {
      lcr |= (UART_LCR_PEN | UART_LCR_EPS);
    }

  putreg32(lcr | UART_LCR_DLAB, MPFS_CONSOLE_BASE + MPFS_UART_LCR_OFFSET);
  config_baud_divisors();
  putreg32(lcr, MPFS_CONSOLE_BASE + MPFS_UART_LCR_OFFSET);

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
