/****************************************************************************
 * arch/arm/src/stm32/stm32_getc.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select U[S]ART console base address */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART1_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART1_BASE
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART2_BASE
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART3_BASE
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART4_BASE
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART5_BASE
#  elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_USART6_BASE
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART7_BASE
#  elif defined(CONFIG_UART8_SERIAL_CONSOLE)
#    define STM32_CONSOLE_BASE     STM32_UART8_BASE
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getc
 *
 * Description:
 *   Read one byte from the serial console
 *
 *   REVIST:  If used with the serial driver enabled, then this could
 *   interfere with the serial driver operations.  Serial interrupts should
 *   be disabled when this function executes in that case.
 *
 ****************************************************************************/

int up_getc(void)
{
  uint32_t ch = 0;

#ifdef HAVE_CONSOLE
  /* While there is any error, read and discard bytes to clear the errors */

  while ((getreg32(STM32_CONSOLE_BASE + STM32_USART_SR_OFFSET) &
         (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0)
    {
      (void)getreg32(STM32_CONSOLE_BASE + STM32_USART_RDR_OFFSET);
    }

  /* Wait until the RX data register has a character to be read */

  while ((getreg32(STM32_CONSOLE_BASE + STM32_USART_SR_OFFSET) & USART_SR_RXNE) == 0);

  /* Then read the character */

  ch = getreg32(STM32_CONSOLE_BASE + STM32_USART_RDR_OFFSET);
#endif /* HAVE_CONSOLE */

  return (int)ch;
}
