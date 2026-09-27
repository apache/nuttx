/****************************************************************************
 * arch/arm/src/n32h7/n32_uart.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_N32H7_N32H7_UART_H
#define __ARCH_ARM_SRC_N32H7_N32H7_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Disable UARTs not supported by the chip */
#if N32H7_NUSART < 8
#  undef CONFIG_N32H7_USART8
#endif
#if N32H7_NUSART < 7
#  undef CONFIG_N32H7_USART7
#endif
#if N32H7_NUSART < 6
#  undef CONFIG_N32H7_USART6
#endif
#if N32H7_NUSART < 5
#  undef CONFIG_N32H7_USART5
#endif
#if N32H7_NUSART < 4
#  undef CONFIG_N32H7_USART4
#endif
#if N32H7_NUSART < 3
#  undef CONFIG_N32H7_USART3
#endif
#if N32H7_NUSART < 2
#  undef CONFIG_N32H7_USART2
#endif
#if N32H7_NUSART < 1
#  undef CONFIG_N32H7_USART1
#endif

#if N32H7_NUART < 7
#  undef CONFIG_N32H7_UART15
#endif
#if N32H7_NUART < 6
#  undef CONFIG_N32H7_UART14
#endif
#if N32H7_NUART < 5
#  undef CONFIG_N32H7_UART13
#endif
#if N32H7_NUART < 4
#  undef CONFIG_N32H7_UART12
#endif
#if N32H7_NUART < 3
#  undef CONFIG_N32H7_UART11
#endif
#if N32H7_NUART < 2
#  undef CONFIG_N32H7_UART10
#endif
#if N32H7_NUART < 1
#  undef CONFIG_N32H7_UART9
#endif

/* Check if any UART is enabled */
#if defined(CONFIG_N32H7_USART1) || defined(CONFIG_N32H7_USART2) || \
    defined(CONFIG_N32H7_USART3) || defined(CONFIG_N32H7_USART4) || \
    defined(CONFIG_N32H7_USART5) || defined(CONFIG_N32H7_USART6) || \
    defined(CONFIG_N32H7_USART7) || defined(CONFIG_N32H7_USART8) || \
    defined(CONFIG_N32H7_UART9)  || defined(CONFIG_N32H7_UART10) || \
    defined(CONFIG_N32H7_UART11) || defined(CONFIG_N32H7_UART12) || \
    defined(CONFIG_N32H7_UART13) || defined(CONFIG_N32H7_UART14) || \
    defined(CONFIG_N32H7_UART15)
#  define HAVE_UART 1
#endif

/* Console selection */
#undef CONSOLE_UART
#undef HAVE_CONSOLE

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART1)
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART2)
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART3)
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART4_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART4)
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART5)
#  define CONSOLE_UART 5
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART6)
#  define CONSOLE_UART 6
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART7_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART7)
#  define CONSOLE_UART 7
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART8_SERIAL_CONSOLE) && defined(CONFIG_N32H7_USART8)
#  define CONSOLE_UART 8
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART9_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART9)
#  define CONSOLE_UART 9
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART10_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART10)
#  define CONSOLE_UART 10
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART11_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART11)
#  define CONSOLE_UART 11
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART12_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART12)
#  define CONSOLE_UART 12
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART13_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART13)
#  define CONSOLE_UART 13
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART14_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART14)
#  define CONSOLE_UART 14
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART15_SERIAL_CONSOLE) && defined(CONFIG_N32H7_UART15)
#  define CONSOLE_UART 15
#  define HAVE_CONSOLE 1
#else
#  define CONSOLE_UART 0
#  undef HAVE_CONSOLE
#endif

#define CONFIG_N32H7_SERIAL_RXDMA_BUFFER_SIZE 64

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

uart_dev_t *n32_serial_get_uart(int uart_num);
void n32_serial_dma_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_N32H7_N32_UART_H */
