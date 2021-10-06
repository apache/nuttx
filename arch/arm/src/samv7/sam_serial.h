/****************************************************************************
 * arch/arm/src/samv7/sam_serial.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H
#define __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif



#ifdef CONFIG_SAMV7_UART0
void sam_board_disable_uart0 (bool disabled);
void sam_board_init_uart0 (void);
#endif

#ifdef CONFIG_SAMV7_UART1
void sam_board_disable_uart1 (bool disabled);
void sam_board_init_uart1 (void);
#endif

#ifdef CONFIG_SAMV7_UART2
void sam_board_disable_uart2 (bool disabled);
void sam_board_init_uart2 (void);
#endif

#ifdef CONFIG_SAMV7_UART3
void sam_board_disable_uart3 (bool disabled);
void sam_board_init_uart3 (void);
#endif

#ifdef CONFIG_SAMV7_USART0
void sam_board_disable_usart0 (bool disabled);
void sam_board_init_usart0 (void);
#endif

#ifdef CONFIG_SAMV7_USART1
void sam_board_disable_usart1 (bool disabled);
void sam_board_init_usart1 (void);
#endif

#ifdef CONFIG_SAMV7_USART2
void sam_board_disable_usart2 (bool disabled);
void sam_board_init_usart2 (void);
#endif



#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SPI_H */
