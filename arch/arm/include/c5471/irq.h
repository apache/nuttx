/****************************************************************************
 * arch/arm/include/c5471/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_C5471_IRQ_H
#define __ARCH_ARM_INCLUDE_C5471_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* C5471 Interrupts */

#define C5471_IRQ_TIMER0         0
#define C5471_IRQ_TIMER1         1
#define C5471_IRQ_TIMER2         2
#define C5471_IRQ_GPIO0          3
#define C5471_IRQ_ETHER          4
#define C5471_IRQ_KBGPIO_0_7     5
#define C5471_IRQ_UART           6
#define C5471_IRQ_UART_IRDA      7
#define C5471_IRQ_KBGPIO_8_15    8
#define C5471_IRQ_GPIO3          9
#define C5471_IRQ_GPIO2         10
#define C5471_IRQ_I2C           11
#define C5471_IRQ_GPIO1         12
#define C5471_IRQ_SPI           13
#define C5471_IRQ_GPIO_4_19     14
#define C5471_IRQ_API           15

#define C5471_IRQ_WATCHDOG      C5471_IRQ_TIMER0
#define C5471_IRQ_SYSTIMER      C5471_IRQ_TIMER2
#define NR_IRQS                 (C5471_IRQ_API+1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_C5471_IRQ_H */
