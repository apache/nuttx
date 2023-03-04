/****************************************************************************
 * arch/arm64/src/fvp-v8r/serial_pl011.h
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

#ifndef __ARCH_ARM64_SRC_FVP_V8R_SERIAL_PL011_H
#define __ARCH_ARM64_SRC_FVP_V8R_SERIAL_PL011_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_FVP_UART_PL011
#define CONFIG_UART0_BASE      0x9c090000
#define CONFIG_UART0_IRQ       (GIC_SPI_INT_BASE + 5)

#define CONFIG_UART1_BASE      0x9c0a0000
#define CONFIG_UART1_IRQ       (GIC_SPI_INT_BASE + 6)

#define CONFIG_UART2_BASE      0x9c0b0000
#define CONFIG_UART2_IRQ       (GIC_SPI_INT_BASE + 7)

#define CONFIG_UART3_BASE      0x9c0c0000
#define CONFIG_UART3_IRQ       (GIC_SPI_INT_BASE + 8)

#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_FVP_V8R_SERIAL_PL011_H */
