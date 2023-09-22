/****************************************************************************
 * arch/arm64/src/imx8/imx8_serial.h
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

#ifndef __ARCH_ARM64_SRC_IMX8_IMX8_SERIAL_H
#define __ARCH_ARM64_SRC_IMX8_IMX8_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_internal.h"

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

#ifdef CONFIG_ARCH_CHIP_IMX8_QUADMAX
#define SPI_OFFSET                  32
#define CONFIG_IMX8QM_UART1_BASE    0x5a060000
#define CONFIG_IMX8QM_UART1_IRQ     (SPI_OFFSET + 345)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void imx8_earlyserialinit(void);

#endif

/****************************************************************************
 * Name: uart_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT) && defined(IMX8_HAVE_UART)
void uart_earlyserialinit(void);

#endif

/****************************************************************************
 * Name: uart_serialinit
 *
 * Description:
 *   Register the UART serial console and serial ports.  This assumes that
 *   uart_earlyserialinit was called previously.
 *
 ****************************************************************************/

#ifdef IMX8_HAVE_UART
void uart_serialinit(void);

#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMX6_IMX_SERIAL_H */
