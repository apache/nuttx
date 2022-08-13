/****************************************************************************
 * arch/arm/src/imx6/imx_lowputc.h
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_LOWPUTC_H
#define __ARCH_ARM_SRC_IMX6_IMX_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef IMX_HAVE_UART
/* This structure describes the configuration of an UART */

struct uart_config_s
{
  uint32_t baud;          /* Configured baud */
  uint8_t  parity;        /* 0=none, 1=odd, 2=even */
  uint8_t  bits;          /* Number of bits (5-9) */
  bool     stopbits2;     /* true: Configure with 2 stop bits instead of 1 */
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void imx_lowsetup(void);

/****************************************************************************
 * Name: imx_uart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef IMX_HAVE_UART
int imx_uart_configure(uint32_t base,
                       const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: imx_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.
 *   This will even work BEFORE the console is initialized if we are booting
 *   from U-Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

#ifdef IMX_HAVE_UART
void imx_lowputc(int ch);
#endif

#endif /* __ARCH_ARM_SRC_IMX6_IMX_LOWPUTC_H */
