/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lowputc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_LOWPUTC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_LOWPUTC_H

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
#include "imxrt_config.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
/* This structure describes the configuration of an UART */

struct uart_config_s
{
  uint32_t baud;          /* Configured baud */
  uint8_t  parity;        /* 0=none, 1=odd, 2=even */
  uint8_t  bits;          /* Number of bits (5-9) */
  bool     stopbits2;     /* true: Configure with 2 stop bits instead of 1 */
  bool     userts;        /* True: Assert RTS when there are data to be sent */
  bool     invrts;        /* True: Invert sense of RTS pin (true=active high) */
  bool     usects;        /* True: Condition transmission on CTS asserted */
  bool     users485;      /* True: Assert RTS while transmission progresses */
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization including setup of the console UART.  This UART done
 *   early so that the serial console is available for debugging very early
 *   in the boot sequence.
 *
 ****************************************************************************/

void imxrt_lowsetup(void);

/****************************************************************************
 * Name: imxrt_lpuart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ****************************************************************************/

#ifdef HAVE_LPUART_DEVICE
int imxrt_lpuart_configure(uint32_t base,
                           const struct uart_config_s *config);
#endif

/****************************************************************************
 * Name: imxrt_lowputc
 *
 * Description:
 *   Output a byte with as few system dependencies as possible.  This will
 *   even work BEFORE the console is initialized if we are booting from U-
 *   Boot (and the same UART is used for the console, of course.)
 *
 ****************************************************************************/

#if defined(HAVE_LPUART_DEVICE)
void imxrt_lowputc(int ch);
#else
#  define imxrt_lowputc(ch)
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_LOWPUTC_H */
