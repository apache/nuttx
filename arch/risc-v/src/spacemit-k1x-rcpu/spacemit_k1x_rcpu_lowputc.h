/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_LOWPUTC_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/spacemit_k1x_rcpu_uart.h"
#include "spacemit_k1x_rcpu_memorymap.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes one RCPU UART.  The RCPU exposes two (r_uart0 and r_uart1); only
 * r_uart0 is wired on the boards seen so far, but nothing here assumes that.
 */

struct spacemit_k1x_rcpu_uart_config_s
{
  uint32_t base;      /* Register base, e.g. SPACEMIT_K1X_RCPU_RUART0_BASE         */
  uint32_t baud;      /* Line speed in bits per second               */
  uint32_t clock;     /* Functional clock feeding the block, in Hz   */
  uint8_t  parity;    /* 0 none, 1 odd, 2 even                       */
  uint8_t  bits;      /* Data bits, 5 to 8                           */
  uint8_t  stopbits;  /* 1 or 2                                      */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: spacemit_k1x_rcpu_lowsetup
 *
 * Description:
 *   Configure the console UART for polled output.  Called from the start up
 *   path before any dependency on the driver framework exists.
 *
 ****************************************************************************/

extern void spacemit_k1x_rcpu_lowsetup(void);

/****************************************************************************
 * Name: spacemit_k1x_rcpu_uart_configure
 *
 * Description:
 *   Program one UART from a configuration record.  Shared by the early
 *   console and by the interrupt driven driver so that the baud rate and the
 *   frame format are computed in exactly one place.
 *
 ****************************************************************************/

void spacemit_k1x_rcpu_uart_configure(
  const struct spacemit_k1x_rcpu_uart_config_s *config);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_LOWPUTC_H */
