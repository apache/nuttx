/****************************************************************************
 * arch/risc-v/src/hpm6000/hpm_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HPM_LOWPUTC_H
#define __ARCH_RISCV_SRC_HPM6000_HPM_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef HAVE_UART_DEVICE
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
 * Name: hpm_lowsetup
 ****************************************************************************/

EXTERN void hpm_lowsetup(void);

#ifdef HAVE_UART_DEVICE
int hpm_uart_configure(uint32_t base, const struct uart_config_s *config);
#endif

#ifdef HAVE_UART_DEVICE
void hpm_lowputc(int ch);
#else
#  define hpm_lowputc(ch)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM6000_HPM_LOWPUTC_H */
