/****************************************************************************
 * arch/risc-v/src/bl602/bl602_lowputc.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_LOWPUTC_H
#define __ARCH_RISCV_SRC_BL602_BL602_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct uart_config_s
{
  uint8_t  idx;       /* Uart idx */
  uint32_t baud;      /* Configured baud */
  uint8_t  iflow_ctl; /* Input flow control supported */
  uint8_t  oflow_ctl; /* Output flow control supported. */
  uint8_t  data_bits; /* Number of bits per word */
  bool     stop_bits; /* true=2 stop bits; false=1 stop bit */
  uint8_t  parity;    /* Parity selection:  0=none, 1=odd, 2=even */
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN void bl602_lowsetup(void);

EXTERN void bl602_uart_configure(const struct uart_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_LOWPUTC_H */
