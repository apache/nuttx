/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_serial.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_SERIAL_H
#define __ARCH_ARM_SRC_RZV2H_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "r_sci_b_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_FSP_STATUS_PARITY_ERR   (1u << 0)
#define RZV2H_FSP_STATUS_FRAMING_ERR  (1u << 1)
#define RZV2H_FSP_STATUS_OVERRUN_ERR  (1u << 2)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void rzv2h_fsp_uart_cb(uart_callback_args_t *args);

#if defined(USE_EARLYSERIALINIT) && defined(CONFIG_RZV2H_SCI_B)
void rzv2h_earlyserialinit(void);
#endif

#endif /* __ARCH_ARM_SRC_RZV2H_SERIAL_H */
