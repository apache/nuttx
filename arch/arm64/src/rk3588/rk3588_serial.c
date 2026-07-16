/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_serial.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_16550_UART

#include <nuttx/serial/uart_16550.h>

#include "arm64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_earlyserialinit(void)
{
  /* The Orange Pi boot chain already programs the console UART.  Preserve
   * that state and let the generic 16550 driver take over without forcing
   * a baud-rate reprogramming step.
   */

  u16550_earlyserialinit();
}

void arm64_serialinit(void)
{
  u16550_serialinit();
}

#endif /* CONFIG_16550_UART */
