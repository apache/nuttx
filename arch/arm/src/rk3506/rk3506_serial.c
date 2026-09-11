/***************************************************************************
 * arch/arm/src/rk3506/rk3506_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <nuttx/serial/uart_16550.h>

#include "arm_internal.h"

#ifdef CONFIG_16550_UART

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   see arm_internal.h
 *
 ***************************************************************************/

void arm_earlyserialinit(void)
{
  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

  u16550_earlyserialinit();
}

/***************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   see arm_internal.h
 *
 ***************************************************************************/

void arm_serialinit(void)
{
  u16550_serialinit();
}

#endif /* CONFIG_16550_UART */
