/****************************************************************************
 * arch/arm/src/pp502x/pp502x_serial.c
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

#include <stdint.h>

#include <nuttx/serial/uart_16550.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on UART0 without using the serial driver.
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
  while ((getreg32(PP502X_UART0_BASE + PP502X_UART_LSR_OFFSET) &
          PP502X_UART_LSR_THRE) == 0)
    {
    }

  putreg32((uint32_t)ch, PP502X_UART0_BASE + PP502X_UART_THR_OFFSET);
}

#ifdef CONFIG_16550_UART

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Configure the console UART before the OS starts.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  u16550_earlyserialinit();
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register the serial devices.
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
void arm_serialinit(void)
{
  u16550_serialinit();
}
#endif

#endif /* CONFIG_16550_UART */
