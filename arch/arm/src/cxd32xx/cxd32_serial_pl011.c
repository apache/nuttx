/***************************************************************************
 * arch/arm/src/cxd32xx/cxd32_serial_pl011.c
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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/serial/uart_pl011.h>

#include "arm_internal.h"

#ifdef USE_SERIALDRIVER

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

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

  pl011_earlyserialinit();
}
#endif

/***************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   see arm_internal.h
 *
 ***************************************************************************/

void arm_serialinit(void)
{
  pl011_serialinit();
}

#ifdef CONFIG_UART_PL011_PLATFORMIF
/***************************************************************************
 * Name: pl011_platform interface
 *
 * Description:
 *   see drivers/serial/serial_pl011.c
 *        pl011_setup
 *        pl011_shutdown
 *
 ***************************************************************************/

int pl011_platform_setup(uint32_t base)
{
  /* If needed, implement platform specific process such as enabling pl011
   * to reduce power consumption.
   */

  return 0;
}

int pl011_platform_shutdown(uint32_t base)
{
  /* If needed, implement platform specific process such as disabling pl011
   * to reduce power consumption.
   */

  return 0;
}
#endif /* CONFIG_UART_PL011_PLATFORMIF */
#endif /* USE_SERIALDRIVER */
