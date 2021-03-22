/****************************************************************************
 * arch/arm/src/kinetis/kinetis_serialinit.c
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

#include "kinetis_config.h"
#include "kinetis.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(HAVE_UART_DEVICE) && !defined(HAVE_LPUART_DEVICE)
#  undef CONFIG_KINETS_LPUART_LOWEST
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_earlyserialinit
 *
 * Description:
 *   Performs the low level UART and LPUART initialization early in debug
 *   so that the serial console will be available during bootup.  This must
 *   be called before arm_serialinit.  NOTE:  This function depends on GPIO
 *   pin configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

#if defined(USE_EARLYSERIALINIT)
void kinetis_earlyserialinit(void)
{
#if defined(HAVE_UART_DEVICE)
  /* Initialize UART drivers */

  kinetis_uart_earlyserialinit();
#endif

#if defined(HAVE_LPUART_DEVICE)
  /* Initialize LPUART drivers */

  kinetis_lpuart_earlyserialinit();
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register all the serial console and serial ports.  This assumes
 *   that kinetis_earlyserialinit was called previously.
 *
 ****************************************************************************/

#if defined(USE_SERIALDRIVER)
void arm_serialinit(void)
{
#if defined(HAVE_UART_DEVICE) ||defined(HAVE_LPUART_DEVICE)
  uint32_t start = 0;
#endif

  /* Register the console and drivers */

#if defined(HAVE_LPUART_DEVICE) && defined(CONFIG_KINETS_LPUART_LOWEST)
  /* Register LPUART drivers in starting positions */

  start = kinetis_lpuart_serialinit(start);
#endif

#if defined(HAVE_UART_DEVICE)
  /* Register UART drivers */

  start = kinetis_uart_serialinit(start);
#endif

#if defined(HAVE_LPUART_DEVICE) && !defined(CONFIG_KINETS_LPUART_LOWEST)
  /* Register LPUART drivers in last positions */

  start = kinetis_lpuart_serialinit(start);
#endif
}
#endif /* USE_SERIALDRIVER */
