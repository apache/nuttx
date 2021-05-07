/****************************************************************************
 * arch/arm/src/sama5/sam_serialinit.c
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

#include "sam_config.h"
#include "sam_dbgu.h"
#include "sam_serial.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: sam_earlyserialinit
 *
 * Description:
 *   Performs the low level serial initialization early so that the serial
 *   console will be available during bootup.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void sam_earlyserialinit(void)
{
  /* NOTE:  All PIO configuration for the USARTs was performed in
   * sam_lowsetup
   */

#if defined(SAMA5_HAVE_UART) || defined(SAMA5_HAVE_USART)
  /* Initialize UART/USART drivers */

  uart_earlyserialinit();
#endif

#ifdef SAMA5_HAVE_FLEXCOM_USART
  /* Initialize Flexcom USARTs */

  flexus_earlyserialinit();
#endif
}
#endif

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register all serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#if defined(SAMA5_HAVE_UART) || defined(SAMA5_HAVE_USART)
  /* Register UART/USART drivers */

  uart_serialinit();
#endif

#ifdef SAMA5_HAVE_FLEXCOM_USART
  /* Register Flexcom USART drivers */

  flexus_serialinit();
#endif

  /* Register the DBGU as well */

#ifdef CONFIG_SAMA5_DBGU
  sam_dbgu_register();
#endif
}

#endif /* USE_SERIALDRIVER */
