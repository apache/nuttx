/****************************************************************************
 * arch/arm/src/sama5/sam_serialinit.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/****************************************************************************
 * Name: sam_earlyserialinit
 *
 * Description:
 *   Performs the low level serial initialization early so that the serial
 *   console will be available during bootup.  This must be called
 *   before up_serialinit.
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

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register all serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
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
