/****************************************************************************
 * boards/renesas/m16c/skp16c26/src/m16c_lcdconsole.c
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

#include <nuttx/arch.h>

#include "renesas_internal.h"
#include "skp16c26.h"

/* Only use the LCD as a console if there are is no serial console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART0)
#  define HAVE_SERIALCONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART1)
#  define HAVE_SERIALCONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART2)
#  define HAVE_SERIALCONSOLE 1
#else
#  undef HAVE_SERIALCONSOLE
#endif

#if !defined(HAVE_SERIALCONSOLE) && defined(CONFIG_SLCD) && \
     defined(CONFIG_SLCD_CONSOLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level UART initialization early in  debug so that the
 *   serial console will be available during bootup.
 *   This must be called before up_consoleinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
# warning "You probably need to define CONFIG_ARCH_LOWCONSOLE"
void up_earlyconsoleinit(void)
{
  /* There is probably a problem if we are here */
}
#endif

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/

#if USE_SERIALDRIVER
# warning "You probably need to define CONFIG_ARCH_LOWCONSOLE"
void up_consoleinit(void)
{
  /* There is probably a problem if we are here */
}
#endif

/****************************************************************************
 * Name: renesas_lowputc
 *
 * Description:
 *   Output one character on the console
 *
 ****************************************************************************/

void renesas_lowputc(char ch)
{
  up_lcdputc(ch);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one character on the console
 *
 ****************************************************************************/

int up_putc(int ch)
{
  up_lcdputc(ch);
  return ch;
}

#endif /* !HAVE_SERIALCONSOLE && CONFIG_SLCD && CONFIG_SLCD_CONSOLE */
