/****************************************************************************
 * arch/risc-v/src/hpm6750/hpm6750_lowputc.c
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
#include "hpm_config.h"
#include "hpm.h"
#include "board.h"
#include "hpm_debug_console.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#define HAVE_UART
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
    console_send_byte(ch);
#endif /* HAVE_CONSOLE */
}


/****************************************************************************
 * Name: hpm_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void hpm_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  board_init_console();

#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
