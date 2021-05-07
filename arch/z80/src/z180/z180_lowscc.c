/****************************************************************************
 * arch/z80/src/z180/z180_lowscc.c
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
#include <string.h>

#include <arch/io.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "chip.h"
#include "common/z80_internal.h"
#include "z180_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_Z180_SCC_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z181_SCC_CR
#  define CONSOLE_DR           Z181_SCC_DR
#  define CONSOLE_BAUD         CONFIG_Z180_SCC_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_SCC_BITS
#  define CONSOLE_2STOP        CONFIG_Z180_SCC_2STOP
#  define CONSOLE_PARITY       CONFIG_Z180_SCC_PARITY

#elif defined(CONFIG_Z180_ESCCA_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z182_ESCCA_CR
#  define CONSOLE_DR           Z182_ESCCA_DR
#  define CONSOLE_BAUD         CONFIG_Z180_ESCCA_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_ESCCA_BITS
#  define CONSOLE_2STOP        CONFIG_Z180_ESCCA_2STOP
#  define CONSOLE_PARITY       CONFIG_Z180_ESCCA_PARITY

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define CONSOLE_CR           Z182_ESCCB_CR
#  define CONSOLE_DR           Z182_ESCCB_DR
#  define CONSOLE_BAUD         CONFIG_Z180_ESCCB_BAUD
#  define CONSOLE_BITS         CONFIG_Z180_ESCCB_BITS
#  define CONSOLE_PARITY       CONFIG_Z180_ESCCB_PARITY
#endif

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
 * Name: z180_putc
 *
 * Description:
 *   Low-level character output
 *
 ****************************************************************************/

#ifdef HAVE_SCC_CONSOLE
void z180_putc(uint8_t ch) __naked
{
  __asm
txbe:
  in0 a, (CONSOLE_CR) ; read RR0
  bit 2, a ; bit 2, tx buffer empty?
  jr z, txbe ; no, wait until the tx buffer is empty

  ld a, 4(ix) ; character to output
  out (CONSOLE_DR), a ; send it
  ret
  __endasm;
}
#endif
