/****************************************************************************
 * arch/x86/src/qemu/qemu_lowputc.c
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
#include <arch/io.h>
#include "x86_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* COM1 port addresses */

#define COM1_PORT 0x3f8   /* COM1: I/O port 0x3f8, IRQ 4 */
#define COM2_PORT 0x2f8   /* COM2: I/O port 0x2f8, IRQ 3 */
#define COM3_PORT 0x3e8   /* COM3: I/O port 0x3e8, IRQ 4 */
#define COM4_PORT 0x2e8   /* COM4: I/O port 0x2e8, IRQ 3 */

/* 16650 register offsets */

#define COM_RBR   0       /* DLAB=0, Receiver Buffer (read) */
#define COM_THR   0       /* DLAB=0, Transmitter Holding Register (write) */
#define COM_DLL   0       /* DLAB=1, Divisor Latch (least significant byte) */
#define COM_IER   1       /* DLAB=0, Interrupt Enable */
#define COM_DLM   1       /* DLAB=1, Divisor Latch(most significant byte) */
#define COM_IIR   2       /* Interrupt Identification (read) */
#define COM_FCR   2       /* FIFO Control (write) */
#define COM_LCR   3       /* Line Control */
#define COM_MCR   4       /* MODEM Control */
#define COM_LSR   5       /* Line Status */
#define COM_MSR   6       /* MODEM Status */
#define COM_SCR   7       /* Scratch */

/* 16650 register bit definitions */

#define LSR_THRE  (1 << 5) /* Bit 5: Transmitter Holding Register Empty */

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
 * Name: x86_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void x86_lowputc(char ch)
{
  /* Wait until the Transmitter Holding Register (THR) is empty. */

  while ((inb(COM1_PORT + COM_LSR) & LSR_THRE) == 0);

  /* Then output the character to the THR */

  outb(ch, COM1_PORT + COM_THR);
}
