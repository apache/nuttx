/**************************************************************************
 * arch/arm/src/c5471/c5471_lowputc.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Public Data
 **************************************************************************/

/**************************************************************************
 * Private Data
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: arm_lowputc
 **************************************************************************/

/* This assembly language version has the advantage that it can does not
 * require a C stack and uses only r0-r1.  Hence it can be used during
 * early boot phases.
 */

	.text
	.global	arm_lowputc
	.type	arm_lowputc, function
arm_lowputc:
	/* On entry, r0 holds the character to be printed */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
	ldr	r2, =UART_IRDA_BASE	/* r2=IRDA UART base */
#else
	ldr	r2, =UART_MODEM_BASE	/* r2=Modem UART base */
#endif

	/* Poll bit 0 of the UART_SSR register.  When the bit
	 * is clear, the TX FIFO is no longer full
	 */

1:	ldr	r1, [r2, #UART_SSR_OFFS]
	tst	r1, #UART_SSR_TXFULL
	bne	1b

	/* Send the character by writing it into the UART_THR
	 * register.
	 */

	str	r0, [r2, #UART_THR_OFFS]

	/* Wait for the tranmsit holding register (THR) to be
	 * emptied. This is determined when bit 6 of the LSR
	 * is set.
	 */

2:	ldr	r1, [r2, #UART_LSR_OFFS]
	tst	r1, #0x00000020
	beq	2b

	/* If the character that we just sent was a linefeed,
	 * then send a carriage return as well.
	 */

	teq	r0, #'\n'
	moveq	r0, #'\r'
	beq	1b

	/* And return */

	mov	pc, lr
