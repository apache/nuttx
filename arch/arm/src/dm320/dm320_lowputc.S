/**************************************************************************
 * arch/arm/src/dm320/dm320_lowputc.S
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

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ldr	r2, =DM320_UART1_REGISTER_BASE	/* r2=UART1 base */
#else
	ldr	r2, =DM320_UART0_REGISTER_BASE	/* r2=UART0 base */
#endif

	/* Poll the TX fifo trigger level bit of the UART_SSR
	 * register.  When the bit is non-zero, the TX FIFO is no
	 * longer full
	 */

1:	ldrh	r1, [r2, #UART_SR]
	tst	r1, #UART_SR_TFTI
	beq	1b

	/* Send the character by writing it into the UART_DTRR
	 * register.
	 */

	strh	r0, [r2, #UART_DTRR]

	/* Wait for the tranmsit register to be emptied. This is
	 * determined when TX register empty bit of the SR is zero.
	 */

2:	ldrh	r1, [r2, #UART_SR]
	tst	r1, #UART_SR_TREF
	bne	2b

	/* If the character that we just sent was a linefeed,
	 * then send a carriage return as well.
	 */

	teq	r0, #'\n'
	moveq	r0, #'\r'
	beq 	1b

	/* And return */

	mov	pc, lr
