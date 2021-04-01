/**************************************************************************
 * arch/arm/src/imx1/imx_lowputc.S
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
	ldr	r2, =IMX_UART1_VBASE	/* r2=UART1 base */
#else
	ldr	r2, =IMX_UART2_VBASE	/* r2=UART0 base */
#endif

	/* Poll the TX fifo trigger level bit of the UART status register #2 .
	 * When the bit is non-zero, the TX Buffer FIFO is empty.
	 */

1:	ldr	r1, [r2, #UART_USR2]
	tst	r1, #UART_USR2_TXFE
	beq	1b

	/* Send the character by writing it into the UART_DTRR
	 * register.
	 */

	str	r0, [r2, #UART_TXD0]

	/* If the character that we just sent was a linefeed,
	 * then send a carriage return as well.
	 */

	teq	r0, #'\n'
	moveq	r0, #'\r'
	beq	1b

	/* Wait for the tranmsit register to be emptied. When the bit is
	 * non-zero, the TX Buffer FIFO is empty.
	 */

2:	ldrh	r1, [r2, #UART_USR2]
	tst	r1, #UART_USR2_TXFE
	beq	2b

	/* Then return */

	mov	pc, lr
	.end
