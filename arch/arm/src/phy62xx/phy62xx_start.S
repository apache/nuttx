/***************************************************************************
 * arch/arm/src/phy62xx/phy62xx_start.S
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
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include "exc_return.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"phy62xx_start.S"

#if 0
	.text
	.align	2
	.code	16
	.thumb_func
	.globl		__start
	.type	__start, function
__start:

	ldr     r1,  =(_ebss+CONFIG_IDLETHREAD_STACKSIZE)
	msr		msp, r1					/* r2>>sp */

	bl		c_start				/* R0=IRQ, R1=register save area on stack */
	bx c_start

	.size	__start, .__start

#endif

	.text
	.align	2
	.code	16
	.globl		__start
	.thumb_func
	.type	__start, function
__start:

	ldr     r1,  =(_ebss+CONFIG_IDLETHREAD_STACKSIZE)
	msr		msp, r1					/* r2>>sp */
	bl		c_start				/* R0=IRQ, R1=register save area on stack */

	.size	__start, .-__start

	.end
