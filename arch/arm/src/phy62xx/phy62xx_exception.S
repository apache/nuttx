/****************************************************************************
 * arch/arm/src/phy62xx/phy62xx_exception.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include "exc_return.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		exception_phy
	.file		"phy62xx_exception.S"

/****************************************************************************
 * .text
 ****************************************************************************/

/* Common exception handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      IPSR contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

	.text
	.align	2
	.code	16
	.thumb_func
	.type	exception_phy, function
exception_phy:

	/* Complete the context save */
	.extern exception_common

	pop		{r4}
	pop		{r1}

	/* Jump to exception_common in arm_exception.S */

	b		exception_common

	.size	exception_phy, .-exception_phy

	.end


