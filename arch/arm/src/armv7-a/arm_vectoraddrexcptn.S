/****************************************************************************
 * arch/arm/src/armv7-a/arm_vectoraddrexceptn.S
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
#include <nuttx/irq.h>

	.file	"arm_vectoraddrexcptn.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	arm_vectoraddrexcptn

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

	.text
	.syntax	unified
	.arm

/****************************************************************************
 *  Name: arm_vectoraddrexcptn
 *
 * Description:
 *   Shouldn't happen.  This exception handler is in a separate file from
 *   other vector handlers because some processors do not support the
 *   Address Exception vector.
 *
 ****************************************************************************/

	.globl	arm_vectoraddrexcptn
	.type	arm_vectoraddrexcptn, %function
arm_vectoraddrexcptn:
	b		arm_vectoraddrexcptn
	.size	arm_vectoraddrexcptn, . - arm_vectoraddrexcptn
	.end
