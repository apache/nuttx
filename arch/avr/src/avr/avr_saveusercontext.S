/************************************************************************************
 * arch/avr/src/avr/avr_saveusercontext.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "excptmacros.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.file		"avr_saveusercontext.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context
 *
 ****************************************************************************/

	.text
	.globl	up_saveusercontext
	.func	up_saveusercontext
up_saveusercontext:
	/* Use X [r26:r27] to reference the save structure.  (X is Call-used) */

	movw	r26, r24

	/* Save the context to saveregs */

	USER_SAVE

	.endfunc
	.end
