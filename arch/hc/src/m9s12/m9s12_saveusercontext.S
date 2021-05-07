/**************************************************************************
 * arch/hc/src/m9s12/m9s12_saveusercontext.S
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

#include <arch/irq.h>

#include "up_internal.h"
#include "m9s12.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

#ifdef CONFIG_HCS12_NONBANKED
#  define CALL   jsr
#  define RETURN rts
#else
#  define CALL   call
#  define RETURN rtc
#endif

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
 * Name: up_saveusercontext
 *
 * Description:
 *	 Create this state save strucure:
 *   Low Address  [PPAGE]
 *                [soft regisers]
 *                XYH
 *                XYL
 *                ZH
 *                ZL
 *                TMPH
 *                TMPL
 *                FRAMEH
 *                FRAMEL
 *                SP    <-- SP after interrupt
 *                CCR
 *                B
 *                A
 *                XH
 *                XL
 *                YH
 *                YL
 *                PCH
 *   High Address PCL
 *
 *   On entry:
 *    D=Pointer to save save structure
 *    TOS=return address
 *
 **************************************************************************/

	.text
	.globl	up_saveusercontext
	.type	up_saveusercontext, function
up_saveusercontext:
	/* Exchange D with X.  Now X points to the save structure. */

	xgdx

	/* Save the PPAGE register */

#ifndef CONFIG_HCS12_NONBANKED
	movb	HCS12_MMC_PPAGE, 1, x+
#endif

	/* Save the soft registers */

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#endif
#if CONFIG_HCS12_MSOFTREGS > 1
	movw	_.d2, 2, x+
#endif
#if CONFIG_HCS12_MSOFTREGS > 0
	movw	_.d1, 2, x+
#endif

	/* It is not necessary to save the value of _.tmp, _.z, or _.xy */

	ldd		#0
	std		2, x+			/* Save _.xy = 0 */
	std		2, x+			/* Save _.z = 0 */
	std		2, x+			/* Save _.tmp = 0 */

	/* Save _.frame */

	movw	_.frame, 2, x+

	/* Save the value of the stack "before" this function was called */

	tfr		sp, d			/* D = current SP */
	addd	#(TOTALFRAME_SIZE-INTFRAME_SIZE)
	std		2, x+			/* Save the value of SP on entry */

	/* Save the CCR */

	tpa						/* A = CCR */
	staa	1, x+			/* Save CCR in the structure */

	/* D (A:B) is the return value.  Save 1 as the new return value as it
	 * will appear after a context switch back to the current thread.
	 */

	ldd		#1
	std		2, x+			/* Save D = 1 */

	/* X, Y do not need to be preserved.  Write zeros to these locations */

	ldd		#0
	std		2, x+			/* Save X = 0 */
	std		2, x+			/* Save Y = 0 */

	/* Fetch the 2-byte return address from the stack and save it at the
	 * end of the state save area
	 */

	movw	0, sp, 2, x+	/* Save PCH and PCL */

#if __INT__ == 32 /* 32-bit ABI */
	ldx		#0
#endif
	clra
	clrb
	RETURN
	.size	up_saveusercontext, . - up_saveusercontext
	.end
