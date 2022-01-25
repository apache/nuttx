
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
