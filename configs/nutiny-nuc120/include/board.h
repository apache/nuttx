/************************************************************************************
 * configs/nutiny-nuc120/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef _BITNO
#define _BITNO

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/

/* LED definitions ******************************************************************/
/* The NuTiny has a single green LED that can be controlled from sofware.  This LED
 * is connected to PIN17.
 */

/* LED index values for use with nuc_setled() */

#define BOARD_LED         0 /* Green LED */
#define BOARD_NLEDS       1

/* LED bits for use with nuc_setleds() */

#define BOARD_LED_BIT     (1 << BOARD_LED)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
 * NuTiny.  The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  ------------- ------------
 *   LED_STARTED          NuttX has been started   LED ON
 *   LED_HEAPALLOCATE     Heap has been allocated  LED ON
 *   LED_IRQSENABLED      Interrupts enabled       LED ON
 *   LED_STACKCREATED     Idle stack created       LED ON
 *   LED_INIRQ            In an interrupt          LED should glow
 *   LED_SIGNAL           In a signal handler      LED might glow
 *   LED_ASSERTION        An assertion failed      LED ON while handling the assertion
 *   LED_PANIC            The system has crashed   LED Blinking at 2Hz
 *   LED_IDLE             STM32 is is sleep mode   (Optional, not used)
 */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  0
#define LED_IRQSENABLED   0
#define LED_STACKCREATED  0
#define LED_INIRQ         0
#define LED_SIGNAL        0
#define LED_ASSERTION     0
#define LED_PANIC         0

/* Button definitions ***************************************************************/
/* The NuTiny has no buttons */

#define NUM_BUTTONS        0

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: nuc_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void nuc_boardinitialize(void);

/************************************************************************************
 * Name:  nuc_ledinit, nuc_setled, and nuc_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void nuc_ledinit(void);
EXTERN void nuc_setled(int led, bool ledon);
EXTERN void nuc_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* _BITNO */
