/************************************************************************************
 * configs/avr32dev1/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* #define AVR32_FRCOSC      15200         RCOsc frequency in Hz */

#define AVR32_FOSC32         32768      /* OSC32 frequency in Hz */
#define AVR32_OSC32STARTUP   3          /* OSC32 startup time in RCOsc periods */

#define AVR32_FOSC0          12000000   /* OSC0 frequency in Hz */
#define AVR32_OSC0STARTUP    3          /* OSC0 startup time in RCOsc periods.

/* #define AVR32_FOSC1       12000000      OSC1 frequency: Hz.
 * #define AVR32_OSC1STARTUP 3             OSC1 startup time in RCOsc periods.
 */

/* Select OSC0 as the main clock */

#define AVR32_CLOCK_OSC0     1
#undef  AVR32_CLOCK_PLL0

#define AVR32_CPU_CLOCK      AVR32_FOSC0
#define AVR32_PBA_CLOCK      AVR32_FOSC0

/* Pin muliplexing selecion *********************************************************/

#define PINMUX_USART1_RXD    PINMUX_USART1_RXD_1
#define PINMUX_USART1_TXD    PINMUX_USART1_TXD_1

/* LED definitions ******************************************************************/
/* The AVR32DEV1 board has 3 LEDs, two of which can be controlled through GPIO pins */

                                /*    ON        OFF    */
								/* LED1 LED2 LED1 LED2 */
#define LED_STARTED          0  /* OFF  OFF  OFF  OFF  */
#define LED_HEAPALLOCATE     0  /* OFF  OFF  OFF  OFF  */
#define LED_IRQSENABLED      0  /* OFF  OFF  OFF  OFF  */
#define LED_STACKCREATED     1  /* ON   OFF  OFF  OFF  */
#define LED_INIRQ            2  /* ON   ON   ON   OFF  */
#define LED_SIGNAL           2  /* ON   ON   ON   OFF  */
#define LED_ASSERTION        2  /* ON   ON   ON   OFF  */
#define LED_PANIC            2  /* ON   ON   ON   OFF  */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

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
 * Name: avr32_boardinitialize
 *
 * Description:
 *   All AVR32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void avr32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */

