/****************************************************************************
 * board/board.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* LEDs
 *
 * The 16z board has 7 LEDs, five of which are controllable via software:
 *
 *   ----- ------ ------ ------------------------------------
 *   LED   Color  Signal Description
 *   ----- ------ ------ ------------------------------------
 *   LED1  Red     3V3   Indicates the presence of +3.3V
 *   LED2  Red     5V    Indicates the presence of +5V

 *   LED3  Blue    ~RF   Controlled via PH2.  Notes: 1, 2
 *   LED4  Green   ~SXM  Controlled via PH3.  Notes: 1, 3
 *   LED5  Green   ~SD1  Controlled via PJ0.  Notes: 1, 4
 *   LED6  Yellow  ~SD2  Controlled via PJ4.  Notes: 1, 5
 *   LED7  Yellow  ~SD0  Controlled via PJ7.  Notes: 1, 6
 *   ----- ------ ------ ------------------------------------
 *
 *   Note 1:  Pulled high so a low output illuminates the LED.
 *   Note 2:  PH2/~RF is also used by the RF transceiver, X2.  That part is not
 *            populated on my board.
 *   Note 3:  ~SXM is the chip select for the serial memory, U4.  That part is
 *            not populated on my board.
 *   Note 4:  ~SD1 is the chip select for the SD card 1, X11.
 *   Note 5:  ~SD2 is the chip select for the SD card 2, X10.
 *   Note 6:  ~SD0 is the chip select for the microSD 0, X12.
 *
 * In conclusion:  None of the LEDs are available to indicate software status
 * without potentially sacrificing board functionality.  If the RF transceiver
 * is not installed (CONFIG_16Z_RFTRANSCEIVER=n) and if LED support is
 * requested (CONFIG_ARCH_LEDS), then LED3 will be used to indicate status:  A
 * solid color means that the board has boot successfully; flashing at a rate
 * of approximately 2Hz indicates a software failure.
 */

#define __LED3_BIT (1 << 0)
#define __LED4_BIT (1 << 1)
#undef  HAVE_16Z_LED3
#undef  HAVE_16Z_LED4

#if !defined(CONFIG_16Z_RFTRANSCEIVER)
#  define __LEDPANIC __LED3_BIT
#  define HAVE_16Z_LED3
#elif !defined(CONFIG_16Z_SERIAL_MEMORY)
#  define __LEDPANIC __LED4_BIT
#  define HAVE_16Z_LED4
#else
#  define __LEDPANIC (0)
#endif

#if !defined(CONFIG_16Z_SERIAL_MEMORY) && !defined(HAVE_16Z_LED4)
#  define __LEDSTARTED __LED4_BIT
#  define HAVE_16Z_LED4
#else
#  define __LEDSTARTED __LEDPANIC
#endif
                                        /* LED3   LED4 */
#define LED_STARTED       0             /* OFF    OFF  */
#define LED_HEAPALLOCATE  0             /* N/C    N/C  */
#define LED_IRQSENABLED   0             /* N/C    N/C  */
#define LED_STACKCREATED  __LEDSTARTED  /* N/C    ON   */
#define LED_INIRQ         0             /* N/C    N/C  */
#define LED_SIGNAL        0             /* N/C    N/C  */
#define LED_ASSERTION     0             /* N/C    N/C  */
#define LED_PANIC         __LEDPANIC    /* ON     N/C  */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __ARCH_BOARD_BOARD_H */
