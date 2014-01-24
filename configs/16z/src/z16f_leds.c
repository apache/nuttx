/****************************************************************************
 * configs/16z/z16f_leds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void)
{
  /* All GPIO initialization is done in up_lowinit() */
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
#if defined(HAVE_16Z_LED3) || defined(HAVE_16Z_LED4)
  /* The following operations must be atomic */

  irqstate_t  flags = irqsave();
  uint8_t regval = getreg8(Z16F_GPIOH_OUT);

#ifdef HAVE_16Z_LED3
  /* LED3 is available at PH2 and illuminated by a low output */

  if ((led & __LED3_BIT) != 0)
    {
      regval &= ~(1 << 2);
    }
#endif

#ifdef HAVE_16Z_LED4
  /* LED4 is available at PH3 and illuminated by a low output */

  if ((led & __LED4_BIT) != 0)
    {
      regval &= ~(1 << 3);
    }
#endif

  putreg8(regval, Z16F_GPIOH_OUT);
  irqrestore(flags);
#endif
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
#if defined(HAVE_16Z_LED3) || defined(HAVE_16Z_LED4)
  /* The following operations must be atomic */

  irqstate_t  flags = irqsave();
  uint8_t regval = getreg8(Z16F_GPIOH_OUT);

#ifdef HAVE_16Z_LED3
  /* LED3 is available at PH2 and illuminated by a low output */

  if ((led & __LED3_BIT) != 0)
    {
      regval |= (1 << 2);
    }
#endif

#ifdef HAVE_16Z_LED4
  /* LED4 is available at PH3 and illuminated by a low output */

  if ((led & __LED4_BIT) != 0)
    {
      regval |= (1 << 3);
    }
#endif

  putreg8(regval, Z16F_GPIOH_OUT);
  irqrestore(flags);
#endif
}

#endif /* CONFIG_ARCH_LEDS */
