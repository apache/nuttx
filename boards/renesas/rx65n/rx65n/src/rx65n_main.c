/****************************************************************************
 * configs/rx65n-grrose/src/rx65n.main.c
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in>
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

/****************************************************************************
 *Included files
 ***************************************************************************/

#include "rx65n_macrodriver.h"
#include "arch/board/board.h"
#include "rx65n_definitions.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled1_on
 *
 * Description:
 *   Turns on LED 0
 *
 ****************************************************************************/

void board_autoled1_on(int led)
{
  LED0 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled2_on
 *
 * Description:
 *   Turns on LED 1
 *
 ****************************************************************************/

void board_autoled2_on(int led)
{
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turns on LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  LED0 = LED_ON;
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled1_off
 *
 * Description:
 *   Turns off LED 0
 *
 ****************************************************************************/

void board_autoled1_off(int led)
{
  LED0 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled2_off
 *
 * Description:
 *   Turns off LED 1
 *
 ****************************************************************************/

void board_autoled2_off(int led)
{
  LED1 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turns off LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  LED0 = LED_OFF;
  LED1 = LED_OFF;
}
