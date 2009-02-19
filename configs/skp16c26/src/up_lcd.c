/************************************************************************************
 * configs/scp16c26/src/up_lcd.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_LCD

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LCD commands *********************************************************************/

#define LCD_CLEAR        0x01    /* Clear LCD display and home cursor */
#define LCD_HOME_L1      0x80    /* move cursor to line 1 */
#define LCD_HOME_L2      0xc0    /* move cursor to line 2 */
#define CURSOR_MODE_DEC  0x04    /* Cursor auto decrement after R/W */
#define CURSOR_MODE_INC  0x06    /* Cursor auto increment after R/W */
#define FUNCTION_SET     0x28    /* Setup, 4 bits,2 lines, 5X7 */
#define LCD_CURSOR_ON    0x0e    /* Display ON with Cursor */
#define LCD_CURSOR_OFF   0x0c    /* Display ON with Cursor off */
#define LCD_CURSOR_BLINK 0x0d    /* Display on with blinking cursor */
#define LCD_CURSOR_LEFT  0x10    /* Move Cursor Left One Position */
#define LCD_CURSOR_RIGHT 0x14    /* Move Cursor Right One Position */

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_lcddelay
 ************************************************************************************/

static void up_lcddelay(uint16 count)
{
  uint32 counter = (uint16)count << 8;
  while(counter--)
    {
      asm("\tnop\n\tnop\n\tnop\n" : :); /* 3 NOPs */
    }
}

/************************************************************************************
 * Name: up_setrs
 ************************************************************************************/

static inline void up_setrs(boolean data)
{
  /* Set/clear bit 1 of port 6 */

  register ubyte regval = getreg8(M16C_P6);
  if (data)
    {
      regval |= (1 << 0);  /* High = data */
    }
  else
    {
      regval &= ~(1 << 0); /* Low = control */
    }
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_seten
 ************************************************************************************/

static inline void up_seten(void)
{
  /* Set bit 1 of port 6 */

  register ubyte regval = getreg8(M16C_P6);
  regval = (1 << 1);
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_clren
 ************************************************************************************/

static inline void up_clren(void)
{
  /* Clear bit 1 of port 6 */

  register ubyte regval = getreg8(M16C_P6);
  regval &= ~(1 << 1);
  putreg8(regval, M16C_P6);
}

/************************************************************************************
 * Name: up_enpluse
 ************************************************************************************/

static inline void up_enpulse(boolean data)
{
  up_seten();                 /* EN enable chip (HIGH) */
  up_lcddelay(0);             /* Short delay */
  up_clren();                 /* Latch data by setting EN low */
  up_lcddelay(0);             /* Short delay for data writes */
  if (!data) up_lcddelay(0);  /* Longer delay for control writes */
}

/************************************************************************************
 * Name: up_lcdwrite
 ************************************************************************************/

void up_lcdwrite(boolean data, ubyte ch)
{
  up_setrs(data);				/* Set RS appropriately */

  /* Write upper nibble first.  Only the lower 4 bits of P9 are valid.  The upper four
   * bits are reserved and must be zero.
   */

  putreg8(ch >> 4, M16C_P9);
  up_enpulse(data);

  /* Write lower nibble second */

  putreg8(ch & 0x0f, M16C_P9);
  up_enpulse(data);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_lcdinit
 ************************************************************************************/

void up_lcdinit(void)
{
  ubyte regval;

  /* Enable writing to PD9 by selecting bit 2 in the protection register */

  regval = getreg8(M16C_PRCR);
  regval |= (1 << 2);
  putreg8(regval, M16C_PRCR);

  /* We can't read PD9, so we can't OR the values in */

  putreg8(0x0f, M16C_PD9);

  /* Set EN (port 6 bit 1) and Set RS (port 6 bit 0) as outputs */

  regval = getreg8(M16C_P6);
  regval |= (1 << 1) | (1 << 0);
  putreg8(regval, M16C_P6);

  regval = getreg8(M16C_PD6);
  regval |= (1 << 1) | (1 << 0);
  putreg8(regval, M16C_PD6);

  /* Set EN low */

  up_clren();

  /* Write the reset sequence */

  up_lcdwrite(FALSE, 0x33);
  up_lcddelay(20);
  up_lcdwrite(FALSE, 0x32);
  up_lcddelay(20);
  up_lcdwrite(FALSE, FUNCTION_SET);	/* reset sequence */
  up_lcdwrite(FALSE, FUNCTION_SET);
  up_lcdwrite(FALSE, LCD_CURSOR_OFF);
  up_lcdwrite(FALSE, LCD_CLEAR);
  up_lcdwrite(FALSE, LCD_HOME_L1);
}

/************************************************************************************
 * Name: up_lcdputc
 ************************************************************************************/

void up_lcdputc(char ch)
{
  up_lcdwrite(TRUE, ch);
}

#endif /* CONFIG_ARCH_LCD */
