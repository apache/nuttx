/************************************************************************************
 * configs/cc3200/src/cc3200_util.c
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *   Author: Jim Ewing <jim@droidifi.com>
 *
 *   Adapted from code Copyright (C) 2014 Texas Instruments Incorporated
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

#include <sys/types.h>
#include <arch/board/cc3200_utils.h>

#include "cc3200_launchpad.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: cc3200_putc
 ************************************************************************************/

void cc3200_putc(char c)
{
  while(HWREG(0x4000C000 + 0x00000018) & 0x00000020)
    ;

  HWREG(0x4000C000) = c;
}

/************************************************************************************
 * Name: cc3200_getc
 ************************************************************************************/

char cc3200_getc(void)
{
  if (!(HWREG(0x4000C000 + 0x00000018) & 0x00000010))
    {
      return HWREG(0x4000C000);
    }
  else
    {
      return -1;
    }
}

/************************************************************************************
 * Name: cc3200_print
 ************************************************************************************/

void cc3200_print(char* str)
{
  while (str && *str != '\0')
    {
      cc3200_putc(*str++);
    }
}

/************************************************************************************
 * Name: cc3200_pin_config_set
 ************************************************************************************/

void cc3200_pin_config_set(uint32_t pin, uint32_t pin_strength, uint32_t pin_type)
{
  uint32_t pad;

  pad = g_cc3200_pinmap[pin & 0x3F];

  switch (pin_type)
    {
    case PIN_TYPE_ANALOG:
      HWREG(0x4402E144) |= ((0x80 << pad) & (0x1E << 8));
      pad = ((pad << 2) + PAD_CONFIG_BASE);
      HWREG(pad) |= 0xC00;
      break;

    default:
      HWREG(0x4402E144) &= ~((0x80 << pad) & (0x1E << 8));
      pad = ((pad << 2) + PAD_CONFIG_BASE);
      HWREG(pad) = ((HWREG(pad) & ~(PAD_STRENGTH_MASK | PAD_TYPE_MASK)) | (pin_strength | pin_type ));
      break;
    }
}

/************************************************************************************
 * Name: cc3200_pin_mode_set
 ************************************************************************************/

void cc3200_pin_mode_set(uint32_t pin, uint32_t pin_mode)
{
  uint32_t pad;

  pad = g_cc3200_pinmap[pin & 0x3F];
  pad = ((pad << 2) + PAD_CONFIG_BASE);
  HWREG(pad) = (((HWREG(pad) & ~PAD_MODE_MASK) |  pin_mode) & ~(3<<10));
}

/************************************************************************************
 * Name: cc3200_pin_type_uart
 ************************************************************************************/

void cc3200_pin_type_uart(uint32_t pin, uint32_t pin_mode)
{
  cc3200_pin_mode_set(pin, pin_mode);
  cc3200_pin_config_set(pin, PIN_STRENGTH_2MA, PIN_TYPE_STD);
}

/************************************************************************************
 * Name: cc3200_init
 ************************************************************************************/

void cc3200_init(void)
{
  uint8_t x=16;

  HWREG(0x4402F064) |= 0x800000;
  HWREG(0x4402F800  + 0x00000418) |= (1<<4);
  HWREG(0x4402E16C) |= 0x3C;
  HWREG(0x44025000 + 0x00000048) |= 0x00000001;
  while(--x)
    ;
  HWREG(0x44025000 + 0x00000048) &= ~0x00000001;
  HWREG(0x4402F804) = 0x0;
  HWREG(0x4402F804) = 0x1;

  if (((HWREG(0x4402F0C8) & 0xFF) == 0x2))
    {
      HWREG(0x4402E110) = ((HWREG(0x4402E110) & ~0xC0F) | 0x2);
      HWREG(0x4402E114) = ((HWREG(0x4402E110) & ~0xC0F) | 0x2);
    }

  HWREG(0x4402E184) |= 0x2;

  if ((HWREG(0x4402E0A4) & 0xF) == 0x1)
    {
      HWREG(0x4402E0A4) = ((HWREG(0x4402E0A4) & ~0xF));
    }

  if ((HWREG(0x4402E0A8) & 0xF) == 0x1)
    {
      HWREG(0x4402E0A8) = ((HWREG(0x4402E0A8) & ~0xF));
    }

  if (((HWREG(0x4402DC78) >> 22) & 0xF) == 0xE)
    {
      HWREG(0x4402F0B0) = ((HWREG(0x4402F0B0) & ~(0x00FC0000))|(0x32 << 18));
    }
  else
    {
      HWREG(0x4402F0B0) = ((HWREG(0x4402F0B0) & ~(0x00FC0000))|(0x29 << 18));
    }
}
