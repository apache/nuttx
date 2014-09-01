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

#include "nuttx/arch.h"
#include "up_arch.h"

#include "cc3200_launchpad.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: cc3200_print
 ************************************************************************************/

void cc3200_print(char* str)
{
  while (str && *str != '\0')
    {
      up_putc(*str++);
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
      putreg32(getreg32(0x4402E144) | ((0x80 << pad) & (0x1E << 8)), 0x4402E144);
      pad = ((pad << 2) + PAD_CONFIG_BASE);
      putreg32(getreg32(pad) | 0xC00, pad);
      break;

    default:
      putreg32(getreg32(0x4402E144) & ~((0x80 << pad) & (0x1E << 8)), 0x4402E144);
      pad = ((pad << 2) + PAD_CONFIG_BASE);
      putreg32(((getreg32(pad) & ~(PAD_STRENGTH_MASK | PAD_TYPE_MASK)) | (pin_strength | pin_type )), pad);
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
  putreg32( (((getreg32(pad) & ~PAD_MODE_MASK) |  pin_mode) & ~(3<<10)), pad);
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

  putreg32(getreg32(0x4402F064) | 0x800000,0x4402F064);
  putreg32(getreg32(0x4402F800  + 0x00000418) | (1<<4), 0x4402F800  + 0x00000418);
  putreg32(getreg32(0x4402E16C) | 0x3C, 0x4402E16C);
  putreg32(getreg32(0x44025000 + 0x00000048) | 0x00000001, 0x44025000 + 0x00000048);
  while(--x)
    ;
  putreg32(getreg32(0x44025000 + 0x00000048) & ~0x00000001, 0x44025000 + 0x00000048);
  putreg32(0x0, 0x4402F804);
  putreg32(0x1, 0x4402F804);

  if (((getreg32(0x4402F0C8) & 0xFF) == 0x2))
    {
      putreg32((getreg32(0x4402E110) & ~0xC0F) | 0x2, 0x4402E110);
      putreg32((getreg32(0x4402E114) & ~0xC0F) | 0x2, 0x4402E114);
    }

  putreg32(getreg32(0x4402E184) | 0x2, 0x4402E184);

  if ((getreg32(0x4402E0A4) & 0xF) == 0x1)
    {
      putreg32(getreg32(0x4402E0A4) & ~0xF, 0x4402E0A4);
    }

  if ((getreg32(0x4402E0A8) & 0xF) == 0x1)
    {
      putreg32(getreg32(0x4402E0A8) & ~0xF, 0x4402E0A8);
    }

  if (((getreg32(0x4402DC78) >> 22) & 0xF) == 0xE)
    {
      putreg32((getreg32(0x4402F0B0) & ~(0x00FC0000)) | (0x32 << 18), 0x4402F0B0);
    }
  else
    {
      putreg32((getreg32(0x4402F0B0) & ~(0x00FC0000)) | (0x29 << 18), 0x4402F0B0);
    }
}
