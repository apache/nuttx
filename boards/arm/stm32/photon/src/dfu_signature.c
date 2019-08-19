/****************************************************************************
 * boards/arm/stm32/photon/src/dfu_signature.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

__attribute__((packed)) struct dfu_signature
{
  uint32_t linker_start_address;
  uint32_t linker_end_address;
  uint8_t  reserved[4];
  uint16_t board_id;
  uint8_t  firmware_type1;
  uint8_t  firmware_type2;
  uint8_t  reserved2[8];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _firmware_start;
extern uint32_t _firmware_end;

/****************************************************************************
 * Private Data
 ****************************************************************************/

__attribute__((externally_visible, section(".dfu_signature")))
  const struct dfu_signature dfu_sign =
{
  (uint32_t)&_firmware_start, /* Flash image start address */
  (uint32_t)&_firmware_end,   /* Flash image end address */
  {0, 0, 0, 0},               /* reserved */
  6,                          /* Current board is photon */
  4, 1,                       /* Firmware is "system-part1" */
  {0, 0, 0, 0, 0, 0, 0, 0}    /* reserved */
};
