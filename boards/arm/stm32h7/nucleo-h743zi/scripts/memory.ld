/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/scripts/memory.ld
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

/* TODO: Add description for memory organisation */

MEMORY
{
  /* ITCM boot address */

  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 64K

  /* 2048Kb FLASH */

  kflash (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx) : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx) : ORIGIN = 0x08040000, LENGTH = 1792K

  /* 288Kb SRAM123 */

  ksram (rwx) : ORIGIN = 0x30000000, LENGTH = 16K
  usram (rwx) : ORIGIN = 0x30004000, LENGTH = 16K
  xsram (rwx) : ORIGIN = 0x30008000, LENGTH = 288K - 32K

  /* 512Kb of contiguous AXI SRAM */

  sram (rwx)  : ORIGIN = 0x24000000, LENGTH = 512K

  /* DTCM SRAM */

  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH = 64K

  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
}
