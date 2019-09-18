/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/scripts/memory.ld
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
 ****************************************************************************/

/* The STM32L4R9AI has 2048Kb of FLASH beginning at address 0x0800:0000,
 * 192Kb of SRAM beginning at address 0x2000:0000, and 64Kb of SRAM2 beginning
 * at address 0x1000:0000, and 384Kb of SRAM3 at address 0x2004:0000.
 *
 * The SRAM2 has special properties (e.g. parity check, separate power domain for
 * retention in low power modes, write protection). It is also aliased at address
 * 0x2003:0000 to be contiguous with SRAM1 and SRAM3.
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000 where
 * the code expects to begin execution by jumping to the entry point in the
 * 0x0800:0000 address range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 128Kb of
 * FLASH and 16Kb of SRAM.  That is an excessive amount for the kernel which
 * should fit into 64KB and, of course, can be optimized as needed (See
 * also boards/arm/stm32l4/stm32l4r9ai-disco/scripts/kernel-space.ld).  Allowing the
 * additional does permit addition debug instrumentation to be added to the
 * kernel space without overflowing the partition.
 *
 * Alignment of the user space FLASH partition is also a critical factor:
 * The user space FLASH partition will be spanned with a single region of
 * size 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 512KB at it would have to be positioned at
 * 0x08800000.  If you change this address, don't forget to change the
 * CONFIG_NUTTX_USERSPACE configuration setting to match and to modify
 * the check in kernel/userspace.c.
 */

MEMORY
{
    /* 2048Kb FLASH */

  kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x08040000, LENGTH = 1792K

    /* 640Kb of contiguous SRAM */

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 16K
  usram (rwx)      : ORIGIN = 0x20004000, LENGTH = 16K
  xsram (rwx)      : ORIGIN = 0x20008000, LENGTH = 192K + 64K - 32K

  sram3 (rwx)      : ORIGIN = 0x20040000, LENGTH = 384K
}
