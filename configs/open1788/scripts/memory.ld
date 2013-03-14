/****************************************************************************
 * configs/open1788/knsh/memory.ld
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

/* The LPC1788 has 512Kb of FLASH beginning at address 0x0000:0000 and
 * 96Kb of total SRAM: 64Kb of SRAM in the CPU block beginning at address
 * 0x10000000 and 32Kb of Peripheral SRAM in two banks, 8Kb at addresses
 * 0x20000000 bank0 first and 8kb at 0x20002000 at bank0 second. And 16Kb
 * at 0x20004000 on bank1.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 64Kb of
 * FLASH and 4Kb of SRAM.  That, of course, can be optimized as needed (See
 * also configs/open1788/scripts/kernel-space.ld).  A detailed memory map
 * for the CPU SRAM region is as follows:
 *
 *   0x10000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x10000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x10000 1000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   ------- ----   Beginning of kernel heap
 *   0x10000 8000:  Beginning of user heap.  Can vary with other settings.
 *   0x10001 0000:  End+1 of CPU RAM
 */

MEMORY
{
    /* 256Kb FLASH */

    kflash (rx)      : ORIGIN = 0x00000000, LENGTH = 64K
    uflash (rx)      : ORIGIN = 0x00010000, LENGTH = 448K

    /* 64Kb of SRAM in the CPU block */

    ksram (rwx)      : ORIGIN = 0x10000000, LENGTH = 4K
    usram (rwx)      : ORIGIN = 0x10001000, LENGTH = 60K

    /* Other peripheral memory (free, nothing is linked here) */

    ahbram8_b0a(rwx) : ORIGIN = 0x20000000, LENGTH = 8K
    ahbram8_b0b(rwx) : ORIGIN = 0x20002000, LENGTH = 8K
    ahbram16(rwx)    : ORIGIN = 0x20004000, LENGTH = 16K
}
