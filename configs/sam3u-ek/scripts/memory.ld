/****************************************************************************
 * configs/sam3u-ek/scripts/user-space.ld
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

/* The AT91SAM3U-4 has 256KB of FLASH beginning at address 0x0008:0000,
 * 32KB of SRAM beginning at address 0x2000:0000, and 16Kb of SRAM beginning
 * at address 0x2008:000 (used only for heap).  When booting from FLASH,
 * FLASH memory is aliased to address 0x0000:0000 where the code expects to
 * begin execution by jumping to the entry point in the 0x0800:0000 address
 * range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 64KB of
 * FLASH and 4KB of SRAM.  That, of course, can be optimized as needed (See
 * also configs/open1788/scripts/kernel-space.ld).
 *
 * Alignment of the user space FLASH partition is a critical factor: The
 * user space FLASH partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.  As
 * a consequence, as the user-space increases in size, the alignmment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 128KB at it would have to be positioned at
 * 0x000a0000.  If you change this address, don't forget to change the
 * CONFIG_NUTTX_USERSPACE configuration setting to match and to modify
 * the check in kernel/userspace.c.
 *
 * For the same reasons, the maximum size of the SRAM mapping is limited to
 * 4KB.  Both of these alignment limitations could be reduced by using
 * multiple regions to map the FLASH/SDRAM range or perhaps with some
 * clever use of subregions.
 *
 * A detailed memory map for the 16Kb SRAM region is as follows:
 *
 *   0x20000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   ------- ----   Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x20000 1000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   ------- ----   Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------- ----   Beginning of user heap.  Can vary with other settings.
 *   0x20000 8000:  End+1 of CPU RAM
 */

MEMORY
{
    /* 256Kb FLASH */

    kflash (rx)  : ORIGIN = 0x00080000, LENGTH = 64K
    uflash (rx)  : ORIGIN = 0x00090000, LENGTH = 64K
    xflash (rx)  : ORIGIN = 0x000a0000, LENGTH = 128K

    /* 32Kb SRAM */

    ksram1 (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
    usram1 (rwx) : ORIGIN = 0x20001000, LENGTH = 4K
    xsram1 (rwx) : ORIGIN = 0x20001000, LENGTH = 24K

    /* 16Kb SRAM */

    sram2 (rwx) : ORIGIN = 0x20080000, LENGTH = 16K
}
