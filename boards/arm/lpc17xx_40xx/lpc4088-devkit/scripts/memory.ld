/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/scripts/memory.ld
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

/* The LPC4088 has 512Kb of FLASH beginning at address 0x0000:0000 and
 * 96KB of total SRAM: 64KB of SRAM in the CPU block beginning at address
 * 0x10000000 and 32Kb of Peripheral SRAM in two banks, 8KB at addresses
 * 0x20000000 bank0 first and 8KB at 0x20002000 at bank0 second. And 16KB
 * at 0x20004000 on bank1.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 256KB of
 * FLASH and 4KB of SRAM.  That, of course, can be optimized as needed (See
 * also boards/arm/lpc17xx_40xx/lpc4088-devkit/scripts/kernel-space.ld);
 * 256KB is probably much more than is needed by the RTOS!
 * That size is selected only because it is available due to alignment
 * issues for the user space FLASH memory.
 *
 * Alignment of the user space FLASH partition is a critical factor: The
 * user space FLASH partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.  As
 * a consequence, as the user-space increases in size, the alignment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 256KB at it would have to be positioned at
 * 0x00400000.  If you change this address, don't forget to change the
 * CONFIG_NUTTX_USERSPACE configuration setting to match and to modify
 * the check in kernel/userspace.c.
 *
 * For the same reasons, the maximum size of the SRAM mapping is limited to
 * 4KB.  Both of these alignment limitations could be reduced by using
 * multiple regions to map the FLASH/SDRAM range or perhaps with some
 * clever use of subregions.
 *
 * A detailed memory map for the 64KB CPU SRAM region is as follows:
 *
 *   0x10000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x10000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x10000 1000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   ------- ----   Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   0x10000 8000:  Beginning of user heap.  Can vary with other settings.
 *   0x10001 0000:  End+1 of CPU RAM
 */

MEMORY
{
  /* 256Kb FLASH */

  kflash (rx)      : ORIGIN = 0x00000000, LENGTH = 256K /* More than needed */
  uflash (rx)      : ORIGIN = 0x00040000, LENGTH = 256K

  /* 64Kb of SRAM in the CPU block */

  ksram (rwx)      : ORIGIN = 0x10000000, LENGTH = 4K   /* May include waste */
  usram (rwx)      : ORIGIN = 0x10001000, LENGTH = 4K
  xsram (rwx)      : ORIGIN = 0x10002000, LENGTH = 24K  /* All used as heap */

  /* Other peripheral memory (free, nothing is linked here) */

  ahbram8_b0a(rwx) : ORIGIN = 0x20000000, LENGTH = 8K
  ahbram8_b0b(rwx) : ORIGIN = 0x20002000, LENGTH = 8K
  ahbram16(rwx)    : ORIGIN = 0x20004000, LENGTH = 16K
}
