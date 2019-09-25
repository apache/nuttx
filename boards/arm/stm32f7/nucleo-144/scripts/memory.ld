/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/scripts/memory.ld
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

/* The STM32F746NGH6 has 1024Kb of main FLASH memory.  This FLASH memory can
 * be accessed from either the AXIM interface at address 0x0800:0000 or from
 * the ITCM interface at address 0x0020:0000.
 *
 * Additional information, including the option bytes, is available at at
 * FLASH at address 0x1ff0:0000 (AXIM) or 0x0010:0000 (ITCM).
 *
 * In the STM32F746NGH6, two different boot spaces can be selected through
 * the BOOT pin and the boot base address programmed in the BOOT_ADD0 and
 * BOOT_ADD1 option bytes:
 *
 *   1) BOOT=0: Boot address defined by user option byte BOOT_ADD0[15:0].
 *      ST programmed value: Flash on ITCM at 0x0020:0000
 *   2) BOOT=1: Boot address defined by user option byte BOOT_ADD1[15:0].
 *      ST programmed value: System bootloader at 0x0010:0000
 *
 * NuttX does not modify these option bytes.  On the unmodified STM32F746G
 * DISCO board, the BOOT0 pin is at ground so by default, the STM32 will boot
 * to address 0x0020:0000 in ITCM FLASH.
 *
 * The STM32F746NGH6 also has 320Kb of data SRAM (in addition to ITCM SRAM).
 * SRAM is split up into three blocks:
 *
 *   1)  64Kb of DTCM SRM beginning at address 0x2000:0000
 *   2) 240Kb of SRAM1 beginning at address 0x2001:0000
 *   3)  16Kb of SRAM2 beginning at address 0x2004:c000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 128Kb of
 * FLASH and 4Kb of SRAM.  That is an excessive amount for the kernel which
 * should fit into 64KB and, of course, can be optimized as needed (See
 * also boards/stm32f746g-disco/scripts/kernel-space.ld).  Allowing the
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
 *
 * For the same reasons, the maximum size of the SRAM mapping is limited to
 * 4KB.  Both of these alignment limitations could be reduced by using
 * multiple regions to map the FLASH/SDRAM range or perhaps with some
 * clever use of subregions.
 *
 * A detailed memory map for the 112KB SRAM region is as follows:
 *
 *   0x20001 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x20001 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x20001 1000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   0x20001 2000:  Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------- ----   Beginning of user heap.  Can vary with other settings.
 *   0x20004 c000:  End+1 of SRAM1
 */

MEMORY
{
  /* ITCM boot address */

  itcm  (rwx) : ORIGIN = 0x00200000, LENGTH = 1024K

  /* 1024KB FLASH */

  kflash (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx) : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx) : ORIGIN = 0x08040000, LENGTH = 768K

  /* 240KB of contiguous SRAM1 */

  ksram (rwx) : ORIGIN = 0x20010000, LENGTH = 4K
  usram (rwx) : ORIGIN = 0x20011000, LENGTH = 4K
  xsram (rwx) : ORIGIN = 0x20012000, LENGTH = 240K - 8K

  /* DTCM SRAM */

  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  sram2 (rwx) : ORIGIN = 0x2004c000, LENGTH = 16K
}
