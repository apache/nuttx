/****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/scripts/memory.ld
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

/* The i.MXRT1050-EVK has 64Mb of Hyper FLASH beginning at address,
 * 0x0060:0000, 512Kb of DTCM RAM beginning at 0x2000:0000, and 512Kb OCRAM
 * beginning at 0x2020:0000.  Neither DTCM or SDRAM are used in this
 * configuration.
 *
 * The user and kernel space partitions will be spanned with a single
 * region of size 2**n bytes.  As a consequence, as the partitions increase
 * in size, the alignment requirement also increases.  The sizes below give
 * the largest possible user address spaces (but leave far too much for the
 * OS).
 *
 * The solution to this wasted memory is to (1) use more than one region to
 * span the user spaces, or (2) poke holes in a larger region (via sub-
 * regions) to trim it to fit better.
 *
 * A detailed memory map for the 512KB SRAM region is as follows:
 *
 *   0x2020 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------ ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x2020 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                 determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                 adjustments for alignment.  Typical is 1KB.
 *   ------ ----   Padded to 4KB
 *   0x2040 0000:  User .data region.  Size is variable.
 *   ------- ----  User .bss region  Size is variable.
 *   0x2042 0000:  Beginning of kernel heap.  Size determined by
 *                 CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------ ----   Beginning of user heap.  Can vary with other settings.
 *   0x2080 0000:  End+1 of mappable OCRAM
 */

/* Specify the memory areas */

MEMORY
{
  /* 64Mb of HyperFLASH */

  kflash (rx)  : ORIGIN = 0x60000000, LENGTH = 1M
  uflash (rx)  : ORIGIN = 0x60200000, LENGTH = 1M
  flash  (rx)  : ORIGIN = 0x60400000, LENGTH = 62M

  /* 512Kb of OCRAM */

  dtcm   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
  kocram (rwx) : ORIGIN = 0x20200000, LENGTH = 256K
  uocram (rwx) : ORIGIN = 0x20240000, LENGTH = 256K
}
