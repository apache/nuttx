/****************************************************************************
 * boards/arm/sam34/sam3u-ek/scripts/memory.ld
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The ATSAM3U4E has 256KB of FLASH beginning at address 0x0008:0000,
 * 32KB of SRAM beginning at address 0x2000:0000, and 16Kb of SRAM beginning
 * at address 0x2008:000 (used only for heap).  When booting from FLASH,
 * FLASH memory is aliased to address 0x0000:0000 where the code expects to
 * begin execution by jumping to the entry point in the 0x0800:0000 address
 * range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 64KB of
 * FLASH and 4KB of SRAM.  That, of course, can be optimized as needed (See
 * also boards/open1788/scripts/kernel-space.ld).
 *
 * Alignment of the user space FLASH partition is a critical factor: The
 * user space FLASH partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.  As
 * a consequence, as the user-space increases in size, the alignment
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
  xsram1 (rwx) : ORIGIN = 0x20002000, LENGTH = 24K

  /* 16Kb SRAM */

  sram2 (rwx) : ORIGIN = 0x20080000, LENGTH = 16K
}
