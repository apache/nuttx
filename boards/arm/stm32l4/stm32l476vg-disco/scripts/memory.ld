/****************************************************************************
 * boards/arm/stm32l4/stm32l476vg-disco/scripts/memory.ld
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

/* The STM32L476VG has 1024Kb of FLASH beginning at address 0x0800:0000,
 * 96Kb of SRAM beginning at address 0x2000:0000, and 32Kb of SRAM2 beginning
 * at address 0x1000:0000.  The SRAM2 has special properties (e.g. parity
 * check, separate power domain for retention in low power modes, write
 * protection).  Alas, it is non-contiguous with main SRAM.
 * When booting from FLASH,
 *
 * FLASH memory is aliased to address 0x0000:0000 where the code expects to
 * begin execution by jumping to the entry point in the 0x0800:0000 address
 * range.
 *
 * For MPU support, the kernel-mode NuttX section is assumed to be 128Kb of
 * FLASH and 4Kb of SRAM.  That is an excessive amount for the kernel which
 * should fit into 64KB and, of course, can be optimized as needed (See
 * also boards/arm/stm32l4/stm32l476vg-disco/scripts/kernel-space.ld).  Allowing the
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
 * A detailed memory map for the 96KB SRAM region is as follows:
 *
 *   0x20000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x20000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x20000 2000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   0x20000 4000:  Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------- ----   Beginning of user heap.  Can vary with other settings.
 *   0x20001 8000:  End+1 of CPU RAM
 */

MEMORY
{
  /* 1024Kb FLASH */

  kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
  uflash (rx)      : ORIGIN = 0x08020000, LENGTH = 128K
  xflash (rx)      : ORIGIN = 0x08040000, LENGTH = 768K

  /* 96Kb of contiguous SRAM */

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 8K
  usram (rwx)      : ORIGIN = 0x20002000, LENGTH = 8K
  xsram (rwx)      : ORIGIN = 0x20004000, LENGTH = 80K
}
