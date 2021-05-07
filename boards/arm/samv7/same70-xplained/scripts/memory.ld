/****************************************************************************
 * boards/arm/samv7/same70-xplained/scripts/memory.ld
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

/* The SAMe70Q21 has 2048Kb of FLASH beginning at address 0x0040:0000 and
 * 384Kb of SRAM beginining at 0x2040:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0400:0000 address range.
 *
 * The user space partition will be spanned with a single region of size
 * 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignment
 * requirement also increases.  The sizes below give the largest possible
 * user address spaces (but leave far too much for the OS).
 *
 * The solution to this wasted memory is to (1) use more than one region to
 * span the user spaces, or (2) poke holes in a larger region to trim it
 * to fit better.
 *
 * A detailed memory map for the 384KB SRAM region is as follows:
 *
 *   0x2040 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------ ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x2040 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                 determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                 adjustments for alignment.  Typical is 1KB.
 *   ------ ----   Padded to 4KB
 *   0x2042 0000:  User .data region.  Size is variable.
 *   ------- ----  User .bss region  Size is variable.
 *   0x2044 0000:  Beginning of kernel heap.  Size determined by
 *                 CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------ ----   Beginning of user heap.  Can vary with other settings.
 *   0x2046 0000:  End+1 of mappable internal SRAM
 */

MEMORY
{
  /* 2048KiB of internal FLASH */

  kflash (rx)      : ORIGIN = 0x00400000, LENGTH = 1M
  uflash (rx)      : ORIGIN = 0x00500000, LENGTH = 1M

  /* 384Kb of internal SRAM */

  ksram (rwx)      : ORIGIN = 0x20400000, LENGTH = 128K
  usram (rwx)      : ORIGIN = 0x20420000, LENGTH = 128K
  xsram (rwx)      : ORIGIN = 0x20440000, LENGTH = 128K
}
