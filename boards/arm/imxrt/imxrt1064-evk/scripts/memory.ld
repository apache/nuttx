/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/scripts/memory.ld
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

/* The i.MXRT1064-EVK has 8MiB of Hyper FLASH beginning at address,
 * 0x0060:0000, Up to 512Kb of DTCM RAM beginning at 0x2000:0000, and 1MiB OCRAM
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
  /* 8MiB of HyperFLASH */

  kflash (rx)  : ORIGIN = 0x60000000, LENGTH = 1M
  uflash (rx)  : ORIGIN = 0x60200000, LENGTH = 1M
  flash  (rx)  : ORIGIN = 0x60400000, LENGTH = 6M

  /* 1MiB of OCRAM */

  dtcm   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
  kocram (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
  uocram (rwx) : ORIGIN = 0x20240000, LENGTH = 512K
}
