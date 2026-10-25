/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/boot_start.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Entry of the stage-1 boot loader.  The mask ROM copies it from the boot
 * header area of the flash to its load address and calls it.  If no valid
 * NuttX image is found it returns to the mask ROM.
 */

  .section .start, "ax"
  .global _start

_start:
  la      t0, g_rom_ctx                 /* Remember how to get back */
  sw      sp, 0(t0)
  sw      ra, 4(t0)

  la      sp, _stack_top                /* Private stack (ICRAM) */

  la      t0, _sbss                     /* Clear .bss */
  la      t1, _ebss
1:
  bgeu    t0, t1, 2f
  sw      zero, 0(t0)
  addi    t0, t0, 4
  j       1b

2:
  call    boot_main                     /* Returns the entry point or 0 */
  beqz    a0, 3f
  jr      a0                            /* NuttX __start, no return */

3:
  la      t0, g_rom_ctx                 /* Back to the mask ROM */
  lw      sp, 0(t0)
  lw      ra, 4(t0)
  ret

  .data
  .balign 4
g_rom_ctx:
  .word   0, 0
