/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/stub_start.S
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

/* Entry of the flash programming stub.  The mask ROM UART loader calls it
 * at its load address (0x12000) as a command handler; it never returns.
 * The host patches the parameter block before loading the stub.
 */

  .section .start, "ax"
  .global _start
  .global g_stub_param

_start:
  j       1f

  .balign 4
g_stub_param:
  .word   0x54534241                    /* "ABST" */
  .word   921600                        /* UART baud, patched by the host */

1:
  sw      zero, 0x544(zero)             /* PICEN = 0 */
  li      t0, 3
  sw      t0, 0x530(zero)               /* PICCON: IE and unit off */

  la      t0, _sbss                     /* Clear .bss */
  la      t1, _ebss
2:
  bgeu    t0, t1, 3f
  sw      zero, 0(t0)
  addi    t0, t0, 4
  j       2b

3:
  la      sp, _stack_top
  call    stub_main
4:
  j       4b
