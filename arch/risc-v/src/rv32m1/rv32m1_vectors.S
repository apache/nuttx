/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_vectors.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

  /* Imported Symbols */

  .extern __start
  .extern exception_common

#ifdef CONFIG_RV32M1_ITCM
  .section .boot, "ax"
  .option norvc
  .org 0x80
  j __start

  .section SECTION_ITCM
__reset:
  lui t0, %hi(__start)
  addi t0, t0, %lo(__start)
  jalr x0, 0(t0)
#endif

  .section .vectors, "ax"
  .option norvc
  j exception_common  /* 0  */
  j exception_common  /* 1  */
  j exception_common  /* 2  */
  j exception_common  /* 3  */
  j exception_common  /* 4  */
  j exception_common  /* 5  */
  j exception_common  /* 6  */
  j exception_common  /* 7  */
  j exception_common  /* 8  */
  j exception_common  /* 9  */
  j exception_common  /* 10 */
  j exception_common  /* 11 */
  j exception_common  /* 12 */
  j exception_common  /* 13 */
  j exception_common  /* 14 */
  j exception_common  /* 15 */
  j exception_common  /* 16 */
  j exception_common  /* 17 */
  j exception_common  /* 18 */
  j exception_common  /* 19 */
  j exception_common  /* 20 */
  j exception_common  /* 21 */
  j exception_common  /* 22 */
  j exception_common  /* 23 */
  j exception_common  /* 24 */
  j exception_common  /* 25 */
  j exception_common  /* 26 */
  j exception_common  /* 27 */
  j exception_common  /* 28 */
  j exception_common  /* 29 */
  j exception_common  /* 30 */
  j exception_common  /* 31 */
#ifdef CONFIG_RV32M1_ITCM
  j __reset
#else
  j __start
#endif
  j exception_common  /* Illegal instruction */
  j exception_common  /* Ecall */
  j exception_common  /* LSU error */
