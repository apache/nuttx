/****************************************************************************
 * arch/risc-v/src/espressif/esp_head.S
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

#include <nuttx/config.h>

#include <arch/irq.h>

#include "chip.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global  __start

/****************************************************************************
 * Section: .text
 ****************************************************************************/

  .section .text

/****************************************************************************
 * Name: __start
 ****************************************************************************/

__start:
  .option push
  .option norelax

  /* Set stack pointer to the idle thread stack */

  lui  sp, %hi(ESP_IDLESTACK_TOP)
  addi sp, sp, %lo(ESP_IDLESTACK_TOP)

  /* Set gp pointer */

  la   gp, __global_pointer$

  /* Disable all interrupts (i.e. timer, external) in mstatus */

  csrw mstatus, zero

  .option pop

  /* Initialize the Machine Trap-Vector */

  lui  t0, %hi(_vector_table)
  addi t0, t0, %lo(_vector_table)
  csrw  mtvec, t0

  /* Jump to __esp_start */

  jal  x1, __esp_start

  /* We shouldn't return from __esp_start */

  ret
