/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_systemreset.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Reset the chip with a 1 ms watchdog timeout.  After the reset the mask
 *   ROM listens for its UART loader for a short time before booting from
 *   flash, so this is also how the host gets back to the ROM loader.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  up_irq_save();
  putreg32(WDTCON_RESET_1MS, AB32VG1_WDTCON);

  for (; ; );
}
