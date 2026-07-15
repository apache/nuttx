/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32vw55x_boardinitialize
 *
 * Description:
 *   Board-specific initialization, provided by the board logic.
 *
 ****************************************************************************/

void gd32vw55x_boardinitialize(void);

/****************************************************************************
 * Name: gd32vw55x_irq_set_trigger
 *
 * Description:
 *   Select the ECLIC trigger mode of an interrupt source (see
 *   ECLIC_INTATTR_TRIG_* in hardware/gd32vw55x_eclic.h).
 *
 ****************************************************************************/

void gd32vw55x_irq_set_trigger(int irq, uint8_t trig);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_H */
