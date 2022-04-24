/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_irq.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_IRQ_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_MASK_REG                   REG_ADDR32(0x640)
#define IRQ_PRIO_REG                   REG_ADDR32(0x644)
#define IRQ_SRC_REG                    REG_ADDR32(0x648)
#define IRQ_SRC3_REG                   REG_ADDR8(0x64a)
#define IRQ_EN_REG                     REG_ADDR8(0x643)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_IRQ_H */
