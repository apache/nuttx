/****************************************************************************
 * arch/arm/include/s32k1xx/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_S32K1XX_IRQ_H
#define __ARCH_ARM_INCLUDE_S32K1XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_S32K14X)
#  include <arch/chip/s32k14x_irq.h>
#elif defined(CONFIG_ARCH_CHIP_S32K11X)
#  include <arch/chip/s32k11x_irq.h>
#else
#  error Unrecognized S32K1XX part
#endif

#endif /* __ARCH_ARM_INCLUDE_S32K1XX_IRQ_H */
