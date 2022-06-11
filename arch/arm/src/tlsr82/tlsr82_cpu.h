/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_cpu.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_TLSR82_CPU_H
#define __ARCH_ARM_SRC_TLSR82_TLSR82_CPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  LDO_MODE      = 0x40, /* LDO mode */
  DCDC_LDO_MODE = 0x41, /* DCDC_LDO mode */
  DCDC_MODE     = 0x43, /* DCDC mode (16pin chip not suported) */
} power_mode_t;

typedef enum
{
  EXTERNAL_XTAL_24M = 0,
  EXTERNAL_XTAL_48M = 1,
} xtal_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void tlsr82_cpu_wakeup_init(power_mode_t power_mode, xtal_t xtal);

#endif /* __ARCH_ARM_SRC_TLSR82_TLSR82_CPU_H */
