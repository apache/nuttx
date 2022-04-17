/****************************************************************************
 * arch/arm/src/stm32/stm32_hall3ph.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_HALL3PH_H
#define __ARCH_ARM_SRC_STM32_STM32_HALL3PH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* 3-phase Hall effect sensor configuration */

struct stm32_hall3ph_cfg_s
{
  int gpio_pha;                 /* Hall phase A pincfg */
  int gpio_phb;                 /* Hall phase B pincfg */
  int gpio_phc;                 /* Hall phase C pincfg */
  int samples;                  /* Number of samples taken from GPIO */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hall3ph_initialize
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hall3ph_initialize
 *
 * Description:
 *   Initialize the 3-phase Hall effect driver
 *
 ****************************************************************************/

int stm32_hall3ph_initialize(const char *devpath,
                             struct stm32_hall3ph_cfg_s *cfg);

#endif /* __ARCH_ARM_SRC_STM32_STM32_HALL3PH_H */
