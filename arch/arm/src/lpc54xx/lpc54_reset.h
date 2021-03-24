/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_reset.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_RESET_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the correct definitions for the configured chip */

#if defined(CONFIG_ARCH_FAMILY_LPC546XX)
#  include "lpc546x_reset.h"
#else
#  error "Unsupported LPC54 architecture"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_reset
 *
 * Description:
 *   Reset the selected peripheral
 *
 ****************************************************************************/

void lpc54_reset(uintptr_t setreg, uintptr_t clrreg,
                 uintptr_t statreg, uint32_t mask);

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_RESET_H */
