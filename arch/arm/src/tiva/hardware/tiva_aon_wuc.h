/****************************************************************************
 * arch/arm/src/tiva/hardware/tiva_aon_wuc.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_AON_WUC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_AON_WUC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the pin mapping file for the specific Tiva/Stellaris/SimpleLink
 * chip
 */

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C) || \
    defined(CONFIG_ARCH_CHIP_CC13X2)
  /* These architectures do not support the AON WUC block */
#elif defined(CONFIG_ARCH_CHIP_CC13X0)
#  include "hardware/cc13x0/cc13x0_aon_wuc.h"
#else
#  error "Unsupported Tiva/Stellaris/SimpleLink AON WUC"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_AON_WUC_H */
