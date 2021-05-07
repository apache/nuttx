/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_pinmux.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PINMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is just a wrapper around pin muxing header files for the select
 * S32K1xx family.
 */

#if defined(CONFIG_ARCH_CHIP_S32K116)
#  include "hardware/s32k116_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_S32K118)
#  include "hardware/s32k118_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_S32K142)
#  include "hardware/s32k142_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_S32K144)
#  include "hardware/s32k144_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_S32K146)
#  include "hardware/s32k146_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_S32K148)
#  include "hardware/s32k148_pinmux.h"
#else
#  error "No pin multiplexing for this S32K1xx part"
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
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PINMUX_H */
