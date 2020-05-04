/************************************************************************************
 * arch/arm/src/lpc43xx/chip.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities file */

#include <arch/lpc43xx/chip.h>

/* For each chip supported in chip.h, the following are provided to customize the
 * environment for the specific LPC43XX chip:
 *
 * Define ARMV7M_PERIPHERAL_INTERRUPTS - This is needed by common/arm_vectors.c.
 *   This definition provides the number of "external" interrupt vectors supported
 *   by the specific LPC43 chip.
 *
 *   For the Cortex-M3 core, this should always be equal to the value
 *   LPC43M4_IRQ_NEXTINT defined in include/lpc43xx/irq.h.  For the Cortex-M0
 *   core, this should always be equal to the value LPC43M0_IRQ_NEXTINT defined
 *   in include/lpc43xx/irq.h (At present, only the Cortex-M4 core is supported)
 *
 * Include the chip-specific memory map header file, and
 * Include the chip-specific pin configuration.
 *
 * These header files may or may not be shared between different chips.  That
 * decision depends on the similarity of the chip peripheral.
 */

#if defined(CONFIG_ARCH_CHIP_LPC4310FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4310FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4337JBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4357fet256_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4337FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc4310203050_memorymap.h"
#  include "hardware/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4353fbd208_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4353fet180_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4353fet256_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4357fet180_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4357fbd208_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4357fet256_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4370FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4357fet256_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4337JET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "hardware/lpc435357_memorymap.h"
#  include "hardware/lpc4337jet100_pinconfig.h"
#else
#  error "Unsupported LPC43xx chip"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_H */
