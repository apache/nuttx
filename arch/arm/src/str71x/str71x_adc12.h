/****************************************************************************
 * arch/arm/src/str71x/str71x_adc12.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H
#define __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC12 registers **********************************************************/

#define STR71X_ADC12_DATA0    (STR71X_ADC12_BASE + 0x0000)    /* 16-bits wide */
#define STR71X_ADC12_DATA1    (STR71X_ADC12_BASE + 0x0008)    /* 16-bits wide */
#define STR71X_ADC12_DATA2    (STR71X_ADC12_BASE + 0x0010)    /* 16-bits wide */
#define STR71X_ADC12_DATA3    (STR71X_ADC12_BASE + 0x0018)    /* 16-bits wide */
#define STR71X_ADC12_CSR      (STR71X_ADC12_BASE + 0x0020)    /* 16-bits wide */
#define STR71X_ADC12_CPR      (STR71X_ADC12_BASE + 0x0030)    /* 16-bits wide */

/* Register bit settings ****************************************************/

/* ADC12 Conversion modes */

#define STR71X_ADC12_SINGLE   (0)
#define STR71X_ADC12_ROUND    (1)

/* ADC12 Channels */

#define STR71X_ADC12_CHANNEL0 (0x00)
#define STR71X_ADC12_CHANNEL1 (0x10)
#define STR71X_ADC12_CHANNEL2 (0x20)
#define STR71X_ADC12_CHANNEL3 (0x30)

/* ADC12 control status register */

#define STR71X_ADC12_DA0      (0x0001)
#define STR71X_ADC12_DA1      (0x0002)
#define STR71X_ADC12_DA2      (0x0004)
#define STR71X_ADC12_DA3      (0x0008)
#define STR71X_ADC12_OR       (0x2000)

/* Interrupt bits for channel n */

#define STR71X_ADC12_IT0      (0x0100)
#define STR71X_ADC12_IT1      (0x0200)
#define STR71X_ADC12_IT2      (0x0400)
#define STR71X_ADC12_IT3      (0x0800)
#define STR71X_ADC12_ITALL    (0x0f00)

/* Mode selection */

#define STR71X_ADC12_MODE     (0x0040)

/* Converter configuration */

#define STR71X_ADC12_START    (0x0020)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_ADC12_H */
