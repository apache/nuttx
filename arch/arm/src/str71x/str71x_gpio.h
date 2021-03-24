/****************************************************************************
 * arch/arm/src/str71x/str71x_gpio.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H
#define __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO register offsets ****************************************************/

#define STR71X_GPIO_PC0_OFFSET (0x0000)  /* 16-bits wide */
#define STR71X_GPIO_PC1_OFFSET (0x0004)  /* 16-bits wide */
#define STR71X_GPIO_PC2_OFFSET (0x0008)  /* 16-bits wide */
#define STR71X_GPIO_PD_OFFSET  (0x000c)  /* 16-bits wide */

/* GPIO register addresses **************************************************/

#define STR71X_GPIO_PC0(b)     ((b) + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO_PC1(b)     ((b) + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO_PC2(b)     ((b) + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO_PD(b)      ((b) + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO0_PC0       (STR71X_GPIO0_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO0_PC1       (STR71X_GPIO0_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO0_PC2       (STR71X_GPIO0_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO0_PD        (STR71X_GPIO0_BASE + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO1_PC0       (STR71X_GPIO1_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO1_PC1       (STR71X_GPIO1_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO1_PC2       (STR71X_GPIO1_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO1_PD        (STR71X_GPIO1_BASE + STR71X_GPIO_PD_OFFSET)

#define STR71X_GPIO2_PC0       (STR71X_GPIO2_BASE + STR71X_GPIO_PC0_OFFSET)
#define STR71X_GPIO2_PC1       (STR71X_GPIO2_BASE + STR71X_GPIO_PC1_OFFSET)
#define STR71X_GPIO2_PC2       (STR71X_GPIO2_BASE + STR71X_GPIO_PC2_OFFSET)
#define STR71X_GPIO2_PD        (STR71X_GPIO2_BASE + STR71X_GPIO_PD_OFFSET)

/* Register bit settings ****************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_GPIO_H */
