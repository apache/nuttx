/****************************************************************************
 * arch/arm/src/str71x/str71x_emi.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_EMI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_EMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* External Memory Interfac (EMI) register offset ***************************/

#define STR71X_EMI_BCON0_OFFSET     (0x0000) /* 16-bits wide */
#define STR71X_EMI_BCON1_OFFSET     (0x0004) /* 16-bits wide */
#define STR71X_EMI_BCON2_OFFSET     (0x0008) /* 16-bits wide */
#define STR71X_EMI_BCON3_OFFSET     (0x000c) /* 16-bits wide */

/* External Memory Interfac (EMI) register addresses ************************/

#define STR71X_EMI_BCON0            (STR71X_EMI_BASE + STR71X_EMI_BCON0_OFFSET)
#define STR71X_EMI_BCON1            (STR71X_EMI_BASE + STR71X_EMI_BCON1_OFFSET)
#define STR71X_EMI_BCON2            (STR71X_EMI_BASE + STR71X_EMI_BCON2_OFFSET)
#define STR71X_EMI_BCON3            (STR71X_EMI_BASE + STR71X_EMI_BCON3_OFFSET)

/* Register bit settings ****************************************************/

/* Bank-N configuration register (BCONn) bit definitions */

#define STR71X_EMIBCON_BSIZEMASK    (0x0003) /* Bits 0-1: Bank size */
#define STR71X_EMIBCON_BSIZE8       (0x0000) /*   8-bit */
#define STR71X_EMIBCON_BSIZE16      (0x0001) /*   16-bit */
#define STR71X_EMIBCON_WSMASK       (0x003c) /* Bits 2-5: Wait states */
#define STR71X_EMIBCON_WS0          (0x0000) /*   0 waitstates */
#define STR71X_EMIBCON_WS1          (0x0004) /*   1 waitstates */
#define STR71X_EMIBCON_WS2          (0x0008) /*   2 waitstates */
#define STR71X_EMIBCON_WS3          (0x000c) /*   3 waitstates */
#define STR71X_EMIBCON_WS4          (0x0010) /*   4 waitstates */
#define STR71X_EMIBCON_WS5          (0x0014) /*   5 waitstates */
#define STR71X_EMIBCON_WS6          (0x0018) /*   6 waitstates */
#define STR71X_EMIBCON_WS7          (0x001c) /*   7 waitstates */
#define STR71X_EMIBCON_WS8          (0x0020) /*   8 waitstates */
#define STR71X_EMIBCON_WS9          (0x0024) /*   9 waitstates */
#define STR71X_EMIBCON_WS10         (0x0028) /*   10 waitstates */
#define STR71X_EMIBCON_WS11         (0x002c) /*   11 waitstates */
#define STR71X_EMIBCON_WS12         (0x0030) /*   12 waitstates */
#define STR71X_EMIBCON_WS13         (0x0034) /*   13 waitstates */
#define STR71X_EMIBCON_WS14         (0x0038) /*   14 waitstates */
#define STR71X_EMIBCON_WS15         (0x003c) /*   15 waitstates */
#define STR71X_EMIBCON_ENABLE       (0x8000) /* Bit 15: Bank enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_EMI_H */
