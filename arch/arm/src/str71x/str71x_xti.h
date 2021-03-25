/****************************************************************************
 * arch/arm/src/str71x/str71x_xti.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_XTI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_XTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* External Interrupt Controller (XTI) registers ****************************/

#define STR71X_XTI_SR               (STR71X_XTI_BASE + 0x001c)  /* 8-bits wide */
#define STR71X_XTI_CTRL             (STR71X_XTI_BASE + 0x0024)  /* 8-bits wide */
#define STR71X_XTI_MRH              (STR71X_XTI_BASE + 0x0028)  /* 8-bits wide */
#define STR71X_XTI_MRL              (STR71X_XTI_BASE + 0x002c)  /* 8-bits wide */
#define STR71X_XTI_TRH              (STR71X_XTI_BASE + 0x0030)  /* 8-bits wide */
#define STR71X_XTI_TRL              (STR71X_XTI_BASE + 0x0034)  /* 8-bits wide */
#define STR71X_XTI_PRH              (STR71X_XTI_BASE + 0x0038)  /* 8-bits wide */
#define STR71X_XTI_PRL              (STR71X_XTI_BASE + 0x003c)  /* 8-bits wide */

/* Register bit settings ****************************************************/

/* Control register (CTRL) */

#define STR71X_XTICTRL_WKUPINT      (0x01)
#define STR71X_XTICTRL_ID1S         (0x02)
#define STR71X_XTICTRL_STOP         (0x04)

/* Most registers are address by external interrupt line in two 8-bit high
 * and low registers
 */

#define STR71X_XTI_LINE(n)          (1 << (n))
#define STR71X_XTI_LINE0            STR71X_XTI_LINE(0) /* Low register */
#define STR71X_XTI_LINE1            STR71X_XTI_LINE(1)
#define STR71X_XTI_LINE2            STR71X_XTI_LINE(2)
#define STR71X_XTI_LINE3            STR71X_XTI_LINE(3)
#define STR71X_XTI_LINE4            STR71X_XTI_LINE(4)
#define STR71X_XTI_LINE5            STR71X_XTI_LINE(5)
#define STR71X_XTI_LINE6            STR71X_XTI_LINE(6)
#define STR71X_XTI_LINE7            STR71X_XTI_LINE(7)

#define STR71X_XTI_LINE8            STR71X_XTI_LINE(8) /* High register */
#define STR71X_XTI_LINE9            STR71X_XTI_LINE(9)
#define STR71X_XTI_LINE10           STR71X_XTI_LINE(10)
#define STR71X_XTI_LINE11           STR71X_XTI_LINE(11)
#define STR71X_XTI_LINE12           STR71X_XTI_LINE(12)
#define STR71X_XTI_LINE13           STR71X_XTI_LINE(13)
#define STR71X_XTI_LINE14           STR71X_XTI_LINE(14)
#define STR71X_XTI_LINE15           STR71X_XTI_LINE(15)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* _ARCH_ARM_SRC_STR71X_STR71X_XTI_H */
