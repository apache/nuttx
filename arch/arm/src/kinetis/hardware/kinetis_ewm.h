/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_ewm.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_EWM_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_EWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_EWM_CTRL_OFFSET    0x0000 /* Control Register */
#define KINETIS_EWM_SERV_OFFSET    0x0001 /* Service Register */
#define KINETIS_EWM_CMPL_OFFSET    0x0002 /* Compare Low Register */
#define KINETIS_EWM_CMPH_OFFSET    0x0003 /* Compare High Register */

/* Register Addresses *******************************************************/

#define KINETIS_EWM_CTRL           (KINETIS_EWM_BASE+KINETIS_EWM_CTRL_OFFSET)
#define KINETIS_EWM_SERV           (KINETIS_EWM_BASE+KINETIS_EWM_SERV_OFFSET)
#define KINETIS_EWM_CMPL           (KINETIS_EWM_BASE+KINETIS_EWM_CMPL_OFFSET)
#define KINETIS_EWM_CMPH           (KINETIS_EWM_BASE+KINETIS_EWM_CMPH_OFFSET)

/* Register Bit Definitions *************************************************/

/* Control Register (8-bit) */

#define EWM_CTRL_EWMEN             (1 << 0)  /* Bit 0:  EWM enable */
#define EWM_CTRL_ASSIN             (1 << 2)  /* Bit 1:  EWM_in's Assertion State Select */
#define EWM_CTRL_INEN              (1 << 3)  /* Bit 2:  Input Enable */
                                             /* Bits 7–3: Reserved */

/* Service Register (8-bit values:  0xb4 followed by 0x2c) */

/* Compare Low Register (8-bit compare low value) */

/* Compare High Register (8-bit compare high value) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_EWM_H */
