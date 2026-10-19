/****************************************************************************
 * arch/arm/src/gd32w51x/hardware/gd32w51x_trng.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TRNG_H
#define __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TRNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32w51x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32W51X_TRNG_CTL_OFFSET    0x0000  /* Control register */
#define GD32W51X_TRNG_STAT_OFFSET   0x0004  /* Status register */
#define GD32W51X_TRNG_DATA_OFFSET   0x0008  /* Data register */

/* Register addresses *******************************************************/

#define GD32W51X_TRNG_CTL   (GD32W51X_TRNG_BASE + GD32W51X_TRNG_CTL_OFFSET)
#define GD32W51X_TRNG_STAT  (GD32W51X_TRNG_BASE + GD32W51X_TRNG_STAT_OFFSET)
#define GD32W51X_TRNG_DATA  (GD32W51X_TRNG_BASE + GD32W51X_TRNG_DATA_OFFSET)

/* Register bit definitions *************************************************/

/* Control register */

#define TRNG_CTL_TRNGEN              (1 << 2)  /* TRNG enable */
#define TRNG_CTL_IE                  (1 << 3)  /* Interrupt enable */

/* Status register */

#define TRNG_STAT_DRDY               (1 << 0)  /* Random data ready */
#define TRNG_STAT_CECS               (1 << 1)  /* Clock error status */
#define TRNG_STAT_SECS               (1 << 2)  /* Seed error current status */
#define TRNG_STAT_CEIF               (1 << 5)  /* Clock error int flag */
#define TRNG_STAT_SEIF               (1 << 6)  /* Seed error interrupt flag */

#endif /* __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TRNG_H */
