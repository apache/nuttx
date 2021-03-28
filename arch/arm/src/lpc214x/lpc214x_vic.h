/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_vic.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_VIC_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_VIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All VIC registers are 32-bits wide */

#define vic_getreg(o)   getreg32(LPC214X_VIC_BASE+(o))
#define vic_putreg(v,o) putreg32((v),LPC214X_VIC_BASE+(o))

/* Vector Control Register bit definitions */

#define LPC214X_VECTCNTL_IRQMASK  (0x0000001f)
#define LPC214X_VECTCNTL_IRQSHIFT (0)
#define LPC214X_VECTCNTL_ENABLE   (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_VIC_H */
