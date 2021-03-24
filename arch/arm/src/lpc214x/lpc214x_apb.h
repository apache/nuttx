/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_apb.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_APB_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_APB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register address definitions *********************************************/

#define LPC214X_APB_APBDIV   (0xe01fc100)  /* 8-bit R/W APB divider register */

/* Register bit definitions *************************************************/

/* APB divider register */

#define LPC214X_APBDIV_MASK   (0x03)        /* Bit 0:1:    APB divider value */
#define LPC214X_APBDIV_DIV4   (0x00)        /* Bit 0:1=00: APB=PCLK/4 */
#define LPC214X_APBDIV_DIV1   (0x01)        /* Bit 0:1=01: APB=PCLK */
#define LPC214X_APBDIV_DIV2   (0x02)        /* Bit 0:1=10: APB=PCLK/2 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_APB_H */
