/************************************************************************************
 * arch/arm/src/lpc55xx/chip/lpc55_gpio.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_GPIO_H
#define __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/lpc55_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define LPC55_GPIO_PORT0            0
#define LPC55_GPIO_PORT1            1
#define LPC55_GPIO_NPORTS           2

/* Register offsets *****************************************************************/
/* Byte and word access to individual pins */

#define LPC55_GPIO_B_OFFSET(p)      (0x0000 + (p))
#define LPC55_GPIO_W_OFFSET(p)      (0x1000 + ((p) << 2))

/* Word access to individual port regisers */

#define LPC55_GPIO_PORT_OFFSET(n)   ((n) << 2)
#define LPC55_GPIO_DIR_OFFSET(n)    (0x2000 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_MASK_OFFSET(n)   (0x2080 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_PIN_OFFSET(n)    (0x2100 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_MPIN_OFFSET(n)   (0x2180 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_SET_OFFSET(n)    (0x2200 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_CLR_OFFSET(n)    (0x2280 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_NOT_OFFSET(n)    (0x2300 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_DIRSET_OFFSET(n) (0x2380 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_DIRCLR_OFFSET(n) (0x2400 + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_DIRNOT_OFFSET(n) (0x2480 + LPC55_GPIO_PORT_OFFSET(n))

/* Register addresses ***************************************************************/

/* Byte and word access to individual pins */

#define LPC55_GPIO_B(p)             (LPC55_GPIO_BASE + LPC55_GPIO_B_OFFSET(p))
#define LPC55_GPIO_W(p)             (LPC55_GPIO_BASE + LPC55_GPIO_W_OFFSET(p))

/* Word access to individual port regisers */

#define LPC55_GPIO_PORT(n)          (LPC55_GPIO_BASE + LPC55_GPIO_PORT_OFFSET(n))
#define LPC55_GPIO_DIR(n)           (LPC55_GPIO_BASE + LPC55_GPIO_DIR_OFFSET(n))
#define LPC55_GPIO_MASK(n)          (LPC55_GPIO_BASE + LPC55_GPIO_MASK_OFFSET(n))
#define LPC55_GPIO_PIN(n)           (LPC55_GPIO_BASE + LPC55_GPIO_PIN_OFFSET(n))
#define LPC55_GPIO_MPIN(n)          (LPC55_GPIO_BASE + LPC55_GPIO_MPIN_OFFSET(n))
#define LPC55_GPIO_SET(n)           (LPC55_GPIO_BASE + LPC55_GPIO_SET_OFFSET(n))
#define LPC55_GPIO_CLR(n)           (LPC55_GPIO_BASE + LPC55_GPIO_CLR_OFFSET(n))
#define LPC55_GPIO_NOT(n)           (LPC55_GPIO_BASE + LPC55_GPIO_NOT_OFFSET(n))
#define LPC55_GPIO_DIRSET(n)        (LPC55_GPIO_BASE + LPC55_GPIO_DIRSET_OFFSET(n))
#define LPC55_GPIO_DIRCLR(n)        (LPC55_GPIO_BASE + LPC55_GPIO_DIRCLR_OFFSET(n))
#define LPC55_GPIO_DIRNOT(n)        (LPC55_GPIO_BASE + LPC55_GPIO_DIRNOT_OFFSET(n))

/* Register bit definitions *********************************************************/

/* Port registers are all bit arrays with one bit corresponding each of the 32 pins
 * of the port.
 */

#define GPIO_PORT_BIT(n)            (1 << ((n) & 31))

#endif /* __ARCH_ARM_SRC_LPC55XX_HARDWARE_LPC55_GPIO_H */
