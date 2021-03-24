/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_gint.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC54_GINT_CTRL_OFFSET         0x0000  /* GPIO grouped interrupt control */
#define LPC54_GINT_PORT_POL0_OFFSET    0x0020  /* GPIO grouped interrupt port 0 polarity */
#define LPC54_GINT_PORT_POL0_OFFSET    0x0024  /* GPIO grouped interrupt port 1 polarity */
#define LPC54_GINT_PORT_ENA0_OFFSET    0x0040  /* GPIO grouped interrupt port 0 enable */
#define LPC54_GINT_PORT_ENA1_OFFSET    0x0044  /* GPIO grouped interrupt port 1 enable */

/* Register addresses *******************************************************/

#define LPC54_GINT0_CTRL               (LPC54_GINT0_BASE + LPC54_GINT_CTRL_OFFSET)
#define LPC54_GINT0_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL0_OFFSET)
#define LPC54_GINT0_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL1_OFFSET)
#define LPC54_GINT0_PORT_ENA0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA0_OFFSET)
#define LPC54_GINT0_PORT_ENA1          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA1_OFFSET)

#define LPC54_GINT1_CTRL               (LPC54_GINT0_BASE + LPC54_GINT_CTRL_OFFSET)
#define LPC54_GINT1_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL0_OFFSET)
#define LPC54_GINT1_PORT_POL0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_POL1_OFFSET)
#define LPC54_GINT1_PORT_ENA0          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA0_OFFSET)
#define LPC54_GINT1_PORT_ENA1          (LPC54_GINT0_BASE + LPC54_GINT_PORT_ENA1_OFFSET)

/* Register bit definitions *************************************************/

/* GPIO grouped interrupt control */

#define GINT_CTRL_INT                  (1 << 0)  /* Bit 0:  Group interrupt status */
#define GINT_CTRL_COMB                 (1 << 1)  /* Bit 1:  Combine enabled inputs for group interrupt 0 */
#define GINT_CTRL_TRIG                 (1 << 2)  /* Bit 2"  Group interrupt trigger 0 */

/* GPIO grouped interrupt port 0/1 polarity */

#define GINT_PORT_POL0(n)              (1 << (n)) /* Configure pin polarity of port0 pins for group interrupt */
#define GINT_PORT_POL1(n)              (1 << (n)) /* Configure pin polarity of port1 pins for group interrupt */

/* GPIO grouped interrupt port 0/1 enable */

#define GINT_PORT_ENA0(n)              (1 << (n)) /* Enable port0 pin for group interrupt */
#define GINT_PORT_ENA1(n)              (1 << (n)) /* Enable port1 pin for group interrupt */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_GINT_H */
