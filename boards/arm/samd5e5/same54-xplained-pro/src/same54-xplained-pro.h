/****************************************************************************
 * boards/arm/samd5e5/same54-xplained-pro/src/same54-xplained-pro.h
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

#ifndef __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_SRC_SAME54_XPLAINED_PRO_H
#define __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_SRC_SAME54_XPLAINED_PRO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Metro-M4 GPIOs ***********************************************************/

/* LEDs
 *
 *   The SAME54 Xplained Pro has three LEDs,
 *   but only one is controllable by software:
 *
 *   1. LED0 near the edge of the board
 *
 *
 *   ----------------- -----------
 *   SAMD5E5           FUNCTION
 *   ----------------- -----------
 *   PC18              GPIO output
 *
 */

#define PORT_LED0 (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                   PORTC | PORT_PIN18)
/* Ethernet *****************************************************************/

/* PHY pins:
 *
 *   -------- ----------------- -----------
 *   PHY      SAMD5E5           FUNCTION
 *   -------- ----------------- -----------
 *   Reset    PD12              GPIO output
 *   IRQ      PC21              GPIO input
 */

#define PORT_PHY_RESET  (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                         PORTD | PORT_PIN12)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_led_pminitialize
 *
 * Description:
 *   Register LED power management features.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void sam_led_pminitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_SRC_SAME54_XPLAINED_PRO_H */
