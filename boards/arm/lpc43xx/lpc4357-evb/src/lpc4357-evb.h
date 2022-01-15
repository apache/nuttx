/****************************************************************************
 * boards/arm/lpc43xx/lpc4357-evb/src/lpc4357-evb.h
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

#ifndef __BOARDS_ARM_LPC43XX_LPC4357_EVB_SRC_LPC4357_EVB_H
#define __BOARDS_ARM_LPC43XX_LPC4357_EVB_SRC_LPC4357_EVB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "lpc43_pinconfig.h"
#include "lpc43_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The LPC4357-EVB has one user-controllable LED labelled D6 controlled by
 * the signal LED_3V3:
 *
 *  ---- ------- -------------
 *  LED  SIGNAL  MCU
 *  ---- ------- -------------
 *   D6  LED_3V3 PE_7 GPIO7[7]
 *  ---- ------- -------------
 *
 * A low output illuminates the LED.
 *
 * Definitions to configure LED pins as GPIOs:
 *
 * - Floating
 * - Normal drive
 * - No buffering, glitch filtering, slew=slow
 */

#define PINCONFIG_LED  PINCONF_GPIO7p7

/* Definitions to configure LED GPIO as outputs */

#define GPIO_LED       (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT7 | GPIO_PIN7)

/* Button definitions *******************************************************/

/* to be provided */

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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC43XX_LPC4357_EVB_SRC_LPC4357_EVB_H */
