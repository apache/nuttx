/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H
#define __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "a64_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LEDs *********************************************************************/

/* Green LED on PD18 */

#define LED1 (PIO_OUTPUT | PIO_PULL_NONE | \
              PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
              PIO_OUTPUT_SET | PIO_PORT_PIOD | PIO_PIN18)

/* Red LED on PD19 */

#define LED2 (PIO_OUTPUT | PIO_PULL_NONE | \
              PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
              PIO_OUTPUT_SET | PIO_PORT_PIOD | PIO_PIN19)

/* Blue LED on PD20 */

#define LED3 (PIO_OUTPUT | PIO_PULL_NONE | \
              PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
              PIO_OUTPUT_SET | PIO_PORT_PIOD | PIO_PIN20)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int pinephone_bringup(void);
#endif

/****************************************************************************
 * Name: pinephone_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pinephone_led_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H */
