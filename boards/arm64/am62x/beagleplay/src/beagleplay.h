/****************************************************************************
 * boards/arm64/am62x/beagleplay/src/beagleplay.h
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

#ifndef __BOARDS_ARM64_AM62X_BEAGLEPLAY_SRC_BEAGLEPLAY_H
#define __BOARDS_ARM64_AM62X_BEAGLEPLAY_SRC_BEAGLEPLAY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: beagleplay_bringup
 *
 * Description:
 *   Bring up board-level drivers and subsystems.  Called from
 *   board_late_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
int beagleplay_bringup(void);
#endif

/****************************************************************************
 * Name: beagleplay_led_initialize
 *
 * Description:
 *   Configure the user LEDs as GPIO outputs.  Called from
 *   am62x_board_initialize() when CONFIG_ARCH_LEDS is set.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void beagleplay_led_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_AM62X_BEAGLEPLAY_SRC_BEAGLEPLAY_H */
