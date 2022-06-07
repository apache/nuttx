/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/nucleo-wl55jc.h
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

#ifndef __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_SRC_NUCLEO_WL55JC_H
#define __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_SRC_NUCLEO_WL55JC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC             1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

#define GPIO_LED_GREEN (GPIO_PORTB|GPIO_PIN9 |GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz)
#define GPIO_LED_RED   (GPIO_PORTB|GPIO_PIN11|GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz)
#define GPIO_LED_BLUE  (GPIO_PORTB|GPIO_PIN15|GPIO_OUTPUT|GPIO_PULLUP|GPIO_SPEED_50MHz)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_leds_initialize
 *
 * Description:
 *   Initialize LEDs
 *
 ****************************************************************************/

void board_leds_initialize(void);

#endif /* __BOARDS_ARM_STM32WL5_NUCLEO_WL55JC_SRC_NUCLEO_WL55JC_H */
