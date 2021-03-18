/****************************************************************************
 * boards/arm/kl/teensy-lc/src/teensy-lc.h
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

#ifndef __BOARDS_ARM_KL_TEENSY_LC_SRC_TEENSY_LC_H
#define __BOARDS_ARM_KL_TEENSY_LC_SRC_TEENSY_LC_H

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

#define GPIO_LED (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTC | PIN5)

/* Button definitions *******************************************************/

/* Chip selects *************************************************************/

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
 * Name: kl_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Freedom KL25Z
 *   board.
 *
 ****************************************************************************/

void weak_function kl_spidev_initialize(void);

/****************************************************************************
 * Name: kl_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int kl_pwm_setup(void);
#endif

/****************************************************************************
 * Name: kl_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void kl_led_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KL_TEENSY_LC_SRC_TEENSY_LC_H */
