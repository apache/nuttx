/****************************************************************************
 * boards/arm/rp23xx/xiao-rp2350/src/rp23xx_pico.h
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

#ifndef __BOARDS_ARM_RP23XX_XIAO_RP2350_SRC_RP23XX_PICO_H
#define __BOARDS_ARM_RP23XX_XIAO_RP2350_SRC_RP23XX_PICO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* LEDs */

#define GPIO_LED1       25 /* The board's LED is connected to this pin */

/* No buttons on board */

int rp23xx_bringup(void);

#ifdef CONFIG_DEV_GPIO
int rp23xx_dev_gpio_init(void);
#endif

#endif /* __BOARDS_ARM_RP23XX_XIAO_RP2350_SRC_RP23XX_PICO_H */
