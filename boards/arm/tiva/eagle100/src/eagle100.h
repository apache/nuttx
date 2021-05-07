/****************************************************************************
 * boards/arm/tiva/eagle100/src/eagle100.h
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

#ifndef __BOARDS_ARM_TIVA_EAGLE100_SRC_EAGLE100_H
#define __BOARDS_ARM_TIVA_EAGLE100_SRC_EAGLE100_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SSI modules does this chip support? The LM3S6918 supports 2 SSI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if TIVA_NSSI == 0
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI1
#elif TIVA_NSSI == 1
#  undef CONFIG_TIVA_SSI1
#endif

/* Eagle-100 GPIOs **********************************************************/

/* GPIO for microSD card chip select */

#define SDCCS_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                    GPIO_VALUE_ONE | GPIO_PORTG | 1)
#define LED_GPIO   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTE | 1)

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: lm_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Eagle100 board.
 *
 ****************************************************************************/

void weak_function lm_ssidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_EAGLE100_SRC_EAGLE100_H */
