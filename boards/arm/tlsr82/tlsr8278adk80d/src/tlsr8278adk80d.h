/****************************************************************************
 * boards/arm/tlsr82/tlsr8278adk80d/src/tlsr8278adk80d.h
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

#ifndef __BOARDS_ARM_TLSR82_TLSR8278ADK80D_SRC_TLSR8278ADK80D_H
#define __BOARDS_ARM_TLSR82_TLSR8278ADK80D_SRC_TLSR8278ADK80D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_NGPIO 4

#define GPIO_IN_CFG          (GPIO_AF_INPUT | GPIO_PUPD_NONE)
#define GPIO_OUT_CFG         (GPIO_AF_OUTPUT | GPIO_PUPD_NONE)
#define GPIO_INT_CFG         (GPIO_AF_INPUT | GPIO_IRQ_NORMAL | GPIO_POL_RISE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int tlsr8278_bringup(void);

int tlsr82_gpio_initialize(void);

#endif /* __BOARDS_ARM_TLSR82_TLSR8278ADK80D_SRC_TLSR8278ADK80D_H */
