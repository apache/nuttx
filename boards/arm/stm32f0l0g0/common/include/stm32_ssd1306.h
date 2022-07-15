/****************************************************************************
 * boards/arm/stm32f0l0g0/common/include/stm32_ssd1306.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_COMMON_INCLUDE_STM32_SSD1306_H
#define __BOARDS_ARM_STM32F0L0G0_COMMON_INCLUDE_STM32_SSD1306_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_ssd1306_initialize
 *
 * Description:
 *   Initialize and register the device
 *
 * Input Parameters:
 *   busno - The I2C or SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ssd1306_initialize(int busno);

/****************************************************************************
 * Name: board_ssd1306_getdev
 *
 * Description:
 *   Get the SSD1306 device driver instance
 *
 * Returned Value:
 *   Pointer to the instance
 *
 ****************************************************************************/

struct lcd_dev_s *board_ssd1306_getdev(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_STM32F0L0G0_COMMON_INCLUDE_STM32_SSD1306_H */
