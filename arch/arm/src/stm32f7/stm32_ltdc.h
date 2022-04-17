/****************************************************************************
 * arch/arm/src/stm32f7/stm32_ltdc.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_LTDC_H
#define __ARCH_ARM_SRC_STM32F7_STM32_LTDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ltdcreset
 *
 * Description:
 *   Reset LTDC via APB2RSTR
 *
 ****************************************************************************/

void stm32_ltdcreset(void);

/****************************************************************************
 * Name: stm32_ltdcinitialize
 *
 * Description:
 *   Initialize the ltdc controller
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int stm32_ltdcinitialize(void);

/****************************************************************************
 * Name: stm32_ltdcuninitialize
 *
 * Description:
 *   Uninitialize the ltdc controller
 *
 ****************************************************************************/

void stm32_ltdcuninitialize(void);

/****************************************************************************
 * Name: stm32_ltdcgetvplane
 *
 * Description:
 *   Get video plane reference used by framebuffer interface
 *
 * Parameter:
 *   vplane - Video plane
 *
 * Returned Value:
 *   Video plane reference
 *
 ****************************************************************************/

struct fb_vtable_s *stm32_ltdcgetvplane(int vplane);

/****************************************************************************
 * Name: stm32_lcd_backlight
 *
 * Description:
 *   If CONFIG_STM32F7_LCD_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_LCD_BACKLIGHT
void stm32_backlight(bool blon);
#endif
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_LTDC_H */
