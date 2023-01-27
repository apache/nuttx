/****************************************************************************
 * boards/arm/samv7/common/include/board_gpio_enc.h
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

#ifndef __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_GPIO_ENC_H
#define __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_GPIO_ENC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sam_gpio.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_enc_init
 *
 * Description:
 *   Initialize and register the ENC driver.
 *
 * Input Parameters:
 *   enca_cfg - ENC_A pin
 *   encb_cfg - ENC_B pin
 *   enca_irq - ENC_A interrupt
 *   encb_irq - ENC_B interrupt
 *
 ****************************************************************************/

int sam_gpio_enc_init(gpio_pinset_t enca_cfg, gpio_pinset_t encb_cfg,
                      int enca_irq, int encb_irq);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_GPIO_ENC_H */
