/****************************************************************************
 * arch/arm/src/stm32u5/stm32.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_STM32_H
#define __ARCH_ARM_SRC_STM32U5_STM32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"

/* Peripherals **************************************************************/

#include "chip.h"
#include "stm32_dbgmcu.h"
#include "stm32_flash.h"
#include "stm32_gpio.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"
#include "stm32_spi.h"
#include "stm32_tim.h"
#include "stm32_uart.h"
#include "stm32_lowputc.h"

#endif /* __ARCH_ARM_SRC_STM32U5_STM32_H */
