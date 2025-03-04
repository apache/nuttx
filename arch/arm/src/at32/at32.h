/****************************************************************************
 * arch/arm/src/at32/at32.h
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

#ifndef __ARCH_ARM_SRC_AT32_AT32_H
#define __ARCH_ARM_SRC_AT32_AT32_H

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
#include "at32_gpio.h"
#include "at32_uart.h"
#include "at32_exti.h"
#include "at32_flash.h"
#include "at32_dma.h"
#include "at32_pwr.h"
#include "at32_rcc.h"
#include "at32_lowputc.h"
#include "at32_adc.h"
#include "at32_can.h"
#include "at32_i2c.h"
#include "at32_rtc.h"
#include "at32_sdio.h"
#include "at32_spi.h"
#include "at32_tim.h"
#include "at32_eth.h"
#include "at32_wdg.h"
#include "at32_dbgmcu.h"

#endif /* __ARCH_ARM_SRC_AT32_AT32_H */
