/****************************************************************************
 * arch/arm/src/stm32/stm32_comp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/analog/comp.h>
#include <nuttx/analog/ioctl.h>

#include "arm_arch.h"
#include "chip.h"
#include "stm32_comp.h"
#include "stm32_gpio.h"

/* This file is only a thin shell that includes the correct COMP
 * implementation. At this moment STM32 COMP IP versions 1 and 2 are
 * supported.
 *   - STM32 COMP IP version 1: SMT32F33XX
 *   - STM32 COMP IP version 2: SMT32G4XXX
 */

#if defined(CONFIG_STM32_HAVE_IP_COMP_V1)
#  include "stm32_comp_v1.c"
#elif defined(CONFIG_STM32_HAVE_IP_COMP_V2)
#  include "stm32_comp_v2.c"
#else
#  error "STM32 COMP IP version not supported."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/