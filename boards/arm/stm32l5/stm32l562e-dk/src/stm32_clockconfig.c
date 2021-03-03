/****************************************************************************
 * boards/arm/stm32l5/stm32l562e-dk/src/stm32_clockconfig.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   Currently the STM32L562E-DK board support is restricted to running NuttX
 *   in the Non-Secure domain together with TrustedFirmware-M (TFM).  In this
 *   setup the clock configuration is done by TFM, not by NuttX.  Thus, the
 *   board's configuration sets CONFIG_ARCH_BOARD_STM32L5_CUSTOM_CLOCKCONFIG
 *   to avoid the standard clock config logic to run and instead do just
 *   nothing in this function.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_BOARD_STM32L5_CUSTOM_CLOCKCONFIG)
void stm32l5_board_clockconfig(void)
{
}
#endif
