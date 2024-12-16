/****************************************************************************
 * arch/arm/src/imx9/imx9_gpiobase.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include "imx9_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_IMX93)
/* Base address for the GPIO memory mapped registers */

const uintptr_t g_gpio_base[] =
{
  IMX9_GPIO1_BASE,
  IMX9_GPIO2_BASE,
  IMX9_GPIO3_BASE,
  IMX9_GPIO4_BASE,
};
#elif defined(CONFIG_ARCH_CHIP_IMX9_CORTEX_M)
/* Base address for the GPIO memory mapped registers */

const uintptr_t g_gpio_base[] =
{
  IMX9_GPIO1_BASE,
  IMX9_GPIO2_BASE,
  IMX9_GPIO3_BASE,
  IMX9_GPIO4_BASE,
};
#else
#  error Unrecognized i.MX9 architecture
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
