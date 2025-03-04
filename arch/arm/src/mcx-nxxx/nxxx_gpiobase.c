/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_gpiobase.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nxxx_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_N236)
/* Base address for the GPIO memory mapped registers */

const uintptr_t g_gpio_base[] =
{
  NXXX_GPIO0_BASE,
  NXXX_GPIO1_BASE,
  NXXX_GPIO2_BASE,
  NXXX_GPIO3_BASE,
  NXXX_GPIO4_BASE,
  NXXX_GPIO5_BASE,
};
#else
#  error Unrecognized NXXx architecture
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
