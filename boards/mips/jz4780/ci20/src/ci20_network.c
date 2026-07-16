/****************************************************************************
 * boards/mips/jz4780/ci20/src/ci20_network.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_DM90x0)

#include <debug.h>
#include <arch/board/board.h>

#include <nuttx/net/dm90x0.h>

#include "chip.h"
#include "mips_internal.h"

#include "jz4780_gpio.h"
#include "ci20.h"

/* PE19 is the ethernet interrupt input pin (ETHNET_INT in schematic) */

#define ETHNET_INT   (GPIO_MODE_INTR_RISE | GPIO_PORTE | GPIO_PIN19)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void mips_netinitialize(void)
{
  jz4780_configgpio(ETHNET_INT);

  (void)dm9x_initialize();
}

#endif /* CONFIG_NET && CONFIG_NET_DM90x0 */
