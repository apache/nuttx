/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_boot.c
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
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"
#include "imxrt_clockconfig.h"

#include "imxrt1180-evk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

void imxrt_boardinitialize(void)
{
  /* Configure the two user LEDs */

  imxrt_clockall_gpio4();
  imxrt_config_gpio(GPIO_LED1);
  imxrt_config_gpio(GPIO_LED2);
  imxrt_gpio_write(GPIO_LED2, true);
}

/****************************************************************************
 * Name: board_late_initialize
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  imxrt_bringup();
}
#endif
