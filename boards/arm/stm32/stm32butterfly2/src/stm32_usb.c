/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_usb.c
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

#include <debug.h>

#include "stm32_gpio.h"

#include "stm32_butterfly2.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usb_initialize
 *
 * Description:
 *   Initializes USB pins
 ****************************************************************************/

void stm32_usb_initialize(void)
{
  uinfo("INFO: Initializing usb otgfs gpio pins\n");

  stm32_configgpio(GPIO_OTGFS_VBUS);
  stm32_configgpio(GPIO_OTGFS_PWRON);
}
