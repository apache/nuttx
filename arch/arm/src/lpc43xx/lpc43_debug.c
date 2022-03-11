/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_debug.c
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

#include <errno.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc43_pinconfig.h"
#include "lpc43_gpio.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lpc43_pin_dump
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided pin
 *   configuration
 *
 ****************************************************************************/

int lpc43_pin_dump(uint32_t pinconf, const char *msg)
{
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Function:  lpc43_gpio_dump
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided base
 *   address
 *
 ****************************************************************************/

int lpc43_gpio_dump(uint16_t gpiocfg, const char *msg)
{
#warning "Missing logic"
  return -ENOSYS;
}

#endif /* CONFIG_DEBUG_FEATURES */
