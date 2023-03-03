/****************************************************************************
 * arch/arm/src/nrf53/nrf53_cpunet.c
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

#include <stdbool.h>

#include "arm_internal.h"
#include "hardware/nrf53_reset.h"
#include "nrf53_cpunet.h"
#ifdef CONFIG_NRF53_NET_GPIO_ALLOW_ALL
#  include "nrf53_gpio.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_cpunet_power
 ****************************************************************************/

void nrf53_cpunet_power(bool enable)
{
  if (enable)
    {
      /* Reset force off bit */

      putreg32(RESET_NETWORK_FORCEOFF_RELEASE, NRF53_RESET_NETWORK_FORCEOFF);
    }
  else
    {
      putreg32(RESET_NETWORK_FORCEOFF_HOLD, NRF53_RESET_NETWORK_FORCEOFF);
    }
}

/****************************************************************************
 * Name: nrf53_cpunet_boot
 ****************************************************************************/

void nrf53_cpunet_boot(void)
{
#ifdef CONFIG_NRF53_NET_GPIO_ALLOW_ALL
  /* Allow all GPIO for the Net core for now */

  nrf53_gpio_cpunet_allow_all();
#else
  /* Or use custom board configuration */

  nrf53_board_gpio_cpunet_allow();
#endif

#ifdef CONFIG_NRF53_NET_POWER_ON_BOOT
  /* Turn on the Net core */

  nrf53_cpunet_power(true);
#endif
}
