/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf53_cpunet_boot.c
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

#include "nrf53_gpio.h"
#include "nrf53_cpunet.h"

#include <arch/board/board.h>

#ifndef CONFIG_NRF53_NET_GPIO_ALLOW_ALL
/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf53_board_gpio_cpunet_allow(void)
{
  /* UART0 pins */

  nrf53_gpio_cpunet_allow(BOARD_NET_UART0_RX_PIN);
  nrf53_gpio_cpunet_allow(BOARD_NET_UART0_TX_PIN);
}
#endif
