/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_port.c
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

#include "rx65n_macrodriver.h"
#include "rx65n_port.h"
#include "arch/board/board.h"
#include "arch/board/rx65n_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r_port_create
 *
 * Description:
 * Port Initialization
 ****************************************************************************/

void r_port_create(void)
{
  led_port_create();
  sci_port_create();
#ifdef CONFIG_USBDEV
  r_usbdev_port_enable();
#endif
}

#ifdef CONFIG_RX65N_EMAC0
void r_ether_port_configuration(void)
{
  /* Port configuration */

  /* Enable LEDs. */

  /* Start with LEDs OFF */

  PORT7.PODR.BIT.B3 = 1;
  PORTG.PODR.BIT.B7 = 1;
  PORTG.PODR.BIT.B6 = 1;
  PORTG.PODR.BIT.B5 = 1;

  /* SET LED pins as outputs */

  PORT7.PDR.BIT.B3 = 1;
  PORTG.PDR.BIT.B7 = 1;
  PORTG.PDR.BIT.B6 = 1;
  PORTG.PDR.BIT.B5 = 1;

  /* Enable Switches */

  /* Set pins as Inputs */

  PORT0.PDR.BIT.B3 = 0;
  PORT0.PDR.BIT.B5 = 0;
  PORT0.PDR.BIT.B7 = 0;

  /* Set port mode registers for switches. */

  PORT0.PMR.BIT.B3 = 0;
  PORT0.PMR.BIT.B5 = 0;
  PORT0.PMR.BIT.B7 = 0;
}
#endif
