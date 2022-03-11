/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_hardware_setup.c
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
#include "rx65n_cgc.h"
#include "rx65n_icu.h"
#include "rx65n_port.h"
#include "rx65n_sci.h"
#include "chip.h"
#include "up_internal.h"
#include "rx65n_definitions.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void r_ether_port_configuration(void);
void r_ether_pheriperal_enable(void);
void r_usb_port_enable(void);

/****************************************************************************
 * Name: r_system_init
 *
 * Description:
 * Initialization of hardware
 ****************************************************************************/

void r_system_init(void)
{
  /* Enable writing to registers related to operating modes,LPC, CGC
   * and software reset
   */

  SYSTEM.PRCR.WORD = 0xa50b;

  /* Enable writing to MPC pin function control registers */

  MPC.PWPR.BIT.B0WI = 0;
  MPC.PWPR.BIT.PFSWE = 1;

  /* Set peripheral settings */

  r_cgc_create();
  r_port_create();

#ifdef CONFIG_RX65N_EMAC0
  r_ether_port_configuration();
  r_ether_pheriperal_enable();
#endif

#if defined(CONFIG_USBHOST)
  r_usb_port_enable();
#endif

  /* Disable writing to MPC pin function control registers */

  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI = 1;

  /* Enable protection */

  SYSTEM.PRCR.WORD = 0xa500;
}

/****************************************************************************
 * Name: hardware_setup
 *
 * Description:
 * Initialization of hardware
 ****************************************************************************/

int hardware_setup(void)
{
  r_system_init();

  return (1U);
}
