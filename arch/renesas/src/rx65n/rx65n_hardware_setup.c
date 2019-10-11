/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_hardware_setup.c
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in> 
 *           Surya <surya.prakash@tataelxsi.co.in>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "up_arch.h"
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
  r_icu_create();
  r_port_create();

#ifdef CONFIG_RX65N_EMAC0
  r_ether_port_configuration();
  r_ether_pheriperal_enable();
#endif

  /* Disable writing to MPC pin function control registers */

  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI = 1;

  /* Enable protection */

  SYSTEM.PRCR.WORD = 0xa500;

  r_config_icu_software_start();
  r_config_icu_software2_start();
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
