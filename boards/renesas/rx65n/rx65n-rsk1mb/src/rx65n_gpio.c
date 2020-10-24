/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk1mb/src/rx65n_gpio.c
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
 * Name: led_port_create
 *
 * Description:
 * Port Initialization for RX65N RSK1MB Board
 ****************************************************************************/

#if   defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB)
void led_port_create(void)
{
  /* LED Port initialization of RX65N RSK1MB */

  /* LED_PORTINIT(0); */

  PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
  PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
  PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
  PORT0.PMR.BYTE   = 0x00u;
  PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                     _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
}

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 * SCI Port Initialization for RX65N RSK2MB Board
 ****************************************************************************/

void sci_port_create(void)
{
  /* SCI Port initialization for RX65N-RSK1MB */

  PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
  PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
  PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
  PORT5.PMR.BYTE   = 0x00u;
  PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                     _80_PDR5_DEFAULT;

  /* General Purpose I/O Port initialization for RX65N-RSK1MB */

  PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
  PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
  PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT7.PMR.BYTE   = 0x00u;
  PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
  PORT9.PMR.BYTE   = 0x00u;
  PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
  PORTJ.PMR.BYTE   = 0x00u;
  PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
}

/****************************************************************************
 * Name: r_ether_pheriperal_enable
 *
 * Description:
 * Ethernet Peripheral enabling
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC0
void r_ether_pheriperal_enable(void)
{
  /* TODO */
}
#endif

/****************************************************************************
 * Name: sci2_init_port
 *
 * Description:
 * SCI2 Initialization RX65N RSK1MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI2
inline void sci2_init_port(void)
{
  /* Set RXD2 pin (P52) */

  MPC.P52PFS.BYTE  = 0x0au;
  PORT5.PMR.BIT.B2 = 1u;

  /* Set TXD2 pin (P50) */

  PORT5.PODR.BIT.BT0 = 1u;
  MPC.P50PFS.BYTE    = 0x0au;
  PORT5.PDR.BIT.BT0  = 1u;
  PORT5.PMR.BIT.BT0  = 1u;
}
#endif
#endif /* CONFIG_ARCH_BOARD_RX65N_RSK1MB*/
