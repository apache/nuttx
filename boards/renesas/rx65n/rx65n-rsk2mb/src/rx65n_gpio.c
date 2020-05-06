/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk2mb/src/rx65n_gpio.c
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
 * LED Port Initialization for RX65N RSK2MB Board
 ****************************************************************************/

#if   defined (CONFIG_ARCH_BOARD_RX65N_RSK2MB) 
void led_port_create(void)
{
  /* LED Port initialization of RX65N RSK2MB */

  /* LED_PORTINIT(0); */

  PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT7.PMR.BYTE   = 0x00u;
  PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
}

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 * SCI Port Initialization for RX65N RSK2MB Board
 ****************************************************************************/

void sci_port_create(void)
{
  /* SCI Port initialization for RX65N-RSK2MB */

  PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
  PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
  PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
  PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
  PORT5.PMR.BYTE   = 0x00u;
  PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                     _80_PDR5_DEFAULT;
  PORTJ.PMR.BYTE   = 0x00u;
  PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;

  /* General Purpose I/O Port initialization for RX65N-RSK2MB */

  PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
  PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
  PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
  PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
  PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT0.PMR.BYTE   = 0x00u;
  PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                     _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
  PORT9.PMR.BYTE   = 0x00u;
  PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
}

/****************************************************************************
 * Name: r_ether_pheriperal_enable
 *
 * Description:
 * Ethernet Pheriperal enabling
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC0
void r_ether_pheriperal_enable(void)
{
  /* Set ET0_TX_CLK pin */

  MPC.PC4PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B4 = 1u;

  /* Set ET0_RX_CLK pin */

  MPC.P76PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B6 = 1u;

  /* Set ET0_TX_EN pin */

  MPC.P80PFS.BYTE = 0x11u;
  PORT8.PMR.BIT.BT0 = 1u;

  /* Set ET0_ETXD3 pin */

  MPC.PC6PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B6 = 1u;

  /* Set ET0_ETXD2 pin */

  MPC.PC5PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B5 = 1u;

  /* Set ET0_ETXD1 pin */

  MPC.P82PFS.BYTE = 0x11u;
  PORT8.PMR.BIT.B2 = 1u;

  /* Set ET0_ETXD0 pin */

  MPC.P81PFS.BYTE = 0x11u;
  PORT8.PMR.BIT.B1 = 1u;

  /* Set ET0_TX_ER pin */

  MPC.PC3PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B3 = 1u;

  /* Set ET0_RX_DV pin */

  MPC.PC2PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B2 = 1u;

  /* Set ET0_ERXD3 pin */

  MPC.PC0PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.BT0 = 1u;

  /* Set ET0_ERXD2 pin */

  MPC.PC1PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B1 = 1u;

  /* Set ET0_ERXD1 pin */

  MPC.P74PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B4 = 1u;

  /* Set ET0_ERXD0 pin */

  MPC.P75PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B5 = 1u;

  /* Set ET0_RX_ER pin */

  MPC.P77PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B7 = 1u;

  /* Set ET0_CRS pin */

  MPC.P83PFS.BYTE = 0x11u;
  PORT8.PMR.BIT.B3 = 1u;

  /* Set ET0_COL pin */

  MPC.PC7PFS.BYTE = 0x11u;
  PORTC.PMR.BIT.B7 = 1u;

  /* Set ET0_MDC pin */

  MPC.P72PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B2 = 1u;

  /* Set ET0_MDIO pin */

  MPC.P71PFS.BYTE = 0x11u;
  PORT7.PMR.BIT.B1 = 1u;

  /* Set ET0_LINKSTA pin */

  MPC.P54PFS.BYTE = 0x11u;
  PORT5.PMR.BIT.B4 = 1u;

  /* Set ET0_LINKSTA pin */

  MPC.P34PFS.BYTE = 0x11u;
  PORT3.PMR.BIT.B4 = 1u;

  /* Set VBUS pin for USB */

  /* Referred from r_usb_basic_pinset.c */

  MPC.P16PFS.BYTE = 0x12u;

  /* PORT1.PMR.BYTE |= 0x40; */

  PORT1.PMR.BIT.B6 = 1u;

  /* Set USB0_OVRCURA pin */

  MPC.P14PFS.BYTE = 0x12u;
  PORT1.PMR.BIT.B4 = 1u;
}
#endif

/****************************************************************************
 * Name: sci1_init_port
 *
 * Description:
 * SCI1 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI1
inline void sci1_init_port(void)
{
  /* Set RXD1 pin (PF2) */

  MPC.PF2PFS.BYTE   = 0x0au;
  PORTF.PMR.BIT.B2  = 1u;

  /* Set TXD1 pin (PF1) */

  PORTF.PODR.BIT.B1 = 1u;
  MPC.PF1PFS.BYTE   = 0x0au;
  PORTF.PDR.BIT.B1  = 1u;
  PORTF.PMR.BIT.B1  = 1u;
}
#endif

/****************************************************************************
 * Name: sci2_init_port
 *
 * Description:
 * SCI2 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI2
inline void sci2_init_port(void)
{
  /* Set RXD2 pin (P52) */

  MPC.P52PFS.BYTE   = 0x0au;
  PORT5.PMR.BIT.B2  = 1u;

  /* Set TXD2 pin (P50) */

  PORT5.PODR.BIT.BT0 = 1u;
  MPC.P50PFS.BYTE    = 0x0au;
  PORT5.PDR.BIT.BT0  = 1u;
  PORT5.PMR.BIT.BT0  = 1u;
}
#endif

/****************************************************************************
 * Name: sci8_init_port
 *
 * Description:
 * SCI8 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI8
inline void sci8_init_port(void)
{
  /* Set RXD8 pin (PJ1) */

  MPC.PJ1PFS.BYTE  = 0x0au;
  PORTJ.PMR.BIT.B1 = 1u;

  /* Set TXD8 pin (PJ2) */

  PORTJ.PODR.BIT.B2 = 1u;
  MPC.PJ2PFS.BYTE   = 0x0au;
  PORTJ.PDR.BIT.B2  = 1u;
  PORTJ.PMR.BIT.B2  = 1u;
}
#endif

/****************************************************************************
 * Name: sci12_init_port
 *
 * Description:
 * SCI12 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI12
inline void sci12_init_port(void)
{
  /* Set RXD12 pin */

  MPC.PE2PFS.BYTE = 0x0cu;
  PORTE.PMR.BYTE |= 0x04u;

  /* Set TXD12 pin */

  PORTE.PODR.BYTE |= 0x02u;
  MPC.PE1PFS.BYTE = 0x0cu;
  PORTE.PDR.BYTE |= 0x02u;

  /* Set RXD12 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD12 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif
#endif /* CONFIG_ARCH_BOARD_RX65N_RSK2MB */