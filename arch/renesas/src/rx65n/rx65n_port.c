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
#if  defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB)

  /* LED_PORTINIT(0); */

  PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
  PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
  PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
  PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
  PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
  PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
  PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
  PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
  PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT0.PMR.BYTE   = 0x00u;
  PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                     _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
  PORT5.PMR.BYTE   = 0x00u;
  PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                     _80_PDR5_DEFAULT;
  PORT7.PMR.BYTE   = 0x00u;
  PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
  PORT9.PMR.BYTE   = 0x00u;
  PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
  PORTJ.PMR.BYTE   = 0x00u;
  PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif   defined (CONFIG_ARCH_BOARD_RX65N_RSK2MB)

  /* LED_PORTINIT(0); */

  PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
  PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
  PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
  PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
  PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
  PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
  PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
  PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
  PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
  PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
  PORT0.PMR.BYTE   = 0x00u;
  PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                     _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
  PORT5.PMR.BYTE   = 0x00u;
  PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                     _80_PDR5_DEFAULT;
  PORT7.PMR.BYTE   = 0x00u;
  PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
  PORT9.PMR.BYTE   = 0x00u;
  PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
  PORTJ.PMR.BYTE   = 0x00u;
  PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif   defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
  LED_PORTINIT(0);

  /* SCI0(UART)  direction */

  PORT2.PODR.BIT.B2 = 0;  PORT2.PMR.BIT.B2 = 0;   PORT2.PDR.BIT.B2 = 1;

  /* SCI2(UART)  direction */

  PORT1.PODR.BIT.B4 = 0;  PORT1.PMR.BIT.B4 = 0;   PORT1.PDR.BIT.B4 = 1;

  /* SCI5(UART)  direction */

  PORTC.PODR.BIT.B4 = 0;  PORTC.PMR.BIT.B4 = 0;   PORTC.PDR.BIT.B4 = 1;

  /* SCI6(UART)  direction */

  PORT3.PODR.BIT.B4 = 0;  PORT3.PMR.BIT.B4 = 0;   PORT3.PDR.BIT.B4 = 1;

  /* SCI8(RS485) direction */

  PORTC.PODR.BIT.B5 = 0;  PORTC.PMR.BIT.B5 = 0;   PORTC.PDR.BIT.B5 = 1;
#else
#  error "No Selection for PORT definition in rx65n_port.c"
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

void r_ether_pheriperal_enable(void)
{
#if defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB)

  /* TODO */

#elif defined(CONFIG_ARCH_BOARD_RX65N_RSK2MB)

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

#elif defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)

  /* Set ET0_MDC(PA4_ET_MDC) pin */

  MPC.PA4PFS.BYTE = 0x11u;
  PORTA.PMR.BIT.B4 = 1u;

  /* Set ET0_MDIO(PA3_ET_MDIO) pin */

  MPC.PA3PFS.BYTE = 0x11u;
  PORTA.PMR.BIT.B3 = 1u;

  /* Set REF50CK0 (PB2_ET_CLK) pin */

  MPC.PB2PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B2 = 1u;

  /* Set RMII0_CRS_DV(PB7_ET_CRS) pin */

  MPC.PB7PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B7 = 1u;

  /* Set RMII0_RXD0(PB1_ET_RXD0) pin */

  MPC.PB1PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B1 = 1u;

  /* Set RMII0_RXD1(PB0_ET_RXD1) pin */

  MPC.PB0PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.BT0 = 1u;

  /* Set RMII0_RX_ER(PB3_ET_RXER) pin */

  MPC.PB3PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B3 = 1u;

  /* Set RMII0_ETXD0(PB5_ET_TXD0) pin */

  MPC.PB5PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B5 = 1u;

  /* Set RMII0_ETXD1(PB6_ET_TXD1) pin */

  MPC.PB6PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B6 = 1u;

  /* Set RMII0_TXD_EN(PB4_ET_TXEN) pin */

  MPC.PB4PFS.BYTE = 0x12u;
  PORTB.PMR.BIT.B4 = 1u;

  /* Set RXD2 pin */

  MPC.P52PFS.BYTE = 0x0au;
  PORT5.PMR.BIT.B2 = 1u;

  /* Set TXD2 pin */

  PORT5.PODR.BYTE |= 0x01u;
  MPC.P50PFS.BYTE = 0x0au;
  PORT5.PDR.BYTE |= 0x01u;

  /* Set ET0_LINKSTA(PA5_ET_LINK) pin */

  MPC.PA5PFS.BYTE = 0x11u;
  PORTA.PMR.BIT.B5 = 1u;

  /* Set ETHER reset(PA6_ET_RST) pin */

  MPC.PA6PFS.BYTE = 0x12u;
  PORTA.PMR.BIT.B6 = 1u;
#endif
}
#endif
