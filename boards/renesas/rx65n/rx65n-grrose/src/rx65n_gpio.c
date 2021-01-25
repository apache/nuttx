/****************************************************************************
 * boards/renesas/rx65n/rx65n-grrose/src/rx65n_gpio.c
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
 * LED Port Initialization for RX65N GRROSE Board
 ****************************************************************************/

#if   defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
void led_port_create(void)
{
  /* LED Port initialization of RX65N GRROSE */

  LED_PORTINIT(0);
}

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 * SCI Port Initialization for RX65N GRROSE Board
 ****************************************************************************/

void sci_port_create(void)
{
  /* SCI Port initialization for RX65N-GRROSE */

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
}
#endif

/****************************************************************************
 * Name: r_usbdev_port_enable
 *
 * Description:
 * USB Device enabling
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void r_usbdev_port_enable(void)
{
  /* Set USB0_VBUS pin */

  MPC.P16PFS.BYTE = 0x11;
  PORT1.PMR.BIT.B6 = 1;
}
#endif

/****************************************************************************
 * Name: r_usb_port_enable
 *
 * Description:
 * USB Enabling for RX65N RSK2MB
 ****************************************************************************/

#if defined(CONFIG_USBHOST)
void r_usb_port_enable(void)
{
  /* Set VBUS pin for USB */

  MPC.P16PFS.BYTE = 0x11u;

  /* PORT1.PMR.BYTE |= 0x40; */

  PORT1.PMR.BIT.B6 = 1u;
}
#endif

/****************************************************************************
 * Name: sci0_init_port
 *
 * Description:
 * SCI0 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef  CONFIG_RX65N_SCI0
inline void sci0_init_port(void)
{
  /* Set RXD0 pin (P21) */

  MPC.P21PFS.BYTE   = 0x0au;
  PORT2.PMR.BIT.B1  = 1u;

  /* Set TXD0 pin (P20) */

  PORT2.PODR.BIT.BT0 = 1u;
  MPC.P20PFS.BYTE    = 0x0au;
  PORT2.PDR.BIT.BT0  = 1u;
  PORT2.PMR.BIT.BT0  = 1u;
}
#endif

/****************************************************************************
 * Name: sci1_init_port
 *
 * Description:
 * SCI1 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI1
inline void sci1_init_port(void)
{
  /* Set RXD1 pin (P30) */

  MPC.P30PFS.BYTE   = 0x0au;
  PORT3.PMR.BIT.BT0 = 1u;

  /* Set TXD1 pin (P26) */

  PORT2.PODR.BIT.B6 = 1u;
  MPC.P26PFS.BYTE   = 0x0au;
  PORT2.PDR.BIT.B6  = 1u;
  PORT2.PMR.BIT.B6  = 1u;
}
#endif

/****************************************************************************
 * Name: sci2_init_port
 *
 * Description:
 * SCI2 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI2
inline void sci2_init_port(void)
{
  /* Set RXD2 pin (P12) */

  MPC.P12PFS.BYTE  = 0x0au;
  PORT1.PMR.BIT.B2 = 1u;

  /* Set TXD2 pin (P13) */

  PORT1.PODR.BIT.B3  = 1u;
  MPC.P13PFS.BYTE    = 0x0au;
  PORT1.PDR.BIT.B3   = 1u;
  PORT1.PMR.BIT.B3   = 1u;
}
#endif

/****************************************************************************
 * Name: sci3_init_port
 *
 * Description:
 * SCI3 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI3
inline void sci3_init_port(void)
{
  /* Set RXD3 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD3 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX  = 1u;
   * PORTX.PMR.BIT.BX  = 1u;
   */

  /* Set RXD2 pin (P25) */

  MPC.P25PFS.BYTE  = 0x0au;
  PORT2.PMR.BIT.B5 = 1u;

  /* Set TXD2 pin (P23) */

  PORT2.PODR.BIT.B3  = 1u;
  MPC.P23PFS.BYTE    = 0x0au;
  PORT2.PDR.BIT.B3   = 1u;
  PORT2.PMR.BIT.B3   = 1u;
}
#endif

/****************************************************************************
 * Name: sci5_init_port
 *
 * Description:
 * SCI5 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI5
inline void sci5_init_port(void)
{
  /* Set RXD3 pin (PC2) */

  MPC.PC2PFS.BYTE  = 0x0au;
  PORTC.PMR.BIT.B2 = 1u;

  /* Set TXD3 pin (PC3) */

  PORTC.PODR.BIT.B3 = 1u;
  MPC.PC3PFS.BYTE   = 0x0au;
  PORTC.PDR.BIT.B3  = 1u;
  PORTC.PMR.BIT.B3  = 1u;
}
#endif

/****************************************************************************
 * Name: sci6_init_port
 *
 * Description:
 * SCI6 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI6
inline void sci6_init_port(void)
{
  /* Set RXD6 pin (P33) */

  MPC.P33PFS.BYTE  = 0x0au;
  PORT3.PMR.BIT.B3 = 1u;

  /* Set TXD6 pin (P32) */

  PORT3.PODR.BIT.B2 = 1u;
  MPC.P32PFS.BYTE   = 0x0au;
  PORT3.PDR.BIT.B2  = 1u;
  PORT3.PMR.BIT.B2  = 1u;
}
#endif

/****************************************************************************
 * Name: sci8_init_port
 *
 * Description:
 * SCI8 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI8
inline void sci8_init_port(void)
{
  /* Set RXD8 pin (PC6) */

  MPC.PC6PFS.BYTE  = 0x0au;
  PORTC.PMR.BIT.B6 = 1u;

  /* Set TXD8 pin (PC7) */

  PORTC.PODR.BIT.B7 = 1u;
  MPC.PC7PFS.BYTE   = 0x0au;
  PORTC.PDR.BIT.B7  = 1u;
  PORTC.PMR.BIT.B7  = 1u;
}
#endif

/****************************************************************************
 * Name: rspi_pinconfig
 *
 * Description: RSPI pinconfiguration for channel
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Description:
 *RSPI pin(SCK,MOSI and MISO) configuration
 ****************************************************************************/

#ifdef CONFIG_RX65N_RSPI
void rspi_pinconfig(int bus)
{
  /* Set RSPI signal ports to peripheral mode */

  switch (bus)
    {
      case RX65N_RSPI_CHANNEL0:
#ifdef CONFIG_RX65N_RSPI0

        /* Configure RSPCKA */

        MPC.PC5PFS.BYTE = 0x0d;
        PORTC.PMR.BIT.B5 = 1;

        /* Configure MOSIA */

        MPC.PC6PFS.BYTE = 0x0d; /* This config will block SCI8 function */
        PORTC.PMR.BIT.B6 = 1;

        /* Configure MISOA */

        MPC.PC7PFS.BYTE = 0x0d; /* This config will block SCI8 function */
        PORTC.PMR.BIT.B7 = 1;

        /* Configure SSLA0 */

        MPC.PC4PFS.BYTE = 0x0d;
        PORTC.PMR.BIT.B4 = 1;
#endif
        break;

      case RX65N_RSPI_CHANNEL1:
#ifdef CONFIG_RX65N_RSPI1

        /* Configure RSPCKB */

        MPC.PE5PFS.BYTE = 0x0d;
        PORTE.PMR.BIT.B5 = 1;

        /* Configure MOSIB */

        MPC.PE6PFS.BYTE = 0x0d;
        PORTE.PMR.BIT.B6 = 1;

        /* Configure MISOB */

        MPC.PE7PFS.BYTE = 0x0d;
        PORTE.PMR.BIT.B7 = 1;

        /* Configure SSLB0 */

        MPC.PE4PFS.BYTE = 0x0d;
        PORTE.PMR.BIT.B4 = 1;
#endif
        break;

    case RX65N_RSPI_CHANNEL2:
#ifdef CONFIG_RX65N_RSPI2

        /* Configure RSPCKC */

        MPC.PD3PFS.BYTE = 0x0d;
        PORTD.PMR.BIT.B3 = 1;

        /* Configure MOSIC */

        MPC.PD1PFS.BYTE = 0x0d;
        PORTD.PMR.BIT.B1 = 1;

        /* Configure MISOC */

        MPC.PD2PFS.BYTE = 0x0d;
        PORTD.PMR.BIT.B2 = 1;

        /* Configure SSLC0 */

        MPC.PD4PFS.BYTE = 0x0d;
        PORTD.PMR.BIT.B4 = 1;
#endif
        break;

      default:
        break;
    }
}
#endif

/****************************************************************************
 * Name: riic0_init_port
 *
 * Description:
 * RIIC0 Initialization RX65N GRROSE
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC0
inline void riic0_init_port(void)
{
  /* Set SCL0 pin (P12) */

  MPC.P12PFS.BYTE  = 0x0fu;
  PORT1.PMR.BIT.B2 = 1u;

  /* Set SDA0 pin (P13) */

  MPC.P13PFS.BYTE   = 0x0fu;
  PORT1.PMR.BIT.B3  = 1u;
}
#endif

/****************************************************************************
 * Name: riic1_init_port
 *
 * Description:
 * RIIC1 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC1
inline void riic1_init_port(void)
{
  /* Set SCL0 pin (P21) */

  MPC.P21PFS.BYTE  = 0x0fu;
  PORT2.PMR.BIT.B1 = 1u;

  /* Set SDA0 pin (P20) */

  MPC.P20PFS.BYTE   = 0x0fu;
  PORT2.PMR.BIT.BT0  = 1u;
}
#endif

/****************************************************************************
 * Name: riic2_init_port
 *
 * Description:
 * RIIC2 Initialization RX65N RSK2MB
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC2
inline void riic2_init_port(void)
{
  /* Set SCL0 pin (P16) */

  MPC.P16PFS.BYTE  = 0x0fu;
  PORT1.PMR.BIT.B6 = 1u;

  /* Set SDA0 pin (P17) */

  MPC.P17PFS.BYTE   = 0x0fu;
  PORT1.PMR.BIT.B7  = 1u;
}
#endif
#endif /* CONFIG_ARCH_BOARD_RX65N_GRROSE */