/****************************************************************************
 * boards/renesas/rx65n/rx65n-grrose/include/rx65n_gpio.h
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

#ifndef __BOARDS_RENESAS_RX65N_RX65N_GRROSE_INCLUDE_RX65N_GPIO_H
#define __BOARDS_RENESAS_RX65N_RX65N_GRROSE_INCLUDE_RX65N_GPIO_H

#if defined(CONFIG_ARCH_RX65N_GRROSE)
  #define PHY_STS_REG                  0x1f
  #define PHY_STS_REG_AUTO_NEG         (1 << 12)
  #define PHY_STS_READ_REG             PHY_REG_STATUS
  #define PHY_STS_BIT_MASK             (0x4)
  #define PHY_STS_SHIFT_COUNT          (0x02)
#else
  #define PHY_STS_REG			       0x10
  #define PHY_STS_REG_LINK             (1 << 0)
  #define PHY_STS_READ_REG             PHY_STS_REG
  #define PHY_STS_BIT_MASK             (0x1)
  #define PHY_STS_SHIFT_COUNT          (0x0)
#endif

#if defined(CONFIG_ARCH_RX65N_GRROSE)
  #define PHY_SET_MODE_REG PHY_RMII_SET_MODE
#else
  #define PHY_SET_MODE_REG PHY_MII_SET_MODE
#endif

#if defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
  #define RX65N_MAC_ADDRL 0x00509074
  #define RX65N_MAC_ADDRH 0x0000989c
#else
  #define RX65N_MAC_ADDRL 0x00000000
  #define RX65N_MAC_ADDRH 0x00000000
#endif

/* RSPI channel number */

#define RX65N_RSPI_CHANNEL0 0
#define RX65N_RSPI_CHANNEL1 1
#define RX65N_RSPI_CHANNEL2 2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 *   Initializes SCI Ports of RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sci_port_create(void);

/****************************************************************************
 * Name: led_port_create
 *
 * Description:
 *   Initializes LED Ports of RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void led_port_create(void);

/****************************************************************************
 * Name: r_ether_pheriperal_enable
 *
 * Description:
 *   Ethernet Peripheral enabling
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC0
void r_ether_pheriperal_enable(void);
#endif

/****************************************************************************
 * Name: r_usbdev_port_enable
 *
 * Description:
 *   USB device port settings
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void r_usbdev_port_enable(void);
#endif

/****************************************************************************
 * Name: sci0_init_port
 *
 * Description:
 *   SCI0 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef  CONFIG_RX65N_SCI0
void sci0_init_port(void);
#endif

/****************************************************************************
 * Name: sci1_init_port
 *
 * Description:
 *   SCI1 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI1
void sci1_init_port(void);
#endif

/****************************************************************************
 * Name: sci2_init_port
 *
 * Description:
 *   SCI2 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI2
void sci2_init_port(void);
#endif

/****************************************************************************
 * Name: sci3_init_port
 *
 * Description:
 *   SCI3 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI3
void sci3_init_port(void);
#endif

/****************************************************************************
 * Name: sci5_init_port
 *
 * Description:
 *   SCI5 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI5
void sci5_init_port(void);
#endif

/****************************************************************************
 * Name: sci6_init_port
 *
 * Description:
 *   SCI6 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI6
void sci6_init_port(void);
#endif

/****************************************************************************
 * Name: sci8_init_port
 *
 * Description:
 *   SCI8 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI8
void sci8_init_port(void);
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
  void rspi_pinconfig(int bus);
#endif

/****************************************************************************
 * Name: riic0_init_port
 *
 * Description:
 *   RIIC0 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC0
void riic0_init_port(void);
#endif

/****************************************************************************
 * Name: riic1_init_port
 *
 * Description:
 *   RIIC1 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC1
void riic1_init_port(void);
#endif

/****************************************************************************
 * Name: riic2_init_port
 *
 * Description:
 *   RIIC2 Initialization RX65N GRROSE
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_RIIC2
void riic2_init_port(void);
#endif
#endif /* __BOARDS_RENESAS_RX65N_RX65N_GRROSE_INCLUDE_RX65N_GPIO_H */

