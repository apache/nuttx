/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk2mb/include/rx65n_gpio.h
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

#ifndef __BOARDS_RENESAS_RX65N_RX65N_RSK2MB_INCLUDE_RX65N_GPIO_H
#define __BOARDS_RENESAS_RX65N_RX65N_RSK2MB_INCLUDE_RX65N_GPIO_H

#if defined(CONFIG_ARCH_RX65N_RSK2MB)
  #define PHY_STS_REG                  0x10
  #define PHY_STS_REG_LINK             (1 << 0)
  #define PHY_STS_READ_REG             PHY_STS_REG
  #define PHY_STS_BIT_MASK             (0x1)
  #define PHY_STS_SHIFT_COUNT          (0x0)
#else
  #define PHY_STS_REG                  0x1f
  #define PHY_STS_REG_AUTO_NEG         (1 << 12)
  #define PHY_STS_READ_REG             PHY_REG_STATUS
  #define PHY_STS_BIT_MASK             (0x4)
  #define PHY_STS_SHIFT_COUNT          (0x02)
#endif

#if defined(CONFIG_ARCH_BOARD_RX65N_RSK2MB)
  #define PHY_SET_MODE_REG PHY_MII_SET_MODE
#else
  #define PHY_SET_MODE_REG PHY_RMII_SET_MODE
#endif

#if   defined (CONFIG_ARCH_BOARD_RX65N_RSK2MB)
  #define RX65N_MAC_ADDRL 0x00509074
  #define RX65N_MAC_ADDRH 0x0000949c
#else
  #define RX65N_MAC_ADDRL 0x00000000
  #define RX65N_MAC_ADDRH 0x00000000
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 *   Initializes SCI Ports of RX65N RSK2MB
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
 *   Initializes LED Ports of RX65N RSK2MB
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
 *   Ethernet Pheriperal enabling
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
 * Name: sci1_init_port
 *
 * Description:
 *   SCI1 Initialization RX65N RSK2MB
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
 *   SCI2 Initialization RX65N RSK2MB
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
 * Name: sci8_init_port
 *
 * Description:
 *   SCI8 Initialization RX65N RSK2MB
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
 * Name: sci12_init_port
 *
 * Description:
 *   SCI12 Initialization RX65N RSK2MB
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI12
void sci12_init_port(void);
#endif
#endif /* __BOARDS_RENESAS_RX65N_RX65N_RSK2MB_INCLUDE_RX65N_GPIO_H */