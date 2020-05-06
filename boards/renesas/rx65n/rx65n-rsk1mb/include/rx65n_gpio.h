/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk1mb/include/rx65n_gpio.h
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

#ifndef __BOARDS_RENESAS_RX65N_RX65N_RSK1MB_INCLUDE_RX65N_GPIO_H
#define __BOARDS_RENESAS_RX65N_RX65N_RSK1MB_INCLUDE_RX65N_GPIO_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sci_port_create
 *
 * Description:
 *   Initializes SCI Ports of RX65N RSK1MB
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
 *   Initializes LED Ports of RX65N RSK1MB
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
 * Name: sci2_init_port
 *
 * Description:
 *   SCI2 Initialization RX65N RSK1MB
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

#endif /* __BOARDS_RENESAS_RX65N_RX65N_RSK1MB_INCLUDE_RX65N_GPIO_H */

