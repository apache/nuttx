/****************************************************************************
 * arch/arm/src/xmc4/xmc4_ecat.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "xmc4_ecat.h"

#include <arch/board/board.h>
#include <xmc4_gpio.h>

#include "arm_internal.h"
#include "hardware/xmc4_pinmux.h"
#include "hardware/xmc4_scu.h"
#include "debug.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GET_GPIO(p)   ((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK))

/* MDIO configuration */

#if   GET_GPIO(ECAT_MDO) == (GPIO_PORT0 | GPIO_PIN12)
    #define MDIO  SCU_ECAT0CON_MDIOA
#elif GET_GPIO(ECAT_MDO) == (GPIO_PORT4 | GPIO_PIN2)
    #define MDIO  SCU_ECAT0CON_MDIOB
#elif GET_GPIO(ECAT_MDO) == (GPIO_PORT9 | GPIO_PIN7)
    #define MDIO  SCU_ECAT0CON_MDIOC
#else
    #error "EtherCAT : Unknown MDIO configuration"
#endif

#if defined(CONFIG_XMC4_ECAT_P0)

/* Port 0 RX D0 */

#if   GET_GPIO(ECAT_P0_RXD0) == (GPIO_PORT1 | GPIO_PIN4)
    #define P0_RXD0  SCU_ECAT0CON_PORT0_RXD0A
#elif GET_GPIO(ECAT_P0_RXD0) == (GPIO_PORT5 | GPIO_PIN0)
    #define P0_RXD0  SCU_ECAT0CON_PORT0_RXD0B
#elif GET_GPIO(ECAT_P0_RXD0) == (GPIO_PORT7 | GPIO_PIN4)
    #define P0_RXD0  SCU_ECAT0CON_PORT0_RXD0C
#else
    #error "EtherCAT : Unknown P0_RXD0 configuration"
#endif

/* Port 0 RX D1 */

#if   GET_GPIO(ECAT_P0_RXD1) == (GPIO_PORT1 | GPIO_PIN5)
    #define P0_RXD1  SCU_ECAT0CON_PORT0_RXD1A
#elif GET_GPIO(ECAT_P0_RXD1) == (GPIO_PORT5 | GPIO_PIN1)
    #define P0_RXD1  SCU_ECAT0CON_PORT0_RXD1B
#elif GET_GPIO(ECAT_P0_RXD1) == (GPIO_PORT7 | GPIO_PIN5)
    #define P0_RXD1  SCU_ECAT0CON_PORT0_RXD1C
#else
    #error "EtherCAT : Unknown P0_RXD1 configuration"
#endif

/* Port 0 RX D2 */

#if   GET_GPIO(ECAT_P0_RXD2) == (GPIO_PORT1 | GPIO_PIN10)
    #define P0_RXD2  SCU_ECAT0CON_PORT0_RXD2A
#elif GET_GPIO(ECAT_P0_RXD2) == (GPIO_PORT5 | GPIO_PIN2)
    #define P0_RXD2  SCU_ECAT0CON_PORT0_RXD2B
#elif GET_GPIO(ECAT_P0_RXD2) == (GPIO_PORT7 | GPIO_PIN6)
    #define P0_RXD2  SCU_ECAT0CON_PORT0_RXD2C
#else
    #error "EtherCAT : Unknown P0_RXD2 configuration"
#endif

/* Port 0 RX D3 */

#if   GET_GPIO(ECAT_P0_RXD3) == (GPIO_PORT1 | GPIO_PIN11)
    #define P0_RXD3  SCU_ECAT0CON_PORT0_RXD3A
#elif GET_GPIO(ECAT_P0_RXD3) == (GPIO_PORT5 | GPIO_PIN7)
    #define P0_RXD3  SCU_ECAT0CON_PORT0_RXD3B
#elif GET_GPIO(ECAT_P0_RXD3) == (GPIO_PORT7 | GPIO_PIN7)
    #define P0_RXD3  SCU_ECAT0CON_PORT0_RXD3C
#else
    #error "EtherCAT : Unknown P0_RXD3 configuration"
#endif

/* Port 0 RX CLK */

#if   GET_GPIO(ECAT_P0_RX_CLK) == (GPIO_PORT1 | GPIO_PIN1)
    #define P0_RX_CLK  SCU_ECAT0CON_PORT0_RX_CLKA
#elif GET_GPIO(ECAT_P0_RX_CLK) == (GPIO_PORT5 | GPIO_PIN4)
    #define P0_RX_CLK  SCU_ECAT0CON_PORT0_RX_CLKB
#elif GET_GPIO(ECAT_P0_RX_CLK) == (GPIO_PORT7 | GPIO_PIN10)
    #define P0_RX_CLK  SCU_ECAT0CON_PORT0_RX_CLKC
#else
    #error "EtherCAT : Unknown P0_RX_CLK configuration"
#endif

/* Port 0 RX DV */

#if   GET_GPIO(ECAT_P0_RX_DV) == (GPIO_PORT1 | GPIO_PIN9)
    #define P0_RX_DV  SCU_ECAT0CON_PORT0_RX_DVA
#elif GET_GPIO(ECAT_P0_RX_DV) == (GPIO_PORT5 | GPIO_PIN6)
    #define P0_RX_DV  SCU_ECAT0CON_PORT0_RX_DVB
#elif GET_GPIO(ECAT_P0_RX_DV) == (GPIO_PORT7 | GPIO_PIN11)
    #define P0_RX_DV  SCU_ECAT0CON_PORT0_RX_DVC
#else
    #error "EtherCAT : Unknown P0_RX_DV configuration"
#endif

/* Port 0 RX ERR */

#if   GET_GPIO(ECAT_P0_RX_ERR) == (GPIO_PORT4 | GPIO_PIN0)
    #define P0_RX_ERR  SCU_ECAT0CON_PORT0_RX_ERRA
#elif GET_GPIO(ECAT_P0_RX_ERR) == (GPIO_PORT2 | GPIO_PIN6)
    #define P0_RX_ERR  SCU_ECAT0CON_PORT0_RX_ERRB
#elif GET_GPIO(ECAT_P0_RX_ERR) == (GPIO_PORT7 | GPIO_PIN9)
    #define P0_RX_ERR  SCU_ECAT0CON_PORT0_RX_ERRC
#else
    #error "EtherCAT : Unknown P0_RX_ERR configuration"
#endif

/* Port 0 LINK */

#if   GET_GPIO(ECAT_P0_LINK_STATUS) == (GPIO_PORT4 | GPIO_PIN1)
    #define P0_LINK_STATUS  SCU_ECAT0CON_PORT0_LINKA
#elif GET_GPIO(ECAT_P0_LINK_STATUS) == (GPIO_PORT1 | GPIO_PIN15)
    #define P0_LINK_STATUS  SCU_ECAT0CON_PORT0_LINKB
#elif GET_GPIO(ECAT_P0_LINK_STATUS) == (GPIO_PORT9 | GPIO_PIN10)
    #define P0_LINK_STATUS  SCU_ECAT0CON_PORT0_LINKC
#else
    #error "EtherCAT : Unknown P0_LINK_STATUS configuration"
#endif

/* Port 0 TX CLK */

#if   GET_GPIO(ECAT_P0_TX_CLK) == (GPIO_PORT1 | GPIO_PIN0)
    #define P0_TX_CLK  SCU_ECAT0CON_PORT0_TX_CLKA
#elif GET_GPIO(ECAT_P0_TX_CLK) == (GPIO_PORT5 | GPIO_PIN5)
    #define P0_TX_CLK  SCU_ECAT0CON_PORT0_TX_CLKB
#elif GET_GPIO(ECAT_P0_TX_CLK) == (GPIO_PORT9 | GPIO_PIN1)
    #define P0_TX_CLK  SCU_ECAT0CON_PORT0_TX_CLKC
#else
    #error "EtherCAT : Unknown P0_TX_CLK configuration"
#endif

#else /* not CONFIG_XMC4_ECAT_P0 */
/* When port 0 is not available, the unused MII need to be tied
 * to not connected pins.
 */
#define P0_RXD0 SCU_ECAT0CON_PORT0_RXD0D;
#define P0_RXD1 SCU_ECAT0CON_PORT0_RXD1D;
#define P0_RXD2 SCU_ECAT0CON_PORT0_RXD2D;
#define P0_RXD3 SCU_ECAT0CON_PORT0_RXD3D;
#define P0_RX_CLK SCU_ECAT0CON_PORT0_RX_CLKD;
#define P0_RX_DV SCU_ECAT0CON_PORT0_RX_DVD;
#define P0_RX_ERR SCU_ECAT0CON_PORT0_RX_ERRD;
#define P0_LINK_STATUS SCU_ECAT0CON_PORT0_LINKB;
#define P0_TX_CLK SCU_ECAT0CON_PORT0_TX_CLKD;
#endif /* CONFIG_XMC4_ECAT_P0 */

#if defined(CONFIG_XMC4_ECAT_P1)

/* Port 1 RX D0 */

#if   GET_GPIO(ECAT_P1_RXD0) == (GPIO_PORT0 | GPIO_PIN11)
    #define P1_RXD0  SCU_ECAT0CON_PORT1_RXD0A
#elif GET_GPIO(ECAT_P1_RXD0) == (GPIO_PORT14 | GPIO_PIN7)
    #define P1_RXD0  SCU_ECAT0CON_PORT1_RXD0B
#elif GET_GPIO(ECAT_P1_RXD0) == (GPIO_PORT8 | GPIO_PIN4)
    #define P1_RXD0  SCU_ECAT0CON_PORT1_RXD0C
#else
    #error "EtherCAT : Unknown P1_RXD0 configuration"
#endif

/* Port 1 RX D1 */

#if   GET_GPIO(ECAT_P1_RXD1) == (GPIO_PORT0 | GPIO_PIN6)
    #define P1_RXD1  SCU_ECAT0CON_PORT1_RXD1A
#elif GET_GPIO(ECAT_P1_RXD1) == (GPIO_PORT14 | GPIO_PIN12)
    #define P1_RXD1  SCU_ECAT0CON_PORT1_RXD1B
#elif GET_GPIO(ECAT_P1_RXD1) == (GPIO_PORT8 | GPIO_PIN5)
    #define P1_RXD1  SCU_ECAT0CON_PORT1_RXD1C
#else
    #error "EtherCAT : Unknown P1_RXD1 configuration"
#endif

/* Port 1 RX D2 */

#if   GET_GPIO(ECAT_P1_RXD2) == (GPIO_PORT0 | GPIO_PIN5)
    #define P1_RXD2  SCU_ECAT0CON_PORT1_RXD2A
#elif GET_GPIO(ECAT_P1_RXD2) == (GPIO_PORT14 | GPIO_PIN13)
    #define P1_RXD2  SCU_ECAT0CON_PORT1_RXD2B
#elif GET_GPIO(ECAT_P1_RXD2) == (GPIO_PORT8 | GPIO_PIN6)
    #define P1_RXD2  SCU_ECAT0CON_PORT1_RXD2C
#else
    #error "EtherCAT : Unknown P1_RXD2 configuration"
#endif

/* Port 1 RX D3 */

#if   GET_GPIO(ECAT_P1_RXD3) == (GPIO_PORT0 | GPIO_PIN4)
    #define P1_RXD3  SCU_ECAT0CON_PORT1_RXD3A
#elif GET_GPIO(ECAT_P1_RXD3) == (GPIO_PORT14 | GPIO_PIN14)
    #define P1_RXD3  SCU_ECAT0CON_PORT1_RXD3B
#elif GET_GPIO(ECAT_P1_RXD3) == (GPIO_PORT8 | GPIO_PIN7)
    #define P1_RXD3  SCU_ECAT0CON_PORT1_RXD3C
#else
    #error "EtherCAT : Unknown P1_RXD3 configuration"
#endif

/* Port 1 RX CLK */

#if   GET_GPIO(ECAT_P1_RX_CLK) == (GPIO_PORT0 | GPIO_PIN1)
    #define P1_RX_CLK  SCU_ECAT0CON_PORT1_RX_CLKA
#elif GET_GPIO(ECAT_P1_RX_CLK) == (GPIO_PORT14 | GPIO_PIN6)
    #define P1_RX_CLK  SCU_ECAT0CON_PORT1_RX_CLKB
#elif GET_GPIO(ECAT_P1_RX_CLK) == (GPIO_PORT8 | GPIO_PIN10)
    #define P1_RX_CLK  SCU_ECAT0CON_PORT1_RX_CLKC
#else
    #error "EtherCAT : Unknown P1_RX_CLK configuration"
#endif

/* Port 1 RX DV */

#if   GET_GPIO(ECAT_P1_RX_DV) == (GPIO_PORT0 | GPIO_PIN9)
    #define P1_RX_DV  SCU_ECAT0CON_PORT1_RX_DVA
#elif GET_GPIO(ECAT_P1_RX_DV) == (GPIO_PORT14 | GPIO_PIN15)
    #define P1_RX_DV  SCU_ECAT0CON_PORT1_RX_DVB
#elif GET_GPIO(ECAT_P1_RX_DV) == (GPIO_PORT8 | GPIO_PIN11)
    #define P1_RX_DV  SCU_ECAT0CON_PORT1_RX_DVC
#else
    #error "EtherCAT : Unknown P1_RX_DV configuration"
#endif

/* Port 1 RX ERR */

#if   GET_GPIO(ECAT_P1_RX_ERR) == (GPIO_PORT3 | GPIO_PIN5)
    #define P1_RX_ERR  SCU_ECAT0CON_PORT1_RX_ERRA
#elif GET_GPIO(ECAT_P1_RX_ERR) == (GPIO_PORT15 | GPIO_PIN2)
    #define P1_RX_ERR  SCU_ECAT0CON_PORT1_RX_ERRB
#elif GET_GPIO(ECAT_P1_RX_ERR) == (GPIO_PORT8 | GPIO_PIN9)
    #define P1_RX_ERR  SCU_ECAT0CON_PORT1_RX_ERRC
#else
    #error "EtherCAT : Unknown P1_RX_ERR configuration"
#endif

/* Port 1 LINK */

#if   GET_GPIO(ECAT_P1_LINK_STATUS) == (GPIO_PORT3 | GPIO_PIN4)
    #define P1_LINK_STATUS  SCU_ECAT0CON_PORT1_LINKA
#elif GET_GPIO(ECAT_P1_LINK_STATUS) == (GPIO_PORT15 | GPIO_PIN3)
    #define P1_LINK_STATUS  SCU_ECAT0CON_PORT1_LINKB
#elif GET_GPIO(ECAT_P1_LINK_STATUS) == (GPIO_PORT9 | GPIO_PIN11)
    #define P1_LINK_STATUS  SCU_ECAT0CON_PORT1_LINKC
#else
    #error "EtherCAT : Unknown P1_LINK_STATUS configuration"
#endif

/* Port 1 TX CLK */

#if   GET_GPIO(ECAT_P1_TX_CLK) == (GPIO_PORT0 | GPIO_PIN10)
    #define P1_TX_CLK  SCU_ECAT0CON_PORT1_TX_CLKA
#elif GET_GPIO(ECAT_P1_TX_CLK) == (GPIO_PORT5 | GPIO_PIN9)
    #define P1_TX_CLK  SCU_ECAT0CON_PORT1_TX_CLKB
#elif GET_GPIO(ECAT_P1_TX_CLK) == (GPIO_PORT9 | GPIO_PIN0)
    #define P1_TX_CLK  SCU_ECAT0CON_PORT1_TX_CLKC
#else
    #error "EtherCAT : Unknown P1_TX_CLK configuration"
#endif

#else /* not CONFIG_XMC4_ECAT_P1 */
/* When port 1 is not available, the unused MII need to be tied
 * to not connected pins.
 */
#define P1_RXD0 SCU_ECAT0CON_PORT1_RXD0D;
#define P1_RXD1 SCU_ECAT0CON_PORT1_RXD1D;
#define P1_RXD2 SCU_ECAT0CON_PORT1_RXD2D;
#define P1_RXD3 SCU_ECAT0CON_PORT1_RXD3D;
#define P1_RX_CLK SCU_ECAT0CON_PORT1_RX_CLKD;
#define P1_RX_DV SCU_ECAT0CON_PORT1_RX_DVD;
#define P1_RX_ERR SCU_ECAT0CON_PORT1_RX_ERRD;
#define P1_LINK_STATUS SCU_ECAT0CON_PORT1_LINKB;
#define P1_TX_CLK SCU_ECAT0CON_PORT1_TX_CLKD;
#endif /* CONFIG_XMC4_ECAT_P1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void xmc4_ecat_initialize()
{
#ifdef CONFIG_XMC4_ECAT_P0
  /* Init inputs */

  xmc4_gpio_config(ECAT_P0_LINK_STATUS);
  xmc4_gpio_config(ECAT_P0_RXD3);
  xmc4_gpio_config(ECAT_P0_RXD2);
  xmc4_gpio_config(ECAT_P0_RXD1);
  xmc4_gpio_config(ECAT_P0_RXD0);
  xmc4_gpio_config(ECAT_P0_RX_DV);
  xmc4_gpio_config(ECAT_P0_RX_CLK);
  xmc4_gpio_config(ECAT_P0_RX_ERR);
  xmc4_gpio_config(ECAT_P0_TX_CLK);

  /* Init outputs */

  xmc4_gpio_config(ECAT_P0_LED_LINK_ACT);
  xmc4_gpio_config(ECAT_P0_TXD3);
  xmc4_gpio_config(ECAT_P0_TXD2);
  xmc4_gpio_config(ECAT_P0_TXD1);
  xmc4_gpio_config(ECAT_P0_TXD0);
  xmc4_gpio_config(ECAT_P0_TX_EN);
#endif

#ifdef CONFIG_XMC4_ECAT_P1
  /* Init inputs */

  xmc4_gpio_config(ECAT_P1_LINK_STATUS);
  xmc4_gpio_config(ECAT_P1_RXD3);
  xmc4_gpio_config(ECAT_P1_RXD2);
  xmc4_gpio_config(ECAT_P1_RXD1);
  xmc4_gpio_config(ECAT_P1_RXD0);
  xmc4_gpio_config(ECAT_P1_RX_DV);
  xmc4_gpio_config(ECAT_P1_RX_CLK);
  xmc4_gpio_config(ECAT_P1_RX_ERR);
  xmc4_gpio_config(ECAT_P1_TX_CLK);

  /* Init outputs */

  xmc4_gpio_config(ECAT_P1_LED_LINK_ACT);
  xmc4_gpio_config(ECAT_P1_TXD3);
  xmc4_gpio_config(ECAT_P1_TXD2);
  xmc4_gpio_config(ECAT_P1_TXD1);
  xmc4_gpio_config(ECAT_P1_TXD0);
  xmc4_gpio_config(ECAT_P1_TX_EN);
#endif

  xmc4_gpio_config(ECAT_MDO);
  xmc4_gpio_config(ECAT_CLK_25);
  xmc4_gpio_config(ECAT_LED_ERR);
  xmc4_gpio_config(ECAT_LED_RUN);
  xmc4_gpio_config(ECAT_MCLK);
  xmc4_gpio_config(ECAT_PHY_RESET);

  /* Configure clocks */

#ifndef BOARD_ENABLE_USBPLL
#  error "EtherCAT need USBPLL clock enabled !"
#endif

  uint32_t ecatclkcr = 0;
  ecatclkcr |= SCU_ECATCLKCR_ECATSEL_FPLLUSB;
  ecatclkcr |= (BOARD_ECAT_DIV << SCU_ECATCLKCR_ECADIV_SHIFT);
  putreg32(ecatclkcr, XMC4_SCU_ECATCLKCR);

  /* ECAT reset */

  putreg32(SCU_PR2_ECAT0RS, XMC4_SCU_PRSET2);
  while (!(getreg32(XMC4_SCU_PRSTAT2) & SCU_PR2_ECAT0RS)) /* is ecat reset */
    {
    };

  /* Gate peripheral clock */

  putreg32(SCU_CGAT2_ECAT, XMC4_SCU_CGATSET2);

  /* init port control MII */

  xmc4_ecat_initialize_port_control();

  /* ECAT init, Ungate peripheral */

  putreg32(SCU_CGAT2_ECAT, XMC4_SCU_CGATCLR2);

  /* Deassert reset */

  putreg32(SCU_PR2_ECAT0RS, XMC4_SCU_PRCLR2);
  while ((getreg32(XMC4_SCU_PRSTAT2) & SCU_PR2_ECAT0RS)) /* is ecat reset */
    {
    };
}

void xmc4_ecat_initialize_port_control()
{
  /* common */

  uint32_t ecat0_con_conf = 0;                          /* Default value to 0x0000 0000 but described here for reference */
  ecat0_con_conf |= 0 << SCU_ECAT0CON_ECATRSTEN_SHIFT;  /* Reset request by master disabled */
  ecat0_con_conf |= 0 << SCU_ECAT0CON_PHY_OFFSET_SHIFT; /* Offset of the PHY address offset (port 0) */
  ecat0_con_conf |= MDIO;                               /* MDIO input select */
  putreg32(ecat0_con_conf, XMC4_SCU_ECAT0CON);

  /* port0 */

  uint32_t ecat0_port0_conf = 0;
  ecat0_port0_conf |= P0_RXD0;
  ecat0_port0_conf |= P0_RXD1;
  ecat0_port0_conf |= P0_RXD2;
  ecat0_port0_conf |= P0_RXD3;
  ecat0_port0_conf |= P0_RX_CLK;
  ecat0_port0_conf |= P0_RX_DV;
  ecat0_port0_conf |= P0_RX_ERR;
  ecat0_port0_conf |= P0_LINK_STATUS;
  ecat0_port0_conf |= P0_TX_CLK;
  putreg32(ecat0_port0_conf, XMC4_SCU_ECAT0CONP0);

  /* port 1 */

  uint32_t ecat0_port1_conf = 0;
  ecat0_port1_conf |= P1_RXD0;
  ecat0_port1_conf |= P1_RXD1;
  ecat0_port1_conf |= P1_RXD2;
  ecat0_port1_conf |= P1_RXD3;
  ecat0_port1_conf |= P1_RX_CLK;
  ecat0_port1_conf |= P1_RX_DV;
  ecat0_port1_conf |= P1_RX_ERR;
  ecat0_port1_conf |= P1_LINK_STATUS;
  ecat0_port1_conf |= P1_TX_CLK;
  putreg32(ecat0_port1_conf, XMC4_SCU_ECAT0CONP1);
}
