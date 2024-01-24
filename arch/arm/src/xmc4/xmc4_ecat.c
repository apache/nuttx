/****************************************************************************
 * arch/arm/src/xmc4/xmc4_ecat.c
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
 * Public Functions
 ****************************************************************************/

void xmc4_ecat_initialize()
{
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
  xmc4_gpio_config(ECAT_P1_LINK_STATUS);
  xmc4_gpio_config(ECAT_P1_RXD3);
  xmc4_gpio_config(ECAT_P1_RXD2);
  xmc4_gpio_config(ECAT_P1_RXD1);
  xmc4_gpio_config(ECAT_P1_RXD0);
  xmc4_gpio_config(ECAT_P1_RX_DV);
  xmc4_gpio_config(ECAT_P1_RX_CLK);
  xmc4_gpio_config(ECAT_P1_RX_ERR);
  xmc4_gpio_config(ECAT_P1_TX_CLK);
  xmc4_gpio_config(ECAT_MDO);

  /* Init outputs */

  xmc4_gpio_config(ECAT_P0_LED_LINK_ACT);
  xmc4_gpio_config(ECAT_P0_TXD3);
  xmc4_gpio_config(ECAT_P0_TXD2);
  xmc4_gpio_config(ECAT_P0_TXD1);
  xmc4_gpio_config(ECAT_P0_TXD0);
  xmc4_gpio_config(ECAT_P0_TX_EN);
  xmc4_gpio_config(ECAT_P1_LED_LINK_ACT);
  xmc4_gpio_config(ECAT_P1_TXD3);
  xmc4_gpio_config(ECAT_P1_TXD2);
  xmc4_gpio_config(ECAT_P1_TXD1);
  xmc4_gpio_config(ECAT_P1_TXD0);
  xmc4_gpio_config(ECAT_P1_TX_EN);
  xmc4_gpio_config(ECAT_CLK_25);
  xmc4_gpio_config(ECAT_LED_ERR);
  xmc4_gpio_config(ECAT_LED_RUN);
  xmc4_gpio_config(ECAT_MCLK);
  xmc4_gpio_config(ECAT_PHY_RESET);

  /* configure PLL */

  #define SCU_ECATCLKCR_PLL ((0 << 16) | (1 << 0))
  putreg32(SCU_ECATCLKCR_PLL, XMC4_SCU_ECATCLKCR);

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

  uint32_t mdio_conf = 0;
  switch (ECAT_MDO & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
    case GPIO_PORT0 | GPIO_PIN12:
    {
      mdio_conf = SCU_ECAT0CON_MDIOA;
      break;
    }

    case GPIO_PORT4 | GPIO_PIN2:
    {
      mdio_conf = SCU_ECAT0CON_MDIOB;
      break;
    }

    case GPIO_PORT9 | GPIO_PIN7:
    {
      mdio_conf = SCU_ECAT0CON_MDIOC;
      break;
    }

    default:
    {
      nerr("Unknown mdio config \n");
    }
  }

  uint32_t ecat0_con_conf = 0;
  ecat0_con_conf |= SCU_ECAT0CON_PHY_OFFSET;
  ecat0_con_conf |= SCU_ECAT0CON_ECATRSTEN;
  ecat0_con_conf |= mdio_conf;

  /* port0 */

  uint32_t port0_rxd0_conf = 0;
  switch (ECAT_P0_RXD0 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN4:
      {
          port0_rxd0_conf = SCU_ECAT0CON_PORT0_RXD0A;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN0:
      {
          port0_rxd0_conf = SCU_ECAT0CON_PORT0_RXD0B;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN4:
      {
          port0_rxd0_conf = SCU_ECAT0CON_PORT0_RXD0C;
          break;
      }

      default:
      {
          nerr("Unknown port0_rxd0 config \n");
      }
  }

  uint32_t port0_rxd1_conf = 0;
  switch (ECAT_P0_RXD1 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN5:
      {
          port0_rxd1_conf = SCU_ECAT0CON_PORT0_RXD1A;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN1:
      {
          port0_rxd1_conf = SCU_ECAT0CON_PORT0_RXD1B;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN5:
      {
          port0_rxd1_conf = SCU_ECAT0CON_PORT0_RXD1C;
          break;
      }

      default:
      {
          nerr("Unknown port0_rxd1 config \n");
      }
  }

  uint32_t port0_rxd2_conf = 0;
  switch (ECAT_P0_RXD2 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN10:
      {
          port0_rxd2_conf = SCU_ECAT0CON_PORT0_RXD2A;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN2:
      {
          port0_rxd2_conf = SCU_ECAT0CON_PORT0_RXD2B;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN6:
      {
          port0_rxd2_conf = SCU_ECAT0CON_PORT0_RXD2C;
          break;
      }

      default:
      {
          nerr("Unknown port0_rxd2 config \n");
      }
  }

  uint32_t port0_rxd3_conf = 0;
  switch (ECAT_P0_RXD3 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN11:
      {
          port0_rxd3_conf = SCU_ECAT0CON_PORT0_RXD3A;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN7:
      {
          port0_rxd3_conf = SCU_ECAT0CON_PORT0_RXD3B;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN7:
      {
          port0_rxd3_conf = SCU_ECAT0CON_PORT0_RXD3C;
          break;
      }

      default:
      {
          nerr("Unknown port0_rxd2 config \n");
      }
  }

  uint32_t port0_rx_clk_conf = 0;
  switch (ECAT_P0_RX_CLK & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN1:
      {
          port0_rx_clk_conf = SCU_ECAT0CON_PORT0_RX_CLKA;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN4:
      {
          port0_rx_clk_conf = SCU_ECAT0CON_PORT0_RX_CLKB;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN10:
      {
          port0_rx_clk_conf = SCU_ECAT0CON_PORT0_RX_CLKC;
          break;
      }

      default:
      {
          nerr("Unknown port0_rx_clk config \n");
      }
  }

  uint32_t port0_rx_dv_conf = 0;
  switch (ECAT_P0_RX_DV & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN9:
      {
          port0_rx_dv_conf = SCU_ECAT0CON_PORT0_RX_DVA;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN6:
      {
          port0_rx_dv_conf = SCU_ECAT0CON_PORT0_RX_DVB;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN11:
      {
          port0_rx_dv_conf = SCU_ECAT0CON_PORT0_RX_DVC;
          break;
      }

      default:
      {
          nerr("Unknown port0_rx_dv config \n");
      }
  }

  uint32_t port0_rx_err_conf = 0;
  switch (ECAT_P0_RX_ERR & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT4 | GPIO_PIN0:
      {
          port0_rx_err_conf = SCU_ECAT0CON_PORT0_RX_ERRA;
          break;
      }

      case GPIO_PORT2 | GPIO_PIN6:
      {
          port0_rx_err_conf = SCU_ECAT0CON_PORT0_RX_ERRB;
          break;
      }

      case GPIO_PORT7 | GPIO_PIN9:
      {
          port0_rx_err_conf = SCU_ECAT0CON_PORT0_RX_ERRC;
          break;
      }

      default:
      {
          nerr("Unknown port0_rx_err config \n");
      }
  }

  uint32_t port0_link_conf = 0;
  switch (ECAT_P0_LINK_STATUS & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT4 | GPIO_PIN1:
      {
          port0_link_conf = SCU_ECAT0CON_PORT0_LINKA;
          break;
      }

      case GPIO_PORT1 | GPIO_PIN15:
      {
          port0_link_conf = SCU_ECAT0CON_PORT0_LINKB;
          break;
      }

      case GPIO_PORT9 | GPIO_PIN10:
      {
          port0_link_conf = SCU_ECAT0CON_PORT0_LINKC;
          break;
      }

      default:
      {
          nerr("Unknown port0_link config \n");
      }
  }

  uint32_t port0_tx_clk_conf = 0;
  switch (ECAT_P0_TX_CLK & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT1 | GPIO_PIN0:
      {
          port0_tx_clk_conf = SCU_ECAT0CON_PORT0_TX_CLKA;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN5:
      {
          port0_tx_clk_conf = SCU_ECAT0CON_PORT0_TX_CLKB;
          break;
      }

      case GPIO_PORT9 | GPIO_PIN1:
      {
          port0_tx_clk_conf = SCU_ECAT0CON_PORT0_TX_CLKC;
          break;
      }

      default:
      {
          nerr("Unknown port0_tx_clk config \n");
      }
  }

  uint32_t ecat0_port0_conf = 0;
  ecat0_port0_conf |= port0_rxd0_conf;
  ecat0_port0_conf |= port0_rxd1_conf;
  ecat0_port0_conf |= port0_rxd2_conf;
  ecat0_port0_conf |= port0_rxd3_conf;
  ecat0_port0_conf |= port0_rx_clk_conf;
  ecat0_port0_conf |= port0_rx_dv_conf;
  ecat0_port0_conf |= port0_rx_err_conf;
  ecat0_port0_conf |= port0_link_conf;
  ecat0_port0_conf |= port0_tx_clk_conf;

  putreg32(ecat0_port0_conf, XMC4_SCU_ECAT0CONP0);

  /* port 1 */

  uint32_t port1_rxd0_conf = 0;
  switch (ECAT_P1_RXD0 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN11:
      {
          port1_rxd0_conf = SCU_ECAT0CON_PORT1_RXD0A;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN7:
      {
          port1_rxd0_conf = SCU_ECAT0CON_PORT1_RXD0B;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN4:
      {
          port1_rxd0_conf = SCU_ECAT0CON_PORT1_RXD0C;
          break;
      }

      default:
      {
          nerr("Unknown port1_rxd0 config \n");
      }
  }

  uint32_t port1_rxd1_conf = 0;
  switch (ECAT_P1_RXD1 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN6:
      {
          port1_rxd1_conf = SCU_ECAT0CON_PORT1_RXD1A;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN12:
      {
          port1_rxd1_conf = SCU_ECAT0CON_PORT1_RXD1B;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN5:
      {
          port1_rxd1_conf = SCU_ECAT0CON_PORT1_RXD1C;
          break;
      }

      default:
      {
          nerr("Unknown port1_rxd1 config \n");
      }
  }

  uint32_t port1_rxd2_conf = 0;
  switch (ECAT_P1_RXD2 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN5:
      {
          port1_rxd2_conf = SCU_ECAT0CON_PORT1_RXD2A;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN13:
      {
          port1_rxd2_conf = SCU_ECAT0CON_PORT1_RXD2B;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN6:
      {
          port1_rxd2_conf = SCU_ECAT0CON_PORT1_RXD2C;
          break;
      }

      default:
      {
          nerr("Unknown port1_rxd2 config \n");
      }
  }

  uint32_t port1_rxd3_conf = 0;
  switch (ECAT_P1_RXD3 & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN4:
      {
          port1_rxd3_conf = SCU_ECAT0CON_PORT1_RXD3A;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN14:
      {
          port1_rxd3_conf = SCU_ECAT0CON_PORT1_RXD3B;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN7:
      {
          port1_rxd3_conf = SCU_ECAT0CON_PORT1_RXD3C;
          break;
      }

      default:
      {
          nerr("Unknown port1_rxd3 config \n");
      }
  }

  uint32_t port1_rx_clk_conf = 0;
  switch (ECAT_P1_RX_CLK & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN1:
      {
          port1_rx_clk_conf = SCU_ECAT0CON_PORT1_RX_CLKA;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN6:
      {
          port1_rx_clk_conf = SCU_ECAT0CON_PORT1_RX_CLKB;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN10:
      {
          port1_rx_clk_conf = SCU_ECAT0CON_PORT1_RX_CLKC;
          break;
      }

      default:
      {
          nerr("Unknown port1_rx_clk config \n");
      }
  }

  uint32_t port1_rx_dv_conf = 0;
  switch (ECAT_P1_RX_DV & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN9:
      {
          port1_rx_dv_conf = SCU_ECAT0CON_PORT1_RX_DVA;
          break;
      }

      case GPIO_PORT14 | GPIO_PIN15:
      {
          port1_rx_dv_conf = SCU_ECAT0CON_PORT1_RX_DVB;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN11:
      {
          port1_rx_dv_conf = SCU_ECAT0CON_PORT1_RX_DVC;
          break;
      }

      default:
      {
          nerr("Unknown port1_rx_dv config \n");
      }
  }

  uint32_t port1_link_conf = 0;
  switch (ECAT_P1_LINK_STATUS & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT3 | GPIO_PIN4:
      {
          port1_link_conf = SCU_ECAT0CON_PORT1_LINKA;
          break;
      }

      case GPIO_PORT15 | GPIO_PIN3:
      {
          port1_link_conf = SCU_ECAT0CON_PORT1_LINKB;
          break;
      }

      case GPIO_PORT9 | GPIO_PIN11:
      {
          port1_link_conf = SCU_ECAT0CON_PORT1_LINKC;
          break;
      }

      default:
      {
          nerr("Unknown port1_link config \n");
      }
  }

  uint32_t port1_tx_clk_conf = 0;
  switch (ECAT_P1_TX_CLK & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT0 | GPIO_PIN10:
      {
          port1_tx_clk_conf = SCU_ECAT0CON_PORT1_TX_CLKA;
          break;
      }

      case GPIO_PORT5 | GPIO_PIN9:
      {
          port1_tx_clk_conf = SCU_ECAT0CON_PORT1_TX_CLKB;
          break;
      }

      case GPIO_PORT9 | GPIO_PIN0:
      {
          port1_tx_clk_conf = SCU_ECAT0CON_PORT1_TX_CLKC;
          break;
      }

      default:
      {
          nerr("Unknown port1_tx_clk config \n");
      }
  }

  uint32_t port1_rx_err_conf = 0;
  switch (ECAT_P1_RX_ERR & (GPIO_PORT_MASK | GPIO_PIN_MASK))
  {
      case GPIO_PORT3 | GPIO_PIN5:
      {
          port1_rx_err_conf = SCU_ECAT0CON_PORT1_RX_ERRA;
          break;
      }

      case GPIO_PORT15 | GPIO_PIN2:
      {
          port1_rx_err_conf = SCU_ECAT0CON_PORT1_RX_ERRB;
          break;
      }

      case GPIO_PORT8 | GPIO_PIN9:
      {
          port1_rx_err_conf = SCU_ECAT0CON_PORT1_RX_ERRC;
          break;
      }

      default:
      {
          nerr("Unknown port1_rx_err config \n");
      }
  }

  /* When port 1 is not available, the unused MII need to be tied
  * to not connected pins.
  */

#ifndef CONFIG_ECAT_P1_ENABLE
  port1_rxd0_conf   = SCU_ECAT0CON_PORT1_RXD0D;
  port1_rxd1_conf   = SCU_ECAT0CON_PORT1_RXD1D;
  port1_rxd2_conf   = SCU_ECAT0CON_PORT1_RXD2D;
  port1_rxd3_conf   = SCU_ECAT0CON_PORT1_RXD3D;
  port1_rx_clk_conf = SCU_ECAT0CON_PORT1_RX_CLKD;
  port1_rx_dv_conf  = SCU_ECAT0CON_PORT1_RX_DVD;
  port1_rx_err_conf = SCU_ECAT0CON_PORT1_RX_ERRD;
  port1_link_conf   = SCU_ECAT0CON_PORT1_LINKB;
  port1_tx_clk_conf = SCU_ECAT0CON_PORT1_TX_CLKD;
#endif

  uint32_t ecat0_port1_conf = 0;
  ecat0_port1_conf |= port1_rxd0_conf;
  ecat0_port1_conf |= port1_rxd1_conf;
  ecat0_port1_conf |= port1_rxd2_conf;
  ecat0_port1_conf |= port1_rxd3_conf;
  ecat0_port1_conf |= port1_rx_clk_conf;
  ecat0_port1_conf |= port1_rx_dv_conf;
  ecat0_port1_conf |= port1_rx_err_conf;
  ecat0_port1_conf |= port1_link_conf;
  ecat0_port1_conf |= port1_tx_clk_conf;

  putreg32(ecat0_port1_conf, XMC4_SCU_ECAT0CONP1);
}
