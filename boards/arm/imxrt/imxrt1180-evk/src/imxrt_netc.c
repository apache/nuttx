/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_netc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/net/realtek_phy.h>

#include "arm_internal.h"
#include "imxrt_clockconfig.h"
#include "imxrt_gpio.h"
#include "imxrt_netc.h"
#include "imxrt1180-evk.h"
#include "hardware/rt118x/imxrt118x_ocotp.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_RTL8201F_PHY(addr) \
  .phyaddr = (addr), \
  .interface = PHY_INTERFACE_RMII

#define BOARD_RTL8211F_PHY(addr) \
  .phyaddr = (addr), \
  .interface = PHY_INTERFACE_RGMII_ID

#define RTL8201F_RESET_DELAY_MS       10
#define RTL8201F_RESET_POST_DELAY_MS  150
#define RTL8211F_RESET_DELAY_MS       10
#define RTL8211F_RESET_POST_DELAY_MS  50

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_netc_board_port_s
{
  imxrt_pinset_t pins[12];
  imxrt_pinset_t reset;
  unsigned int npins;
  unsigned int reset_delay;
  unsigned int reset_post_delay;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imxrt_netc_board_port_s
g_netc_ports[IMXRT_NETC_PHY_PORT_COUNT] =
{
  [0] =
  {
    .pins = BOARD_ENET1_PORT0_PINS,
    .reset = GPIO_ENET1_PORT0_PHY_RESET,
    .npins = 8,
    .reset_delay = RTL8201F_RESET_DELAY_MS,
    .reset_post_delay = RTL8201F_RESET_POST_DELAY_MS
  },
  [1] =
  {
    .pins = BOARD_ENET1_PORT1_PINS,
    .reset = GPIO_ENET1_PORT1_PHY_RESET,
    .npins = 12,
    .reset_delay = RTL8211F_RESET_DELAY_MS,
    .reset_post_delay = RTL8211F_RESET_POST_DELAY_MS
  },
  [2] =
  {
    .pins = BOARD_ENET1_PORT2_PINS,
    .reset = GPIO_ENET1_PORT2_PHY_RESET,
    .npins = 12,
    .reset_delay = RTL8211F_RESET_DELAY_MS,
    .reset_post_delay = RTL8211F_RESET_POST_DELAY_MS
  },
  [3] =
  {
    .pins = BOARD_ENET1_PORT3_PINS,
    .reset = GPIO_ENET1_PORT3_PHY_RESET,
    .npins = 12,
    .reset_delay = RTL8211F_RESET_DELAY_MS,
    .reset_post_delay = RTL8211F_RESET_POST_DELAY_MS
  },
  [4] =
  {
    .pins = BOARD_ENET0_PORT0_PINS,
    .reset = GPIO_ENET0_PORT0_PHY_RESET,
    .npins = 8,
    .reset_delay = RTL8201F_RESET_DELAY_MS,
    .reset_post_delay = RTL8201F_RESET_POST_DELAY_MS
  }
};

static const struct imxrt_netc_config_s g_netc_config =
{
  .enetc =
  {
    [0] =
    {
      .msgintr = 2,
#ifdef CONFIG_IMXRT_NETC_ENETC0
      .enabled = true,
      .ports =
      {
        [0] =
        {
          .enabled = true,
          BOARD_RTL8201F_PHY(3)
        }
      }
#endif
    },
    [1] =
    {
      .msgintr = 1,
#if defined(CONFIG_IMXRT_NETC_ENETC1_PORT0) || \
    defined(CONFIG_IMXRT_NETC_ENETC1_PORT1) || \
    defined(CONFIG_IMXRT_NETC_ENETC1_PORT2) || \
    defined(CONFIG_IMXRT_NETC_ENETC1_PORT3)
      .enabled = true,
#endif
      .ports =
      {
#ifdef CONFIG_IMXRT_NETC_ENETC1_PORT0
        [0] =
        {
          .enabled = true,
          BOARD_RTL8201F_PHY(2)
        },
#endif
#ifdef CONFIG_IMXRT_NETC_ENETC1_PORT1
        [1] =
        {
          .enabled = true,
          BOARD_RTL8211F_PHY(5)
        },
#endif
#ifdef CONFIG_IMXRT_NETC_ENETC1_PORT2
        [2] =
        {
          .enabled = true,
          BOARD_RTL8211F_PHY(4)
        },
#endif
#ifdef CONFIG_IMXRT_NETC_ENETC1_PORT3
        [3] =
        {
          .enabled = true,
          BOARD_RTL8211F_PHY(7)
        },
#endif
      }
    }
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_netc_phyinit(unsigned int if_idx,
                       const struct imxrt_netc_port_s *config)
{
  const struct imxrt_netc_board_port_s *port;
  uint16_t gpio;
  unsigned int gpioport;
  uint32_t mask;
  int ret;

  /* config not used */

  (void) config;

  DEBUGASSERT(if_idx < nitems(g_netc_ports));
  port = &g_netc_ports[if_idx];
  ret = imxrt_config_gpio(port->reset | GPIO_SION_ENABLE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: reset pin for port %u failed: %d\n",
             if_idx, ret);
      return ret;
    }

  /* Enable input readback of the open-drain reset pad. */

  gpio = IMXRT_PINSET_GPIO(port->reset);
  gpioport = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  mask = 1u << ((gpio & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
  modifyreg32(IMXRT_GPIO_PIDR(gpioport), mask, 0);

  imxrt_gpio_write(port->reset, false);
  up_mdelay(port->reset_delay);
  imxrt_gpio_write(port->reset, true);
  up_mdelay(port->reset_post_delay);
  return OK;
}

int imxrt_netc_boardinitialize(void)
{
  struct imxrt_netc_config_s config = g_netc_config;
  const struct imxrt_netc_port_s *port;
  uint32_t uidl;
  uint32_t uidml;
  unsigned int i;
  unsigned int pin;
  int ret;

  ret = imxrt_ccm_gate_on(CCM_LPCG_IOMUXC1, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: IOMUXC1 clock failed: %d\n", ret);
      return ret;
    }

  ret = imxrt_clockall_gpio4();
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: GPIO4 clock failed: %d\n", ret);
      return ret;
    }

  if (config.enetc[1].enabled)
    {
      ret = imxrt_clockall_gpio6();
      if (ret < 0)
        {
          syslog(LOG_ERR, "NETC: GPIO6 clock failed: %d\n", ret);
          return ret;
        }
    }

  ret = imxrt_config_gpio(GPIO_NETC_MDC);
  if (ret == OK)
    {
      ret = imxrt_config_gpio(GPIO_NETC_MDIO);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: MDIO pin configuration failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_IMXRT_NETC_ENETC0
  ret = imxrt_config_gpio(GPIO_ENET4_PHYAD1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: PHY4 strap pin configuration failed: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_IMXRT_NETC_ENETC1_PORT0
  ret = imxrt_config_gpio(GPIO_ENET0_PHYAD1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: PHY0 strap pin configuration failed: %d\n",
             ret);
      return ret;
    }
#endif

  for (i = 0; i < nitems(g_netc_ports); i++)
    {
      port = i == 4 ? &config.enetc[0].ports[0] :
                      &config.enetc[1].ports[i];
      if (!port->enabled)
        {
          continue;
        }

      for (pin = 0; pin < g_netc_ports[i].npins; pin++)
        {
          ret = imxrt_config_gpio(g_netc_ports[i].pins[pin]);
          if (ret < 0)
            {
              syslog(LOG_ERR, "NETC: port %u pin %u failed: %d\n",
                     i, pin, ret);
              return ret;
            }
        }
    }

  uidl = getreg32(IMXRT_OCOTP_UNIQUE_ID_MSB);
  uidml = getreg32(IMXRT_OCOTP_UNIQUE_ID_LSB);
  config.enetc[1].mac[0] = ((uidml >> 8) & 0xfe) | 0x02;
  config.enetc[1].mac[1] = uidml;
  config.enetc[1].mac[2] = uidl >> 24;
  config.enetc[1].mac[3] = uidl >> 16;
  config.enetc[1].mac[4] = uidl >> 8;
  config.enetc[1].mac[5] = uidl;
  memcpy(config.enetc[0].mac, config.enetc[1].mac,
         sizeof(config.enetc[0].mac));
  config.enetc[0].mac[5] ^= 1;

#ifdef CONFIG_NET_PHY_REALTEK_RTL8201F
  ret = rtl8201f_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: RTL8201F registration failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NET_PHY_REALTEK_RTL8211F
  ret = rtl8211f_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "NETC: RTL8211F registration failed: %d\n", ret);
      return ret;
    }
#endif

  /* Perform PHY resets */

  for (i = 0; i < IMXRT_NETC_PHY_PORT_COUNT; i++)
    {
      board_netc_phyinit(i, NULL);
    }

  return imxrt_netc_initialize(&config);
}
