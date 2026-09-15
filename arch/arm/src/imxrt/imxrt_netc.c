/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc.c
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

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arpa/inet.h>

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "imxrt_clockconfig.h"
#include "imxrt_netc.h"
#include "hardware/rt118x/imxrt118x_netc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETC_MDC_MAX_FREQUENCY          2500000u
#define NETC_MDC_DIVISOR                (2u * NETC_MDC_MAX_FREQUENCY)
#define NETC_MDIO_HOLD_RECOMMENDED      2u

#define NETC_PHY_ID1                    2u
#define NETC_PHY_ID2                    3u
#define NETC_PHY_BMSR                   1u
#define NETC_PHY_SPECIFIC_STATUS        0x11u
#define NETC_PHY_PAGE_SELECT            0x1eu
#define NETC_PHY_PAGE_DATA              0x1fu
#define NETC_YT8531_CHIP_CONFIG         0xa001u
#define NETC_YT8531_RGMII_CONFIG1       0xa003u
#define NETC_YT8531_CCR_RXC_DLY_EN      (1u << 8)
#define NETC_YT8531_RX_DELAY_SHIFT      10
#define NETC_YT8531_TX_DELAY_SHIFT      0
#define NETC_YT8531_DELAY_MASK          0xfu

#define NETC_PHY_BMSR_LINK              (1u << 2)
#define NETC_PHY_BMSR_AN_COMPLETE       (1u << 5)
#define NETC_PHY_STATUS_LINK            (1u << 10)
#define NETC_PHY_STATUS_DUPLEX          (1u << 13)
#define NETC_PHY_STATUS_SPEED_SHIFT     14
#define NETC_PHY_STATUS_SPEED_MASK      (3u << NETC_PHY_STATUS_SPEED_SHIFT)
#define NETC_PHY_STATUS_SPEED_10        (0u << NETC_PHY_STATUS_SPEED_SHIFT)
#define NETC_PHY_STATUS_SPEED_100       (1u << NETC_PHY_STATUS_SPEED_SHIFT)
#define NETC_PHY_STATUS_SPEED_1000      (2u << NETC_PHY_STATUS_SPEED_SHIFT)

#define NETC_RING_COUNT                 8u
/* Port-masquerade TX uses SI management+1 ring per MCUX SWT_SendFrame. */
#define NETC_TX_BDR                     1u
#define NETC_RX_BDR                     0u
#define NETC_SI_BDR_COUNT               2u
#define NETC_DESCRIPTOR_ALIGNMENT       128u
#define NETC_DMA_ALIGNMENT              32u
#define NETC_DMA_BUFFER_SIZE            1536u
#define NETC_POLL_DELAY                 MSEC2TICK(10)
#define NETC_LINK_POLL_INTERVAL_MS      100u

#define NETC_RX_READY                   (1ull << 62)
#define NETC_RX_FINAL                   (1ull << 63)
#define NETC_RX_ERROR_SHIFT             48
#define NETC_TX_FINAL                   (1ull << 63)
/* MCUX port-masquerade TX: FLQ=2 + switch ingress port (no SMSO). */
#define NETC_TX_SWITCH_MASQUERADE(port) \
  ((uint64_t)(((2u << 24) | \
               (((uint32_t)(port) & 0x1fu) << 16))) << 32)
/* MCUX / Zephyr DSA no-tag: FLQ=2 + SMSO + egress port (bypass bridge). */
#define NETC_TX_SWITCH_DIRECT(port) \
  ((uint64_t)(((2u << 24) | (1u << 23) | \
               (((uint32_t)(port) & 0x1fu) << 16))) << 32)
#define NETC_ETH_MIN_FRAME              60u

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else
#  define ETHWORK LPWORK
#endif

#define IOMUXC_MUX_SION                (1u << 4)
#define IOMUXC_PAD_ETHERNET             0x06u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_netc_pin_s
{
  uintptr_t mux;
  uintptr_t input;
  uintptr_t pad;
  uint8_t mode;
  uint8_t daisy;
  bool input_enable;
};

struct imxrt_netc_desc_s
{
  uint64_t word0;
  uint64_t word1;
};

struct imxrt_netc_link_s
{
  bool up;
  bool full_duplex;
  uint16_t speed;
};

struct imxrt_netc_driver_s
{
  bool registered;
  bool ifup;
  uint8_t active_port;
  uint8_t active_phy;
  uint16_t txhead;
  uint16_t txclean;
  uint16_t rxtail;
  struct wdog_s polltimer;
  struct work_s pollwork;
  struct net_driver_s dev;
  uint8_t buffer[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE]
    aligned_data(NETC_DMA_ALIGNMENT);
  struct imxrt_netc_desc_s txring[NETC_RING_COUNT]
    aligned_data(NETC_DESCRIPTOR_ALIGNMENT);
  struct imxrt_netc_desc_s rxring[NETC_RING_COUNT]
    aligned_data(NETC_DESCRIPTOR_ALIGNMENT);
  uint8_t txpool[NETC_RING_COUNT][NETC_DMA_BUFFER_SIZE]
    aligned_data(NETC_DMA_ALIGNMENT);
  uint8_t rxpool[NETC_RING_COUNT][NETC_DMA_BUFFER_SIZE]
    aligned_data(NETC_DMA_ALIGNMENT);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imxrt_netc_pin_s g_netc_pins[] =
{
  {0x42a10184u, 0,           0x42a103ccu, 7, 0, false},
  {0x42a10188u, 0x42a10798u, 0x42a103d0u, 7, 4, false},
  {0x42a100ccu, 0,           0x42a10314u, 3, 0, false},
  {0x42a100d0u, 0,           0x42a10318u, 3, 0, false},
  {0x42a100c4u, 0,           0x42a1030cu, 4, 0, false},
  {0x42a100c8u, 0,           0x42a10310u, 4, 0, false},
  {0x42a100d4u, 0,           0x42a1031cu, 3, 0, false},
  {0x42a100d8u, 0x42a107f4u, 0x42a10320u, 3, 1, true},
  {0x42a100b8u, 0x42a107d8u, 0x42a10300u, 4, 0, false},
  {0x42a100dcu, 0x42a107e4u, 0x42a10324u, 3, 1, false},
  {0x42a100e0u, 0x42a107e8u, 0x42a10328u, 3, 1, false},
  {0x42a100bcu, 0x42a107ecu, 0x42a10304u, 4, 0, false},
  {0x42a100c0u, 0x42a107f0u, 0x42a10308u, 4, 0, false},
  {0x42a100e4u, 0x42a107dcu, 0x42a1032cu, 3, 1, false},
  {0x42a1007cu, 0,           0x42a102c4u, 4, 0, false},
  {0x42a10078u, 0,           0x42a102c0u, 4, 0, false},
  {0x42a100a4u, 0,           0x42a102ecu, 4, 0, false},
  {0x42a100a0u, 0,           0x42a102e8u, 4, 0, false},
  {0x42a10080u, 0,           0x42a102c8u, 4, 0, false},
  {0x42a10084u, 0x42a10814u, 0x42a102ccu, 4, 1, true},
  {0x42a100a8u, 0x42a107f8u, 0x42a102f0u, 4, 2, false},
  {0x42a10088u, 0x42a10804u, 0x42a102d0u, 4, 1, false},
  {0x42a1008cu, 0x42a10808u, 0x42a102d4u, 4, 1, false},
  {0x42a10098u, 0x42a1080cu, 0x42a102e0u, 4, 1, false},
  {0x42a1009cu, 0x42a10810u, 0x42a102e4u, 4, 1, false},
  {0x42a10090u, 0x42a107fcu, 0x42a102d8u, 4, 1, false},
};

static struct imxrt_netc_driver_s g_netc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void imxrt_netc_poll_expiry(wdparm_t arg);

static int imxrt_netc_wait_clear(uintptr_t address, uint32_t mask)
{
  unsigned int timeout;

  for (timeout = 0; timeout < CONFIG_IMXRT_NETC_MDIO_TIMEOUT_US; timeout++)
    {
      if ((getreg32(address) & mask) == 0)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

static void imxrt_netc_configure_pins(void)
{
  unsigned int i;

  for (i = 0; i < sizeof(g_netc_pins) / sizeof(g_netc_pins[0]); i++)
    {
      uint32_t mux = g_netc_pins[i].mode;

      if (g_netc_pins[i].input_enable)
        {
          mux |= IOMUXC_MUX_SION;
        }

      putreg32(mux, g_netc_pins[i].mux);
      if (g_netc_pins[i].input != 0)
        {
          putreg32(g_netc_pins[i].daisy, g_netc_pins[i].input);
        }

      putreg32(IOMUXC_PAD_ETHERNET, g_netc_pins[i].pad);
    }
}

static int imxrt_netc_board_initialize(void)
{
  uint32_t regval;
  int ret;

  modifyreg32(IMXRT_NETC_LINK0_CFG, NETC_LINK_CFG_MII_PROTOCOL_MASK,
              NETC_LINK_CFG_MII_PROTOCOL_RGMII);
  modifyreg32(IMXRT_NETC_LINK2_CFG, NETC_LINK_CFG_MII_PROTOCOL_MASK,
              NETC_LINK_CFG_MII_PROTOCOL_RGMII);

  modifyreg32(IMXRT_NETC_PRIV_NETCRR, NETC_PRIV_NETCRR_LOCK, 0);
  ret = imxrt_netc_wait_clear(IMXRT_NETC_PRIV_NETCRR,
                              NETC_PRIV_NETCRR_LOCK);
  if (ret < 0)
    {
      nerr("NETC: IERB unlock timed out\n");
      return ret;
    }

  regval = getreg32(IMXRT_NETC_IERB_RCMSIAMQR);
  regval &= ~NETC_IERB_RCMSIAMQR_MSI_MASK;
  regval |= NETC_IERB_RCMSIAMQR_MSI_CM33;
  putreg32(regval, IMXRT_NETC_IERB_RCMSIAMQR);
  putreg32(NETC_IERB_LINK_PHY_ADDRESS(
             CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS),
           IMXRT_NETC_IERB_L0BCR);
  putreg32(NETC_IERB_LINK_PHY_ADDRESS(
             CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS),
           IMXRT_NETC_IERB_L2BCR);

  modifyreg32(IMXRT_NETC_PRIV_NETCRR, 0, NETC_PRIV_NETCRR_LOCK);
  ret = imxrt_netc_wait_clear(IMXRT_NETC_PRIV_NETCSR,
                              NETC_PRIV_NETCSR_STATE);
  if (ret < 0)
    {
      nerr("NETC: IERB relock timed out\n");
    }

  return ret;
}

static int imxrt_netc_emdio_initialize(void)
{
  uint32_t divisor;
  uint16_t regval;
  int ret;

  ret = imxrt_netc_clocks_configure();
  if (ret < 0)
    {
      nerr("NETC: clock configuration failed: %d\n", ret);
      return ret;
    }

  imxrt_netc_configure_pins();

  ret = imxrt_netc_board_initialize();
  if (ret < 0)
    {
      return ret;
    }

  /* Reset the independent EMDIO PCI function, then permit its register
   * accesses.  MSI-X is deliberately not configured in this MDIO-only slice.
   */

  regval = getreg16(IMXRT_NETC_F1_DEVICE_CONTROL);
  putreg16(regval | NETC_PCI_DEVICE_CONTROL_FLR,
           IMXRT_NETC_F1_DEVICE_CONTROL);

  ret = imxrt_netc_wait_clear(IMXRT_NETC_F1_DEVICE_CONTROL,
                              NETC_PCI_DEVICE_CONTROL_FLR);
  if (ret < 0)
    {
      nerr("NETC: EMDIO function reset timed out\n");
      return ret;
    }

  regval = getreg16(IMXRT_NETC_F1_COMMAND);
  regval |= NETC_PCI_COMMAND_MEMORY | NETC_PCI_COMMAND_MASTER;
  putreg16(regval, IMXRT_NETC_F1_COMMAND);

  divisor = (CONFIG_IMXRT_NETC_CLOCK_FREQUENCY +
             NETC_MDC_DIVISOR - 1) / NETC_MDC_DIVISOR;
  if (divisor == 0 || divisor > 0x1ffu)
    {
      return -ERANGE;
    }

  putreg32((divisor << NETC_EMDIO_CFG_DIV_SHIFT) |
           (NETC_MDIO_HOLD_RECOMMENDED << NETC_EMDIO_CFG_HOLD_SHIFT),
           IMXRT_NETC_EMDIO_CFG);
  return OK;
}

static int imxrt_netc_probe_phy(uint8_t phyaddr, const char *port)
{
  uint16_t phyid1;
  uint16_t phyid2;
  int ret;

  ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_ID1, &phyid1);
  if (ret == OK)
    {
      ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_ID2, &phyid2);
    }

  if (ret < 0)
    {
      nerr("NETC: FRDM switch %s PHY address %u probe failed: %d\n",
           port, phyaddr, ret);
      return ret;
    }

  if ((phyid1 == 0 && phyid2 == 0) ||
      (phyid1 == 0xffff && phyid2 == 0xffff))
    {
      nerr("NETC: invalid %s PHY ID %04x:%04x at address %u\n",
           port, phyid1, phyid2, phyaddr);
      return -ENODEV;
    }

  ninfo("NETC: FRDM switch %s PHY ID %04x:%04x at address %u\n",
        port, phyid1, phyid2, phyaddr);
  return OK;
}

static int imxrt_netc_mdio_write(uint8_t phyaddr, uint8_t regaddr,
                                 uint16_t value)
{
  uint32_t config;
  int ret;

  if (phyaddr > 31 || regaddr > 31)
    {
      return -EINVAL;
    }

  config = getreg32(IMXRT_NETC_EMDIO_CFG);
  config &= ~NETC_EMDIO_CFG_CLAUSE45;
  putreg32(config, IMXRT_NETC_EMDIO_CFG);
  putreg32(((uint32_t)phyaddr << NETC_EMDIO_CONTROL_PHY_SHIFT) |
           ((uint32_t)regaddr << NETC_EMDIO_CONTROL_REG_SHIFT),
           IMXRT_NETC_EMDIO_CONTROL);
  putreg32(value, IMXRT_NETC_EMDIO_DATA);

  ret = imxrt_netc_wait_clear(IMXRT_NETC_EMDIO_CFG,
                              NETC_EMDIO_CFG_BUSY);
  if (ret < 0)
    {
      return ret;
    }

  config = getreg32(IMXRT_NETC_EMDIO_CFG);
  if ((config & NETC_EMDIO_CFG_ADDRESS_ERROR) != 0)
    {
      return -EIO;
    }

  return OK;
}

static int imxrt_netc_phy_modify_ext(uint8_t phyaddr, uint16_t regaddr,
                                     uint16_t mask, uint16_t set)
{
  uint16_t value;
  int ret;

  ret = imxrt_netc_mdio_write(phyaddr, NETC_PHY_PAGE_SELECT, regaddr);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_PAGE_DATA, &value);
  if (ret < 0)
    {
      return ret;
    }

  value = (value & ~mask) | set;
  return imxrt_netc_mdio_write(phyaddr, NETC_PHY_PAGE_DATA, value);
}

static int imxrt_netc_yt8531_configure_delay(uint8_t phyaddr)
{
  uint16_t delays;
  int ret;

  /* Zephyr FRDM: clear RXC_DLY_EN, then program RGMII delay selects. */

  ret = imxrt_netc_phy_modify_ext(phyaddr, NETC_YT8531_CHIP_CONFIG,
                                  NETC_YT8531_CCR_RXC_DLY_EN, 0);
  if (ret < 0)
    {
      return ret;
    }

  delays = ((CONFIG_IMXRT_NETC_YT8531_RX_DELAY &
             NETC_YT8531_DELAY_MASK) << NETC_YT8531_RX_DELAY_SHIFT) |
           ((CONFIG_IMXRT_NETC_YT8531_TX_DELAY &
             NETC_YT8531_DELAY_MASK) << NETC_YT8531_TX_DELAY_SHIFT);

  ret = imxrt_netc_phy_modify_ext(phyaddr, NETC_YT8531_RGMII_CONFIG1,
                                  (NETC_YT8531_DELAY_MASK <<
                                   NETC_YT8531_RX_DELAY_SHIFT) |
                                  (NETC_YT8531_DELAY_MASK <<
                                   NETC_YT8531_TX_DELAY_SHIFT),
                                  delays);
  if (ret < 0)
    {
      nerr("NETC: YT8531 delay config failed at PHY %u: %d\n",
           phyaddr, ret);
      return ret;
    }

  ninfo("NETC: YT8531 PHY %u RGMII delay rx=%u tx=%u\n",
        phyaddr, CONFIG_IMXRT_NETC_YT8531_RX_DELAY,
        CONFIG_IMXRT_NETC_YT8531_TX_DELAY);
  return OK;
}

static int imxrt_netc_phy_link(uint8_t phyaddr,
                               struct imxrt_netc_link_s *link)
{
  uint16_t bmsr;
  uint16_t status;
  int ret;

  ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_BMSR, &bmsr);
  if (ret == OK)
    {
      ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_BMSR, &bmsr);
    }

  if (ret < 0)
    {
      return ret;
    }

  memset(link, 0, sizeof(*link));
  if ((bmsr & NETC_PHY_BMSR_LINK) == 0)
    {
      return OK;
    }

  if ((bmsr & NETC_PHY_BMSR_AN_COMPLETE) == 0)
    {
      return -EAGAIN;
    }

  ret = imxrt_netc_mdio_read(phyaddr, NETC_PHY_SPECIFIC_STATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  link->up = (status & NETC_PHY_STATUS_LINK) != 0;
  link->full_duplex = (status & NETC_PHY_STATUS_DUPLEX) != 0;

  switch (status & NETC_PHY_STATUS_SPEED_MASK)
    {
      case NETC_PHY_STATUS_SPEED_10:
        link->speed = 10;
        break;

      case NETC_PHY_STATUS_SPEED_100:
        link->speed = 100;
        break;

      case NETC_PHY_STATUS_SPEED_1000:
        link->speed = 1000;
        break;

      default:
        return -EPROTO;
    }

  return OK;
}

static void imxrt_netc_clean(void *address, size_t size)
{
  up_clean_dcache((uintptr_t)address, (uintptr_t)address + size);
}

static void imxrt_netc_invalidate(void *address, size_t size)
{
  up_invalidate_dcache((uintptr_t)address, (uintptr_t)address + size);
}

static void imxrt_netc_set_mac(struct imxrt_netc_driver_s *priv)
{
  const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;
  uint32_t low;
  uint32_t high;

  low = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
        ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
  high = (uint32_t)mac[4] | ((uint32_t)mac[5] << 8);

  putreg32(low, IMXRT_NETC_ENETC1_PSIPMAR0);
  putreg32(high, IMXRT_NETC_ENETC1_PSIPMAR1);

  /* Bring-up: accept untagged + UC/MC promiscuous so ARP/DHCP replies are
   * not dropped by default L2/VLAN filters while the data path is proven.
   */

  putreg32(NETC_ENETC_PSIPMMR_SI0_MAC_UP | NETC_ENETC_PSIPMMR_SI0_MAC_MP,
           IMXRT_NETC_ENETC1_PSIPMMR);
  putreg32(NETC_ENETC_PSIPVMR_SI0_VLAN_P | NETC_ENETC_PSIPVMR_SI0_VLAN_UTA,
           IMXRT_NETC_ENETC1_PSIPVMR);
}

static int imxrt_netc_reset_function(uintptr_t control,
                                     uintptr_t command,
                                     const char *name)
{
  uint16_t regval;
  int ret;

  /* NETC requires PCI memory and bus-master access before the function
   * reset so that the Ethernet MAC reset path can complete.
   */

  regval = getreg16(command);
  regval |= NETC_PCI_COMMAND_MEMORY | NETC_PCI_COMMAND_MASTER;
  putreg16(regval, command);

  regval = getreg16(control);
  putreg16(regval | NETC_PCI_DEVICE_CONTROL_FLR, control);
  ret = imxrt_netc_wait_clear(control, NETC_PCI_DEVICE_CONTROL_FLR);
  if (ret < 0)
    {
      nerr("NETC: %s function reset timed out\n", name);
      return ret;
    }

  /* FLR clears the command register. */

  regval = getreg16(command);
  regval |= NETC_PCI_COMMAND_MEMORY | NETC_PCI_COMMAND_MASTER;
  putreg16(regval, command);
  return OK;
}

static void
imxrt_netc_switch_port_set_mac(uint8_t port, const uint8_t *mac)
{
  uint32_t low;
  uint32_t high;

  low = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
        ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
  high = (uint32_t)mac[4] | ((uint32_t)mac[5] << 8);
  putreg32(low, NETC_SWITCH_PORT_PMAR0(port));
  putreg32(high, NETC_SWITCH_PORT_PMAR1(port));
}

static int imxrt_netc_eth_mac_software_reset(uint8_t port)
{
  int ret;

  /* Match SWT_Init: hold Tx/Rx disabled while clearing MAC FIFOs. */

  modifyreg32(NETC_SWITCH_PORT_POR(port), 0,
              NETC_SWITCH_PORT_TXRX_DISABLE);
  putreg32(getreg32(NETC_SWITCH_PORT_COMMAND(port)) | NETC_SWITCH_MAC_SWR,
           NETC_SWITCH_PORT_COMMAND(port));
  ret = imxrt_netc_wait_clear(NETC_SWITCH_PORT_COMMAND(port),
                              NETC_SWITCH_MAC_SWR);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(getreg32(NETC_SWITCH_PORT_COMMAND_PM1(port)) |
           NETC_SWITCH_MAC_SWR,
           NETC_SWITCH_PORT_COMMAND_PM1(port));
  return imxrt_netc_wait_clear(NETC_SWITCH_PORT_COMMAND_PM1(port),
                               NETC_SWITCH_MAC_SWR);
}

static void
imxrt_netc_switch_port_configure(uint8_t port,
                                 const struct imxrt_netc_link_s *link,
                                 const uint8_t *mac)
{
  uint32_t mode;
  uint32_t command;
  unsigned int tc;

  putreg32(NETC_SWITCH_PORT_SPEED_1G, NETC_SWITCH_PORT_PCR(port));
  putreg32(0, NETC_SWITCH_PORT_PSGCR(port));
  putreg32(NETC_SWITCH_PORT_PPDU_BCO, NETC_SWITCH_PORT_PRXSDUOR(port));
  putreg32(NETC_SWITCH_PORT_PPDU_BCO, NETC_SWITCH_PORT_PTXSDUOR(port));
  imxrt_netc_switch_port_set_mac(port, mac);
  modifyreg32(NETC_SWITCH_PORT_PTGSCR(port),
              NETC_SWITCH_PORT_TIME_GATE_ENABLE, 0);
  putreg32(NETC_SWITCH_PORT_TC_GATES_OPEN,
           NETC_SWITCH_PORT_PDGSR(port));
  for (tc = 0; tc < 8; tc++)
    {
      putreg32(NETC_SWITCH_PORT_TC_MPDU_MAX,
               NETC_SWITCH_PORT_PTCTMSDUR(port, tc));
    }

  putreg32(NETC_SWITCH_PORT_ACCEPT_ALL_TAGS |
           NETC_SWITCH_PORT_RXVAM,
           NETC_SWITCH_PORT_BPDVR(port));
  putreg32(NETC_SWITCH_STG0_FORWARD, NETC_SWITCH_PORT_BPSTGSR(port));
  putreg32(0, NETC_SWITCH_PORT_BPCR(port));
  putreg32(NETC_SWITCH_STORM_ENTRY_DISABLE,
           NETC_SWITCH_PORT_BPSCR0(port));
  putreg32(NETC_SWITCH_STORM_ENTRY_DISABLE,
           NETC_SWITCH_PORT_BPSCR1(port));

  if (port != 4)
    {
      mode = NETC_SWITCH_IFMODE_RGMII;
      if (link->speed == 1000)
        {
          mode |= NETC_SWITCH_IFMODE_SPEED_1000;
        }
      else if (link->speed == 100)
        {
          mode |= NETC_SWITCH_IFMODE_SPEED_100;
        }
      else
        {
          mode |= NETC_SWITCH_IFMODE_SPEED_10;
        }

      if (!link->full_duplex)
        {
          mode |= NETC_SWITCH_IFMODE_HALF_DUPLEX;
        }

      /* MCUX NETC_PortConfigEthMac programs express + preempt MACs. */

      putreg32(mode, NETC_SWITCH_PORT_IF_MODE(port));
      putreg32(mode, NETC_SWITCH_PORT_IF_MODE_PM1(port));
      putreg32(NETC_SWITCH_MAC_MAX_FRAME,
               NETC_SWITCH_PORT_MAXFRM(port));
      putreg32(NETC_SWITCH_MAC_MAX_FRAME,
               NETC_SWITCH_PORT_MAXFRM_PM1(port));
      putreg32(NETC_SWITCH_PORT_CUT_THROUGH_OFF,
               NETC_SWITCH_PORT_PCTFCR(port));
      command = NETC_SWITCH_MAC_TX_ENABLE |
                NETC_SWITCH_MAC_RX_ENABLE |
                NETC_SWITCH_MAC_CNT_FRM_EN |
                NETC_SWITCH_MAC_TX_PAD;
      putreg32(command, NETC_SWITCH_PORT_COMMAND(port));
      putreg32(command, NETC_SWITCH_PORT_COMMAND_PM1(port));
    }

  modifyreg32(NETC_SWITCH_PORT_POR(port),
              NETC_SWITCH_PORT_TXRX_DISABLE, 0);
}

static int imxrt_netc_switch_initialize(
  struct imxrt_netc_driver_s *priv,
  const struct imxrt_netc_link_s *port0,
  const struct imxrt_netc_link_s *port2)
{
  struct imxrt_netc_link_s internal =
  {
    .up = true,
    .full_duplex = true,
    .speed = 1000
  };

  const uint8_t *mac = priv->dev.d_mac.ether.ether_addr_octet;
  uint16_t cmd;
  int ret;

  ninfo("NETC: resetting switch eth MACs then function\n");

  /* SWT_Init enables PCI memory/master before eth MAC soft-reset. */

  cmd = getreg16(IMXRT_NETC_F2_COMMAND);
  cmd |= NETC_PCI_COMMAND_MEMORY | NETC_PCI_COMMAND_MASTER;
  putreg16(cmd, IMXRT_NETC_F2_COMMAND);

  ret = imxrt_netc_eth_mac_software_reset(0);
  if (ret < 0)
    {
      nerr("NETC: port 0 MAC soft-reset timed out\n");
      return ret;
    }

  ret = imxrt_netc_eth_mac_software_reset(2);
  if (ret < 0)
    {
      nerr("NETC: port 2 MAC soft-reset timed out\n");
      return ret;
    }

  ret = imxrt_netc_reset_function(IMXRT_NETC_F2_DEVICE_CONTROL,
                                  IMXRT_NETC_F2_COMMAND, "switch");
  if (ret < 0)
    {
      return ret;
    }

  ninfo("NETC: configuring switch core and ports\n");
  putreg32(NETC_SWITCH_VLAN_PORTS_MASK,
           IMXRT_NETC_SWITCH_VFHTDECR0);
  putreg32(0, IMXRT_NETC_SWITCH_VFHTDECR1);
  putreg32(NETC_SWITCH_FDB_LOOKUP_FLOOD |
           NETC_SWITCH_HW_MAC_LEARN,
           IMXRT_NETC_SWITCH_VFHTDECR2);

  imxrt_netc_switch_port_configure(0, port0, mac);
  imxrt_netc_switch_port_configure(2, port2, mac);
  imxrt_netc_switch_port_configure(4, &internal, mac);

  if (port0->up)
    {
      priv->active_port = 0;
      priv->active_phy = CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS;
    }
  else if (port2->up)
    {
      priv->active_port = 2;
      priv->active_phy = CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS;
    }
  else
    {
      return -ENETDOWN;
    }

  ninfo("NETC: switch selected external port %u\n", priv->active_port);
  return OK;
}

static int imxrt_netc_rings_initialize(struct imxrt_netc_driver_s *priv)
{
  uintptr_t address;
  uint32_t capability;
  unsigned int i;
  int ret;

  ninfo("NETC: enabling and resetting ENETC1 function\n");
  ret = imxrt_netc_reset_function(IMXRT_NETC_F4_DEVICE_CONTROL,
                                  IMXRT_NETC_F4_COMMAND, "ENETC1");
  if (ret < 0)
    {
      return ret;
    }

  capability = getreg32(IMXRT_NETC_ENETC1_ECAPR2);
  if (capability == 0 || capability == UINT32_MAX)
    {
      nerr("NETC: ENETC1 unavailable (clock or ELE/TRDC access)\n");
      return -EACCES;
    }

  ninfo("NETC: ENETC1 capability=%08" PRIx32 "\n", capability);
  memset(priv->txring, 0, sizeof(priv->txring));
  memset(priv->rxring, 0, sizeof(priv->rxring));
  priv->txhead = 0;
  priv->txclean = 0;
  priv->rxtail = 0;

  for (i = 0; i < NETC_RING_COUNT; i++)
    {
      priv->rxring[i].word0 = (uintptr_t)priv->rxpool[i];
    }

  imxrt_netc_clean(priv->txring, sizeof(priv->txring));
  imxrt_netc_clean(priv->rxring, sizeof(priv->rxring));
  imxrt_netc_invalidate(priv->rxpool, sizeof(priv->rxpool));

  /* Match the NXP management-ENETC setup: advertise one regular ring plus
   * one reserved management ring.  Regular Rx uses index 0; port-masquerade
   * Tx uses index 1 (MCUX cannot masquerade on SI0 hw ring 0).  Tx and Rx
   * register blocks are independent even when they share a BDR index.
   */

  putreg32(NETC_ENETC_PSICFGR0_TXRINGS(NETC_SI_BDR_COUNT) |
           NETC_ENETC_PSICFGR0_RXRINGS(NETC_SI_BDR_COUNT),
           IMXRT_NETC_ENETC1_PSICFGR0);
  imxrt_netc_set_mac(priv);

  /* RSS is disabled and RX uses a single ring, so one group of one ring. */

  putreg32(NETC_SI_SIRBGCR_NUM_GROUPS(0) |
           NETC_SI_SIRBGCR_RINGS_PER_GROUP(1),
           IMXRT_NETC_SI_SIRBGCR);

  address = (uintptr_t)priv->txring;
  putreg32((uint32_t)address, IMXRT_NETC_SI_TBBAR0(NETC_TX_BDR));
  putreg32(0, IMXRT_NETC_SI_TBBAR1(NETC_TX_BDR));
  putreg32(0, IMXRT_NETC_SI_TBPIR(NETC_TX_BDR));
  putreg32(0, IMXRT_NETC_SI_TBCIR(NETC_TX_BDR));
  putreg32(NETC_RING_COUNT, IMXRT_NETC_SI_TBLENR(NETC_TX_BDR));
  putreg32(0, IMXRT_NETC_SI_TBIER(NETC_TX_BDR));
  putreg32(NETC_SI_TBMR_ENABLE, IMXRT_NETC_SI_TBMR(NETC_TX_BDR));

  address = (uintptr_t)priv->rxring;
  putreg32((uint32_t)address, IMXRT_NETC_SI_RBBAR0(NETC_RX_BDR));
  putreg32(0, IMXRT_NETC_SI_RBBAR1(NETC_RX_BDR));
  putreg32(NETC_DMA_BUFFER_SIZE, IMXRT_NETC_SI_RBBSR(NETC_RX_BDR));
  putreg32(0, IMXRT_NETC_SI_RBCIR(NETC_RX_BDR));
  putreg32(NETC_RING_COUNT, IMXRT_NETC_SI_RBLENR(NETC_RX_BDR));
  putreg32(0, IMXRT_NETC_SI_RBIER(NETC_RX_BDR));
  putreg32(NETC_SI_RBMR_ENABLE, IMXRT_NETC_SI_RBMR(NETC_RX_BDR));

  modifyreg32(IMXRT_NETC_ENETC1_PMR, 0, NETC_ENETC_PMR_SI0EN);
  modifyreg32(IMXRT_NETC_SI_SIMR, 0, NETC_SI_SIMR_ENABLE);

  if ((getreg32(IMXRT_NETC_SI_SIMR) & NETC_SI_SIMR_ENABLE) == 0 ||
      (getreg32(IMXRT_NETC_SI_RBMR(NETC_RX_BDR)) &
       NETC_SI_RBMR_ENABLE) == 0)
    {
      nerr("NETC: ENETC1 descriptor enable rejected "
           "(BLK_CTRL/IERB/TRDC)\n");
      return -EACCES;
    }

  ninfo("NETC: ENETC1 polling descriptor rings enabled\n");
  return OK;
}

static void imxrt_netc_reclaim_tx(struct imxrt_netc_driver_s *priv)
{
  uint16_t consumer =
    getreg32(IMXRT_NETC_SI_TBCIR(NETC_TX_BDR)) % NETC_RING_COUNT;

  while (priv->txclean != consumer)
    {
      priv->txclean = (priv->txclean + 1) % NETC_RING_COUNT;
    }
}

static int imxrt_netc_transmit(struct imxrt_netc_driver_s *priv)
{
  struct imxrt_netc_desc_s *desc;
  uint16_t next;
  uint16_t framelen;
  uint16_t buflen;
  uint16_t consumer;

  imxrt_netc_reclaim_tx(priv);
  next = (priv->txhead + 1) % NETC_RING_COUNT;
  if (next == priv->txclean)
    {
      return -EBUSY;
    }

  if (priv->dev.d_len < 16 || priv->dev.d_len > NETC_DMA_BUFFER_SIZE)
    {
      NETDEV_TXERRORS(&priv->dev);
      return -EMSGSIZE;
    }

  framelen = priv->dev.d_len;
  buflen = framelen < NETC_ETH_MIN_FRAME ? NETC_ETH_MIN_FRAME : framelen;
  memset(priv->txpool[priv->txhead], 0, buflen);
  memcpy(priv->txpool[priv->txhead], priv->dev.d_buf, framelen);
  imxrt_netc_clean(priv->txpool[priv->txhead], buflen);

  desc = &priv->txring[priv->txhead];
  memset(desc, 0, sizeof(*desc));
  desc->word0 = (uintptr_t)priv->txpool[priv->txhead];
  desc->word1 = (uint64_t)buflen |
                ((uint64_t)buflen << 16) |
                NETC_TX_SWITCH_MASQUERADE(4) |
                NETC_TX_FINAL;
  imxrt_netc_clean(desc, sizeof(*desc));

  priv->txhead = next;
  putreg32(priv->txhead, IMXRT_NETC_SI_TBPIR(NETC_TX_BDR));
  consumer =
    getreg32(IMXRT_NETC_SI_TBCIR(NETC_TX_BDR)) % NETC_RING_COUNT;
  ninfo("NETC: TX queued len=%u ingress=4 egress=%u head=%u cir=%u\n",
        framelen, priv->active_port, priv->txhead, consumer);
  NETDEV_TXPACKETS(&priv->dev);
  NETDEV_TXDONE(&priv->dev);
  priv->dev.d_len = 0;
  return OK;
}

static int imxrt_netc_txpoll(struct net_driver_s *dev)
{
  return imxrt_netc_transmit(dev->d_private);
}

static void imxrt_netc_dispatch(struct imxrt_netc_driver_s *priv)
{
  struct eth_hdr_s *hdr = (struct eth_hdr_s *)priv->dev.d_buf;

  NETDEV_RXPACKETS(&priv->dev);

#ifdef CONFIG_NET_IPv4
  if (hdr->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(&priv->dev);
      ipv4_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (hdr->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(&priv->dev);
      ipv6_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (hdr->type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->dev);
      arp_input(&priv->dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->dev);
      priv->dev.d_len = 0;
    }

  if (priv->dev.d_len > 0)
    {
      imxrt_netc_transmit(priv);
    }
}

static void imxrt_netc_receive(struct imxrt_netc_driver_s *priv)
{
  struct imxrt_netc_desc_s *desc;
  unsigned int budget = NETC_RING_COUNT;
  uint16_t length;

  while (budget-- > 0)
    {
      desc = &priv->rxring[priv->rxtail];
      imxrt_netc_invalidate(desc, sizeof(*desc));
      if ((desc->word1 & NETC_RX_READY) == 0)
        {
          break;
        }

      length = desc->word1 & 0xffffu;
      ninfo("NETC: RX ready len=%u word1=%016" PRIx64 "\n",
            length, desc->word1);
      if ((desc->word1 & NETC_RX_FINAL) == 0 ||
          ((desc->word1 >> NETC_RX_ERROR_SHIFT) & 0xffu) != 0 ||
          length == 0 || length > NETC_DMA_BUFFER_SIZE ||
          length > sizeof(priv->buffer))
        {
          NETDEV_RXERRORS(&priv->dev);
        }
      else
        {
          imxrt_netc_invalidate(priv->rxpool[priv->rxtail], length);
          memcpy(priv->buffer, priv->rxpool[priv->rxtail], length);
          priv->dev.d_buf = priv->buffer;
          priv->dev.d_len = length;
          imxrt_netc_dispatch(priv);
        }

      desc->word0 = (uintptr_t)priv->rxpool[priv->rxtail];
      desc->word1 = 0;
      imxrt_netc_clean(desc, sizeof(*desc));
      priv->rxtail = (priv->rxtail + 1) % NETC_RING_COUNT;
      putreg32(priv->rxtail, IMXRT_NETC_SI_RBCIR(NETC_RX_BDR));
    }

  priv->dev.d_buf = priv->buffer;
  priv->dev.d_len = 0;
}

static void imxrt_netc_poll_work(void *arg)
{
  struct imxrt_netc_driver_s *priv = arg;
  static unsigned int diagnostic_ticks;

  net_lock();
  if (priv->ifup)
    {
      imxrt_netc_receive(priv);
      imxrt_netc_reclaim_tx(priv);
      devif_poll(&priv->dev, imxrt_netc_txpoll);
      if (++diagnostic_ticks == 100)
        {
          /* Named counters only; never scan reserved gaps
           * (fault @ +0x1d0).
           */

          ninfo("NETC: path counters ext-rx=%" PRIu32
                " ext-tx=%" PRIu32
                " p0-rxd=%" PRIu32 "/%08" PRIx32
                " p0-txd=%" PRIu32 "/%08" PRIx32
                " p0-brd=%" PRIu32 "/%08" PRIx32
                " p4-brd=%" PRIu32 "/%08" PRIx32
                " conduit-bcast=%" PRIu32
                " si-frames=%" PRIu32 " rbpir=%" PRIu32 "\n",
                getreg32(NETC_SWITCH_PORT_RX_PACKETS(0)),
                getreg32(NETC_SWITCH_PORT_TX_PACKETS(0)),
                getreg32(NETC_SWITCH_PORT_RX_DISCARD(0)),
                getreg32(NETC_SWITCH_PORT_RX_DISCARD_R0(0)),
                getreg32(NETC_SWITCH_PORT_TX_DISCARD(0)),
                getreg32(NETC_SWITCH_PORT_TX_DISCARD_R0(0)),
                getreg32(NETC_SWITCH_PORT_BRIDGE_DISCARD(0)),
                getreg32(NETC_SWITCH_PORT_BRIDGE_DISCARD_R0(0)),
                getreg32(NETC_SWITCH_PORT_BRIDGE_DISCARD(4)),
                getreg32(NETC_SWITCH_PORT_BRIDGE_DISCARD_R0(4)),
                getreg32(NETC_SWITCH_PSEUDO_RX_BCAST(4)),
                getreg32(IMXRT_NETC_ENETC1_SI_BASE + 0x0308u),
                getreg32(IMXRT_NETC_SI_RBPIR(NETC_RX_BDR)));
          diagnostic_ticks = 0;
        }

      wd_start(&priv->polltimer, NETC_POLL_DELAY,
               imxrt_netc_poll_expiry, (wdparm_t)priv);
    }

  net_unlock();
}

static void imxrt_netc_poll_expiry(wdparm_t arg)
{
  struct imxrt_netc_driver_s *priv =
    (struct imxrt_netc_driver_s *)arg;

  work_queue(ETHWORK, &priv->pollwork, imxrt_netc_poll_work, priv, 0);
}

static int imxrt_netc_wait_link(struct imxrt_netc_link_s *port0,
                                struct imxrt_netc_link_s *port2,
                                int *port0_ret, int *port2_ret)
{
  unsigned int elapsed = 0;
  unsigned int delay;
  int ret;

  for (; ; )
    {
      *port0_ret =
        imxrt_netc_phy_link(CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS, port0);
      *port2_ret =
        imxrt_netc_phy_link(CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS, port2);

      if ((*port0_ret == OK && port0->up) ||
          (*port2_ret == OK && port2->up))
        {
          return OK;
        }

      if (elapsed >= CONFIG_IMXRT_NETC_LINK_TIMEOUT_MS)
        {
          return -ETIMEDOUT;
        }

      delay = CONFIG_IMXRT_NETC_LINK_TIMEOUT_MS - elapsed;
      if (delay > NETC_LINK_POLL_INTERVAL_MS)
        {
          delay = NETC_LINK_POLL_INTERVAL_MS;
        }

      ret = nxsig_usleep(delay * 1000u);
      if (ret < 0)
        {
          return ret;
        }

      elapsed += delay;
    }
}

static int imxrt_netc_ifup(struct net_driver_s *dev)
{
  struct imxrt_netc_driver_s *priv = dev->d_private;
  struct imxrt_netc_link_s port0;
  struct imxrt_netc_link_s port2;
  int port0_ret;
  int port2_ret;
  int ret;

  memset(&port0, 0, sizeof(port0));
  memset(&port2, 0, sizeof(port2));
  ret = imxrt_netc_wait_link(&port0, &port2, &port0_ret, &port2_ret);

  ninfo("NETC: switch port 0 PHY %u link=%s status=%d speed=%u "
        "duplex=%s\n", CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS,
        port0_ret == OK && port0.up ? "up" : "down", port0_ret,
        port0.speed, port0.full_duplex ? "full" : "half");
  ninfo("NETC: switch port 2 PHY %u link=%s status=%d speed=%u "
        "duplex=%s\n", CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS,
        port2_ret == OK && port2.up ? "up" : "down", port2_ret,
        port2.speed, port2.full_duplex ? "full" : "half");

  if (ret < 0)
    {
      nerr("NETC: no negotiated PHY link after %u ms: %d\n",
           CONFIG_IMXRT_NETC_LINK_TIMEOUT_MS, ret);
      return ret;
    }

  ret = imxrt_netc_switch_initialize(priv, &port0, &port2);
  if (ret < 0)
    {
      nerr("NETC: no usable FRDM switch link: %d\n", ret);
      return ret;
    }

  ret = imxrt_netc_rings_initialize(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->ifup = true;
  netdev_carrier_on(dev);
  wd_start(&priv->polltimer, NETC_POLL_DELAY,
           imxrt_netc_poll_expiry, (wdparm_t)priv);
  ninfo("NETC: eth0 up through switch port %u PHY %u "
        "(polling data path)\n", priv->active_port, priv->active_phy);
  return OK;
}

static int imxrt_netc_ifdown(struct net_driver_s *dev)
{
  struct imxrt_netc_driver_s *priv = dev->d_private;

  priv->ifup = false;
  wd_cancel(&priv->polltimer);
  work_cancel(ETHWORK, &priv->pollwork);
  modifyreg32(IMXRT_NETC_SI_SIMR, NETC_SI_SIMR_ENABLE, 0);
  modifyreg32(IMXRT_NETC_ENETC1_PMR, NETC_ENETC_PMR_SI0EN, 0);
  netdev_carrier_off(dev);
  return OK;
}

static int imxrt_netc_txavail(struct net_driver_s *dev)
{
  struct imxrt_netc_driver_s *priv = dev->d_private;

  if (priv->ifup && work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, imxrt_netc_poll_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_mdio_read(uint8_t phyaddr, uint8_t regaddr,
                         uint16_t *value)
{
  uint32_t config;
  int ret;

  if (value == NULL || phyaddr > 31 || regaddr > 31)
    {
      return -EINVAL;
    }

  config = getreg32(IMXRT_NETC_EMDIO_CFG);
  config &= ~NETC_EMDIO_CFG_CLAUSE45;
  putreg32(config, IMXRT_NETC_EMDIO_CFG);
  putreg32(NETC_EMDIO_CONTROL_READ |
           ((uint32_t)phyaddr << NETC_EMDIO_CONTROL_PHY_SHIFT) |
           ((uint32_t)regaddr << NETC_EMDIO_CONTROL_REG_SHIFT),
           IMXRT_NETC_EMDIO_CONTROL);

  ret = imxrt_netc_wait_clear(IMXRT_NETC_EMDIO_CFG,
                              NETC_EMDIO_CFG_BUSY);
  if (ret < 0)
    {
      return ret;
    }

  config = getreg32(IMXRT_NETC_EMDIO_CFG);
  if ((config & (NETC_EMDIO_CFG_READ_ERROR |
                 NETC_EMDIO_CFG_ADDRESS_ERROR)) != 0)
    {
      return -EIO;
    }

  *value = getreg32(IMXRT_NETC_EMDIO_DATA) & 0xffffu;
  return OK;
}

int imxrt_netc_initialize(void)
{
  struct imxrt_netc_driver_s *priv = &g_netc;
  uint8_t *mac;
  int port0;
  int port2;
  int ret;

  if (priv->registered)
    {
      return -EALREADY;
    }

  memset(priv, 0, sizeof(*priv));
  ret = imxrt_netc_emdio_initialize();
  if (ret < 0)
    {
      nerr("NETC: EMDIO initialization failed: %d\n", ret);
      return ret;
    }

  port0 = imxrt_netc_probe_phy(CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS,
                               "port 0");
  port2 = imxrt_netc_probe_phy(CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS,
                               "port 2");
  if (port0 < 0 && port2 < 0)
    {
      return -ENODEV;
    }

  if (port0 == OK)
    {
      ret = imxrt_netc_yt8531_configure_delay(
              CONFIG_IMXRT_NETC_PORT0_PHY_ADDRESS);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (port2 == OK)
    {
      ret = imxrt_netc_yt8531_configure_delay(
              CONFIG_IMXRT_NETC_PORT2_PHY_ADDRESS);
      if (ret < 0)
        {
          return ret;
        }
    }

  priv->dev.d_buf       = priv->buffer;
  priv->dev.d_ifup      = imxrt_netc_ifup;
  priv->dev.d_ifdown    = imxrt_netc_ifdown;
  priv->dev.d_txavail   = imxrt_netc_txavail;
  priv->dev.d_private   = priv;

  /* Fork-local bench MAC until a board-unique source is wired. */

  mac = priv->dev.d_mac.ether.ether_addr_octet;
  mac[0] = 0x02;
  mac[1] = 0x00;
  mac[2] = 0x00;
  mac[3] = 0x01;
  mac[4] = 0x03;
  mac[5] = 0x00;

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret == OK)
    {
      priv->registered = true;
      ninfo("NETC: registered ENETC1/switch interface eth0; "
            "data path uses honest 10 ms polling\n");
    }

  return ret;
}
