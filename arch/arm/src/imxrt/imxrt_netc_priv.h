/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_priv.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/cache.h>
#include <nuttx/compiler.h>
#include <nuttx/net/mdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <arch/barriers.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_netc.h"
#include "hardware/imxrt_netc.h"
#include "hardware/imxrt_netc_switch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETC_DMA_ALIGN        128
#define NETC_PACKET_SIZE      CONFIG_NET_ETH_PKTSIZE
#define NETC_BUFFER_SIZE      ((NETC_PACKET_SIZE + NETC_DMA_ALIGN - 1) & \
                              ~(NETC_DMA_ALIGN - 1))
#define NETC_TX_SLOTS         (CONFIG_IMXRT_NETC_NTXDESC / 2)
#define NETC_RX_SLOTS         CONFIG_IMXRT_NETC_NRXDESC
#define NETC_TX_QUOTA         (NETC_TX_SLOTS - 1)

#ifdef CONFIG_ARCH_CORTEXM33
#  define NETC_DCACHE_LINESIZE ARMV8M_DCACHE_LINESIZE
#else
#  define NETC_DCACHE_LINESIZE ARMV7M_DCACHE_LINESIZE
#endif

#define IMXRT_NETC_EMDIO_PF   IMXRT_NETC_PF(1)
#define IMXRT_NETC_SW0_PF     IMXRT_NETC_PF(2)
#define IMXRT_NETC_ENETC0_PF  IMXRT_NETC_PF(3)
#define IMXRT_NETC_ENETC1_PF  IMXRT_NETC_PF(4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum imxrt_netc_attachment_e
{
  IMXRT_NETC_DIRECT = 0,
  IMXRT_NETC_SWITCH
};

struct imxrt_netc_ports_hw_s
{
  uint8_t reg_idx;
  enum imxrt_netc_mdio_e mdio;
};

struct imxrt_netc_hw_s
{
  uintptr_t si;
  uintptr_t enetc;
  uintptr_t port;
  uintptr_t mac;
  uintptr_t function;
  uintptr_t msix;
  enum imxrt_netc_attachment_e attachment;
  uint8_t portcount;
  uint8_t sw_cpu_port_reg_idx;
  struct imxrt_netc_ports_hw_s ports[IMXRT_NETC_SW0_NPORTS];
};

struct imxrt_netc_link_s
{
  struct mdio_bus_s *bus;         /* Registered endpoint, shared by PHYs */
  struct phy_device_s *phydev;
  uint16_t speed;                 /* Zero means link down */
};

static_assert(
  sizeof(struct imxrt_netc_txdesc_s) % NETC_DCACHE_LINESIZE == 0 &&
  sizeof(struct imxrt_netc_rxdesc_s) % NETC_DCACHE_LINESIZE == 0 &&
  sizeof(struct imxrt_netc_cmd_s) % NETC_DCACHE_LINESIZE == 0 &&
  NETC_DMA_ALIGN % NETC_DCACHE_LINESIZE == 0,
  "NETC DMA entries must occupy whole cache lines");

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int imxrt_netc_wait(uintptr_t reg, uint32_t mask, uint32_t value);
int imxrt_netc_reset(uintptr_t function, uintptr_t common);
void *imxrt_netc_alloc(size_t size, uint32_t *dma_bus);

int imxrt_netc_port_prepare(const struct imxrt_netc_ports_hw_s *hw,
                           const struct imxrt_netc_port_s *config);
int imxrt_netc_port_initialize(const struct imxrt_netc_ports_hw_s *hw,
                              const struct imxrt_netc_port_s *config,
                              struct imxrt_netc_link_s *link,
                              uint32_t clock_hz);
void imxrt_netc_port_uninitialize(const struct imxrt_netc_ports_hw_s *hw,
                                 struct imxrt_netc_link_s *link);
int imxrt_netc_port_link(struct imxrt_netc_link_s *link, uint16_t *speed);
int imxrt_netc_port_mode(uintptr_t port, uintptr_t mac,
                        enum phy_interface_e mode, uint16_t speed);
void imxrt_netc_port_sdu(uintptr_t port);
void imxrt_netc_pseudo(uintptr_t port);

int imxrt_netc_switch_initialize(const struct imxrt_netc_hw_s *hw,
                                const struct imxrt_netc_enetc_s *config,
                                uint32_t clock_hz);
int imxrt_netc_switch_uninitialize(void);
int imxrt_netc_switch_cpu(bool enable, const uint8_t *mac);
int imxrt_netc_switch_link(uint8_t *linkmask);

#ifdef CONFIG_NETDEV_IOCTL
int imxrt_netc_switch_ioctl(int cmd, unsigned long arg);
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_PRIV_H */
