/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mii.h>
#include <stdbool.h>
#include <stdint.h>
#include <arch/imxrt/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Total amount of external ports / PHY devices supported
 * by NETC. This is also the port register index limit.
 */

#define IMXRT_NETC_PHY_PORT_COUNT    5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MDIO access endpoints.
 * PORT0..PORT3 access the PHYs on switch physical links 0..3.
 * ENETC0 accesses the PHY on physical link 4.
 * Per-port endpoints require a fixed PHY address matching the
 * corresponding IERB LINKn_BCR PHY address field. If IERB is already
 * locked, that field must already match the configured address.
 * EMDIO accesses the external MDIO bus and supports PHY address probing.
 */

enum imxrt_netc_mdio_e
{
  IMXRT_NETC_EMDIO = 0,
  IMXRT_NETC_PORT0_MDIO,
  IMXRT_NETC_PORT1_MDIO,
  IMXRT_NETC_PORT2_MDIO,
  IMXRT_NETC_PORT3_MDIO,
  IMXRT_NETC_ENETC0_MDIO,
  IMXRT_NETC_MDIO_COUNT
};

struct imxrt_netc_port_s
{
  bool enabled;
  struct phy_desc_s phy;
};

struct imxrt_netc_enetc_s
{
  bool enabled;
  uint8_t msgintr;        /* Exclusively reserved MSGINTR channel 1..6 */
  uint8_t mac[6];
  struct imxrt_netc_port_s ports[IMXRT_NETC_SW0_NPORTS];
};

struct imxrt_netc_config_s
{
  struct imxrt_netc_enetc_s enetc[IMXRT_NETC_NENETC];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Function: board_netc_phyinit
 *
 * Description:
 *   Perform board-specific PHY initialization, such as configuring GPIOs
 *   and resetting the PHY. When CONFIG_IMXRT_NETC_PHYINIT is enabled,
 *   board logic must provide this function. The NETC driver calls it
 *   during port preparation, after interface mode and clock-direction
 *   setup and before MDIO probing.
 *
 * Input Parameters:
 *   if_idx - Physical NETC link index (0..4), identifying the PHY to
 *            initialize.
 *   config - Configuration for the port, including its PHY settings.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_NETC_PHYINIT
int board_netc_phyinit(unsigned int if_idx,
                       const struct imxrt_netc_port_s *config);
#endif

/****************************************************************************
 * Function: imxrt_netc_initialize
 *
 * Description:
 *   Initialize shared NETC resources and register the enabled ENETC
 *   network interfaces.
 *
 * Input Parameters:
 *   config - Board-specific configuration selecting enabled ENETC
 *            instances and ports, MAC addresses, interrupt channels,
 *            and per-port PHY settings.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int imxrt_netc_initialize(const struct imxrt_netc_config_s *config);

#ifdef __cplusplus
}
#endif
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_NETC_H */
