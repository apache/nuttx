/****************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_enet.h
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

#ifndef __ARCH_ARM64_SRC_ZYNQ_MPSOC_ZYNQ_ENET_H
#define __ARCH_ARM64_SRC_ZYNQ_MPSOC_ZYNQ_ENET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wqueue.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/phy.h>

#include "hardware/zynq_gmac.h"

#ifdef CONFIG_ZYNQ_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definitions for use with zynq_phy_boardinitialize */

#define EMAC_INTF 0

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum phy_interface_s
{
    PHY_INTERFACE_MODE_MII,
    PHY_INTERFACE_MODE_GMII,
    PHY_INTERFACE_MODE_SGMII,
    PHY_INTERFACE_MODE_QSGMII,
    PHY_INTERFACE_MODE_TBI,
    PHY_INTERFACE_MODE_RMII,
    PHY_INTERFACE_MODE_RGMII,
    PHY_INTERFACE_MODE_RGMII_ID,
    PHY_INTERFACE_MODE_RGMII_RXID,
    PHY_INTERFACE_MODE_RGMII_TXID,
    PHY_INTERFACE_MODE_RTBI,
    PHY_INTERFACE_MODE_XGMII,
    PHY_INTERFACE_MODE_UNKNOWN,
    PHY_INTERFACE_MODE_NONE /* Must be last */
};

struct zynq_gmac_s;

/* The zynq_phy_s encapsulates all state information for phy chip */

struct phy_s
{
  uint32_t              uid;       /* PHY id (set by boardinitialize) */
  void                  *priv;     /* PHY private Interface */

  /* Called to configure the PHY, and modify the controller
   * based on the results.  Should be called after phy_connect
   */

  int (*config)(struct zynq_gmac_s *gmac);
  enum phy_interface_s  interface;   /* PHY interface type */
  uint8_t               phyaddr;     /* PHY address (pre-defined by pins on reset) */
};

/* The zynq_gmac_s encapsulates all state information for GMAC peripheral */

struct zynq_gmac_s
{
  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;         /* Interface understood by the network */

  const uint32_t        base;        /* Base address of ENET controller */
  uint8_t               num;         /* gmac 0 1 2 3 */
  bool                  ifup;        /* true:ifup false:ifdown */
  const int             irq;         /* Enet interrupt */
  struct wdog_s         txtimeout;   /* TX timeout timer */
  struct work_s         irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s         pollwork;    /* For deferring poll work to the work queue */

  struct phy_s          phy;         /* Used to holds phy information */

  /* Used to track transmit and receive descriptors */

  uint16_t              txhead;      /* Circular buffer head index */
  uint16_t              txtail;      /* Circualr buffer tail index */
  uint16_t              rxndx;       /* RX index for current processing RX descriptor */

  uint8_t              *rxbuffer;    /* Allocated RX buffers */
  uint8_t              *txbuffer;    /* Allocated TX buffers */
  struct gmac_rxdesc_s *rxdesc;      /* Allocated RX descriptors */
  struct gmac_txdesc_s *txdesc;      /* Allocated TX descriptors */

  /* Debug stuff */

#ifdef CONFIG_ZYNQ_GMAC_REGDEBUG
  bool                  wrlast;     /* Last was a write */
  uintptr_t             addrlast;   /* Last address */
  uint32_t              vallast;    /* Last value */
  int                   ntimes;     /* Number of times */
#endif
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: zynq_gmac_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value identifies
 *          which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int zynq_gmac_initialize(int intf);

/****************************************************************************
 * Function: zynq_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_ZYNQ_ENET_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   zynq_phyinitialize();  The ZYNQ MPSOC Ethernet driver will call this
 *   function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   gmac - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ZYNQ_ENET_PHYINIT
int zynq_phy_boardinitialize(struct zynq_gmac_s *gmac);
#endif

/****************************************************************************
 * Function: zynq_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int zynq_phyread(struct zynq_gmac_s *priv, uint8_t phyaddr,
                 uint8_t regaddr, uint16_t *phyval);

/****************************************************************************
 * Function: zynq_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *   phyaddr - The PHY device address
 *   regaddr - The PHY register address
 *   phyval - The 16-bit value to write to the PHY register.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int zynq_phywrite(struct zynq_gmac_s *priv, uint8_t phyaddr,
                  uint8_t regaddr, uint16_t phyval);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ZYNQ_ENET */
#endif /* __ARCH_ARM64_SRC_ZYNQ_MPSOC_ZYNQ_ENET_H */
