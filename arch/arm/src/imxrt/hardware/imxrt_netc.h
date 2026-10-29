/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_netc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base address helpers.
 * (IMXRT1180RM Rev. 10, chapters 3, 53 and 54.)
 */

#define IMXRT_NETC_PF(n)            (IMXRT_NETC_ECAM_BASE + (n) * 0x1000)
#define IMXRT_NETC_EMDIO_REGS       (IMXRT_EMDIO_BASE + 0x1c00)
#define IMXRT_NETC_ENETC0_MDIO_REGS (IMXRT_ENETC0_ETH_MAC_BASE + 0xc00)
#define IMXRT_NETC_PORT_MDIO(n)     (IMXRT_SW0_BASE + 0x5c00 + (n) * 0x4000)
#define IMXRT_NETC_SW_PORT(n)       (IMXRT_SW0_BASE + ((n) + 1) * 0x4000)
#define IMXRT_NETC_SW_MAC(n)        (IMXRT_NETC_SW_PORT(n) + 0x1000)

/* BLK_CTRL_WAKEUP (IMXRT1180RM Rev. 10, section 16.6.1.)
 * These protocol encodings differ from NETC PMIFR.
 */

#define IMXRT_NETC_PORT_MISC_CFG        (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x24)
#define NETC_PORT_MISC_RMII_CLK_OUT(n)  (1u << (n))
#define IMXRT_NETC_LINK_CFG(n)          (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + \
                                          0x100 + (n) * 4)
#define NETC_LINK_CFG_MII_PROT_MASK     0x0fu
#define NETC_LINK_CFG_MII               0u
#define NETC_LINK_CFG_RMII              1u
#define NETC_LINK_CFG_RGMII             2u
#define NETC_LINK_CFG_REVMII            (1u << 31)

#define NETC_PRB_NETCRR                 0x0100
#define NETC_PRB_NETCSR                 0x0104
#define NETC_PRB_SR                     (1 << 0)
#define NETC_PRB_LOCK                   (1 << 1)
#define NETC_PRB_ERROR                  (1 << 0)
#define NETC_PRB_STATE                  (1 << 1)
#define NETC_IERB_CLKFR                 0x0178
#define NETC_IERB_CLKCR                 0x017c
#define NETC_IERB_CLOCK(mhz, ns)        ((mhz) | ((ns) << 16))

/* Nonzero whole-MHz FREQ encoding limits, not device operating limits. */

#define NETC_IERB_CLKCR_MIN_HZ      1000000u
#define NETC_IERB_CLKCR_MAX_HZ      2047000000u

#define NETC_IERB_LINKBCR(n)        (0x1010 + (n) * 0x40)
#define NETC_IERB_PHYADDR_MASK      (31 << 8)
#define NETC_IERB_PHYADDR(n)        ((uint32_t)(n) << 8)

#define NETC_MSGINTR_MSIIR          0x00
#define NETC_MSGINTR_MSIR           0x04
#define NETC_MSIX_ADDRESS           0x00
#define NETC_MSIX_ADDRESS_HI        0x04
#define NETC_MSIX_DATA              0x08
#define NETC_MSIX_CONTROL           0x0c
#define NETC_MSIX_VECTOR_MASK       1

/* ECAM configuration offsets and masks.
 * (IMXRT1180RM Rev. 10, sections 53.4.2.2 and 53.4.6.)
 */

#define NETC_PCI_CMD                0x0004
#define NETC_PCI_CMD_MEM_ACCESS     (1 << 1)
#define NETC_PCI_CMD_ENABLE         0x0006
#define NETC_PCI_DEVCTL             0x0048
#define NETC_PCI_FLR                (1 << 15)
#define NETC_PCI_MSIXCTL            0x0082
#define NETC_PCI_MSIX_ENABLE        (1 << 15)
#define NETC_PCI_MSIX_MASK          (1 << 14)
#define NETC_PCI_SRIOVCTL           0x0158
#define NETC_PCI_VF_ENABLE          (1 << 0)

/* Switch and ENETC common registers (IMXRT1180RM sec. 53.4.6.7). */

#define NETC_COMMON_OSR             0x1010
#define NETC_COMMON_STATE           (1 << 0)
#define NETC_COMMON_ITM_STATE       (1 << 1)

#define NETC_SI_SIMR                0x0000
#define NETC_SI_EN                  (1u << 31)
#define NETC_SI_SIRBGCR             0x0038
#define NETC_SI_SIBCAR              0x0040
#define NETC_DMA_NONCOHERENT        0x02020202
#define NETC_SI_SIIPVBDRMR0         0x0150
#define NETC_SI_SIMSITRVR(n)        (0x0b00 + (n) * 4)
#define NETC_SI_SIMSIRRVR(n)        (0x0b80 + (n) * 4)

#define NETC_TX_BASE(n)             (0x8000 + (n) * 0x200)
#define NETC_RX_BASE(n)             (0x8100 + (n) * 0x200)
#define NETC_BDR_MR                 0x00
#define NETC_BDR_EN                 (1u << 31)
#define NETC_RX_EXTENDED            (1 << 2)
#define NETC_BDR_SR                 0x04
#define NETC_BDR_SBE                (1 << 16)
#define NETC_RX_BSR                 0x08
#define NETC_RX_CIR                 0x0c
#define NETC_BDR_BAR0               0x10
#define NETC_BDR_BAR1               0x14
#define NETC_BDR_PIR                0x18
#define NETC_TX_CIR                 0x1c
#define NETC_BDR_LENR               0x20
#define NETC_BDR_IER                0xa0
#define NETC_BDR_IDR                0xa4
#define NETC_TX_FRAME_IE            (1 << 1)
#define NETC_RX_THRESHOLD_IE        (1 << 0)
#define NETC_BDR_ICR0               0xa8
#define NETC_RX_COALESCE            (1u << 31)
#define NETC_RX_THRESHOLD           1
#define NETC_BDR_ICR1               0xac
#define NETC_BDR_INDEX_MASK         0xffff

#define NETC_ENETC_PMR              0x0010
#define NETC_ENETC_SI0EN            (1 << 16)
#define NETC_ENETC_PARCSCR          0x009c
#define NETC_ENETC_PSIPMMR          0x0200
#define NETC_ENETC_MCPROMISC        (1 << 16)
#define NETC_ENETC_PSIPVMR          0x0204
#define NETC_ENETC_VLANPROMISC      (1 << 0)
#define NETC_ENETC_VLANUNTAGGED     (1 << 16)
#define NETC_ENETC_PRIO2TCMR0       0x0380
#define NETC_ENETC_SMHRBDRMR(n)     (0x0880 + ((n) - 1) * 4)
#define NETC_ENETC_PSICFGR0         0x2010
#define NETC_ENETC_PSICFGR2         0x2018
#define NETC_ENETC_RINGS(tx, rx)    ((uint32_t)(tx) | ((uint32_t)(rx) << 16))

#define NETC_PORT_PCR               0x0010
#define NETC_PORT_PSPEED_MASK       (0x3fffu << 16)
#define NETC_PORT_PSPEED(n)         ((uint32_t)(n) << 16)
#define NETC_PORT_PMAR0             0x0020
#define NETC_PORT_PMAR1             0x0024
#define NETC_PORT_POR               0x0100
#define NETC_PORT_DISABLE           3
#define NETC_PORT_PSR               0x0104
#define NETC_PORT_BUSY              3
#define NETC_PORT_RXBUSY            (1 << 1)
#define NETC_PORT_PRXSDUOR          0x0108
#define NETC_PORT_PTXSDUOR          0x010c
#define NETC_PORT_PTCTMSDUR(n)      (0x0208 + (n) * 0x20)
#define NETC_PORT_MAXSDU_DISABLE    (1 << 24)
#define NETC_PSEUDO_PPMCR           0x0010
#define NETC_PSEUDO_TXPAD           (1 << 16)

#define NETC_MAC_CMD                0x0008
#define NETC_MAC_TX                 (1 << 0)
#define NETC_MAC_RX                 (1 << 1)
#define NETC_MAC_TXPAD              (1 << 15)
#define NETC_MAC_MAXFRM             0x0014
#define NETC_MAC_MINFRM             0x0018
#define NETC_MAC_IEVENT             0x0040
#define NETC_MAC_TXEMPTY            (1 << 5)
#define NETC_MAC_RXEMPTY            (1 << 6)
#define NETC_MAC_IFMODE             0x0300
#define NETC_MAC_IF_MASK            (7 | (1 << 4) | (1 << 6) | (3 << 13))
#define NETC_MAC_IF_MII             1
#define NETC_MAC_IF_RMII            3
#define NETC_MAC_IF_RGMII           4
#define NETC_MAC_IF_10M             (1 << 4)
#define NETC_MAC_IF_HALF            (1 << 6)
#define NETC_MAC_IF_SSP(n)          ((uint32_t)(n) << 13)

#define NETC_MDIO_CFG               0x00
#define NETC_MDIO_BUSY              (1u << 31)
#define NETC_MDIO_READ_ERROR        (1 << 1)
#define NETC_MDIO_ADDR_ERROR        (1 << 28)
#define NETC_MDIO_COMPLETE          (1 << 30)
#define NETC_MDIO_C45               (1 << 6)
#define NETC_MDIO_NEG               (1 << 23)
#define NETC_MDIO_DIV(n)            ((uint32_t)(n) << 7)
#define NETC_MDIO_HOLD(n)           ((uint32_t)(n) << 2)
#define NETC_MDIO_CTL               0x04
#define NETC_MDIO_DATA              0x08
#define NETC_MDIO_READ              (1 << 15)
#define NETC_MDIO_PHY(n)            ((uint32_t)(n) << 5)

#define NETC_BD_FINAL               (1u << 31)
#define NETC_TX_BD_EXTENDED         (1 << 30)
#define NETC_TX_BD_INTERRUPT        (1 << 29)
#define NETC_TX_BD_WRITTEN          (1 << 26)
#define NETC_TX_BD_ERROR_MASK       (0x1ff << 16)
#define NETC_RX_BD_ERROR_MASK       (0xff << 16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A TX packet uses two hardware BDs; the extension has no offloads enabled.
 * RX uses the hardware's extended 32-byte format. Each independently owned
 * entry occupies a complete Cortex-M7/M33 cache line.
 */

struct imxrt_netc_txdesc_s
{
  uint32_t address;
  uint32_t address_hi;
  uint16_t length;
  uint16_t frame_length;
  uint32_t flags;
  uint32_t timestamp;
  uint32_t vlan;
  uint32_t reserved;
  uint32_t extension;
};

struct imxrt_netc_rxdesc_s
{
  uint32_t address;
  uint32_t address_hi;
  uint32_t length;
  uint32_t flags;
  uint32_t timestamp;
  uint32_t reserved[3];
};

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_NETC_H */
