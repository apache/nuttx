/****************************************************************************
 * drivers/pci/pcie_dw.h
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

#ifndef _PCIE_DW_H
#define _PCIE_DW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_epc.h>
#include <nuttx/pci/pci_epf.h>
#include <nuttx/pci/pcie_dw.h>
#include <nuttx/spinlock.h>
#include <nuttx/list.h>
#include <nuttx/nuttx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __bf_shf(x)                               (ffsll(x) - 1)
#define FIELD_PREP(_mask, _val)                   \
    (((_val) << __bf_shf(_mask)) & (_mask))
#define FIELD_GET(_mask, _reg)                    \
    (((_reg) & (_mask)) >> __bf_shf(_mask))

/* DWC PCIe IP-core versions (native support since v4.70a) */

#define DW_PCIE_VER_365A                          0x3336352a
#define DW_PCIE_VER_460A                          0x3436302a
#define DW_PCIE_VER_470A                          0x3437302a
#define DW_PCIE_VER_480A                          0x3438302a
#define DW_PCIE_VER_490A                          0x3439302a
#define DW_PCIE_VER_520A                          0x3532302a
#define DW_PCIE_VER_540A                          0x3534302a

#define __dw_pcie_ver_cmp(_pci, _ver, _op)        \
    ((_pci)->version _op DW_PCIE_VER_ ## _ver)

#define dw_pcie_ver_is(_pci, _ver)                \
    __dw_pcie_ver_cmp(_pci, _ver, ==)

#define dw_pcie_ver_is_ge(_pci, _ver)             \
    __dw_pcie_ver_cmp(_pci, _ver, >=)

#define dw_pcie_ver_type_is(_pci, _ver, _type)    \
  (__dw_pcie_ver_cmp(_pci, _ver, ==) && \
   __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, ==))

#define dw_pcie_ver_type_is_ge(_pci, _ver, _type) \
  (__dw_pcie_ver_cmp(_pci, _ver, ==) && \
   __dw_pcie_ver_cmp(_pci, TYPE_ ## _type, >=))

/* DWC PCIe controller capabilities */

#define DW_PCIE_CAP_REQ_RES                       0
#define DW_PCIE_CAP_IATU_UNROLL                   1
#define DW_PCIE_CAP_CDM_CHECK                     2

#define dw_pcie_cap_is(_pci, _cap) \
  test_bit(DW_PCIE_CAP_ ## _cap, &(_pci)->caps)

#define dw_pcie_cap_set(_pci, _cap) \
  set_bit(DW_PCIE_CAP_ ## _cap, &(_pci)->caps)

/* Parameters for the waiting for link up routine */

#define DW_PCIE_LINK_WAIT_MAX_RETRIES             10
#define DW_PCIE_LINK_WAIT_SLEEP_MS                90

/* Parameters for the waiting for iATU enabled routine */

#define DW_PCIE_LINK_WAIT_MAX_IATU_RETRIES        5
#define DW_PCIE_LINK_WAIT_IATU                    9

/* Synopsys-specific PCIe configuration registers */

#define DW_PCIE_PORT_FORCE                        0x708
#define DW_PCIE_PORT_FORCE_DO_DESKEW_FOR_SRIS     BIT(23)

#define DW_PCIE_PORT_AFR                          0x70C
#define DW_PCIE_PORT_AFR_N_FTS_MASK               GENMASK(15, 8)
#define DW_PCIE_PORT_AFR_N_FTS(n)                 FIELD_PREP(DW_PCIE_PORT_AFR_N_FTS_MASK, n)
#define DW_PCIE_PORT_AFR_CC_N_FTS_MASK            GENMASK(23, 16)
#define DW_PCIE_PORT_AFR_CC_N_FTS(n)              FIELD_PREP(DW_PCIE_PORT_AFR_CC_N_FTS_MASK, n)
#define DW_PCIE_PORT_AFR_ENTER_ASPM               BIT(30)
#define DW_PCIE_PORT_AFR_L0S_ENTRANCE_LAT_SHIFT   24
#define DW_PCIE_PORT_AFR_L0S_ENTRANCE_LAT_MASK    GENMASK(26, 24)
#define DW_PCIE_PORT_AFR_L1_ENTRANCE_LAT_SHIFT    27
#define DW_PCIE_PORT_AFR_L1_ENTRANCE_LAT_MASK     GENMASK(29, 27)

#define DW_PCIE_PORT_LINK_CONTROL                 0x710
#define DW_PCIE_PORT_LINK_DLL_LINK_EN             BIT(5)
#define DW_PCIE_PORT_LINK_FAST_LINK_MODE          BIT(7)
#define DW_PCIE_PORT_LINK_MODE_MASK               GENMASK(21, 16)
#define DW_PCIE_PORT_LINK_MODE(n)                 FIELD_PREP(DW_PCIE_PORT_LINK_MODE_MASK, n)
#define DW_PCIE_PORT_LINK_MODE_1_LANES            DW_PCIE_PORT_LINK_MODE(0x1)
#define DW_PCIE_PORT_LINK_MODE_2_LANES            DW_PCIE_PORT_LINK_MODE(0x3)
#define DW_PCIE_PORT_LINK_MODE_4_LANES            DW_PCIE_PORT_LINK_MODE(0x7)
#define DW_PCIE_PORT_LINK_MODE_8_LANES            DW_PCIE_PORT_LINK_MODE(0xf)

#define DW_PCIE_PORT_LANE_SKEW                    0x714
#define DW_PCIE_PORT_LANE_SKEW_INSERT_MASK        GENMASK(23, 0)

#define DW_PCIE_PORT_DEBUG0                       0x728
#define DW_PCIE_PORT_LOGIC_LTSSM_STATE_MASK       0x1f
#define DW_PCIE_PORT_LOGIC_LTSSM_STATE_L0         0x11
#define DW_PCIE_PORT_DEBUG1                       0x72C
#define DW_PCIE_PORT_DEBUG1_LINK_UP               BIT(4)
#define DW_PCIE_PORT_DEBUG1_LINK_IN_TRAINING      BIT(29)

#define DW_PCIE_LINK_WIDTH_SPEED_CONTROL          0x80C
#define DW_PCIE_PORT_LOGIC_N_FTS_MASK             GENMASK(7, 0)
#define DW_PCIE_PORT_LOGIC_SPEED_CHANGE           BIT(17)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH_MASK        GENMASK(12, 8)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH(n)          FIELD_PREP(DW_PCIE_PORT_LOGIC_LINK_WIDTH_MASK, n)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH_1_LANES     DW_PCIE_PORT_LOGIC_LINK_WIDTH(0x1)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH_2_LANES     DW_PCIE_PORT_LOGIC_LINK_WIDTH(0x2)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH_4_LANES     DW_PCIE_PORT_LOGIC_LINK_WIDTH(0x4)
#define DW_PCIE_PORT_LOGIC_LINK_WIDTH_8_LANES     DW_PCIE_PORT_LOGIC_LINK_WIDTH(0x8)

#define DW_PCIE_MSI_ADDR_LO                       0x820
#define DW_PCIE_MSI_ADDR_HI                       0x824
#define DW_PCIE_MSI_INTR0_ENABLE                  0x828
#define DW_PCIE_MSI_INTR0_MASK                    0x82C
#define DW_PCIE_MSI_INTR0_STATUS                  0x830

#define DW_PCIE_PORT_MULTI_LANE_CTRL              0x8C0
#define DW_PCIE_PORT_MLTI_UPCFG_SUPPORT           BIT(7)

#define DW_PCIE_VERSION_NUMBER                    0x8F8
#define DW_PCIE_VERSION_TYPE                      0x8FC

/* iATU inbound and outbound windows CSRs. Before the IP-core v4.80a each
 * iATU region CSRs had been indirectly accessible by means of the dedicated
 * viewport selector. The iATU/eDMA CSRs space was re-designed in DWC PCIe
 * v4.80a in a way so the viewport was unrolled into the directly accessible
 * iATU/eDMA CSRs space.
 */

#define DW_PCIE_ATU_VIEWPORT                      0x900
#define DW_PCIE_ATU_REGION_DIR_IB                 BIT(31)
#define DW_PCIE_ATU_REGION_DIR_OB                 0
#define DW_PCIE_ATU_VIEWPORT_BASE                 0x904
#define DW_PCIE_ATU_UNROLL_BASE(dir, index) \
  (((index) << 9) | ((dir == DW_PCIE_ATU_REGION_DIR_IB) ? BIT(8) : 0))
#define DW_PCIE_ATU_VIEWPORT_SIZE                 0x2C
#define DW_PCIE_ATU_REGION_CTRL1                  0x000
#define DW_PCIE_ATU_INCREASE_REGION_SIZE          BIT(13)
#define DW_PCIE_ATU_TYPE_MEM                      0x0
#define DW_PCIE_ATU_TYPE_IO                       0x2
#define DW_PCIE_ATU_TYPE_CFG0                     0x4
#define DW_PCIE_ATU_TYPE_CFG1                     0x5
#define DW_PCIE_ATU_TYPE_MSG                      0x10
#define DW_PCIE_ATU_TD                            BIT(8)
#define DW_PCIE_ATU_FUNC_NUM(pf)                  ((pf) << 20)
#define DW_PCIE_ATU_REGION_CTRL2                  0x004
#define DW_PCIE_ATU_ENABLE                        BIT(31)
#define DW_PCIE_ATU_BAR_MODE_ENABLE               BIT(30)
#define DW_PCIE_ATU_INHIBIT_PAYLOAD               BIT(22)
#define DW_PCIE_ATU_FUNC_NUM_MATCH_EN             BIT(19)
#define DW_PCIE_ATU_LOWER_BASE                    0x008
#define DW_PCIE_ATU_UPPER_BASE                    0x00C
#define DW_PCIE_ATU_LIMIT                         0x010
#define DW_PCIE_ATU_LOWER_TARGET                  0x014
#define DW_PCIE_ATU_BUS(x)                        FIELD_PREP(GENMASK(31, 24), x)
#define DW_PCIE_ATU_DEV(x)                        FIELD_PREP(GENMASK(23, 19), x)
#define DW_PCIE_ATU_FUNC(x)                       FIELD_PREP(GENMASK(18, 16), x)
#define DW_PCIE_ATU_UPPER_TARGET                  0x018
#define DW_PCIE_ATU_UPPER_LIMIT                   0x020

#define DW_PCIE_MISC_CONTROL_1_OFF                0x8BC
#define DW_PCIE_DBI_RO_WR_EN                      BIT(0)

#define DW_PCIE_MSIX_DOORBELL                     0x948
#define DW_PCIE_MSIX_DOORBELL_PF_SHIFT            24

#define DW_PCIE_DMA_VIEWPORT_BASE                 0x970
#define DW_PCIE_DMA_UNROLL_BASE                   0x80000
#define DW_PCIE_DMA_CTRL                          0x008
#define DW_PCIE_DMA_NUM_WR_CHAN                   GENMASK(3, 0)
#define DW_PCIE_DMA_NUM_RD_CHAN                   GENMASK(19, 16)

#define DW_PCIE_PL_CHK_REG_CONTROL_STATUS         0xB20
#define DW_PCIE_PL_CHK_REG_CHK_REG_START          BIT(0)
#define DW_PCIE_PL_CHK_REG_CHK_REG_CONTINUOUS     BIT(1)
#define DW_PCIE_PL_CHK_REG_CHK_REG_COMPARISON_ERR BIT(16)
#define DW_PCIE_PL_CHK_REG_CHK_REG_LOGIC_ERROR    BIT(17)
#define DW_PCIE_PL_CHK_REG_CHK_REG_COMPLETE       BIT(18)

#define DW_PCIE_PL_CHK_REG_ERR_ADDR               0xB28

/* 16.0 GT/s (Gen 4) lane margining register definitions
 */

#define DW_PCIE_GEN4_LANE_MARGINING_1_OFF         0xB80
#define DW_PCIE_MARGINING_MAX_VOLTAGE_OFFSET      GENMASK(29, 24)
#define DW_PCIE_MARGINING_NUM_VOLTAGE_STEPS       GENMASK(22, 16)
#define DW_PCIE_MARGINING_MAX_TIMING_OFFSET       GENMASK(13, 8)
#define DW_PCIE_MARGINING_NUM_TIMING_STEPS        GENMASK(5, 0)

#define DW_PCIE_GEN4_LANE_MARGINING_2_OFF         0xB84
#define DW_PCIE_MARGINING_IND_ERROR_SAMPLER       BIT(28)
#define DW_PCIE_MARGINING_SAMPLE_REPORTING_METHOD BIT(27)
#define DW_PCIE_MARGINING_IND_LEFT_RIGHT_TIMING   BIT(26)
#define DW_PCIE_MARGINING_IND_UP_DOWN_VOLTAGE     BIT(25)
#define DW_PCIE_MARGINING_VOLTAGE_SUPPORTED       BIT(24)
#define DW_PCIE_MARGINING_MAXLANES                GENMASK(20, 16)
#define DW_PCIE_MARGINING_SAMPLE_RATE_TIMING      GENMASK(13, 8)
#define DW_PCIE_MARGINING_SAMPLE_RATE_VOLTAGE     GENMASK(5, 0)

/* iATU Unroll-specific register definitions
 * From 4.80 core version the address translation will be made by unroll
 */

#define DW_PCIE_ATU_UNR_REGION_CTRL1              0x00
#define DW_PCIE_ATU_UNR_REGION_CTRL2              0x04
#define DW_PCIE_ATU_UNR_LOWER_BASE                0x08
#define DW_PCIE_ATU_UNR_UPPER_BASE                0x0C
#define DW_PCIE_ATU_UNR_LOWER_LIMIT               0x10
#define DW_PCIE_ATU_UNR_LOWER_TARGET              0x14
#define DW_PCIE_ATU_UNR_UPPER_TARGET              0x18
#define DW_PCIE_ATU_UNR_UPPER_LIMIT               0x20

/* RAS-DES register definitions
 */
#define DW_PCIE_RAS_DES_EVENT_COUNTER_CONTROL     0x8
#define DW_PCIE_EVENT_COUNTER_ALL_CLEAR           0x3
#define DW_PCIE_EVENT_COUNTER_ENABLE_ALL          0x7
#define DW_PCIE_EVENT_COUNTER_ENABLE_SHIFT        2
#define DW_PCIE_EVENT_COUNTER_EVENT_SEL_MASK      GENMASK(7, 0)
#define DW_PCIE_EVENT_COUNTER_EVENT_SEL_SHIFT     16
#define DW_PCIE_EVENT_COUNTER_EVENT_Tx_L0S        0x2
#define DW_PCIE_EVENT_COUNTER_EVENT_Rx_L0S        0x3
#define DW_PCIE_EVENT_COUNTER_EVENT_L1            0x5
#define DW_PCIE_EVENT_COUNTER_EVENT_L1_1          0x7
#define DW_PCIE_EVENT_COUNTER_EVENT_L1_2          0x8
#define DW_PCIE_EVENT_COUNTER_GROUP_SEL_SHIFT     24
#define DW_PCIE_EVENT_COUNTER_GROUP_5             0x5

#define DW_PCIE_RAS_DES_EVENT_COUNTER_DATA        0xc

/* The default address offset between dbi_base and atu_base. Root
 * controller drivers are not required to initialize atu_base
 * if the offset matches this default; the driver core
 * automatically derives atu_base From dbi_base using this offset,
 * if atu_base not set.
 */

#define DW_PCIE_DEFAULT_DBI_ATU_OFFSET            (0x3 << 20)
#define DW_PCIE_DEFAULT_DBI_DMA_OFFSET            DW_PCIE_DMA_UNROLL_BASE

/* Maximum number of inbound/outbound iATUs */

#define DW_PCIE_MAX_IATU_IN                       256
#define DW_PCIE_MAX_IATU_OUT                      256

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void dw_pcie_version_detect(FAR struct dw_pcie_s *pci);
uint32_t dw_pcie_read_dbi(FAR struct dw_pcie_s *pci, uint32_t reg,
                          size_t size);
void dw_pcie_write_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, size_t size,
                       uint32_t val);
void dw_pcie_write_dbi2(FAR struct dw_pcie_s *pci, uint32_t reg, size_t size,
                        uint32_t val);
void
dw_pcie_writel_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint32_t val);
uint32_t dw_pcie_readl_dbi(FAR struct dw_pcie_s *pci, uint32_t reg);
void
dw_pcie_writew_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint16_t val);
uint16_t dw_pcie_readw_dbi(FAR struct dw_pcie_s *pci, uint32_t reg);
void
dw_pcie_writeb_dbi(FAR struct dw_pcie_s *pci, uint32_t reg, uint8_t val);
uint8_t dw_pcie_readb_dbi(FAR struct dw_pcie_s *pci, uint32_t reg);
void
dw_pcie_writel_dbi2(FAR struct dw_pcie_s *pci, uint32_t reg, uint32_t val);
uint32_t dw_pcie_ep_get_dbi_offset(FAR struct dw_pcie_ep_s *ep,
                                   uint8_t funcno);
uint32_t dw_pcie_ep_read_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                             uint32_t reg, size_t size);
void dw_pcie_ep_write_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                          uint32_t reg, size_t size, uint32_t val);
void dw_pcie_ep_writel_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint32_t reg, uint32_t val);
uint32_t dw_pcie_ep_readl_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                              uint32_t reg);
void dw_pcie_ep_writew_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint32_t reg, uint16_t val);
uint16_t dw_pcie_ep_readw_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                              uint32_t reg);
void dw_pcie_ep_writeb_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                           uint32_t reg, uint8_t val);
uint8_t dw_pcie_ep_readb_dbi(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                             uint32_t reg);
uint32_t
dw_pcie_ep_get_dbi2_offset(FAR struct dw_pcie_ep_s *ep, uint8_t funcno);
void
dw_pcie_ep_write_dbi2(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                      uint32_t reg, size_t size, uint32_t val);
void dw_pcie_ep_writel_dbi2(FAR struct dw_pcie_ep_s *ep, uint8_t funcno,
                            uint32_t reg, uint32_t val);
void dw_pcie_dbi_ro_wr_en(FAR struct dw_pcie_s *pci);
void dw_pcie_dbi_ro_wr_dis(FAR struct dw_pcie_s *pci);
void dw_pcie_ep_linkup(FAR struct dw_pcie_ep_s *ep);
void dw_pcie_ep_linkdown(FAR struct dw_pcie_ep_s *ep);
int dw_pcie_prog_outbound_atu(FAR struct dw_pcie_s *pci,
                              const FAR struct dw_pcie_ob_atu_cfg_s *atu);
int
dw_pcie_prog_inbound_atu(FAR struct dw_pcie_s *pci, int index, int type,
                         uint64_t cpu_addr, uint64_t pci_addr,
                         uint64_t size);
int
dw_pcie_prog_ep_inbound_atu(FAR struct dw_pcie_s *pci, uint8_t funcno,
                            int index, int type, uint64_t cpu_addr,
                            uint8_t bar);
void dw_pcie_disable_atu(FAR struct dw_pcie_s *pci, uint32_t dir, int index);
void dw_pcie_setup(FAR struct dw_pcie_s *pci);
void dw_pcie_iatu_detect(FAR struct dw_pcie_s *pci);

#endif /* _PCIE_DW_H */
