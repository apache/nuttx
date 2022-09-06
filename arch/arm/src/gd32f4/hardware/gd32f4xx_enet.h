/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_enet.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_ENET_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_ENET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if GD32_NETHERNET > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* MAC Registers */

#define GD32_ENET_MAC_CFG_OFFSET             0x0000 /* MAC configuration register offset */
#define GD32_ENET_MAC_FRMF_OFFSET            0x0004 /* MAC frame filter register offset */
#define GD32_ENET_MAC_HLH_OFFSET             0x0008 /* MAC hash list high register offset */
#define GD32_ENET_MAC_HLL_OFFSET             0x000C /* MAC hash list low register offset */
#define GD32_ENET_MAC_PHY_CTL_OFFSET         0x0010 /* MAC PHY control register offset */
#define GD32_ENET_MAC_PHY_DATA_OFFSET        0x0014 /* MAC PHY data register offset */
#define GD32_ENET_MAC_FCTL_OFFSET            0x0018 /* MAC flow control register offset */
#define GD32_ENET_MAC_VLT_OFFSET             0x001C /* MAC VLAN tag register offset */
#define GD32_ENET_MAC_RWFF_OFFSET            0x0028 /* MAC remote wakeup frame filter register offset */
#define GD32_ENET_MAC_WUM_OFFSET             0x002C /* MAC wakeup managenment register offset */

#define GD32_ENET_MAC_DBG_OFFSET             0x0034 /* MAC debug register offset */
#define GD32_ENET_MAC_INTF_OFFSET            0x0038 /* MAC interrupt flag register offset */
#define GD32_ENET_MAC_INTMSK_OFFSET          0x003C /* MAC interrupt mask register offset */
#define GD32_ENET_MAC_ADDR0H_OFFSET          0x0040 /* MAC address 0 high register offset */
#define GD32_ENET_MAC_ADDR0L_OFFSET          0x0044 /* MAC address 0 low register offset */
#define GD32_ENET_MAC_ADDR1H_OFFSET          0x0048 /* MAC address 1 high register offset */
#define GD32_ENET_MAC_ADDR1L_OFFSET          0x004C /* MAC address 1 low register offset */
#define GD32_ENET_MAC_ADDR2H_OFFSET          0x0050 /* MAC address 2 high register offset */
#define GD32_ENET_MAC_ADDR2L_OFFSET          0x0054 /* MAC address 2 low register offset */
#define GD32_ENET_MAC_ADDR3H_OFFSET          0x0058 /* MAC address 3 high register offset */
#define GD32_ENET_MAC_ADDR3L_OFFSET          0x005C /* MAC address 3 low register offset */

#define GD32_ENET_MAC_FCTH_OFFSET            0x1080 /* MAC flow control threshold register offset */

/* MSC Registers */

#define GD32_ENET_MSC_CTL_OFFSET             0x0100 /* MSC control register offset */
#define GD32_ENET_MSC_RINTF_OFFSET           0x0104 /* MSC receive interrupt flag register offset */
#define GD32_ENET_MSC_TINTF_OFFSET           0x0108 /* MSC transmit interrupt flag register offset */
#define GD32_ENET_MSC_RINTMSK_OFFSET         0x010C /* MSC receive interrupt mask register offset */
#define GD32_ENET_MSC_TINTMSK_OFFSET         0x0110 /* MSC transmit interrupt mask register offset */

#define GD32_ENET_MSC_SCCNT_OFFSET           0x014C /* MSC transmitted good frames after a single collision counter register offset */
#define GD32_ENET_MSC_MSCCNT_OFFSET          0x0150 /* MSC transmitted good frames after more than a signle collision counter register offset */

#define GD32_ENET_MSC_TGFCNT_OFFSET          0x0168 /* MSC transmitted good frames counter register offset */

#define GD32_ENET_MSC_RFCECNT_OFFSET         0x0194 /* MSC received frames with CRC error counter register offset */
#define GD32_ENET_MSC_RFAECNT_OFFSET         0x0198 /* MSC received frames with alignment error counter offset */

#define GD32_ENET_MSC_RGUFCNT_OFFSET         0x01C4 /* MSC received good unicast frames counter register offset */

/* PTP Registers */

#define GD32_ENET_PTP_TSCTL_OFFSET           0x0700 /* PTP time stamp control register offset */
#define GD32_ENET_PTP_SSINC_OFFSET           0x0704 /* PTP subsecond increment register offset */
#define GD32_ENET_PTP_TSH_OFFSET             0x0708 /* PTP time stamp high register offset */
#define GD32_ENET_PTP_TSL_OFFSET             0x070C /* PTP time stamp low register offset */
#define GD32_ENET_PTP_TSUH_OFFSET            0x0710 /* PTP time stamp update high register offset */
#define GD32_ENET_PTP_TSUL_OFFSET            0x0714 /* PTP time stamp update low register offset */
#define GD32_ENET_PTP_TSADDEND_OFFSET        0x0718 /* PTP time stamp addend register offset */
#define GD32_ENET_PTP_ETH_OFFSET             0x071C /* PTP expected time high register offset */
#define GD32_ENET_PTP_ETL_OFFSET             0x0720 /* PTP expected time low register offset */
#define GD32_ENET_PTP_TSF_OFFSET             0x0728 /* PTP time stamp flag register offset */
#define GD32_ENET_PTP_PPSCTL_OFFSET          0x072C /* PTP PPS control register offset */

/* DMA Registers */

#define GD32_ENET_DMA_BCTL_OFFSET            0x1000 /* DMA bus control register offset */
#define GD32_ENET_DMA_TPEN_OFFSET            0x1004 /* DMA transmit poll enable register offset */
#define GD32_ENET_DMA_RPEN_OFFSET            0x1008 /* DMA receive poll enable register offset */
#define GD32_ENET_DMA_RDTADDR_OFFSET         0x100C /* DMA receive descriptor table address register offset */
#define GD32_ENET_DMA_TDTADDR_OFFSET         0x1010 /* DMA transmit descriptor table address register offset */
#define GD32_ENET_DMA_STAT_OFFSET            0x1014 /* DMA status register offset */
#define GD32_ENET_DMA_CTL_OFFSET             0x1018 /* DMA control register offset */
#define GD32_ENET_DMA_INTEN_OFFSET           0x101C /* DMA interrupt enable register offset */
#define GD32_ENET_DMA_MFBOCNT_OFFSET         0x1020 /* DMA missed frame and buffer overflow counter register offset */
#define GD32_ENET_DMA_RSWDC_OFFSET           0x1024 /* DMA receive state watchdog counter register offset */

#define GD32_ENET_DMA_CTDADDR_OFFSET         0x1048 /* DMA current transmit descriptor address register offset */
#define GD32_ENET_DMA_CRDADDR_OFFSET         0x104C /* DMA current receive descriptor address register offset */
#define GD32_ENET_DMA_CTBADDR_OFFSET         0x1050 /* DMA current transmit buffer address register offset */
#define GD32_ENET_DMA_CRBADDR_OFFSET         0x1054 /* DMA current receive buffer address register offset */

/* Register Base Addresses **************************************************/

/* MAC Registers */

#define GD32_ENET_MAC_CFG                    (GD32_ENET_BASE + GD32_ENET_MAC_CFG_OFFSET)
#define GD32_ENET_MAC_FRMF                   (GD32_ENET_BASE + GD32_ENET_MAC_FRMF_OFFSET)
#define GD32_ENET_MAC_HLH                    (GD32_ENET_BASE + GD32_ENET_MAC_HLH_OFFSET)
#define GD32_ENET_MAC_HLL                    (GD32_ENET_BASE + GD32_ENET_MAC_HLL_OFFSET)
#define GD32_ENET_MAC_PHY_CTL                (GD32_ENET_BASE + GD32_ENET_MAC_PHY_CTL_OFFSET)
#define GD32_ENET_MAC_PHY_DATA               (GD32_ENET_BASE + GD32_ENET_MAC_PHY_DATA_OFFSET)
#define GD32_ENET_MAC_FCTL                   (GD32_ENET_BASE + GD32_ENET_MAC_FCTL_OFFSET)
#define GD32_ENET_MAC_VLT                    (GD32_ENET_BASE + GD32_ENET_MAC_VLT_OFFSET)
#define GD32_ENET_MAC_RWFF                   (GD32_ENET_BASE + GD32_ENET_MAC_RWFF_OFFSET)
#define GD32_ENET_MAC_WUM                    (GD32_ENET_BASE + GD32_ENET_MAC_WUM_OFFSET)
#define GD32_ENET_MAC_DBG                    (GD32_ENET_BASE + GD32_ENET_MAC_DBG_OFFSET)
#define GD32_ENET_MAC_INTF                   (GD32_ENET_BASE + GD32_ENET_MAC_INTF_OFFSET)
#define GD32_ENET_MAC_INTMSK                 (GD32_ENET_BASE + GD32_ENET_MAC_INTMSK_OFFSET)
#define GD32_ENET_MAC_ADDR0H                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR0H_OFFSET)
#define GD32_ENET_MAC_ADDR0L                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR0L_OFFSET)
#define GD32_ENET_MAC_ADDR1H                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR1H_OFFSET)
#define GD32_ENET_MAC_ADDR1L                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR1L_OFFSET)
#define GD32_ENET_MAC_ADDT2H                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDT2H_OFFSET)
#define GD32_ENET_MAC_ADDR2L                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR2L_OFFSET)
#define GD32_ENET_MAC_ADDR3H                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR3H_OFFSET)
#define GD32_ENET_MAC_ADDR3L                 (GD32_ENET_BASE + GD32_ENET_MAC_ADDR3L_OFFSET)
#define GD32_ENET_MAC_FCTH                   (GD32_ENET_BASE + GD32_ENET_MAC_FCTH_OFFSET)

/* MSC Registers */

#define GD32_ENET_MSC_CTL                    (GD32_ENET_BASE + GD32_ENET_MSC_CTL_OFFSET)
#define GD32_ENET_MSC_RINTF                  (GD32_ENET_BASE + GD32_ENET_MSC_RINTF_OFFSET)
#define GD32_ENET_MSC_TINTF                  (GD32_ENET_BASE + GD32_ENET_MSC_TINTF_OFFSET)
#define GD32_ENET_MSC_RINTMSK                (GD32_ENET_BASE + GD32_ENET_MSC_RINTMSK_OFFSET)
#define GD32_ENET_MSC_TINTMSK                (GD32_ENET_BASE + GD32_ENET_MSC_TINTMSK_OFFSET)
#define GD32_ENET_MSC_SCCNT                  (GD32_ENET_BASE + GD32_ENET_MSC_SCCNT_OFFSET)
#define GD32_ENET_MSC_MSCCNT                 (GD32_ENET_BASE + GD32_ENET_MSC_MSCCNT_OFFSET)
#define GD32_ENET_MSC_TGFCNT                 (GD32_ENET_BASE + GD32_ENET_MSC_TGFCNT_OFFSET)
#define GD32_ENET_MSC_RFCECNT                (GD32_ENET_BASE + GD32_ENET_MSC_RFCECNT_OFFSET)
#define GD32_ENET_MSC_RFAECNT                (GD32_ENET_BASE + GD32_ENET_MSC_RFAECNT_OFFSET)
#define GD32_ENET_MSC_RGUFCNT                (GD32_ENET_BASE + GD32_ENET_MSC_RGUFCNT_OFFSET)

/* PTP Registers */

#define GD32_ENET_PTP_TSCTL                  (GD32_ENET_BASE + GD32_ENET_PTP_TSCTL_OFFSET)
#define GD32_ENET_PTP_SSINC                  (GD32_ENET_BASE + GD32_ENET_PTP_SSINC_OFFSET)
#define GD32_ENET_PTP_TSH                    (GD32_ENET_BASE + GD32_ENET_PTP_TSH_OFFSET)
#define GD32_ENET_PTP_TSL                    (GD32_ENET_BASE + GD32_ENET_PTP_TSL_OFFSET)
#define GD32_ENET_PTP_TSUH                   (GD32_ENET_BASE + GD32_ENET_PTP_TSUH_OFFSET)
#define GD32_ENET_PTP_TSUL                   (GD32_ENET_BASE + GD32_ENET_PTP_TSUL_OFFSET)
#define GD32_ENET_PTP_TSADDEND               (GD32_ENET_BASE + GD32_ENET_PTP_TSADDEND_OFFSET)
#define GD32_ENET_PTP_ETH                    (GD32_ENET_BASE + GD32_ENET_PTP_ETH_OFFSET)
#define GD32_ENET_PTP_ETL                    (GD32_ENET_BASE + GD32_ENET_PTP_ETL_OFFSET)
#define GD32_ENET_PTP_TSF                    (GD32_ENET_BASE + GD32_ENET_PTP_TSF_OFFSET)
#define GD32_ENET_PTP_PPSCTL                 (GD32_ENET_BASE + GD32_ENET_PTP_PPSCTL_OFFSET)

/* DMA Registers */

#define GD32_ENET_DMA_BCTL                   (GD32_ENET_BASE + GD32_ENET_DMA_BCTL_OFFSET)
#define GD32_ENET_DMA_TPEN                   (GD32_ENET_BASE + GD32_ENET_DMA_TPEN_OFFSET)
#define GD32_ENET_DMA_RPEN                   (GD32_ENET_BASE + GD32_ENET_DMA_RPEN_OFFSET)
#define GD32_ENET_DMA_RDTADDR                (GD32_ENET_BASE + GD32_ENET_DMA_RDTADDR_OFFSET)
#define GD32_ENET_DMA_TDTADDR                (GD32_ENET_BASE + GD32_ENET_DMA_TDTADDR_OFFSET)
#define GD32_ENET_DMA_STAT                   (GD32_ENET_BASE + GD32_ENET_DMA_STAT_OFFSET)
#define GD32_ENET_DMA_CTL                    (GD32_ENET_BASE + GD32_ENET_DMA_CTL_OFFSET)
#define GD32_ENET_DMA_INTEN                  (GD32_ENET_BASE + GD32_ENET_DMA_INTEN_OFFSET)
#define GD32_ENET_DMA_MFBOCNT                (GD32_ENET_BASE + GD32_ENET_DMA_MFBOCNT_OFFSET)
#define GD32_ENET_DMA_RSWDC                  (GD32_ENET_BASE + GD32_ENET_DMA_RSWDC_OFFSET)
#define GD32_ENET_DMA_CTDADDR                (GD32_ENET_BASE + GD32_ENET_DMA_CTDADDR_OFFSET)
#define GD32_ENET_DMA_CRDADDR                (GD32_ENET_BASE + GD32_ENET_DMA_CRDADDR_OFFSET)
#define GD32_ENET_DMA_CTBADDR                (GD32_ENET_BASE + GD32_ENET_DMA_CTBADDR_OFFSET)
#define GD32_ENET_DMA_CRBADDR                (GD32_ENET_BASE + GD32_ENET_DMA_CRBADDR_OFFSET)

/* Register Bit-Field Definitions */

/* MAC Registers */

/* MAC configuration register (GD32_ENET_MAC_CFG) */

#define ENET_MAC_CFG_REN                (1 << 2)                           /* Bit 2:  receiver enable */
#define ENET_MAC_CFG_TEN                (1 << 3)                           /* Bit 3:  transmitter enable */
#define ENET_MAC_CFG_DFC                (1 << 4)                           /* Bit 4:  defferal check */

#define ENET_MAC_CFG_BOL_SHIFT          (5)                                /* Bits 5-6: back-off limit */
#define ENET_MAC_CFG_BOL_MASK           (3 << ENET_MAC_CFG_BOL_SHIFT)
#define ENET_BACKOFFLIMIT_10            (0 << ENET_MAC_CFG_BOL_SHIFT)      /* 00: k = min (n, 10) */
#define ENET_BACKOFFLIMIT_8             (1 << ENET_MAC_CFG_BOL_SHIFT)      /* 01: k = min (n, 8) */
#define ENET_BACKOFFLIMIT_4             (2 << ENET_MAC_CFG_BOL_SHIFT)      /* 10: k = min (n, 4) */
#define ENET_BACKOFFLIMIT_1             (3 << ENET_MAC_CFG_BOL_SHIFT)      /* 11: k = min (n, 1) */

#define ENET_MAC_CFG_APCD               (1 << 7)                           /* Bit 7:  automatic pad/CRC drop */
#define ENET_MAC_CFG_RTD                (1 << 9)                           /* Bit 9:  retry disable */
#define ENET_MAC_CFG_IPFCO              (1 << 10)                          /* Bit 10: IP frame checksum offload */
#define ENET_MAC_CFG_DPM                (1 << 11)                          /* Bit 11: duplex mode */
#define ENET_MAC_CFG_LBM                (1 << 12)                          /* Bit 12: loopback mode */
#define ENET_MAC_CFG_ROD                (1 << 13)                          /* Bit 13: receive own disable */
#define ENET_MAC_CFG_SPD                (1 << 14)                          /* Bit 14: fast ethernet speed */
#define ENET_MAC_CFG_CSD                (1 << 16)                          /* Bit 16: carrier sense disable */

#define ENET_MAC_CFG_IGBS_SHIFT         (17)                               /* Bits 17-19: inter-frame gap bit selection */
#define ENET_MAC_CFG_IGBS_MASK          (7 << ENET_MAC_CFG_IGBS_SHIFT)
#define ENET_MAC_CFG_IGBS(n)            ((12 - ((n) >> 3)) << ENET_MAC_CFG_IGBS_SHIFT) /* n bit times, n=40, 48,...96 */

#define ENET_MAC_CFG_JBD                (1 << 22)                          /* Bit 22: jabber disable */
#define ENET_MAC_CFG_WDD                (1 << 23)                          /* Bit 23: watchdog disable */
#define ENET_MAC_CFG_TFCD               (1 << 25)                          /* Bit 25: type frame CRC dropping */

/* MAC frame filter register (GD32_ENET_MAC_FRMF) */

#define ENET_MAC_FRMF_PM                (1 << 0)                           /* Bit 0: promiscuous mode */
#define ENET_MAC_FRMF_HUF               (1 << 1)                           /* Bit 1: hash unicast filter */
#define ENET_MAC_FRMF_HMF               (1 << 2)                           /* Bit 2: hash multicast filter */
#define ENET_MAC_FRMF_DAIFLT            (1 << 3)                           /* Bit 3: destination address inverse filtering enable */
#define ENET_MAC_FRMF_MFD               (1 << 4)                           /* Bit 4: multicast filter disable */
#define ENET_MAC_FRMF_BFRMD             (1 << 5)                           /* Bit 5: broadcast frame disable */

#define ENET_MAC_FRMF_PCFRM_SHIFT       (6)                                /* Bits 6-7: pass control frames */
#define ENET_MAC_FRMF_PCFRM_MASK        (3 << ENET_MAC_FRMF_PCFRM_SHIFT)
#define ENET_PCFRM_PREVENT_ALL          (0 << ENET_MAC_FRMF_PCFRM_SHIFT)   /* prevents all control frames */
#define ENET_PCFRM_PREVENT_PAUSEFRAME   (1 << ENET_MAC_FRMF_PCFRM_SHIFT)   /* prevents all except Pause control frames */
#define ENET_PCFRM_FORWARD_ALL          (2 << ENET_MAC_FRMF_PCFRM_SHIFT)   /* forwards all control frames */
#define ENET_PCFRM_FORWARD_FILTERED     (3 << ENET_MAC_FRMF_PCFRM_SHIFT)   /* forwards all that pass address filter */

#define ENET_MAC_FRMF_SAIFLT            (1 << 8)                           /* Bit 8: source address inverse filtering */
#define ENET_MAC_FRMF_SAFLT             (1 << 9)                           /* Bit 9: source address filter */
#define ENET_MAC_FRMF_HPFLT             (1 << 10)                          /* Bit 10: hash or perfect filter */
#define ENET_MAC_FRMF_FAR               (1 << 31)                          /* Bit 31: receive all frames */

/* MAC hash list high registers (GD32_ENET_MAC_HLH) */

/* MAC hash list low registers (GD32_ENET_MAC_HLL) */

/* MAC PHY control register (GD32_ENET_MAC_PHY_CTL) */

#define ENET_MAC_PHY_CTL_PB              (1 << 0)                          /* Bit 0: PHY busy */
#define ENET_MAC_PHY_CTL_PW              (1 << 1)                          /* Bit 1: PHY write */

#define ENET_MAC_PHY_CTL_CLR_SHIFT       (2)                               /* Bits 2-4: clock range */
#define ENET_MAC_PHY_CTL_CLR_MASK        (7 << ENET_MAC_PHY_CTL_CLR_SHIFT)
#define ENET_MDC_HCLK_DIV42              (0 << ENET_MAC_PHY_CTL_CLR_SHIFT) /* 000 60-100 MHz  HCLK/42 */
#define ENET_MDC_HCLK_DIV62              (1 << ENET_MAC_PHY_CTL_CLR_SHIFT) /* 001 100-150 MHz HCLK/62 */
#define ENET_MDC_HCLK_DIV16              (2 << ENET_MAC_PHY_CTL_CLR_SHIFT) /* 010 20-35 MHz   HCLK/16 */
#define ENET_MDC_HCLK_DIV26              (3 << ENET_MAC_PHY_CTL_CLR_SHIFT) /* 011 35-60 MHz   HCLK/26 */
#define ENET_MDC_HCLK_DIV102             (4 << ENET_MAC_PHY_CTL_CLR_SHIFT) /* 100 150-240 MHz HCLK/102 */

#define ENET_MAC_PHY_CTL_PR_SHIFT        (6)                               /* Bits 6-10: PHY register */
#define ENET_MAC_PHY_CTL_PR_MASK         (31 << ENET_MAC_PHY_CTL_PR_SHIFT)

#define ENET_MAC_PHY_CTL_PA_SHIFT        (11)                              /* Bits 11-15: PHY address */
#define ENET_MAC_PHY_CTL_PA_MASK         (31 << ENET_MAC_PHY_CTL_PA_SHIFT)

/* MAC PHY data register (GD32_ENET_MAC_PHY_DATA) */

#define ENET_MAC_PHY_DATA_PD_MASK        (0xFFFF)

/* MAC flow control register (GD32_ENET_MAC_FCTL) */

#define ENET_MAC_FCTL_FLCBBKPA           (1 << 0)                          /* Bit 0: flow control busy(in full duplex mode)/backpressure activate(in half duplex mode) */
#define ENET_MAC_FCTL_TFCEN              (1 << 1)                          /* Bit 1: transmit flow control enable */
#define ENET_MAC_FCTL_RFCEN              (1 << 2)                          /* Bit 2: receive flow control enable */
#define ENET_MAC_FCTL_UPFDT              (1 << 3)                          /* Bit 3: unicast pause frame detect */

#define ENET_MAC_FCTL_PLTS_SHIFT         (4)                               /* Bits 4-5: pause low threshold */
#define ENET_MAC_FCTL_PLTS_MASK          (3 << ENET_MAC_FCTL_PLTS_SHIFT)
#define ENET_PAUSETIME_MINUS4            (0 << ENET_MAC_FCTL_PLTS_SHIFT)   /* 00 pause time minus 4 slot times */
#define ENET_PAUSETIME_MINUS28           (1 << ENET_MAC_FCTL_PLTS_SHIFT)   /* 01 pause time minus 28 slot times */
#define ENET_PAUSETIME_MINUS144          (2 << ENET_MAC_FCTL_PLTS_SHIFT)   /* 10 pause time minus 144 slot times */
#define ENET_PAUSETIME_MINUS256          (3 << ENET_MAC_FCTL_PLTS_SHIFT)   /* 11 pause time minus 256 slot times */

#define ENET_MAC_FCTL_DZQP               (1 << 7)                          /* Bit 7: disable zero-quanta pause */

#define ENET_MAC_FCTL_PTM_SHIFT          (16)                              /* Bits 16-31: pause time */
#define ENET_MAC_FCTL_PTM_MASK           (0xFFFF << ENET_MAC_FCTL_PTM_SHIFT)

/* MAC VLAN tag register (GD32_ENET_MAC_VLT) */

#define ENET_MAC_VLT_VLTI_SHIFT          (0)                               /* Bits 0-15: VLAN tag identifier(for receive frames) */
#define ENET_MAC_VLT_VLTI_MASK           (0xFFFF << ENET_MAC_VLT_VLTI_SHIFT)

#define ENET_MAC_VLT_VLTC                (1 << 16)                         /* Bit 16: 12-bit VLAN tag comparison */

/* MAC remote wakeup frame filter register (GD32_ENET_MAC_RWFF) */

/* MAC wakeup management register (GD32_ENET_MAC_WUM) */

#define ENET_MAC_WUM_PWD                 (1 << 0)                          /* Bit 0: power down */
#define ENET_MAC_WUM_MPEN                (1 << 1)                          /* Bit 1: magic Packet enable */
#define ENET_MAC_WUM_WFEN                (1 << 2)                          /* Bit 2: wakeup frame enable */
#define ENET_MAC_WUM_MPKR                (1 << 5)                          /* Bit 5: magic packet received */
#define ENET_MAC_WUM_WUFR                (1 << 6)                          /* Bit 6: wakeup frame received */
#define ENET_MAC_WUM_GU                  (1 << 9)                          /* Bit 9: global unicast */
#define ENET_MAC_WUM_WUFFRPR             (1 << 31)                         /* Bit 31: wakeup frame filter register pointer reset */

/* MAC debug register (GD32_ENET_MAC_DBG) */

#define ENET_MAC_DBG_MRNI                (1 << 0)                          /* Bit 0: MAC receive state not idle */

#define ENET_MAC_DBG_RXAFS_SHIFT         (1)                               /* Bits 1-2: Rx asynchronous FIFO status */
#define ENET_MAC_DBG_RXAFS_MASK          (3 << ENET_MAC_DBG_RXAFS_SHIFT)

#define ENET_MAC_DBG_RXFW                (1 << 4)                          /* Bit 4: RxFIFO is writing */

#define ENET_MAC_DBG_RXFRS_SHIFT         (5)                               /* Bits 5-6: RxFIFO read controller status */
#define ENET_MAC_DBG_RXFRS_MASK          (3 << ENET_MAC_DBG_RXFRS_SHIFT)
#define ENET_MAC_DBG_RXFRS_IDLE          (0 << ENET_MAC_DBG_RXFRS_SHIFT)   /* 00: IDLE state */
#define ENET_MAC_DBG_RXFRS_RFRAME        (1 << ENET_MAC_DBG_RXFRS_SHIFT)   /* 01: reading frame data */
#define ENET_MAC_DBG_RXFRS_RSTATUS       (2 << ENET_MAC_DBG_RXFRS_SHIFT)   /* 10: reading frame status (or time-stamp) */
#define ENET_MAC_DBG_RXFRS_FLUSHING      (3 << ENET_MAC_DBG_RXFRS_SHIFT)   /* 11: flushing the frame data and status */

#define ENET_MAC_DBG_RXFS_SHIFT          (8)                               /* Bits 8-9: rxFIFO fill level */
#define ENET_MAC_DBG_RXFS_MASK           (3 << ENET_MAC_DBG_RXFS_SHIFT)
#define ENET_MAC_DBG_RXFS_EMPTY          (0 << ENET_MAC_DBG_RXFS_SHIFT)    /* 00: RxFIFO empty */
#define ENET_MAC_DBG_RXFS_DEACT          (1 << ENET_MAC_DBG_RXFS_SHIFT)    /* 01: RxFIFO fill-level below flow-control de-activate threshold */
#define ENET_MAC_DBG_RXFS_ACTIV          (2 << ENET_MAC_DBG_RXFS_SHIFT)    /* 10: RxFIFO fill-level above flow-control activate threshold */
#define ENET_MAC_DBG_RXFS_FULL           (3 << ENET_MAC_DBG_RXFS_SHIFT)    /* 11: RxFIFO full */

#define ENET_MAC_DBG_MTNI                (1 << 16)                         /* Bit 16: MAC transmit state not idle */

#define ENET_MAC_DBG_SOMT_SHIFT          (17)                              /* Bits 17-18: status of mac transmitter */
#define ENET_MAC_DBG_SOMT_MASK           (3 << ENET_MAC_DBG_SOMT_SHIFT)
#define ENET_MAC_DBG_SOMT_IDLE           (0 << ENET_MAC_DBG_SOMT_SHIFT)    /* 00: idle */
#define ENET_MAC_DBG_SOMT_WAITING        (1 << ENET_MAC_DBG_SOMT_SHIFT)    /* 01: waiting for Status of previous frame or IFG/backoff period to be over */
#define ENET_MAC_DBG_SOMT_PAUSE          (2 << ENET_MAC_DBG_SOMT_SHIFT)    /* 10: generating and transmitting a Pause control frame */
#define ENET_MAC_DBG_SOMT_FRAME          (3 << ENET_MAC_DBG_SOMT_SHIFT)    /* 11: transferring input frame for transmission */

#define ENET_MAC_DBG_PCS                 (1 << 19)                         /* Bit 19: pause condition status */

#define ENET_MAC_DBG_TXFRS_SHIFT         (20)                              /* Bits 20-21: TxFIFO read operation status */
#define ENET_MAC_DBG_TXFRS_MASK          (3 << ENET_MAC_DBG_TXFRS_SHIFT)
#define ENET_MAC_DBG_TXFRS_IDLE          (0 << ENET_MAC_DBG_TXFRS_SHIFT)   /* 00: idle state */
#define ENET_MAC_DBG_TXFRS_READ          (1 << ENET_MAC_DBG_TXFRS_SHIFT)   /* 01: read state */
#define ENET_MAC_DBG_TXFRS_WAITING       (2 << ENET_MAC_DBG_TXFRS_SHIFT)   /* 10: waiting for TxStatus from MAC transmitter */
#define ENET_MAC_DBG_TXFRS_WRITING       (3 << ENET_MAC_DBG_TXFRS_SHIFT)   /* 11: writing the received TxStatus or flushing the TxFIFO */

#define ENET_MAC_DBG_TXFW                (1 << 22)                         /* Bit 22: TxFIFO is writing */
#define ENET_MAC_DBG_TXFNE               (1 << 24)                         /* Bit 24: TxFIFO not empty flag */
#define ENET_MAC_DBG_TXFF                (1 << 25)                         /* Bit 25: TxFIFO full flag */

/* MAC interrupt flag register (GD32_ENET_MAC_INTF) */

#define ENET_MAC_INTF_WUM                (1 << 3)                          /* Bit 3: WUM status */
#define ENET_MAC_INTF_MSC                (1 << 4)                          /* Bit 4: MSC status */
#define ENET_MAC_INTF_MSCR               (1 << 5)                          /* Bit 5: MSC receive status */
#define ENET_MAC_INTF_MSCT               (1 << 6)                          /* Bit 6: MSC transmit status */
#define ENET_MAC_INTF_TMST               (1 << 9)                          /* Bit 9: timestamp trigger status */

/* MAC interrupt mask register (GD32_ENET_MAC_INTMSK) */

#define ENET_MAC_INTMSK_WUMIM            (1 << 3)                          /* Bit 3: PMT interrupt mask */
#define ENET_MAC_INTMSK_TMSTIM           (1 << 9)                          /* Bit 9: Time stamp trigger interrupt mask */
#define ENET_MAC_INTMSK_ALLINTS          (ENET_MAC_INTMSK_WUMIM | ENET_MAC_INTMSK_TMSTIM)

/* MAC address 0 high register (GD32_ENET_MAC_ADDR0H) */

#define ENET_MAC_ADDR0H_ADDR0H_SHIFT     (0)                               /* Bits 0-15: MAC address0 high [47:32] */
#define ENET_MAC_ADDR0H_ADDR0H_MASK      (0xFFFF << ENET_MAC_ADDR0H_ADDR0H_SHIFT)

#define ENET_MAC_ADDR0H_MO               (1 << 31)                         /* Bit 31:always read 1 and must be kept */

/* MAC address 0 low register (GD32_ENET_MAC_ADDR0L) */

/* MAC address 1 high register (GD32_ENET_MAC_ADDR1H) */

#define ENET_MAC_ADDR1H_ADDR1H_SHIFT     (0)                               /* Bits 0-15: MAC address1 high [47:32] */
#define ENET_MAC_ADDR1H_ADDR1H_MASK      (0xFFFF << ENET_MAC_ADDR1H_ADDR1H_SHIFT)

#define ENET_MAC_ADDR1H_MB_SHIFT         (24)                              /* Bits 24-29: mask byte control */
#define ENET_MAC_ADDR1H_MB_MASK          (0x3F << ENET_MAC_ADDR1H_MB_SHIFT)
#define ENET_MAC_ADDR1H_MB_40_47         (0x20 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 29: ENET_MAC_ADDR1H[8-15] */
#define ENET_MAC_ADDR1H_MB_32_39         (0x10 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 28: ENET_MAC_ADDR1H[0-7] */
#define ENET_MAC_ADDR1H_MB_24_31         (0x08 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 27: ENET_MAC_ADDR1L[24-31] */
#define ENET_MAC_ADDR1H_MB_16_23         (0x04 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 26: ENET_MAC_ADDR1L[16-23] */
#define ENET_MAC_ADDR1H_MB_8_15          (0x02 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 25: ENET_MAC_ADDR1L[8-15] */
#define ENET_MAC_ADDR1H_MB_0_7           (0x01 << ENET_MAC_ADDR1H_MB_SHIFT)      /* Bit 24: ENET_MAC_ADDR1L[0-7] */

#define ENET_MAC_ADDR1H_SAF              (1 << 30)                        /* Bit 30: source address filter */
#define ENET_MAC_ADDR1H_AFE              (1 << 31)                        /* Bit 31: address filter enable */

/* MAC address 1 low register (GD32_ENET_MAC_ADDR1L) */

/* MAC address 2 high register (GD32_ENET_MAC_ADDR2H) */

#define ENET_MAC_ADDR2H_ADDR2H_SHIFT     (0)                              /* Bits 0-15: MAC address2 high [47:32] */
#define ENET_MAC_ADDR2H_ADDR2H_MASK      (0xFFFF << ENET_MAC_ADDR2H_ADDR2H_SHIFT)

#define ENET_MAC_ADDR2H_MB_SHIFT        (24)                              /* Bits 24-29: Mask byte control */
#define ENET_MAC_ADDR2H_MB_MASK         (0x3F << ENET_MAC_ADDR2H_MB_SHIFT)
#define ENET_MAC_ADDR2H_MB_40_47        (0x20 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 29: ENET_MAC_ADDR2H[8-15] */
#define ENET_MAC_ADDR2H_MB_32_39        (0x10 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 28: ENET_MAC_ADDR2H[0-7] */
#define ENET_MAC_ADDR2H_MB_24_31        (0x08 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 27: ENET_MAC_ADDR2L[24-31] */
#define ENET_MAC_ADDR2H_MB_16_23        (0x04 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 26: ENET_MAC_ADDR2L[16-23] */
#define ENET_MAC_ADDR2H_MB_8_15         (0x02 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 25: ENET_MAC_ADDR2L[8-15] */
#define ENET_MAC_ADDR2H_MB_0_7          (0x01 << ENET_MAC_ADDR2H_MB_SHIFT)       /* Bit 24: ENET_MAC_ADDR2L[0-7] */

#define ENET_MAC_ADDR2H_SAF             (1 << 30)                         /* Bit 30: source address filter */
#define ENET_MAC_ADDR2H_AFE             (1 << 31)                         /* Bit 31: address filter enable */

/* MAC address 2 low register (GD32_ENET_MAC_ADDR2L) */

/* MAC address 3 high register (GD32_ENET_MAC_ADDR3H) */

#define ENET_MAC_ADDR3H_ADDR3H_SHIFT     (0)                              /* Bits 0-15: MAC address3 high [47:32] */
#define ENET_MAC_ADDR3H_ADDR3H_MASK      (0xFFFF << ENET_MAC_ADDR3H_ADDR3H_SHIFT)

#define ENET_MAC_ADDR3H_MB_SHIFT        (24)                              /* Bits 24-29: Mask byte control */
#define ENET_MAC_ADDR3H_MB_MASK         (0x3F << ENET_MAC_ADDR3H_MB_SHIFT)
#define ENET_MAC_ADDR3H_MB_40_47        (0x20 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 29: ENET_MAC_ADDR3H[8-15] */
#define ENET_MAC_ADDR3H_MB_32_39        (0x10 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 28: ENET_MAC_ADDR3H[0-7] */
#define ENET_MAC_ADDR3H_MB_24_31        (0x08 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 27: ENET_MAC_ADDR3L[24-31] */
#define ENET_MAC_ADDR3H_MB_16_23        (0x04 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 26: ENET_MAC_ADDR3L[16-23] */
#define ENET_MAC_ADDR3H_MB_8_15         (0x02 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 25: ENET_MAC_ADDR3L[8-15] */
#define ENET_MAC_ADDR3H_MB_0_7          (0x01 << ENET_MAC_ADDR3H_MB_SHIFT)         /* Bit 24: ENET_MAC_ADDR3L[0-7] */

#define ENET_MAC_ADDR3H_SAF             (1 << 30)                         /* Bit 30: source address filter */
#define ENET_MAC_ADDR3H_AFE             (1 << 31)                         /* Bit 31: address filter enable */

/* MAC address 3 low register (GD32_ENET_MAC_ADDR3L) */

/* MAC flow control threshold register (GD32_ENET_MAC_FCTH) */

#define ENET_MAC_FCTH_RFA_SHIFT         (0)                               /* Bits 0-2 threshold of active flow control */
#define ENET_MAC_FCTH_RFA_MASK          (7 << ENET_MAC_FCTH_RFA_SHIFT)

#define ENET_MAC_FCTH_RFD_SHIFT         (4)                               /* Bits 4-6 threshold of deactive flow control */
#define ENET_MAC_FCTH_RFD_MASK          (7 << ENET_MAC_FCTH_RFD_SHIFT)

/* MSC Registers */

/* MSC control register (GD32_ENET_MSC_CTL) */

#define ENET_MSC_CTL_CTR                (1 << 0)                          /* Bit 0: counter reset */
#define ENET_MSC_CTL_CTSR               (1 << 1)                          /* Bit 1: counter stop rollover */
#define ENET_MSC_CTL_RTOR               (1 << 2)                          /* Bit 2: reset on read */
#define ENET_MSC_CTL_MCFZ               (1 << 3)                          /* Bit 3: MSC counter freeze */
#define ENET_MSC_CTL_PMC                (1 << 4)                          /* Bit 4: preset MSC counter */
#define ENET_MSC_CTL_AFHPM              (1 << 5)                          /* Bit 5: almost full or half preset mode */

/* MSC receive interrupt flag register (GD32_ENET_MSC_RINTF) */

#define ENET_MSC_RINTF_RFCE             (1 << 5)                          /* Bit 5: received frames CRC error */
#define ENET_MSC_RINTF_RFAE             (1 << 6)                          /* Bit 6: received frames alignment error */
#define ENET_MSC_RINTF_RGUF             (1 << 17)                         /* Bit 17: received good unicast frames */

/* MSC transmit interrupt flag register (GD32_ENET_MSC_TINTF) */

#define ETH_MMCTI_TGFSC                 (1 << 14)                         /* Bit 14: transmitted good frames single collision */
#define ETH_MMCTI_TGFMSC                (1 << 15)                         /* Bit 15: transmitted good frames more single collision */
#define ETH_MMCTI_TGF                   (1 << 21)                         /* Bit 21: transmitted good frames */

/* MSC receive interrupt mask register (GD32_ENET_MSC_RINTMSK) */

#define ENET_MSC_RINTMSK_RFCEIM         (1 << 5)                          /* Bit 5: received frame CRC error interrupt mask */
#define ENET_MSC_RINTMSK_RFAEIM         (1 << 6)                          /* Bit 6: received frames alignment error interrupt mask */
#define ENET_MSC_RINTMSK_RGUFIM         (1 << 17)                         /* Bit 17: received good unicast frames interrupt mask */

/* MSC transmit interrupt mask register (GD32_ENET_MSC_TINTMSK) */

#define ENET_MSC_TINTMSK_TGFSCIM        (1 << 14)                         /* Bit 14: received frame CRC error interrupt mask */
#define ENET_MSC_TINTMSK_TGFMSCIM       (1 << 15)                         /* Bit 15: received frames alignment error interrupt mask */
#define ENET_MSC_TINTMSK_TGFIM          (1 << 21)                         /* Bit 21: received good unicast frames interrupt mask */

/* MSC transmitted good frames after a single collision counter register
 * (GD32_ENET_MSC_SCCNT)
 */

/* MSC transmitted good frames after more than a single collision counter
 * register (GD32_ENET_MSC_MSCCNT)
 */

/* MSC transmitted good frames counter register (GD32_ENET_MSC_TGFCNT) */

/* MSC received frames with CRC error counter register
 * (GD32_ENET_MSC_RFCECNT)
 */

/* MSC received frames with alignment error counter register
 * (GD32_ENET_MSC_RFAECNT)
 */

/* MSC received good unicast frames counter register
 * (GD32_ENET_MSC_RGUFCNT)
 */

/* PTP Registers */

/* PTP time stamp control register (GD32_ENET_PTP_TSCTL) */

#define ENET_PTP_TSCTL_TMSEN            (1 << 0)                          /* Bit 0:  timestamp enable */
#define ENET_PTP_TSCTL_TMSFCU           (1 << 1)                          /* Bit 1:  timestamp fine or coarse update */
#define ENET_PTP_TSCTL_TMSSTI           (1 << 2)                          /* Bit 2:  timestamp system time initialize */
#define ENET_PTP_TSCTL_TMSSTU           (1 << 3)                          /* Bit 3:  timestamp system time update */
#define ENET_PTP_TSCTL_TMSITEN          (1 << 4)                          /* Bit 4:  timestamp interrupt trigger enable */
#define ENET_PTP_TSCTL_TMSARU           (1 << 5)                          /* Bit 5:  timestamp addend register update */

#define ENET_PTP_TSCTL_ARFSEN           (1 << 8)                          /* Bit 8:  all received frames snapshot enable */
#define ENET_PTP_TSCTL_SCROM            (1 << 9)                          /* Bit 9:  subsecond counter rollover mode */
#define ENET_PTP_TSCTL_PFSV             (1 << 10)                         /* Bit 10: PTP frame snooping version */
#define ENET_PTP_TSCTL_ESEN             (1 << 11)                         /* Bit 11: received Ethernet snapshot enable */
#define ENET_PTP_TSCTL_IP6SEN           (1 << 12)                         /* Bit 12: received IPv6 snapshot enable */
#define ENET_PTP_TSCTL_IP4SEN           (1 << 13)                         /* Bit 13: received IPv4 snapshot enable */
#define ENET_PTP_TSCTL_ETMSEN           (1 << 14)                         /* Bit 14: received event type message snapshot enable */
#define ENET_PTP_TSCTL_MNMSEN           (1 << 15)                         /* Bit 15: received master node message snapshot enable */

#define ENET_PTP_TSCTL_CKNT_SHIFT       (16)                              /* Bits 16-17: clock node type for time stamp */
#define ENET_PTP_TSCTL_CKNT_MASK        (3 << ENET_PTP_TSCTL_CKNT_SHIFT)
#define ENET_CKNT_ORDINARY              (0 << ENET_PTP_TSCTL_CKNT_SHIFT)  /* 00: ordinary clock */
#define ENET_CKNT_BOUNDARY              (1 << ENET_PTP_TSCTL_CKNT_SHIFT)  /* 01: boundary clock */
#define ENET_CKNT_END_TO_END            (2 << ENET_PTP_TSCTL_CKNT_SHIFT)  /* 10: end-to-end transparent clock */
#define ENET_CKNT_PEER_TO_PEER          (3 << ENET_PTP_TSCTL_CKNT_SHIFT)  /* 11: peer-to-peer transparent clock */

#define ENET_PTP_TSCTL_MAFEN            (1 << 18)                         /* Bit 18: MAC address filter enable for PTP frame */

/* PTP subsecond increment register (GD32_ENET_PTP_SSINC) */

#define ENET_PTP_SSINC_STMSSI_MASK      (0xFF)

/* PTP time stamp high register (GD32_ENET_PTP_TSH) */

/* PTP time stamp low register (GD32_ENET_PTP_TSL) */

#define ENET_PTP_TSL_STMSS_MASK         (0x7FFFFFFF)                      /* Bits 0-30: system time subseconds */

#define ENET_PTP_TSL_STS                (1 << 31)                         /* Bit 31: system time sign */

/* PTP time stamp update high register (GD32_ENET_PTP_TSUH) */

/* PTP time stamp update low register (GD32_ENET_PTP_TSUL) */

#define ENET_PTP_TSUL_TMSUSS_MASK       (0x7FFFFFFF)                      /* Bits 0-30: timestamp update subseconds */

#define ENET_PTP_TSUL_TMSUPNS           (1 << 31)                         /* Bit 31: timestamp update positive or negative sign */

/* PTP time stamp addend register (GD32_ENET_PTP_TSADDAND) */

/* PTP expected time high register (GD32_ENET_PTP_ETH) */

/* PTP expected time low register (GD32_ENET_PTP_ETL) */

/* PTP time stamp flag register (GD32_ENET_PTP_TSF) */

#define ENET_PTP_TSF_TSSCO              (1 << 0)                          /* Bit 0: timestamp second counter overflow */
#define ENET_PTP_TSF_TTM                (1 << 1)                          /* Bit 1: target time match */

/* PTP PPS control register (GD32_ENET_PTP_PPSCTL) */

#define ENET_PTP_PPSCTL_PPSOFC_SHIFT    (0)       /* Bits 0-3: PPS output frequency configuration */
#define ENET_PTP_PPSCTL_PPSOFC_MASK     (15 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)
#define ENET_PPSOFC_1HZ                 (0 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 1Hz frequency */
#define ENET_PPSOFC_2HZ                 (1 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 2Hz frequency */
#define ENET_PPSOFC_4HZ                 (2 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 4Hz frequency */
#define ENET_PPSOFC_8HZ                 (3 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 8Hz frequency */
#define ENET_PPSOFC_16HZ                (4 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 16Hz frequency */
#define ENET_PPSOFC_32HZ                (5 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 32Hz frequency */
#define ENET_PPSOFC_64HZ                (6 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 64Hz frequency */
#define ENET_PPSOFC_128HZ               (7 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 128Hz frequency */
#define ENET_PPSOFC_256HZ               (8 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 256Hz frequency */
#define ENET_PPSOFC_512HZ               (9 << ENET_PTP_PPSCTL_PPSOFC_SHIFT)  /* PPS output 512Hz frequency */
#define ENET_PPSOFC_1KHZ                (10 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 1024Hz frequency */
#define ENET_PPSOFC_2KHZ                (11 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 2048Hz frequency */
#define ENET_PPSOFC_4KHZ                (12 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 4096Hz frequency */
#define ENET_PPSOFC_8KHZ                (13 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 8192Hz frequency */
#define ENET_PPSOFC_16KHZ               (14 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 16384Hz frequency */
#define ENET_PPSOFC_32KHZ               (15 << ENET_PTP_PPSCTL_PPSOFC_SHIFT) /* PPS output 32768Hz frequency */

/* DMA Registers */

/* DMA bus control register (GD32_ENET_DMA_BCTL) */

#define ENET_DMA_BCTL_SWR               (1 << 0)                             /* Bit 0: software reset */
#define ENET_DMA_BCTL_DAB               (1 << 1)                             /* Bit 1: DMA arbitration */

#define ENET_DMA_BCTL_DPSL_SHIFT        (2)                                  /* Bits 2-6: descriptor skip length */
#define ENET_DMA_BCTL_DPSL_MASK         (31 << ENET_DMA_BCTL_DPSL_SHIFT)
#define ENET_DMA_BCTL_DPSL(n)           ((n) << ENET_DMA_BCTL_DPSL_SHIFT)

#define ENET_DMA_BCTL_DFM               (1 << 7)                             /* Bit 7: descriptor format mode */

#define ENET_DMA_BCTL_PGBL_SHIFT        (8)                                  /* Bits 8-13: programmable burst length */
#define ENET_DMA_BCTL_PGBL_MASK         (0x3F << ENET_DMA_BCTL_PGBL_SHIFT)
#define ENET_DMA_BCTL_PGBL(n)           ((n) << ENET_DMA_BCTL_PGBL_SHIFT)    /* n = 1, 2, 4, 8, 16, 32 */

#define ENET_DMA_BCTL_RTPR_SHIFT        (14)                                 /* Bits 14-15: Rx Tx priority ratio */
#define ENET_DMA_BCTL_RTPR_MASK         (3 << ENET_DMA_BCTL_RTPR_SHIFT)
#define ENET_ARBITRATION_RXTX_1_1       (0 << ENET_DMA_BCTL_RTPR_SHIFT)      /* receive and transmit priority ratio is 1:1 */
#define ENET_ARBITRATION_RXTX_2_1       (1 << ENET_DMA_BCTL_RTPR_SHIFT)      /* receive and transmit priority ratio is 2:1 */
#define ENET_ARBITRATION_RXTX_3_1       (2 << ENET_DMA_BCTL_RTPR_SHIFT)      /* receive and transmit priority ratio is 3:1 */
#define ENET_ARBITRATION_RXTX_4_1       (3 << ENET_DMA_BCTL_RTPR_SHIFT)      /* receive and transmit priority ratio is 4:1 */

#define ENET_DMA_BCTL_FB                (1 << 16)                            /* Bit 16: fixed burst */

#define ENET_DMA_BCTL_RXDP_SHIFT        (17)                                 /* Bits 17-22: Rx DMA PGBL */
#define ENET_DMA_BCTL_RXDP_MASK         (0x3F << ENET_DMA_BCTL_RXDP_SHIFT)
#define ENET_DMA_BCTL_RXDP(n)           ((n) << ENET_DMA_BCTL_RXDP_SHIFT)    /* n = 1, 2, 4, 8, 16, 32 */

#define ENET_DMA_BCTL_UIP               (1 << 23)                            /* Bit 23: use independent PGBL */
#define ENET_DMA_BCTL_FPBL              (1 << 24)                            /* Bit 24: four times PGBL mod */
#define ENET_DMA_BCTL_AA                (1 << 25)                            /* Bit 25: address-aligned */
#define ENET_DMA_BCTL_MB                (1 << 26)                            /* Bit 26: mixed burst */

/* DMA transmit poll enable register (GD32_ENET_DMA_TPEN) */

/* DMA receive poll enable register (GD32_ENET_DMA_RPEN) */

/* DMA receive descriptor table address register (GD32_ENET_DMA_RDTADDR) */

/* DMA transmit descriptor table address register (GD32_ENET_DMA_TDTADDR) */

/* DMA status register (GD32_ENET_DMA_STAT) */

#define ENET_DMA_STAT_TS                (1 << 0)                             /* Bit 0: transmit status */
#define ENET_DMA_STAT_TPS               (1 << 1)                             /* Bit 1: transmit process stopped status */
#define ENET_DMA_STAT_TBU               (1 << 2)                             /* Bit 2: transmit buffer unavailable status */
#define ENET_DMA_STAT_TJT               (1 << 3)                             /* Bit 3: transmit jabber timeout status */
#define ENET_DMA_STAT_RO                (1 << 4)                             /* Bit 4: receive overflow status */
#define ENET_DMA_STAT_TU                (1 << 5)                             /* Bit 5: transmit underflow status */
#define ENET_DMA_STAT_RS                (1 << 6)                             /* Bit 6: receive status */
#define ENET_DMA_STAT_RBU               (1 << 7)                             /* Bit 7: receive buffer unavailable status */
#define ENET_DMA_STAT_RPS               (1 << 8)                             /* Bit 8: receive process stopped status */
#define ENET_DMA_STAT_RWT               (1 << 9)                             /* Bit 9: receive watchdog timeout status */
#define ENET_DMA_STAT_ET                (1 << 10)                            /* Bit 10: early transmit status */
#define ENET_DMA_STAT_FBE               (1 << 13)                            /* Bit 13: fatal bus error status */
#define ENET_DMA_STAT_ER                (1 << 14)                            /* Bit 14: early receive status */
#define ENET_DMA_STAT_AI                (1 << 15)                            /* Bit 15: abnormal interrupt summary */
#define ENET_DMA_STAT_NI                (1 << 16)                            /* Bit 16: normal interrupt summary */

#define ENET_DMA_STAT_RP_SHIFT          (17)                                 /* Bits 17-19: receive process state */
#define ENET_DMA_STAT_RP_MASK           (7 << ENET_DMA_STAT_RP_SHIFT)
#define ENET_RX_STATE_STOPPED           (0 << ENET_DMA_STAT_RP_SHIFT)        /* 000: reset or stop rx command issued */
#define ENET_RX_STATE_FETCHING          (1 << ENET_DMA_STAT_RP_SHIFT)        /* 001: fetching the Rx descriptor */
#define ENET_RX_STATE_WAITING           (3 << ENET_DMA_STAT_RP_SHIFT)        /* 011: waiting for receive packet */
#define ENET_RX_STATE_SUSPENDED         (4 << ENET_DMA_STAT_RP_SHIFT)        /* 100: Rx descriptor unavailable */
#define ENET_RX_STATE_CLOSING           (5 << ENET_DMA_STAT_RP_SHIFT)        /* 101: closing receive descriptor */
#define ENET_RX_STATE_QUEUING           (6 << ENET_DMA_STAT_RP_SHIFT)        /* 111: transferring the receive packet data from recevie buffer to host memory */

#define ENET_DMA_STAT_TP_SHIFT          (20)                                 /* Bits 20-22: transmit process state */
#define ENET_DMA_STAT_TP_MASK           (7 << ENET_DMA_STAT_TP_SHIFT)
#define ENET_TX_STATE_STOPPED           (0 << ENET_DMA_STAT_TP_SHIFT)        /* 000: reset or stop Tx Command issued */
#define ENET_TX_STATE_FETCHING          (1 << ENET_DMA_STAT_TP_SHIFT)        /* 001: fetching the Tx descriptor */
#define ENET_TX_STATE_WAITING           (2 << ENET_DMA_STAT_TP_SHIFT)        /* 010: waiting for status */
#define ENET_TX_STATE_READING           (3 << ENET_DMA_STAT_TP_SHIFT)        /* 011: reading the data from host memory buffer and queuing it to transmit buffer */
#define ENET_TX_STATE_SUSPENDED         (6 << ENET_DMA_STAT_TP_SHIFT)        /* 110: Tx descriptor unavailabe or transmit buffer underflow */
#define ENET_TX_STATE_CLOSING           (7 << ENET_DMA_STAT_TP_SHIFT)        /* 111: closing Tx descriptor */

#define ENET_DMA_STAT_EB_SHIFT          (23)                                 /* Bits 23-25: error bits status */
#define ENET_DMA_STAT_EB_MASK           (7 << ENET_DMA_STAT_EB_SHIFT)
#define ENET_ERROR_TXDATA_TRANSFER      (1 << ENET_DMA_STAT_EB_SHIFT)        /* Bit 23 1 error during data transfer by TxDMA or RxDMA */
#define ENET_ERROR_READ_TRANSFER        (2 << ENET_DMA_STAT_EB_SHIFT)        /* Bit 24 1 error during write transfer or read transfer */
#define ENET_ERROR_DESC_ACCESS          (4 << ENET_DMA_STAT_EB_SHIFT)        /* Bit 25 1 error during descriptor or buffer access */

#define ENET_DMA_STAT_MSC               (1 << 27)                            /* Bit 27: MSC status */
#define ENET_DMA_STAT_WUM               (1 << 28)                            /* Bit 28: WUM status */
#define ENET_DMA_STAT_TST               (1 << 29)                            /* Bit 29: timestamp trigger status */

/* DMA control register (GD32_ENET_DMA_CTL) */

#define ENET_DMA_CTL_SRE                (1 << 1)                             /* Bit 1: start/stop receive enable */
#define ENET_DMA_CTL_OSF                (1 << 2)                             /* Bit 2: operate on second frame */

#define ENET_DMA_CTL_RTHC_SHIFT         (3)                                  /* Bits 3-4: receive threshold control */
#define ENET_DMA_CTL_RTHC_MASK          (3 << ENET_DMA_CTL_RTHC_SHIFT)
#define ENET_RX_THRESHOLD_64BYTES       (0 << ENET_DMA_CTL_RTHC_SHIFT)       /* threshold level is 64 Bytes */
#define ENET_RX_THRESHOLD_32BYTES       (1 << ENET_DMA_CTL_RTHC_SHIFT)       /* threshold level is 32 Bytes */
#define ENET_RX_THRESHOLD_96BYTES       (2 << ENET_DMA_CTL_RTHC_SHIFT)       /* threshold level is 96 Bytes */
#define ENET_RX_THRESHOLD_128BYTES      (3 << ENET_DMA_CTL_RTHC_SHIFT)       /* threshold level is 128 Bytes */

#define ENET_DMA_CTL_FUF                (1 << 6)                             /* Bit 6: forward undersized good frames */
#define ENET_DMA_CTL_FERF               (1 << 7)                             /* Bit 7: forward error frames */
#define ENET_DMA_CTL_STE                (1 << 13)                            /* Bit 13: start/stop transmission enable */

#define ENET_DMA_CTL_TTHC_SHIFT         (14)                                 /* Bits 14-16: transmit threshold control */
#define ENET_DMA_CTL_TTHC_MASK          (7 << ENET_DMA_CTL_TTHC_SHIFT)
#define ENET_TX_THRESHOLD_64BYTES       (0 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 64 Bytes */
#define ENET_TX_THRESHOLD_128BYTES      (1 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 128 Bytes */
#define ENET_TX_THRESHOLD_192BYTES      (2 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 192 Bytes */
#define ENET_TX_THRESHOLD_256BYTES      (3 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 256 Bytes */
#define ENET_TX_THRESHOLD_40BYTES       (4 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 40 Bytes */
#define ENET_TX_THRESHOLD_32BYTES       (5 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 32 Bytes */
#define ENET_TX_THRESHOLD_24BYTES       (6 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 24 Bytes */
#define ENET_TX_THRESHOLD_16BYTES       (7 << ENET_DMA_CTL_TTHC_SHIFT)       /* threshold level is 16 Bytes */

#define ENET_DMA_CTL_FTF                (1 << 20)                            /* Bit 20: flush transmit FIFO */
#define ENET_DMA_CTL_TSFD               (1 << 21)                            /* Bit 21: transmit store-and-forward */
#define ENET_DMA_CTL_DAFRF              (1 << 24)                            /* Bit 24: disable flushing of received frames */
#define ENET_DMA_CTL_RSFD               (1 << 25)                            /* Bit 25: receive store-and-forward */
#define ENET_DMA_CTL_DTCERFD            (1 << 26)                            /* Bit 26: dropping of TCP/IP checksum error frames disable */

/* DMA interrupt enable register (GD32_ENET_DMA_INTEN) */

#define ENET_DMA_INTEN_TIE              (1 << 0)                             /* Bit 0: transmit interrupt enable */
#define ENET_DMA_INTEN_TPSIE            (1 << 1)                             /* Bit 1: transmit process stopped interrupt enable */
#define ENET_DMA_INTEN_TBUIE            (1 << 2)                             /* Bit 2: transmit buffer unavailable interrupt enable */
#define ENET_DMA_INTEN_TJTIE            (1 << 3)                             /* Bit 3: transmit jabber timeout interrupt enable */
#define ENET_DMA_INTEN_ROIE             (1 << 4)                             /* Bit 4: receive overflow interrupt enable */
#define ENET_DMA_INTEN_TUIE             (1 << 5)                             /* Bit 5: transmit underflow interrupt enable */
#define ENET_DMA_INTEN_RIE              (1 << 6)                             /* Bit 6: receive interrupt enable */
#define ENET_DMA_INTEN_RBUIE            (1 << 7)                             /* Bit 7: receive buffer unavailable interrupt enable */
#define ENET_DMA_INTEN_RPSIE            (1 << 8)                             /* Bit 8: receive process stopped interrupt enable */
#define ENET_DMA_INTEN_RWTIE            (1 << 9)                             /* Bit 9: receive watchdog timeout interrupt enable */
#define ENET_DMA_INTEN_ETIE             (1 << 10)                            /* Bit 10: early transmit interrupt enable */
#define ENET_DMA_INTEN_FBEIE            (1 << 13)                            /* Bit 13: fatal bus error interrupt enable */
#define ENET_DMA_INTEN_ERIE             (1 << 14)                            /* Bit 14: early receive interrupt enable */
#define ENET_DMA_INTEN_AIE              (1 << 15)                            /* Bit 15: abnormal interrupt summary enable */
#define ENET_DMA_INTEN_NIE              (1 << 16)                            /* Bit 16: normal interrupt summary enable */

/* DMA missed frame and buffer overflow counter register
 * (GD32_ENET_DMA_MFBOCNT)
 */

#define ENET_DMA_MFBOCNT_MSFC_SHIFT     (0)                                  /* Bits 0-15: missed frames by the controller */
#define ENET_DMA_MFBOCNT_MSFC_MASK      (0xFFFF << ENET_DMA_MFBOCNT_MSFC_SHIFT)

#define ENET_DMA_MFBOCNT_MSFA_SHIFT     (17)                                 /* Bits 17-27: missed frames by the application */
#define ENET_DMA_MFBOCNT_MSFA_MASK      (0x7FF << ENET_DMA_MFBOCNT_MSFC_SHIFT)

/* DMA receive state watchdog counter register (GD32_ENET_DMA_RSWDC) */

#define ETH_DMARSWTR_MASK               (0xFF)

/* DMA current transmit descriptor address register (GD32_ENET_DMA_CTDADDR) */

/* DMA current receive descriptor address register (GD32_ENET_DMA_CRDADDR) */

/* DMA current transmit buffer address register (GD32-ENET_DMA_CTBADDR) */

/* DMA current receive buffer address register （GD32_ENET_DMA_CRBADDR） */

/* DMA Descriptors */

/* ENET DMA Tx descriptor TDES0 */

#define ENET_TDES0_DB                   (1 << 0)                             /* Bit 0: deferred */
#define ENET_TDES0_UFE                  (1 << 1)                             /* Bit 1: underflow error */
#define ENET_TDES0_EXD                  (1 << 2)                             /* Bit 2: excessive deferral */

#define ENET_TDES0_COCNT_SHIFT          (3)                                  /* Bits 3-6: collision count */
#define ENET_TDES0_COCNT_MASK           (15 << ENET_TDES0_COCNT_SHIFT)

#define ENET_TDES0_VFRM                 (1 << 7)                             /* Bit 7: VLAN frame */
#define ENET_TDES0_ECO                  (1 << 8)                             /* Bit 8: excessive collision */
#define ENET_TDES0_LCO                  (1 << 9)                             /* Bit 9: late collision */
#define ENET_TDES0_NCA                  (1 << 10)                            /* Bit 10: no carrier */
#define ENET_TDES0_LCA                  (1 << 11)                            /* Bit 11: loss of carrier */
#define ENET_TDES0_IPPE                 (1 << 12)                            /* Bit 12: IP payload error */
#define ENET_TDES0_FRMF                 (1 << 13)                            /* Bit 13: frame flushed */
#define ENET_TDES0_JT                   (1 << 14)                            /* Bit 14: jabber timeout */
#define ENET_TDES0_ES                   (1 << 15)                            /* Bit 15: error summaryy */
#define ENET_TDES0_IPHE                 (1 << 16)                            /* Bit 16: IP header error */
#define ENET_TDES0_TTMSS                (1 << 17)                            /* Bit 17: transmit timestamp status */
#define ENET_TDES0_TCHM                 (1 << 20)                            /* Bit 20: the second address chained mode */
#define ENET_TDES0_TERM                 (1 << 21)                            /* Bit 21: transmit end of ring mode */

#define ENET_TDES0_CM_SHIFT                 (22)                             /* Bits 22-23: checksum mode */
#define ENET_TDES0_CM_MASK                  (3 << ENET_TDES0_CM_SHIFT)
#define ENET_CHECKSUM_DISABLE               (0 << ENET_TDES0_CM_SHIFT)       /* checksum insertion disabled */
#define ENET_CHECKSUM_IPV4HEADER            (1 << ENET_TDES0_CM_SHIFT)       /* only IP header checksum calculation and insertion are enabled */
#define ENET_CHECKSUM_TCPUDPICMP_SEGMENT    (2 << ENET_TDES0_CM_SHIFT)       /* TCP/UDP/ICMP checksum insertion calculated but pseudo-header */
#define ENET_CHECKSUM_TCPUDPICMP_FULL       (3 << ENET_TDES0_CM_SHIFT)       /* TCP/UDP/ICMP checksum insertion fully calculated */

#define ENET_TDES0_TTSEN                (1 << 25)                            /* Bit 25: transmit timestamp function enable */
#define ENET_TDES0_DPAD                 (1 << 26)                            /* Bit 26: disable adding pad */
#define ENET_TDES0_DCRC                 (1 << 27)                            /* Bit 27: disable CRC */
#define ENET_TDES0_FSG                  (1 << 28)                            /* Bit 28: first segment */
#define ENET_TDES0_LSG                  (1 << 29)                            /* Bit 29: last segment */
#define ENET_TDES0_INTC                 (1 << 30)                            /* Bit 30: interrupt on completion */
#define ENET_TDES0_DAV                  (1 << 31)                            /* Bit 31: DAV bit */

/* ENET DMA Tx descriptor TDES1 */

#define ENET_TDES1_TB1S_SHIFT           (0)                                  /* Bits 0-12: transmit buffer 1 size */
#define ENET_TDES1_TB1S_MASK            (0x1FFF << ENET_TDES1_TB1S_SHIFT)

#define ENET_TDES1_TB2S_SHIFT           (16)                                 /* Bits 16-28: transmit buffer 2 size */
#define ENET_TDES1_TB2S_MASK            (0x1FFF << ENET_TDES1_TB2S_SHIFT)

/* ENET DMA Tx descriptor TDES2 (32-bit address) */

/* ENET DMA Tx descriptor TDES3 (32-bit address) */

/* ENET DMA Tx descriptor TDES6 (32-bit time stamp) */

/* ENET DMA Tx descriptor TDES7 (32-bit time stamp) */

/* ENET DMA Rx descriptor RDES0 */

#define ENET_RDES0_PCERR                (1 << 0)                            /* Bit 0: payload checksum error */
#define ENET_RDES0_EXSV                 (1 << 0)                            /* Bit 0: extended status valid */
#define ENET_RDES0_CERR                 (1 << 1)                            /* Bit 1: CRC error */
#define ENET_RDES0_DBERR                (1 << 2)                            /* Bit 2: dribble bit error */
#define ENET_RDES0_RERR                 (1 << 3)                            /* Bit 3: receive error */
#define ENET_RDES0_RWDT                 (1 << 4)                            /* Bit 4: receive watchdog timeout */
#define ENET_RDES0_FRMT                 (1 << 5)                            /* Bit 5: frame type */
#define ENET_RDES0_LCO                  (1 << 6)                            /* Bit 6: late collision */
#define ENET_RDES0_IPHERR               (1 << 7)                            /* Bit 7: IP frame header error */
#define ENET_RDES0_TSV                  (1 << 7)                            /* Bit 7: timestamp valid */
#define ENET_RDES0_LDES                 (1 << 8)                            /* Bit 8: last descriptor */
#define ENET_RDES0_FDES                 (1 << 9)                            /* Bit 9: first descriptor */
#define ENET_RDES0_VTAG                 (1 << 10)                           /* Bit 10: VLAN tag */
#define ENET_RDES0_OERR                 (1 << 11)                           /* Bit 11: overflow Error */
#define ENET_RDES0_LERR                 (1 << 12)                           /* Bit 12: length error */
#define ENET_RDES0_SAFF                 (1 << 13)                           /* Bit 13: source address filter fail */
#define ENET_RDES0_DERR                 (1 << 14)                           /* Bit 14: descriptor error */
#define ENET_RDES0_ERRS                 (1 << 15)                           /* Bit 15: error summary */

#define ENET_RDES0_FRML_SHIFT           (16)                                /* Bits 16-29: frame length */
#define ENET_RDES0_FRML_MASK            (0x3FFF << ENET_RDES0_FRML_SHIFT)

#define ENET_RDES0_DAFF                 (1 << 30)                           /* Bit 30: destination address filter fail */
#define ENET_RDES0_DAV                  (1 << 31)                           /* Bit 31: descriptor available */

/* ENET DMA Rx descriptor RDES1 */

#define ENET_RDES1_RB1S_SHIFT           (0)                                 /* Bits 0-12: receive buffer 1 size */
#define ENET_RDES1_RB1S_MASK            (0x1FFF << ENET_RDES1_RB1S_SHIFT)

#define ENET_RDES1_RCHM                 (1 << 14)                           /* Bit 14: receive chained mode for second address */
#define ENET_RDES1_RERM                 (1 << 15)                           /* Bit 15: receive end of ring mode */

#define ENET_RDES1_RB2S_SHIFT           (16)                                /* Bits 16-28: receive buffer 2 size */
#define ENET_RDES1_RB2S_MASK            (0x1FFF << ENET_RDES1_RB2S_SHIFT)

#define ENET_RDES1_DINTC                (1 << 31)                           /* Bit 31: disable interrupt on completion */

/* ENET DMA Rx descriptor RDES2 (32-bit address) */

/* ENET DMA Rx descriptor RDES3 (32-bit address) */

/* ENET DMA Rx descriptor RDES4 */

#define ENET_RDES4_IPPLDT_SHIFT         (0)                                 /* Bits 0-2: IP frame payload type */
#define ENET_RDES4_IPPLDT_MASK          (7 << ENET_RDES4_IPPLDT_SHIFT)
#define ENET_RDES4_IPPLDT_UDP           (1 << ENET_RDES4_IPPLDT_SHIFT)      /* UDP payload in IP datagram */
#define ENET_RDES4_IPPLDT_TCP           (2 << ENET_RDES4_IPPLDT_SHIFT)      /* TCP payload in IP datagram */
#define ENET_RDES4_IPPLDT_ICMP          (3 << ENET_RDES4_IPPLDT_SHIFT)      /* ICMP payload in IP datagram */

#define ENET_RDES4_IPHERR               (1 << 3)                            /* Bit 3: IP frame header error */
#define ENET_RDES4_IPPLDERR             (1 << 4)                            /* Bit 4: IP frame payload error */
#define ENET_RDES4_IPCKSB               (1 << 5)                            /* Bit 5: IP frame checksum bypassed */
#define ENET_RDES4_IPF4                 (1 << 6)                            /* Bit 6: IP frame in version 4 */
#define ENET_RDES4_IPF6                 (1 << 7)                            /* Bit 7: IP frame in version 6 */

#define ENET_RDES4_PTPMT_SHIFT          (8)                                 /* Bits 8-11: PTP message type */
#define ENET_RDES4_PTPMT_MASK           (15 << ENET_RDES4_PTPMT_SHIFT)
#define ENET_RDES4_PTPMT_NONE           (0 << ENET_RDES4_PTPMT_SHIFT)       /* No PTP message received */
#define ENET_RDES4_PTPMT_SYNC           (1 << ENET_RDES4_PTPMT_SHIFT)       /* SYNC (all clock types) */
#define ENET_RDES4_PTPMT_FOLLOWUP       (2 << ENET_RDES4_PTPMT_SHIFT)       /* Follow_Up (all clock types) */
#define ENET_RDES4_PTPMT_DELAYREQ       (3 << ENET_RDES4_PTPMT_SHIFT)       /* Delay_Req (all clock types) */
#define ENET_RDES4_PTPMT_DELAYRESP      (4 << ENET_RDES4_PTPMT_SHIFT)       /* Delay_Resp (all clock types) */
#define ENET_RDES4_PTPMT_PDELREQAM      (5 << ENET_RDES4_PTPMT_SHIFT)       /* Pdelay_Req (in peer-to-peer transparent clock) or Announce (in ordinary or boundary clock) */
#define ENET_RDES4_PTPMT_PDELREQMM      (6 << ETH_RDES4_PMT_SHIFT)          /* Pdelay_Resp (in peer-to-peer transparent clock) or Management (in ordinary or boundary clock) */
#define ENET_RDES4_PTPMT_PDELREQFUS     (7 << ETH_RDES4_PMT_SHIFT)          /* Pdelay_Resp_Follow_Up (in peer-to-peer transparent clock) or Signaling (for ordinary or boundary clock) */

#define ENET_RDES4_PTPOEF               (1 << 12)                           /* Bit 12: ENET_RDES4_PTPMT */
#define ENET_RDES4_PTPVF                (1 << 13)                           /* Bit 13: PTP version format */

/* ENET DMA Rx descriptor RDES5 (Reserved) */

/* ENET DMA Rx descriptor RDES6 (32-bit time stamp) */

/* ENET DMA Rx descriptor RDES7 (32-bit time stamp) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Ethernet TX DMA Descriptor */

struct enet_txdesc_s
{
  /* Normal DMA descriptor words */

  volatile uint32_t tdes0;   /* Status */
  volatile uint32_t tdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t tdes2;   /* Buffer1 address pointer */
  volatile uint32_t tdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp */

#ifdef CONFIG_GD32F4_ENET_ENHANCEDDESC
  volatile uint32_t tdes4;   /* Reserved */
  volatile uint32_t tdes5;   /* Reserved */
  volatile uint32_t tdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t tdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/* Ethernet RX DMA Descriptor */

struct enet_rxdesc_s
{
  volatile uint32_t rdes0;   /* Status */
  volatile uint32_t rdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t rdes2;   /* Buffer1 address pointer */
  volatile uint32_t rdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp and PTP support */

#ifdef CONFIG_GD32F4_ENET_ENHANCEDDESC
  volatile uint32_t rdes4;   /* Extended status for PTP receive descriptor */
  volatile uint32_t rdes5;   /* Reserved */
  volatile uint32_t rdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t rdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

#endif /* __ASSEMBLY__ */
#endif /* GD32_NETHERNET > 0 */
#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_ENET_H */
