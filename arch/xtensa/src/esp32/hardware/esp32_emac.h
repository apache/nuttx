/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_emac.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_EMAC_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_EMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Ethernet MAC Register Base Address
 ****************************************************************************/

#define EMAC_REG_BASE         (0x3ff69000)

/****************************************************************************
 * MAC Address Register Address
 ****************************************************************************/

#define MAC_ADDR0_REG         (0x3ff5a004)
#define MAC_ADDR1_REG         (0x3ff5a008)

/****************************************************************************
 * Ethernet MAC Registers Offsets
 ****************************************************************************/

/* Ethernet MAC DMA bus mode configuration */

#define EMAC_DMA_BMR_OFFSET   (0x0000)

/* Ethernet MAC DMA start TX */

#define EMAC_DMA_STR_OFFSET   (0x0004)

/* Ethernet MAC DMA start RX */

#define EMAC_DMA_SRR_OFFSET   (0x0008)

/* Ethernet MAC DMA RX description base address */

#define EMAC_DMA_RDBR_OFFSET  (0x000c)

/* Ethernet MAC DMA TX description base address */

#define EMAC_DMA_TDBR_OFFSET  (0x0010)

/* Ethernet MAC DMA interrupt and other status */

#define EMAC_DMA_SR_OFFSET    (0x0014)

/* Ethernet MAC DMA operation mode */

#define EMAC_DMA_OMR_OFFSET   (0x0018)

/* Ethernet MAC DMA interrupt enable */

#define EMAC_DMA_IER_OFFSET   (0x001c)

/* Ethernet MAC DMA drop frame and overflow count */

#define EMAC_DMA_DFR_OFFSET   (0x0020)

/* Ethernet MAC RX watch dog timer */

#define EMAC_DMA_RWDR_OFFSET  (0x0024)

/* Ethernet MAC current TX description address */

#define EMAC_DMA_CTDAR_OFFSET (0x0048)

/* Ethernet MAC current RX description address */

#define EMAC_DMA_CRDAR_OFFSET (0x004c)

/* Ethernet MAC current TX description buffer address */

#define EMAC_DMA_CTBAR_OFFSET (0x0050)

/* Ethernet MAC current RX description buffer address */

#define EMAC_DMA_CRBAR_OFFSET (0x0054)

/* Ethernet MAC RMII clock divide */

#define EMAC_ECOCR_OFFSET     (0x800)

/* Ethernet MAC RMII clock half and whole divide */

#define EMAC_EOCCR_OFFSET     (0x804)

/* Clock resource selection and enbale */

#define EMAC_ECCR_OFFSET      (0x808)

/* Selection MII or RMII interface of PHY */

#define EMAC_PIR_OFFSET       (0x80C)

/* RAM power-down enable */

#define EMAC_PDR_OFFSET       (0x810)

/* Ethernet MAC configuration */

#define EMAC_CR_OFFSET        (0x1000)

/* Ethernet MAC frame filter */

#define EMAC_FFR_OFFSET       (0x1004)

/* Ethernet MAC access PHY chip address */

#define EMAC_MAR_OFFSET       (0x1010)

/* Ethernet MAC access PHY chip data */

#define EMAC_MDR_OFFSET       (0x1014)

/* Ethernet MAC frame flow control */

#define EMAC_FCR_OFFSET       (0x1018)

/* Ethernet MAC status debugging */

#define EMAC_DBGR_OFFSET      (0x1024)

/* Ethernet MAC remote wake-up frame filter */

#define EMAC_RWUFFR_OFFSET    (0x1028)

/* Ethernet MAC PMT control and status */

#define EMAC_PMTCSR_OFFSET    (0x102c)

/* Ethernet MAC LPI control and status */

#define EMAC_LPICSR_OFFSET    (0x1030)

/* Ethernet MAC LPI timers control */

#define EMAC_LPICR_OFFSET     (0x1034)

/* Ethernet MAC interrupt status */

#define EMAC_ISR_OFFSET       (0x1038)

/* Ethernet MAC interrupt mask */

#define EMAC_IMR_OFFSET       (0x103c)

/* Ethernet MAC address high 16 bits */

#define EMAC_MA0HR_OFFSET     (0x1040)

/* Ethernet MAC address low 32 bits */

#define EMAC_MA0LR_OFFSET     (0x1044)

/* Ethernet MAC address1 filter high 16 bits */

#define EMAC_MA1HR_OFFSET     (0x1048)

/* Ethernet MAC address1 filter low 32 bits */

#define EMAC_MA1LR_OFFSET     (0x104c)

/* Ethernet MAC address2 filter high 16 bits */

#define EMAC_MA2HR_OFFSET     (0x1050)

/* Ethernet MAC address2 filter low 32 bits */

#define EMAC_MA2LR_OFFSET     (0x1054)

/* Ethernet MAC address3 filter high 16 bits */

#define EMAC_MA3HR_OFFSET     (0x1058)

/* Ethernet MAC address3 filter low 32 bits */

#define EMAC_MA3LR_OFFSET     (0x105c)

/* Ethernet MAC address4 filter high 16 bits */

#define EMAC_MA4HR_OFFSET     (0x1060)

/* Ethernet MAC address4 filter low 32 bits */

#define EMAC_MA4LR_OFFSET     (0x1064)

/* Ethernet MAC address5 filter high 16 bits */

#define EMAC_MA5HR_OFFSET     (0x1068)

/* Ethernet MAC address5 filter low 32 bits */

#define EMAC_MA5LR_OFFSET     (0x106c)

/* Ethernet MAC address6 filter high 16 bits */

#define EMAC_MA6HR_OFFSET     (0x1070)

/* Ethernet MAC address6 filter low 32 bits */

#define EMAC_MA6LR_OFFSET     (0x1074)

/* Ethernet MAC address7 filter high 16 bits */

#define EMAC_MA7HR_OFFSET     (0x1078)

/* Ethernet MAC address7 filter low 32 bits */

#define EMAC_MA7LR_OFFSET     (0x107c)

/* Ethernet MAC link communication status */

#define EMAC_LCSR_OFFSET      (0x10d8)

/* Ethernet MAC watch dog timeout */

#define EMAC_WDR_OFFSET       (0x10dc)

/****************************************************************************
 * Register Bitfield Definitions
 ****************************************************************************/

/* Register EMAC_DMA_BMR ****************************************************/

#define EMAC_MB_E             (BIT(26)) /* Mixed burst */
#define EMAC_AAB_E            (BIT(25)) /* Address align burst */
#define EMAC_PBLX8_E          (BIT(24)) /* PBL value x8 */
#define EMAC_SPBL_E           (BIT(23)) /* Seperated PBL */
#define EMAC_RXDMA_PBL_S      (17)      /* RX DMA PBL value shift */
#define EMAC_RXDMA_PBL_V      (0x3f)    /* RX DMA PBL value max value */
#define EMAC_FB_E             (BIT(16)) /* Fixed burst */
#define EMAC_PR_S             (14)      /* Priority ratio shift */
#define EMAC_PR_V             (0x3)     /* Priority ratio max value */
#define EMAC_PBL_S            (8)       /* Program burst length shift */
#define EMAC_PBL_V            (0x3f)    /* Program burst length max value */
#define EMAC_ADS_E            (BIT(7))  /* Extend DMA description size */
#define EMAC_DSL_S            (2)       /* Skip length shift */
#define EMAC_DSL_V            (0x1f)    /* Skip length max value */
#define EMAC_DAS_E            (BIT(1))  /* DMA arbitration scheme */
#define EMAC_SR_E             (BIT(0))  /* Software reset */

/* Register EMAC_DMA_SR *****************************************************/

#define EMAC_TTI              (BIT(29)) /* Timestamp triggers interrupt */
#define EMAC_PMTI             (BIT(28)) /* PMT interrupt  */
#define EMAC_EB_S             (23)      /* Error bits shift */
#define EMAC_EB_V             (0x7)     /* Error bits max value */
#define EMAC_DTFS_S           (20)      /* DMA TX FSM state shift */
#define EMAC_DTFS_V           (0x7)     /* DMA TX FSM state max value */
#define EMAC_DRFS_S           (17)      /* DMA RX FSM state shift */
#define EMAC_DRFS_V           (0x7)     /* DMA RX FSM state max value */
#define EMAC_NIS              (BIT(16)) /* Normal interrupt summary */
#define EMAC_AIS              (BIT(15)) /* Abnormal interrupt summary */
#define EMAC_ERI              (BIT(14)) /* Early receive interrupt */
#define EMAC_FBEI             (BIT(13)) /* Fatal bus error interrupt */
#define EMAC_ETI              (BIT(10)) /* Early transmit interrupt */
#define EMAC_RWDTO            (BIT(9))  /* Receive watch dog timeout */
#define EMAC_RPS              (BIT(8))  /* Receive process stop */
#define EMAC_RBU              (BIT(7))  /* Receive buffer unavailable */
#define EMAC_RI               (BIT(6))  /* Receive interrupt */
#define EMAC_TUF              (BIT(5))  /* Transmit underflow */
#define EMAC_ROF              (BIT(4))  /* Receive overflow */
#define EMAC_TJTO             (BIT(3))  /* Transmit jabber timeout */
#define EMAC_TBU              (BIT(2))  /* Transmit buffer unavailable */
#define EMAC_TPS              (BIT(1))  /* Transmit process stop */
#define EMAC_TI               (BIT(0))  /* Transmit interrupt */

/* Register EMAC_DMA_OMR ****************************************************/

#define EMAC_DTIEF_D          (BIT(26)) /* Disable drop TCPIP error frame */
#define EMAC_FRF_E            (BIT(25)) /* Forward received frame */
#define EMAC_FRF_D            (BIT(24)) /* Disable flush received frame*/
#define EMAC_FFSF_E           (BIT(21)) /* Forward FIFO stored frame */
#define EMAC_FTF_E            (BIT(20)) /* Flush TX FiFo */
#define EMAC_TTC_S            (14)      /* TX threshold control shift */
#define EMAC_TTC_V            (0x7)     /* TX threshold control max value */
#define EMAC_SST_E            (BIT(13)) /* Start Stop transmit */
#define EMAC_FEF_E            (BIT(7))  /* Forward error frame */
#define EMAC_FSF_E            (BIT(6))  /* Forward small(<64B) frame */
#define EMAC_FLF_E            (BIT(5))  /* Forward large frame */
#define EMAC_RTC_S            (3)       /* RX threshold control shift */
#define EMAC_RTC_V            (0x3)     /* RX threshold control max value */
#define EMAC_OSF_E            (BIT(2))  /* Operate second frame */
#define EMAC_SSR_E            (BIT(1))  /* Start Stop receive */

/* Register EMAC_EOCCR ******************************************************/

#define EMAC_OSEC_E           (BIT(24)) /* OSC select external clock */
#define EMAC_OHDF100M_S       (18)      /* Half divide frequency shift when 100MHz */
#define EMAC_OHDF100M_V       (0x3f)    /* Half divide frequency max value when 100MHz */
#define EMAC_ODF100M_S        (12)      /* Divide frequency shift when 100MHz */
#define EMAC_ODF100M_V        (0x3f)    /* Divide frequency max value when 100MHz */
#define EMAC_OHDF10M_S        (6)       /* Half divide frequency shift when 10MHz */
#define EMAC_OHDF10M_V        (0x3f)    /* Half divide frequency max value when 10MHz */
#define EMAC_ODF10M_S         (0)       /* Divide frequency shift when 10MHz */
#define EMAC_ODF10M_V         (0x3f)    /* Divide frequency max value when 10MHz */

/* Register EMAC_ECCR *******************************************************/

#define EMAC_RXC_E            (BIT(3))  /* RX clock */
#define EMAC_TXC_E            (BIT(2))  /* TX clock */
#define EMAC_IAC_E            (BIT(1))  /* Internal APLL clock */
#define EMAC_EXC_E            (BIT(0))  /* External XTAL clock */

/* Register EMAC_PIR ********************************************************/

#define EMAC_PIS_S            (13)                /* PHY interf. sel. shift */
#define EMAC_PIS_MII          (0)                 /* MII interface */
#define EMAC_PIS_RMII         (0x4 << EMAC_PIS_S) /* RMII interface */

/* Register EMAC_CR *********************************************************/

#define EMAC_SAIRC_S          (28)      /* Control frame src address shift */
#define EMAC_SAIRC_V          (0x7)     /* Control frame src address max value */
#define EMAC_PST2KF_E         (BIT(27)) /* Pass smaller that 2K frame */
#define EMAC_WD_D             (BIT(23)) /* Disable watch dog */
#define EMAC_JT_D             (BIT(22)) /* Disable Jabber timer */
#define EMAC_RLF_E            (BIT(21)) /* Receive large frame */
#define EMAC_TFMIFG_S         (17)      /* Time frame min IFG shift */
#define EMAC_TFMIFG_V         (0x7)     /* Time frame min IFG max value */
#define EMAC_DCRS_E           (BIT(16)) /* Drop CRS */
#define EMAC_SS_E             (BIT(15)) /* Speed select */
#define EMAC_100M_E           (BIT(14)) /* 100MHz */
#define EMAC_RXO_E            (BIT(13)) /* Stop RX when trigger TX_EN */
#define EMAC_LB_E             (BIT(12)) /* Loop back */
#define EMAC_FD_E             (BIT(11)) /* Full duplex */
#define EMAC_RIPCOFFLOAD_E    (BIT(10)) /* Calculate ethernet payload */
#define EMAC_TXR_E            (BIT(9))  /* TX retry */
#define EMAC_SPOFCS_E         (BIT(7))  /* Strip pad or FCS */
#define EMAC_BOL_S            (5)       /* Backoff limit shift */
#define EMAC_BOL_V            (0x3)     /* Backoff limit max value */
#define EMAC_DF_E             (BIT(4))  /* Deferral check */
#define EMAC_TX_E             (BIT(3))  /* Enable EMAC TX */
#define EMAC_RX_E             (BIT(2))  /* Enable EMAC RX */
#define EMAC_PLTF_S           (0)       /* Frame preamble bytes select shift */
#define EMAC_PLTF_V           (0x3)     /* Frame preamble bytes select max value */

/* Register EMAC_FFR ********************************************************/

#define EMAC_RA_E             (BIT(31)) /* Receive all frame */
#define EMAC_SAF_E            (BIT(9))  /* Src address filter */
#define EMAC_SARF_E           (BIT(8))  /* Src address reverse filter */
#define EMAC_PCF_S            (6)       /* Proccess control frame shift */
#define EMAC_PCF_V            (0x3)     /* Proccess control frame max value */
#define EMAC_BF_D             (BIT(5))  /* Disable pass broadcast frame */
#define EMAC_PMF_E            (BIT(4))  /* Pass multicast frame */
#define EMAC_DAIF_E           (BIT(3))  /* multicast and unicast reverse filter */
#define EMAC_PA_E             (BIT(0))  /* Pass all frame */

/* Register EMAC_MAR ********************************************************/

#define EMAC_PCA_S            (11)      /* PHY chip address shift */
#define EMAC_PCA_V            (0x1f)    /* PHY chip address max value */
#define EMAC_PCRA_S           (6)       /* PHY chip register address shift */
#define EMAC_PCRA_V           (0x1f)    /* PHY chip register address max value */
#define EMAC_SMICS_S          (2)       /* SMI clock source shift */
#define EMAC_SMICS_V          (0xf)     /* SMI clock source max value */
#define EMAC_HW_E             (BIT(1))  /* Enable PHY write */
#define EMAC_PIB              (BIT(0))  /* PHY is busy */

/* Register EMAC_FCR ********************************************************/

#define EMAC_CFPT_S           (16)      /* Control frame pause time shift */
#define EMAC_CFPT_V           (0xffff)  /* Control frame pause time max value */
#define EMAC_PFPT_S           (4)       /* Pause frame pause threshold shift */
#define EMAC_PFPT_V           (0x3)     /* Pause frame pause threshold max value */
#define EMAC_PPFWAA_E         (BIT(3))  /* Process pause frame when address approved */
#define EMAC_RXFC_E           (BIT(2))  /* Pause RX when receive pause frame */
#define EMAC_TXFC_E           (BIT(1))  /* TX flow control transmit pause frame */
#define EMAC_FCBBA_E          (BIT(0))  /* Start pause frame usage */

/* Register EMAC_DBGR *******************************************************/

#define EMAC_TXFF             (BIT(25)) /* TX FiFo is full */
#define EMAC_TXFNF            (BIT(24)) /* TX FiFo is not full */
#define EMAC_TXFA             (BIT(23)) /* TX FiFo is active */
#define EMAC_TXFS_S           (20)      /* TX FiFo status shift */
#define EMAC_TXFS_V           (0x3)     /* TX FiFo status max value */
#define EMAC_EPS              (BIT(19)) /* MAC enters pause state */
#define EMAC_TFCS_S           (17)      /* Transmit frame control status shift */
#define EMAC_TFCS_V           (0x3)     /* Transmit frame control status max value */
#define EMAC_TXCA             (BIT(16)) /* Transmit control is active */
#define EMAC_RFFL_S           (8)       /* RX FiFi fill level shift */
#define EMAC_RFFL_V           (0x3)     /* RX FiFi fill level max value  */
#define EMAC_RXFS_S           (5)       /* RX FiFo status shift */
#define EMAC_RXFS_V           (0x3)     /* RX FiFo status max value */
#define EMAC_RXFA             (BIT(4))  /* RX FiFo is active */
#define EMAC_FCS_S            (1)       /* FiFo status shift */
#define EMAC_FCS_V            (0x3)     /* FiFo status max value */
#define EMAC_RXCA             (BIT(0))  /* RX control is active */

/* RX DMA description TDES0 register ****************************************/

#define EMAC_RXDMA_OWN        (BIT(31)) /* Own by Hardware */
#define EMAC_RXDMA_DAFF       (BIT(30)) /* Dest address filter Fail */
#define EMAC_RXDMA_FL_S       (16)      /* Received frame length shift */
#define EMAC_RXDMA_FL_V       (0x1fff)  /* Received frame length max value */
#define EMAC_RXDMA_ES         (BIT(15)) /* Error summary */
#define EMAC_RXDMA_DE         (BIT(14)) /* Description error */
#define EMAC_RXDMA_SAFF       (BIT(13)) /* Rsc address filter Fail */
#define EMAC_RXDMA_LE         (BIT(12)) /* Length error */
#define EMAC_RXDMA_OE         (BIT(11)) /* Overflow error */
#define EMAC_RXDMA_VT         (BIT(10)) /* VLAN tag */
#define EMAC_RXDMA_FS         (BIT(9))  /* First segment of frame */
#define EMAC_RXDMA_LS         (BIT(8))  /* Last segment of frame */
#define EMAC_RXDMA_TSA        (BIT(7))  /* Timestamp available */
#define EMAC_RXDMA_RC         (BIT(6))  /* Receive collision */
#define EMAC_RXDMA_FT         (BIT(5))  /* Frame type */
#define EMAC_RXDMA_TO         (BIT(4))  /* Receive timeout */
#define EMAC_RXDMA_RE         (BIT(3))  /* Receive error */
#define EMAC_RXDMA_DBE        (BIT(2))  /* Dribble bit error */
#define EMAC_RXDMA_CE         (BIT(1))  /* CRC error */
#define EMAC_RXDMA_ESA        (BIT(0))  /* Extended status available */

/* RX DMA description TDES1 register ****************************************/

#define EMAC_RXDMA_SRI        (BIT(31)) /* Stop RI interrupt */
#define EMAC_RXDMA_RER        (BIT(15)) /* Receive end of ring */
#define EMAC_RXDMA_RCH        (BIT(14)) /* Second address chained */
#define EMAC_RXDMA_RBS_S      (0)       /* Receive bufer size shift */
#define EMAC_RXDMA_RBS_V      (0x1fff)  /* Receive bufer size max value */

/* TX DMA description TDES0 register ****************************************/

#define EMAC_TXDMA_OWN        (BIT(31)) /* Own by Hardware */
#define EMAC_TXDMA_CI         (BIT(30)) /* Enable complete interrupt */
#define EMAC_TXDMA_LS         (BIT(29)) /* Last segment of frame */
#define EMAC_TXDMA_FS         (BIT(28)) /* First segment of frame */
#define EMAC_TXDMA_DC         (BIT(27)) /* Disable CRC generation */
#define EMAC_TXDMA_DP         (BIT(26)) /* Disable PAD generation */
#define EMAC_TXDMA_ETTS       (BIT(25)) /* Enable transmit timestamp */
#define EMAC_TXDMA_RC         (BIT(24)) /* Replace CRC of frame */
#define EMAC_TXDMA_CCI_S      (22)      /* Control checksum operation shift */
#define EMAC_TXDMA_CCI_V      (0x3)     /* Control checksum operation max value */
#define EMAC_TXDMA_TER        (BIT(21)) /* Transmit end of ring */
#define EMAC_TXDMA_TCH        (BIT(20)) /* Second address chained */
#define EMAC_TXDMA_VIC_S      (18)      /*  Control VLAN insertion shift */
#define EMAC_TXDMA_VIC_V      (0x3)     /*  Control VLAN insertion max value */
#define EMAC_TXDMA_TTSS       (BIT(17)) /* Transmit timestamp status */
#define EMAC_TXDMA_IHE        (BIT(16)) /* IP header error */
#define EMAC_TXDMA_ES         (BIT(15)) /* Error summary */
#define EMAC_TXDMA_JT         (BIT(14)) /* Jabber Timeout */
#define EMAC_TXDMA_FF         (BIT(13)) /* Frame is flushed */
#define EMAC_TXDMA_IPE        (BIT(12)) /* IP payload error */
#define EMAC_TXDMA_LC         (BIT(11)) /* Lose carrier */
#define EMAC_TXDMA_NC         (BIT(10)) /* No carrier */
#define EMAC_TXDMA_TC         (BIT(9))  /* Transmit collision */
#define EMAC_TXDMA_EC         (BIT(8))  /* Excessive collision */
#define EMAC_TXDMA_VF         (BIT(7))  /* VLAN frame */
#define EMAC_TXDMA_CC_S       (4)       /* Collision counter shift */
#define EMAC_TXDMA_CC_V       (0xf)     /* Collision counter max value */
#define EMAC_TXDMA_ED         (BIT(2))  /* Excessive deferral */
#define EMAC_TXDMA_UE         (BIT(1))  /* Underflow Error */
#define EMAC_TXDMA_DB         (BIT(0))  /* Deferred bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* RX DMA description structure */

struct emac_rxdesc_s
{
  /* RX description status */

  uint32_t status;

  /* RX description control */

  uint32_t ctrl;

  /* RX buffer pointer */

  uint8_t *pbuf;

  /* Next RX description */

  struct emac_rxdesc_s *next;

  /* RX description extend status */

  uint32_t ext_status;

  /* RX description reserved data */

  uint32_t reserved;

  /* Receive frame timestamp low */

  uint32_t ts_l;

  /* Receive frame timestamp high */

  uint32_t ts_h;
};

/* TX DMA description structure */

struct emac_txdesc_s
{
  /* TX description control */

  uint32_t ctrl;

  /* RX description extend control */

  uint32_t ext_ctrl;

  /* TX buffer pointer */

  uint8_t *pbuf;

  /* Next TX description */

  struct emac_txdesc_s *next;

  /* RX description reserved data 0 */

  uint32_t reserved0;

  /* RX description reserved data 1 */

  uint32_t reserved2;

  /* Transmit frame timestamp low */

  uint32_t ts_l;

  /* Transmit frame timestamp high */

  uint32_t ts_h;
};

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_EMAC_H */
