/*
 * Copyright (c) 2021-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#ifndef HPM_MCAN_H
#define HPM_MCAN_H

typedef struct {
    __R  uint8_t  RESERVED0[4];                /* 0x0 - 0x3: Reserved */
    __R  uint32_t ENDN;                        /* 0x4: endian register */
    __R  uint8_t  RESERVED1[4];                /* 0x8 - 0xB: Reserved */
    __RW uint32_t DBTP;                        /* 0xC: data bit timing and prescaler, writeable when CCCR.CCE and CCCR.INT are set */
    __RW uint32_t TEST;                        /* 0x10: test register */
    __RW uint32_t RWD;                         /* 0x14: ram watchdog */
    __RW uint32_t CCCR;                        /* 0x18: CC control register */
    __RW uint32_t NBTP;                        /* 0x1C: nominal bit timing and prescaler register */
    __RW uint32_t TSCC;                        /* 0x20: timestamp counter configuration */
    __R  uint32_t TSCV;                        /* 0x24: timestamp counter value */
    __RW uint32_t TOCC;                        /* 0x28: timeout counter configuration */
    __R  uint32_t TOCV;                        /* 0x2C: timeout counter value */
    __R  uint8_t  RESERVED2[16];               /* 0x30 - 0x3F: Reserved */
    __R  uint32_t ECR;                         /* 0x40: error counter register */
    __R  uint32_t PSR;                         /* 0x44: protocol status register */
    __RW uint32_t TDCR;                        /* 0x48: transmitter delay compensation */
    __R  uint8_t  RESERVED3[4];                /* 0x4C - 0x4F: Reserved */
    __RW uint32_t IR;                          /* 0x50: interrupt register */
    __RW uint32_t IE;                          /* 0x54: interrupt enable */
    __RW uint32_t ILS;                         /* 0x58: interrupt line select */
    __RW uint32_t ILE;                         /* 0x5C: interrupt line enable */
    __R  uint8_t  RESERVED4[32];               /* 0x60 - 0x7F: Reserved */
    __RW uint32_t GFC;                         /* 0x80: global filter configuration */
    __RW uint32_t SIDFC;                       /* 0x84: standard ID filter configuration */
    __RW uint32_t XIDFC;                       /* 0x88: extended ID filter configuration */
    __R  uint8_t  RESERVED5[4];                /* 0x8C - 0x8F: Reserved */
    __RW uint32_t XIDAM;                       /* 0x90: extended id and mask */
    __R  uint32_t HPMS;                        /* 0x94: high priority message status */
    __RW uint32_t NDAT1;                       /* 0x98: new data1 */
    __RW uint32_t NDAT2;                       /* 0x9C: new data2 */
    __RW uint32_t RXF0C;                       /* 0xA0: rx fifo 0 configuration */
    __R  uint32_t RXF0S;                       /* 0xA4: rx fifo 0 status */
    __RW uint32_t RXF0A;                       /* 0xA8: rx fifo0 acknowledge */
    __RW uint32_t RXBC;                        /* 0xAC: rx buffer configuration */
    __RW uint32_t RXF1C;                       /* 0xB0: rx fifo1 configuration */
    __R  uint32_t RXF1S;                       /* 0xB4: rx fifo1 status */
    __RW uint32_t RXF1A;                       /* 0xB8: rx fifo 1 acknowledge */
    __RW uint32_t RXESC;                       /* 0xBC: rx buffer/fifo element size configuration */
    __RW uint32_t TXBC;                        /* 0xC0: tx buffer configuration */
    __R  uint32_t TXFQS;                       /* 0xC4: tx fifo/queue status */
    __RW uint32_t TXESC;                       /* 0xC8: tx buffer element size configuration */
    __R  uint32_t TXBRP;                       /* 0xCC: tx buffer request pending */
    __RW uint32_t TXBAR;                       /* 0xD0: tx buffer add request */
    __RW uint32_t TXBCR;                       /* 0xD4: tx buffer cancellation request */
    __R  uint32_t TXBTO;                       /* 0xD8: tx buffer transmission occurred */
    __R  uint32_t TXBCF;                       /* 0xDC: tx buffer cancellation finished */
    __RW uint32_t TXBTIE;                      /* 0xE0: tx buffer transmission interrupt enable */
    __RW uint32_t TXBCIE;                      /* 0xE4: tx buffer cancellation finished interrupt enable */
    __R  uint8_t  RESERVED6[8];                /* 0xE8 - 0xEF: Reserved */
    __RW uint32_t TXEFC;                       /* 0xF0: tx event fifo configuration */
    __R  uint32_t TXEFS;                       /* 0xF4: tx event fifo status */
    __RW uint32_t TXEFA;                       /* 0xF8: tx event fifo acknowledge */
    __R  uint8_t  RESERVED7[260];              /* 0xFC - 0x1FF: Reserved */
    __R  uint32_t TS_SEL[16];                  /* 0x200 - 0x23C: timestamp 0 */
    __R  uint32_t CREL;                        /* 0x240: core release register */
    __RW uint32_t TSCFG;                       /* 0x244: timestamp configuration */
    __R  uint32_t TSS1;                        /* 0x248: timestamp status1 */
    __R  uint32_t TSS2;                        /* 0x24C: timestamp status2 */
    __R  uint32_t ATB;                         /* 0x250: actual timebase */
    __R  uint32_t ATBH;                        /* 0x254: actual timebase high */
    __R  uint8_t  RESERVED8[424];              /* 0x258 - 0x3FF: Reserved */
    __RW uint32_t GLB_CTL;                     /* 0x400: global control */
    __R  uint32_t GLB_STATUS;                  /* 0x404: global status */
    __R  uint32_t GLB_CAN_IR;                  /* 0x408: global m_can_ir */
    __R  uint8_t  RESERVED9[7156];             /* 0x40C - 0x1FFF: Reserved */
    __RW uint32_t MESSAGE_BUFF[640];           /* 0x2000 - 0x29FC: message buff */
} MCAN_Type;


/* Bitfield definition for register: ENDN */
/*
 * EVT (R)
 *
 * evt
 */
#define MCAN_ENDN_EVT_MASK (0xFFFFFFFFUL)
#define MCAN_ENDN_EVT_SHIFT (0U)
#define MCAN_ENDN_EVT_GET(x) (((uint32_t)(x) & MCAN_ENDN_EVT_MASK) >> MCAN_ENDN_EVT_SHIFT)

/* Bitfield definition for register: DBTP */
/*
 * TDC (RW)
 *
 * transmitter delay compensation enable
 */
#define MCAN_DBTP_TDC_MASK (0x800000UL)
#define MCAN_DBTP_TDC_SHIFT (23U)
#define MCAN_DBTP_TDC_SET(x) (((uint32_t)(x) << MCAN_DBTP_TDC_SHIFT) & MCAN_DBTP_TDC_MASK)
#define MCAN_DBTP_TDC_GET(x) (((uint32_t)(x) & MCAN_DBTP_TDC_MASK) >> MCAN_DBTP_TDC_SHIFT)

/*
 * DBRP (RW)
 *
 * data bit rate prescaler, value range is 0-31 when TDC = 0, 0-1 when TDC=1
 */
#define MCAN_DBTP_DBRP_MASK (0x1F0000UL)
#define MCAN_DBTP_DBRP_SHIFT (16U)
#define MCAN_DBTP_DBRP_SET(x) (((uint32_t)(x) << MCAN_DBTP_DBRP_SHIFT) & MCAN_DBTP_DBRP_MASK)
#define MCAN_DBTP_DBRP_GET(x) (((uint32_t)(x) & MCAN_DBTP_DBRP_MASK) >> MCAN_DBTP_DBRP_SHIFT)

/*
 * DTSEG1 (RW)
 *
 * data time segment before sample point, value range 0-31
 */
#define MCAN_DBTP_DTSEG1_MASK (0x1F00U)
#define MCAN_DBTP_DTSEG1_SHIFT (8U)
#define MCAN_DBTP_DTSEG1_SET(x) (((uint32_t)(x) << MCAN_DBTP_DTSEG1_SHIFT) & MCAN_DBTP_DTSEG1_MASK)
#define MCAN_DBTP_DTSEG1_GET(x) (((uint32_t)(x) & MCAN_DBTP_DTSEG1_MASK) >> MCAN_DBTP_DTSEG1_SHIFT)

/*
 * DTSEG2 (RW)
 *
 * data time segment after sample point，value range is 0-15. the data bit time= DTSEG1+DTSEG2+3
 */
#define MCAN_DBTP_DTSEG2_MASK (0xF0U)
#define MCAN_DBTP_DTSEG2_SHIFT (4U)
#define MCAN_DBTP_DTSEG2_SET(x) (((uint32_t)(x) << MCAN_DBTP_DTSEG2_SHIFT) & MCAN_DBTP_DTSEG2_MASK)
#define MCAN_DBTP_DTSEG2_GET(x) (((uint32_t)(x) & MCAN_DBTP_DTSEG2_MASK) >> MCAN_DBTP_DTSEG2_SHIFT)

/*
 * DSJW (RW)
 *
 * data synchronization jump width, value range is 0-15
 */
#define MCAN_DBTP_DSJW_MASK (0xFU)
#define MCAN_DBTP_DSJW_SHIFT (0U)
#define MCAN_DBTP_DSJW_SET(x) (((uint32_t)(x) << MCAN_DBTP_DSJW_SHIFT) & MCAN_DBTP_DSJW_MASK)
#define MCAN_DBTP_DSJW_GET(x) (((uint32_t)(x) & MCAN_DBTP_DSJW_MASK) >> MCAN_DBTP_DSJW_SHIFT)

/* Bitfield definition for register: TEST */
/*
 * SVAL (RW)
 *
 * value of TXBNS valid
 */
#define MCAN_TEST_SVAL_MASK (0x200000UL)
#define MCAN_TEST_SVAL_SHIFT (21U)
#define MCAN_TEST_SVAL_SET(x) (((uint32_t)(x) << MCAN_TEST_SVAL_SHIFT) & MCAN_TEST_SVAL_MASK)
#define MCAN_TEST_SVAL_GET(x) (((uint32_t)(x) & MCAN_TEST_SVAL_MASK) >> MCAN_TEST_SVAL_SHIFT)

/*
 * TXBNS (RW)
 *
 * tx buffer number of message whose transmission was started last
 */
#define MCAN_TEST_TXBNS_MASK (0x1F0000UL)
#define MCAN_TEST_TXBNS_SHIFT (16U)
#define MCAN_TEST_TXBNS_SET(x) (((uint32_t)(x) << MCAN_TEST_TXBNS_SHIFT) & MCAN_TEST_TXBNS_MASK)
#define MCAN_TEST_TXBNS_GET(x) (((uint32_t)(x) & MCAN_TEST_TXBNS_MASK) >> MCAN_TEST_TXBNS_SHIFT)

/*
 * PVAL (RW)
 *
 * value of TXBNP valid
 */
#define MCAN_TEST_PVAL_MASK (0x2000U)
#define MCAN_TEST_PVAL_SHIFT (13U)
#define MCAN_TEST_PVAL_SET(x) (((uint32_t)(x) << MCAN_TEST_PVAL_SHIFT) & MCAN_TEST_PVAL_MASK)
#define MCAN_TEST_PVAL_GET(x) (((uint32_t)(x) & MCAN_TEST_PVAL_MASK) >> MCAN_TEST_PVAL_SHIFT)

/*
 * TXBNP (RW)
 *
 * tx buffer number of message that is ready for transmission
 */
#define MCAN_TEST_TXBNP_MASK (0x1F00U)
#define MCAN_TEST_TXBNP_SHIFT (8U)
#define MCAN_TEST_TXBNP_SET(x) (((uint32_t)(x) << MCAN_TEST_TXBNP_SHIFT) & MCAN_TEST_TXBNP_MASK)
#define MCAN_TEST_TXBNP_GET(x) (((uint32_t)(x) & MCAN_TEST_TXBNP_MASK) >> MCAN_TEST_TXBNP_SHIFT)

/*
 * RX (RW)
 *
 * the actual value of pin m_can_rx, 0= dominant  1=recessive
 */
#define MCAN_TEST_RX_MASK (0x80U)
#define MCAN_TEST_RX_SHIFT (7U)
#define MCAN_TEST_RX_SET(x) (((uint32_t)(x) << MCAN_TEST_RX_SHIFT) & MCAN_TEST_RX_MASK)
#define MCAN_TEST_RX_GET(x) (((uint32_t)(x) & MCAN_TEST_RX_MASK) >> MCAN_TEST_RX_SHIFT)

/*
 * TX (RW)
 *
 * control of transmit pin, 00- m_can_tx controlled by can core, 01- sample point can be monitored  at m_can_tx, 10- dominant level at pin m_can_tx, 11- recessive at pin m_can_tx
 */
#define MCAN_TEST_TX_MASK (0x60U)
#define MCAN_TEST_TX_SHIFT (5U)
#define MCAN_TEST_TX_SET(x) (((uint32_t)(x) << MCAN_TEST_TX_SHIFT) & MCAN_TEST_TX_MASK)
#define MCAN_TEST_TX_GET(x) (((uint32_t)(x) & MCAN_TEST_TX_MASK) >> MCAN_TEST_TX_SHIFT)

/*
 * LBCK (RW)
 *
 * loopback mode enable
 */
#define MCAN_TEST_LBCK_MASK (0x10U)
#define MCAN_TEST_LBCK_SHIFT (4U)
#define MCAN_TEST_LBCK_SET(x) (((uint32_t)(x) << MCAN_TEST_LBCK_SHIFT) & MCAN_TEST_LBCK_MASK)
#define MCAN_TEST_LBCK_GET(x) (((uint32_t)(x) & MCAN_TEST_LBCK_MASK) >> MCAN_TEST_LBCK_SHIFT)

/* Bitfield definition for register: RWD */
/*
 * WDV (R)
 *
 * actual message counter value
 */
#define MCAN_RWD_WDV_MASK (0xFF00U)
#define MCAN_RWD_WDV_SHIFT (8U)
#define MCAN_RWD_WDV_GET(x) (((uint32_t)(x) & MCAN_RWD_WDV_MASK) >> MCAN_RWD_WDV_SHIFT)

/*
 * WDC (RW)
 *
 * start value of counter, the counter is disabled when 0 is configured
 */
#define MCAN_RWD_WDC_MASK (0xFFU)
#define MCAN_RWD_WDC_SHIFT (0U)
#define MCAN_RWD_WDC_SET(x) (((uint32_t)(x) << MCAN_RWD_WDC_SHIFT) & MCAN_RWD_WDC_MASK)
#define MCAN_RWD_WDC_GET(x) (((uint32_t)(x) & MCAN_RWD_WDC_MASK) >> MCAN_RWD_WDC_SHIFT)

/* Bitfield definition for register: CCCR */
/*
 * NISO (RW)
 *
 * non iso operation, 0-ISO 11898-1:2015 frame formate, 1-Bosch can fd specification v1.0
 */
#define MCAN_CCCR_NISO_MASK (0x8000U)
#define MCAN_CCCR_NISO_SHIFT (15U)
#define MCAN_CCCR_NISO_SET(x) (((uint32_t)(x) << MCAN_CCCR_NISO_SHIFT) & MCAN_CCCR_NISO_MASK)
#define MCAN_CCCR_NISO_GET(x) (((uint32_t)(x) & MCAN_CCCR_NISO_MASK) >> MCAN_CCCR_NISO_SHIFT)

/*
 * TXP (RW)
 *
 * transmit pause, pause two bit times between frame
 */
#define MCAN_CCCR_TXP_MASK (0x4000U)
#define MCAN_CCCR_TXP_SHIFT (14U)
#define MCAN_CCCR_TXP_SET(x) (((uint32_t)(x) << MCAN_CCCR_TXP_SHIFT) & MCAN_CCCR_TXP_MASK)
#define MCAN_CCCR_TXP_GET(x) (((uint32_t)(x) & MCAN_CCCR_TXP_MASK) >> MCAN_CCCR_TXP_SHIFT)

/*
 * EFBI (RW)
 *
 * edge filtering during bus integration, 1- two consecutive dominant tq required to detect an edge for hard synchronization
 */
#define MCAN_CCCR_EFBI_MASK (0x2000U)
#define MCAN_CCCR_EFBI_SHIFT (13U)
#define MCAN_CCCR_EFBI_SET(x) (((uint32_t)(x) << MCAN_CCCR_EFBI_SHIFT) & MCAN_CCCR_EFBI_MASK)
#define MCAN_CCCR_EFBI_GET(x) (((uint32_t)(x) & MCAN_CCCR_EFBI_MASK) >> MCAN_CCCR_EFBI_SHIFT)

/*
 * PXHD (RW)
 *
 * protocal exception handing disable
 */
#define MCAN_CCCR_PXHD_MASK (0x1000U)
#define MCAN_CCCR_PXHD_SHIFT (12U)
#define MCAN_CCCR_PXHD_SET(x) (((uint32_t)(x) << MCAN_CCCR_PXHD_SHIFT) & MCAN_CCCR_PXHD_MASK)
#define MCAN_CCCR_PXHD_GET(x) (((uint32_t)(x) & MCAN_CCCR_PXHD_MASK) >> MCAN_CCCR_PXHD_SHIFT)

/*
 * WMM (RW)
 *
 * wide message marker, 1-16bit message marker
 */
#define MCAN_CCCR_WMM_MASK (0x800U)
#define MCAN_CCCR_WMM_SHIFT (11U)
#define MCAN_CCCR_WMM_SET(x) (((uint32_t)(x) << MCAN_CCCR_WMM_SHIFT) & MCAN_CCCR_WMM_MASK)
#define MCAN_CCCR_WMM_GET(x) (((uint32_t)(x) & MCAN_CCCR_WMM_MASK) >> MCAN_CCCR_WMM_SHIFT)

/*
 * UTSU (RW)
 *
 * use timestamping unit, 0-internal time stamping, 1-external time stamping by TSU
 */
#define MCAN_CCCR_UTSU_MASK (0x400U)
#define MCAN_CCCR_UTSU_SHIFT (10U)
#define MCAN_CCCR_UTSU_SET(x) (((uint32_t)(x) << MCAN_CCCR_UTSU_SHIFT) & MCAN_CCCR_UTSU_MASK)
#define MCAN_CCCR_UTSU_GET(x) (((uint32_t)(x) & MCAN_CCCR_UTSU_MASK) >> MCAN_CCCR_UTSU_SHIFT)

/*
 * BRSE (RW)
 *
 * bit rate switch enable, different bit-rate can be supply to data segement
 */
#define MCAN_CCCR_BRSE_MASK (0x200U)
#define MCAN_CCCR_BRSE_SHIFT (9U)
#define MCAN_CCCR_BRSE_SET(x) (((uint32_t)(x) << MCAN_CCCR_BRSE_SHIFT) & MCAN_CCCR_BRSE_MASK)
#define MCAN_CCCR_BRSE_GET(x) (((uint32_t)(x) & MCAN_CCCR_BRSE_MASK) >> MCAN_CCCR_BRSE_SHIFT)

/*
 * FDOE (RW)
 *
 * 1-FD operation enable
 */
#define MCAN_CCCR_FDOE_MASK (0x100U)
#define MCAN_CCCR_FDOE_SHIFT (8U)
#define MCAN_CCCR_FDOE_SET(x) (((uint32_t)(x) << MCAN_CCCR_FDOE_SHIFT) & MCAN_CCCR_FDOE_MASK)
#define MCAN_CCCR_FDOE_GET(x) (((uint32_t)(x) & MCAN_CCCR_FDOE_MASK) >> MCAN_CCCR_FDOE_SHIFT)

/*
 * TEST (RW)
 *
 * 1- test mode, wirte access to register TEST enable
 */
#define MCAN_CCCR_TEST_MASK (0x80U)
#define MCAN_CCCR_TEST_SHIFT (7U)
#define MCAN_CCCR_TEST_SET(x) (((uint32_t)(x) << MCAN_CCCR_TEST_SHIFT) & MCAN_CCCR_TEST_MASK)
#define MCAN_CCCR_TEST_GET(x) (((uint32_t)(x) & MCAN_CCCR_TEST_MASK) >> MCAN_CCCR_TEST_SHIFT)

/*
 * DAR (RW)
 *
 * 1- automatic retransmission disabled
 */
#define MCAN_CCCR_DAR_MASK (0x40U)
#define MCAN_CCCR_DAR_SHIFT (6U)
#define MCAN_CCCR_DAR_SET(x) (((uint32_t)(x) << MCAN_CCCR_DAR_SHIFT) & MCAN_CCCR_DAR_MASK)
#define MCAN_CCCR_DAR_GET(x) (((uint32_t)(x) & MCAN_CCCR_DAR_MASK) >> MCAN_CCCR_DAR_SHIFT)

/*
 * MON (RW)
 *
 * 1- bus monitoring mode enable
 */
#define MCAN_CCCR_MON_MASK (0x20U)
#define MCAN_CCCR_MON_SHIFT (5U)
#define MCAN_CCCR_MON_SET(x) (((uint32_t)(x) << MCAN_CCCR_MON_SHIFT) & MCAN_CCCR_MON_MASK)
#define MCAN_CCCR_MON_GET(x) (((uint32_t)(x) & MCAN_CCCR_MON_MASK) >> MCAN_CCCR_MON_SHIFT)

/*
 * CSR (RW)
 *
 * 1- clock stop request
 */
#define MCAN_CCCR_CSR_MASK (0x10U)
#define MCAN_CCCR_CSR_SHIFT (4U)
#define MCAN_CCCR_CSR_SET(x) (((uint32_t)(x) << MCAN_CCCR_CSR_SHIFT) & MCAN_CCCR_CSR_MASK)
#define MCAN_CCCR_CSR_GET(x) (((uint32_t)(x) & MCAN_CCCR_CSR_MASK) >> MCAN_CCCR_CSR_SHIFT)

/*
 * CSA (R)
 *
 * 1- M_CAN may be set in power down by stopping m_can_hclk and m_can_cclk
 */
#define MCAN_CCCR_CSA_MASK (0x8U)
#define MCAN_CCCR_CSA_SHIFT (3U)
#define MCAN_CCCR_CSA_GET(x) (((uint32_t)(x) & MCAN_CCCR_CSA_MASK) >> MCAN_CCCR_CSA_SHIFT)

/*
 * ASM (RW)
 *
 * 1- restricted operation mode active, able to receive frames and give acknoledge, but can not send data frames remote frames active error frames and overload frames.
 */
#define MCAN_CCCR_ASM_MASK (0x4U)
#define MCAN_CCCR_ASM_SHIFT (2U)
#define MCAN_CCCR_ASM_SET(x) (((uint32_t)(x) << MCAN_CCCR_ASM_SHIFT) & MCAN_CCCR_ASM_MASK)
#define MCAN_CCCR_ASM_GET(x) (((uint32_t)(x) & MCAN_CCCR_ASM_MASK) >> MCAN_CCCR_ASM_SHIFT)

/*
 * CCE (RW)
 *
 * 1- The CPU has write access to the protected configuration registers
 */
#define MCAN_CCCR_CCE_MASK (0x2U)
#define MCAN_CCCR_CCE_SHIFT (1U)
#define MCAN_CCCR_CCE_SET(x) (((uint32_t)(x) << MCAN_CCCR_CCE_SHIFT) & MCAN_CCCR_CCE_MASK)
#define MCAN_CCCR_CCE_GET(x) (((uint32_t)(x) & MCAN_CCCR_CCE_MASK) >> MCAN_CCCR_CCE_SHIFT)

/*
 * INIT (RW)
 *
 * 1- initialization is started
 */
#define MCAN_CCCR_INIT_MASK (0x1U)
#define MCAN_CCCR_INIT_SHIFT (0U)
#define MCAN_CCCR_INIT_SET(x) (((uint32_t)(x) << MCAN_CCCR_INIT_SHIFT) & MCAN_CCCR_INIT_MASK)
#define MCAN_CCCR_INIT_GET(x) (((uint32_t)(x) & MCAN_CCCR_INIT_MASK) >> MCAN_CCCR_INIT_SHIFT)

/* Bitfield definition for register: NBTP */
/*
 * NSJW (RW)
 *
 * Nnominal synchronization jump width
 */
#define MCAN_NBTP_NSJW_MASK (0xFE000000UL)
#define MCAN_NBTP_NSJW_SHIFT (25U)
#define MCAN_NBTP_NSJW_SET(x) (((uint32_t)(x) << MCAN_NBTP_NSJW_SHIFT) & MCAN_NBTP_NSJW_MASK)
#define MCAN_NBTP_NSJW_GET(x) (((uint32_t)(x) & MCAN_NBTP_NSJW_MASK) >> MCAN_NBTP_NSJW_SHIFT)

/*
 * NBRP (RW)
 *
 * nominal bit rate prescaler
 */
#define MCAN_NBTP_NBRP_MASK (0x1FF0000UL)
#define MCAN_NBTP_NBRP_SHIFT (16U)
#define MCAN_NBTP_NBRP_SET(x) (((uint32_t)(x) << MCAN_NBTP_NBRP_SHIFT) & MCAN_NBTP_NBRP_MASK)
#define MCAN_NBTP_NBRP_GET(x) (((uint32_t)(x) & MCAN_NBTP_NBRP_MASK) >> MCAN_NBTP_NBRP_SHIFT)

/*
 * NTSEG1 (RW)
 *
 * nominal time segement before sample point
 */
#define MCAN_NBTP_NTSEG1_MASK (0xFF00U)
#define MCAN_NBTP_NTSEG1_SHIFT (8U)
#define MCAN_NBTP_NTSEG1_SET(x) (((uint32_t)(x) << MCAN_NBTP_NTSEG1_SHIFT) & MCAN_NBTP_NTSEG1_MASK)
#define MCAN_NBTP_NTSEG1_GET(x) (((uint32_t)(x) & MCAN_NBTP_NTSEG1_MASK) >> MCAN_NBTP_NTSEG1_SHIFT)

/*
 * NTSEG2 (RW)
 *
 * nominal time segement after sample point
 */
#define MCAN_NBTP_NTSEG2_MASK (0x7FU)
#define MCAN_NBTP_NTSEG2_SHIFT (0U)
#define MCAN_NBTP_NTSEG2_SET(x) (((uint32_t)(x) << MCAN_NBTP_NTSEG2_SHIFT) & MCAN_NBTP_NTSEG2_MASK)
#define MCAN_NBTP_NTSEG2_GET(x) (((uint32_t)(x) & MCAN_NBTP_NTSEG2_MASK) >> MCAN_NBTP_NTSEG2_SHIFT)

/* Bitfield definition for register: TSCC */
/*
 * TCP (RW)
 *
 * timestamp counter configuration
 */
#define MCAN_TSCC_TCP_MASK (0xF0000UL)
#define MCAN_TSCC_TCP_SHIFT (16U)
#define MCAN_TSCC_TCP_SET(x) (((uint32_t)(x) << MCAN_TSCC_TCP_SHIFT) & MCAN_TSCC_TCP_MASK)
#define MCAN_TSCC_TCP_GET(x) (((uint32_t)(x) & MCAN_TSCC_TCP_MASK) >> MCAN_TSCC_TCP_SHIFT)

/*
 * TSS (RW)
 *
 * timestamp select
 */
#define MCAN_TSCC_TSS_MASK (0x3U)
#define MCAN_TSCC_TSS_SHIFT (0U)
#define MCAN_TSCC_TSS_SET(x) (((uint32_t)(x) << MCAN_TSCC_TSS_SHIFT) & MCAN_TSCC_TSS_MASK)
#define MCAN_TSCC_TSS_GET(x) (((uint32_t)(x) & MCAN_TSCC_TSS_MASK) >> MCAN_TSCC_TSS_SHIFT)

/* Bitfield definition for register: TSCV */
/*
 * TSC (RC)
 *
 * timestamp counter
 */
#define MCAN_TSCV_TSC_MASK (0xFFFFU)
#define MCAN_TSCV_TSC_SHIFT (0U)
#define MCAN_TSCV_TSC_GET(x) (((uint32_t)(x) & MCAN_TSCV_TSC_MASK) >> MCAN_TSCV_TSC_SHIFT)

/* Bitfield definition for register: TOCC */
/*
 * TOP (RW)
 *
 * timeout period
 */
#define MCAN_TOCC_TOP_MASK (0xFFFF0000UL)
#define MCAN_TOCC_TOP_SHIFT (16U)
#define MCAN_TOCC_TOP_SET(x) (((uint32_t)(x) << MCAN_TOCC_TOP_SHIFT) & MCAN_TOCC_TOP_MASK)
#define MCAN_TOCC_TOP_GET(x) (((uint32_t)(x) & MCAN_TOCC_TOP_MASK) >> MCAN_TOCC_TOP_SHIFT)

/*
 * TOS (RW)
 *
 * timeout select
 */
#define MCAN_TOCC_TOS_MASK (0x6U)
#define MCAN_TOCC_TOS_SHIFT (1U)
#define MCAN_TOCC_TOS_SET(x) (((uint32_t)(x) << MCAN_TOCC_TOS_SHIFT) & MCAN_TOCC_TOS_MASK)
#define MCAN_TOCC_TOS_GET(x) (((uint32_t)(x) & MCAN_TOCC_TOS_MASK) >> MCAN_TOCC_TOS_SHIFT)

/*
 * RP (RW)
 *
 * enable timeout counter
 */
#define MCAN_TOCC_RP_MASK (0x1U)
#define MCAN_TOCC_RP_SHIFT (0U)
#define MCAN_TOCC_RP_SET(x) (((uint32_t)(x) << MCAN_TOCC_RP_SHIFT) & MCAN_TOCC_RP_MASK)
#define MCAN_TOCC_RP_GET(x) (((uint32_t)(x) & MCAN_TOCC_RP_MASK) >> MCAN_TOCC_RP_SHIFT)

/* Bitfield definition for register: TOCV */
/*
 * TOC (RC)
 *
 * timeout counter value
 */
#define MCAN_TOCV_TOC_MASK (0xFFFFU)
#define MCAN_TOCV_TOC_SHIFT (0U)
#define MCAN_TOCV_TOC_GET(x) (((uint32_t)(x) & MCAN_TOCV_TOC_MASK) >> MCAN_TOCV_TOC_SHIFT)

/* Bitfield definition for register: ECR */
/*
 * CEL (R)
 *
 * can error logging
 */
#define MCAN_ECR_CEL_MASK (0xFF0000UL)
#define MCAN_ECR_CEL_SHIFT (16U)
#define MCAN_ECR_CEL_GET(x) (((uint32_t)(x) & MCAN_ECR_CEL_MASK) >> MCAN_ECR_CEL_SHIFT)

/*
 * RP (R)
 *
 * receive error passive
 */
#define MCAN_ECR_RP_MASK (0x8000U)
#define MCAN_ECR_RP_SHIFT (15U)
#define MCAN_ECR_RP_GET(x) (((uint32_t)(x) & MCAN_ECR_RP_MASK) >> MCAN_ECR_RP_SHIFT)

/*
 * REC (R)
 *
 * receive error counter
 */
#define MCAN_ECR_REC_MASK (0x7F00U)
#define MCAN_ECR_REC_SHIFT (8U)
#define MCAN_ECR_REC_GET(x) (((uint32_t)(x) & MCAN_ECR_REC_MASK) >> MCAN_ECR_REC_SHIFT)

/*
 * TEC (R)
 *
 * transmit error counter
 */
#define MCAN_ECR_TEC_MASK (0xFFU)
#define MCAN_ECR_TEC_SHIFT (0U)
#define MCAN_ECR_TEC_GET(x) (((uint32_t)(x) & MCAN_ECR_TEC_MASK) >> MCAN_ECR_TEC_SHIFT)

/* Bitfield definition for register: PSR */
/*
 * TDCV (R)
 *
 * transmitter delay compensation value
 */
#define MCAN_PSR_TDCV_MASK (0x7F0000UL)
#define MCAN_PSR_TDCV_SHIFT (16U)
#define MCAN_PSR_TDCV_GET(x) (((uint32_t)(x) & MCAN_PSR_TDCV_MASK) >> MCAN_PSR_TDCV_SHIFT)

/*
 * PXE (R)
 *
 * protocol exception event
 */
#define MCAN_PSR_PXE_MASK (0x4000U)
#define MCAN_PSR_PXE_SHIFT (14U)
#define MCAN_PSR_PXE_GET(x) (((uint32_t)(x) & MCAN_PSR_PXE_MASK) >> MCAN_PSR_PXE_SHIFT)

/*
 * RFDF (R)
 *
 * received a can fd message
 */
#define MCAN_PSR_RFDF_MASK (0x2000U)
#define MCAN_PSR_RFDF_SHIFT (13U)
#define MCAN_PSR_RFDF_GET(x) (((uint32_t)(x) & MCAN_PSR_RFDF_MASK) >> MCAN_PSR_RFDF_SHIFT)

/*
 * RBRS (R)
 *
 * brs flag of last received can fd message
 */
#define MCAN_PSR_RBRS_MASK (0x1000U)
#define MCAN_PSR_RBRS_SHIFT (12U)
#define MCAN_PSR_RBRS_GET(x) (((uint32_t)(x) & MCAN_PSR_RBRS_MASK) >> MCAN_PSR_RBRS_SHIFT)

/*
 * RESI (R)
 *
 * ESI flag of last received can fd message
 */
#define MCAN_PSR_RESI_MASK (0x800U)
#define MCAN_PSR_RESI_SHIFT (11U)
#define MCAN_PSR_RESI_GET(x) (((uint32_t)(x) & MCAN_PSR_RESI_MASK) >> MCAN_PSR_RESI_SHIFT)

/*
 * DLEC (R)
 *
 * data phase last error code
 */
#define MCAN_PSR_DLEC_MASK (0x700U)
#define MCAN_PSR_DLEC_SHIFT (8U)
#define MCAN_PSR_DLEC_GET(x) (((uint32_t)(x) & MCAN_PSR_DLEC_MASK) >> MCAN_PSR_DLEC_SHIFT)

/*
 * BO (R)
 *
 * bus_off state
 */
#define MCAN_PSR_BO_MASK (0x80U)
#define MCAN_PSR_BO_SHIFT (7U)
#define MCAN_PSR_BO_GET(x) (((uint32_t)(x) & MCAN_PSR_BO_MASK) >> MCAN_PSR_BO_SHIFT)

/*
 * EW (R)
 *
 * warning status
 */
#define MCAN_PSR_EW_MASK (0x40U)
#define MCAN_PSR_EW_SHIFT (6U)
#define MCAN_PSR_EW_GET(x) (((uint32_t)(x) & MCAN_PSR_EW_MASK) >> MCAN_PSR_EW_SHIFT)

/*
 * EP (R)
 *
 * error passive
 */
#define MCAN_PSR_EP_MASK (0x20U)
#define MCAN_PSR_EP_SHIFT (5U)
#define MCAN_PSR_EP_GET(x) (((uint32_t)(x) & MCAN_PSR_EP_MASK) >> MCAN_PSR_EP_SHIFT)

/*
 * ACT (R)
 *
 * activity
 */
#define MCAN_PSR_ACT_MASK (0x18U)
#define MCAN_PSR_ACT_SHIFT (3U)
#define MCAN_PSR_ACT_GET(x) (((uint32_t)(x) & MCAN_PSR_ACT_MASK) >> MCAN_PSR_ACT_SHIFT)

/*
 * LEC (R)
 *
 * last error code
 */
#define MCAN_PSR_LEC_MASK (0x7U)
#define MCAN_PSR_LEC_SHIFT (0U)
#define MCAN_PSR_LEC_GET(x) (((uint32_t)(x) & MCAN_PSR_LEC_MASK) >> MCAN_PSR_LEC_SHIFT)

/* Bitfield definition for register: TDCR */
/*
 * TDCO (RW)
 *
 * transmitter delay compensation ssp offset
 */
#define MCAN_TDCR_TDCO_MASK (0x7F00U)
#define MCAN_TDCR_TDCO_SHIFT (8U)
#define MCAN_TDCR_TDCO_SET(x) (((uint32_t)(x) << MCAN_TDCR_TDCO_SHIFT) & MCAN_TDCR_TDCO_MASK)
#define MCAN_TDCR_TDCO_GET(x) (((uint32_t)(x) & MCAN_TDCR_TDCO_MASK) >> MCAN_TDCR_TDCO_SHIFT)

/*
 * TDCF (RW)
 *
 * transmitter delay compensation filter window length
 */
#define MCAN_TDCR_TDCF_MASK (0x7FU)
#define MCAN_TDCR_TDCF_SHIFT (0U)
#define MCAN_TDCR_TDCF_SET(x) (((uint32_t)(x) << MCAN_TDCR_TDCF_SHIFT) & MCAN_TDCR_TDCF_MASK)
#define MCAN_TDCR_TDCF_GET(x) (((uint32_t)(x) & MCAN_TDCR_TDCF_MASK) >> MCAN_TDCR_TDCF_SHIFT)

/* Bitfield definition for register: IR */
/*
 * ARA (RW)
 *
 * access to reserved address
 */
#define MCAN_IR_ARA_MASK (0x20000000UL)
#define MCAN_IR_ARA_SHIFT (29U)
#define MCAN_IR_ARA_SET(x) (((uint32_t)(x) << MCAN_IR_ARA_SHIFT) & MCAN_IR_ARA_MASK)
#define MCAN_IR_ARA_GET(x) (((uint32_t)(x) & MCAN_IR_ARA_MASK) >> MCAN_IR_ARA_SHIFT)

/*
 * PED (RW)
 *
 * protocol error in data phase
 */
#define MCAN_IR_PED_MASK (0x10000000UL)
#define MCAN_IR_PED_SHIFT (28U)
#define MCAN_IR_PED_SET(x) (((uint32_t)(x) << MCAN_IR_PED_SHIFT) & MCAN_IR_PED_MASK)
#define MCAN_IR_PED_GET(x) (((uint32_t)(x) & MCAN_IR_PED_MASK) >> MCAN_IR_PED_SHIFT)

/*
 * PEA (RW)
 *
 * protocol error in arbitration phase
 */
#define MCAN_IR_PEA_MASK (0x8000000UL)
#define MCAN_IR_PEA_SHIFT (27U)
#define MCAN_IR_PEA_SET(x) (((uint32_t)(x) << MCAN_IR_PEA_SHIFT) & MCAN_IR_PEA_MASK)
#define MCAN_IR_PEA_GET(x) (((uint32_t)(x) & MCAN_IR_PEA_MASK) >> MCAN_IR_PEA_SHIFT)

/*
 * WDI (RW)
 *
 * watchdog interrupt
 */
#define MCAN_IR_WDI_MASK (0x4000000UL)
#define MCAN_IR_WDI_SHIFT (26U)
#define MCAN_IR_WDI_SET(x) (((uint32_t)(x) << MCAN_IR_WDI_SHIFT) & MCAN_IR_WDI_MASK)
#define MCAN_IR_WDI_GET(x) (((uint32_t)(x) & MCAN_IR_WDI_MASK) >> MCAN_IR_WDI_SHIFT)

/*
 * BO (RW)
 *
 * bus_off status
 */
#define MCAN_IR_BO_MASK (0x2000000UL)
#define MCAN_IR_BO_SHIFT (25U)
#define MCAN_IR_BO_SET(x) (((uint32_t)(x) << MCAN_IR_BO_SHIFT) & MCAN_IR_BO_MASK)
#define MCAN_IR_BO_GET(x) (((uint32_t)(x) & MCAN_IR_BO_MASK) >> MCAN_IR_BO_SHIFT)

/*
 * EW (RW)
 *
 * warning status
 */
#define MCAN_IR_EW_MASK (0x1000000UL)
#define MCAN_IR_EW_SHIFT (24U)
#define MCAN_IR_EW_SET(x) (((uint32_t)(x) << MCAN_IR_EW_SHIFT) & MCAN_IR_EW_MASK)
#define MCAN_IR_EW_GET(x) (((uint32_t)(x) & MCAN_IR_EW_MASK) >> MCAN_IR_EW_SHIFT)

/*
 * EP (RW)
 *
 * error passive
 */
#define MCAN_IR_EP_MASK (0x800000UL)
#define MCAN_IR_EP_SHIFT (23U)
#define MCAN_IR_EP_SET(x) (((uint32_t)(x) << MCAN_IR_EP_SHIFT) & MCAN_IR_EP_MASK)
#define MCAN_IR_EP_GET(x) (((uint32_t)(x) & MCAN_IR_EP_MASK) >> MCAN_IR_EP_SHIFT)

/*
 * ELO (RW)
 *
 * error logging overflow
 */
#define MCAN_IR_ELO_MASK (0x400000UL)
#define MCAN_IR_ELO_SHIFT (22U)
#define MCAN_IR_ELO_SET(x) (((uint32_t)(x) << MCAN_IR_ELO_SHIFT) & MCAN_IR_ELO_MASK)
#define MCAN_IR_ELO_GET(x) (((uint32_t)(x) & MCAN_IR_ELO_MASK) >> MCAN_IR_ELO_SHIFT)

/*
 * BEU (RW)
 *
 * bit error uncorrected
 */
#define MCAN_IR_BEU_MASK (0x200000UL)
#define MCAN_IR_BEU_SHIFT (21U)
#define MCAN_IR_BEU_SET(x) (((uint32_t)(x) << MCAN_IR_BEU_SHIFT) & MCAN_IR_BEU_MASK)
#define MCAN_IR_BEU_GET(x) (((uint32_t)(x) & MCAN_IR_BEU_MASK) >> MCAN_IR_BEU_SHIFT)

/*
 * BEC (RW)
 *
 * bit error corrected
 */
#define MCAN_IR_BEC_MASK (0x100000UL)
#define MCAN_IR_BEC_SHIFT (20U)
#define MCAN_IR_BEC_SET(x) (((uint32_t)(x) << MCAN_IR_BEC_SHIFT) & MCAN_IR_BEC_MASK)
#define MCAN_IR_BEC_GET(x) (((uint32_t)(x) & MCAN_IR_BEC_MASK) >> MCAN_IR_BEC_SHIFT)

/*
 * DRX (RW)
 *
 * message stored to dedicated rx buffer
 */
#define MCAN_IR_DRX_MASK (0x80000UL)
#define MCAN_IR_DRX_SHIFT (19U)
#define MCAN_IR_DRX_SET(x) (((uint32_t)(x) << MCAN_IR_DRX_SHIFT) & MCAN_IR_DRX_MASK)
#define MCAN_IR_DRX_GET(x) (((uint32_t)(x) & MCAN_IR_DRX_MASK) >> MCAN_IR_DRX_SHIFT)

/*
 * TOO (RW)
 *
 * timeout occurred
 */
#define MCAN_IR_TOO_MASK (0x40000UL)
#define MCAN_IR_TOO_SHIFT (18U)
#define MCAN_IR_TOO_SET(x) (((uint32_t)(x) << MCAN_IR_TOO_SHIFT) & MCAN_IR_TOO_MASK)
#define MCAN_IR_TOO_GET(x) (((uint32_t)(x) & MCAN_IR_TOO_MASK) >> MCAN_IR_TOO_SHIFT)

/*
 * MRAF (RW)
 *
 * message RAM access failure
 */
#define MCAN_IR_MRAF_MASK (0x20000UL)
#define MCAN_IR_MRAF_SHIFT (17U)
#define MCAN_IR_MRAF_SET(x) (((uint32_t)(x) << MCAN_IR_MRAF_SHIFT) & MCAN_IR_MRAF_MASK)
#define MCAN_IR_MRAF_GET(x) (((uint32_t)(x) & MCAN_IR_MRAF_MASK) >> MCAN_IR_MRAF_SHIFT)

/*
 * TSW (RW)
 *
 * timestap wraparound
 */
#define MCAN_IR_TSW_MASK (0x10000UL)
#define MCAN_IR_TSW_SHIFT (16U)
#define MCAN_IR_TSW_SET(x) (((uint32_t)(x) << MCAN_IR_TSW_SHIFT) & MCAN_IR_TSW_MASK)
#define MCAN_IR_TSW_GET(x) (((uint32_t)(x) & MCAN_IR_TSW_MASK) >> MCAN_IR_TSW_SHIFT)

/*
 * TEFL (RW)
 *
 * tx event fifo element lost
 */
#define MCAN_IR_TEFL_MASK (0x8000U)
#define MCAN_IR_TEFL_SHIFT (15U)
#define MCAN_IR_TEFL_SET(x) (((uint32_t)(x) << MCAN_IR_TEFL_SHIFT) & MCAN_IR_TEFL_MASK)
#define MCAN_IR_TEFL_GET(x) (((uint32_t)(x) & MCAN_IR_TEFL_MASK) >> MCAN_IR_TEFL_SHIFT)

/*
 * TEFF (RW)
 *
 * tx event fifo full
 */
#define MCAN_IR_TEFF_MASK (0x4000U)
#define MCAN_IR_TEFF_SHIFT (14U)
#define MCAN_IR_TEFF_SET(x) (((uint32_t)(x) << MCAN_IR_TEFF_SHIFT) & MCAN_IR_TEFF_MASK)
#define MCAN_IR_TEFF_GET(x) (((uint32_t)(x) & MCAN_IR_TEFF_MASK) >> MCAN_IR_TEFF_SHIFT)

/*
 * TEFW (RW)
 *
 * tx event fifo watermark reached
 */
#define MCAN_IR_TEFW_MASK (0x2000U)
#define MCAN_IR_TEFW_SHIFT (13U)
#define MCAN_IR_TEFW_SET(x) (((uint32_t)(x) << MCAN_IR_TEFW_SHIFT) & MCAN_IR_TEFW_MASK)
#define MCAN_IR_TEFW_GET(x) (((uint32_t)(x) & MCAN_IR_TEFW_MASK) >> MCAN_IR_TEFW_SHIFT)

/*
 * TEFN (RW)
 *
 * tx event fifo new entry
 */
#define MCAN_IR_TEFN_MASK (0x1000U)
#define MCAN_IR_TEFN_SHIFT (12U)
#define MCAN_IR_TEFN_SET(x) (((uint32_t)(x) << MCAN_IR_TEFN_SHIFT) & MCAN_IR_TEFN_MASK)
#define MCAN_IR_TEFN_GET(x) (((uint32_t)(x) & MCAN_IR_TEFN_MASK) >> MCAN_IR_TEFN_SHIFT)

/*
 * TFE (RW)
 *
 * tx fifo empty
 */
#define MCAN_IR_TFE_MASK (0x800U)
#define MCAN_IR_TFE_SHIFT (11U)
#define MCAN_IR_TFE_SET(x) (((uint32_t)(x) << MCAN_IR_TFE_SHIFT) & MCAN_IR_TFE_MASK)
#define MCAN_IR_TFE_GET(x) (((uint32_t)(x) & MCAN_IR_TFE_MASK) >> MCAN_IR_TFE_SHIFT)

/*
 * TCF (RW)
 *
 * transmission cancellation finished
 */
#define MCAN_IR_TCF_MASK (0x400U)
#define MCAN_IR_TCF_SHIFT (10U)
#define MCAN_IR_TCF_SET(x) (((uint32_t)(x) << MCAN_IR_TCF_SHIFT) & MCAN_IR_TCF_MASK)
#define MCAN_IR_TCF_GET(x) (((uint32_t)(x) & MCAN_IR_TCF_MASK) >> MCAN_IR_TCF_SHIFT)

/*
 * TC (RW)
 *
 * transmission completed
 */
#define MCAN_IR_TC_MASK (0x200U)
#define MCAN_IR_TC_SHIFT (9U)
#define MCAN_IR_TC_SET(x) (((uint32_t)(x) << MCAN_IR_TC_SHIFT) & MCAN_IR_TC_MASK)
#define MCAN_IR_TC_GET(x) (((uint32_t)(x) & MCAN_IR_TC_MASK) >> MCAN_IR_TC_SHIFT)

/*
 * HPM (RW)
 *
 * high priority message
 */
#define MCAN_IR_HPM_MASK (0x100U)
#define MCAN_IR_HPM_SHIFT (8U)
#define MCAN_IR_HPM_SET(x) (((uint32_t)(x) << MCAN_IR_HPM_SHIFT) & MCAN_IR_HPM_MASK)
#define MCAN_IR_HPM_GET(x) (((uint32_t)(x) & MCAN_IR_HPM_MASK) >> MCAN_IR_HPM_SHIFT)

/*
 * RF1L (RW)
 *
 * rx fifo 1 message lost
 */
#define MCAN_IR_RF1L_MASK (0x80U)
#define MCAN_IR_RF1L_SHIFT (7U)
#define MCAN_IR_RF1L_SET(x) (((uint32_t)(x) << MCAN_IR_RF1L_SHIFT) & MCAN_IR_RF1L_MASK)
#define MCAN_IR_RF1L_GET(x) (((uint32_t)(x) & MCAN_IR_RF1L_MASK) >> MCAN_IR_RF1L_SHIFT)

/*
 * RF1F (RW)
 *
 * rx fifo 1 full
 */
#define MCAN_IR_RF1F_MASK (0x40U)
#define MCAN_IR_RF1F_SHIFT (6U)
#define MCAN_IR_RF1F_SET(x) (((uint32_t)(x) << MCAN_IR_RF1F_SHIFT) & MCAN_IR_RF1F_MASK)
#define MCAN_IR_RF1F_GET(x) (((uint32_t)(x) & MCAN_IR_RF1F_MASK) >> MCAN_IR_RF1F_SHIFT)

/*
 * RF1W (RW)
 *
 * rx fifo 1 watermark reached
 */
#define MCAN_IR_RF1W_MASK (0x20U)
#define MCAN_IR_RF1W_SHIFT (5U)
#define MCAN_IR_RF1W_SET(x) (((uint32_t)(x) << MCAN_IR_RF1W_SHIFT) & MCAN_IR_RF1W_MASK)
#define MCAN_IR_RF1W_GET(x) (((uint32_t)(x) & MCAN_IR_RF1W_MASK) >> MCAN_IR_RF1W_SHIFT)

/*
 * RF1N (RW)
 *
 * rx fifo 1 new message
 */
#define MCAN_IR_RF1N_MASK (0x10U)
#define MCAN_IR_RF1N_SHIFT (4U)
#define MCAN_IR_RF1N_SET(x) (((uint32_t)(x) << MCAN_IR_RF1N_SHIFT) & MCAN_IR_RF1N_MASK)
#define MCAN_IR_RF1N_GET(x) (((uint32_t)(x) & MCAN_IR_RF1N_MASK) >> MCAN_IR_RF1N_SHIFT)

/*
 * RF0L (RW)
 *
 * rx fifo 0 message lost
 */
#define MCAN_IR_RF0L_MASK (0x8U)
#define MCAN_IR_RF0L_SHIFT (3U)
#define MCAN_IR_RF0L_SET(x) (((uint32_t)(x) << MCAN_IR_RF0L_SHIFT) & MCAN_IR_RF0L_MASK)
#define MCAN_IR_RF0L_GET(x) (((uint32_t)(x) & MCAN_IR_RF0L_MASK) >> MCAN_IR_RF0L_SHIFT)

/*
 * RF0F (RW)
 *
 * rx fifo 0 full
 */
#define MCAN_IR_RF0F_MASK (0x4U)
#define MCAN_IR_RF0F_SHIFT (2U)
#define MCAN_IR_RF0F_SET(x) (((uint32_t)(x) << MCAN_IR_RF0F_SHIFT) & MCAN_IR_RF0F_MASK)
#define MCAN_IR_RF0F_GET(x) (((uint32_t)(x) & MCAN_IR_RF0F_MASK) >> MCAN_IR_RF0F_SHIFT)

/*
 * RF0W (RW)
 *
 * rx fifo 0 watermark reched
 */
#define MCAN_IR_RF0W_MASK (0x2U)
#define MCAN_IR_RF0W_SHIFT (1U)
#define MCAN_IR_RF0W_SET(x) (((uint32_t)(x) << MCAN_IR_RF0W_SHIFT) & MCAN_IR_RF0W_MASK)
#define MCAN_IR_RF0W_GET(x) (((uint32_t)(x) & MCAN_IR_RF0W_MASK) >> MCAN_IR_RF0W_SHIFT)

/*
 * RF0N (RW)
 *
 * rx fifo 0 new message
 */
#define MCAN_IR_RF0N_MASK (0x1U)
#define MCAN_IR_RF0N_SHIFT (0U)
#define MCAN_IR_RF0N_SET(x) (((uint32_t)(x) << MCAN_IR_RF0N_SHIFT) & MCAN_IR_RF0N_MASK)
#define MCAN_IR_RF0N_GET(x) (((uint32_t)(x) & MCAN_IR_RF0N_MASK) >> MCAN_IR_RF0N_SHIFT)

/* Bitfield definition for register: IE */
/*
 * ARAE (RW)
 *
 * access to reserved address enable
 */
#define MCAN_IE_ARAE_MASK (0x20000000UL)
#define MCAN_IE_ARAE_SHIFT (29U)
#define MCAN_IE_ARAE_SET(x) (((uint32_t)(x) << MCAN_IE_ARAE_SHIFT) & MCAN_IE_ARAE_MASK)
#define MCAN_IE_ARAE_GET(x) (((uint32_t)(x) & MCAN_IE_ARAE_MASK) >> MCAN_IE_ARAE_SHIFT)

/*
 * PEDE (RW)
 *
 * protocol error in data phase enable
 */
#define MCAN_IE_PEDE_MASK (0x10000000UL)
#define MCAN_IE_PEDE_SHIFT (28U)
#define MCAN_IE_PEDE_SET(x) (((uint32_t)(x) << MCAN_IE_PEDE_SHIFT) & MCAN_IE_PEDE_MASK)
#define MCAN_IE_PEDE_GET(x) (((uint32_t)(x) & MCAN_IE_PEDE_MASK) >> MCAN_IE_PEDE_SHIFT)

/*
 * PEAE (RW)
 *
 * protocol error in arbitration phase enable
 */
#define MCAN_IE_PEAE_MASK (0x8000000UL)
#define MCAN_IE_PEAE_SHIFT (27U)
#define MCAN_IE_PEAE_SET(x) (((uint32_t)(x) << MCAN_IE_PEAE_SHIFT) & MCAN_IE_PEAE_MASK)
#define MCAN_IE_PEAE_GET(x) (((uint32_t)(x) & MCAN_IE_PEAE_MASK) >> MCAN_IE_PEAE_SHIFT)

/*
 * WDIE (RW)
 *
 * watchdog interrupt enable
 */
#define MCAN_IE_WDIE_MASK (0x4000000UL)
#define MCAN_IE_WDIE_SHIFT (26U)
#define MCAN_IE_WDIE_SET(x) (((uint32_t)(x) << MCAN_IE_WDIE_SHIFT) & MCAN_IE_WDIE_MASK)
#define MCAN_IE_WDIE_GET(x) (((uint32_t)(x) & MCAN_IE_WDIE_MASK) >> MCAN_IE_WDIE_SHIFT)

/*
 * BOE (RW)
 *
 * bus off status interrupt enable
 */
#define MCAN_IE_BOE_MASK (0x2000000UL)
#define MCAN_IE_BOE_SHIFT (25U)
#define MCAN_IE_BOE_SET(x) (((uint32_t)(x) << MCAN_IE_BOE_SHIFT) & MCAN_IE_BOE_MASK)
#define MCAN_IE_BOE_GET(x) (((uint32_t)(x) & MCAN_IE_BOE_MASK) >> MCAN_IE_BOE_SHIFT)

/*
 * EWE (RW)
 *
 * warning status interrupt enable
 */
#define MCAN_IE_EWE_MASK (0x1000000UL)
#define MCAN_IE_EWE_SHIFT (24U)
#define MCAN_IE_EWE_SET(x) (((uint32_t)(x) << MCAN_IE_EWE_SHIFT) & MCAN_IE_EWE_MASK)
#define MCAN_IE_EWE_GET(x) (((uint32_t)(x) & MCAN_IE_EWE_MASK) >> MCAN_IE_EWE_SHIFT)

/*
 * EPE (RW)
 *
 * error passive interrupt enable
 */
#define MCAN_IE_EPE_MASK (0x800000UL)
#define MCAN_IE_EPE_SHIFT (23U)
#define MCAN_IE_EPE_SET(x) (((uint32_t)(x) << MCAN_IE_EPE_SHIFT) & MCAN_IE_EPE_MASK)
#define MCAN_IE_EPE_GET(x) (((uint32_t)(x) & MCAN_IE_EPE_MASK) >> MCAN_IE_EPE_SHIFT)

/*
 * ELOE (RW)
 *
 * error logging overflow interrupt enable
 */
#define MCAN_IE_ELOE_MASK (0x400000UL)
#define MCAN_IE_ELOE_SHIFT (22U)
#define MCAN_IE_ELOE_SET(x) (((uint32_t)(x) << MCAN_IE_ELOE_SHIFT) & MCAN_IE_ELOE_MASK)
#define MCAN_IE_ELOE_GET(x) (((uint32_t)(x) & MCAN_IE_ELOE_MASK) >> MCAN_IE_ELOE_SHIFT)

/*
 * BEUE (RW)
 *
 * bit error uncorrected interrupt enable
 */
#define MCAN_IE_BEUE_MASK (0x200000UL)
#define MCAN_IE_BEUE_SHIFT (21U)
#define MCAN_IE_BEUE_SET(x) (((uint32_t)(x) << MCAN_IE_BEUE_SHIFT) & MCAN_IE_BEUE_MASK)
#define MCAN_IE_BEUE_GET(x) (((uint32_t)(x) & MCAN_IE_BEUE_MASK) >> MCAN_IE_BEUE_SHIFT)

/*
 * BECE (RW)
 *
 * bit error corrected interrupt enable
 */
#define MCAN_IE_BECE_MASK (0x100000UL)
#define MCAN_IE_BECE_SHIFT (20U)
#define MCAN_IE_BECE_SET(x) (((uint32_t)(x) << MCAN_IE_BECE_SHIFT) & MCAN_IE_BECE_MASK)
#define MCAN_IE_BECE_GET(x) (((uint32_t)(x) & MCAN_IE_BECE_MASK) >> MCAN_IE_BECE_SHIFT)

/*
 * DRXE (RW)
 *
 * message stored to dedicated rx buffer interrupt enable
 */
#define MCAN_IE_DRXE_MASK (0x80000UL)
#define MCAN_IE_DRXE_SHIFT (19U)
#define MCAN_IE_DRXE_SET(x) (((uint32_t)(x) << MCAN_IE_DRXE_SHIFT) & MCAN_IE_DRXE_MASK)
#define MCAN_IE_DRXE_GET(x) (((uint32_t)(x) & MCAN_IE_DRXE_MASK) >> MCAN_IE_DRXE_SHIFT)

/*
 * TOOE (RW)
 *
 * timeout occurred interrupt enable
 */
#define MCAN_IE_TOOE_MASK (0x40000UL)
#define MCAN_IE_TOOE_SHIFT (18U)
#define MCAN_IE_TOOE_SET(x) (((uint32_t)(x) << MCAN_IE_TOOE_SHIFT) & MCAN_IE_TOOE_MASK)
#define MCAN_IE_TOOE_GET(x) (((uint32_t)(x) & MCAN_IE_TOOE_MASK) >> MCAN_IE_TOOE_SHIFT)

/*
 * MRAFE (RW)
 *
 * message ram access failure interrupt enable
 */
#define MCAN_IE_MRAFE_MASK (0x20000UL)
#define MCAN_IE_MRAFE_SHIFT (17U)
#define MCAN_IE_MRAFE_SET(x) (((uint32_t)(x) << MCAN_IE_MRAFE_SHIFT) & MCAN_IE_MRAFE_MASK)
#define MCAN_IE_MRAFE_GET(x) (((uint32_t)(x) & MCAN_IE_MRAFE_MASK) >> MCAN_IE_MRAFE_SHIFT)

/*
 * TSWE (RW)
 *
 * timestamp wraparound interrupt enable
 */
#define MCAN_IE_TSWE_MASK (0x10000UL)
#define MCAN_IE_TSWE_SHIFT (16U)
#define MCAN_IE_TSWE_SET(x) (((uint32_t)(x) << MCAN_IE_TSWE_SHIFT) & MCAN_IE_TSWE_MASK)
#define MCAN_IE_TSWE_GET(x) (((uint32_t)(x) & MCAN_IE_TSWE_MASK) >> MCAN_IE_TSWE_SHIFT)

/*
 * TEFLE (RW)
 *
 * tx event fifo event lost interrupt enable
 */
#define MCAN_IE_TEFLE_MASK (0x8000U)
#define MCAN_IE_TEFLE_SHIFT (15U)
#define MCAN_IE_TEFLE_SET(x) (((uint32_t)(x) << MCAN_IE_TEFLE_SHIFT) & MCAN_IE_TEFLE_MASK)
#define MCAN_IE_TEFLE_GET(x) (((uint32_t)(x) & MCAN_IE_TEFLE_MASK) >> MCAN_IE_TEFLE_SHIFT)

/*
 * TEFFE (RW)
 *
 * tx event fifo full interrupt enable
 */
#define MCAN_IE_TEFFE_MASK (0x4000U)
#define MCAN_IE_TEFFE_SHIFT (14U)
#define MCAN_IE_TEFFE_SET(x) (((uint32_t)(x) << MCAN_IE_TEFFE_SHIFT) & MCAN_IE_TEFFE_MASK)
#define MCAN_IE_TEFFE_GET(x) (((uint32_t)(x) & MCAN_IE_TEFFE_MASK) >> MCAN_IE_TEFFE_SHIFT)

/*
 * TEFWE (RW)
 *
 * tx event fifo watermark reached interrupt enable
 */
#define MCAN_IE_TEFWE_MASK (0x2000U)
#define MCAN_IE_TEFWE_SHIFT (13U)
#define MCAN_IE_TEFWE_SET(x) (((uint32_t)(x) << MCAN_IE_TEFWE_SHIFT) & MCAN_IE_TEFWE_MASK)
#define MCAN_IE_TEFWE_GET(x) (((uint32_t)(x) & MCAN_IE_TEFWE_MASK) >> MCAN_IE_TEFWE_SHIFT)

/*
 * TEFNE (RW)
 *
 * tx event fifo new entry interrupt enable
 */
#define MCAN_IE_TEFNE_MASK (0x1000U)
#define MCAN_IE_TEFNE_SHIFT (12U)
#define MCAN_IE_TEFNE_SET(x) (((uint32_t)(x) << MCAN_IE_TEFNE_SHIFT) & MCAN_IE_TEFNE_MASK)
#define MCAN_IE_TEFNE_GET(x) (((uint32_t)(x) & MCAN_IE_TEFNE_MASK) >> MCAN_IE_TEFNE_SHIFT)

/*
 * TFEE (RW)
 *
 * tx fifo empty interrupt enable
 */
#define MCAN_IE_TFEE_MASK (0x800U)
#define MCAN_IE_TFEE_SHIFT (11U)
#define MCAN_IE_TFEE_SET(x) (((uint32_t)(x) << MCAN_IE_TFEE_SHIFT) & MCAN_IE_TFEE_MASK)
#define MCAN_IE_TFEE_GET(x) (((uint32_t)(x) & MCAN_IE_TFEE_MASK) >> MCAN_IE_TFEE_SHIFT)

/*
 * TCFE (RW)
 *
 * transmission cancellation finished interrupt enable
 */
#define MCAN_IE_TCFE_MASK (0x400U)
#define MCAN_IE_TCFE_SHIFT (10U)
#define MCAN_IE_TCFE_SET(x) (((uint32_t)(x) << MCAN_IE_TCFE_SHIFT) & MCAN_IE_TCFE_MASK)
#define MCAN_IE_TCFE_GET(x) (((uint32_t)(x) & MCAN_IE_TCFE_MASK) >> MCAN_IE_TCFE_SHIFT)

/*
 * TCE (RW)
 *
 * transmission completed interrupt enable
 */
#define MCAN_IE_TCE_MASK (0x200U)
#define MCAN_IE_TCE_SHIFT (9U)
#define MCAN_IE_TCE_SET(x) (((uint32_t)(x) << MCAN_IE_TCE_SHIFT) & MCAN_IE_TCE_MASK)
#define MCAN_IE_TCE_GET(x) (((uint32_t)(x) & MCAN_IE_TCE_MASK) >> MCAN_IE_TCE_SHIFT)

/*
 * HPME (RW)
 *
 * high priority message interrupt enable
 */
#define MCAN_IE_HPME_MASK (0x100U)
#define MCAN_IE_HPME_SHIFT (8U)
#define MCAN_IE_HPME_SET(x) (((uint32_t)(x) << MCAN_IE_HPME_SHIFT) & MCAN_IE_HPME_MASK)
#define MCAN_IE_HPME_GET(x) (((uint32_t)(x) & MCAN_IE_HPME_MASK) >> MCAN_IE_HPME_SHIFT)

/*
 * RF1LE (RW)
 *
 * rx fifo 1 message lost interrupt enable
 */
#define MCAN_IE_RF1LE_MASK (0x80U)
#define MCAN_IE_RF1LE_SHIFT (7U)
#define MCAN_IE_RF1LE_SET(x) (((uint32_t)(x) << MCAN_IE_RF1LE_SHIFT) & MCAN_IE_RF1LE_MASK)
#define MCAN_IE_RF1LE_GET(x) (((uint32_t)(x) & MCAN_IE_RF1LE_MASK) >> MCAN_IE_RF1LE_SHIFT)

/*
 * RF1FE (RW)
 *
 * rx fifo 1 full interrupt enable
 */
#define MCAN_IE_RF1FE_MASK (0x40U)
#define MCAN_IE_RF1FE_SHIFT (6U)
#define MCAN_IE_RF1FE_SET(x) (((uint32_t)(x) << MCAN_IE_RF1FE_SHIFT) & MCAN_IE_RF1FE_MASK)
#define MCAN_IE_RF1FE_GET(x) (((uint32_t)(x) & MCAN_IE_RF1FE_MASK) >> MCAN_IE_RF1FE_SHIFT)

/*
 * RF1WE (RW)
 *
 * rx fifo 1 watermark reached interrupt enable
 */
#define MCAN_IE_RF1WE_MASK (0x20U)
#define MCAN_IE_RF1WE_SHIFT (5U)
#define MCAN_IE_RF1WE_SET(x) (((uint32_t)(x) << MCAN_IE_RF1WE_SHIFT) & MCAN_IE_RF1WE_MASK)
#define MCAN_IE_RF1WE_GET(x) (((uint32_t)(x) & MCAN_IE_RF1WE_MASK) >> MCAN_IE_RF1WE_SHIFT)

/*
 * RF1NE (RW)
 *
 * rx fifo 1 new message interrupt enable
 */
#define MCAN_IE_RF1NE_MASK (0x10U)
#define MCAN_IE_RF1NE_SHIFT (4U)
#define MCAN_IE_RF1NE_SET(x) (((uint32_t)(x) << MCAN_IE_RF1NE_SHIFT) & MCAN_IE_RF1NE_MASK)
#define MCAN_IE_RF1NE_GET(x) (((uint32_t)(x) & MCAN_IE_RF1NE_MASK) >> MCAN_IE_RF1NE_SHIFT)

/*
 * RF0LE (RW)
 *
 * rx fifo 0 message lost interrupt enable
 */
#define MCAN_IE_RF0LE_MASK (0x8U)
#define MCAN_IE_RF0LE_SHIFT (3U)
#define MCAN_IE_RF0LE_SET(x) (((uint32_t)(x) << MCAN_IE_RF0LE_SHIFT) & MCAN_IE_RF0LE_MASK)
#define MCAN_IE_RF0LE_GET(x) (((uint32_t)(x) & MCAN_IE_RF0LE_MASK) >> MCAN_IE_RF0LE_SHIFT)

/*
 * RF0FE (RW)
 *
 * rx fifo 0 full interrupt enable
 */
#define MCAN_IE_RF0FE_MASK (0x4U)
#define MCAN_IE_RF0FE_SHIFT (2U)
#define MCAN_IE_RF0FE_SET(x) (((uint32_t)(x) << MCAN_IE_RF0FE_SHIFT) & MCAN_IE_RF0FE_MASK)
#define MCAN_IE_RF0FE_GET(x) (((uint32_t)(x) & MCAN_IE_RF0FE_MASK) >> MCAN_IE_RF0FE_SHIFT)

/*
 * RF0WE (RW)
 *
 * rx fifo 0 watermark reached interrupt enable
 */
#define MCAN_IE_RF0WE_MASK (0x2U)
#define MCAN_IE_RF0WE_SHIFT (1U)
#define MCAN_IE_RF0WE_SET(x) (((uint32_t)(x) << MCAN_IE_RF0WE_SHIFT) & MCAN_IE_RF0WE_MASK)
#define MCAN_IE_RF0WE_GET(x) (((uint32_t)(x) & MCAN_IE_RF0WE_MASK) >> MCAN_IE_RF0WE_SHIFT)

/*
 * RF0NE (RW)
 *
 * rx fifo 0 new message interrupt enable
 */
#define MCAN_IE_RF0NE_MASK (0x1U)
#define MCAN_IE_RF0NE_SHIFT (0U)
#define MCAN_IE_RF0NE_SET(x) (((uint32_t)(x) << MCAN_IE_RF0NE_SHIFT) & MCAN_IE_RF0NE_MASK)
#define MCAN_IE_RF0NE_GET(x) (((uint32_t)(x) & MCAN_IE_RF0NE_MASK) >> MCAN_IE_RF0NE_SHIFT)

/* Bitfield definition for register: ILS */
/*
 * ARAL (RW)
 *
 * access to reserved address line
 */
#define MCAN_ILS_ARAL_MASK (0x20000000UL)
#define MCAN_ILS_ARAL_SHIFT (29U)
#define MCAN_ILS_ARAL_SET(x) (((uint32_t)(x) << MCAN_ILS_ARAL_SHIFT) & MCAN_ILS_ARAL_MASK)
#define MCAN_ILS_ARAL_GET(x) (((uint32_t)(x) & MCAN_ILS_ARAL_MASK) >> MCAN_ILS_ARAL_SHIFT)

/*
 * PEDL (RW)
 *
 * protocol error in data phase line
 */
#define MCAN_ILS_PEDL_MASK (0x10000000UL)
#define MCAN_ILS_PEDL_SHIFT (28U)
#define MCAN_ILS_PEDL_SET(x) (((uint32_t)(x) << MCAN_ILS_PEDL_SHIFT) & MCAN_ILS_PEDL_MASK)
#define MCAN_ILS_PEDL_GET(x) (((uint32_t)(x) & MCAN_ILS_PEDL_MASK) >> MCAN_ILS_PEDL_SHIFT)

/*
 * PEAL (RW)
 *
 * protocol error in arbitration phase line
 */
#define MCAN_ILS_PEAL_MASK (0x8000000UL)
#define MCAN_ILS_PEAL_SHIFT (27U)
#define MCAN_ILS_PEAL_SET(x) (((uint32_t)(x) << MCAN_ILS_PEAL_SHIFT) & MCAN_ILS_PEAL_MASK)
#define MCAN_ILS_PEAL_GET(x) (((uint32_t)(x) & MCAN_ILS_PEAL_MASK) >> MCAN_ILS_PEAL_SHIFT)

/*
 * WDIL (RW)
 *
 * watchdog interrupt line
 */
#define MCAN_ILS_WDIL_MASK (0x4000000UL)
#define MCAN_ILS_WDIL_SHIFT (26U)
#define MCAN_ILS_WDIL_SET(x) (((uint32_t)(x) << MCAN_ILS_WDIL_SHIFT) & MCAN_ILS_WDIL_MASK)
#define MCAN_ILS_WDIL_GET(x) (((uint32_t)(x) & MCAN_ILS_WDIL_MASK) >> MCAN_ILS_WDIL_SHIFT)

/*
 * BOL (RW)
 *
 * bus off status interrupt line
 */
#define MCAN_ILS_BOL_MASK (0x2000000UL)
#define MCAN_ILS_BOL_SHIFT (25U)
#define MCAN_ILS_BOL_SET(x) (((uint32_t)(x) << MCAN_ILS_BOL_SHIFT) & MCAN_ILS_BOL_MASK)
#define MCAN_ILS_BOL_GET(x) (((uint32_t)(x) & MCAN_ILS_BOL_MASK) >> MCAN_ILS_BOL_SHIFT)

/*
 * EWL (RW)
 *
 * warning status interrupt line
 */
#define MCAN_ILS_EWL_MASK (0x1000000UL)
#define MCAN_ILS_EWL_SHIFT (24U)
#define MCAN_ILS_EWL_SET(x) (((uint32_t)(x) << MCAN_ILS_EWL_SHIFT) & MCAN_ILS_EWL_MASK)
#define MCAN_ILS_EWL_GET(x) (((uint32_t)(x) & MCAN_ILS_EWL_MASK) >> MCAN_ILS_EWL_SHIFT)

/*
 * EPL (RW)
 *
 * error passive interrupt line
 */
#define MCAN_ILS_EPL_MASK (0x800000UL)
#define MCAN_ILS_EPL_SHIFT (23U)
#define MCAN_ILS_EPL_SET(x) (((uint32_t)(x) << MCAN_ILS_EPL_SHIFT) & MCAN_ILS_EPL_MASK)
#define MCAN_ILS_EPL_GET(x) (((uint32_t)(x) & MCAN_ILS_EPL_MASK) >> MCAN_ILS_EPL_SHIFT)

/*
 * ELOL (RW)
 *
 * error logging overflow interrupt line
 */
#define MCAN_ILS_ELOL_MASK (0x400000UL)
#define MCAN_ILS_ELOL_SHIFT (22U)
#define MCAN_ILS_ELOL_SET(x) (((uint32_t)(x) << MCAN_ILS_ELOL_SHIFT) & MCAN_ILS_ELOL_MASK)
#define MCAN_ILS_ELOL_GET(x) (((uint32_t)(x) & MCAN_ILS_ELOL_MASK) >> MCAN_ILS_ELOL_SHIFT)

/*
 * BEUL (RW)
 *
 * bit error uncorrected interrupt line
 */
#define MCAN_ILS_BEUL_MASK (0x200000UL)
#define MCAN_ILS_BEUL_SHIFT (21U)
#define MCAN_ILS_BEUL_SET(x) (((uint32_t)(x) << MCAN_ILS_BEUL_SHIFT) & MCAN_ILS_BEUL_MASK)
#define MCAN_ILS_BEUL_GET(x) (((uint32_t)(x) & MCAN_ILS_BEUL_MASK) >> MCAN_ILS_BEUL_SHIFT)

/*
 * BECL (RW)
 *
 * bit error corrected interrupt line
 */
#define MCAN_ILS_BECL_MASK (0x100000UL)
#define MCAN_ILS_BECL_SHIFT (20U)
#define MCAN_ILS_BECL_SET(x) (((uint32_t)(x) << MCAN_ILS_BECL_SHIFT) & MCAN_ILS_BECL_MASK)
#define MCAN_ILS_BECL_GET(x) (((uint32_t)(x) & MCAN_ILS_BECL_MASK) >> MCAN_ILS_BECL_SHIFT)

/*
 * DRXL (RW)
 *
 * message stored to dedicated rx buffer interrupt line
 */
#define MCAN_ILS_DRXL_MASK (0x80000UL)
#define MCAN_ILS_DRXL_SHIFT (19U)
#define MCAN_ILS_DRXL_SET(x) (((uint32_t)(x) << MCAN_ILS_DRXL_SHIFT) & MCAN_ILS_DRXL_MASK)
#define MCAN_ILS_DRXL_GET(x) (((uint32_t)(x) & MCAN_ILS_DRXL_MASK) >> MCAN_ILS_DRXL_SHIFT)

/*
 * TOOL (RW)
 *
 * timeout occurred interrupt line
 */
#define MCAN_ILS_TOOL_MASK (0x40000UL)
#define MCAN_ILS_TOOL_SHIFT (18U)
#define MCAN_ILS_TOOL_SET(x) (((uint32_t)(x) << MCAN_ILS_TOOL_SHIFT) & MCAN_ILS_TOOL_MASK)
#define MCAN_ILS_TOOL_GET(x) (((uint32_t)(x) & MCAN_ILS_TOOL_MASK) >> MCAN_ILS_TOOL_SHIFT)

/*
 * MRAFL (RW)
 *
 * message ram access failure interrupt line
 */
#define MCAN_ILS_MRAFL_MASK (0x20000UL)
#define MCAN_ILS_MRAFL_SHIFT (17U)
#define MCAN_ILS_MRAFL_SET(x) (((uint32_t)(x) << MCAN_ILS_MRAFL_SHIFT) & MCAN_ILS_MRAFL_MASK)
#define MCAN_ILS_MRAFL_GET(x) (((uint32_t)(x) & MCAN_ILS_MRAFL_MASK) >> MCAN_ILS_MRAFL_SHIFT)

/*
 * TSWL (RW)
 *
 * timestamp wraparound interrupt line
 */
#define MCAN_ILS_TSWL_MASK (0x10000UL)
#define MCAN_ILS_TSWL_SHIFT (16U)
#define MCAN_ILS_TSWL_SET(x) (((uint32_t)(x) << MCAN_ILS_TSWL_SHIFT) & MCAN_ILS_TSWL_MASK)
#define MCAN_ILS_TSWL_GET(x) (((uint32_t)(x) & MCAN_ILS_TSWL_MASK) >> MCAN_ILS_TSWL_SHIFT)

/*
 * TEFLL (RW)
 *
 * tx event fifo event lost interrupt line
 */
#define MCAN_ILS_TEFLL_MASK (0x8000U)
#define MCAN_ILS_TEFLL_SHIFT (15U)
#define MCAN_ILS_TEFLL_SET(x) (((uint32_t)(x) << MCAN_ILS_TEFLL_SHIFT) & MCAN_ILS_TEFLL_MASK)
#define MCAN_ILS_TEFLL_GET(x) (((uint32_t)(x) & MCAN_ILS_TEFLL_MASK) >> MCAN_ILS_TEFLL_SHIFT)

/*
 * TEFFL (RW)
 *
 * tx event fifo full interrupt line
 */
#define MCAN_ILS_TEFFL_MASK (0x4000U)
#define MCAN_ILS_TEFFL_SHIFT (14U)
#define MCAN_ILS_TEFFL_SET(x) (((uint32_t)(x) << MCAN_ILS_TEFFL_SHIFT) & MCAN_ILS_TEFFL_MASK)
#define MCAN_ILS_TEFFL_GET(x) (((uint32_t)(x) & MCAN_ILS_TEFFL_MASK) >> MCAN_ILS_TEFFL_SHIFT)

/*
 * TEFWL (RW)
 *
 * tx event fifo watermark reached interrupt line
 */
#define MCAN_ILS_TEFWL_MASK (0x2000U)
#define MCAN_ILS_TEFWL_SHIFT (13U)
#define MCAN_ILS_TEFWL_SET(x) (((uint32_t)(x) << MCAN_ILS_TEFWL_SHIFT) & MCAN_ILS_TEFWL_MASK)
#define MCAN_ILS_TEFWL_GET(x) (((uint32_t)(x) & MCAN_ILS_TEFWL_MASK) >> MCAN_ILS_TEFWL_SHIFT)

/*
 * TEFNL (RW)
 *
 * tx event fifo new entry interrupt line
 */
#define MCAN_ILS_TEFNL_MASK (0x1000U)
#define MCAN_ILS_TEFNL_SHIFT (12U)
#define MCAN_ILS_TEFNL_SET(x) (((uint32_t)(x) << MCAN_ILS_TEFNL_SHIFT) & MCAN_ILS_TEFNL_MASK)
#define MCAN_ILS_TEFNL_GET(x) (((uint32_t)(x) & MCAN_ILS_TEFNL_MASK) >> MCAN_ILS_TEFNL_SHIFT)

/*
 * TFEL (RW)
 *
 * tx fifo empty interrupt line
 */
#define MCAN_ILS_TFEL_MASK (0x800U)
#define MCAN_ILS_TFEL_SHIFT (11U)
#define MCAN_ILS_TFEL_SET(x) (((uint32_t)(x) << MCAN_ILS_TFEL_SHIFT) & MCAN_ILS_TFEL_MASK)
#define MCAN_ILS_TFEL_GET(x) (((uint32_t)(x) & MCAN_ILS_TFEL_MASK) >> MCAN_ILS_TFEL_SHIFT)

/*
 * TCFL (RW)
 *
 * transmission cancellation finished interrupt line
 */
#define MCAN_ILS_TCFL_MASK (0x400U)
#define MCAN_ILS_TCFL_SHIFT (10U)
#define MCAN_ILS_TCFL_SET(x) (((uint32_t)(x) << MCAN_ILS_TCFL_SHIFT) & MCAN_ILS_TCFL_MASK)
#define MCAN_ILS_TCFL_GET(x) (((uint32_t)(x) & MCAN_ILS_TCFL_MASK) >> MCAN_ILS_TCFL_SHIFT)

/*
 * TCL (RW)
 *
 * transmission completed interrupt line
 */
#define MCAN_ILS_TCL_MASK (0x200U)
#define MCAN_ILS_TCL_SHIFT (9U)
#define MCAN_ILS_TCL_SET(x) (((uint32_t)(x) << MCAN_ILS_TCL_SHIFT) & MCAN_ILS_TCL_MASK)
#define MCAN_ILS_TCL_GET(x) (((uint32_t)(x) & MCAN_ILS_TCL_MASK) >> MCAN_ILS_TCL_SHIFT)

/*
 * HPML (RW)
 *
 * high priority message interrupt line
 */
#define MCAN_ILS_HPML_MASK (0x100U)
#define MCAN_ILS_HPML_SHIFT (8U)
#define MCAN_ILS_HPML_SET(x) (((uint32_t)(x) << MCAN_ILS_HPML_SHIFT) & MCAN_ILS_HPML_MASK)
#define MCAN_ILS_HPML_GET(x) (((uint32_t)(x) & MCAN_ILS_HPML_MASK) >> MCAN_ILS_HPML_SHIFT)

/*
 * RF1LL (RW)
 *
 * rx fifo 1 message lost interrupt line
 */
#define MCAN_ILS_RF1LL_MASK (0x80U)
#define MCAN_ILS_RF1LL_SHIFT (7U)
#define MCAN_ILS_RF1LL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF1LL_SHIFT) & MCAN_ILS_RF1LL_MASK)
#define MCAN_ILS_RF1LL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF1LL_MASK) >> MCAN_ILS_RF1LL_SHIFT)

/*
 * RF1FL (RW)
 *
 * rx fifo 1 full interrupt line
 */
#define MCAN_ILS_RF1FL_MASK (0x40U)
#define MCAN_ILS_RF1FL_SHIFT (6U)
#define MCAN_ILS_RF1FL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF1FL_SHIFT) & MCAN_ILS_RF1FL_MASK)
#define MCAN_ILS_RF1FL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF1FL_MASK) >> MCAN_ILS_RF1FL_SHIFT)

/*
 * RF1WL (RW)
 *
 * rx fifo 1 watermark reached interrupt line
 */
#define MCAN_ILS_RF1WL_MASK (0x20U)
#define MCAN_ILS_RF1WL_SHIFT (5U)
#define MCAN_ILS_RF1WL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF1WL_SHIFT) & MCAN_ILS_RF1WL_MASK)
#define MCAN_ILS_RF1WL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF1WL_MASK) >> MCAN_ILS_RF1WL_SHIFT)

/*
 * RF1NL (RW)
 *
 * rx fifo 1 new message interrupt line
 */
#define MCAN_ILS_RF1NL_MASK (0x10U)
#define MCAN_ILS_RF1NL_SHIFT (4U)
#define MCAN_ILS_RF1NL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF1NL_SHIFT) & MCAN_ILS_RF1NL_MASK)
#define MCAN_ILS_RF1NL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF1NL_MASK) >> MCAN_ILS_RF1NL_SHIFT)

/*
 * RF0LL (RW)
 *
 * rx fifo 0 message lost interrupt line
 */
#define MCAN_ILS_RF0LL_MASK (0x8U)
#define MCAN_ILS_RF0LL_SHIFT (3U)
#define MCAN_ILS_RF0LL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF0LL_SHIFT) & MCAN_ILS_RF0LL_MASK)
#define MCAN_ILS_RF0LL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF0LL_MASK) >> MCAN_ILS_RF0LL_SHIFT)

/*
 * RF0FL (RW)
 *
 * rx fifo 0 full interrupt line
 */
#define MCAN_ILS_RF0FL_MASK (0x4U)
#define MCAN_ILS_RF0FL_SHIFT (2U)
#define MCAN_ILS_RF0FL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF0FL_SHIFT) & MCAN_ILS_RF0FL_MASK)
#define MCAN_ILS_RF0FL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF0FL_MASK) >> MCAN_ILS_RF0FL_SHIFT)

/*
 * RF0WL (RW)
 *
 * rx fifo 0 watermark reached interrupt line
 */
#define MCAN_ILS_RF0WL_MASK (0x2U)
#define MCAN_ILS_RF0WL_SHIFT (1U)
#define MCAN_ILS_RF0WL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF0WL_SHIFT) & MCAN_ILS_RF0WL_MASK)
#define MCAN_ILS_RF0WL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF0WL_MASK) >> MCAN_ILS_RF0WL_SHIFT)

/*
 * RF0NL (RW)
 *
 * rx fifo 0 new message interrupt line
 */
#define MCAN_ILS_RF0NL_MASK (0x1U)
#define MCAN_ILS_RF0NL_SHIFT (0U)
#define MCAN_ILS_RF0NL_SET(x) (((uint32_t)(x) << MCAN_ILS_RF0NL_SHIFT) & MCAN_ILS_RF0NL_MASK)
#define MCAN_ILS_RF0NL_GET(x) (((uint32_t)(x) & MCAN_ILS_RF0NL_MASK) >> MCAN_ILS_RF0NL_SHIFT)

/* Bitfield definition for register: ILE */
/*
 * EINT1 (RW)
 *
 * enable interrupt line 1
 */
#define MCAN_ILE_EINT1_MASK (0x2U)
#define MCAN_ILE_EINT1_SHIFT (1U)
#define MCAN_ILE_EINT1_SET(x) (((uint32_t)(x) << MCAN_ILE_EINT1_SHIFT) & MCAN_ILE_EINT1_MASK)
#define MCAN_ILE_EINT1_GET(x) (((uint32_t)(x) & MCAN_ILE_EINT1_MASK) >> MCAN_ILE_EINT1_SHIFT)

/*
 * EINT0 (RW)
 *
 * enable interrupt line 0
 */
#define MCAN_ILE_EINT0_MASK (0x1U)
#define MCAN_ILE_EINT0_SHIFT (0U)
#define MCAN_ILE_EINT0_SET(x) (((uint32_t)(x) << MCAN_ILE_EINT0_SHIFT) & MCAN_ILE_EINT0_MASK)
#define MCAN_ILE_EINT0_GET(x) (((uint32_t)(x) & MCAN_ILE_EINT0_MASK) >> MCAN_ILE_EINT0_SHIFT)

/* Bitfield definition for register: GFC */
/*
 * ANFS (RW)
 *
 * accept non-matching frames standard
 */
#define MCAN_GFC_ANFS_MASK (0x30U)
#define MCAN_GFC_ANFS_SHIFT (4U)
#define MCAN_GFC_ANFS_SET(x) (((uint32_t)(x) << MCAN_GFC_ANFS_SHIFT) & MCAN_GFC_ANFS_MASK)
#define MCAN_GFC_ANFS_GET(x) (((uint32_t)(x) & MCAN_GFC_ANFS_MASK) >> MCAN_GFC_ANFS_SHIFT)

/*
 * ANFE (RW)
 *
 * accept non-matching frames extended
 */
#define MCAN_GFC_ANFE_MASK (0xCU)
#define MCAN_GFC_ANFE_SHIFT (2U)
#define MCAN_GFC_ANFE_SET(x) (((uint32_t)(x) << MCAN_GFC_ANFE_SHIFT) & MCAN_GFC_ANFE_MASK)
#define MCAN_GFC_ANFE_GET(x) (((uint32_t)(x) & MCAN_GFC_ANFE_MASK) >> MCAN_GFC_ANFE_SHIFT)

/*
 * RRFS (RW)
 *
 * reject remote frames standards
 */
#define MCAN_GFC_RRFS_MASK (0x2U)
#define MCAN_GFC_RRFS_SHIFT (1U)
#define MCAN_GFC_RRFS_SET(x) (((uint32_t)(x) << MCAN_GFC_RRFS_SHIFT) & MCAN_GFC_RRFS_MASK)
#define MCAN_GFC_RRFS_GET(x) (((uint32_t)(x) & MCAN_GFC_RRFS_MASK) >> MCAN_GFC_RRFS_SHIFT)

/*
 * RRFE (RW)
 *
 * reject remote frames extended
 */
#define MCAN_GFC_RRFE_MASK (0x1U)
#define MCAN_GFC_RRFE_SHIFT (0U)
#define MCAN_GFC_RRFE_SET(x) (((uint32_t)(x) << MCAN_GFC_RRFE_SHIFT) & MCAN_GFC_RRFE_MASK)
#define MCAN_GFC_RRFE_GET(x) (((uint32_t)(x) & MCAN_GFC_RRFE_MASK) >> MCAN_GFC_RRFE_SHIFT)

/* Bitfield definition for register: SIDFC */
/*
 * LSS (RW)
 *
 * list size standard
 */
#define MCAN_SIDFC_LSS_MASK (0xFF0000UL)
#define MCAN_SIDFC_LSS_SHIFT (16U)
#define MCAN_SIDFC_LSS_SET(x) (((uint32_t)(x) << MCAN_SIDFC_LSS_SHIFT) & MCAN_SIDFC_LSS_MASK)
#define MCAN_SIDFC_LSS_GET(x) (((uint32_t)(x) & MCAN_SIDFC_LSS_MASK) >> MCAN_SIDFC_LSS_SHIFT)

/*
 * FLSSA (RW)
 *
 * filter list standard start address
 */
#define MCAN_SIDFC_FLSSA_MASK (0xFFFCU)
#define MCAN_SIDFC_FLSSA_SHIFT (2U)
#define MCAN_SIDFC_FLSSA_SET(x) (((uint32_t)(x) << MCAN_SIDFC_FLSSA_SHIFT) & MCAN_SIDFC_FLSSA_MASK)
#define MCAN_SIDFC_FLSSA_GET(x) (((uint32_t)(x) & MCAN_SIDFC_FLSSA_MASK) >> MCAN_SIDFC_FLSSA_SHIFT)

/* Bitfield definition for register: XIDFC */
/*
 * LSE (RW)
 *
 * list size extended
 */
#define MCAN_XIDFC_LSE_MASK (0x7F0000UL)
#define MCAN_XIDFC_LSE_SHIFT (16U)
#define MCAN_XIDFC_LSE_SET(x) (((uint32_t)(x) << MCAN_XIDFC_LSE_SHIFT) & MCAN_XIDFC_LSE_MASK)
#define MCAN_XIDFC_LSE_GET(x) (((uint32_t)(x) & MCAN_XIDFC_LSE_MASK) >> MCAN_XIDFC_LSE_SHIFT)

/*
 * FLESA (RW)
 *
 * filter list extended start address
 */
#define MCAN_XIDFC_FLESA_MASK (0xFFFCU)
#define MCAN_XIDFC_FLESA_SHIFT (2U)
#define MCAN_XIDFC_FLESA_SET(x) (((uint32_t)(x) << MCAN_XIDFC_FLESA_SHIFT) & MCAN_XIDFC_FLESA_MASK)
#define MCAN_XIDFC_FLESA_GET(x) (((uint32_t)(x) & MCAN_XIDFC_FLESA_MASK) >> MCAN_XIDFC_FLESA_SHIFT)

/* Bitfield definition for register: XIDAM */
/*
 * EIDM (RW)
 *
 * extended ID mask
 */
#define MCAN_XIDAM_EIDM_MASK (0x1FFFFFFFUL)
#define MCAN_XIDAM_EIDM_SHIFT (0U)
#define MCAN_XIDAM_EIDM_SET(x) (((uint32_t)(x) << MCAN_XIDAM_EIDM_SHIFT) & MCAN_XIDAM_EIDM_MASK)
#define MCAN_XIDAM_EIDM_GET(x) (((uint32_t)(x) & MCAN_XIDAM_EIDM_MASK) >> MCAN_XIDAM_EIDM_SHIFT)

/* Bitfield definition for register: HPMS */
/*
 * FLST (R)
 *
 * filter list
 */
#define MCAN_HPMS_FLST_MASK (0x8000U)
#define MCAN_HPMS_FLST_SHIFT (15U)
#define MCAN_HPMS_FLST_GET(x) (((uint32_t)(x) & MCAN_HPMS_FLST_MASK) >> MCAN_HPMS_FLST_SHIFT)

/*
 * FIDX (R)
 *
 * filter index
 */
#define MCAN_HPMS_FIDX_MASK (0x7F00U)
#define MCAN_HPMS_FIDX_SHIFT (8U)
#define MCAN_HPMS_FIDX_GET(x) (((uint32_t)(x) & MCAN_HPMS_FIDX_MASK) >> MCAN_HPMS_FIDX_SHIFT)

/*
 * MSI (R)
 *
 * message storage indicator
 */
#define MCAN_HPMS_MSI_MASK (0xC0U)
#define MCAN_HPMS_MSI_SHIFT (6U)
#define MCAN_HPMS_MSI_GET(x) (((uint32_t)(x) & MCAN_HPMS_MSI_MASK) >> MCAN_HPMS_MSI_SHIFT)

/*
 * BIDX (R)
 *
 * buffer index
 */
#define MCAN_HPMS_BIDX_MASK (0x3FU)
#define MCAN_HPMS_BIDX_SHIFT (0U)
#define MCAN_HPMS_BIDX_GET(x) (((uint32_t)(x) & MCAN_HPMS_BIDX_MASK) >> MCAN_HPMS_BIDX_SHIFT)

/* Bitfield definition for register: NDAT1 */
/*
 * ND1 (RW)
 *
 * new data 31-0
 */
#define MCAN_NDAT1_ND1_MASK (0xFFFFFFFFUL)
#define MCAN_NDAT1_ND1_SHIFT (0U)
#define MCAN_NDAT1_ND1_SET(x) (((uint32_t)(x) << MCAN_NDAT1_ND1_SHIFT) & MCAN_NDAT1_ND1_MASK)
#define MCAN_NDAT1_ND1_GET(x) (((uint32_t)(x) & MCAN_NDAT1_ND1_MASK) >> MCAN_NDAT1_ND1_SHIFT)

/* Bitfield definition for register: NDAT2 */
/*
 * ND2 (RW)
 *
 * new data 63-32
 */
#define MCAN_NDAT2_ND2_MASK (0xFFFFFFFFUL)
#define MCAN_NDAT2_ND2_SHIFT (0U)
#define MCAN_NDAT2_ND2_SET(x) (((uint32_t)(x) << MCAN_NDAT2_ND2_SHIFT) & MCAN_NDAT2_ND2_MASK)
#define MCAN_NDAT2_ND2_GET(x) (((uint32_t)(x) & MCAN_NDAT2_ND2_MASK) >> MCAN_NDAT2_ND2_SHIFT)

/* Bitfield definition for register: RXF0C */
/*
 * F0OM (RW)
 *
 * fifo 0 operation mode
 */
#define MCAN_RXF0C_F0OM_MASK (0x80000000UL)
#define MCAN_RXF0C_F0OM_SHIFT (31U)
#define MCAN_RXF0C_F0OM_SET(x) (((uint32_t)(x) << MCAN_RXF0C_F0OM_SHIFT) & MCAN_RXF0C_F0OM_MASK)
#define MCAN_RXF0C_F0OM_GET(x) (((uint32_t)(x) & MCAN_RXF0C_F0OM_MASK) >> MCAN_RXF0C_F0OM_SHIFT)

/*
 * F0WM (RW)
 *
 * rx fifo 0 watermark
 */
#define MCAN_RXF0C_F0WM_MASK (0x7F000000UL)
#define MCAN_RXF0C_F0WM_SHIFT (24U)
#define MCAN_RXF0C_F0WM_SET(x) (((uint32_t)(x) << MCAN_RXF0C_F0WM_SHIFT) & MCAN_RXF0C_F0WM_MASK)
#define MCAN_RXF0C_F0WM_GET(x) (((uint32_t)(x) & MCAN_RXF0C_F0WM_MASK) >> MCAN_RXF0C_F0WM_SHIFT)

/*
 * F0S (RW)
 *
 * rx fifo 0 size
 */
#define MCAN_RXF0C_F0S_MASK (0x7F0000UL)
#define MCAN_RXF0C_F0S_SHIFT (16U)
#define MCAN_RXF0C_F0S_SET(x) (((uint32_t)(x) << MCAN_RXF0C_F0S_SHIFT) & MCAN_RXF0C_F0S_MASK)
#define MCAN_RXF0C_F0S_GET(x) (((uint32_t)(x) & MCAN_RXF0C_F0S_MASK) >> MCAN_RXF0C_F0S_SHIFT)

/*
 * F0SA (RW)
 *
 * rx fifo 0 start address
 */
#define MCAN_RXF0C_F0SA_MASK (0xFFFCU)
#define MCAN_RXF0C_F0SA_SHIFT (2U)
#define MCAN_RXF0C_F0SA_SET(x) (((uint32_t)(x) << MCAN_RXF0C_F0SA_SHIFT) & MCAN_RXF0C_F0SA_MASK)
#define MCAN_RXF0C_F0SA_GET(x) (((uint32_t)(x) & MCAN_RXF0C_F0SA_MASK) >> MCAN_RXF0C_F0SA_SHIFT)

/* Bitfield definition for register: RXF0S */
/*
 * RF0L (R)
 *
 * rx fifo 0 message lost
 */
#define MCAN_RXF0S_RF0L_MASK (0x2000000UL)
#define MCAN_RXF0S_RF0L_SHIFT (25U)
#define MCAN_RXF0S_RF0L_GET(x) (((uint32_t)(x) & MCAN_RXF0S_RF0L_MASK) >> MCAN_RXF0S_RF0L_SHIFT)

/*
 * F0F (R)
 *
 * rx fifo 0 full
 */
#define MCAN_RXF0S_F0F_MASK (0x1000000UL)
#define MCAN_RXF0S_F0F_SHIFT (24U)
#define MCAN_RXF0S_F0F_GET(x) (((uint32_t)(x) & MCAN_RXF0S_F0F_MASK) >> MCAN_RXF0S_F0F_SHIFT)

/*
 * F0PI (R)
 *
 * rx fifo 0 put index
 */
#define MCAN_RXF0S_F0PI_MASK (0x3F0000UL)
#define MCAN_RXF0S_F0PI_SHIFT (16U)
#define MCAN_RXF0S_F0PI_GET(x) (((uint32_t)(x) & MCAN_RXF0S_F0PI_MASK) >> MCAN_RXF0S_F0PI_SHIFT)

/*
 * F0GI (R)
 *
 * rx fifo 0 get index
 */
#define MCAN_RXF0S_F0GI_MASK (0x3F00U)
#define MCAN_RXF0S_F0GI_SHIFT (8U)
#define MCAN_RXF0S_F0GI_GET(x) (((uint32_t)(x) & MCAN_RXF0S_F0GI_MASK) >> MCAN_RXF0S_F0GI_SHIFT)

/*
 * F0FL (R)
 *
 * rx fifo 0 fill level
 */
#define MCAN_RXF0S_F0FL_MASK (0x7FU)
#define MCAN_RXF0S_F0FL_SHIFT (0U)
#define MCAN_RXF0S_F0FL_GET(x) (((uint32_t)(x) & MCAN_RXF0S_F0FL_MASK) >> MCAN_RXF0S_F0FL_SHIFT)

/* Bitfield definition for register: RXF0A */
/*
 * F0AI (RW)
 *
 * rx fifo0 ackowledge index
 */
#define MCAN_RXF0A_F0AI_MASK (0x3FU)
#define MCAN_RXF0A_F0AI_SHIFT (0U)
#define MCAN_RXF0A_F0AI_SET(x) (((uint32_t)(x) << MCAN_RXF0A_F0AI_SHIFT) & MCAN_RXF0A_F0AI_MASK)
#define MCAN_RXF0A_F0AI_GET(x) (((uint32_t)(x) & MCAN_RXF0A_F0AI_MASK) >> MCAN_RXF0A_F0AI_SHIFT)

/* Bitfield definition for register: RXBC */
/*
 * RBSA (RW)
 *
 * rx buffer start address
 */
#define MCAN_RXBC_RBSA_MASK (0xFFFCU)
#define MCAN_RXBC_RBSA_SHIFT (2U)
#define MCAN_RXBC_RBSA_SET(x) (((uint32_t)(x) << MCAN_RXBC_RBSA_SHIFT) & MCAN_RXBC_RBSA_MASK)
#define MCAN_RXBC_RBSA_GET(x) (((uint32_t)(x) & MCAN_RXBC_RBSA_MASK) >> MCAN_RXBC_RBSA_SHIFT)

/* Bitfield definition for register: RXF1C */
/*
 * F1OM (RW)
 *
 * fifo 1 operation mode
 */
#define MCAN_RXF1C_F1OM_MASK (0x80000000UL)
#define MCAN_RXF1C_F1OM_SHIFT (31U)
#define MCAN_RXF1C_F1OM_SET(x) (((uint32_t)(x) << MCAN_RXF1C_F1OM_SHIFT) & MCAN_RXF1C_F1OM_MASK)
#define MCAN_RXF1C_F1OM_GET(x) (((uint32_t)(x) & MCAN_RXF1C_F1OM_MASK) >> MCAN_RXF1C_F1OM_SHIFT)

/*
 * F1WM (RW)
 *
 * rx fifo 1 watermark
 */
#define MCAN_RXF1C_F1WM_MASK (0x7F000000UL)
#define MCAN_RXF1C_F1WM_SHIFT (24U)
#define MCAN_RXF1C_F1WM_SET(x) (((uint32_t)(x) << MCAN_RXF1C_F1WM_SHIFT) & MCAN_RXF1C_F1WM_MASK)
#define MCAN_RXF1C_F1WM_GET(x) (((uint32_t)(x) & MCAN_RXF1C_F1WM_MASK) >> MCAN_RXF1C_F1WM_SHIFT)

/*
 * F1S (RW)
 *
 * rx fifo1 size
 */
#define MCAN_RXF1C_F1S_MASK (0x7F0000UL)
#define MCAN_RXF1C_F1S_SHIFT (16U)
#define MCAN_RXF1C_F1S_SET(x) (((uint32_t)(x) << MCAN_RXF1C_F1S_SHIFT) & MCAN_RXF1C_F1S_MASK)
#define MCAN_RXF1C_F1S_GET(x) (((uint32_t)(x) & MCAN_RXF1C_F1S_MASK) >> MCAN_RXF1C_F1S_SHIFT)

/*
 * F1SA (RW)
 *
 * rx fifo1 start address
 */
#define MCAN_RXF1C_F1SA_MASK (0xFFFCU)
#define MCAN_RXF1C_F1SA_SHIFT (2U)
#define MCAN_RXF1C_F1SA_SET(x) (((uint32_t)(x) << MCAN_RXF1C_F1SA_SHIFT) & MCAN_RXF1C_F1SA_MASK)
#define MCAN_RXF1C_F1SA_GET(x) (((uint32_t)(x) & MCAN_RXF1C_F1SA_MASK) >> MCAN_RXF1C_F1SA_SHIFT)

/* Bitfield definition for register: RXF1S */
/*
 * DMS (R)
 *
 * debug message status
 */
#define MCAN_RXF1S_DMS_MASK (0xC0000000UL)
#define MCAN_RXF1S_DMS_SHIFT (30U)
#define MCAN_RXF1S_DMS_GET(x) (((uint32_t)(x) & MCAN_RXF1S_DMS_MASK) >> MCAN_RXF1S_DMS_SHIFT)

/*
 * RF1L (R)
 *
 * rx fifo1 message lost
 */
#define MCAN_RXF1S_RF1L_MASK (0x2000000UL)
#define MCAN_RXF1S_RF1L_SHIFT (25U)
#define MCAN_RXF1S_RF1L_GET(x) (((uint32_t)(x) & MCAN_RXF1S_RF1L_MASK) >> MCAN_RXF1S_RF1L_SHIFT)

/*
 * F1F (R)
 *
 * rx fifo1 full
 */
#define MCAN_RXF1S_F1F_MASK (0x1000000UL)
#define MCAN_RXF1S_F1F_SHIFT (24U)
#define MCAN_RXF1S_F1F_GET(x) (((uint32_t)(x) & MCAN_RXF1S_F1F_MASK) >> MCAN_RXF1S_F1F_SHIFT)

/*
 * F1PI (R)
 *
 * rx fifo 1 put index
 */
#define MCAN_RXF1S_F1PI_MASK (0x3F0000UL)
#define MCAN_RXF1S_F1PI_SHIFT (16U)
#define MCAN_RXF1S_F1PI_GET(x) (((uint32_t)(x) & MCAN_RXF1S_F1PI_MASK) >> MCAN_RXF1S_F1PI_SHIFT)

/*
 * F1GI (R)
 *
 * rx fifo 1 get index
 */
#define MCAN_RXF1S_F1GI_MASK (0x3F00U)
#define MCAN_RXF1S_F1GI_SHIFT (8U)
#define MCAN_RXF1S_F1GI_GET(x) (((uint32_t)(x) & MCAN_RXF1S_F1GI_MASK) >> MCAN_RXF1S_F1GI_SHIFT)

/*
 * F1FL (R)
 *
 * rx fifo 1 fill level
 */
#define MCAN_RXF1S_F1FL_MASK (0x7FU)
#define MCAN_RXF1S_F1FL_SHIFT (0U)
#define MCAN_RXF1S_F1FL_GET(x) (((uint32_t)(x) & MCAN_RXF1S_F1FL_MASK) >> MCAN_RXF1S_F1FL_SHIFT)

/* Bitfield definition for register: RXF1A */
/*
 * F1AI (RW)
 *
 * rx fifo1 acknowledge index
 */
#define MCAN_RXF1A_F1AI_MASK (0x3FU)
#define MCAN_RXF1A_F1AI_SHIFT (0U)
#define MCAN_RXF1A_F1AI_SET(x) (((uint32_t)(x) << MCAN_RXF1A_F1AI_SHIFT) & MCAN_RXF1A_F1AI_MASK)
#define MCAN_RXF1A_F1AI_GET(x) (((uint32_t)(x) & MCAN_RXF1A_F1AI_MASK) >> MCAN_RXF1A_F1AI_SHIFT)

/* Bitfield definition for register: RXESC */
/*
 * RBDS (RW)
 *
 * rx buffer data field size
 */
#define MCAN_RXESC_RBDS_MASK (0x700U)
#define MCAN_RXESC_RBDS_SHIFT (8U)
#define MCAN_RXESC_RBDS_SET(x) (((uint32_t)(x) << MCAN_RXESC_RBDS_SHIFT) & MCAN_RXESC_RBDS_MASK)
#define MCAN_RXESC_RBDS_GET(x) (((uint32_t)(x) & MCAN_RXESC_RBDS_MASK) >> MCAN_RXESC_RBDS_SHIFT)

/*
 * F1DS (RW)
 *
 * rx fifo 1 data field size
 */
#define MCAN_RXESC_F1DS_MASK (0x70U)
#define MCAN_RXESC_F1DS_SHIFT (4U)
#define MCAN_RXESC_F1DS_SET(x) (((uint32_t)(x) << MCAN_RXESC_F1DS_SHIFT) & MCAN_RXESC_F1DS_MASK)
#define MCAN_RXESC_F1DS_GET(x) (((uint32_t)(x) & MCAN_RXESC_F1DS_MASK) >> MCAN_RXESC_F1DS_SHIFT)

/*
 * F0DS (RW)
 *
 * rx fifo 0 data field size
 */
#define MCAN_RXESC_F0DS_MASK (0x7U)
#define MCAN_RXESC_F0DS_SHIFT (0U)
#define MCAN_RXESC_F0DS_SET(x) (((uint32_t)(x) << MCAN_RXESC_F0DS_SHIFT) & MCAN_RXESC_F0DS_MASK)
#define MCAN_RXESC_F0DS_GET(x) (((uint32_t)(x) & MCAN_RXESC_F0DS_MASK) >> MCAN_RXESC_F0DS_SHIFT)

/* Bitfield definition for register: TXBC */
/*
 * TFQM (RW)
 *
 * tx fifo/queue mode
 */
#define MCAN_TXBC_TFQM_MASK (0x40000000UL)
#define MCAN_TXBC_TFQM_SHIFT (30U)
#define MCAN_TXBC_TFQM_SET(x) (((uint32_t)(x) << MCAN_TXBC_TFQM_SHIFT) & MCAN_TXBC_TFQM_MASK)
#define MCAN_TXBC_TFQM_GET(x) (((uint32_t)(x) & MCAN_TXBC_TFQM_MASK) >> MCAN_TXBC_TFQM_SHIFT)

/*
 * TFQS (RW)
 *
 * transmit FIFO/queue size
 */
#define MCAN_TXBC_TFQS_MASK (0x3F000000UL)
#define MCAN_TXBC_TFQS_SHIFT (24U)
#define MCAN_TXBC_TFQS_SET(x) (((uint32_t)(x) << MCAN_TXBC_TFQS_SHIFT) & MCAN_TXBC_TFQS_MASK)
#define MCAN_TXBC_TFQS_GET(x) (((uint32_t)(x) & MCAN_TXBC_TFQS_MASK) >> MCAN_TXBC_TFQS_SHIFT)

/*
 * NDTB (RW)
 *
 * number of dedicated transmit buffers
 */
#define MCAN_TXBC_NDTB_MASK (0x3F0000UL)
#define MCAN_TXBC_NDTB_SHIFT (16U)
#define MCAN_TXBC_NDTB_SET(x) (((uint32_t)(x) << MCAN_TXBC_NDTB_SHIFT) & MCAN_TXBC_NDTB_MASK)
#define MCAN_TXBC_NDTB_GET(x) (((uint32_t)(x) & MCAN_TXBC_NDTB_MASK) >> MCAN_TXBC_NDTB_SHIFT)

/*
 * TBSA (RW)
 *
 * tx buffer start address
 */
#define MCAN_TXBC_TBSA_MASK (0xFFFCU)
#define MCAN_TXBC_TBSA_SHIFT (2U)
#define MCAN_TXBC_TBSA_SET(x) (((uint32_t)(x) << MCAN_TXBC_TBSA_SHIFT) & MCAN_TXBC_TBSA_MASK)
#define MCAN_TXBC_TBSA_GET(x) (((uint32_t)(x) & MCAN_TXBC_TBSA_MASK) >> MCAN_TXBC_TBSA_SHIFT)

/* Bitfield definition for register: TXFQS */
/*
 * TFQF (R)
 *
 * tx fifo/queue full
 */
#define MCAN_TXFQS_TFQF_MASK (0x200000UL)
#define MCAN_TXFQS_TFQF_SHIFT (21U)
#define MCAN_TXFQS_TFQF_GET(x) (((uint32_t)(x) & MCAN_TXFQS_TFQF_MASK) >> MCAN_TXFQS_TFQF_SHIFT)

/*
 * TFQPI (R)
 *
 * tx fifo/queue put index
 */
#define MCAN_TXFQS_TFQPI_MASK (0x1F0000UL)
#define MCAN_TXFQS_TFQPI_SHIFT (16U)
#define MCAN_TXFQS_TFQPI_GET(x) (((uint32_t)(x) & MCAN_TXFQS_TFQPI_MASK) >> MCAN_TXFQS_TFQPI_SHIFT)

/*
 * TFGI (R)
 *
 * tx fifo get index
 */
#define MCAN_TXFQS_TFGI_MASK (0x1F00U)
#define MCAN_TXFQS_TFGI_SHIFT (8U)
#define MCAN_TXFQS_TFGI_GET(x) (((uint32_t)(x) & MCAN_TXFQS_TFGI_MASK) >> MCAN_TXFQS_TFGI_SHIFT)

/*
 * TFFL (R)
 *
 * tx fifo free level
 */
#define MCAN_TXFQS_TFFL_MASK (0x3FU)
#define MCAN_TXFQS_TFFL_SHIFT (0U)
#define MCAN_TXFQS_TFFL_GET(x) (((uint32_t)(x) & MCAN_TXFQS_TFFL_MASK) >> MCAN_TXFQS_TFFL_SHIFT)

/* Bitfield definition for register: TXESC */
/*
 * TBDS (RW)
 *
 * tx buffer data field size
 */
#define MCAN_TXESC_TBDS_MASK (0x7U)
#define MCAN_TXESC_TBDS_SHIFT (0U)
#define MCAN_TXESC_TBDS_SET(x) (((uint32_t)(x) << MCAN_TXESC_TBDS_SHIFT) & MCAN_TXESC_TBDS_MASK)
#define MCAN_TXESC_TBDS_GET(x) (((uint32_t)(x) & MCAN_TXESC_TBDS_MASK) >> MCAN_TXESC_TBDS_SHIFT)

/* Bitfield definition for register: TXBRP */
/*
 * TRP (R)
 *
 * transmission request pending
 */
#define MCAN_TXBRP_TRP_MASK (0xFFFFFFFFUL)
#define MCAN_TXBRP_TRP_SHIFT (0U)
#define MCAN_TXBRP_TRP_GET(x) (((uint32_t)(x) & MCAN_TXBRP_TRP_MASK) >> MCAN_TXBRP_TRP_SHIFT)

/* Bitfield definition for register: TXBAR */
/*
 * AR (RW)
 *
 * add request
 */
#define MCAN_TXBAR_AR_MASK (0xFFFFFFFFUL)
#define MCAN_TXBAR_AR_SHIFT (0U)
#define MCAN_TXBAR_AR_SET(x) (((uint32_t)(x) << MCAN_TXBAR_AR_SHIFT) & MCAN_TXBAR_AR_MASK)
#define MCAN_TXBAR_AR_GET(x) (((uint32_t)(x) & MCAN_TXBAR_AR_MASK) >> MCAN_TXBAR_AR_SHIFT)

/* Bitfield definition for register: TXBCR */
/*
 * CR (RW)
 *
 * cancellation request
 */
#define MCAN_TXBCR_CR_MASK (0xFFFFFFFFUL)
#define MCAN_TXBCR_CR_SHIFT (0U)
#define MCAN_TXBCR_CR_SET(x) (((uint32_t)(x) << MCAN_TXBCR_CR_SHIFT) & MCAN_TXBCR_CR_MASK)
#define MCAN_TXBCR_CR_GET(x) (((uint32_t)(x) & MCAN_TXBCR_CR_MASK) >> MCAN_TXBCR_CR_SHIFT)

/* Bitfield definition for register: TXBTO */
/*
 * TO (R)
 *
 * transmission occurred
 */
#define MCAN_TXBTO_TO_MASK (0xFFFFFFFFUL)
#define MCAN_TXBTO_TO_SHIFT (0U)
#define MCAN_TXBTO_TO_GET(x) (((uint32_t)(x) & MCAN_TXBTO_TO_MASK) >> MCAN_TXBTO_TO_SHIFT)

/* Bitfield definition for register: TXBCF */
/*
 * CF (R)
 *
 * cancellation finished
 */
#define MCAN_TXBCF_CF_MASK (0xFFFFFFFFUL)
#define MCAN_TXBCF_CF_SHIFT (0U)
#define MCAN_TXBCF_CF_GET(x) (((uint32_t)(x) & MCAN_TXBCF_CF_MASK) >> MCAN_TXBCF_CF_SHIFT)

/* Bitfield definition for register: TXBTIE */
/*
 * TIE (RW)
 *
 * transmission interrupt enable
 */
#define MCAN_TXBTIE_TIE_MASK (0xFFFFFFFFUL)
#define MCAN_TXBTIE_TIE_SHIFT (0U)
#define MCAN_TXBTIE_TIE_SET(x) (((uint32_t)(x) << MCAN_TXBTIE_TIE_SHIFT) & MCAN_TXBTIE_TIE_MASK)
#define MCAN_TXBTIE_TIE_GET(x) (((uint32_t)(x) & MCAN_TXBTIE_TIE_MASK) >> MCAN_TXBTIE_TIE_SHIFT)

/* Bitfield definition for register: TXBCIE */
/*
 * CFIE (RW)
 *
 * cancellation finished interrupt enable
 */
#define MCAN_TXBCIE_CFIE_MASK (0xFFFFFFFFUL)
#define MCAN_TXBCIE_CFIE_SHIFT (0U)
#define MCAN_TXBCIE_CFIE_SET(x) (((uint32_t)(x) << MCAN_TXBCIE_CFIE_SHIFT) & MCAN_TXBCIE_CFIE_MASK)
#define MCAN_TXBCIE_CFIE_GET(x) (((uint32_t)(x) & MCAN_TXBCIE_CFIE_MASK) >> MCAN_TXBCIE_CFIE_SHIFT)

/* Bitfield definition for register: TXEFC */
/*
 * EFWM (RW)
 *
 * event fifo watermark
 */
#define MCAN_TXEFC_EFWM_MASK (0x3F000000UL)
#define MCAN_TXEFC_EFWM_SHIFT (24U)
#define MCAN_TXEFC_EFWM_SET(x) (((uint32_t)(x) << MCAN_TXEFC_EFWM_SHIFT) & MCAN_TXEFC_EFWM_MASK)
#define MCAN_TXEFC_EFWM_GET(x) (((uint32_t)(x) & MCAN_TXEFC_EFWM_MASK) >> MCAN_TXEFC_EFWM_SHIFT)

/*
 * EFS (RW)
 *
 * event fifo size
 */
#define MCAN_TXEFC_EFS_MASK (0x3F0000UL)
#define MCAN_TXEFC_EFS_SHIFT (16U)
#define MCAN_TXEFC_EFS_SET(x) (((uint32_t)(x) << MCAN_TXEFC_EFS_SHIFT) & MCAN_TXEFC_EFS_MASK)
#define MCAN_TXEFC_EFS_GET(x) (((uint32_t)(x) & MCAN_TXEFC_EFS_MASK) >> MCAN_TXEFC_EFS_SHIFT)

/*
 * EFSA (RW)
 *
 * event fifo start address
 */
#define MCAN_TXEFC_EFSA_MASK (0xFFFCU)
#define MCAN_TXEFC_EFSA_SHIFT (2U)
#define MCAN_TXEFC_EFSA_SET(x) (((uint32_t)(x) << MCAN_TXEFC_EFSA_SHIFT) & MCAN_TXEFC_EFSA_MASK)
#define MCAN_TXEFC_EFSA_GET(x) (((uint32_t)(x) & MCAN_TXEFC_EFSA_MASK) >> MCAN_TXEFC_EFSA_SHIFT)

/* Bitfield definition for register: TXEFS */
/*
 * TEFL (R)
 *
 * tx event fifo element lost
 */
#define MCAN_TXEFS_TEFL_MASK (0x2000000UL)
#define MCAN_TXEFS_TEFL_SHIFT (25U)
#define MCAN_TXEFS_TEFL_GET(x) (((uint32_t)(x) & MCAN_TXEFS_TEFL_MASK) >> MCAN_TXEFS_TEFL_SHIFT)

/*
 * EFF (R)
 *
 * event fifo full
 */
#define MCAN_TXEFS_EFF_MASK (0x1000000UL)
#define MCAN_TXEFS_EFF_SHIFT (24U)
#define MCAN_TXEFS_EFF_GET(x) (((uint32_t)(x) & MCAN_TXEFS_EFF_MASK) >> MCAN_TXEFS_EFF_SHIFT)

/*
 * EFPI (R)
 *
 * event fifo put index
 */
#define MCAN_TXEFS_EFPI_MASK (0x1F0000UL)
#define MCAN_TXEFS_EFPI_SHIFT (16U)
#define MCAN_TXEFS_EFPI_GET(x) (((uint32_t)(x) & MCAN_TXEFS_EFPI_MASK) >> MCAN_TXEFS_EFPI_SHIFT)

/*
 * EFGI (R)
 *
 * event fifo get index
 */
#define MCAN_TXEFS_EFGI_MASK (0x1F00U)
#define MCAN_TXEFS_EFGI_SHIFT (8U)
#define MCAN_TXEFS_EFGI_GET(x) (((uint32_t)(x) & MCAN_TXEFS_EFGI_MASK) >> MCAN_TXEFS_EFGI_SHIFT)

/*
 * EFFL (R)
 *
 * event fifo fill level
 */
#define MCAN_TXEFS_EFFL_MASK (0x3FU)
#define MCAN_TXEFS_EFFL_SHIFT (0U)
#define MCAN_TXEFS_EFFL_GET(x) (((uint32_t)(x) & MCAN_TXEFS_EFFL_MASK) >> MCAN_TXEFS_EFFL_SHIFT)

/* Bitfield definition for register: TXEFA */
/*
 * EFAI (RW)
 *
 * event fifo acknowledge index
 */
#define MCAN_TXEFA_EFAI_MASK (0x1FU)
#define MCAN_TXEFA_EFAI_SHIFT (0U)
#define MCAN_TXEFA_EFAI_SET(x) (((uint32_t)(x) << MCAN_TXEFA_EFAI_SHIFT) & MCAN_TXEFA_EFAI_MASK)
#define MCAN_TXEFA_EFAI_GET(x) (((uint32_t)(x) & MCAN_TXEFA_EFAI_MASK) >> MCAN_TXEFA_EFAI_SHIFT)

/* Bitfield definition for register array: TS_SEL */
/*
 * TS (R)
 *
 * timestamp word
 */
#define MCAN_TS_SEL_TS_MASK (0xFFFFFFFFUL)
#define MCAN_TS_SEL_TS_SHIFT (0U)
#define MCAN_TS_SEL_TS_GET(x) (((uint32_t)(x) & MCAN_TS_SEL_TS_MASK) >> MCAN_TS_SEL_TS_SHIFT)

/* Bitfield definition for register: CREL */
/*
 * REL (R)
 *
 * core release
 */
#define MCAN_CREL_REL_MASK (0xF0000000UL)
#define MCAN_CREL_REL_SHIFT (28U)
#define MCAN_CREL_REL_GET(x) (((uint32_t)(x) & MCAN_CREL_REL_MASK) >> MCAN_CREL_REL_SHIFT)

/*
 * STEP (R)
 *
 * step of core release
 */
#define MCAN_CREL_STEP_MASK (0xF000000UL)
#define MCAN_CREL_STEP_SHIFT (24U)
#define MCAN_CREL_STEP_GET(x) (((uint32_t)(x) & MCAN_CREL_STEP_MASK) >> MCAN_CREL_STEP_SHIFT)

/*
 * SUBSTEP (R)
 *
 * sub-step of core release
 */
#define MCAN_CREL_SUBSTEP_MASK (0xF00000UL)
#define MCAN_CREL_SUBSTEP_SHIFT (20U)
#define MCAN_CREL_SUBSTEP_GET(x) (((uint32_t)(x) & MCAN_CREL_SUBSTEP_MASK) >> MCAN_CREL_SUBSTEP_SHIFT)

/*
 * YEAR (R)
 *
 * timestamp year
 */
#define MCAN_CREL_YEAR_MASK (0xF0000UL)
#define MCAN_CREL_YEAR_SHIFT (16U)
#define MCAN_CREL_YEAR_GET(x) (((uint32_t)(x) & MCAN_CREL_YEAR_MASK) >> MCAN_CREL_YEAR_SHIFT)

/*
 * MON (R)
 *
 * timestamp month
 */
#define MCAN_CREL_MON_MASK (0xFF00U)
#define MCAN_CREL_MON_SHIFT (8U)
#define MCAN_CREL_MON_GET(x) (((uint32_t)(x) & MCAN_CREL_MON_MASK) >> MCAN_CREL_MON_SHIFT)

/*
 * DAY (R)
 *
 * timestamp day
 */
#define MCAN_CREL_DAY_MASK (0xFFU)
#define MCAN_CREL_DAY_SHIFT (0U)
#define MCAN_CREL_DAY_GET(x) (((uint32_t)(x) & MCAN_CREL_DAY_MASK) >> MCAN_CREL_DAY_SHIFT)

/* Bitfield definition for register: TSCFG */
/*
 * TBPRE (RW)
 *
 * timebase prescaler, based on AHB clock
 */
#define MCAN_TSCFG_TBPRE_MASK (0xFF00U)
#define MCAN_TSCFG_TBPRE_SHIFT (8U)
#define MCAN_TSCFG_TBPRE_SET(x) (((uint32_t)(x) << MCAN_TSCFG_TBPRE_SHIFT) & MCAN_TSCFG_TBPRE_MASK)
#define MCAN_TSCFG_TBPRE_GET(x) (((uint32_t)(x) & MCAN_TSCFG_TBPRE_MASK) >> MCAN_TSCFG_TBPRE_SHIFT)

/*
 * EN64 (RW)
 *
 * set to use 64bit timestamp.
 * when enabled, tsu can save up to 8 different timestamps, TS(k) and TS(k+1) are used for one 64bit timestamp, k is 0~7.
 * TSP can be used to select different one
 */
#define MCAN_TSCFG_EN64_MASK (0x8U)
#define MCAN_TSCFG_EN64_SHIFT (3U)
#define MCAN_TSCFG_EN64_SET(x) (((uint32_t)(x) << MCAN_TSCFG_EN64_SHIFT) & MCAN_TSCFG_EN64_MASK)
#define MCAN_TSCFG_EN64_GET(x) (((uint32_t)(x) & MCAN_TSCFG_EN64_MASK) >> MCAN_TSCFG_EN64_SHIFT)

/*
 * SCP (RW)
 *
 * select capturing position
 */
#define MCAN_TSCFG_SCP_MASK (0x4U)
#define MCAN_TSCFG_SCP_SHIFT (2U)
#define MCAN_TSCFG_SCP_SET(x) (((uint32_t)(x) << MCAN_TSCFG_SCP_SHIFT) & MCAN_TSCFG_SCP_MASK)
#define MCAN_TSCFG_SCP_GET(x) (((uint32_t)(x) & MCAN_TSCFG_SCP_MASK) >> MCAN_TSCFG_SCP_SHIFT)

/*
 * TBCS (RW)
 *
 * timebase counter select
 */
#define MCAN_TSCFG_TBCS_MASK (0x2U)
#define MCAN_TSCFG_TBCS_SHIFT (1U)
#define MCAN_TSCFG_TBCS_SET(x) (((uint32_t)(x) << MCAN_TSCFG_TBCS_SHIFT) & MCAN_TSCFG_TBCS_MASK)
#define MCAN_TSCFG_TBCS_GET(x) (((uint32_t)(x) & MCAN_TSCFG_TBCS_MASK) >> MCAN_TSCFG_TBCS_SHIFT)

/*
 * TSUE (RW)
 *
 * timestamp unit enable
 */
#define MCAN_TSCFG_TSUE_MASK (0x1U)
#define MCAN_TSCFG_TSUE_SHIFT (0U)
#define MCAN_TSCFG_TSUE_SET(x) (((uint32_t)(x) << MCAN_TSCFG_TSUE_SHIFT) & MCAN_TSCFG_TSUE_MASK)
#define MCAN_TSCFG_TSUE_GET(x) (((uint32_t)(x) & MCAN_TSCFG_TSUE_MASK) >> MCAN_TSCFG_TSUE_SHIFT)

/* Bitfield definition for register: TSS1 */
/*
 * TSL (R)
 *
 * timestamp lost
 */
#define MCAN_TSS1_TSL_MASK (0xFFFF0000UL)
#define MCAN_TSS1_TSL_SHIFT (16U)
#define MCAN_TSS1_TSL_GET(x) (((uint32_t)(x) & MCAN_TSS1_TSL_MASK) >> MCAN_TSS1_TSL_SHIFT)

/*
 * TSN (R)
 *
 * timestamp new
 */
#define MCAN_TSS1_TSN_MASK (0xFFFFU)
#define MCAN_TSS1_TSN_SHIFT (0U)
#define MCAN_TSS1_TSN_GET(x) (((uint32_t)(x) & MCAN_TSS1_TSN_MASK) >> MCAN_TSS1_TSN_SHIFT)

/* Bitfield definition for register: TSS2 */
/*
 * ITBG (R)
 *
 * internal timebase and sof select generic
 */
#define MCAN_TSS2_ITBG_MASK (0xC000U)
#define MCAN_TSS2_ITBG_SHIFT (14U)
#define MCAN_TSS2_ITBG_GET(x) (((uint32_t)(x) & MCAN_TSS2_ITBG_MASK) >> MCAN_TSS2_ITBG_SHIFT)

/*
 * NTSG (R)
 *
 * number of timestamps generic
 */
#define MCAN_TSS2_NTSG_MASK (0x3000U)
#define MCAN_TSS2_NTSG_SHIFT (12U)
#define MCAN_TSS2_NTSG_GET(x) (((uint32_t)(x) & MCAN_TSS2_NTSG_MASK) >> MCAN_TSS2_NTSG_SHIFT)

/*
 * TSP (R)
 *
 * timestamp pointer
 */
#define MCAN_TSS2_TSP_MASK (0xFU)
#define MCAN_TSS2_TSP_SHIFT (0U)
#define MCAN_TSS2_TSP_GET(x) (((uint32_t)(x) & MCAN_TSS2_TSP_MASK) >> MCAN_TSS2_TSP_SHIFT)

/* Bitfield definition for register: ATB */
/*
 * TB (RC)
 *
 * timebase for timestamp generation 31-0
 */
#define MCAN_ATB_TB_MASK (0xFFFFFFFFUL)
#define MCAN_ATB_TB_SHIFT (0U)
#define MCAN_ATB_TB_GET(x) (((uint32_t)(x) & MCAN_ATB_TB_MASK) >> MCAN_ATB_TB_SHIFT)

/* Bitfield definition for register: ATBH */
/*
 * TBH (RC)
 *
 * timebase for timestamp generation 63-32
 */
#define MCAN_ATBH_TBH_MASK (0xFFFFFFFFUL)
#define MCAN_ATBH_TBH_SHIFT (0U)
#define MCAN_ATBH_TBH_GET(x) (((uint32_t)(x) & MCAN_ATBH_TBH_MASK) >> MCAN_ATBH_TBH_SHIFT)

/* Bitfield definition for register: GLB_CTL */
/*
 * M_CAN_STBY (RW)
 *
 */
#define MCAN_GLB_CTL_M_CAN_STBY_MASK (0x80000000UL)
#define MCAN_GLB_CTL_M_CAN_STBY_SHIFT (31U)
#define MCAN_GLB_CTL_M_CAN_STBY_SET(x) (((uint32_t)(x) << MCAN_GLB_CTL_M_CAN_STBY_SHIFT) & MCAN_GLB_CTL_M_CAN_STBY_MASK)
#define MCAN_GLB_CTL_M_CAN_STBY_GET(x) (((uint32_t)(x) & MCAN_GLB_CTL_M_CAN_STBY_MASK) >> MCAN_GLB_CTL_M_CAN_STBY_SHIFT)

/*
 * STBY_CLR_EN (RW)
 *
 */
#define MCAN_GLB_CTL_STBY_CLR_EN_MASK (0x40000000UL)
#define MCAN_GLB_CTL_STBY_CLR_EN_SHIFT (30U)
#define MCAN_GLB_CTL_STBY_CLR_EN_SET(x) (((uint32_t)(x) << MCAN_GLB_CTL_STBY_CLR_EN_SHIFT) & MCAN_GLB_CTL_STBY_CLR_EN_MASK)
#define MCAN_GLB_CTL_STBY_CLR_EN_GET(x) (((uint32_t)(x) & MCAN_GLB_CTL_STBY_CLR_EN_MASK) >> MCAN_GLB_CTL_STBY_CLR_EN_SHIFT)

/*
 * STBY_POL (RW)
 *
 */
#define MCAN_GLB_CTL_STBY_POL_MASK (0x20000000UL)
#define MCAN_GLB_CTL_STBY_POL_SHIFT (29U)
#define MCAN_GLB_CTL_STBY_POL_SET(x) (((uint32_t)(x) << MCAN_GLB_CTL_STBY_POL_SHIFT) & MCAN_GLB_CTL_STBY_POL_MASK)
#define MCAN_GLB_CTL_STBY_POL_GET(x) (((uint32_t)(x) & MCAN_GLB_CTL_STBY_POL_MASK) >> MCAN_GLB_CTL_STBY_POL_SHIFT)

/*
 * M_CAN_DIS_MORD (RW)
 *
 */
#define MCAN_GLB_CTL_M_CAN_DIS_MORD_MASK (0x8U)
#define MCAN_GLB_CTL_M_CAN_DIS_MORD_SHIFT (3U)
#define MCAN_GLB_CTL_M_CAN_DIS_MORD_SET(x) (((uint32_t)(x) << MCAN_GLB_CTL_M_CAN_DIS_MORD_SHIFT) & MCAN_GLB_CTL_M_CAN_DIS_MORD_MASK)
#define MCAN_GLB_CTL_M_CAN_DIS_MORD_GET(x) (((uint32_t)(x) & MCAN_GLB_CTL_M_CAN_DIS_MORD_MASK) >> MCAN_GLB_CTL_M_CAN_DIS_MORD_SHIFT)

/*
 * TSU_TBIN_SEL (RW)
 *
 */
#define MCAN_GLB_CTL_TSU_TBIN_SEL_MASK (0x7U)
#define MCAN_GLB_CTL_TSU_TBIN_SEL_SHIFT (0U)
#define MCAN_GLB_CTL_TSU_TBIN_SEL_SET(x) (((uint32_t)(x) << MCAN_GLB_CTL_TSU_TBIN_SEL_SHIFT) & MCAN_GLB_CTL_TSU_TBIN_SEL_MASK)
#define MCAN_GLB_CTL_TSU_TBIN_SEL_GET(x) (((uint32_t)(x) & MCAN_GLB_CTL_TSU_TBIN_SEL_MASK) >> MCAN_GLB_CTL_TSU_TBIN_SEL_SHIFT)

/* Bitfield definition for register: GLB_STATUS */
/*
 * M_CAN_INT1 (R)
 *
 */
#define MCAN_GLB_STATUS_M_CAN_INT1_MASK (0x8U)
#define MCAN_GLB_STATUS_M_CAN_INT1_SHIFT (3U)
#define MCAN_GLB_STATUS_M_CAN_INT1_GET(x) (((uint32_t)(x) & MCAN_GLB_STATUS_M_CAN_INT1_MASK) >> MCAN_GLB_STATUS_M_CAN_INT1_SHIFT)

/*
 * M_CAN_INT0 (R)
 *
 */
#define MCAN_GLB_STATUS_M_CAN_INT0_MASK (0x4U)
#define MCAN_GLB_STATUS_M_CAN_INT0_SHIFT (2U)
#define MCAN_GLB_STATUS_M_CAN_INT0_GET(x) (((uint32_t)(x) & MCAN_GLB_STATUS_M_CAN_INT0_MASK) >> MCAN_GLB_STATUS_M_CAN_INT0_SHIFT)

/*
 * M_CAN_AEI_ARA (R)
 *
 */
#define MCAN_GLB_STATUS_M_CAN_AEI_ARA_MASK (0x2U)
#define MCAN_GLB_STATUS_M_CAN_AEI_ARA_SHIFT (1U)
#define MCAN_GLB_STATUS_M_CAN_AEI_ARA_GET(x) (((uint32_t)(x) & MCAN_GLB_STATUS_M_CAN_AEI_ARA_MASK) >> MCAN_GLB_STATUS_M_CAN_AEI_ARA_SHIFT)

/*
 * M_CAN_ACT_TX (R)
 *
 */
#define MCAN_GLB_STATUS_M_CAN_ACT_TX_MASK (0x1U)
#define MCAN_GLB_STATUS_M_CAN_ACT_TX_SHIFT (0U)
#define MCAN_GLB_STATUS_M_CAN_ACT_TX_GET(x) (((uint32_t)(x) & MCAN_GLB_STATUS_M_CAN_ACT_TX_MASK) >> MCAN_GLB_STATUS_M_CAN_ACT_TX_SHIFT)

/* Bitfield definition for register: GLB_CAN_IR */
/*
 * M_CAN_IR (R)
 *
 */
#define MCAN_GLB_CAN_IR_M_CAN_IR_MASK (0xFFFFFFFFUL)
#define MCAN_GLB_CAN_IR_M_CAN_IR_SHIFT (0U)
#define MCAN_GLB_CAN_IR_M_CAN_IR_GET(x) (((uint32_t)(x) & MCAN_GLB_CAN_IR_M_CAN_IR_MASK) >> MCAN_GLB_CAN_IR_M_CAN_IR_SHIFT)

/* Bitfield definition for register array: MESSAGE_BUFF */
/*
 * DATA (RW)
 *
 */
#define MCAN_MESSAGE_BUFF_DATA_MASK (0xFFFFFFFFUL)
#define MCAN_MESSAGE_BUFF_DATA_SHIFT (0U)
#define MCAN_MESSAGE_BUFF_DATA_SET(x) (((uint32_t)(x) << MCAN_MESSAGE_BUFF_DATA_SHIFT) & MCAN_MESSAGE_BUFF_DATA_MASK)
#define MCAN_MESSAGE_BUFF_DATA_GET(x) (((uint32_t)(x) & MCAN_MESSAGE_BUFF_DATA_MASK) >> MCAN_MESSAGE_BUFF_DATA_SHIFT)



/* TS_SEL register group index macro definition */
#define MCAN_TS_SEL_TS_SEL0 (0UL)
#define MCAN_TS_SEL_TS_SEL1 (1UL)
#define MCAN_TS_SEL_TS_SEL2 (2UL)
#define MCAN_TS_SEL_TS_SEL3 (3UL)
#define MCAN_TS_SEL_TS_SEL4 (4UL)
#define MCAN_TS_SEL_TS_SEL5 (5UL)
#define MCAN_TS_SEL_TS_SEL6 (6UL)
#define MCAN_TS_SEL_TS_SEL7 (7UL)
#define MCAN_TS_SEL_TS_SEL8 (8UL)
#define MCAN_TS_SEL_TS_SEL9 (9UL)
#define MCAN_TS_SEL_TS_SEL10 (10UL)
#define MCAN_TS_SEL_TS_SEL11 (11UL)
#define MCAN_TS_SEL_TS_SEL12 (12UL)
#define MCAN_TS_SEL_TS_SEL13 (13UL)
#define MCAN_TS_SEL_TS_SEL14 (14UL)
#define MCAN_TS_SEL_TS_SEL15 (15UL)

/* MESSAGE_BUFF register group index macro definition */
#define MCAN_MESSAGE_BUFF_0 (0UL)
#define MCAN_MESSAGE_BUFF_1 (1UL)
#define MCAN_MESSAGE_BUFF_2 (2UL)
#define MCAN_MESSAGE_BUFF_3 (3UL)
#define MCAN_MESSAGE_BUFF_4 (4UL)
#define MCAN_MESSAGE_BUFF_5 (5UL)
#define MCAN_MESSAGE_BUFF_6 (6UL)
#define MCAN_MESSAGE_BUFF_7 (7UL)
#define MCAN_MESSAGE_BUFF_8 (8UL)
#define MCAN_MESSAGE_BUFF_9 (9UL)
#define MCAN_MESSAGE_BUFF_10 (10UL)
#define MCAN_MESSAGE_BUFF_11 (11UL)
#define MCAN_MESSAGE_BUFF_12 (12UL)
#define MCAN_MESSAGE_BUFF_13 (13UL)
#define MCAN_MESSAGE_BUFF_14 (14UL)
#define MCAN_MESSAGE_BUFF_15 (15UL)
#define MCAN_MESSAGE_BUFF_16 (16UL)
#define MCAN_MESSAGE_BUFF_17 (17UL)
#define MCAN_MESSAGE_BUFF_18 (18UL)
#define MCAN_MESSAGE_BUFF_19 (19UL)
#define MCAN_MESSAGE_BUFF_20 (20UL)
#define MCAN_MESSAGE_BUFF_21 (21UL)
#define MCAN_MESSAGE_BUFF_22 (22UL)
#define MCAN_MESSAGE_BUFF_23 (23UL)
#define MCAN_MESSAGE_BUFF_24 (24UL)
#define MCAN_MESSAGE_BUFF_25 (25UL)
#define MCAN_MESSAGE_BUFF_26 (26UL)
#define MCAN_MESSAGE_BUFF_27 (27UL)
#define MCAN_MESSAGE_BUFF_28 (28UL)
#define MCAN_MESSAGE_BUFF_29 (29UL)
#define MCAN_MESSAGE_BUFF_30 (30UL)
#define MCAN_MESSAGE_BUFF_31 (31UL)
#define MCAN_MESSAGE_BUFF_32 (32UL)
#define MCAN_MESSAGE_BUFF_33 (33UL)
#define MCAN_MESSAGE_BUFF_34 (34UL)
#define MCAN_MESSAGE_BUFF_35 (35UL)
#define MCAN_MESSAGE_BUFF_36 (36UL)
#define MCAN_MESSAGE_BUFF_37 (37UL)
#define MCAN_MESSAGE_BUFF_38 (38UL)
#define MCAN_MESSAGE_BUFF_39 (39UL)
#define MCAN_MESSAGE_BUFF_40 (40UL)
#define MCAN_MESSAGE_BUFF_41 (41UL)
#define MCAN_MESSAGE_BUFF_42 (42UL)
#define MCAN_MESSAGE_BUFF_43 (43UL)
#define MCAN_MESSAGE_BUFF_44 (44UL)
#define MCAN_MESSAGE_BUFF_45 (45UL)
#define MCAN_MESSAGE_BUFF_46 (46UL)
#define MCAN_MESSAGE_BUFF_47 (47UL)
#define MCAN_MESSAGE_BUFF_48 (48UL)
#define MCAN_MESSAGE_BUFF_49 (49UL)
#define MCAN_MESSAGE_BUFF_50 (50UL)
#define MCAN_MESSAGE_BUFF_51 (51UL)
#define MCAN_MESSAGE_BUFF_52 (52UL)
#define MCAN_MESSAGE_BUFF_53 (53UL)
#define MCAN_MESSAGE_BUFF_54 (54UL)
#define MCAN_MESSAGE_BUFF_55 (55UL)
#define MCAN_MESSAGE_BUFF_56 (56UL)
#define MCAN_MESSAGE_BUFF_57 (57UL)
#define MCAN_MESSAGE_BUFF_58 (58UL)
#define MCAN_MESSAGE_BUFF_59 (59UL)
#define MCAN_MESSAGE_BUFF_60 (60UL)
#define MCAN_MESSAGE_BUFF_61 (61UL)
#define MCAN_MESSAGE_BUFF_62 (62UL)
#define MCAN_MESSAGE_BUFF_63 (63UL)
#define MCAN_MESSAGE_BUFF_64 (64UL)
#define MCAN_MESSAGE_BUFF_65 (65UL)
#define MCAN_MESSAGE_BUFF_66 (66UL)
#define MCAN_MESSAGE_BUFF_67 (67UL)
#define MCAN_MESSAGE_BUFF_68 (68UL)
#define MCAN_MESSAGE_BUFF_69 (69UL)
#define MCAN_MESSAGE_BUFF_70 (70UL)
#define MCAN_MESSAGE_BUFF_71 (71UL)
#define MCAN_MESSAGE_BUFF_72 (72UL)
#define MCAN_MESSAGE_BUFF_73 (73UL)
#define MCAN_MESSAGE_BUFF_74 (74UL)
#define MCAN_MESSAGE_BUFF_75 (75UL)
#define MCAN_MESSAGE_BUFF_76 (76UL)
#define MCAN_MESSAGE_BUFF_77 (77UL)
#define MCAN_MESSAGE_BUFF_78 (78UL)
#define MCAN_MESSAGE_BUFF_79 (79UL)
#define MCAN_MESSAGE_BUFF_80 (80UL)
#define MCAN_MESSAGE_BUFF_81 (81UL)
#define MCAN_MESSAGE_BUFF_82 (82UL)
#define MCAN_MESSAGE_BUFF_83 (83UL)
#define MCAN_MESSAGE_BUFF_84 (84UL)
#define MCAN_MESSAGE_BUFF_85 (85UL)
#define MCAN_MESSAGE_BUFF_86 (86UL)
#define MCAN_MESSAGE_BUFF_87 (87UL)
#define MCAN_MESSAGE_BUFF_88 (88UL)
#define MCAN_MESSAGE_BUFF_89 (89UL)
#define MCAN_MESSAGE_BUFF_90 (90UL)
#define MCAN_MESSAGE_BUFF_91 (91UL)
#define MCAN_MESSAGE_BUFF_92 (92UL)
#define MCAN_MESSAGE_BUFF_93 (93UL)
#define MCAN_MESSAGE_BUFF_94 (94UL)
#define MCAN_MESSAGE_BUFF_95 (95UL)
#define MCAN_MESSAGE_BUFF_96 (96UL)
#define MCAN_MESSAGE_BUFF_97 (97UL)
#define MCAN_MESSAGE_BUFF_98 (98UL)
#define MCAN_MESSAGE_BUFF_99 (99UL)
#define MCAN_MESSAGE_BUFF_100 (100UL)
#define MCAN_MESSAGE_BUFF_101 (101UL)
#define MCAN_MESSAGE_BUFF_102 (102UL)
#define MCAN_MESSAGE_BUFF_103 (103UL)
#define MCAN_MESSAGE_BUFF_104 (104UL)
#define MCAN_MESSAGE_BUFF_105 (105UL)
#define MCAN_MESSAGE_BUFF_106 (106UL)
#define MCAN_MESSAGE_BUFF_107 (107UL)
#define MCAN_MESSAGE_BUFF_108 (108UL)
#define MCAN_MESSAGE_BUFF_109 (109UL)
#define MCAN_MESSAGE_BUFF_110 (110UL)
#define MCAN_MESSAGE_BUFF_111 (111UL)
#define MCAN_MESSAGE_BUFF_112 (112UL)
#define MCAN_MESSAGE_BUFF_113 (113UL)
#define MCAN_MESSAGE_BUFF_114 (114UL)
#define MCAN_MESSAGE_BUFF_115 (115UL)
#define MCAN_MESSAGE_BUFF_116 (116UL)
#define MCAN_MESSAGE_BUFF_117 (117UL)
#define MCAN_MESSAGE_BUFF_118 (118UL)
#define MCAN_MESSAGE_BUFF_119 (119UL)
#define MCAN_MESSAGE_BUFF_120 (120UL)
#define MCAN_MESSAGE_BUFF_121 (121UL)
#define MCAN_MESSAGE_BUFF_122 (122UL)
#define MCAN_MESSAGE_BUFF_123 (123UL)
#define MCAN_MESSAGE_BUFF_124 (124UL)
#define MCAN_MESSAGE_BUFF_125 (125UL)
#define MCAN_MESSAGE_BUFF_126 (126UL)
#define MCAN_MESSAGE_BUFF_127 (127UL)
#define MCAN_MESSAGE_BUFF_128 (128UL)
#define MCAN_MESSAGE_BUFF_129 (129UL)
#define MCAN_MESSAGE_BUFF_130 (130UL)
#define MCAN_MESSAGE_BUFF_131 (131UL)
#define MCAN_MESSAGE_BUFF_132 (132UL)
#define MCAN_MESSAGE_BUFF_133 (133UL)
#define MCAN_MESSAGE_BUFF_134 (134UL)
#define MCAN_MESSAGE_BUFF_135 (135UL)
#define MCAN_MESSAGE_BUFF_136 (136UL)
#define MCAN_MESSAGE_BUFF_137 (137UL)
#define MCAN_MESSAGE_BUFF_138 (138UL)
#define MCAN_MESSAGE_BUFF_139 (139UL)
#define MCAN_MESSAGE_BUFF_140 (140UL)
#define MCAN_MESSAGE_BUFF_141 (141UL)
#define MCAN_MESSAGE_BUFF_142 (142UL)
#define MCAN_MESSAGE_BUFF_143 (143UL)
#define MCAN_MESSAGE_BUFF_144 (144UL)
#define MCAN_MESSAGE_BUFF_145 (145UL)
#define MCAN_MESSAGE_BUFF_146 (146UL)
#define MCAN_MESSAGE_BUFF_147 (147UL)
#define MCAN_MESSAGE_BUFF_148 (148UL)
#define MCAN_MESSAGE_BUFF_149 (149UL)
#define MCAN_MESSAGE_BUFF_150 (150UL)
#define MCAN_MESSAGE_BUFF_151 (151UL)
#define MCAN_MESSAGE_BUFF_152 (152UL)
#define MCAN_MESSAGE_BUFF_153 (153UL)
#define MCAN_MESSAGE_BUFF_154 (154UL)
#define MCAN_MESSAGE_BUFF_155 (155UL)
#define MCAN_MESSAGE_BUFF_156 (156UL)
#define MCAN_MESSAGE_BUFF_157 (157UL)
#define MCAN_MESSAGE_BUFF_158 (158UL)
#define MCAN_MESSAGE_BUFF_159 (159UL)
#define MCAN_MESSAGE_BUFF_160 (160UL)
#define MCAN_MESSAGE_BUFF_161 (161UL)
#define MCAN_MESSAGE_BUFF_162 (162UL)
#define MCAN_MESSAGE_BUFF_163 (163UL)
#define MCAN_MESSAGE_BUFF_164 (164UL)
#define MCAN_MESSAGE_BUFF_165 (165UL)
#define MCAN_MESSAGE_BUFF_166 (166UL)
#define MCAN_MESSAGE_BUFF_167 (167UL)
#define MCAN_MESSAGE_BUFF_168 (168UL)
#define MCAN_MESSAGE_BUFF_169 (169UL)
#define MCAN_MESSAGE_BUFF_170 (170UL)
#define MCAN_MESSAGE_BUFF_171 (171UL)
#define MCAN_MESSAGE_BUFF_172 (172UL)
#define MCAN_MESSAGE_BUFF_173 (173UL)
#define MCAN_MESSAGE_BUFF_174 (174UL)
#define MCAN_MESSAGE_BUFF_175 (175UL)
#define MCAN_MESSAGE_BUFF_176 (176UL)
#define MCAN_MESSAGE_BUFF_177 (177UL)
#define MCAN_MESSAGE_BUFF_178 (178UL)
#define MCAN_MESSAGE_BUFF_179 (179UL)
#define MCAN_MESSAGE_BUFF_180 (180UL)
#define MCAN_MESSAGE_BUFF_181 (181UL)
#define MCAN_MESSAGE_BUFF_182 (182UL)
#define MCAN_MESSAGE_BUFF_183 (183UL)
#define MCAN_MESSAGE_BUFF_184 (184UL)
#define MCAN_MESSAGE_BUFF_185 (185UL)
#define MCAN_MESSAGE_BUFF_186 (186UL)
#define MCAN_MESSAGE_BUFF_187 (187UL)
#define MCAN_MESSAGE_BUFF_188 (188UL)
#define MCAN_MESSAGE_BUFF_189 (189UL)
#define MCAN_MESSAGE_BUFF_190 (190UL)
#define MCAN_MESSAGE_BUFF_191 (191UL)
#define MCAN_MESSAGE_BUFF_192 (192UL)
#define MCAN_MESSAGE_BUFF_193 (193UL)
#define MCAN_MESSAGE_BUFF_194 (194UL)
#define MCAN_MESSAGE_BUFF_195 (195UL)
#define MCAN_MESSAGE_BUFF_196 (196UL)
#define MCAN_MESSAGE_BUFF_197 (197UL)
#define MCAN_MESSAGE_BUFF_198 (198UL)
#define MCAN_MESSAGE_BUFF_199 (199UL)
#define MCAN_MESSAGE_BUFF_200 (200UL)
#define MCAN_MESSAGE_BUFF_201 (201UL)
#define MCAN_MESSAGE_BUFF_202 (202UL)
#define MCAN_MESSAGE_BUFF_203 (203UL)
#define MCAN_MESSAGE_BUFF_204 (204UL)
#define MCAN_MESSAGE_BUFF_205 (205UL)
#define MCAN_MESSAGE_BUFF_206 (206UL)
#define MCAN_MESSAGE_BUFF_207 (207UL)
#define MCAN_MESSAGE_BUFF_208 (208UL)
#define MCAN_MESSAGE_BUFF_209 (209UL)
#define MCAN_MESSAGE_BUFF_210 (210UL)
#define MCAN_MESSAGE_BUFF_211 (211UL)
#define MCAN_MESSAGE_BUFF_212 (212UL)
#define MCAN_MESSAGE_BUFF_213 (213UL)
#define MCAN_MESSAGE_BUFF_214 (214UL)
#define MCAN_MESSAGE_BUFF_215 (215UL)
#define MCAN_MESSAGE_BUFF_216 (216UL)
#define MCAN_MESSAGE_BUFF_217 (217UL)
#define MCAN_MESSAGE_BUFF_218 (218UL)
#define MCAN_MESSAGE_BUFF_219 (219UL)
#define MCAN_MESSAGE_BUFF_220 (220UL)
#define MCAN_MESSAGE_BUFF_221 (221UL)
#define MCAN_MESSAGE_BUFF_222 (222UL)
#define MCAN_MESSAGE_BUFF_223 (223UL)
#define MCAN_MESSAGE_BUFF_224 (224UL)
#define MCAN_MESSAGE_BUFF_225 (225UL)
#define MCAN_MESSAGE_BUFF_226 (226UL)
#define MCAN_MESSAGE_BUFF_227 (227UL)
#define MCAN_MESSAGE_BUFF_228 (228UL)
#define MCAN_MESSAGE_BUFF_229 (229UL)
#define MCAN_MESSAGE_BUFF_230 (230UL)
#define MCAN_MESSAGE_BUFF_231 (231UL)
#define MCAN_MESSAGE_BUFF_232 (232UL)
#define MCAN_MESSAGE_BUFF_233 (233UL)
#define MCAN_MESSAGE_BUFF_234 (234UL)
#define MCAN_MESSAGE_BUFF_235 (235UL)
#define MCAN_MESSAGE_BUFF_236 (236UL)
#define MCAN_MESSAGE_BUFF_237 (237UL)
#define MCAN_MESSAGE_BUFF_238 (238UL)
#define MCAN_MESSAGE_BUFF_239 (239UL)
#define MCAN_MESSAGE_BUFF_240 (240UL)
#define MCAN_MESSAGE_BUFF_241 (241UL)
#define MCAN_MESSAGE_BUFF_242 (242UL)
#define MCAN_MESSAGE_BUFF_243 (243UL)
#define MCAN_MESSAGE_BUFF_244 (244UL)
#define MCAN_MESSAGE_BUFF_245 (245UL)
#define MCAN_MESSAGE_BUFF_246 (246UL)
#define MCAN_MESSAGE_BUFF_247 (247UL)
#define MCAN_MESSAGE_BUFF_248 (248UL)
#define MCAN_MESSAGE_BUFF_249 (249UL)
#define MCAN_MESSAGE_BUFF_250 (250UL)
#define MCAN_MESSAGE_BUFF_251 (251UL)
#define MCAN_MESSAGE_BUFF_252 (252UL)
#define MCAN_MESSAGE_BUFF_253 (253UL)
#define MCAN_MESSAGE_BUFF_254 (254UL)
#define MCAN_MESSAGE_BUFF_255 (255UL)
#define MCAN_MESSAGE_BUFF_256 (256UL)
#define MCAN_MESSAGE_BUFF_257 (257UL)
#define MCAN_MESSAGE_BUFF_258 (258UL)
#define MCAN_MESSAGE_BUFF_259 (259UL)
#define MCAN_MESSAGE_BUFF_260 (260UL)
#define MCAN_MESSAGE_BUFF_261 (261UL)
#define MCAN_MESSAGE_BUFF_262 (262UL)
#define MCAN_MESSAGE_BUFF_263 (263UL)
#define MCAN_MESSAGE_BUFF_264 (264UL)
#define MCAN_MESSAGE_BUFF_265 (265UL)
#define MCAN_MESSAGE_BUFF_266 (266UL)
#define MCAN_MESSAGE_BUFF_267 (267UL)
#define MCAN_MESSAGE_BUFF_268 (268UL)
#define MCAN_MESSAGE_BUFF_269 (269UL)
#define MCAN_MESSAGE_BUFF_270 (270UL)
#define MCAN_MESSAGE_BUFF_271 (271UL)
#define MCAN_MESSAGE_BUFF_272 (272UL)
#define MCAN_MESSAGE_BUFF_273 (273UL)
#define MCAN_MESSAGE_BUFF_274 (274UL)
#define MCAN_MESSAGE_BUFF_275 (275UL)
#define MCAN_MESSAGE_BUFF_276 (276UL)
#define MCAN_MESSAGE_BUFF_277 (277UL)
#define MCAN_MESSAGE_BUFF_278 (278UL)
#define MCAN_MESSAGE_BUFF_279 (279UL)
#define MCAN_MESSAGE_BUFF_280 (280UL)
#define MCAN_MESSAGE_BUFF_281 (281UL)
#define MCAN_MESSAGE_BUFF_282 (282UL)
#define MCAN_MESSAGE_BUFF_283 (283UL)
#define MCAN_MESSAGE_BUFF_284 (284UL)
#define MCAN_MESSAGE_BUFF_285 (285UL)
#define MCAN_MESSAGE_BUFF_286 (286UL)
#define MCAN_MESSAGE_BUFF_287 (287UL)
#define MCAN_MESSAGE_BUFF_288 (288UL)
#define MCAN_MESSAGE_BUFF_289 (289UL)
#define MCAN_MESSAGE_BUFF_290 (290UL)
#define MCAN_MESSAGE_BUFF_291 (291UL)
#define MCAN_MESSAGE_BUFF_292 (292UL)
#define MCAN_MESSAGE_BUFF_293 (293UL)
#define MCAN_MESSAGE_BUFF_294 (294UL)
#define MCAN_MESSAGE_BUFF_295 (295UL)
#define MCAN_MESSAGE_BUFF_296 (296UL)
#define MCAN_MESSAGE_BUFF_297 (297UL)
#define MCAN_MESSAGE_BUFF_298 (298UL)
#define MCAN_MESSAGE_BUFF_299 (299UL)
#define MCAN_MESSAGE_BUFF_300 (300UL)
#define MCAN_MESSAGE_BUFF_301 (301UL)
#define MCAN_MESSAGE_BUFF_302 (302UL)
#define MCAN_MESSAGE_BUFF_303 (303UL)
#define MCAN_MESSAGE_BUFF_304 (304UL)
#define MCAN_MESSAGE_BUFF_305 (305UL)
#define MCAN_MESSAGE_BUFF_306 (306UL)
#define MCAN_MESSAGE_BUFF_307 (307UL)
#define MCAN_MESSAGE_BUFF_308 (308UL)
#define MCAN_MESSAGE_BUFF_309 (309UL)
#define MCAN_MESSAGE_BUFF_310 (310UL)
#define MCAN_MESSAGE_BUFF_311 (311UL)
#define MCAN_MESSAGE_BUFF_312 (312UL)
#define MCAN_MESSAGE_BUFF_313 (313UL)
#define MCAN_MESSAGE_BUFF_314 (314UL)
#define MCAN_MESSAGE_BUFF_315 (315UL)
#define MCAN_MESSAGE_BUFF_316 (316UL)
#define MCAN_MESSAGE_BUFF_317 (317UL)
#define MCAN_MESSAGE_BUFF_318 (318UL)
#define MCAN_MESSAGE_BUFF_319 (319UL)
#define MCAN_MESSAGE_BUFF_320 (320UL)
#define MCAN_MESSAGE_BUFF_321 (321UL)
#define MCAN_MESSAGE_BUFF_322 (322UL)
#define MCAN_MESSAGE_BUFF_323 (323UL)
#define MCAN_MESSAGE_BUFF_324 (324UL)
#define MCAN_MESSAGE_BUFF_325 (325UL)
#define MCAN_MESSAGE_BUFF_326 (326UL)
#define MCAN_MESSAGE_BUFF_327 (327UL)
#define MCAN_MESSAGE_BUFF_328 (328UL)
#define MCAN_MESSAGE_BUFF_329 (329UL)
#define MCAN_MESSAGE_BUFF_330 (330UL)
#define MCAN_MESSAGE_BUFF_331 (331UL)
#define MCAN_MESSAGE_BUFF_332 (332UL)
#define MCAN_MESSAGE_BUFF_333 (333UL)
#define MCAN_MESSAGE_BUFF_334 (334UL)
#define MCAN_MESSAGE_BUFF_335 (335UL)
#define MCAN_MESSAGE_BUFF_336 (336UL)
#define MCAN_MESSAGE_BUFF_337 (337UL)
#define MCAN_MESSAGE_BUFF_338 (338UL)
#define MCAN_MESSAGE_BUFF_339 (339UL)
#define MCAN_MESSAGE_BUFF_340 (340UL)
#define MCAN_MESSAGE_BUFF_341 (341UL)
#define MCAN_MESSAGE_BUFF_342 (342UL)
#define MCAN_MESSAGE_BUFF_343 (343UL)
#define MCAN_MESSAGE_BUFF_344 (344UL)
#define MCAN_MESSAGE_BUFF_345 (345UL)
#define MCAN_MESSAGE_BUFF_346 (346UL)
#define MCAN_MESSAGE_BUFF_347 (347UL)
#define MCAN_MESSAGE_BUFF_348 (348UL)
#define MCAN_MESSAGE_BUFF_349 (349UL)
#define MCAN_MESSAGE_BUFF_350 (350UL)
#define MCAN_MESSAGE_BUFF_351 (351UL)
#define MCAN_MESSAGE_BUFF_352 (352UL)
#define MCAN_MESSAGE_BUFF_353 (353UL)
#define MCAN_MESSAGE_BUFF_354 (354UL)
#define MCAN_MESSAGE_BUFF_355 (355UL)
#define MCAN_MESSAGE_BUFF_356 (356UL)
#define MCAN_MESSAGE_BUFF_357 (357UL)
#define MCAN_MESSAGE_BUFF_358 (358UL)
#define MCAN_MESSAGE_BUFF_359 (359UL)
#define MCAN_MESSAGE_BUFF_360 (360UL)
#define MCAN_MESSAGE_BUFF_361 (361UL)
#define MCAN_MESSAGE_BUFF_362 (362UL)
#define MCAN_MESSAGE_BUFF_363 (363UL)
#define MCAN_MESSAGE_BUFF_364 (364UL)
#define MCAN_MESSAGE_BUFF_365 (365UL)
#define MCAN_MESSAGE_BUFF_366 (366UL)
#define MCAN_MESSAGE_BUFF_367 (367UL)
#define MCAN_MESSAGE_BUFF_368 (368UL)
#define MCAN_MESSAGE_BUFF_369 (369UL)
#define MCAN_MESSAGE_BUFF_370 (370UL)
#define MCAN_MESSAGE_BUFF_371 (371UL)
#define MCAN_MESSAGE_BUFF_372 (372UL)
#define MCAN_MESSAGE_BUFF_373 (373UL)
#define MCAN_MESSAGE_BUFF_374 (374UL)
#define MCAN_MESSAGE_BUFF_375 (375UL)
#define MCAN_MESSAGE_BUFF_376 (376UL)
#define MCAN_MESSAGE_BUFF_377 (377UL)
#define MCAN_MESSAGE_BUFF_378 (378UL)
#define MCAN_MESSAGE_BUFF_379 (379UL)
#define MCAN_MESSAGE_BUFF_380 (380UL)
#define MCAN_MESSAGE_BUFF_381 (381UL)
#define MCAN_MESSAGE_BUFF_382 (382UL)
#define MCAN_MESSAGE_BUFF_383 (383UL)
#define MCAN_MESSAGE_BUFF_384 (384UL)
#define MCAN_MESSAGE_BUFF_385 (385UL)
#define MCAN_MESSAGE_BUFF_386 (386UL)
#define MCAN_MESSAGE_BUFF_387 (387UL)
#define MCAN_MESSAGE_BUFF_388 (388UL)
#define MCAN_MESSAGE_BUFF_389 (389UL)
#define MCAN_MESSAGE_BUFF_390 (390UL)
#define MCAN_MESSAGE_BUFF_391 (391UL)
#define MCAN_MESSAGE_BUFF_392 (392UL)
#define MCAN_MESSAGE_BUFF_393 (393UL)
#define MCAN_MESSAGE_BUFF_394 (394UL)
#define MCAN_MESSAGE_BUFF_395 (395UL)
#define MCAN_MESSAGE_BUFF_396 (396UL)
#define MCAN_MESSAGE_BUFF_397 (397UL)
#define MCAN_MESSAGE_BUFF_398 (398UL)
#define MCAN_MESSAGE_BUFF_399 (399UL)
#define MCAN_MESSAGE_BUFF_400 (400UL)
#define MCAN_MESSAGE_BUFF_401 (401UL)
#define MCAN_MESSAGE_BUFF_402 (402UL)
#define MCAN_MESSAGE_BUFF_403 (403UL)
#define MCAN_MESSAGE_BUFF_404 (404UL)
#define MCAN_MESSAGE_BUFF_405 (405UL)
#define MCAN_MESSAGE_BUFF_406 (406UL)
#define MCAN_MESSAGE_BUFF_407 (407UL)
#define MCAN_MESSAGE_BUFF_408 (408UL)
#define MCAN_MESSAGE_BUFF_409 (409UL)
#define MCAN_MESSAGE_BUFF_410 (410UL)
#define MCAN_MESSAGE_BUFF_411 (411UL)
#define MCAN_MESSAGE_BUFF_412 (412UL)
#define MCAN_MESSAGE_BUFF_413 (413UL)
#define MCAN_MESSAGE_BUFF_414 (414UL)
#define MCAN_MESSAGE_BUFF_415 (415UL)
#define MCAN_MESSAGE_BUFF_416 (416UL)
#define MCAN_MESSAGE_BUFF_417 (417UL)
#define MCAN_MESSAGE_BUFF_418 (418UL)
#define MCAN_MESSAGE_BUFF_419 (419UL)
#define MCAN_MESSAGE_BUFF_420 (420UL)
#define MCAN_MESSAGE_BUFF_421 (421UL)
#define MCAN_MESSAGE_BUFF_422 (422UL)
#define MCAN_MESSAGE_BUFF_423 (423UL)
#define MCAN_MESSAGE_BUFF_424 (424UL)
#define MCAN_MESSAGE_BUFF_425 (425UL)
#define MCAN_MESSAGE_BUFF_426 (426UL)
#define MCAN_MESSAGE_BUFF_427 (427UL)
#define MCAN_MESSAGE_BUFF_428 (428UL)
#define MCAN_MESSAGE_BUFF_429 (429UL)
#define MCAN_MESSAGE_BUFF_430 (430UL)
#define MCAN_MESSAGE_BUFF_431 (431UL)
#define MCAN_MESSAGE_BUFF_432 (432UL)
#define MCAN_MESSAGE_BUFF_433 (433UL)
#define MCAN_MESSAGE_BUFF_434 (434UL)
#define MCAN_MESSAGE_BUFF_435 (435UL)
#define MCAN_MESSAGE_BUFF_436 (436UL)
#define MCAN_MESSAGE_BUFF_437 (437UL)
#define MCAN_MESSAGE_BUFF_438 (438UL)
#define MCAN_MESSAGE_BUFF_439 (439UL)
#define MCAN_MESSAGE_BUFF_440 (440UL)
#define MCAN_MESSAGE_BUFF_441 (441UL)
#define MCAN_MESSAGE_BUFF_442 (442UL)
#define MCAN_MESSAGE_BUFF_443 (443UL)
#define MCAN_MESSAGE_BUFF_444 (444UL)
#define MCAN_MESSAGE_BUFF_445 (445UL)
#define MCAN_MESSAGE_BUFF_446 (446UL)
#define MCAN_MESSAGE_BUFF_447 (447UL)
#define MCAN_MESSAGE_BUFF_448 (448UL)
#define MCAN_MESSAGE_BUFF_449 (449UL)
#define MCAN_MESSAGE_BUFF_450 (450UL)
#define MCAN_MESSAGE_BUFF_451 (451UL)
#define MCAN_MESSAGE_BUFF_452 (452UL)
#define MCAN_MESSAGE_BUFF_453 (453UL)
#define MCAN_MESSAGE_BUFF_454 (454UL)
#define MCAN_MESSAGE_BUFF_455 (455UL)
#define MCAN_MESSAGE_BUFF_456 (456UL)
#define MCAN_MESSAGE_BUFF_457 (457UL)
#define MCAN_MESSAGE_BUFF_458 (458UL)
#define MCAN_MESSAGE_BUFF_459 (459UL)
#define MCAN_MESSAGE_BUFF_460 (460UL)
#define MCAN_MESSAGE_BUFF_461 (461UL)
#define MCAN_MESSAGE_BUFF_462 (462UL)
#define MCAN_MESSAGE_BUFF_463 (463UL)
#define MCAN_MESSAGE_BUFF_464 (464UL)
#define MCAN_MESSAGE_BUFF_465 (465UL)
#define MCAN_MESSAGE_BUFF_466 (466UL)
#define MCAN_MESSAGE_BUFF_467 (467UL)
#define MCAN_MESSAGE_BUFF_468 (468UL)
#define MCAN_MESSAGE_BUFF_469 (469UL)
#define MCAN_MESSAGE_BUFF_470 (470UL)
#define MCAN_MESSAGE_BUFF_471 (471UL)
#define MCAN_MESSAGE_BUFF_472 (472UL)
#define MCAN_MESSAGE_BUFF_473 (473UL)
#define MCAN_MESSAGE_BUFF_474 (474UL)
#define MCAN_MESSAGE_BUFF_475 (475UL)
#define MCAN_MESSAGE_BUFF_476 (476UL)
#define MCAN_MESSAGE_BUFF_477 (477UL)
#define MCAN_MESSAGE_BUFF_478 (478UL)
#define MCAN_MESSAGE_BUFF_479 (479UL)
#define MCAN_MESSAGE_BUFF_480 (480UL)
#define MCAN_MESSAGE_BUFF_481 (481UL)
#define MCAN_MESSAGE_BUFF_482 (482UL)
#define MCAN_MESSAGE_BUFF_483 (483UL)
#define MCAN_MESSAGE_BUFF_484 (484UL)
#define MCAN_MESSAGE_BUFF_485 (485UL)
#define MCAN_MESSAGE_BUFF_486 (486UL)
#define MCAN_MESSAGE_BUFF_487 (487UL)
#define MCAN_MESSAGE_BUFF_488 (488UL)
#define MCAN_MESSAGE_BUFF_489 (489UL)
#define MCAN_MESSAGE_BUFF_490 (490UL)
#define MCAN_MESSAGE_BUFF_491 (491UL)
#define MCAN_MESSAGE_BUFF_492 (492UL)
#define MCAN_MESSAGE_BUFF_493 (493UL)
#define MCAN_MESSAGE_BUFF_494 (494UL)
#define MCAN_MESSAGE_BUFF_495 (495UL)
#define MCAN_MESSAGE_BUFF_496 (496UL)
#define MCAN_MESSAGE_BUFF_497 (497UL)
#define MCAN_MESSAGE_BUFF_498 (498UL)
#define MCAN_MESSAGE_BUFF_499 (499UL)
#define MCAN_MESSAGE_BUFF_500 (500UL)
#define MCAN_MESSAGE_BUFF_501 (501UL)
#define MCAN_MESSAGE_BUFF_502 (502UL)
#define MCAN_MESSAGE_BUFF_503 (503UL)
#define MCAN_MESSAGE_BUFF_504 (504UL)
#define MCAN_MESSAGE_BUFF_505 (505UL)
#define MCAN_MESSAGE_BUFF_506 (506UL)
#define MCAN_MESSAGE_BUFF_507 (507UL)
#define MCAN_MESSAGE_BUFF_508 (508UL)
#define MCAN_MESSAGE_BUFF_509 (509UL)
#define MCAN_MESSAGE_BUFF_510 (510UL)
#define MCAN_MESSAGE_BUFF_511 (511UL)
#define MCAN_MESSAGE_BUFF_512 (512UL)
#define MCAN_MESSAGE_BUFF_513 (513UL)
#define MCAN_MESSAGE_BUFF_514 (514UL)
#define MCAN_MESSAGE_BUFF_515 (515UL)
#define MCAN_MESSAGE_BUFF_516 (516UL)
#define MCAN_MESSAGE_BUFF_517 (517UL)
#define MCAN_MESSAGE_BUFF_518 (518UL)
#define MCAN_MESSAGE_BUFF_519 (519UL)
#define MCAN_MESSAGE_BUFF_520 (520UL)
#define MCAN_MESSAGE_BUFF_521 (521UL)
#define MCAN_MESSAGE_BUFF_522 (522UL)
#define MCAN_MESSAGE_BUFF_523 (523UL)
#define MCAN_MESSAGE_BUFF_524 (524UL)
#define MCAN_MESSAGE_BUFF_525 (525UL)
#define MCAN_MESSAGE_BUFF_526 (526UL)
#define MCAN_MESSAGE_BUFF_527 (527UL)
#define MCAN_MESSAGE_BUFF_528 (528UL)
#define MCAN_MESSAGE_BUFF_529 (529UL)
#define MCAN_MESSAGE_BUFF_530 (530UL)
#define MCAN_MESSAGE_BUFF_531 (531UL)
#define MCAN_MESSAGE_BUFF_532 (532UL)
#define MCAN_MESSAGE_BUFF_533 (533UL)
#define MCAN_MESSAGE_BUFF_534 (534UL)
#define MCAN_MESSAGE_BUFF_535 (535UL)
#define MCAN_MESSAGE_BUFF_536 (536UL)
#define MCAN_MESSAGE_BUFF_537 (537UL)
#define MCAN_MESSAGE_BUFF_538 (538UL)
#define MCAN_MESSAGE_BUFF_539 (539UL)
#define MCAN_MESSAGE_BUFF_540 (540UL)
#define MCAN_MESSAGE_BUFF_541 (541UL)
#define MCAN_MESSAGE_BUFF_542 (542UL)
#define MCAN_MESSAGE_BUFF_543 (543UL)
#define MCAN_MESSAGE_BUFF_544 (544UL)
#define MCAN_MESSAGE_BUFF_545 (545UL)
#define MCAN_MESSAGE_BUFF_546 (546UL)
#define MCAN_MESSAGE_BUFF_547 (547UL)
#define MCAN_MESSAGE_BUFF_548 (548UL)
#define MCAN_MESSAGE_BUFF_549 (549UL)
#define MCAN_MESSAGE_BUFF_550 (550UL)
#define MCAN_MESSAGE_BUFF_551 (551UL)
#define MCAN_MESSAGE_BUFF_552 (552UL)
#define MCAN_MESSAGE_BUFF_553 (553UL)
#define MCAN_MESSAGE_BUFF_554 (554UL)
#define MCAN_MESSAGE_BUFF_555 (555UL)
#define MCAN_MESSAGE_BUFF_556 (556UL)
#define MCAN_MESSAGE_BUFF_557 (557UL)
#define MCAN_MESSAGE_BUFF_558 (558UL)
#define MCAN_MESSAGE_BUFF_559 (559UL)
#define MCAN_MESSAGE_BUFF_560 (560UL)
#define MCAN_MESSAGE_BUFF_561 (561UL)
#define MCAN_MESSAGE_BUFF_562 (562UL)
#define MCAN_MESSAGE_BUFF_563 (563UL)
#define MCAN_MESSAGE_BUFF_564 (564UL)
#define MCAN_MESSAGE_BUFF_565 (565UL)
#define MCAN_MESSAGE_BUFF_566 (566UL)
#define MCAN_MESSAGE_BUFF_567 (567UL)
#define MCAN_MESSAGE_BUFF_568 (568UL)
#define MCAN_MESSAGE_BUFF_569 (569UL)
#define MCAN_MESSAGE_BUFF_570 (570UL)
#define MCAN_MESSAGE_BUFF_571 (571UL)
#define MCAN_MESSAGE_BUFF_572 (572UL)
#define MCAN_MESSAGE_BUFF_573 (573UL)
#define MCAN_MESSAGE_BUFF_574 (574UL)
#define MCAN_MESSAGE_BUFF_575 (575UL)
#define MCAN_MESSAGE_BUFF_576 (576UL)
#define MCAN_MESSAGE_BUFF_577 (577UL)
#define MCAN_MESSAGE_BUFF_578 (578UL)
#define MCAN_MESSAGE_BUFF_579 (579UL)
#define MCAN_MESSAGE_BUFF_580 (580UL)
#define MCAN_MESSAGE_BUFF_581 (581UL)
#define MCAN_MESSAGE_BUFF_582 (582UL)
#define MCAN_MESSAGE_BUFF_583 (583UL)
#define MCAN_MESSAGE_BUFF_584 (584UL)
#define MCAN_MESSAGE_BUFF_585 (585UL)
#define MCAN_MESSAGE_BUFF_586 (586UL)
#define MCAN_MESSAGE_BUFF_587 (587UL)
#define MCAN_MESSAGE_BUFF_588 (588UL)
#define MCAN_MESSAGE_BUFF_589 (589UL)
#define MCAN_MESSAGE_BUFF_590 (590UL)
#define MCAN_MESSAGE_BUFF_591 (591UL)
#define MCAN_MESSAGE_BUFF_592 (592UL)
#define MCAN_MESSAGE_BUFF_593 (593UL)
#define MCAN_MESSAGE_BUFF_594 (594UL)
#define MCAN_MESSAGE_BUFF_595 (595UL)
#define MCAN_MESSAGE_BUFF_596 (596UL)
#define MCAN_MESSAGE_BUFF_597 (597UL)
#define MCAN_MESSAGE_BUFF_598 (598UL)
#define MCAN_MESSAGE_BUFF_599 (599UL)
#define MCAN_MESSAGE_BUFF_600 (600UL)
#define MCAN_MESSAGE_BUFF_601 (601UL)
#define MCAN_MESSAGE_BUFF_602 (602UL)
#define MCAN_MESSAGE_BUFF_603 (603UL)
#define MCAN_MESSAGE_BUFF_604 (604UL)
#define MCAN_MESSAGE_BUFF_605 (605UL)
#define MCAN_MESSAGE_BUFF_606 (606UL)
#define MCAN_MESSAGE_BUFF_607 (607UL)
#define MCAN_MESSAGE_BUFF_608 (608UL)
#define MCAN_MESSAGE_BUFF_609 (609UL)
#define MCAN_MESSAGE_BUFF_610 (610UL)
#define MCAN_MESSAGE_BUFF_611 (611UL)
#define MCAN_MESSAGE_BUFF_612 (612UL)
#define MCAN_MESSAGE_BUFF_613 (613UL)
#define MCAN_MESSAGE_BUFF_614 (614UL)
#define MCAN_MESSAGE_BUFF_615 (615UL)
#define MCAN_MESSAGE_BUFF_616 (616UL)
#define MCAN_MESSAGE_BUFF_617 (617UL)
#define MCAN_MESSAGE_BUFF_618 (618UL)
#define MCAN_MESSAGE_BUFF_619 (619UL)
#define MCAN_MESSAGE_BUFF_620 (620UL)
#define MCAN_MESSAGE_BUFF_621 (621UL)
#define MCAN_MESSAGE_BUFF_622 (622UL)
#define MCAN_MESSAGE_BUFF_623 (623UL)
#define MCAN_MESSAGE_BUFF_624 (624UL)
#define MCAN_MESSAGE_BUFF_625 (625UL)
#define MCAN_MESSAGE_BUFF_626 (626UL)
#define MCAN_MESSAGE_BUFF_627 (627UL)
#define MCAN_MESSAGE_BUFF_628 (628UL)
#define MCAN_MESSAGE_BUFF_629 (629UL)
#define MCAN_MESSAGE_BUFF_630 (630UL)
#define MCAN_MESSAGE_BUFF_631 (631UL)
#define MCAN_MESSAGE_BUFF_632 (632UL)
#define MCAN_MESSAGE_BUFF_633 (633UL)
#define MCAN_MESSAGE_BUFF_634 (634UL)
#define MCAN_MESSAGE_BUFF_635 (635UL)
#define MCAN_MESSAGE_BUFF_636 (636UL)
#define MCAN_MESSAGE_BUFF_637 (637UL)
#define MCAN_MESSAGE_BUFF_638 (638UL)
#define MCAN_MESSAGE_BUFF_639 (639UL)


#endif /* HPM_MCAN_H */