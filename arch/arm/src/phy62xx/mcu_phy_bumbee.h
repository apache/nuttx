/****************************************************************************
 * arch/arm/src/phy62xx/mcu_phy_bumbee.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_MCU_PHY_BUMBEE_H
#define __ARCH_ARM_SRC_PHY62XX_MCU_PHY_BUMBEE_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "types.h"
#include "bus_dev.h"
typedef enum
{
    MOD_NONE        = 0, MOD_CK802_CPU   = 0,
    MOD_DMA         = 3,
    MOD_AES         = 4,
    MOD_IOMUX       = 7,
    MOD_UART0       = 8,
    MOD_I2C0        = 9,
    MOD_I2C1        = 10,
    MOD_SPI0        = 11,
    MOD_SPI1        = 12,
    MOD_GPIO        = 13,
    MOD_QDEC        = 15,
    MOD_ADCC        = 17,
    MOD_PWM         = 18,
    MOD_SPIF        = 19,
    MOD_VOC         = 20,
    MOD_TIMER5      = 21,
    MOD_TIMER6      = 22,
    MOD_UART1       = 25,

    MOD_CP_CPU      = 0 + 32,
    MOD_BB          = MOD_CP_CPU + 3,
    MOD_TIMER       = MOD_CP_CPU + 4,
    MOD_WDT         = MOD_CP_CPU + 5,
    MOD_COM         = MOD_CP_CPU + 6,
    MOD_KSCAN       = MOD_CP_CPU + 7,
    MOD_BBREG       = MOD_CP_CPU + 9,
    BBLL_RST        = MOD_CP_CPU + 10,  /* can reset,but not gate in here */
    BBTX_RST        = MOD_CP_CPU + 11,  /* can reset,but not gate in here */
    BBRX_RST        = MOD_CP_CPU + 12,  /* can reset,but not gate in here */
    BBMIX_RST       = MOD_CP_CPU + 13,  /* can reset,but not gate in here */
    MOD_TIMER1      = MOD_CP_CPU + 21,
    MOD_TIMER2      = MOD_CP_CPU + 22,
    MOD_TIMER3      = MOD_CP_CPU + 23,
    MOD_TIMER4      = MOD_CP_CPU + 24,

    MOD_PCLK_CACHE  = 0 + 64,
    MOD_HCLK_CACHE  = MOD_PCLK_CACHE + 1,

    MOD_USR0        = 0 + 96,
    MOD_USR1        = MOD_USR0 + 1,
    MOD_USR2        = MOD_USR0 + 2,
    MOD_USR3        = MOD_USR0 + 3,
    MOD_USR4        = MOD_USR0 + 4,
    MOD_USR5        = MOD_USR0 + 5,
    MOD_USR6        = MOD_USR0 + 6,
    MOD_USR7        = MOD_USR0 + 7,
    MOD_USR8        = MOD_USR0 + 8,
    MOD_SYSTEM      = 0xff,
} MODULE_e;

/* SW_CLK -->0x4000f008 */
#define _CLK_NONE        (BIT(0))
#define _CLK_CK802_CPU   (BIT(0))
#define _CLK_DMA         (BIT(3))
#define _CLK_AES         (BIT(4))
#define _CLK_IOMUX       (BIT(7))
#define _CLK_UART0       (BIT(8))
#define _CLK_I2C0        (BIT(9))
#define _CLK_I2C1        (BIT(10))
#define _CLK_SPI0        (BIT(11))
#define _CLK_SPI1        (BIT(12))
#define _CLK_GPIO        (BIT(13))
#define _CLK_QDEC        (BIT(15))
#define _CLK_ADCC        (BIT(17))
#define _CLK_PWM         (BIT(18))
#define _CLK_SPIF        (BIT(19))
#define _CLK_VOC         (BIT(20))
#define _CLK_TIMER5      (BIT(21))
#define _CLK_TIMER6      (BIT(22))
#define _CLK_UART1       (BIT(25))

/* SW_CLK1 -->0x4000f014 */

#define _CLK_M0_CPU      (BIT(0))
#define _CLK_BB          (BIT(3))
#define _CLK_TIMER       (BIT(4))
#define _CLK_WDT         (BIT(5))
#define _CLK_COM         (BIT(6))
#define _CLK_KSCAN       (BIT(7))
#define _CLK_BBREG       (BIT(9))
#define _CLK_TIMER1      (BIT(21))
#define _CLK_TIMER2      (BIT(22))
#define _CLK_TIMER3      (BIT(23))
#define _CLK_TIMER4      (BIT(24))

#define     BB_IRQ_HANDLER              V4_IRQ_HANDLER
#define     KSCAN_IRQ_HANDLER           V5_IRQ_HANDLER
#define     RTC_IRQ_HANDLER             V6_IRQ_HANDLER
#define     CP_COM_IRQ_HANDLER          V7_IRQ_HANDLER
#define     AP_COM_IRQ_HANDLER          V8_IRQ_HANDLER
#define     WDT_IRQ_HANDLER             V10_IRQ_HANDLER
#define     UART0_IRQ_HANDLER           V11_IRQ_HANDLER
#define     I2C0_IRQ_HANDLER            V12_IRQ_HANDLER
#define     I2C1_IRQ_HANDLER            V13_IRQ_HANDLER
#define     SPI0_IRQ_HANDLER            V14_IRQ_HANDLER
#define     SPI1_IRQ_HANDLER            V15_IRQ_HANDLER
#define     GPIO_IRQ_HANDLER            V16_IRQ_HANDLER
#define     UART1_IRQ_HANDLER           V17_IRQ_HANDLER
#define     SPIF_IRQ_HANDLER            V18_IRQ_HANDLER
#define     DMAC_IRQ_HANDLER            V19_IRQ_HANDLER
#define     TIM1_IRQ_HANDLER            V20_IRQ_HANDLER
#define     TIM2_IRQ_HANDLER            V21_IRQ_HANDLER
#define     TIM3_IRQ_HANDLER            V22_IRQ_HANDLER
#define     TIM4_IRQ_HANDLER            V23_IRQ_HANDLER
#define     TIM5_IRQ_HANDLER            V24_IRQ_HANDLER
#define     TIM6_IRQ_HANDLER            V25_IRQ_HANDLER
#define     AES_IRQ_HANDLER             V28_IRQ_HANDLER
#define     ADCC_IRQ_HANDLER            V29_IRQ_HANDLER
#define     QDEC_IRQ_HANDLER            V30_IRQ_HANDLER

typedef struct
{
    __IO uint32_t  CH0_AP_MBOX;          /* 0x00 */
    __IO uint32_t  CH0_CP_MBOX;          /* 0x04 */
    __IO uint32_t  CH1_AP_MBOX;          /* 0x08 */
    __IO uint32_t  CH1_CP_MBOX;          /* 0x0c */
    __IO uint32_t  AP_STATUS;            /* 0x10 */
    __IO uint32_t  CP_STATUS;            /* 0x14 */
    __IO uint32_t  AP_INTEN;             /* 0x18 */
    __IO uint32_t  CP_INTEN;             /* 0x1c */
    __IO uint32_t   remap;               /* 0x20 */
    __IO uint32_t   RXEV_EN;             /* 0x24 */
    __IO uint32_t   STCALIB;             /* 0x28 */
    __IO uint32_t  PERI_MASTER_SELECT;   /* 0x2c */
} AP_COM_TypeDef;

typedef struct
{
    __IO uint32_t  CTRL0;                /* 0x40 */
    __IO uint32_t  CTRL1;                /* 0x44 */
    uint32_t  reserverd[13];
    __IO uint32_t REMAP_TABLE;           /* 0x7c */
    __IO uint32_t REMAP_CTRL[32];        /* 0x80 */
} AP_CACHE_TypeDef;

typedef struct
{
    __IO uint8_t  CR;                    /* 0x0 */
    uint8_t  RESERVED0[3];
    __IO uint32_t TORR;                  /* 0x4 */
    __O  uint32_t CCVR;                  /* 0x8 */
    __IO uint32_t CRR;                   /* 0xc */
    uint8_t  STAT;                       /* 0x10 */
    uint8_t  reserverd1[3];
    __IO uint8_t  EOI;                   /* 0x14 */
    uint8_t  reserverd2[3];
} AP_WDT_TypeDef;

typedef struct
{
    __IO uint32_t SW_RESET0;             /* 0x0 */
    __IO uint32_t SW_RESET1;             /* 0x4 */
    __IO uint32_t SW_CLK;                /* 0x8 */
    __IO uint32_t SW_RESET2;             /* 0xc */
    __IO uint32_t SW_RESET3;             /* 0x10 */
    __IO uint32_t SW_CLK1;               /* 0x14 */
    __IO uint32_t APB_CLK;               /* 0x18 */
    __IO uint32_t APB_CLK_UPDATE;        /* 0x1c */
    __IO uint32_t CACHE_CLOCK_GATE;      /* 0x20 */
    __IO uint32_t CACHE_RST;             /* 0x24 */
    __IO uint32_t CACHE_BYPASS;          /* 0x28 */
} AP_PCR_TypeDef;

typedef struct
{
    __IO uint32_t LoadCount;             /* 0x0 */
    __IO uint32_t CurrentCount;          /* 0x4 */
    __IO uint32_t ControlReg;            /* 0x8 */
    __IO uint32_t EOI;                   /* 0xc */
    __IO uint32_t status;                /* 0x10 */
} AP_TIM_TypeDef;

typedef struct
{
    __IO uint32_t IntStatus;
    __IO uint32_t EOI;
    __IO uint32_t unMaskIntStatus;
    __IO uint32_t version;
} AP_TIM_SYS_TypeDef;

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/* Universal Asynchronous Receiver Transmitter (UARTx) */

typedef struct
{
    union
    {
        __I  uint8_t  RBR;
        __IO  uint8_t  THR;
        __IO uint8_t  DLL;
        uint32_t RESERVED0;              /* 0x0 */
    };
    union
    {
        __IO uint8_t  DLM;
        __IO uint32_t IER;               /* 0x4 */
    };
    union
    {
        __I  uint32_t IIR;               /* 0x8 */
        __IO  uint8_t  FCR;
    };
    __IO uint8_t  LCR;                   /* 0xc */
    uint8_t  RESERVED1[3];               /* Reserved */
    __IO uint32_t MCR;                   /* 0x10 */

    __I  uint8_t  LSR;                   /* 0x14 */
    uint8_t  RESERVED2[3];               /* Reserved */
    __IO uint32_t MSR;                   /* 0x18 */

    __IO uint8_t  SCR;                   /* 0x1c */
    uint8_t  RESERVED3[3];               /* Reserved */

    __IO uint32_t LPDLL;                 /* 0x20 */

    __IO uint32_t LPDLH;                 /* 0x24 */

    __IO uint32_t  recerved[2];

    union
    {
        __IO uint32_t SRBR[16];          /* 0x30~60xc */
        __IO uint32_t STHR[16];
    };
    __IO uint32_t  rFAR;                 /* 0x70 */

    __IO uint32_t  TFR;                  /* 0x74 */

    __IO uint32_t  RFW;                  /* 0x78 */

    __IO uint32_t  USR;                  /* 0x7c */

    __IO uint32_t  TFL;

    __IO uint32_t  RFL;

    __IO uint32_t  SRR;

    __IO uint32_t  SRTS;

    __IO uint32_t  SBCR;

    __IO uint32_t  SDMAM;

    __IO uint32_t  SFE;

    __IO uint32_t  SRT;

    __IO uint32_t  STET;                 /* 0xa0 */

    __IO uint32_t  HTX;

    __IO uint32_t  DMASA;                /* 0xa8 */

    __IO uint32_t  reserved[18];

    __IO uint32_t  CPR;                  /* 0xf4 */

    __IO uint32_t  UCV;

    __IO uint32_t  CTR;
} AP_UART_TypeDef;

/* Inter-Integrated Circuit (I2C)  setup by zjp */

typedef struct
{
    __IO uint32_t  IC_CON;
    __IO uint32_t  IC_TAR;
    __IO uint32_t  IC_SAR;
    __IO uint32_t  IC_HS_MADDR;
    __IO uint32_t  IC_DATA_CMD;          /* 0x10 */
    __IO uint32_t  IC_SS_SCL_HCNT;
    __IO uint32_t  IC_SS_SCL_LCNT;
    __IO uint32_t  IC_FS_SCL_HCNT;
    __IO uint32_t  IC_FS_SCL_LCNT;       /* 0x20 */
    __IO uint32_t  IC_HS_SCL_HCNT;
    __IO uint32_t  IC_HS_SCL_LCNT;
    __IO uint32_t  IC_INTR_STAT;
    __IO uint32_t  IC_INTR_MASK;         /* 0x30 */
    __IO uint32_t  IC_RAW_INTR_STAT;
    __IO uint32_t  IC_RX_TL;
    __IO uint32_t  IC_TX_TL;
    __IO uint32_t  IC_CLR_INTR;          /* 0x40 */
    __IO uint32_t  IC_CLR_UNDER;
    __IO uint32_t  IC_CLR_RX_OVER;
    __IO uint32_t  IC_CLR_TX_OVER;
    __IO uint32_t  IC_CLR_RD_REG;        /* 0x50 */
    __IO uint32_t  IC_CLR_TX_ABRT;
    __IO uint32_t  IC_CLR_RX_DONE;
    __IO uint32_t  IC_CLR_ACTIVITY;
    __IO uint32_t  IC_CLR_STOP_DET;      /* 0x60 */
    __IO uint32_t  IC_CLR_START_DET;
    __IO uint32_t  IC_CLR_GEN_CALL;
    __IO uint32_t  IC_ENABLE;
    __IO uint32_t  IC_STATUS;            /* 0x70 */
    __IO uint32_t  IC_TXFLR;
    __IO uint32_t  IC_RXFLR;
    __IO uint32_t  IC_SDA_HOLD;
    __IO uint32_t  IC_TX_ABRT_SOURCE;    /* 0x80 */
    __IO uint32_t  IC_SLV_DATA_NACK_ONLY;
    __IO uint32_t  IC_DMA_CR;
    __IO uint32_t  IC_DMA_TDLR;
    __IO uint32_t  IC_DMA_RDLR;          /* 0x90 */
    __IO uint32_t  IC_SDA_SETUP;
    __IO uint32_t  IC_ACK_GENERAL_CALL;
    __IO uint32_t  IC_ENABLE_STATUS;
    __IO uint32_t  IC_FS_SPKLEN;         /* 0xa0 */
    __IO uint32_t  IC_HS_SPKLEN;
} AP_I2C_TypeDef;

/* Inter IC Sound (I2S) */

typedef struct
{
    __IO uint32_t IER;
    __IO uint32_t IRER;
    __IO  uint32_t ITER;
    __IO  uint32_t CER;
    __IO  uint32_t CCR;
    __IO uint32_t RXFFR;
    __IO uint32_t TXFFR;
} AP_I2S_BLOCK_TypeDef;

typedef struct
{
    union
    {
        __IO uint32_t LRBR;              /* 0x20 */
        __IO uint32_t LTHR;              /* 0x20 */
    };
    union
    {
        __IO uint32_t RRBR;              /* 0x24 */
        __IO uint32_t RTHR;              /* 0x24 */
    };
    __IO uint32_t RER;                   /* 0x28 */
    __IO uint32_t TER;                   /* 0x2c */
    __IO uint32_t RCR;                   /* 0x30 */
    __IO uint32_t TCR;                   /* 0x34 */
    __IO uint32_t ISR;                   /* 0x38 */
    __IO uint32_t IMR;                   /* 0x3c */
    __IO uint32_t ROR;                   /* 0x40 */
    __IO uint32_t TOR;                   /* 0x44 */
    __IO uint32_t  RFCR;                 /* 0x48 */
    __IO uint32_t  TFCR;                 /* 0x4c */
    __IO uint32_t  RFF;                  /* 0x50 */
    __IO uint32_t  TFF;                  /* 0x54 */
} AP_I2S_TypeDef;

/* General Purpose Input/Output (GPIO) */

typedef struct
{
    __IO uint32_t swporta_dr;            /* 0x00 */
    __IO uint32_t swporta_ddr;           /* 0x04 */
    __IO uint32_t swporta_ctl;           /* 0x08 */
    uint32_t reserved8[9];               /* 0x18-0x2c portC&D */
    __IO uint32_t inten;                 /* 0x30 */
    __IO uint32_t intmask;               /* 0x34 */
    __IO uint32_t inttype_level;         /* 0x38 */
    __IO uint32_t int_polarity;          /* 0x3c */
    __I  uint32_t int_status;            /* 0x40 */
    __IO uint32_t raw_instatus;          /* 0x44 */
    __IO uint32_t debounce;              /* 0x48 */
    __O  uint32_t porta_eoi;             /* 0x4c */
    __I  uint32_t ext_porta;             /* 0x50 */
    uint32_t reserved9[3];               /* 0x58 0x5c */
    __IO uint32_t ls_sync;               /* 0x60 */
    __I  uint32_t id_code;               /* 0x64 */
    uint32_t reserved10[1];              /* 0x68 */
    __I  uint32_t ver_id_code;           /* 0x6c */
    __I  uint32_t config_reg2;           /* 0x70 */
    __I  uint32_t config_reg1;           /* 0x74 */
} AP_GPIO_TypeDef;

/* --------------------  (SPI) -------------------------------- */

typedef struct
{
    __IO uint16_t     CR0;               /* 0x0  */               /* !< Offset: 0x000 Control Register 0 (R/W) */
    uint16_t     reserved1;
    __IO uint16_t     CR1;               /* 0x04 */               /* !< Offset: 0x004 Control Register 1 (R/W) */
    uint16_t     reserved2;
    __IO uint8_t      SSIEN;             /* 0x08 */
    uint8_t      reserved3[3];
    __IO uint8_t      MWCR;              /* 0x0c */
    uint8_t      reserved4[3];
    __IO uint8_t      SER;               /* 0x10 */
    uint8_t      reserved5[3];
    __IO uint32_t     BAUDR;             /* 0x14 */
    __IO uint32_t     TXFTLR;            /* 0x18 */
    __IO uint32_t     RXFTLR;            /* 0x1c */
    __O  uint32_t     TXFLR;             /* 0x20 */
    __O  uint32_t     RXFLR;             /* 0x24 */

    __IO uint8_t      SR;                /* 0x28 */
    uint8_t      reserved7[3];
    __IO uint32_t     IMR;               /* 0x2c */
    __IO uint32_t     ISR;               /* 0x30 */
    __IO uint32_t     RISR;              /* 0x34 */
    __IO uint32_t     TXOICR;            /* 0x38 */
    __IO uint32_t     RXOICR;            /* 0x3c */
    __IO uint32_t     RXUICR;            /* 0x40 */
    __IO uint32_t     MSTICR;            /* 0x44 */
    __IO uint32_t     ICR;               /* 0x48 */
    __IO uint32_t     DMACR;             /* 0x4c */
    __IO uint32_t     DMATDLR;           /* 0x50 */
    __IO uint32_t     DMARDLR;           /* 0x54 */
    __IO uint32_t     IDR;               /* 0x5c */
    __IO uint32_t     SSI_COM_VER;       /* 0x5c */
    __IO uint32_t     DataReg;
} AP_SSI_TypeDef;

typedef struct
{
    __IO uint32_t    Analog_IO_en;       /* 0x00 */
    __IO uint32_t    SPI_debug_en;       /* 0x04 */
    __IO uint32_t    debug_mux_en;       /* 0x08 */
    __IO uint32_t    full_mux0_en;       /* 0x0c */
    __IO uint32_t    full_mux1_en;       /* 0x10 reserved in some soc */
    __IO uint32_t    gpio_pad_en;        /* 0x14 */
    __IO uint32_t    gpio_sel[9];        /* 0x18 */
    __IO uint32_t    pad_pe0;            /* 0x3c */
    __IO uint32_t    pad_pe1;            /* 0x40 */
    __IO uint32_t    pad_ps0;            /* 0x44 */
    __IO uint32_t    pad_ps1;            /* 0x48 */
    __IO uint32_t    keyscan_in_en;      /* 0x4c */
    __IO uint32_t    keyscan_out_en;     /* 0x50 */
} IOMUX_TypeDef;

typedef struct
{
    __IO uint32_t    PWROFF;             /* 0x00 */
    __IO uint32_t    PWRSLP;             /* 0x04 */
    __IO uint32_t    IOCTL[3];           /* 0x08 0x0c 0x10 */
    __IO uint32_t    PMCTL0;             /* 0x14 */
    __IO uint32_t    PMCTL1;             /* 0x18 */
    __IO uint32_t    PMCTL2_0;           /* 0x1c */
    __IO uint32_t    PMCTL2_1;           /* 0x20 */
    __IO uint32_t    RTCCTL;             /* 0x24 */
    __IO uint32_t    RTCCNT;             /* 0x28 */
    __IO uint32_t    RTCCC0;             /* 0x2c */
    __IO uint32_t    RTCCC1;             /* 0x30 */
    __IO uint32_t    RTCCC2;             /* 0x34 */
    __IO uint32_t    RTCFLAG;            /* 0x38 */
    __IO uint32_t    reserved[25];
    __IO uint32_t    REG_S9;             /* 0xa0 */
    __IO uint32_t    REG_S10;            /* 0xa4 */
    __IO uint32_t    REG_S11;            /* 0xa8 */
    __IO uint32_t    IDLE_REG;           /* 0xac */
    __IO uint32_t    GPIO_WAKEUP_SRC[2]; /* 0xb0 b4 */
    __IO uint32_t    PCLK_CLK_GATE;      /* 0xb8 */
    __IO uint32_t    XTAL_16M_CTRL;      /* 0xbc */
    __IO uint32_t    SLEEP_R[4];         /* 0xc0 c4 c8 cc */
} AP_AON_TypeDef;

typedef struct
{
    __IO uint32_t    RTCCTL;             /* 0x24 */
    __IO uint32_t    RTCCNT;             /* 0x28 */
    __IO uint32_t    RTCCC0;             /* 0x2c */
    __IO uint32_t    RTCCC1;             /* 0x30 */
    __IO uint32_t    RTCCC2;             /* 0x34 */
    __IO uint32_t    RTCFLAG;            /* 0x38 */
} AP_RTC_TypeDef;

typedef struct
{
    __IO uint32_t    io_wu_mask_31_0;    /* 0xa0 */
    __IO uint32_t    io_wu_mask_34_32;   /* 0xa4 */
} AP_Wakeup_TypeDef;

typedef struct
{
    __IO uint32_t    CLKSEL;             /* 0x3c */
    __IO uint32_t    CLKHF_CTL0;         /* 0x40 */
    __IO uint32_t    CLKHF_CTL1;         /* 0x44 */
    __IO uint32_t    ANA_CTL;            /* 0x48 */
    __IO uint32_t    mem_0_1_dvs;        /* 0x4c */
    __IO uint32_t    mem_2_3_4_dvs;      /* 0x50 */
    __IO uint32_t    efuse_cfg;          /* 0x54 */
    __IO uint32_t    chip_state;         /* 0x58 */
    __IO uint32_t    cal_rw;             /* 0x5c */
    __IO uint32_t    cal_ro0;            /* 0x60 */
    __IO uint32_t    cal_ro1;            /* 0x64 */
    __IO uint32_t    cal_ro2;            /* 0x68 */
    __IO uint32_t    ADC_CTL0;           /* 0x6c */
    __IO uint32_t    ADC_CTL1;           /* 0x70 */
    __IO uint32_t    ADC_CTL2;           /* 0x74 */
    __IO uint32_t    ADC_CTL3;           /* 0x78 */
    __IO uint32_t    ADC_CTL4;           /* 0x7c */
    uint32_t    reserved1[48];
    __IO uint32_t    EFUSE_PROG[2];      /* 0x140 */
    uint32_t    reserved2[6];
    __IO uint32_t    EFUSE0[2];          /* 0x160 */
    __IO uint32_t    EFUSE1[2];          /* 0x168 */
    __IO uint32_t    EFUSE2[2];          /* 0x170 */
    __IO uint32_t    EFUSE3[2];          /* 0x178 */
    __IO uint32_t    SECURTY_STATE;      /* 0x180 */
} AP_PCRM_TypeDef;

typedef struct
{
    __IO uint32_t    enable;                   /* 0x00 */
    __IO uint32_t    reserve0[2];              /* 0x04~0x08 */
    __IO uint32_t    control_1;                /* 0x0c */
    __IO uint32_t    control_2;                /* 0x10 */
    __IO uint32_t    control_3;                /* 0x14 */
    __IO uint32_t    control_4;                /* 0x18 */
    __IO uint32_t    compare_reset;            /* 0x1c */
    __IO uint32_t    int_pointer_ch0_ch3;      /* 0x20 */
    __IO uint32_t    int_pointer_ch4_ch7;      /* 0x24 */

    /* __IO uint32_t    int_pointer[2]; */     /* 0x20~0x24 */
    __IO uint32_t    reserve1[3];              /* 0x28~0x30 */
    __IO uint32_t    intr_mask;                /* 0x34 */
    __IO uint32_t    intr_clear;               /* 0x38 */
    __IO uint32_t    intr_status;              /* 0x3c */
    __IO uint32_t    compare_cfg[8];           /* 0x40~0x5c */
} AP_ADCC_TypeDef;

typedef struct
{
    __IO uint32_t    config;                       /* 0x0,QSPI Configuration Register,R/W */
    __IO uint32_t    read_instr;                   /* 0x4,Device Read Instruction Register,R/W */
    __IO uint32_t    write_instr;                  /* 0x8,Device Write Instruction Register,R/W */
    __IO uint32_t    delay;                        /* 0xC,QSPI Device Delay Register,R/W */
    __IO uint32_t    rddata_capture;               /* 0x10,Read Data Capture Register,R/W */
    __IO uint32_t    dev_size;                     /* 0x14,Device Size Register,R/W */
    __IO uint32_t    sram_part;                    /* 0x18,SRAM Partition Register,R/W */
    __IO uint32_t    indirect_ahb_addr_trig;       /* 0x1C,Indirect AHB Address Trigger Register,R/W */
    __IO uint32_t    dma_peripheral;               /* 0x20,DMA Peripheral Register,R/W */
    __IO uint32_t    remap;                        /* 0x24,Remap Address Register,R/W */
    __IO uint32_t    mode_bit;                     /* 0x28,Mode Bit Register,R/W */
    __IO uint32_t    sram_fill_level;              /* 0x2C,SRAM Fill Level Register,RO */
    __IO uint32_t    tx_threshold;                 /* 0x30,TX Threshold Register,R/W */
    __IO uint32_t    rx_threshold;                 /* 0x34,RX Threshold Register,R/W */
    __IO uint32_t    wr_completion_ctrl;           /* 0x38,Write Completion Control Register,R/W */
    __IO uint32_t    poll_expire;                  /* 0x3C,Polling Expiration Register,R/W */
    __IO uint32_t    int_status;                   /* 0x40,Interrupt Status Register,R/W */
    __IO uint32_t    int_mask;                     /* 0x44,Interrupt Mask,R/W */
    __I  uint32_t    n1[2];                        /* 0x48~0x4c,Empty */
    __IO uint32_t    low_wr_protection;            /* 0x50,Lower Write Protection Register,R/W */
    __IO uint32_t    up_wr_protection;             /* 0x54,Upper Write Protection Register,R/W */
    __IO uint32_t    wr_protection;                /* 0x58,Write Protection Register,R/W */
    __I  uint32_t    n2;                           /* 0x5c,Empty */
    __IO uint32_t    indirect_rd;                  /* 0x60,Indirect Read Transfer Register,R/W */
    __IO uint32_t    indirect_rd_watermark;        /* 0x64,Indirect Read Transfer Watermark Register,R/W */
    __IO uint32_t    indirect_rd_start_addr;       /* 0x68,Indirect Read Transfer Start Address Register,R/W */
    __IO uint32_t    indirect_rd_num;              /* 0x6C,Indirect Read Transfer Number Bytes Register,R/W */
    __IO uint32_t    indirect_wr;                  /* 0x70,Indirect Write Transfer Register,R/W */
    __IO uint32_t    indirect_wr_watermark;        /* 0x74,Indirect Write Transfer Watermark Register,R/W */
    __IO uint32_t    indirect_wr_start_addr;       /* 0x78,Indirect Write Transfer Start Address Register,R/W */
    __IO uint32_t    indirect_wr_cnt;              /* 0x7C,Indirect Write Transfer Count Register,R/W */
    __IO uint32_t    indirect_ahb_trig_addr_range; /* 0x80,Indirect AHB Trigger Address Range Register,R/W */
    __I  uint32_t    n3[3];                        /* 0x84~0x8c,Empty */
    __IO uint32_t    fcmd;                         /* 0x90,Flash Command Register,R/W */
    __IO uint32_t    fcmd_addr;                    /* 0x94,Flash Command Address Registers,R/W */
    __I  uint32_t    n4[2];                        /* 0x98~0x9c,Empty */
    __IO uint32_t    fcmd_rddata[2];               /* 0xA0,Flash Command Read Data Register (low-a0, up-a4),RO */
    __IO uint32_t    fcmd_wrdata[2];               /* 0xA8,Flash Command Write Data Register (low-a8, up-ac),R/W */
    __IO uint32_t    poll_fstatus;                 /* 0xB0,Polling Flash Status Register,RO */

    /* __IO uint32_t; */                           /* 0xFC,Module ID Register,RO */
} AP_SPIF_TypeDef;

typedef struct
{
    __IO uint32_t    ctrl0;                    /* 0xc0 */
    __IO uint32_t    ctrl1;                    /* 0xc4 */
    __IO uint32_t    mk_in_en;                 /* 0xc8 */
    __IO uint32_t    mkc[6];                   /* 0xcc~0xe0 */
} AP_KSCAN_TypeDef;

typedef struct
{
    __IO uint32_t    pwmen;
} AP_PWM_TypeDef;

typedef struct
{
    __IO uint32_t    ctrl0;
    __IO uint32_t    ctrl1;
} AP_PWMCTRL_TypeDef;

typedef struct
{
    __IO uint32_t    SAR;
    __IO uint32_t    SAR_H;
    __IO uint32_t    DAR;
    __IO uint32_t    DAR_H;
    __IO uint32_t    LLP;
    __IO uint32_t    LLP_H;
    __IO uint32_t    CTL;
    __IO uint32_t    CTL_H;
    __IO uint32_t    SSTAT;
    __IO uint32_t    SSTAT_H;
    __IO uint32_t    DSTAT;
    __IO uint32_t    DSTAT_L;
    __IO uint32_t    SSTATAR;
    __IO uint32_t    SSTATAR_H;
    __IO uint32_t    DSTATAR;
    __IO uint32_t    DSTATAR_H;
    __IO uint32_t    CFG;
    __IO uint32_t    CFG_H;
    __IO uint32_t    rsv[4];
} AP_DMA_CH_TypeDef;

typedef struct
{
    __IO uint32_t    RawTfr;            /* 0x2c0 */
    __IO uint32_t    RawTfr_H;          /* 0x2c4 */
    __IO uint32_t    RawBlock;          /* 0x2c8 */
    __IO uint32_t    RawBlock_H;        /* 0x2cc */
    __IO uint32_t    RawSrcTran;        /* 0x2d0 */
    __IO uint32_t    RawSrcTran_H;      /* 0x2d4 */
    __IO uint32_t    RawDstTran;        /* 0x2d8 */
    __IO uint32_t    RawDstTran_H;      /* 0x2dc */
    __IO uint32_t    RawErr;            /* 0x2e0 */
    __IO uint32_t    RawErr_H;          /* 0x2e4 */

    __IO uint32_t    StatusTfr;         /* 0x2e8 */
    __IO uint32_t    StatusTfr_H;       /* 0x2ec */
    __IO uint32_t    StatusBlock;       /* 0x2f0 */
    __IO uint32_t    StatusBlock_H;     /* 0x2f4 */
    __IO uint32_t    StatusSrcTran;     /* 0x2f8 */
    __IO uint32_t    StatusSrcTran_H;   /* 0x2fc */
    __IO uint32_t    StatusDstTran;     /* 0x300 */
    __IO uint32_t    StatusDstTran_H;   /* 0x304 */
    __IO uint32_t    StatusErr;         /* 0x308 */
    __IO uint32_t    StatusErr_H;       /* 0x30c */

    __IO uint32_t    MaskTfr;           /* 0x310 */
    __IO uint32_t    MaskTfr_H;         /* 0x314 */
    __IO uint32_t    MaskBlock;         /* 0x318 */
    __IO uint32_t    MaskBlock_H;       /* 0x31c */
    __IO uint32_t    MaskSrcTran;       /* 0x320 */
    __IO uint32_t    MaskSrcTran_H;     /* 0x324 */
    __IO uint32_t    MaskDstTran;       /* 0x328 */
    __IO uint32_t    MaskDstTran_H;     /* 0x32c */
    __IO uint32_t    MaskErr;           /* 0x330 */
    __IO uint32_t    MaskErr_H;         /* 0x334 */

    __IO uint32_t    ClearTfr;          /* 0x338 */
    __IO uint32_t    ClearTfr_H;        /* 0x33c */
    __IO uint32_t    ClearBlock;        /* 0x340 */
    __IO uint32_t    ClearBlock_H;      /* 0x344 */
    __IO uint32_t    ClearSrcTran;      /* 0x348 */
    __IO uint32_t    ClearSrcTran_H;    /* 0x34c */
    __IO uint32_t    ClearDstTran;      /* 0x350 */
    __IO uint32_t    ClearDstTran_H;    /* 0x354 */
    __IO uint32_t    ClearErr;          /* 0x358 */
    __IO uint32_t    ClearErr_H;        /* 0x35c */
    __IO uint32_t    StatusInt;         /* 0x360 */
    __IO uint32_t    StatusInt_H;       /* 0x364 */
} AP_DMA_INT_TypeDef;

typedef struct
{
    __IO uint32_t    ReqSrcReg;                /* 0x368 */
    __IO uint32_t    ReqSrcReg_H;              /* 0x36c */
    __IO uint32_t    ReqDstReg;                /* 0x370 */
    __IO uint32_t    ReqDstReg_H;              /* 0x374 */
    __IO uint32_t    SglReqSrcReg;             /* 0x378 */
    __IO uint32_t    SglReqSrcReg_H;           /* 0x37c */
    __IO uint32_t    SglReqDstReg;             /* 0x380 */
    __IO uint32_t    SglReqDstReg_H;           /* 0x384 */
    __IO uint32_t    LstSrcReg;                /* 0x388 */
    __IO uint32_t    LstSrcReg_H;              /* 0x38c */
    __IO uint32_t    LstDstReg;                /* 0x390 */
    __IO uint32_t    LstDstReg_H;              /* 0x394 */
} AP_DMA_SW_HANDSHAKE_TypeDef;

typedef struct
{
    __IO uint32_t    DmaCfgReg;                /* 0x398 */
    __IO uint32_t    DmaCfgReg_H;              /* 0x39c */
    __IO uint32_t    ChEnReg;                  /* 0x3a0 */
    __IO uint32_t    ChEnReg_H;                /* 0x3a4 */
    __IO uint32_t    DmaIdReg;                 /* 0x3a8 */
    __IO uint32_t    DmaIdReg_H;               /* 0x3ac */
    __IO uint32_t    DmaTestReg;               /* 0x3b0 */
    __IO uint32_t    DmaTestReg_H;             /* 0x3b4 */
    __IO uint32_t    rsv1[4];
    __IO uint32_t    DMA_COMP_PARAMS_6;        /* 0x3c8 */
    __IO uint32_t    DMA_COMP_PARAMS_6_H;      /* 0x3cc */
    __IO uint32_t    DMA_COMP_PARAMS_5;        /* 0x3d0 */
    __IO uint32_t    DMA_COMP_PARAMS_5_H;      /* 0x3d4 */
    __IO uint32_t    DMA_COMP_PARAMS_4;        /* 0x3d8 */
    __IO uint32_t    DMA_COMP_PARAMS_4_H;      /* 0x3dc */
    __IO uint32_t    DMA_COMP_PARAMS_3;        /* 0x3e0 */
    __IO uint32_t    DMA_COMP_PARAMS_3_H;      /* 0x3e4 */
    __IO uint32_t    DMA_COMP_PARAMS_2;        /* 0x3e8 */
    __IO uint32_t    DMA_COMP_PARAMS_2_H;      /* 0x3ec */
    __IO uint32_t    DMA_COMP_PARAMS_1;        /* 0x3f0 */
    __IO uint32_t    DMA_COMP_PARAMS_1_H;      /* 0x3f4 */
    __IO uint32_t    DMA_ID;                   /* 0x3f8 */
    __IO uint32_t    DMA_ID_H;                 /* 0x3fc */
} AP_DMA_MISC_TypeDef;

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/****************************************************************************
 * Peripheral memory map(AP)
 ****************************************************************************/

/* Base addresses */

#define AP_APB0_BASE            (0x40000000UL)
#define SPIF_BASE_ADDR          (0x11000000)           /* spif */

#define AP_PCR_BASE             (AP_APB0_BASE + 0x0000) /* pcr */ /* APB0 peripherals   */

#define AP_TIM1_BASE            (AP_APB0_BASE + 0x1000)
#define AP_TIM2_BASE            (AP_APB0_BASE + 0x1014)
#define AP_TIM3_BASE            (AP_APB0_BASE + 0x1028)
#define AP_TIM4_BASE            (AP_APB0_BASE + 0x103c)
#define AP_TIM5_BASE            (AP_APB0_BASE + 0x1050)
#define AP_TIM6_BASE            (AP_APB0_BASE + 0x1064)
#define AP_TIM_SYS_BASE         (AP_APB0_BASE + 0x10a0)

#define AP_WDT_BASE             (AP_APB0_BASE + 0x2000)
#define AP_COM_BASE             (AP_APB0_BASE + 0x3000)  /* com */
#define AP_IOMUX_BASE           (AP_APB0_BASE + 0x3800)  /* iomux */
#define AP_UART0_BASE           (AP_APB0_BASE + 0x4000)  /* uart0 */
#define AP_I2C0_BASE            (AP_APB0_BASE + 0x5000)  /* i2c0 */
#define AP_I2C1_BASE            (AP_APB0_BASE + 0x5800)  /* i2c1 */
#define AP_SPI0_BASE            (AP_APB0_BASE + 0x6000)  /* spi0 */
#define AP_SPI1_BASE            (AP_APB0_BASE + 0x7000)  /* spi1 */
#define AP_GPIOA_BASE           (AP_APB0_BASE + 0x8000)  /* gpio */
#define AP_UART1_BASE           (AP_APB0_BASE + 0x9000)  /* uart1 */
#define AP_DMIC_BASE            (AP_APB0_BASE + 0xA000)
#define AP_QDEC_BASE            (AP_APB0_BASE + 0xB000)  /* qdec */
#define AP_CACHE_BASE           (AP_APB0_BASE + 0xC000)
#define AP_SPIF_BASE            (AP_APB0_BASE + 0xC800)  /* spif */
#define AP_KSCAN_BASE           (AP_APB0_BASE + 0xD0C0)  /* kscan */
#define AP_PWM_BASE             (AP_APB0_BASE + 0xE000)  /* pwm */
#define AP_AON_BASE             (AP_APB0_BASE + 0xF000)  /* aon */
#define AP_RTC_BASE             (AP_APB0_BASE + 0xF024)  /* rtc */
#define AP_PCRM_BASE            (AP_APB0_BASE + 0xF03c)  /* pcrm */
#define AP_WAKEUP_BASE          (AP_APB0_BASE + 0xF0a0)  /* wakeup */
#define AP_DMAC_BASE            (AP_APB0_BASE + 0x10000) /* dmac */
#define ADCC_BASE_ADDR          (AP_APB0_BASE + 0x50000) /* adcc */

/* bb_top */ /* linklayer */

#define SRAM0_BASE_ADDRESS      0x1FFF0000
#define SRAM1_BASE_ADDRESS      0x1FFF4000
#define SRAM2_BASE_ADDRESS      0x1FFF8000

#define AP_PCR                  ((AP_PCR_TypeDef *) AP_PCR_BASE)

#define AP_TIM1                 ((AP_TIM_TypeDef *) AP_TIM1_BASE)
#define AP_TIM2                 ((AP_TIM_TypeDef *) AP_TIM2_BASE)
#define AP_TIM3                 ((AP_TIM_TypeDef *) AP_TIM3_BASE)
#define AP_TIM4                 ((AP_TIM_TypeDef *) AP_TIM4_BASE)
#define AP_TIM5                 ((AP_TIM_TypeDef *) AP_TIM5_BASE)
#define AP_TIM6                 ((AP_TIM_TypeDef *) AP_TIM6_BASE)
#define AP_TIMS                 ((AP_TIM_SYS_TypeDef *) AP_TIM_SYS_BASE)

#define AP_WDT                  ((AP_WDT_TypeDef *) AP_WDT_BASE)
#define AP_COM                  ((AP_COM_TypeDef *) AP_COM_BASE)
#define AP_IOMUX                ((IOMUX_TypeDef *) AP_IOMUX_BASE)
#define AP_UART0                ((AP_UART_TypeDef *) AP_UART0_BASE)
#define AP_I2C0                 ((AP_I2C_TypeDef  *) AP_I2C0_BASE)
#define AP_I2C1                 ((AP_I2C_TypeDef  *) AP_I2C1_BASE)
#define AP_SPI0                 ((AP_SSI_TypeDef *) AP_SPI0_BASE)
#define AP_SPI1                 ((AP_SSI_TypeDef *) AP_SPI1_BASE)
#define AP_GPIO                 ((AP_GPIO_TypeDef *) AP_GPIOA_BASE)
#define AP_UART1                ((AP_UART_TypeDef *) AP_UART1_BASE)
#define AP_CACHE                ((AP_CACHE_TypeDef *) AP_CACHE_BASE)
#define AP_SPIF                 ((AP_SPIF_TypeDef  *) AP_SPIF_BASE)
#define AP_KSCAN                ((AP_KSCAN_TypeDef *) AP_KSCAN_BASE)
#define AP_PWM                  ((AP_PWM_TypeDef *) AP_PWM_BASE)
#define AP_PWM_CTRL(n)          ((AP_PWMCTRL_TypeDef *) (AP_PWM_BASE + 4 + n*12))
#define AP_AON                  ((AP_AON_TypeDef  *) AP_AON_BASE)
#define AP_RTC                  ((AP_RTC_TypeDef  *) AP_RTC_BASE)
#define AP_PCRM                 ((AP_PCRM_TypeDef  *) AP_PCRM_BASE)
#define AP_WAKEUP               ((AP_Wakeup_TypeDef*) AP_WAKEUP_BASE)
#define AP_ADCC                 ((AP_ADCC_TypeDef  *) ADCC_BASE_ADDR)

#define AP_DMA_CH_CFG(n)        ((AP_DMA_CH_TypeDef  *)     (AP_DMAC_BASE + 0x58*n))
#define AP_DMA_INT              ((AP_DMA_INT_TypeDef  *)    (AP_DMAC_BASE + 0x2c0))
#define AP_DMA_SW_HANDSHAKE     ((AP_DMA_SW_HANDSHAKE_TypeDef  *) (AP_DMAC_BASE + 0x368))
#define AP_DMA_MISC             ((AP_DMA_MISC_TypeDef  *)   (AP_DMAC_BASE + 0x398))

/****************************************************************************
 * Peripheral memory map(CP)
 ****************************************************************************/

/* Base addresses */
#define IRQ_PRIO_REALTIME     0
#define IRQ_PRIO_HIGH         1
#define IRQ_PRIO_HAL          2
#define IRQ_PRIO_THREAD       3
#define IRQ_PRIO_APP          3

#endif /* __ARCH_ARM_SRC_PHY62XX_MCU_PHY_BUMBEE_H */
