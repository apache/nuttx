/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_dtc.c
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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_internal.h"
#include "chip.h"
#include "rx65n_definitions.h"
#include "rx65n_dtc.h"

#include <arch/board/board.h>
#include <arch/board/rx65n_gpio.h>

#if defined(CONFIG_RX65N_DTC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DTC IP version */

#define DTC_IP_VER_DTC      (0)
#define DTC_IP_VER_DTCa     (1)
#define DTC_IP_VER_DTCb     (2)

/* Supportted DTC IP version and channel number */

#if defined(CONFIG_ARCH_BOARD_RX65N_RSK2MB) || defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
#define DTC_IP_VER DTC_IP_VER_DTCb
#define DTC_NCHANNELS     (1)
#define DTC_CHANNEL_ID    (0)
#endif

/* Total number of interrupt source that can start DTC */

#if defined(CONFIG_ARCH_BOARD_RX65N_RSK2MB)
#define DTC_NUM_INTERRUPT_SRC  (201)
#else
#define DTC_NUM_INTERRUPT_SRC  (199)
#endif

/* DTC supports same number as all interrupt sources that can start
 * the DTC transfer
 */

#define  DTC_TRANSFER_NCHANNELS DTC_NUM_INTERRUPT_SRC

/* Vector number related macro */

#define DTC_VECTOR_NUMBER (256) /* Hexa Decimal value is ox100 */
#define DTC_EACH_VECTOR_SIZE (4)
#define DTC_VECTOR_TABLE_SIZE ((DTC_VECTOR_NUMBER) * (DTC_EACH_VECTOR_SIZE))
#define DTC_INDEX_TABLE_SIZE ((DTC_VECTOR_NUMBER) * (DTC_EACH_VECTOR_SIZE))
#define DTC_VECTOR_ADDRESS_ALIGN (0x400) /* lower 10 bits of the base address */
#define DTC_VECTOR_ADDRESS_MASK 0xfffffc00
#define DTC_MAX_VECT_NUMBER DTCE_PERIA_INTA255
#define DTC_MIN_VECT_NUMBER DTCE_ICU_SWINT2

#define DTC_INVALID_CMND      ((uint32_t)(0x00000001))

#if defined (CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)

/* Size of DTC Vector table and DTC Index table (in byte units) */

#define DTC_VECTOR_TABLE_SIZE_BYTES (DTC_VECTOR_ADDRESS_ALIGN + DTC_VECTOR_TABLE_SIZE + DTC_INDEX_TABLE_SIZE)

#else

/* Size of DTC Vector table (in byte units) */

#define DTC_VECTOR_TABLE_SIZE_BYTES (DTC_VECTOR_ADDRESS_ALIGN + DTC_VECTOR_TABLE_SIZE)

#endif

/* DTC register mask and value */

#define    DTC_ACT_BIT_MASK            (0x8000) /* DTC Active flag (DTCSTS.ACT) bit mask */
#define    DTC_VECT_NR_MASK            (0x00FF) /* DTC-Activating Vector Number bits mask */
#define    DTC_MAX_16BITS_COUNT_VAL    (65536)  /* The maximum value of 16bit count value */
#define    DTC_MAX_8BITS_COUNT_VAL     (256)    /* The maximum value of 8bit count value */
#define    DTC_MIN_COUNT_VAL           (1)      /* The minimum of count value  and block size */
#define    DTC_ESPSEL_BIT_MASK         (0x8000) /* DTC Sequence transfer vector number setting bit mask */

/****************************************************************************
 * Typedef definitions
 ****************************************************************************/

/* The DTC Mode Internal Register A (MRA) structure */

struct st_dtc_mra_bit
{
#if (DTC_IP_VER == DTC_IP_VER_DTCa)
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t rs:2; /* reserved */
  uint8_t SM:2; /* Transfer Source Address Addressing Mode */
  uint8_t SZ:2; /* DTC Data Transfer Size */
  uint8_t MD:2; /* b7,b6: DTC Transfer Mode Select */
#else
  uint8_t MD:2; /* b7,b6: DTC Transfer Mode Select */
  uint8_t SZ:2; /* DTC Data Transfer Size */
  uint8_t SM:2; /* Transfer Source Address Addressing Mode */
  uint8_t rs:2; /* reserved */
#endif
#else /* (DTC_IP_VER == DTC_IP_VER_DTCb) */
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t WBDIS:1; /* Write-back Disable */
  uint8_t rs:1;    /* reserved */
  uint8_t SM:2;    /* Transfer Source Address Addressing Mode */
  uint8_t SZ:2;    /* DTC Data Transfer Size */
  uint8_t MD:2;    /* b7,b6: DTC Transfer Mode Select */
#else
  uint8_t MD:2;    /* b7,b6: DTC Transfer Mode Select */
  uint8_t SZ:2;    /* DTC Data Transfer Size */
  uint8_t SM:2;    /* Transfer Source Address Addressing Mode */
  uint8_t rs:1;    /* reserved */
  uint8_t WBDIS:1; /* Write-back Disable */
#endif
#endif
};

typedef union dtc_mra
{
  uint8_t BYTE;
  struct st_dtc_mra_bit BIT;
} dtc_mra_t;

/* The DTC Mode Internal Register B (MRB) structure */

struct st_dtc_mrb_bit
{
#if (DTC_IP_VER == DTC_IP_VER_DTCa)
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t rs:2;    /* reserved */
  uint8_t DM  :2;  /* Transfer Destination Address Addressing Mode */
  uint8_t DTS  :1; /* DTC Transfer Mode Select */
  uint8_t DISEL:1; /* DTC Interrupt Select */
  uint8_t CHNS :1; /* DTC Chain Transfer Select */
  uint8_t CHNE :1; /* b7: DTC Chain Transfer Enable */
#else
  uint8_t CHNE :1; /* b7: DTC Chain Transfer Enable */
  uint8_t CHNS :1; /* DTC Chain Transfer Select */
  uint8_t DISEL:1; /* DTC Interrupt Select */
  uint8_t DTS  :1; /* DTC Transfer Mode Select */
  uint8_t DM   :2; /* Transfer Destination Address Addressing Mode */
  uint8_t rs   :2; /* reserved */
#endif
#else /* (DTC_IP_VER == DTC_IP_VER_DTCb) */
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t SQEND:1;  /* Sequence Transfer End */
  uint8_t INDX:1;   /* Index Table Reference */
  uint8_t DM   :2;  /* Transfer Destination Address Addressing Mode */
  uint8_t DTS  :1;  /* DTC Transfer Mode Select */
  uint8_t DISEL:1;  /* DTC Interrupt Select */
  uint8_t CHNS :1;  /* DTC Chain Transfer Select */
  uint8_t CHNE :1;  /* b7: DTC Chain Transfer Enable */
#else
  uint8_t CHNE :1;  /* b7: DTC Chain Transfer Enable */
  uint8_t CHNS :1;  /* DTC Chain Transfer Select */
  uint8_t DISEL:1;  /* DTC Interrupt Select */
  uint8_t DTS  :1;  /* DTC Transfer Mode Select */
  uint8_t DM   :2;  /* Transfer Destination Address Addressing Mode */
  uint8_t INDX:1;   /* Index Table Reference */
  uint8_t SQEND:1;  /* Sequence Transfer End */
#endif
#endif
};

typedef union dtc_mrb
{
  uint8_t BYTE;
  struct st_dtc_mrb_bit BIT;
} dtc_mrb_t;

#if (DTC_IP_VER == DTC_IP_VER_DTCb)
struct st_dtc_mrc_bit
{
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t DISPE :1;
  uint8_t rs :7;    /* reserved */
#else
  uint8_t rs :7;    /* reserved */
  uint8_t DISPE :1;
#endif
};

typedef union dtc_mrc
{
  uint8_t BYTE;
  struct st_dtc_mrc_bit BIT;
} dtc_mrc_t;

#endif /* (DTC_IP_VER == DTC_IP_VER_DTCb) */

/* The DTC Transfer Count Register A (CRA) structure */

struct st_dtc_cra_byte
{
#ifdef __RX_LITTLE_ENDIAN__
  uint8_t CRA_L;
  uint8_t CRA_H;
#else
  uint8_t CRA_H;
  uint8_t CRA_L;
#endif
};

typedef union dtc_cra
{
  uint16_t WORD;
  struct st_dtc_cra_byte BYTE;
} dtc_cra_t;

/* The DTC Transfer Count Register B (CRB) structure */

typedef union dtc_crb
{
  uint16_t WORD;
} dtc_crb_t;

#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE) /* Transfer data in short-address mode */

struct st_first_word
  {
#ifdef __RX_LITTLE_ENDIAN__
    uint8_t SAR[3];
    dtc_mra_t MRA;
#else
    dtc_mra_t MRA;
    uint8_t SAR[3];
#endif
  };

struct st_second_word
  {
#ifdef __RX_LITTLE_ENDIAN__
    uint8_t SAR[3];
    dtc_mrb_t MRB;
#else
    dtc_mrb_t MRB;
    uint8_t DAR[3];
#endif
  };

struct st_third_word
  {
#ifdef __RX_LITTLE_ENDIAN__
    dtc_crb_t CRB;
    dtc_cra_t CRA;
#else
    dtc_cra_t CRA;
    dtc_crb_t CRB;
#endif
  };

typedef union lword1
  {
    uint32_t LWORD;
    struct st_first_word REG;
  } lword1_t;

typedef union lword2
  {
    uint32_t LWORD;
    struct st_second_word REG;
  } lword2_t;

typedef union lword3
  {
    uint32_t LWORD;
    struct st_third_word REG;
  } lword3_t;

typedef struct st_dtc_short_transfer_data
  {
    lword1_t FIRST_LWORD;
    lword2_t SECOND_LWORD;
    lword3_t THIRD_LWORD;
  } st_dtc_internal_registers_t;

#else /* Transfer data in full-address mode */
struct st_first_lword
  {
#ifdef __RX_LITTLE_ENDIAN__
#if (DTC_IP_VER == DTC_IP_VER_DTCa)
  uint16_t reserver; /* reserve area */
#else
  uint8_t reserver; /* reserve area */
  dtc_mrc_t MRC;
#endif
  dtc_mrb_t MRB;
  dtc_mra_t MRA;
#else
  dtc_mra_t MRA;
  dtc_mrb_t MRB;
#if (DTC_IP_VER == DTC_IP_VER_DTCa)
  uint16_t reserver; /* reserve area */
#else
  dtc_mrc_t MRC;
  uint8_t reserver; /* reserve area */
#endif

#endif
  };

struct st_fourth_lword
  {
#ifdef __RX_LITTLE_ENDIAN__
    dtc_crb_t CRB;
    dtc_cra_t CRA;
#else
    dtc_cra_t CRA;
    dtc_crb_t CRB;
#endif
  };

typedef union lword1
  {
    uint32_t LWORD;
    struct st_first_lword REG;
  } lword1_t;

typedef union lword2
  {
    uint32_t SAR;
  }lword2_t;

typedef union lword3
  {
    uint32_t DAR;
  } lword3_t;

typedef union lword4
  {
    uint32_t LWORD;
    struct st_fourth_lword REG;
  } lword4_t;

typedef struct st_dtc_full_transfer_data
  {
    lword1_t FIRST_LWORD;
    lword2_t SECOND_LWORD;
    lword3_t THIRD_LWORD;
    lword4_t FOURTH_LWORD;
  } st_dtc_internal_registers_t;

#endif /* DTC_CFG_SHORT_ADDRESS_MODE */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one dtc channel */

struct rx65n_dtc_s
{
  uint8_t chan;           /* DTC channel number */
  bool initialized;       /* Initialization status */

  uint32_t base;          /* DTC channel register base address */

  uint8_t * vectortable; /* Vector table pointer */
#if defined (CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)
  uint8_t * indextable;  /* Index table pointer for sequence transfer */
#endif

  uint8_t addmode;        /* Address mode */

#if defined(CONFIG_RX65N_DTC_TRANSFER_DATA_READ_SKIP)
  uint8_t readskip;       /* Read skip enable or disable */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Statically allocated vector table */

uint8_t vectortable[DTC_VECTOR_TABLE_SIZE_BYTES];

/* This array describes the state of each dtc: Only one channel */

static struct rx65n_dtc_s g_dtchandle[DTC_NCHANNELS] =
{
  {
    .chan  = DTC_CHANNEL_ID,
    .base  = RX65N_DTC_BASE,
    .vectortable = vectortable,
  },
};

/* The array of all interrupt source */

const dtc_activation_source_t g_dtcsource[DTC_NUM_INTERRUPT_SRC] =
{
  DTCE_ICU_SWINT2, DTCE_ICU_SWINT,
  DTCE_CMT0_CMI0,
  DTCE_CMT1_CMI1,
  DTCE_CMTW0_CMWI0,
  DTCE_CMTW1_CMWI1,
  DTCE_USB0_D0FIFO0, DTCE_USB0_D1FIFO0,
  DTCE_RSPI0_SPRI0, DTCE_RSPI0_SPTI0,
  DTCE_RSPI1_SPRI1, DTCE_RSPI1_SPTI1,
  DTCE_QSPI_SPRI, DTCE_QSPI_SPTI,
  DTCE_SDHI_SBFAI,
  DTCE_MMCIF_MBFAI,
#if defined(BSP_MCU_RX65N_2MB)
  DTCE_RIIC0_RXI0, DTCE_RIIC0_TXI0,
  DTCE_RIIC1_RXI1, DTCE_RIIC1_TXI1,
  DTCE_RIIC2_RXI2, DTCE_RIIC2_TXI2,
#else
  DTCE_RIIC0_RXI0, DTCE_RIIC0_TXI0,
  DTCE_RIIC2_RXI2, DTCE_RIIC2_TXI2,
#endif /* defined(BSP_MCU_RX65N_2MB) */
  DTCE_SCI0_RXI0, DTCE_SCI0_TXI0,
  DTCE_SCI1_RXI1, DTCE_SCI1_TXI1,
  DTCE_SCI2_RXI2, DTCE_SCI2_TXI2,
  DTCE_ICU_IRQ0, DTCE_ICU_IRQ1, DTCE_ICU_IRQ2, DTCE_ICU_IRQ3,
  DTCE_ICU_IRQ4, DTCE_ICU_IRQ5, DTCE_ICU_IRQ6, DTCE_ICU_IRQ7,
  DTCE_ICU_IRQ8, DTCE_ICU_IRQ9, DTCE_ICU_IRQ10, DTCE_ICU_IRQ11,
  DTCE_ICU_IRQ12, DTCE_ICU_IRQ13, DTCE_ICU_IRQ14,
  DTCE_ICU_IRQ15,
  DTCE_SCI3_RXI3, DTCE_SCI3_TXI3,
  DTCE_SCI4_RXI4, DTCE_SCI4_TXI4,
  DTCE_SCI5_RXI5, DTCE_SCI5_TXI5,
  DTCE_SCI6_RXI6, DTCE_SCI6_TXI6,
  DTCE_PDC_PCDFI,
  DTCE_SCI7_RXI7, DTCE_SCI7_TXI7,
  DTCE_SCI8_RXI8, DTCE_SCI8_TXI8,
  DTCE_SCI9_RXI9, DTCE_SCI9_TXI9,
  DTCE_SCI10_RXI10, DTCE_SCI10_TXI10,
  DTCE_RSPI2_SPRI2, DTCE_RSPI2_SPTI2,
  DTCE_SCI11_RXI11, DTCE_SCI11_TXI11,
  DTCE_SCI12_RXI12, DTCE_SCI12_TXI12,
  DTCE_DMAC_DMAC0I, DTCE_DMAC_DMAC1I, DTCE_DMAC_DMAC2I, DTCE_DMAC_DMAC3I,
  DTCE_EXDMAC_EXDMAC0I, DTCE_EXDMAC_EXDMAC1I,
  DTCE_PERIB_INTB128, DTCE_PERIB_INTB129, DTCE_PERIB_INTB130,
  DTCE_PERIB_INTB131, DTCE_PERIB_INTB132,
  DTCE_PERIB_INTB133, DTCE_PERIB_INTB134, DTCE_PERIB_INTB135,
  DTCE_PERIB_INTB136, DTCE_PERIB_INTB137,
  DTCE_PERIB_INTB138, DTCE_PERIB_INTB139, DTCE_PERIB_INTB140,
  DTCE_PERIB_INTB141, DTCE_PERIB_INTB142,
  DTCE_PERIB_INTB143, DTCE_PERIB_INTB144, DTCE_PERIB_INTB145,
  DTCE_PERIB_INTB146, DTCE_PERIB_INTB147,
  DTCE_PERIB_INTB148, DTCE_PERIB_INTB149, DTCE_PERIB_INTB150,
  DTCE_PERIB_INTB151, DTCE_PERIB_INTB152,
  DTCE_PERIB_INTB153, DTCE_PERIB_INTB154, DTCE_PERIB_INTB155,
  DTCE_PERIB_INTB156, DTCE_PERIB_INTB157,
  DTCE_PERIB_INTB158, DTCE_PERIB_INTB159, DTCE_PERIB_INTB160,
  DTCE_PERIB_INTB161, DTCE_PERIB_INTB162,
  DTCE_PERIB_INTB163, DTCE_PERIB_INTB164, DTCE_PERIB_INTB165,
  DTCE_PERIB_INTB166, DTCE_PERIB_INTB167,
  DTCE_PERIB_INTB168, DTCE_PERIB_INTB169, DTCE_PERIB_INTB170,
  DTCE_PERIB_INTB171, DTCE_PERIB_INTB172,
  DTCE_PERIB_INTB173, DTCE_PERIB_INTB174, DTCE_PERIB_INTB175,
  DTCE_PERIB_INTB176, DTCE_PERIB_INTB177,
  DTCE_PERIB_INTB178, DTCE_PERIB_INTB179, DTCE_PERIB_INTB180,
  DTCE_PERIB_INTB181, DTCE_PERIB_INTB182,
  DTCE_PERIB_INTB183, DTCE_PERIB_INTB184, DTCE_PERIB_INTB185,
  DTCE_PERIB_INTB186, DTCE_PERIB_INTB187,
  DTCE_PERIB_INTB188, DTCE_PERIB_INTB189, DTCE_PERIB_INTB190,
  DTCE_PERIB_INTB191, DTCE_PERIB_INTB192,
  DTCE_PERIB_INTB193, DTCE_PERIB_INTB194, DTCE_PERIB_INTB195,
  DTCE_PERIB_INTB196, DTCE_PERIB_INTB197,
  DTCE_PERIB_INTB198, DTCE_PERIB_INTB199, DTCE_PERIB_INTB200,
  DTCE_PERIB_INTB201, DTCE_PERIB_INTB202,
  DTCE_PERIB_INTB203, DTCE_PERIB_INTB204, DTCE_PERIB_INTB205,
  DTCE_PERIB_INTB206, DTCE_PERIB_INTB207,
  DTCE_PERIA_INTA208, DTCE_PERIA_INTA209, DTCE_PERIA_INTA210,
  DTCE_PERIA_INTA211, DTCE_PERIA_INTA212,
  DTCE_PERIA_INTA213, DTCE_PERIA_INTA214, DTCE_PERIA_INTA215,
  DTCE_PERIA_INTA216, DTCE_PERIA_INTA217,
  DTCE_PERIA_INTA218, DTCE_PERIA_INTA219, DTCE_PERIA_INTA220,
  DTCE_PERIA_INTA221, DTCE_PERIA_INTA222,
  DTCE_PERIA_INTA223, DTCE_PERIA_INTA224, DTCE_PERIA_INTA225,
  DTCE_PERIA_INTA226, DTCE_PERIA_INTA227,
  DTCE_PERIA_INTA228, DTCE_PERIA_INTA229, DTCE_PERIA_INTA230,
  DTCE_PERIA_INTA231, DTCE_PERIA_INTA232,
  DTCE_PERIA_INTA233, DTCE_PERIA_INTA234, DTCE_PERIA_INTA235,
  DTCE_PERIA_INTA236, DTCE_PERIA_INTA237,
  DTCE_PERIA_INTA238, DTCE_PERIA_INTA239, DTCE_PERIA_INTA240,
  DTCE_PERIA_INTA241, DTCE_PERIA_INTA242,
  DTCE_PERIA_INTA243, DTCE_PERIA_INTA244, DTCE_PERIA_INTA245,
  DTCE_PERIA_INTA246, DTCE_PERIA_INTA247,
  DTCE_PERIA_INTA248, DTCE_PERIA_INTA249, DTCE_PERIA_INTA250,
  DTCE_PERIA_INTA251, DTCE_PERIA_INTA252,
  DTCE_PERIA_INTA253, DTCE_PERIA_INTA254, DTCE_PERIA_INTA255
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rx65n_dtc_module_enable(void);
static dtc_err_t rx65n_dtc_set_dynamic_transfer_data
      (dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
           dtc_transfer_data_t * p_transfer_data);
static dtc_err_t rx65n_dtc_validate_dynamic_params
     (dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
      dtc_transfer_data_t * p_transfer_data);
static dtc_err_t rx65n_dtc_set_static_transfer_data(
                       dtc_static_transfer_data_cfg_t * p_transfer_cfg,
                       dtc_transfer_data_t * p_transfer_data);
static void rx65n_dtc_readskip_enable(DTC_HANDLE handle);
#ifndef CONFIG_RX65N_DTC_TRANSFER_DATA_READ_SKIP
static void rx65n_dtc_readskip_disable(DTC_HANDLE handle);
#endif

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/* Read register from DTC */

static inline uint32_t dtcchan_getreg(struct rx65n_dtc_s *dtc,
                                      uint32_t offset)
{
  return getreg32(dtc->base + offset);
}

/* Write to DTC register */

static inline void dtcchan_putreg(struct rx65n_dtc_s *dtc,
                                  uint32_t offset, uint32_t value)
{
  putreg32(value, dtc->base + offset);
}

/****************************************************************************
 * Function Name: rx65n_dtc_module_enable
 * Description  : Releases module stop state.
 * Arguments    : None
 * Return Value : None
 ****************************************************************************/

static void rx65n_dtc_module_enable(void)
{
  /* Enable writing to MSTP registers. */

  SYSTEM.PRCR.WORD = 0xa50b;
  MPC.PWPR.BIT.B0WI = 0;
  MPC.PWPR.BIT.PFSWE = 1;

  /* Release from module stop state. */

  MSTP(DTC) = 0;
}

/****************************************************************************
 * Function Name: rx65n_dtc_set_dynamic_transfer_data
 * Description  :
 *       Set the dynamic dynamic transfer parameter in transfer data
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 ****************************************************************************/

static dtc_err_t rx65n_dtc_set_dynamic_transfer_data
                (dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
                 dtc_transfer_data_t * p_transfer_data)
{
  /* Map transfer data to st_dtc_internal_registers_t */

  volatile st_dtc_internal_registers_t *td_ptr
            = (volatile st_dtc_internal_registers_t *)p_transfer_data;

  /* Get  MRA, CRA and CRB internal register from td_ptr */

#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE)/* Short address mode */
  volatile dtc_mra_t  *p_mra = &(td_ptr->FIRST_LWORD.REG.MRA);
  volatile dtc_cra_t *p_cra = &(td_ptr->THIRD_LWORD.REG.CRA);
  volatile dtc_crb_t *p_crb = &(td_ptr->THIRD_LWORD.REG.CRB);

#else /* Full address mode */
  volatile dtc_mra_t  *p_mra = &(td_ptr->FIRST_LWORD.REG.MRA);
  volatile dtc_cra_t *p_cra = &(td_ptr->FOURTH_LWORD.REG.CRA);
  volatile dtc_crb_t * p_crb = &(td_ptr->FOURTH_LWORD.REG.CRB);

#endif

  /* DTC data transfer size */

  if (((p_transfer_cfg->data_size) == DTC_DATA_SIZE_BYTE) ||
        ((p_transfer_cfg->data_size) == DTC_DATA_SIZE_WORD) ||
        ((p_transfer_cfg->data_size) == DTC_DATA_SIZE_LWORD))
    {
      p_mra->BYTE &= (~(3 << 4));
      p_mra->BYTE |= p_transfer_cfg->data_size;
    }
  else
    {
      return DTC_ERR_INVALID_ARG;
    }

  /* Set CRA and CRB internal register */

  switch (p_mra->BIT.MD) /* DTC transfer mode */
    {
      case 0x0: /* Normal mode */
        {
            if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
                p_cra->WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
                p_cra->WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }
        break;
        }

        case 0x1: /* Repeat mode */
        {
            /* Set counter. */

            if (p_transfer_cfg->transfer_count < DTC_MAX_8BITS_COUNT_VAL) /* count 1-255 */
            {
                p_cra->BYTE.CRA_H = (uint8_t)p_transfer_cfg->transfer_count;
                p_cra->BYTE.CRA_L = (uint8_t)p_transfer_cfg->transfer_count;
            }
            else if (DTC_MAX_8BITS_COUNT_VAL
                     == p_transfer_cfg->transfer_count)
            {
                p_cra->BYTE.CRA_H = 0x00;
                p_cra->BYTE.CRA_L = 0x00;
            }
            else /* Transfer count > 256 */
            {
                return DTC_ERR_INVALID_ARG;
            }
        break;
        }

        case 0x2: /* DTC_TRANSFER_MODE_BLOCK - Block transfer mode */
        {
            /* Set counter. */

            if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count) /* Transfer count = 65536 */
            {
                p_crb->WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
                p_crb->WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }

            if (p_transfer_cfg->block_size < DTC_MAX_8BITS_COUNT_VAL) /* Block size 1-255 */
            {
                p_cra->BYTE.CRA_H = (uint8_t)p_transfer_cfg->block_size;
                p_cra->BYTE.CRA_L = (uint8_t)p_transfer_cfg->block_size;
            }
            else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->block_size) /* Block size = 256 */
            {
                p_cra->BYTE.CRA_H = 0;
                p_cra->BYTE.CRA_L = 0;
            }
            else /* Invalid block size */
            {
                return DTC_ERR_INVALID_ARG;
            }
        break;
        }

        default:
        {
            return DTC_ERR_INVALID_ARG;
        break;
        }
    }

  /* Set Source and destination address in transfer memory area */

#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE)

  /* 3 byte SAR */

  td_ptr->FIRST_LWORD.LWORD &= ~(0x00ffffff);
  td_ptr->FIRST_LWORD.LWORD |= (p_transfer_cfg->source_addr & 0x00ffffff);

  /* 3 byte DAR */

  td_ptr->SECOND_LWORD.LWORD &= ~(0x00ffffff);
  td_ptr->SECOND_LWORD.LWORD   |= (p_transfer_cfg->dest_addr & 0x00ffffff);

#else /* Full address mode */

  /* settings for second long word: SAR */

  td_ptr->SECOND_LWORD.SAR = p_transfer_cfg->source_addr; /* 4 byte SAR */

  /* settings for third long word: DAR */

  td_ptr->THIRD_LWORD.DAR = p_transfer_cfg->dest_addr; /* 4 byte DAR */

#endif
  return DTC_SUCCESS;
}

/****************************************************************************
 * Function Name: rx65n_dtc_validate_dynamic_params
 * Description  :
 *       Validate the dynamic parameter
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 *
 ****************************************************************************/

static dtc_err_t rx65n_dtc_validate_dynamic_params
           (dtc_dynamic_transfer_data_cfg_t * p_transfer_cfg,
            dtc_transfer_data_t * p_transfer_data)
{
  if ((NULL == p_transfer_cfg) || (NULL == p_transfer_data))
    {
      return DTC_ERR_NULL_PTR;
    }

  /* Validate transfer count */

  if ((p_transfer_cfg->transfer_count < DTC_MIN_COUNT_VAL) ||
        (p_transfer_cfg->transfer_count > DTC_MAX_16BITS_COUNT_VAL))
    {
      return DTC_ERR_INVALID_ARG;
    }

#if defined (CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE) /* Short-address mode */
/* Address must be in: 0x00000000h to 0x007FFFFF and
 * 0xFF800000 to 0xFFFFFFFF
 */

  if ((p_transfer_cfg->source_addr > 0x007fffff)
            && (p_transfer_cfg->source_addr < 0xff800000))
    {
      return DTC_ERR_INVALID_ARG;
    }

  if ((p_transfer_cfg->dest_addr > 0x007fffff)
            && (p_transfer_cfg->dest_addr < 0xff800000))
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (((uint32_t)p_transfer_data > 0x007fffff)
            && ((uint32_t)p_transfer_data < 0xff800000))
    {
      return DTC_ERR_INVALID_ARG;
    }

#endif
  return DTC_SUCCESS;
}

/****************************************************************************
 * Function Name: rx65n_dtc_set_transfer_data
 * Description  :
 *       Set transfer data
 *
 * Input Parameters :
 *    p_transfer_cfg - Reguested configuration
 *    p_transfer_data  - Transfer information RAM memory
 *
 * Return :
 *     dtc_err_t -  error codes
 *
 ****************************************************************************/

static dtc_err_t rx65n_dtc_set_static_transfer_data(
                        dtc_static_transfer_data_cfg_t * p_transfer_cfg,
                          dtc_transfer_data_t * p_transfer_data)
{
  dtc_mra_t  t_mra;
  dtc_mrb_t  t_mrb;
  dtc_cra_t  t_cra;
  dtc_crb_t  t_crb;

  /* Initialize crb. When normal transfer mode or repeat transfer mode is
   * selected, this register is not used and the set value is ignored.
   */

  t_crb.WORD = 0x0000;

  /* Map transfer data to st_dtc_internal_registers_t */

  volatile st_dtc_internal_registers_t *td_ptr =
       (volatile st_dtc_internal_registers_t *)p_transfer_data;

  /* Prepare  MRA and MRB internal register */

#if (DTC_IP_VER >= DTC_IP_VER_DTCb)
#ifndef CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE /* Full-address mode */
  dtc_mrc_t  t_mrc;
  t_mrc.BYTE = (uint8_t)(p_transfer_cfg->disp_add_enable);
#else
  t_mra.BYTE = (uint8_t)(p_transfer_cfg->writeback_disable
                           | p_transfer_cfg->src_addr_mode
                           | p_transfer_cfg->data_size
                           | p_transfer_cfg->transfer_mode);
  t_mrb.BYTE = (uint8_t)(p_transfer_cfg->sequence_end
                           | p_transfer_cfg->refer_index_table_enable
                           | p_transfer_cfg->dest_addr_mode
                           | p_transfer_cfg->repeat_block_side
                           | p_transfer_cfg->response_interrupt
                           | p_transfer_cfg->chain_transfer_enable
                           | p_transfer_cfg->chain_transfer_mode);
#endif
  t_mra.BYTE = (uint8_t)(p_transfer_cfg->src_addr_mode
                                      | p_transfer_cfg->data_size
                                      | p_transfer_cfg->transfer_mode);

     t_mrb.BYTE = (uint8_t)(p_transfer_cfg->dest_addr_mode
                         | p_transfer_cfg->repeat_block_side
                         | p_transfer_cfg->response_interrupt
                          | p_transfer_cfg->chain_transfer_enable
                          | p_transfer_cfg->chain_transfer_mode);
   #endif

  /* Prepare CRA and CRB internal register */

  switch (t_mra.BIT.MD) /* DTC transfer mode */
    {
      case 0x0: /* Normal mode */
        {
          if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
              t_cra.WORD = 0x0000;
            }
            else /* 1 - 65535 */
            {
              t_cra.WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }
          break;
        }

      case 0x1: /* Repeat mode */
        {
          /* Set counter. */

          if (p_transfer_cfg->transfer_count < DTC_MAX_8BITS_COUNT_VAL) /* count 1-255 */
            {
              t_cra.BYTE.CRA_H = (uint8_t)p_transfer_cfg->transfer_count;
              t_cra.BYTE.CRA_L = (uint8_t)p_transfer_cfg->transfer_count;
            }
          else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->transfer_count)
            {
              t_cra.BYTE.CRA_H = 0x00;
              t_cra.BYTE.CRA_L = 0x00;
            }
          else /* Transfer count > 256 */
            {
              return DTC_ERR_INVALID_ARG;
            }
          break;
        }

      case 0x2: /* DTC_TRANSFER_MODE_BLOCK - Block transfer mode */
        {
          /* Set counter. */

          if (DTC_MAX_16BITS_COUNT_VAL == p_transfer_cfg->transfer_count)/* Transfer count = 65536 */
            {
              t_crb.WORD = 0x0000;
            }
          else /* 1 - 65535 */
            {
              t_crb.WORD = (uint16_t)p_transfer_cfg->transfer_count;
            }

          if (p_transfer_cfg->block_size < DTC_MAX_8BITS_COUNT_VAL) /* Block size 1-255 */
            {
              t_cra.BYTE.CRA_H = (uint8_t)p_transfer_cfg->block_size;
              t_cra.BYTE.CRA_L = (uint8_t)p_transfer_cfg->block_size;
            }
          else if (DTC_MAX_8BITS_COUNT_VAL == p_transfer_cfg->block_size) /* Block size = 256 */
            {
              t_cra.BYTE.CRA_H = 0;
              t_cra.BYTE.CRA_L = 0;
            }
          else /* Invalid block size */
            {
              return DTC_ERR_INVALID_ARG;
            }
          break;
        }

      default:
        {
          return DTC_ERR_INVALID_ARG;
          break;
        }
    }

  /* Set transfer information in RAM area */
#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE)
  /* settings for fist long word: MRA & SAR */

  td_ptr->FIRST_LWORD.LWORD   =  0;     /* clear */
  td_ptr->FIRST_LWORD.REG.MRA =  t_mra; /* 1 byte MRA */
  td_ptr->FIRST_LWORD.LWORD |= (p_transfer_cfg->source_addr & 0x00ffffff);

  /* settings for second long word: MRB & DAR */

  td_ptr->SECOND_LWORD.LWORD   =  0;     /* clear */
  td_ptr->SECOND_LWORD.REG.MRB =  t_mrb; /* 1 byte MRB */
  td_ptr->SECOND_LWORD.LWORD   |= (p_transfer_cfg->dest_addr & 0x00ffffff);

  /* settings for third long word: CRA & CRB */

  td_ptr->THIRD_LWORD.REG.CRA.WORD = t_cra.WORD;

  td_ptr->THIRD_LWORD.REG.CRB.WORD = t_crb.WORD;

#else /* Full address mode */
  /* settings for fist long word: MRA & MRB */

  td_ptr->FIRST_LWORD.REG.MRA.BYTE = t_mra.BYTE; /* 1 byte MRA */
  td_ptr->FIRST_LWORD.REG.MRB.BYTE = t_mrb.BYTE; /* 1 byte MRB */
#if (DTC_IP_VER_DTCb <= DTC_IP_VER)
  td_ptr->FIRST_LWORD.REG.MRC.BYTE = t_mrc.BYTE; /* 1 byte MRC */
#endif /* (DTC_IP_VER_DTCb <= DTC_IP) */

  /* settings for second long word: SAR */

  td_ptr->SECOND_LWORD.SAR = p_transfer_cfg->source_addr; /* 4 byte SAR */

  /* settings for third long word: DAR */

  td_ptr->THIRD_LWORD.DAR = p_transfer_cfg->dest_addr; /* 4 byte DAR */

  /* settings for fourth long word: CRA & CRB */

  td_ptr->FOURTH_LWORD.REG.CRA.WORD = t_cra.WORD;

  td_ptr->FOURTH_LWORD.REG.CRB.WORD = t_crb.WORD;

#endif
  return DTC_SUCCESS;
}

/****************************************************************************
 * Name: rx65n_dtc_readskip_enable
 *
 * Description:
 *   DTC Transfer Information Read Skip enable
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

static void rx65n_dtc_readskip_enable(DTC_HANDLE handle)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  uint8_t regval8;

  /* Configure Read skip bit in DTCCR register */

  regval8 = dtcchan_getreg(dtchandle, DTC_DTCCR_OFFSET);
  regval8 |= (DTC_DTCCR_RRS);
  dtcchan_putreg(dtchandle, DTC_DTCCR_OFFSET, regval8);
}

/****************************************************************************
 * Name: rx65n_dtc_readskip_disable
 *
 * Description:
 *   DTC Transfer Information Read Skip disable
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/
#ifndef CONFIG_RX65N_DTC_TRANSFER_DATA_READ_SKIP
static void rx65n_dtc_readskip_disable(DTC_HANDLE handle)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  uint8_t regval8;

  /* Configure Read skip bit in DTCCR register */

  regval8 = dtcchan_getreg(dtchandle, DTC_DTCCR_OFFSET);
  regval8 &= ~(DTC_DTCCR_RRS);
  dtcchan_putreg(dtchandle, DTC_DTCCR_OFFSET, regval8);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_dtc_start
 *
 * Description:
 *   DTC Module Start
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_start(DTC_HANDLE handle)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  uint8_t regval8;

  /* Configure Read skip bit in DTCST register */

  regval8 = dtcchan_getreg(dtchandle, DTC_DTCST_OFFSET);
  regval8 |= (DTC_DTCST_DTCST);
  dtcchan_putreg(dtchandle, DTC_DTCST_OFFSET, regval8);
}

/****************************************************************************
 * Name: rx65n_dtc_stop
 *
 * Description:
 *   DTC Module Stop
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_stop(DTC_HANDLE handle)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  uint8_t regval8;

  /* Configure Read skip bit in DTCST register */

  regval8 = dtcchan_getreg(dtchandle, DTC_DTCST_OFFSET);
  regval8 &= ~(DTC_DTCST_DTCST);
  dtcchan_putreg(dtchandle, DTC_DTCST_OFFSET, regval8);
}

/****************************************************************************
 * Name: rx65n_dtc_status
 *
 * Description:
 *   Output the status of DTC
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Output Parameters:
 *    p_stat - DTC status structure
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_get_dtchandle()
 *
 ****************************************************************************/

void rx65n_dtc_status(DTC_HANDLE handle, dtc_stat_t *p_stat)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  DEBUGASSERT(dtchandle != NULL || p_stat |= NULL);

  if (dtchandle->initialized)
    {
      /* Check DTC Status */

      if (0 == (DTC.DTCSTS.WORD & DTC_ACT_BIT_MASK)) /* DTC transfer operation is not in progress. */
        {
          p_stat->in_progress = false;

          /* DTC is not in progress. -> vector number is invalid. */
        }
      else /* DTC transfer operation is in progress. */
        {
          p_stat->in_progress = true;

      /* Get the current vector number. */

          p_stat->vect_nr = (uint8_t)(DTC.DTCSTS.WORD & DTC_VECT_NR_MASK);
        }
    }
}

#if defined(CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)
/****************************************************************************
 * Name: rx65n_dtc_seq_enable
 *
 * Description:
 *   DTC Module sequence transfer enable and set respective vector number
 *   for sequence transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source nothing but vector number
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_seq_enable(DTC_HANDLE handle, uint8_t src)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  /* Configure vector number for sequence transfer */

  DTC.DTCSQE.WORD = (DTC_ESPSEL_BIT_MASK | (uint16_t)act_source);
}

/****************************************************************************
 * Name: rx65n_dtc_seq_disable
 *
 * Description:
 *   DTC Module sequence transfer enable and clear vector number for
 *   sequence transfer in DTCSQE
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_seq_disable(DTC_HANDLE handle)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Clear vector number for sequence transfer */

  DTC.DTCSQE.WORD &= (~DTC_ESPSEL_BIT_MASK);
}
#endif

/****************************************************************************
 * Name: rx65n_dtc_srcactivation
 *
 * Description:
 *   Src activation enabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by rx65n_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void rx65n_dtc_srcactivation(DTC_HANDLE handle, uint8_t src)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  DEBUGASSERT(act_source >= DTC_MIN_VECT_NUMBER &&
              act_source <= DTC_MAX_VECT_NUMBER);

  if (dtchandle->initialized)
    {
      /* Enable the interrupt source */

      ICU.DTCER[act_source].BIT.DTCE = 1;
    }
}

/****************************************************************************
 * Name: rx65n_dtc_srcdeactivation
 *
 * Description:
 *   Src activation disabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by rx65n_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void rx65n_dtc_srcdeactivation(DTC_HANDLE handle, uint8_t src)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  DEBUGASSERT(handle != NULL);

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  DEBUGASSERT(act_source >= DTC_MIN_VECT_NUMBER &&
              act_source <= DTC_MAX_VECT_NUMBER);

  if (dtchandle->initialized)
    {
      /* Disable the interrupt source */

      ICU.DTCER[act_source].BIT.DTCE = 0;
    }
}

#if defined(CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)
/****************************************************************************
 * Name: rx65n_dtc_setup_dynamic_transferdata
 *
 * Description:
 *   Setup dynamic transfer info like src and destination address and
 *   number of data to transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    dcfg -Dynamic config parameter
 *    nseq_transfer - Number of sequence transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_seq_dynamic_transferdata(DTC_HANDLE handle,
                                          uint8_t src, uint32_t dcfg,
                                uint32_t nseq_transfer, uint8_t nseq)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  dtc_err_t ret = DTC_SUCCESS;
  uint32_t *ptr         = NULL;
  dtc_transfer_data_t *p_transfer_data = NULL;
  uint8_t   rrs_backup  = 0;
  uint8_t   dtce_backup = 0;
  irqstate_t flags;
  int i;

  /* Chain transfer count */

  uint32_t count = nseq_transfer;

  /* Dynamic user configuration */

  dtc_dynamic_transfer_data_cfg_t *p_dtransfer_cfg =
                     (dtc_dynamic_transfer_data_cfg_t *)dcfg;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  flags = enter_critical_section();

  /* Store old value of DTCERn.DTCE bit. */

  dtce_backup = ICU.DTCER[act_source].BIT.DTCE;

  /* Disable the interrupt source. Clear the DTCER */

  ICU.DTCER[act_source].BIT.DTCE = 0;

  /* Store RRS and clear RRS */

  rrs_backup =  DTC.DTCCR.BIT.RRS;
  DTC.DTCCR.BIT.RRS = 0;

  /* Get Transfer data pointer */

  ptr = (uint32_t *)(dtchandle->indextable + (4 * nseq));
  p_transfer_data  = (dtc_transfer_data_t *) (*ptr);

  ret = rx65n_dtc_validate_dynamic_params(p_dtransfer_cfg, p_transfer_data);
  if (DTC_SUCCESS != ret)
    {
      leave_critical_section(flags);
      return ret;
    }

  if (0 == count)
    {
      /* Set the cpu interrupt to the sequence number. */

      *ptr = DTC_INVALID_CMND;
    }

  else
    {
      /* Update transfer data with address and counter */

      for (i = 0; i < count; i++)
        {
          if (rx65n_dtc_set_dynamic_transfer_data(p_dtransfer_cfg,
                                  p_transfer_data) != DTC_SUCCESS)
            {
              leave_critical_section(flags);
              return DTC_ERR_INVALID_ARG;
            }

          p_dtransfer_cfg++;
          p_transfer_data++;
          count--;
        }
    }

  /* Restore RRS bit */

  DTC.DTCCR.BIT.RRS = rrs_backup;

  /* Restore the DTCE bit. */

  ICU.DTCER[act_source].BIT.DTCE = dtce_backup;

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: rx65n_dtc_setup_static_transferdata
 *
 *
 * Description:
 *   Configure and Creates the transfer data for a activation source.
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    scfg -Static config parameter
 *    pdt -pointer to data transfer
 *    nseq_transfer - Number of sequence transfer
 *    nseq -Sequence number
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_seq_static_transferdata(DTC_HANDLE handle,
                            uint8_t src, uint32_t scfg, uint32_t pdt,
                                 uint32_t nseq_transfer, uint8_t nseq)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  uint32_t *ptr = NULL;
  int i;

  /* Transfer Information count */

  uint32_t count = nseq_transfer;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  /* Static User Configuration */

  dtc_static_transfer_data_cfg_t *p_stransfer_cfg =
                      (dtc_static_transfer_data_cfg_t *)scfg;

  /* Allocate  memory in RAM for transfer information */

  dtc_transfer_data_t *p_transfer_data = (dtc_transfer_data_t *)pdt;

  if ((act_source < DTC_MIN_VECT_NUMBER) ||
      (act_source > DTC_MAX_VECT_NUMBER))
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_stransfer_cfg == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_transfer_data == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  /* Set Transfer information pointer in vector table */

  ptr = (uint32_t *)(dtchandle->indextable + (4 * nseq));
  *ptr = (uint32_t)p_transfer_data;

#if (DTC_IP_VER_DTCb <= DTC_IP_VER)
  /* Set the 0 value */

  p_stransfer_cfg->writeback_disable = DTC_WRITEBACK_ENABLE;
  p_stransfer_cfg->sequence_end = DTC_SEQUENCE_TRANSFER_CONTINUE;
  p_stransfer_cfg->refer_index_table_enable = DTC_REFER_INDEX_TABLE_DISABLE;
  p_stransfer_cfg->disp_add_enable = DTC_SRC_ADDR_DISP_ADD_DISABLE;
#endif

  if (0 == count)
    {
      /* Set the cpu interrupt to the sequence number. */

      *ptr = DTC_INVALID_CMND;
    }
  else
    {
      /* Set transfer Information as per request */

      for (i = 0; i < count ; i++)
        {
          if (rx65n_dtc_set_static_transfer_data(p_stransfer_cfg,
                      p_transfer_data) != DTC_SUCCESS)
            {
              return DTC_ERR_INVALID_ARG;
            }

          p_stransfer_cfg++;
          p_transfer_data++;
          count--;
        }
    }

  return DTC_SUCCESS;
}
#endif

/****************************************************************************
 * Name: rx65n_dtc_setup_dynamic_transferdata
 *
 * Description:
 *   Setup dynamic transfer info like src and destination address and number
 *   of data to transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    dcfg -Dynamic config parameter
 *    nchain - Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_dynamic_transferdata(DTC_HANDLE handle,
                                                uint8_t src,
                                      uint32_t dcfg, uint32_t nchain)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;
  dtc_err_t ret = DTC_SUCCESS;
  uint32_t *ptr         = NULL;
  dtc_transfer_data_t *p_transfer_data = NULL;
  uint8_t   rrs_backup  = 0;
  uint8_t   dtce_backup = 0;
  irqstate_t flags;
  int i;

  /* Chain transfer count */

  uint32_t count = nchain + 1;

  /* Dynamic user configuration */

  dtc_dynamic_transfer_data_cfg_t *p_dtransfer_cfg =
                           (dtc_dynamic_transfer_data_cfg_t *)dcfg;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  flags = enter_critical_section();

  /* Store old value of DTCERn.DTCE bit. */

  dtce_backup = ICU.DTCER[act_source].BIT.DTCE;

  /* Disable the interrupt source. Clear the DTCER */

  ICU.DTCER[act_source].BIT.DTCE = 0;

  /* Store RRS and clear RRS */

  rrs_backup =  DTC.DTCCR.BIT.RRS;
  DTC.DTCCR.BIT.RRS = 0;

  /* Get Transfer data pointer */

  ptr = (uint32_t *)(dtchandle->vectortable + (4 * act_source));
  p_transfer_data  = (dtc_transfer_data_t *) (*ptr);

  ret = rx65n_dtc_validate_dynamic_params(p_dtransfer_cfg, p_transfer_data);
  if (DTC_SUCCESS != ret)
    {
      leave_critical_section(flags);
      return ret;
    }

  /* Update transfer data with address and counter */

  for (i = 0; i < count; i++)
    {
      if (rx65n_dtc_set_dynamic_transfer_data(p_dtransfer_cfg,
                                  p_transfer_data) != DTC_SUCCESS)
        {
          leave_critical_section(flags);
          return DTC_ERR_INVALID_ARG;
        }

      p_dtransfer_cfg++;
      p_transfer_data++;
      count--;
    }

  /* Restore RRS bit */

  DTC.DTCCR.BIT.RRS = rrs_backup;

  /* Restore the DTCE bit. */

  ICU.DTCER[act_source].BIT.DTCE = dtce_backup;

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: rx65n_dtc_setup_static_transferdata
 *
 * Description:
 *   Configure and Creates the transfer data for a activation source.
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    scfg -Pointer to contains the settings for Transfer data
 *    pdt - pointer to data transfer
 *    nchain - Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_static_transferdata(DTC_HANDLE handle, uint8_t src,
                               uint32_t scfg, uint32_t pdt, uint32_t nchain)
{
  struct rx65n_dtc_s *dtchandle = (struct rx65n_dtc_s *)handle;

  uint32_t *ptr = NULL;

  int i;

  /* Transfer Information count */

  uint32_t count = nchain + 1;

  /* Activation source */

  dtc_activation_source_t act_source = (dtc_activation_source_t)src;

  /* Static User Configuration */

  dtc_static_transfer_data_cfg_t *p_stransfer_cfg =
                          (dtc_static_transfer_data_cfg_t *)scfg;

  /* Allocate  memory in RAM for transfer information */

  dtc_transfer_data_t *p_transfer_data = (dtc_transfer_data_t *)pdt;

  if ((act_source < DTC_MIN_VECT_NUMBER) ||
      (act_source > DTC_MAX_VECT_NUMBER))
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_stransfer_cfg == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  if (p_transfer_data == NULL)
    {
      return DTC_ERR_INVALID_ARG;
    }

  /* Set Transfer information pointer in vector table */

  ptr = (uint32_t *)(dtchandle->vectortable + (4 * act_source));
  *ptr = (uint32_t)p_transfer_data;

#if (DTC_IP_VER_DTCb <= DTC_IP_VER)
  /* Set the 0 value */

  p_stransfer_cfg->writeback_disable = DTC_WRITEBACK_ENABLE;
  p_stransfer_cfg->sequence_end = DTC_SEQUENCE_TRANSFER_CONTINUE;
  p_stransfer_cfg->refer_index_table_enable = DTC_REFER_INDEX_TABLE_DISABLE;
  p_stransfer_cfg->disp_add_enable = DTC_SRC_ADDR_DISP_ADD_DISABLE;
#endif

  /* Set transfer Information as per request */

  for (i = 0; i < count ; i++)
    {
      if (rx65n_dtc_set_static_transfer_data(p_stransfer_cfg,
                      p_transfer_data) != DTC_SUCCESS)
        {
          return DTC_ERR_INVALID_ARG;
        }

      p_stransfer_cfg++;
      p_transfer_data++;
      count--;
    }

  return DTC_SUCCESS;
}

/****************************************************************************
 * Name: rx65n_dtc_gethandle
 *
 * Description:
 *   Get DTC handle.
 *
 * Input Parameters:
 *   chan - Identifies the channel resource. For the RX65N, this
 *     is simply the channel number is 0
 *
 * Returned Value:
 *   dtchandle-Provided that 'chan' is valid, this function ALWAYS returns a
 *   non-NULL,void* DTC channel handle.  (If 'chndx' is invalid, the
 *   function will assert if debug is enabled or do something ignorant
 *    otherwise).
 *
 ****************************************************************************/

DTC_HANDLE rx65n_dtc_gethandle(unsigned int chan)
{
  struct rx65n_dtc_s *dtchandle = NULL;

  dtchandle = &g_dtchandle[chan];

  DEBUGASSERT(chan < DTC_NCHANNELS);

  return (DTC_HANDLE)dtchandle;
}

/****************************************************************************
 * Name: rx65n_dtc_initialize
 *
 * Description:
 *   Initialize the DTC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rx65n_dtc_initialize(void)
{
  struct rx65n_dtc_s *dtchandle = NULL;

  int chndx;
  uint8_t *dtctable;
  volatile uint32_t dtce_cnt = 0;

  /* Initialize each DTC channel */

  for (chndx = 0; chndx < DTC_NCHANNELS; chndx++) /* RX65N support only one channel */
    {
      dtchandle = &g_dtchandle[chndx];

      /* Get DTC Vector table */

      dtctable = dtchandle->vectortable;

      /* Clear all DTCER registers of all activation sources */

      for (dtce_cnt = 0 ; dtce_cnt < DTC_NUM_INTERRUPT_SRC; dtce_cnt++)
        {
          ICU.DTCER[g_dtcsource[dtce_cnt]].BIT.DTCE = 0;
        }

      /* Cancel module stop for DMAC and DTC */

      rx65n_dtc_module_enable();

      /* Configure DTC Vector Table Base Register */

      dtchandle->vectortable = (uint8_t *)((uint32_t)(dtctable
                                                + DTC_VECTOR_ADDRESS_ALIGN)
                              & DTC_VECTOR_ADDRESS_MASK);
      dtcchan_putreg(dtchandle, DTC_DTCVBR_OFFSET,
                            (uint32_t)dtchandle->vectortable);

#if defined(CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)
      /* Configure DTC Index Table Base Register for sequence transfer */

      dtchandle->indextable = (uint8_t *)(dtchandle->vectortable
                                            + DTC_VECTOR_TABLE_SIZE);
      dtcchan_putreg(dtchandle, DTC_DTCIBR_OFFSET,
                        (uint32_t)dtchandle->indextable);
#endif

      /* Stop DTC */

      rx65n_dtc_stop(dtchandle);

#if defined(CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)

      /* In sequence transfer mode, set full address mode */

      DTC.DTCADMOD.BIT.SHORT = 0;
      dtchandle->addmode = 0;

      /* Enable sequence transfer */

      DTC.DTCSQE.BIT.ESPSEL = 1;

      /* Clear vector number for sequence transfer */

      DTC.DTCSQE.WORD &= (~DTC_ESPSEL_BIT_MASK);

      /* Configure displacement */

      DTC.DTCDISP = CONFIG_RX65N_DTC_DISPLACEMENT;

#else

      /* Configure address space: short or full address mode */

#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE) /* Short-address mode */
      DTC.DTCADMOD.BIT.SHORT = 1;
      dtchandle->addmode = 1;
#else
      /* Full-address mode */

      DTC.DTCADMOD.BIT.SHORT = 0;
      dtchandle->addmode = 0;

      /* Configure displacement */

      DTC.DTCDISP = CONFIG_RX65N_DTC_DISPLACEMENT;
#endif

#endif

      /* Configure read skip enable/disbale */
#if defined(CONFIG_RX65N_DTC_TRANSFER_DATA_READ_SKIP) /* Read-Skip Enable*/
      rx65n_dtc_readskip_enable(dtchandle);
#else /* Read-Skip disable*/
      rx65n_dtc_readskip_disable(dtchandle);

#endif

      /* Update initializing status */

      dtchandle->initialized =  TRUE;

      /* Start DTC */

      rx65n_dtc_start(dtchandle);
    }
}
#endif /* End of CONFIG_RX65N_DTC*/
