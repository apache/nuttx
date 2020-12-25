/************************************************************************************
 * arch/risc-v/src/gap8/gap8.h
 * Peripheral registers on GAP8
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
 *           Modified from gap_sdk
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_RISC_V_SRC_GAP8_GAP8_H
#define __ARCH_RISC_V_SRC_GAP8_GAP8_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/chip/irq.h>
#include <stdint.h>

/************************************************************************************
 * Public Types
 ************************************************************************************/

#define SOC_PERI_BASE       (0x1A100000UL)     /* SOC Peripherals Base Address */
#define CORE_PERI_BASE      (0x00200000UL)     /* RISC Core Peripheral Base Address */

/* 2 basic timer */

typedef struct
{
  volatile uint32_t CFG_REG_LO;   /* Configuration Register for lower 32-bits */
  volatile uint32_t CFG_REG_HI;   /* Configuration Register for high 32-bits */
  volatile uint32_t VALUE_LO;     /* Timer Value Register for low 32-bits */
  volatile uint32_t VALUE_HI;     /* Timer Value Register for high 32-bits */
  volatile uint32_t CMP_LO;       /* Timer comparator Register for low 32-bits */
  volatile uint32_t CMP_HI;       /* Timer comparator Register for high 32-bits */
  volatile uint32_t START_LO;     /* Timer start Register for low 32-bits */
  volatile uint32_t START_HI;     /* Timer start Register for high 32-bits */
  volatile uint32_t RESET_LO;     /* Timer reset Register for low 32-bits */
  volatile uint32_t RESET_HI;     /* Timer reset Register for high 32-bits */
} basic_tim_reg_t;

#define BASIC_TIM   ((basic_tim_reg_t*)(CORE_PERI_BASE + 0x0400UL))

#define BASIC_TIM_CASC_ENABLE   (1L << 31)
#define BASIC_TIM_CASC_DISABLE  (0L << 31)
#define BASIC_TIM_CLKSRC_FLL    (0L << 7)
#define BASIC_TIM_CLKSRC_32K    (1L << 7)
#define BASIC_TIM_PRESC_ENABLE  (1L << 6)
#define BASIC_TIM_PRESC_DISABLE (0L << 6)
#define BASIC_TIM_ONE_SHOT      (1L << 5)
#define BASIC_TIM_MODE_CONT     (0L << 4)
#define BASIC_TIM_MODE_CYCL     (1L << 4)
#define BASIC_TIM_IRQ_ENABLE    (1L << 2)
#define BASIC_TIM_IRQ_DISABLE   (0L << 2)
#define BASIC_TIM_RESET         (1L << 1)
#define BASIC_TIM_ENABLE        (1L << 0)

typedef struct
{
  volatile uint32_t ICACHE_ENABLE;            /* Cluster Icache Enable Register  */
  volatile uint32_t ICACHE_FLUSH;             /* Cluster Icache Flush Register */
  volatile uint32_t ICACHE_LX_SEL_FLUSH;      /* Cluster Icache Level-X Flush Register or FC Flush Selected Address Register */
  volatile uint32_t ICACHE_SEL_FLUSH_STATUS;  /* Cluster Icache Flush Selected Address Register or FC ICACHE status */
  volatile uint32_t ICACHE_IS_PRI;            /* Cluster Icache is private Icache */
} scbc_reg_t;

#define CORE_SCBC_BASE      (CORE_PERI_BASE +  0x1400UL)   /* RISC Core System Control Block Cache Base Address */
#define SCBC                ((scbc_reg_t*)CORE_SCBC_BASE)  /* Icache SCBC configuration struct */

/* FLL_CTRL */

typedef struct
{
  volatile uint32_t SOC_FLL_STATUS;            /* Status register         */
  volatile uint32_t SOC_CONF1;                 /* Configuration1 register */
  volatile uint32_t SOC_CONF2;                 /* Configuration2 register */
  volatile uint32_t SOC_INTEGRATOR;            /* INTEGRATOR register     */
  volatile uint32_t CLUSTER_FLL_STATUS;        /* Status register         */
  volatile uint32_t CLUSTER_CONF1;             /* Configuration1 register */
  volatile uint32_t CLUSTER_CONF2;             /* Configuration2 register */
  volatile uint32_t CLUSTER_INTEGRATOR;        /* INTEGRATOR register     */
  volatile uint32_t FLL_CONVERGE;              /* Fll Converge register   */
} fll_ctrl_reg_t;

#define FLL_CTRL             ((fll_ctrl_reg_t *)SOC_PERI_BASE)

/* FLL_STATUS - FLL_CTRL status register */

#define FLL_CTRL_STATUS_MULTI_FACTOR_MASK          (0xFFFFU)
#define FLL_CTRL_STATUS_MULTI_FACTOR_SHIFT         (0U)

#define FLL_CTRL_STATUS_MULTI_FACTOR(x)            (((uint32_t)(x) /* << FLL_CTRL_STATUS_MULTI_FACTOR_SHIFT */) & FLL_CTRL_STATUS_MULTI_FACTOR_MASK)

#define READ_FLL_CTRL_STATUS_MULTI_FACTOR(x)       (((uint32_t)(x) & FLL_CTRL_STATUS_MULTI_FACTOR_MASK) /* >> FLL_CTRL_STATUS_MULTI_FACTOR_SHIFT */)

/* SOC_CONF1 - FLL_CTRL configuration 1 register */

#define FLL_CTRL_CONF1_MULTI_FACTOR_MASK           (0xFFFFU)
#define FLL_CTRL_CONF1_MULTI_FACTOR_SHIFT          (0U)

#define FLL_CTRL_CONF1_MULTI_FACTOR(x)             (((uint32_t)(x) /* << FLL_CTRL_CONF1_MULTI_FACTOR_SHIFT */) & FLL_CTRL_CONF1_MULTI_FACTOR_MASK)

#define READ_FLL_CTRL_CONF1_MULTI_FACTOR(x)        (((uint32_t)(x) & FLL_CTRL_CONF1_MULTI_FACTOR_MASK) /* >> FLL_CTRL_CONF1_MULTI_FACTOR_SHIFT */)

#define FLL_CTRL_CONF1_DCO_INPUT_MASK              (0x3FF0000U)
#define FLL_CTRL_CONF1_DCO_INPUT_SHIFT             (16U)
#define FLL_CTRL_CONF1_DCO_INPUT(x)                (((uint32_t)(x) << FLL_CTRL_CONF1_DCO_INPUT_SHIFT) & FLL_CTRL_CONF1_DCO_INPUT_MASK)
#define READ_FLL_CTRL_CONF1_DCO_INPUT(x)           (((uint32_t)(x) & FLL_CTRL_CONF1_DCO_INPUT_MASK) >> FLL_CTRL_CONF1_DCO_INPUT_SHIFT)

#define FLL_CTRL_CONF1_CLK_OUT_DIV_MASK            (0x3C000000U)
#define FLL_CTRL_CONF1_CLK_OUT_DIV_SHIFT           (26U)
#define FLL_CTRL_CONF1_CLK_OUT_DIV(x)              (((uint32_t)(x) << FLL_CTRL_CONF1_CLK_OUT_DIV_SHIFT) & FLL_CTRL_CONF1_CLK_OUT_DIV_MASK)
#define READ_FLL_CTRL_CONF1_CLK_OUT_DIV(x)         (((uint32_t)(x) & FLL_CTRL_CONF1_CLK_OUT_DIV_MASK) >> FLL_CTRL_CONF1_CLK_OUT_DIV_SHIFT)

#define FLL_CTRL_CONF1_OUTPUT_LOCK_EN_MASK         (0x40000000U)
#define FLL_CTRL_CONF1_OUTPUT_LOCK_EN_SHIFT        (30U)
#define FLL_CTRL_CONF1_OUTPUT_LOCK_EN(x)           (((uint32_t)(x) << FLL_CTRL_CONF1_OUTPUT_LOCK_EN_SHIFT) & FLL_CTRL_CONF1_OUTPUT_LOCK_EN_MASK)
#define READ_FLL_CTRL_CONF1_OUTPUT_LOCK_EN(x)      (((uint32_t)(x) & FLL_CTRL_CONF1_OUTPUT_LOCK_EN_MASK) >> FLL_CTRL_CONF1_OUTPUT_LOCK_EN_SHIFT)

#define FLL_CTRL_CONF1_MODE_MASK                   (0x80000000U)
#define FLL_CTRL_CONF1_MODE_SHIFT                  (31U)
#define FLL_CTRL_CONF1_MODE(x)                     (((uint32_t)(x) << FLL_CTRL_CONF1_MODE_SHIFT) & FLL_CTRL_CONF1_MODE_MASK)
#define READ_FLL_CTRL_CONF1_MODE(x)                (((uint32_t)(x) & FLL_CTRL_CONF1_MODE_MASK) >> FLL_CTRL_CONF1_MODE_SHIFT)

/* SOC_CONF2 - FLL_CTRL configuration 2 register */

#define FLL_CTRL_CONF2_LOOPGAIN_MASK               (0xFU)
#define FLL_CTRL_CONF2_LOOPGAIN_SHIF  T            (0U)

#define FLL_CTRL_CONF2_LOOPGAIN(x)                 (((uint32_t)(x) /* << FLL_CTRL_CONF2_LOOPGAIN_SHIFT */) & FLL_CTRL_CONF2_LOOPGAIN_MASK)

#define READ_FLL_CTRL_CONF2_LOOPGAIN(x)            (((uint32_t)(x) & FLL_CTRL_CONF2_LOOPGAIN_MASK)/* >> FLL_CTRL_CONF2_LOOPGAIN_SHIFT */)

#define FLL_CTRL_CONF2_DEASSERT_CYCLES_MASK        (0x3F0U)
#define FLL_CTRL_CONF2_DEASSERT_CYCLES_SHIFT       (4U)
#define FLL_CTRL_CONF2_DEASSERT_CYCLES(x)          (((uint32_t)(x) << FLL_CTRL_CONF2_DEASSERT_CYCLES_SHIFT) & FLL_CTRL_CONF2_DEASSERT_CYCLES_MASK)
#define READ_FLL_CTRL_CONF2_DEASSERT_CYCLES(x)     (((uint32_t)(x) & FLL_CTRL_CONF2_DEASSERT_CYCLES_MASK) >> FLL_CTRL_CONF2_DEASSERT_CYCLES_SHIFT)

#define FLL_CTRL_CONF2_ASSERT_CYCLES_MASK          (0xFC00U)
#define FLL_CTRL_CONF2_ASSERT_CYCLES_SHIFT         (10U)
#define FLL_CTRL_CONF2_ASSERT_CYCLES(x)            (((uint32_t)(x) << FLL_CTRL_CONF2_ASSERT_CYCLES_SHIFT) & FLL_CTRL_CONF2_ASSERT_CYCLES_MASK)
#define READ_FLL_CTRL_CONF2_ASSERT_CYCLES(x)       (((uint32_t)(x) & FLL_CTRL_CONF2_ASSERT_CYCLES_MASK) >> FLL_CTRL_CONF2_ASSERT_CYCLES_SHIFT)

#define FLL_CTRL_CONF2_LOCK_TOLERANCE_MASK         (0xFFF0000U)
#define FLL_CTRL_CONF2_LOCK_TOLERANCE_SHIFT        (16U)
#define FLL_CTRL_CONF2_LOCK_TOLERANCE(x)           (((uint32_t)(x) << FLL_CTRL_CONF2_LOCK_TOLERANCE_SHIFT) & FLL_CTRL_CONF2_LOCK_TOLERANCE_MASK)
#define READ_FLL_CTRL_CONF2_LOCK_TOLERANCE(x)      (((uint32_t)(x) & FLL_CTRL_CONF2_LOCK_TOLERANCE_MASK) >> FLL_CTRL_CONF2_LOCK_TOLERANCE_SHIFT)

#define FLL_CTRL_CONF2_CONF_CLK_SEL_MASK           (0x20000000U)
#define FLL_CTRL_CONF2_CONF_CLK_SEL_SHIFT          (29U)
#define FLL_CTRL_CONF2_CONF_CLK_SEL(x)             (((uint32_t)(x) << FLL_CTRL_CONF2_CONF_CLK_SEL_SHIFT) & FLL_CTRL_CONF2_CONF_CLK_SEL_MASK)
#define READ_FLL_CTRL_CONF2_CONF_CLK_SEL(x)        (((uint32_t)(x) & FLL_CTRL_CONF2_CONF_CLK_SEL_MASK) >> FLL_CTRL_CONF2_CONF_CLK_SEL_SHIFT)

#define FLL_CTRL_CONF2_OPEN_LOOP_MASK              (0x40000000U)
#define FLL_CTRL_CONF2_OPEN_LOOP_SHIFT             (30U)
#define FLL_CTRL_CONF2_OPEN_LOOP(x)                (((uint32_t)(x) << FLL_CTRL_CONF2_OPEN_LOOP_SHIFT) & FLL_CTRL_CONF2_OPEN_LOOP_MASK)
#define READ_FLL_CTRL_CONF2_OPEN_LOOP(x)           (((uint32_t)(x) & FLL_CTRL_CONF2_OPEN_LOOP_MASK) >> FLL_CTRL_CONF2_OPEN_LOOP_SHIFT)

#define FLL_CTRL_CONF2_DITHERING_MASK              (0x80000000U)
#define FLL_CTRL_CONF2_DITHERING_SHIFT             (31U)
#define FLL_CTRL_CONF2_DITHERING(x)                (((uint32_t)(x) << FLL_CTRL_CONF2_DITHERING_SHIFT) & FLL_CTRL_CONF2_DITHERING_MASK)
#define READ_FLL_CTRL_CONF2_DITHERING(x)           (((uint32_t)(x) & FLL_CTRL_CONF2_DITHERING_MASK) >> FLL_CTRL_CONF2_DITHERING_SHIFT)

/* SOC_INTEGRATOR - FLL_CTRL configuration 2 register */

#define FLL_CTRL_INTEGRATOR_FRACT_PART_MASK        (0xFFC0U)
#define FLL_CTRL_INTEGRATOR_FRACT_PART_SHIFT       (6U)
#define FLL_CTRL_INTEGRATOR_FRACT_PART(x)          (((uint32_t)(x) << FLL_CTRL_INTEGRATOR_FRACT_PART_SHIFT) & FLL_CTRL_INTEGRATOR_FRACT_PART_MASK)
#define READ_FLL_CTRL_INTEGRATOR_FRACT_PART(x)     (((uint32_t)(x) & FLL_CTRL_INTEGRATOR_FRACT_PART_MASK) >> FLL_CTRL_INTEGRATOR_FRACT_PART_SHIFT)

#define FLL_CTRL_INTEGRATOR_INT_PART_MASK          (0x3FF0000U)
#define FLL_CTRL_INTEGRATOR_INT_PART_SHIFT         (16U)
#define FLL_CTRL_INTEGRATOR_INT_PART(x)            (((uint32_t)(x) << FLL_CTRL_INTEGRATOR_INT_PART_SHIFT) & FLL_CTRL_INTEGRATOR_INT_PART_MASK)
#define READ_FLL_CTRL_INTEGRATOR_INT_PART(x)       (((uint32_t)(x) & FLL_CTRL_INTEGRATOR_INT_PART_MASK) >> FLL_CTRL_INTEGRATOR_INT_PART_SHIFT)

/* FLL_CONVERGE - FLL_CTRL configuration 2 register */

#define FLL_CTRL_SOC_FLL_CONV_MASK                 (0x1U)
#define FLL_CTRL_SOC_FLL_CONV_SHIFT                (0U)

#define FLL_CTRL_SOC_FLL_CONV(x)                   (((uint32_t)(x) /* << FLL_CTRL_SOC_FLL_CONV_SHIFT */) & FLL_CTRL_SOC_FLL_CONV_MASK)

#define READ_FLL_CTRL_SOC_FLL_CONV(x)              (((uint32_t)(x) & FLL_CTRL_SOC_FLL_CONV_MASK) /* >> FLL_CTRL_SOC_FLL_CONV_SHIFT */)

#define FLL_CTRL_CLUSTER_FLL_CONV_MASK             (0x2U)
#define FLL_CTRL_CLUSTER_FLL_CONV_SHIFT            (1U)
#define FLL_CTRL_CLUSTER_FLL_CONV(x)               (((uint32_t)(x) << FLL_CTRL_CLUSTER_FLL_CONV_SHIFT) & FLL_CTRL_CLUSTER_FLL_CONV_MASK)
#define READ_FLL_CTRL_CLUSTER_FLL_CONV(x)          (((uint32_t)(x) & FLL_CTRL_CLUSTER_FLL_CONV_MASK) >> FLL_CTRL_CLUSTER_FLL_CONV_SHIFT)

/* The number of FLL */

#define FLL_NUM        2

/* The FLL reference frequency */

#define FLL_REF_CLK    32768

/* GPIO - Register Layout Typedef */

typedef struct
{
  volatile uint32_t DIR;         /* gpio direction register */
  volatile uint32_t IN;          /* gpio in register */
  volatile uint32_t OUT;         /* gpio out register */
  volatile uint32_t INTEN;       /* gpio inten register */
  volatile uint32_t INTCFG[2];   /* gpio int configuration registers */
  volatile uint32_t INTSTATUS;   /* gpio int status register */
  volatile uint32_t EN;          /* gpio enable register */
  volatile uint32_t PADCFG[8];   /* pad configuration registers */
} gpio_reg_t;

#define GPIO_INTCFG_TYPE_MASK                     (0x3U)
#define GPIO_INTCFG_TYPE_SHIFT                    (0U)
#define GPIO_INTCFG_TYPE(x)                       (((uint32_t)(x) << GPIO_INTCFG_TYPE_SHIFT) & GPIO_INTCFG_TYPE_MASK)

#define GPIO_INTCFG_TYPE_BITS_WIDTH_MASK          (0x3U)

/* Peripheral GPIOA base pointer */

#define GPIOA                                   ((gpio_reg_t *)(SOC_PERI_BASE + 0x1000u))

/* UDMA - General Register Layout Typedef */

typedef struct
{
  volatile uint32_t RX_SADDR;       /* RX UDMA buffer transfer address register */
  volatile uint32_t RX_SIZE;        /* RX UDMA buffer transfer size register */
  volatile uint32_t RX_CFG;         /* RX UDMA transfer configuration register */
  volatile uint32_t RX_INITCFG;     /* Reserved */
  volatile uint32_t TX_SADDR;       /* TX UDMA buffer transfer address register */
  volatile uint32_t TX_SIZE;        /* TX UDMA buffer transfer size register */
  volatile uint32_t TX_CFG;         /* TX UDMA transfer configuration register */
  volatile uint32_t TX_INITCFG;     /* Reserved */
} udma_reg_t;

/* RX_SADDR - RX TX UDMA buffer transfer address register */

#define UDMA_SADDR_ADDR_MASK                 (0xFFFFU)
#define UDMA_SADDR_ADDR_SHIFT                (0U)
#define UDMA_SADDR_ADDR(x)                   (((uint32_t)(x) /* << UDMA_SADDR_ADDR_SHIFT */) & UDMA_SADDR_ADDR_MASK)

/* RX_SIZE - RX TX UDMA buffer transfer size register */

#define UDMA_SIZE_SIZE_MASK                  (0x1FFFFU)
#define UDMA_SIZE_SIZE_SHIFT                 (0U)
#define UDMA_SIZE_SIZE(x)                    (((uint32_t)(x) << UDMA_SIZE_SIZE_SHIFT) & UDMA_SIZE_SIZE_MASK)

/* RX_CFG - RX TX UDMA transfer configuration register */

#define UDMA_CFG_CONTINOUS_MASK              (0x1U)
#define UDMA_CFG_CONTINOUS_SHIFT             (0U)
#define UDMA_CFG_CONTINOUS(x)                (((uint32_t)(x) /* << UDMA_CFG_CONTINOUS_SHIFT */) & UDMA_CFG_CONTINOUS_MASK)
#define UDMA_CFG_DATA_SIZE_MASK              (0x6U)
#define UDMA_CFG_DATA_SIZE_SHIFT             (1U)
#define UDMA_CFG_DATA_SIZE(x)                (((uint32_t)(x) << UDMA_CFG_DATA_SIZE_SHIFT) & UDMA_CFG_DATA_SIZE_MASK)
#define UDMA_CFG_EN_MASK                     (0x10U)
#define UDMA_CFG_EN_SHIFT                    (4U)
#define UDMA_CFG_EN(x)                       (((uint32_t)(x) << UDMA_CFG_EN_SHIFT) & UDMA_CFG_EN_MASK)
#define UDMA_CFG_CLR_MASK                    (0x20U)
#define UDMA_CFG_CLR_SHIFT                   (5U)
#define UDMA_CFG_CLR(x)                      (((uint32_t)(x) << UDMA_CFG_CLR_SHIFT) & UDMA_CFG_CLR_MASK)

/* Peripheral UDMA base address 0x1A102000 */

#define UDMA_BASE                            (0x1A102000)

/* UDMA Global configuration - Register Layout Typedef */

typedef struct
{
  volatile uint32_t CG;          /* clock gating register */
  volatile uint32_t EVTIN;       /* input event register */
} udma_gc_reg_t;

#define UDMA_GC_BASE                    (UDMA_BASE + 0x780u)
#define UDMA_GC                         ((udma_gc_reg_t *)UDMA_GC_BASE)

/* UDMA_GC - UDMA event in register, User chooses which events can come to
 *           UDMA as reference events, support up to 4 choices
 */

#define UDMA_GC_EVTIN_CHOICE0_MASK      (0xFFU)
#define UDMA_GC_EVTIN_CHOICE0_SHIFT     (0U)
#define UDMA_GC_EVTIN_CHOICE0(x)        (((uint32_t)(x) << UDMA_GC_EVTIN_CHOICE0_SHIFT) & UDMA_GC_EVTIN_CHOICE0_MASK)

#define UDMA_GC_EVTIN_CHOICE1_MASK      (0xFF00U)
#define UDMA_GC_EVTIN_CHOICE1_SHIFT     (8U)
#define UDMA_GC_EVTIN_CHOICE1(x)        (((uint32_t)(x) << UDMA_GC_EVTIN_CHOICE1_SHIFT) & UDMA_GC_EVTIN_CHOICE1_MASK)

#define UDMA_GC_EVTIN_CHOICE2_MASK      (0xFF0000U)
#define UDMA_GC_EVTIN_CHOICE2_SHIFT     (16U)
#define UDMA_GC_EVTIN_CHOICE2(x)        (((uint32_t)(x) << UDMA_GC_EVTIN_CHOICE2_SHIFT) & UDMA_GC_EVTIN_CHOICE2_MASK)

#define UDMA_GC_EVTIN_CHOICE3_MASK      (0xFF000000)
#define UDMA_GC_EVTIN_CHOICE3_SHIFT     (24U)
#define UDMA_GC_EVTIN_CHOICE3(x)        (((uint32_t)(x) << UDMA_GC_EVTIN_CHOICE3_SHIFT) & UDMA_GC_EVTIN_CHOICE3_MASK)

/* LVDS - Register Layout Typedef */

typedef struct
{
  udma_reg_t    UDMA_LVDS;           /* UDMA general register */
  volatile  uint32_t RF_CFG;         /* configuration register */
  volatile  uint32_t RF_GPIO;        /* Reserved */
  volatile  uint32_t RF_STATUS;      /* Status register */
} lvds_reg_t;

#define LVDS_BASE                                 (UDMA_BASE + 0 * 128U)
#define LVDS                                      ((lvds_reg_t *)LVDS_BASE)

/* RF_CFG - LVDS configuration register */

#define LVDS_RF_CFG_TX_ENB_MASK                  (0x1U)
#define LVDS_RF_CFG_TX_ENB_SHIFT                 (0U)
#define LVDS_RF_CFG_TX_ENB(x)                    (((uint32_t)(x) << /* LVDS_RF_CFG_TX_ENB_SHIFT */) & LVDS_RF_CFG_TX_ENB_MASK)

#define LVDS_RF_CFG_TX_OEB_MASK                  (0x2U)
#define LVDS_RF_CFG_TX_OEB_SHIFT                 (1U)
#define LVDS_RF_CFG_TX_OEB(x)                    (((uint32_t)(x) << LVDS_RF_CFG_TX_OEB_SHIFT) & LVDS_RF_CFG_TX_OEB_MASK)

#define LVDS_RF_CFG_TX_MODE_MASK                 (0x4U)
#define LVDS_RF_CFG_TX_MODE_SHIFT                (2U)
#define LVDS_RF_CFG_TX_MODE(x)                   (((uint32_t)(x) << LVDS_RF_CFG_TX_MODE_SHIFT) & LVDS_RF_CFG_TX_MODE_MASK)

#define LVDS_RF_CFG_TX_VSEL_MASK                 (0x8U)
#define LVDS_RF_CFG_TX_VSEL_SHIFT                (3U)
#define LVDS_RF_CFG_TX_VSEL(x)                   (((uint32_t)(x) << LVDS_RF_CFG_TX_VSEL_SHIFT) & LVDS_RF_CFG_TX_VSEL_MASK)

#define LVDS_RF_CFG_RX_ENB_MASK                  (0x10U)
#define LVDS_RF_CFG_RX_ENB_SHIFT                 (4U)
#define LVDS_RF_CFG_RX_ENB(x)                    (((uint32_t)(x) << LVDS_RF_CFG_RX_ENB_SHIFT) & LVDS_RF_CFG_RX_ENB_MASK)

#define LVDS_RF_CFG_SD_RX_EN_MASK                (0x20U)
#define LVDS_RF_CFG_SD_RX_EN_SHIFT               (5U)
#define LVDS_RF_CFG_SD_RX_EN(x)                  (((uint32_t)(x) << LVDS_RF_CFG_SD_RX_EN_SHIFT) & LVDS_RF_CFG_SD_RX_EN_MASK)

#define LVDS_RF_CFG_SD_TX_EN_MASK                (0x40U)
#define LVDS_RF_CFG_SD_TX_EN_SHIFT               (6U)
#define LVDS_RF_CFG_SD_TX_EN(x)                  (((uint32_t)(x) << LVDS_RF_CFG_SD_TX_EN_SHIFT) & LVDS_RF_CFG_SD_TX_EN_MASK)

#define LVDS_RF_CFG_DDR_RX_EN_MASK               (0x80U)
#define LVDS_RF_CFG_DDR_RX_EN_SHIFT              (7U)
#define LVDS_RF_CFG_DDR_RX_EN(x)                 (((uint32_t)(x) << LVDS_RF_CFG_DDR_RX_EN_SHIFT) & LVDS_RF_CFG_DDR_RX_EN_MASK)

#define LVDS_RF_CFG_DDR_TX_EN_MASK               (0x100U)
#define LVDS_RF_CFG_DDR_TX_EN_SHIFT              (8U)
#define LVDS_RF_CFG_DDR_TX_EN(x)                 (((uint32_t)(x) << LVDS_RF_CFG_DDR_TX_EN_SHIFT) & LVDS_RF_CFG_DDR_TX_EN_MASK)

#define LVDS_RF_CFG_CLKSEL_MASK                  (0x200U)
#define LVDS_RF_CFG_CLKSEL_SHIFT                 (9U)
#define LVDS_RF_CFG_CLKSEL(x)                    (((uint32_t)(x) << LVDS_RF_CFG_CLKSEL_SHIFT) & LVDS_RF_CFG_CLKSEL_MASK)

#define LVDS_RF_CFG_MODE_MASK                    (0x400U)
#define LVDS_RF_CFG_MODE_SHIFT                   (10U)
#define LVDS_RF_CFG_MODE(x)                      (((uint32_t)(x) << LVDS_RF_CFG_MODE_SHIFT) & LVDS_RF_CFG_MODE_MASK)

#define LVDS_RF_CFG_MODE_VAL_MASK                (0x800U)
#define LVDS_RF_CFG_MODE_VAL_SHIFT               (11U)
#define LVDS_RF_CFG_MODE_VAL(x)                  (((uint32_t)(x) << LVDS_RF_CFG_MODE_VAL_SHIFT) & LVDS_RF_CFG_MODE_VAL_MASK)

#define LVDS_RF_CFG_MODE_RX_MASK                 (0x1000U)
#define LVDS_RF_CFG_MODE_RX_SHIFT                (12U)
#define LVDS_RF_CFG_MODE_RX(x)                   (((uint32_t)(x) << LVDS_RF_CFG_MODE_RX_SHIFT) & LVDS_RF_CFG_MODE_RX_MASK)

/* RF_STATUS - LVDS Status register */

#define LVDS_RF_STATUS_SYNC_FLAG_MASK            (0x1U)
#define LVDS_RF_STATUS_SYNC_FLAG_SHIFT           (0U)
#define LVDS_RF_STATUS_SYNC_FLAG(x)              (((uint32_t)(x) /* << LVDS_RF_STATUS_SYNC_FLAG_SHIFT */) & LVDS_RF_STATUS_SYNC_FLAG_MASK)

/* ORCA - Register Layout Typedef */

typedef struct
{
  udma_reg_t    UDMA_ORCA;           /* ORCA UDMA general register */
  volatile  uint32_t RF_CFG;         /* ORCA configuration register */
  volatile  uint32_t RF_GPIO;        /* Reserved */
  volatile  uint32_t RF_STATUS;      /* ORCA Status register */
  volatile  uint32_t PAD;            /* ORCA reserved */
  volatile  uint32_t CLKDIV_EN;      /* ORCA uDMA clock divider enable register */
  volatile  uint32_t CLKDIV_CFG;     /* ORCA uDMA clock divider configuration register */
  volatile  uint32_t CLKDIV_UPD;     /* ORCA uDMA clock divider data register */
  volatile  uint32_t ORCA_CFG;       /* ORCA configuration register */
  volatile  uint32_t CNT_EVENT;      /* ORCA Status register */
} orca_reg_t;

#define ORCA_BASE                                 (UDMA_BASE + 0 * 128U)
#define ORCA                                      ((orca_reg_t *)ORCA_BASE)

/* RF_CFG - ORCA configuration register */

#define ORCA_RF_CFG_TX_ENB_MASK                  (0x1U)
#define ORCA_RF_CFG_TX_ENB_SHIFT                 (0U)
#define ORCA_RF_CFG_TX_ENB(x)                    (((uint32_t)(x) /* << ORCA_RF_CFG_TX_ENB_SHIFT */) & ORCA_RF_CFG_TX_ENB_MASK)

#define ORCA_RF_CFG_TX_OEB_MASK                  (0x2U)
#define ORCA_RF_CFG_TX_OEB_SHIFT                 (1U)
#define ORCA_RF_CFG_TX_OEB(x)                    (((uint32_t)(x) << ORCA_RF_CFG_TX_OEB_SHIFT) & ORCA_RF_CFG_TX_OEB_MASK)

#define ORCA_RF_CFG_TX_MODE_MASK                 (0x4U)
#define ORCA_RF_CFG_TX_MODE_SHIFT                (2U)
#define ORCA_RF_CFG_TX_MODE(x)                   (((uint32_t)(x) << ORCA_RF_CFG_TX_MODE_SHIFT) & ORCA_RF_CFG_TX_MODE_MASK)

#define ORCA_RF_CFG_TX_VSEL_MASK                 (0x8U)
#define ORCA_RF_CFG_TX_VSEL_SHIFT                (3U)
#define ORCA_RF_CFG_TX_VSEL(x)                   (((uint32_t)(x) << ORCA_RF_CFG_TX_VSEL_SHIFT) & ORCA_RF_CFG_TX_VSEL_MASK)

#define ORCA_RF_CFG_RX_ENB_MASK                  (0x10U)
#define ORCA_RF_CFG_RX_ENB_SHIFT                 (4U)
#define ORCA_RF_CFG_RX_ENB(x)                    (((uint32_t)(x) << ORCA_RF_CFG_RX_ENB_SHIFT) & ORCA_RF_CFG_RX_ENB_MASK)

#define ORCA_RF_CFG_SD_RX_EN_MASK                (0x20U)
#define ORCA_RF_CFG_SD_RX_EN_SHIFT               (5U)
#define ORCA_RF_CFG_SD_RX_EN(x)                  (((uint32_t)(x) << ORCA_RF_CFG_SD_RX_EN_SHIFT) & ORCA_RF_CFG_SD_RX_EN_MASK)

#define ORCA_RF_CFG_SD_TX_EN_MASK                (0x40U)
#define ORCA_RF_CFG_SD_TX_EN_SHIFT               (6U)
#define ORCA_RF_CFG_SD_TX_EN(x)                  (((uint32_t)(x) << ORCA_RF_CFG_SD_TX_EN_SHIFT) & ORCA_RF_CFG_SD_TX_EN_MASK)

#define ORCA_RF_CFG_DDR_RX_EN_MASK               (0x80U)
#define ORCA_RF_CFG_DDR_RX_EN_SHIFT              (7U)
#define ORCA_RF_CFG_DDR_RX_EN(x)                 (((uint32_t)(x) << ORCA_RF_CFG_DDR_RX_EN_SHIFT) & ORCA_RF_CFG_DDR_RX_EN_MASK)

#define ORCA_RF_CFG_DDR_TX_EN_MASK               (0x100U)
#define ORCA_RF_CFG_DDR_TX_EN_SHIFT              (8U)
#define ORCA_RF_CFG_DDR_TX_EN(x)                 (((uint32_t)(x) << ORCA_RF_CFG_DDR_TX_EN_SHIFT) & ORCA_RF_CFG_DDR_TX_EN_MASK)

#define ORCA_RF_CFG_CLKSEL_MASK                  (0x200U)
#define ORCA_RF_CFG_CLKSEL_SHIFT                 (9U)
#define ORCA_RF_CFG_CLKSEL(x)                    (((uint32_t)(x) << ORCA_RF_CFG_CLKSEL_SHIFT) & ORCA_RF_CFG_CLKSEL_MASK)

#define ORCA_RF_CFG_MODE_MASK                    (0x400U)
#define ORCA_RF_CFG_MODE_SHIFT                   (10U)
#define ORCA_RF_CFG_MODE(x)                      (((uint32_t)(x) << ORCA_RF_CFG_MODE_SHIFT) & ORCA_RF_CFG_MODE_MASK)

#define ORCA_RF_CFG_MODE_VAL_MASK                (0x800U)
#define ORCA_RF_CFG_MODE_VAL_SHIFT               (11U)
#define ORCA_RF_CFG_MODE_VAL(x)                  (((uint32_t)(x) << ORCA_RF_CFG_MODE_VAL_SHIFT) & ORCA_RF_CFG_MODE_VAL_MASK)

#define ORCA_RF_CFG_MODE_RX_MASK                 (0x1000U)
#define ORCA_RF_CFG_MODE_RX_SHIFT                (12U)
#define ORCA_RF_CFG_MODE_RX(x)                   (((uint32_t)(x) << ORCA_RF_CFG_MODE_RX_SHIFT) & ORCA_RF_CFG_MODE_RX_MASK)

/* RF_STATUS - ORCA Status register */

#define ORCA_RF_STATUS_SYNC_FLAG_MASK            (0x1U)
#define ORCA_RF_STATUS_SYNC_FLAG_SHIFT           (0U)
#define ORCA_RF_STATUS_SYNC_FLAG(x)              (((uint32_t)(x) /* << ORCA_RF_STATUS_SYNC_FLAG_SHIFT */) & ORCA_RF_STATUS_SYNC_FLAG_MASK)

/* CLKDIV_EN - ORCA uDMA clock divider enable register */

#define ORCA_CLKDIV_EN_MASK              (0x1U)
#define ORCA_CLKDIV_EN_SHIFT             (0U)
#define ORCA_CLKDIV_EN(x)                (((uint32_t)(x) /* << ORCA_CLKDIV_EN_SHIFT */) & ORCA_CLKDIV_EN_MASK)

/* CLKDIV_CFG - ORCA uDMA clock divider configuration register */

#define ORCA_CLKDIV_CFG_MASK             (0xFFU)
#define ORCA_CLKDIV_CFG_SHIFT            (0U)
#define ORCA_CLKDIV_CFG(x)               (((uint32_t)(x) /* << ORCA_CLKDIV_CFG_SHIFT */) & ORCA_CLKDIV_CFG_MASK)

/* CLKDIV_UDP - ORCA uDMA clock divider enable register */

#define ORCA_CLKDIV_UDP_MASK             (0x1U)
#define ORCA_CLKDIV_UDP_SHIFT            (0U)
#define ORCA_CLKDIV_UDP(x)               (((uint32_t)(x) /* << ORCA_CLKDIV_UDP_SHIFT */) & ORCA_CLKDIV_UDP_MASK)

/* ORCA_CFG - ORCA configuration register */

#define ORCA_CFG_SIZE_MASK               (0xFU)
#define ORCA_CFG_SIZE_SHIFT              (0U)
#define ORCA_CFG_SIZE(x)                 (((uint32_t)(x) /* << ORCA_CFG_SIZE_SHIFT */) & ORCA_CFG_SIZE_MASK)

#define ORCA_CFG_DELAY_MASK              (0xF0U)
#define ORCA_CFG_DELAY_SHIFT             (4U)
#define ORCA_CFG_DELAY(x)                (((uint32_t)(x) << ORCA_CFG_DELAY_SHIFT) & ORCA_CFG_DELAY_MASK)

#define ORCA_CFG_EN_MASK                 (0x100U)
#define ORCA_CFG_EN_SHIFT                (8U)
#define ORCA_CFG_EN(x)                   (((uint32_t)(x) << ORCA_CFG_EN_SHIFT) & ORCA_CFG_EN_MASK)

/* SPIM - Register Layout Typedef */

typedef struct
{
  udma_reg_t    UDMA_SPIM;              /* SPIM UDMA general register */
} spim_reg_t;

#define SPIM0_BASE                                (UDMA_BASE + 1 * 128U)
#define SPIM0                                     ((spim_reg_t *)SPIM0_BASE)
#define SPIM1_BASE                                (UDMA_BASE + 2 * 128U)
#define SPIM1                                     ((spim_reg_t *)SPIM1_BASE)

/* uDMA - SPIM uDMA control CMD */

#define SPIM_CMD_CFG_ID               0
#define SPIM_CMD_SOT_ID               1
#define SPIM_CMD_SEND_CMD_ID          2
#define SPIM_CMD_SEND_ADDR_ID         3
#define SPIM_CMD_DUMMY_ID             4
#define SPIM_CMD_WAIT_ID              5
#define SPIM_CMD_TX_DATA_ID           6
#define SPIM_CMD_RX_DATA_ID           7
#define SPIM_CMD_RPT_ID               8
#define SPIM_CMD_EOT_ID               9
#define SPIM_CMD_RPT_END_ID          10
#define SPIM_CMD_RX_CHECK_ID         11
#define SPIM_CMD_FUL_ID              12

#define SPIM_CMD_MASK                 (0xF0000000U)
#define SPIM_CMD_SHIFT                (28U)
#define SPIM_CMD(x)                   (((uint32_t)(x) << SPIM_CMD_SHIFT) & SPIM_CMD_MASK)

/* SPIM_Register_Masks SPIM Register Masks */

/* CMD_CFG - SPIM configuration register */

#define SPIM_CMD_CFG_CLKDIV_MASK                  (0xFFU)
#define SPIM_CMD_CFG_CLKDIV_SHIFT                 (0U)
#define SPIM_CMD_CFG_CLKDIV(x)                    (((uint32_t)(x) /* << SPIM_CMD_CFG_CLKDIV_SHIFT */) & SPIM_CMD_CFG_CLKDIV_MASK)
#define SPIM_CMD_CFG_CPHA_MASK                    (0x100U)
#define SPIM_CMD_CFG_CPHA_SHIFT                   (8U)
#define SPIM_CMD_CFG_CPHA(x)                      (((uint32_t)(x) << SPIM_CMD_CFG_CPHA_SHIFT) & SPIM_CMD_CFG_CPHA_MASK)
#define SPIM_CMD_CFG_CPOL_MASK                    (0x200U)
#define SPIM_CMD_CFG_CPOL_SHIFT                   (9U)
#define SPIM_CMD_CFG_CPOL(x)                      (((uint32_t)(x) << SPIM_CMD_CFG_CPOL_SHIFT) & SPIM_CMD_CFG_CPOL_MASK)

/* CMD_SOT - SPIM chip select (CS) */

#define SPIM_CMD_SOT_CS_MASK                      (0x3U)
#define SPIM_CMD_SOT_CS_SHIFT                     (0U)
#define SPIM_CMD_SOT_CS(x)                        (((uint32_t)(x) << SPIM_CMD_SOT_CS_SHIFT) & SPIM_CMD_SOT_CS_MASK)

/* CMD_SEND_CMD - SPIM Transmit a command */

#define SPIM_CMD_SEND_VALUEL_MASK                (0xFFU)
#define SPIM_CMD_SEND_VALUEL_SHIFT                (0U)
#define SPIM_CMD_SEND_VALUEL(x)                   (((uint32_t)(x) /* << SPIM_CMD_SEND_VALUEL_SHIFT */) & SPIM_CMD_SEND_VALUEL_MASK)
#define SPIM_CMD_SEND_VALUEH_MASK                 (0xFF00U)
#define SPIM_CMD_SEND_VALUEH_SHIFT                (8U)
#define SPIM_CMD_SEND_VALUEH(x)                   (((uint32_t)(x) << SPIM_CMD_SEND_VALUEH_SHIFT) & SPIM_CMD_SEND_VALUEH_MASK)
#define SPIM_CMD_SEND_CMD_SIZE_MASK               (0x1F0000U)
#define SPIM_CMD_SEND_CMD_SIZE_SHIFT              (16U)
#define SPIM_CMD_SEND_CMD_SIZE(x)                 (((uint32_t)(x) << SPIM_CMD_SEND_CMD_SIZE_SHIFT) & SPIM_CMD_SEND_CMD_SIZE_MASK)
#define SPIM_CMD_SEND_QPI_MASK                    (0x8000000U)
#define SPIM_CMD_SEND_QPI_SHIFT                   (27U)
#define SPIM_CMD_SEND_QPI(x)                      (((uint32_t)(x) << SPIM_CMD_SEND_QPI_SHIFT) & SPIM_CMD_SEND_QPI_MASK)

/* CMD_SEND_ADDR - SPIM Transmit a address */

#define SPIM_CMD_SEND_ADDR_VALUE_MASK             (0xFFFFU)
#define SPIM_CMD_SEND_ADDR_VALUE_SHIFT            (0U)
#define SPIM_CMD_SEND_ADDR_VALUE(x)               (((uint32_t)(x) /* << SPIM_CMD_SEND_ADDR_VALUE_SHIFT */) & SPIM_CMD_SEND_ADDR_VALUE_MASK)
#define SPIM_CMD_SEND_ADDR_CMD_SIZE_MASK          (0x1F0000U)
#define SPIM_CMD_SEND_ADDR_CMD_SIZE_SHIFT         (16U)
#define SPIM_CMD_SEND_ADDR_CMD_SIZE(x)            (((uint32_t)(x) << SPIM_CMD_SEND_ADDR_CMD_SIZE_SHIFT) & SPIM_CMD_SEND_ADDR_CMD_SIZE_MASK)
#define SPIM_CMD_SEND_ADDR_QPI_MASK               (0x8000000U)
#define SPIM_CMD_SEND_ADDR_QPI_SHIFT              (27U)
#define SPIM_CMD_SEND_ADDR_QPI(x)                 (((uint32_t)(x) << SPIM_CMD_SEND_ADDR_QPI_SHIFT) & SPIM_CMD_SEND_ADDR_QPI_MASK)

/* CMD_DUMMY - SPIM number of dummy cycle */

#define SPIM_CMD_DUMMY_CYCLE_MASK                 (0x1F0000U)
#define SPIM_CMD_DUMMY_CYCLE_SHIFT                (16U)
#define SPIM_CMD_DUMMY_CYCLE(x)                   (((uint32_t)(x) << SPIM_CMD_DUMMY_CYCLE_SHIFT) & SPIM_CMD_DUMMY_CYCLE_MASK)

/* CMD_WAIT - SPIM wait in which event - 2 bits */

#define SPIM_CMD_WAIT_EVENT_ID_MASK               (0x3U)
#define SPIM_CMD_WAIT_EVENT_ID_SHIFT              (0U)
#define SPIM_CMD_WAIT_EVENT_ID(x)                 (((uint32_t)(x) /* << SPIM_CMD_WAIT_EVENT_ID_SHIFT */) & SPIM_CMD_WAIT_EVENT_ID_MASK)

/* CMD_TX_DATA - SPIM send data */

#define SPIM_CMD_TX_DATA_SIZE_MASK                (0xFFFFU)
#define SPIM_CMD_TX_DATA_SIZE_SHIFT               (0U)
#define SPIM_CMD_TX_DATA_SIZE(x)                  (((uint32_t)(x) /* << SPIM_CMD_TX_DATA_SIZE_SHIFT */) & SPIM_CMD_TX_DATA_SIZE_MASK)
#define SPIM_CMD_TX_DATA_BYTE_ALIGN_MASK          (0x4000000U)
#define SPIM_CMD_TX_DATA_BYTE_ALIGN_SHIFT         (26U)
#define SPIM_CMD_TX_DATA_BYTE_ALIGN(x)            (((uint32_t)(x) << SPIM_CMD_TX_DATA_BYTE_ALIGN_SHIFT) & SPIM_CMD_TX_DATA_BYTE_ALIGN_MASK)
#define SPIM_CMD_TX_DATA_QPI_MASK                 (0x8000000U)
#define SPIM_CMD_TX_DATA_QPI_SHIFT                (27U)
#define SPIM_CMD_TX_DATA_QPI(x)                   (((uint32_t)(x) << SPIM_CMD_TX_DATA_QPI_SHIFT) & SPIM_CMD_TX_DATA_QPI_MASK)

/* CMD_RX_DATA - SPIM receive data */

#define SPIM_CMD_RX_DATA_SIZE_MASK                (0xFFFFU)
#define SPIM_CMD_RX_DATA_SIZE_SHIFT               (0U)
#define SPIM_CMD_RX_DATA_SIZE(x)                  (((uint32_t)(x) /* << SPIM_CMD_RX_DATA_SIZE_SHIFT */) & SPIM_CMD_RX_DATA_SIZE_MASK)
#define SPIM_CMD_RX_DATA_BYTE_ALIGN_MASK          (0x4000000U)
#define SPIM_CMD_RX_DATA_BYTE_ALIGN_SHIFT         (26U)
#define SPIM_CMD_RX_DATA_BYTE_ALIGN(x)            (((uint32_t)(x) << SPIM_CMD_RX_DATA_BYTE_ALIGN_SHIFT) & SPIM_CMD_RX_DATA_BYTE_ALIGN_MASK)
#define SPIM_CMD_RX_DATA_QPI_MASK                 (0x8000000U)
#define SPIM_CMD_RX_DATA_QPI_SHIFT                (27U)
#define SPIM_CMD_RX_DATA_QPI(x)                   (((uint32_t)(x) << SPIM_CMD_RX_DATA_QPI_SHIFT) & SPIM_CMD_RX_DATA_QPI_MASK)

/* CMD_RPT - SPIM repeat the next transfer N times */

#define SPIM_CMD_RPT_CNT_MASK                     (0xFFFFU)
#define SPIM_CMD_RPT_CNT_SHIFT                    (0U)
#define SPIM_CMD_RPT_CNT(x)                       (((uint32_t)(x) /* << SPIM_CMD_RPT_CNT_SHIFT */) & SPIM_CMD_RPT_CNT_MASK)

/* CMD_EOT - SPIM clears the chip select (CS), and send a end event or not  */

#define SPIM_CMD_EOT_EVENT_GEN_MASK                (0x1U)
#define SPIM_CMD_EOT_EVENT_GEN_SHIFT               (0U)
#define SPIM_CMD_EOT_EVENT_GEN(x)                  (((uint32_t)(x) /* << SPIM_CMD_EOT_EVENT_GEN_SHIFT */) & SPIM_CMD_EOT_EVENT_GEN_MASK)

/* CMD_RPT_END - SPIM End of the repeat loop */

#define SPIM_CMD_RPT_END_SPI_CMD_MASK              (0xFU)
#define SPIM_CMD_RPT_END_SPI_CMD_SHIFT             (0U)
#define SPIM_CMD_RPT_END_SPI_CMD(x)                (((uint32_t)(x) /* << SPIM_CMD_RPT_END_SPI_CMD_SHIFT */) & SPIM_CMD_RPT_END_SPI_CMD_MASK)

/* CMD_RX_CHECK - SPIM check up to 16 bits of data against an expected value  */

#define SPIM_CMD_RX_CHECK_COMP_DATA_MASK            (0xFFFFU)
#define SPIM_CMD_RX_CHECK_COMP_DATA_SHIFT           (0U)
#define SPIM_CMD_RX_CHECK_COMP_DATA(x)              (((uint32_t)(x) /* << SPIM_CMD_RX_CHECK_COMP_DATA_SHIFT */) & SPIM_CMD_RX_CHECK_COMP_DATA_MASK)

/* The number of bits to check, maximum is 16bits */

#define SPIM_CMD_RX_CHECK_STATUS_SIZE_MASK          (0xFF0000U)
#define SPIM_CMD_RX_CHECK_STATUS_SIZE_SHIFT         (16U)
#define SPIM_CMD_RX_CHECK_STATUS_SIZE(x)            (((uint32_t)(x) << SPIM_CMD_RX_CHECK_STATUS_SIZE_SHIFT) & SPIM_CMD_RX_CHECK_STATUS_SIZE_MASK)

/* The type of checking, 0 - Equal; 1 - check the bits of one;
 * 2 - check the bits of zero
 */

#define SPIM_CMD_RX_CHECK_CHECK_TYPE_MASK           (0x3000000U)
#define SPIM_CMD_RX_CHECK_CHECK_TYPE_SHIFT          (24U)
#define SPIM_CMD_RX_CHECK_CHECK_TYPE(x)             (((uint32_t)(x) << SPIM_CMD_RX_CHECK_CHECK_TYPE_SHIFT) & SPIM_CMD_RX_CHECK_CHECK_TYPE_MASK)
#define SPIM_CMD_RX_CHECK_BYTE_ALIGN_MASK           (0x4000000U)
#define SPIM_CMD_RX_CHECK_BYTE_ALIGN_SHIFT          (26U)
#define SPIM_CMD_RX_CHECK_BYTE_ALIGN(x)             (((uint32_t)(x) << SPIM_CMD_RX_CHECK_BYTE_ALIGN_SHIFT) & SPIM_CMD_RX_CHECK_BYTE_ALIGN_MASK)

/* The check bits transferred by QPI or not */

#define SPIM_CMD_RX_CHECK_QPI_MASK                  (0x8000000U)
#define SPIM_CMD_RX_CHECK_QPI_SHIFT                 (27U)
#define SPIM_CMD_RX_CHECK_QPI(x)                    (((uint32_t)(x) << SPIM_CMD_RX_CHECK_QPI_SHIFT) & SPIM_CMD_RX_CHECK_QPI_MASK)

/* CMD_FULL_DULP - SPIM Activate full duplex mode */

#define SPIM_CMD_FULL_SIZE_CMD_MASK                 (0xFFFFU)
#define SPIM_CMD_FULL_SIZE_CMD_SHIFT                (0U)
#define SPIM_CMD_FULL_SIZE_CMD(x)                   (((uint32_t)(x) /* << SPIM_CMD_FULL_SIZE_CMD_SHIFT */) & SPIM_CMD_FULL_SIZE_CMD_MASK)
#define SPIM_CMD_FULL_BYTE_ALIGN_CMD_MASK           (0x4000000U)
#define SPIM_CMD_FULL_BYTE_ALIGN_CMD_SHIFT          (26U)
#define SPIM_CMD_FULL_BYTE_ALIGN_CMD(x)             (((uint32_t)(x) << SPIM_CMD_FULL_BYTE_ALIGN_CMD_SHIFT) & SPIM_CMD_FULL_BYTE_ALIGN_CMD_MASK)

#define SPIM_CMD_CFG(clockDiv,cpol,cpha)   (SPIM_CMD(SPIM_CMD_CFG_ID) \
                                           | SPIM_CMD_CFG_CLKDIV(clockDiv) \
                                           | SPIM_CMD_CFG_CPOL(cpol) \
                                           | SPIM_CMD_CFG_CPHA(cpha))

#define SPIM_CMD_SOT(cs)                   (SPIM_CMD(SPIM_CMD_SOT_ID) \
                                           | SPIM_CMD_SOT_CS(cs))

#define SPIM_CMD_SEND_CMD(cmd,bits,qpi)    (SPIM_CMD(SPIM_CMD_SEND_CMD_ID) \
                                           | SPIM_CMD_SEND_VALUEL((cmd >> 8)) \
                                           | SPIM_CMD_SEND_VALUEH((cmd & 0xFF)) \
                                           | SPIM_CMD_SEND_CMD_SIZE(bits - 1) \
                                           | SPIM_CMD_SEND_QPI(qpi))

#define SPIM_CMD_SEND_ADDR(bits,qpi)       (SPIM_CMD(SPIM_CMD_SEND_ADDR_ID) \
                                           | SPIM_CMD_SEND_ADDR_VALUE(0) \
                                           | SPIM_CMD_SEND_ADDR_CMD_SIZE((bits - 1)) \
                                           | SPIM_CMD_SEND_ADDR_QPI(qpi))

#ifndef __PLATFORM_GVSOC__
#define SPIM_CMD_DUMMY(cycles)             (cycles ? (SPIM_CMD(SPIM_CMD_DUMMY_ID) \
                                           | SPIM_CMD_DUMMY_CYCLE(cycles - 1)) : 0xFFFF0000u)
#else
#define SPIM_CMD_DUMMY(cycles)             (cycles ? (SPIM_CMD(SPIM_CMD_DUMMY_ID) \
                                           | SPIM_CMD_DUMMY_CYCLE(cycles - 1)) : 0x0u)
#endif

#define SPIM_CMD_TX_DATA(bits,qpi,byte_align)  (SPIM_CMD(SPIM_CMD_TX_DATA_ID) \
                                           | SPIM_CMD_TX_DATA_SIZE((bits - 1)) \
                                           | SPIM_CMD_TX_DATA_BYTE_ALIGN(byte_align) \
                                           | SPIM_CMD_TX_DATA_QPI(qpi))

#define SPIM_CMD_RX_DATA(bits,qpi,byte_align)  (SPIM_CMD(SPIM_CMD_RX_DATA_ID) \
                                           | SPIM_CMD_RX_DATA_SIZE((bits - 1)) \
                                           | SPIM_CMD_RX_DATA_BYTE_ALIGN(byte_align) \
                                           | SPIM_CMD_RX_DATA_QPI(qpi))

#define SPIM_CMD_RPT(iter)                 (SPIM_CMD(SPIM_CMD_RPT_ID) \
                                           | SPIM_CMD_RPT_CNT(iter))

#define SPIM_CMD_EOT(evt)                  (SPIM_CMD(SPIM_CMD_EOT_ID) \
                                           | SPIM_CMD_EOT_EVENT_GEN(evt))

#define SPIM_CMD_WAIT(event)               (SPIM_CMD(SPIM_CMD_WAIT_ID) \
                                           | SPIM_CMD_WAIT_EVENT_ID(event))

#define SPIM_CMD_RPT_END()                 (SPIM_CMD(SPIM_CMD_RPT_END_ID))

#define SPIM_CMD_FUL(bits,byte_align)      (SPIM_CMD(SPIM_CMD_RPT_END_ID) \
                                           | SPIM_CMD_FULL_SIZE_CMD((bits - 1)) \
                                           | SPIM_CMD_FULL_BYTE_ALIGN_CMD(byte_align))

#define SPIM_CMD_RX_CHECK(mode,bits,value,qpi,byte_align) \
                                           (SPIM_CMD(SPIM_CMD_RX_CHECK_ID) \
                                           | SPIM_CMD_RX_CHECK_COMP_DATA(value) \
                                           | SPIM_CMD_RX_CHECK_STATUS_SIZE((bits - 1)) \
                                           | SPIM_CMD_RX_CHECK_CHECK_TYPE(mode) \
                                           | SPIM_CMD_RX_CHECK_BYTE_ALIGN(byte_align) \
                                           | SPIM_CMD_RX_CHECK_QPI(qpi))

/* HYPERBUS - Register Layout Typedef */

typedef struct
{
  udma_reg_t    UDMA_HYPERBUS;        /* UDMA general register */
  volatile  uint32_t EXT_ADDR;        /* Memory access address register */
  volatile  uint32_t EXT_CFG;         /* Reserved */
  volatile  uint32_t MEM_CFG0;        /* Memory control Configuration register0 */
  volatile  uint32_t MEM_CFG1;        /* Memory control Configuration register1 */
  volatile  uint32_t MEM_CFG2;        /* Memory control Configuration register2 */
  volatile  uint32_t MEM_CFG3;        /* Memory control Configuration register3 */
  volatile  uint32_t MEM_CFG4;        /* Memory control Configuration register4 */
  volatile  uint32_t MEM_CFG5;        /* Memory control Configuration register5 */
  volatile  uint32_t MEM_CFG6;        /* Memory control Configuration register6 */
  volatile  uint32_t MEM_CFG7;        /* Memory control Configuration register7 */
} hyperbus_reg_t;

#define HYPERBUS_BASE0                                 (UDMA_BASE + 3 * 128U)
#define HYPERBUS0                                      ((hyperbus_reg_t *)HYPERBUS_BASE0)

/* MEM_CFG0 - HYPERBUS Memory control Configuration register0 */

#define HYPERBUS_MEM_CFG0_MBR0_MASK                    (0xFFU)
#define HYPERBUS_MEM_CFG0_MBR0_SHIFT                   (0U)
#define HYPERBUS_MEM_CFG0_MBR0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG0_MBR0_SHIFT) & HYPERBUS_MEM_CFG0_MBR0_MASK)
#define HYPERBUS_MEM_CFG0_LATENCY0_MASK                (0xF00U)
#define HYPERBUS_MEM_CFG0_LATENCY0_SHIFT               (8U)
#define HYPERBUS_MEM_CFG0_LATENCY0(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG0_LATENCY0_SHIFT) & HYPERBUS_MEM_CFG0_LATENCY0_MASK)
#define HYPERBUS_MEM_CFG0_WRAP_SIZE0_MASK              (0x3000U)
#define HYPERBUS_MEM_CFG0_WRAP_SIZE0_SHIFT             (12U)
#define HYPERBUS_MEM_CFG0_WRAP_SIZE0(x)                (((uint32_t)(x) << HYPERBUS_MEM_CFG0_WRAP_SIZE0_SHIFT) & HYPERBUS_MEM_CFG0_WRAP_SIZE0_MASK)

/* MEM_CFG1 - HYPERBUS Memory control Configuration register1 */

#define HYPERBUS_MEM_CFG1_RD_CSHI0_MASK                (0xFU)
#define HYPERBUS_MEM_CFG1_RD_CSHI0_SHIFT               (0U)
#define HYPERBUS_MEM_CFG1_RD_CSHI0(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG1_RD_CSHI0_SHIFT) & HYPERBUS_MEM_CFG1_RD_CSHI0_MASK)
#define HYPERBUS_MEM_CFG1_RD_CSS0_MASK                 (0xF0U)
#define HYPERBUS_MEM_CFG1_RD_CSS0_SHIFT                (4U)
#define HYPERBUS_MEM_CFG1_RD_CSS0(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG1_RD_CSS0_SHIFT) & HYPERBUS_MEM_CFG1_RD_CSS0_MASK)
#define HYPERBUS_MEM_CFG1_RD_CSH0_MASK                 (0xF00U)
#define HYPERBUS_MEM_CFG1_RD_CSH0_SHIFT                (8U)
#define HYPERBUS_MEM_CFG1_RD_CSH0(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG1_RD_CSH0_SHIFT) & HYPERBUS_MEM_CFG1_RD_CSH0_MASK)
#define HYPERBUS_MEM_CFG1_WR_CSHI0_MASK                (0xF000U)
#define HYPERBUS_MEM_CFG1_WR_CSHI0_SHIFT               (12U)
#define HYPERBUS_MEM_CFG1_WR_CSHI0(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG1_WR_CSHI0_SHIFT) & HYPERBUS_MEM_CFG1_WR_CSHI0_MASK)
#define HYPERBUS_MEM_CFG1_WR_CSS0_MASK                 (0xF0000U)
#define HYPERBUS_MEM_CFG1_WR_CSS0_SHIFT                (16U)
#define HYPERBUS_MEM_CFG1_WR_CSS0(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG1_WR_CSS0_SHIFT) & HYPERBUS_MEM_CFG1_WR_CSS0_MASK)
#define HYPERBUS_MEM_CFG1_WR_CSH0_MASK                 (0xF00000U)
#define HYPERBUS_MEM_CFG1_WR_CSH0_SHIFT                (20U)
#define HYPERBUS_MEM_CFG1_WR_CSH0(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG1_WR_CSH0_SHIFT) & HYPERBUS_MEM_CFG1_WR_CSH0_MASK)

/* MEM_CFG2 - HYPERBUS Memory control Configuration register2 */

#define HYPERBUS_MEM_CFG2_RD_MAX_LENGTH0_MASK                    (0x1FFU)
#define HYPERBUS_MEM_CFG2_RD_MAX_LENGTH0_SHIFT                   (0U)
#define HYPERBUS_MEM_CFG2_RD_MAX_LENGTH0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG2_RD_MAX_LENGTH0_SHIFT) & HYPERBUS_MEM_CFG2_RD_MAX_LENGTH0_MASK)
#define HYPERBUS_MEM_CFG2_WR_MAX_LENGTH0_MASK                    (0x1FF0000U)
#define HYPERBUS_MEM_CFG2_WR_MAX_LENGTH0_SHIFT                   (16U)
#define HYPERBUS_MEM_CFG2_WR_MAX_LENGTH0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG2_WR_MAX_LENGTH0_SHIFT) & HYPERBUS_MEM_CFG2_WR_MAX_LENGTH0_MASK)

/* MEM_CFG3 - HYPERBUS Memory control Configuration register3 */

#define HYPERBUS_MEM_CFG3_ACS0_MASK                    (0x1U)
#define HYPERBUS_MEM_CFG3_ACS0_SHIFT                   (0U)
#define HYPERBUS_MEM_CFG3_ACS0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG3_ACS0_SHIFT) & HYPERBUS_MEM_CFG3_ACS0_MASK)
#define HYPERBUS_MEM_CFG3_TCO0_MASK                    (0x2U)
#define HYPERBUS_MEM_CFG3_TCO0_SHIFT                   (1U)
#define HYPERBUS_MEM_CFG3_TCO0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG3_TCO0_SHIFT) & HYPERBUS_MEM_CFG3_TCO0_MASK)
#define HYPERBUS_MEM_CFG3_DT0_MASK                     (0x4U)
#define HYPERBUS_MEM_CFG3_DT0_SHIFT                    (2U)
#define HYPERBUS_MEM_CFG3_DT0(x)                       (((uint32_t)(x) << HYPERBUS_MEM_CFG3_DT0_SHIFT) & HYPERBUS_MEM_CFG3_DT0_MASK)
#define HYPERBUS_MEM_CFG3_CRT0_MASK                    (0x8U)
#define HYPERBUS_MEM_CFG3_CRT0_SHIFT                   (3U)
#define HYPERBUS_MEM_CFG3_CRT0(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG3_CRT0_SHIFT) & HYPERBUS_MEM_CFG3_CRT0_MASK)
#define HYPERBUS_MEM_CFG3_RD_MAX_LEN_EN0_MASK          (0x10U)
#define HYPERBUS_MEM_CFG3_RD_MAX_LEN_EN0_SHIFT         (4U)
#define HYPERBUS_MEM_CFG3_RD_MAX_LEN_EN0(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG3_RD_MAX_LEN_EN0_SHIFT) & HYPERBUS_MEM_CFG3_RD_MAX_LEN_EN0_MASK)
#define HYPERBUS_MEM_CFG3_WR_MAX_LEN_EN0_MASK          (0x20U)
#define HYPERBUS_MEM_CFG3_WR_MAX_LEN_EN0_SHIFT         (5U)
#define HYPERBUS_MEM_CFG3_WR_MAX_LEN_EN0(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG3_WR_MAX_LEN_EN0_SHIFT) & HYPERBUS_MEM_CFG3_WR_MAX_LEN_EN0_MASK)
#define HYPERBUS_MEM_CFG3_RDS_DELAY_ADJ_MASK           (0x300U)
#define HYPERBUS_MEM_CFG3_RDS_DELAY_ADJ_SHIFT          (8U)
#define HYPERBUS_MEM_CFG3_RDS_DELAY_ADJ(x)             (((uint32_t)(x) << HYPERBUS_MEM_CFG3_RDS_DELAY_ADJ_SHIFT) & HYPERBUS_MEM_CFG3_RDS_DELAY_ADJ_MASK)

/* MEM_CFG4 - HYPERBUS Memory control Configuration register4 */

#define HYPERBUS_MEM_CFG4_MBR1_MASK                    (0xFFU)
#define HYPERBUS_MEM_CFG4_MBR1_SHIFT                   (0U)
#define HYPERBUS_MEM_CFG4_MBR1(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG4_MBR1_SHIFT) & HYPERBUS_MEM_CFG4_MBR1_MASK)
#define HYPERBUS_MEM_CFG4_LATENCY1_MASK                (0xF00U)
#define HYPERBUS_MEM_CFG4_LATENCY1_SHIFT                (8U)
#define HYPERBUS_MEM_CFG4_LATENCY1(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG4_LATENCY1_SHIFT) & HYPERBUS_MEM_CFG4_LATENCY1_MASK)
#define HYPERBUS_MEM_CFG4_WRAP_SIZE1_MASK              (0x3000U)
#define HYPERBUS_MEM_CFG4_WRAP_SIZE1_SHIFT             (12U)
#define HYPERBUS_MEM_CFG4_WRAP_SIZE1(x)                (((uint32_t)(x) << HYPERBUS_MEM_CFG4_WRAP_SIZE1_SHIFT) & HYPERBUS_MEM_CFG4_WRAP_SIZE1_MASK)

/* MEM_CFG5 - HYPERBUS Memory control Configuration register5 */

#define HYPERBUS_MEM_CFG5_RD_CSHI1_MASK                (0xFU)
#define HYPERBUS_MEM_CFG5_RD_CSHI1_SHIFT               (0U)
#define HYPERBUS_MEM_CFG5_RD_CSHI1(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG5_RD_CSHI1_SHIFT) & HYPERBUS_MEM_CFG5_RD_CSHI1_MASK)
#define HYPERBUS_MEM_CFG5_RD_CSS1_MASK                 (0xF0U)
#define HYPERBUS_MEM_CFG5_RD_CSS1_SHIFT                (4U)
#define HYPERBUS_MEM_CFG5_RD_CSS1(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG5_RD_CSS1_SHIFT) & HYPERBUS_MEM_CFG5_RD_CSS1_MASK)
#define HYPERBUS_MEM_CFG5_RD_CSH1_MASK                 (0xF00U)
#define HYPERBUS_MEM_CFG5_RD_CSH1_SHIFT                (8U)
#define HYPERBUS_MEM_CFG5_RD_CSH1(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG5_RD_CSH1_SHIFT) & HYPERBUS_MEM_CFG5_RD_CSH1_MASK)
#define HYPERBUS_MEM_CFG5_WR_CSHI1_MASK                (0xF000U)
#define HYPERBUS_MEM_CFG5_WR_CSHI1_SHIFT               (12U)
#define HYPERBUS_MEM_CFG5_WR_CSHI1(x)                  (((uint32_t)(x) << HYPERBUS_MEM_CFG5_WR_CSHI1_SHIFT) & HYPERBUS_MEM_CFG5_WR_CSHI1_MASK)
#define HYPERBUS_MEM_CFG5_WR_CSS1_MASK                 (0xF0000U)
#define HYPERBUS_MEM_CFG5_WR_CSS1_SHIFT                (16U)
#define HYPERBUS_MEM_CFG5_WR_CSS1(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG5_WR_CSS1_SHIFT) & HYPERBUS_MEM_CFG5_WR_CSS1_MASK)
#define HYPERBUS_MEM_CFG5_WR_CSH1_MASK                 (0xF00000U)
#define HYPERBUS_MEM_CFG5_WR_CSH1_SHIFT                (20U)
#define HYPERBUS_MEM_CFG5_WR_CSH1(x)                   (((uint32_t)(x) << HYPERBUS_MEM_CFG5_WR_CSH1_SHIFT) & HYPERBUS_MEM_CFG5_WR_CSH1_MASK)

/* MEM_CFG6 - HYPERBUS Memory control Configuration register6 */

#define HYPERBUS_MEM_CFG6_RD_MAX_LENGTH1_MASK          (0x1FFU)
#define HYPERBUS_MEM_CFG6_RD_MAX_LENGTH1_SHIFT         (0U)
#define HYPERBUS_MEM_CFG6_RD_MAX_LENGTH1(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG6_RD_MAX_LENGTH1_SHIFT) & HYPERBUS_MEM_CFG6_RD_MAX_LENGTH1_MASK)
#define HYPERBUS_MEM_CFG6_WR_MAX_LENGTH1_MASK          (0x1FF0000U)
#define HYPERBUS_MEM_CFG6_WR_MAX_LENGTH1_SHIFT         (16U)
#define HYPERBUS_MEM_CFG6_WR_MAX_LENGTH1(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG6_WR_MAX_LENGTH1_SHIFT) & HYPERBUS_MEM_CFG6_WR_MAX_LENGTH1_MASK)

/* MEM_CFG7 - HYPERBUS Memory control Configuration register7 */

#define HYPERBUS_MEM_CFG7_ACS1_MASK                    (0x1U)
#define HYPERBUS_MEM_CFG7_ACS1_SHIFT                   (0U)
#define HYPERBUS_MEM_CFG7_ACS1(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG7_ACS1_SHIFT) & HYPERBUS_MEM_CFG7_ACS1_MASK)
#define HYPERBUS_MEM_CFG7_TCO1_MASK                    (0x2U)
#define HYPERBUS_MEM_CFG7_TCO1_SHIFT                   (1U)
#define HYPERBUS_MEM_CFG7_TCO1(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG7_TCO1_SHIFT) & HYPERBUS_MEM_CFG7_TCO1_MASK)
#define HYPERBUS_MEM_CFG7_DT1_MASK                     (0x4U)
#define HYPERBUS_MEM_CFG7_DT1_SHIFT                    (2U)
#define HYPERBUS_MEM_CFG7_DT1(x)                       (((uint32_t)(x) << HYPERBUS_MEM_CFG7_DT1_SHIFT) & HYPERBUS_MEM_CFG7_DT1_MASK)
#define HYPERBUS_MEM_CFG7_CRT1_MASK                    (0x8U)
#define HYPERBUS_MEM_CFG7_CRT1_SHIFT                   (3U)
#define HYPERBUS_MEM_CFG7_CRT1(x)                      (((uint32_t)(x) << HYPERBUS_MEM_CFG7_CRT1_SHIFT) & HYPERBUS_MEM_CFG7_CRT1_MASK)
#define HYPERBUS_MEM_CFG7_RD_MAX_LEN_EN1_MASK          (0x10U)
#define HYPERBUS_MEM_CFG7_RD_MAX_LEN_EN1_SHIFT         (4U)
#define HYPERBUS_MEM_CFG7_RD_MAX_LEN_EN1(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG7_RD_MAX_LEN_EN1_SHIFT) & HYPERBUS_MEM_CFG7_RD_MAX_LEN_EN1_MASK)
#define HYPERBUS_MEM_CFG7_WR_MAX_LEN_EN1_MASK          (0x20U)
#define HYPERBUS_MEM_CFG7_WR_MAX_LEN_EN1_SHIFT         (5U)
#define HYPERBUS_MEM_CFG7_WR_MAX_LEN_EN1(x)            (((uint32_t)(x) << HYPERBUS_MEM_CFG7_WR_MAX_LEN_EN1_SHIFT) & HYPERBUS_MEM_CFG7_WR_MAX_LEN_EN1_MASK)

/* UART - Register Layout Typedef */

typedef struct
{
  udma_reg_t    UDMA_UART;      /* UDMA general register */
  volatile  uint32_t STATUS;    /* Status register */
  volatile  uint32_t SETUP;     /* Configuration register */
} uart_reg_t;

#define UART_BASE                                 (UDMA_BASE + 4 * 128U)
#define UART                                      ((uart_reg_t *)UART_BASE)

/* STATUS - UART Status register */

#define UART_STATUS_TX_BUSY_MASK                    (0x1U)
#define UART_STATUS_TX_BUSY_SHIFT                   (0U)
#define UART_STATUS_TX_BUSY(x)                      (((uint32_t)(x) << UART_STATUS_TX_BUSY_SHIFT) & UART_STATUS_TX_BUSY_MASK)
#define UART_STATUS_RX_BUSY_MASK                    (0x2U)
#define UART_STATUS_RX_BUSY_SHIFT                   (1U)
#define UART_STATUS_RX_BUSY(x)                      (((uint32_t)(x) << UART_STATUS_RX_BUSY_SHIFT) & UART_STATUS_RX_BUSY_MASK)
#define UART_STATUS_RX_PE_MASK                      (0x4U)
#define UART_STATUS_RX_PE_SHIFT                     (2U)
#define UART_STATUS_RX_PE(x)                        (((uint32_t)(x) << UART_STATUS_RX_PE_SHIFT) & UART_STATUS_RX_PE_MASK)

/* SETUP - UART SETUP register */

#define UART_SETUP_PARITY_ENA_MASK                  (0x1U)
#define UART_SETUP_PARITY_ENA_SHIFT                 (0U)
#define UART_SETUP_PARITY_ENA(x)                    (((uint32_t)(x) << UART_SETUP_PARITY_ENA_SHIFT) & UART_SETUP_PARITY_ENA_MASK)

#define UART_SETUP_BIT_LENGTH_MASK                  (0x6U)
#define UART_SETUP_BIT_LENGTH_SHIFT                 (1U)
#define UART_SETUP_BIT_LENGTH(x)                    (((uint32_t)(x) << UART_SETUP_BIT_LENGTH_SHIFT) & UART_SETUP_BIT_LENGTH_MASK)

#define UART_SETUP_STOP_BITS_MASK                   (0x8U)
#define UART_SETUP_STOP_BITS_SHIFT                  (3U)
#define UART_SETUP_STOP_BITS(x)                     (((uint32_t)(x) << UART_SETUP_STOP_BITS_SHIFT) & UART_SETUP_STOP_BITS_MASK)

#define UART_SETUP_TX_ENA_MASK                      (0x100U)
#define UART_SETUP_TX_ENA_SHIFT                     (8U)
#define UART_SETUP_TX_ENA(x)                        (((uint32_t)(x) << UART_SETUP_TX_ENA_SHIFT) & UART_SETUP_TX_ENA_MASK)

#define UART_SETUP_RX_ENA_MASK                      (0x200U)
#define UART_SETUP_RX_ENA_SHIFT                     (9U)
#define UART_SETUP_RX_ENA(x)                        (((uint32_t)(x) << UART_SETUP_RX_ENA_SHIFT) & UART_SETUP_RX_ENA_MASK)

#define UART_SETUP_CLKDIV_MASK                      (0xFFFF0000U)
#define UART_SETUP_CLKDIV_SHIFT                     (16U)
#define UART_SETUP_CLKDIV(x)                        (((uint32_t)(x) << UART_SETUP_CLKDIV_SHIFT) & UART_SETUP_CLKDIV_MASK)

/* I2C - Register Layout Typedef */

typedef struct
{
  udma_reg_t     UDMA_I2C;        /* UDMA general register */
  volatile  uint32_t STATUS;      /* Status register */
  volatile  uint32_t SETUP;       /* Configuration register */
} i2c_reg_t;

#define I2C0_BASE                                (UDMA_BASE + 5 * 128U)
#define I2C0                                     ((i2c_reg_t *)I2C0_BASE)
#define I2C1_BASE                                (UDMA_BASE + 6 * 128U)
#define I2C1                                     ((i2c_reg_t *)I2C1_BASE)

/* STATUS - I2C Status register */

#define I2C_STATUS_BUSY_MASK                    (0x1U)
#define I2C_STATUS_BUSY_SHIFT                   (0U)
#define I2C_STATUS_BUSY(x)                      (((uint32_t)(x) << I2C_STATUS_BUSY_SHIFT) & I2C_STATUS_BUSY_MASK)
#define I2C_STATUS_ARB_LOST_MASK                (0x2U)
#define I2C_STATUS_ARB_LOST_SHIFT               (1U)
#define I2C_STATUS_ARB_LOST(x)                  (((uint32_t)(x) << I2C_STATUS_ARB_LOST_SHIFT) & I2C_STATUS_ARB_LOST_MASK)

/* SETUP - I2C SETUP register */

#define I2C_SETUP_DO_RST_MASK                   (0x1U)
#define I2C_SETUP_DO_RST_SHIFT                  (0U)
#define I2C_SETUP_DO_RST(x)                     (((uint32_t)(x) << I2C_SETUP_DO_RST_SHIFT) & I2C_SETUP_DO_RST_MASK)

/* uDMA - I2C uDMA control CMD */

#define I2C_CMD_MASK                 (0xF0U)
#define I2C_CMD_SHIFT                (4U)

#define I2C_CMD_START                ((((uint32_t)(0x0)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x00
#define I2C_CMD_WAIT_EV              ((((uint32_t)(0x1)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x10
#define I2C_CMD_STOP                 ((((uint32_t)(0x2)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x20
#define I2C_CMD_RD_ACK               ((((uint32_t)(0x4)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x40
#define I2C_CMD_RD_NACK              ((((uint32_t)(0x6)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x60
#define I2C_CMD_WR                   ((((uint32_t)(0x8)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0x80
#define I2C_CMD_WAIT                 ((((uint32_t)(0xA)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0xA0
#define I2C_CMD_RPT                  ((((uint32_t)(0xC)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0xC0
#define I2C_CMD_CFG                  ((((uint32_t)(0xE)) << I2C_CMD_SHIFT) & I2C_CMD_MASK) // 0xE0

/* TCDM - Register Layout Typedef */

typedef struct
{
  udma_reg_t     UDMA_TCDM;        /* UDMA general register */
  volatile  uint32_t DST_ADDR;     /* destination address register */
  volatile  uint32_t SRC_ADDR;     /* source address register */
} tcdm_reg_t;

#define TCDM_BASE                                 (UDMA_BASE + 7 * 128U)
#define TCDM                                      ((tcdm_reg_t *)TCDM_BASE)

/* DST_ADDR - TCDM destination address register */

#define TCDM_DST_ADDR_MASK                    (0x1FFFFU)
#define TCDM_DST_ADDR_SHIFT                   (0U)
#define TCDM_DST_ADDR(x)                      (((uint32_t)(x) << TCDM_DST_ADDR_SHIFT) & TCDM_DST_ADDR_MASK)

/* SRC_ADDR - TCDM source address register */

#define TCDM_SRC_ADDR_MASK                    (0x1FFFFU)
#define TCDM_SRC_ADDR_SHIFT                   (0U)
#define TCDM_SRC_ADDR(x)                      (((uint32_t)(x) << TCDM_SRC_ADDR_SHIFT) & TCDM_SRC_ADDR_MASK)

/* I2S - Register Layout Typedef */

typedef struct
{
  udma_reg_t      UDMA_I2S;            /* UDMA general register */
  volatile  uint32_t EXT;              /* external clock configuration register */
  volatile  uint32_t CFG_CLKGEN0;      /* clock/WS generator 0 configuration register */
  volatile  uint32_t CFG_CLKGEN1;      /* clock/WS generator 1 configuration register */
  volatile  uint32_t CHMODE;           /* channels mode configuration register */
  volatile  uint32_t FILT_CH0;         /* channels 0 filtering configuration register */
  volatile  uint32_t FILT_CH1;         /* channels 0 filtering configuration register */
} i2s_reg_t;

#define I2S_BASE                                 (UDMA_BASE + 8 * 128U)
#define I2S                                      ((i2s_reg_t *)I2S_BASE)

/* EXT - I2S external clock configuration Register */

#define I2S_EXT_BITS_WORD_MASK                        (0x1FU)
#define I2S_EXT_BITS_WORD_SHIFT                       (0U)
#define I2S_EXT_BITS_WORD(x)                          (((uint32_t)(x) << I2S_EXT_BITS_WORD_SHIFT) & I2S_EXT_BITS_WORD_MASK)

/* CFG_CLKGEN0 - I2S clock/WS generator 0 configuration Register */

#define I2S_CFG_CLKGEN0_BITS_WORD_MASK                (0x1FU)
#define I2S_CFG_CLKGEN0_BITS_WORD_SHIFT               (0U)
#define I2S_CFG_CLKGEN0_BITS_WORD(x)                  (((uint32_t)(x) << I2S_CFG_CLKGEN0_BITS_WORD_SHIFT) & I2S_CFG_CLKGEN0_BITS_WORD_MASK)
#define I2S_CFG_CLKGEN0_CLK_EN_MASK                   (0x100U)
#define I2S_CFG_CLKGEN0_CLK_EN_SHIFT                  (8U)
#define I2S_CFG_CLKGEN0_CLK_EN(x)                     (((uint32_t)(x) << I2S_CFG_CLKGEN0_CLK_EN_SHIFT) & I2S_CFG_CLKGEN0_CLK_EN_MASK)
#define I2S_CFG_CLKGEN0_CLK_DIV_MASK                  (0xFFFF0000U)
#define I2S_CFG_CLKGEN0_CLK_DIV_SHIFT                 (16U)
#define I2S_CFG_CLKGEN0_CLK_DIV(x)                    (((uint32_t)(x) << I2S_CFG_CLKGEN0_CLK_DIV_SHIFT) & I2S_CFG_CLKGEN0_CLK_DIV_MASK)

/* CFG_CLKGEN1 - I2S clock/WS generator 1 configuration Register */

#define I2S_CFG_CLKGEN1_BITS_WORD_MASK                (0x1FU)
#define I2S_CFG_CLKGEN1_BITS_WORD_SHIFT               (0U)
#define I2S_CFG_CLKGEN1_BITS_WORD(x)                  (((uint32_t)(x) << I2S_CFG_CLKGEN1_BITS_WORD_SHIFT) & I2S_CFG_CLKGEN1_BITS_WORD_MASK)
#define I2S_CFG_CLKGEN1_CLK_EN_MASK                   (0x100U)
#define I2S_CFG_CLKGEN1_CLK_EN_SHIFT                  (8U)
#define I2S_CFG_CLKGEN1_CLK_EN(x)                     (((uint32_t)(x) << I2S_CFG_CLKGEN1_CLK_EN_SHIFT) & I2S_CFG_CLKGEN1_CLK_EN_MASK)
#define I2S_CFG_CLKGEN1_CLK_DIV_MASK                  (0xFFFF0000U)
#define I2S_CFG_CLKGEN1_CLK_DIV_SHIFT                 (16U)
#define I2S_CFG_CLKGEN1_CLK_DIV(x)                    (((uint32_t)(x) << I2S_CFG_CLKGEN1_CLK_DIV_SHIFT) & I2S_CFG_CLKGEN1_CLK_DIV_MASK)

/* CHMODE - I2S channels mode configuration Register */

#define I2S_CHMODE_CH0_SNAP_CAM_MASK                (0x1U)
#define I2S_CHMODE_CH0_SNAP_CAM_SHIFT               (0U)
#define I2S_CHMODE_CH0_SNAP_CAM(x)                  (((uint32_t)(x) << I2S_CHMODE_CH0_SNAP_CAM_SHIFT) & I2S_CHMODE_CH0_SNAP_CAM_MASK)
#define I2S_CHMODE_CH0_LSB_FIRST_MASK               (0x10U)
#define I2S_CHMODE_CH0_LSB_FIRST_SHIFT              (4U)
#define I2S_CHMODE_CH0_LSB_FIRST(x)                 (((uint32_t)(x) << I2S_CHMODE_CH0_LSB_FIRST_SHIFT) & I2S_CHMODE_CH0_LSB_FIRST_MASK)
#define I2S_CHMODE_CH0_PDM_USEFILTER_MASK           (0x100U)
#define I2S_CHMODE_CH0_PDM_USEFILTER_SHIFT          (8U)
#define I2S_CHMODE_CH0_PDM_USEFILTER(x)             (((uint32_t)(x) << I2S_CHMODE_CH0_PDM_USEFILTER_SHIFT) & I2S_CHMODE_CH0_PDM_USEFILTER_MASK)
#define I2S_CHMODE_CH0_PDM_EN_MASK                  (0x1000U)
#define I2S_CHMODE_CH0_PDM_EN_SHIFT                 (12U)
#define I2S_CHMODE_CH0_PDM_EN(x)                    (((uint32_t)(x) << I2S_CHMODE_CH0_PDM_EN_SHIFT) & I2S_CHMODE_CH0_PDM_EN_MASK)
#define I2S_CHMODE_CH0_USEDDR_MASK                  (0x10000U)
#define I2S_CHMODE_CH0_USEDDR_SHIFT                 (16U)
#define I2S_CHMODE_CH0_USEDDR(x)                    (((uint32_t)(x) << I2S_CHMODE_CH0_USEDDR_SHIFT) & I2S_CHMODE_CH0_USEDDR_MASK)
#define I2S_CHMODE_CH0_MODE_MASK                    (0x3000000U)
#define I2S_CHMODE_CH0_MODE_SHIFT                   (24U)
#define I2S_CHMODE_CH0_MODE(x)                      (((uint32_t)(x) << I2S_CHMODE_CH0_MODE_SHIFT) & I2S_CHMODE_CH0_MODE_MASK)

#define I2S_CHMODE_CH1_SNAP_CAM_MASK                (0x2U)
#define I2S_CHMODE_CH1_SNAP_CAM_SHIFT               (1U)
#define I2S_CHMODE_CH1_SNAP_CAM(x)                  (((uint32_t)(x) << I2S_CHMODE_CH1_SNAP_CAM_SHIFT) & I2S_CHMODE_CH1_SNAP_CAM_MASK)
#define I2S_CHMODE_CH1_LSB_FIRST_MASK               (0x20U)
#define I2S_CHMODE_CH1_LSB_FIRST_SHIFT              (5U)
#define I2S_CHMODE_CH1_LSB_FIRST(x)                 (((uint32_t)(x) << I2S_CHMODE_CH1_LSB_FIRST_SHIFT) & I2S_CHMODE_CH1_LSB_FIRST_MASK)
#define I2S_CHMODE_CH1_PDM_USEFILTER_MASK           (0x200U)
#define I2S_CHMODE_CH1_PDM_USEFILTER_SHIFT          (9U)
#define I2S_CHMODE_CH1_PDM_USEFILTER(x)             (((uint32_t)(x) << I2S_CHMODE_CH1_PDM_USEFILTER_SHIFT) & I2S_CHMODE_CH1_PDM_USEFILTER_MASK)
#define I2S_CHMODE_CH1_PDM_EN_MASK                  (0x2000U)
#define I2S_CHMODE_CH1_PDM_EN_SHIFT                 (13U)
#define I2S_CHMODE_CH1_PDM_EN(x)                    (((uint32_t)(x) << I2S_CHMODE_CH1_PDM_EN_SHIFT) & I2S_CHMODE_CH1_PDM_EN_MASK)
#define I2S_CHMODE_CH1_USEDDR_MASK                  (0x20000U)
#define I2S_CHMODE_CH1_USEDDR_SHIFT                 (17U)
#define I2S_CHMODE_CH1_USEDDR(x)                    (((uint32_t)(x) << I2S_CHMODE_CH1_USEDDR_SHIFT) & I2S_CHMODE_CH1_USEDDR_MASK)
#define I2S_CHMODE_CH1_MODE_MASK                    (0xC000000U)
#define I2S_CHMODE_CH1_MODE_SHIFT                   (26U)
#define I2S_CHMODE_CH1_MODE(x)                      (((uint32_t)(x) << I2S_CHMODE_CH1_MODE_SHIFT) & I2S_CHMODE_CH1_MODE_MASK)

/* FILT_CH0 - I2S channels 0 filtering configuration Register */

#define I2S_FILT_CH0_DECIMATION_MASK                (0x3FFU)
#define I2S_FILT_CH0_DECIMATION_SHIFT               (0U)
#define I2S_FILT_CH0_DECIMATION(x)                  (((uint32_t)(x) << I2S_FILT_CH0_DECIMATION_SHIFT) & I2S_FILT_CH0_DECIMATION_MASK)
#define I2S_FILT_CH0_SHIFT_MASK                     (0x70000U)
#define I2S_FILT_CH0_SHIFT_SHIFT                    (16U)
#define I2S_FILT_CH0_SHIFT(x)                       (((uint32_t)(x) << I2S_FILT_CH0_SHIFT_SHIFT) & I2S_FILT_CH0_SHIFT_MASK)

/* FILT_CH1 - I2S channels 0 filtering configuration Register */

#define I2S_FILT_CH1_DECIMATION_MASK                (0x3FFU)
#define I2S_FILT_CH1_DECIMATION_SHIFT               (0U)
#define I2S_FILT_CH1_DECIMATION(x)                  (((uint32_t)(x) << I2S_FILT_CH1_DECIMATION_SHIFT) & I2S_FILT_CH1_DECIMATION_MASK)
#define I2S_FILT_CH1_SHIFT_MASK                     (0x70000U)
#define I2S_FILT_CH1_SHIFT_SHIFT                    (16U)
#define I2S_FILT_CH1_SHIFT(x)                       (((uint32_t)(x) << I2S_FILT_CH1_SHIFT_SHIFT) & I2S_FILT_CH1_SHIFT_MASK)

/* CPI - Register Layout Typedef */

typedef struct
{
  udma_reg_t     UDMA_CPI;          /* UDMA general register */
  volatile  uint32_t CFG_GLOB;      /* global configuration register */
  volatile  uint32_t CFG_LL;        /* lower left comer configuration register */
  volatile  uint32_t CFG_UR;        /* upper right comer configuration register */
  volatile  uint32_t CFG_SIZE;      /* horizontal resolution configuration register */
  volatile  uint32_t CFG_FILTER;    /* RGB coefficients configuration register */
} cpi_reg_t;

#define CPI_BASE                                 (UDMA_BASE + 9 * 128U)
#define CPI                                      ((cpi_reg_t *)CPI_BASE)

/* CFG_GLOB - CPI global configuration register */

#define CPI_CFG_GLOB_FRAMEDROP_EN_MASK             (0x1U)
#define CPI_CFG_GLOB_FRAMEDROP_EN_SHIFT            (0U)
#define CPI_CFG_GLOB_FRAMEDROP_EN(x)               (((uint32_t)(x) << CPI_CFG_GLOB_FRAMEDROP_EN_SHIFT) & CPI_CFG_GLOB_FRAMEDROP_EN_MASK)
#define CPI_CFG_GLOB_FRAMEDROP_VAL_MASK            (0x7EU)
#define CPI_CFG_GLOB_FRAMEDROP_VAL_SHIFT           (1U)
#define CPI_CFG_GLOB_FRAMEDROP_VAL(x)              (((uint32_t)(x) << CPI_CFG_GLOB_FRAMEDROP_VAL_SHIFT) & CPI_CFG_GLOB_FRAMEDROP_VAL_MASK)
#define CPI_CFG_GLOB_FRAMESLICE_EN_MASK            (0x80U)
#define CPI_CFG_GLOB_FRAMESLICE_EN_SHIFT           (7U)
#define CPI_CFG_GLOB_FRAMESLICE_EN(x)              (((uint32_t)(x) << CPI_CFG_GLOB_FRAMESLICE_EN_SHIFT) & CPI_CFG_GLOB_FRAMESLICE_EN_MASK)
#define CPI_CFG_GLOB_FORMAT_MASK                   (0x700U)
#define CPI_CFG_GLOB_FORMAT_SHIFT                  (8U)
#define CPI_CFG_GLOB_FORMAT(x)                     (((uint32_t)(x) << CPI_CFG_GLOB_FORMAT_SHIFT) & CPI_CFG_GLOB_FORMAT_MASK)
#define CPI_CFG_GLOB_SHIFT_MASK                    (0x7800U)
#define CPI_CFG_GLOB_SHIFT_SHIFT                   (11U)
#define CPI_CFG_GLOB_SHIFT(x)                      (((uint32_t)(x) << CPI_CFG_GLOB_SHIFT_SHIFT) & CPI_CFG_GLOB_SHIFT_MASK)
#define CPI_CFG_GLOB_EN_MASK                       (0x80000000U)
#define CPI_CFG_GLOB_EN_SHIFT                      (31U)
#define CPI_CFG_GLOB_EN(x)                         (((uint32_t)(x) << CPI_CFG_GLOB_EN_SHIFT) & CPI_CFG_GLOB_EN_MASK)

/* CFG_LL - CPI lower left comer configuration register */

#define CPI_CFG_LL_FRAMESLICE_LLX_MASK             (0xFFFFU)
#define CPI_CFG_LL_FRAMESLICE_LLX_SHIFT            (0U)
#define CPI_CFG_LL_FRAMESLICE_LLX(x)               (((uint32_t)(x) << CPI_CFG_LL_FRAMESLICE_LLX_SHIFT) & CPI_CFG_LL_FRAMESLICE_LLX_MASK)
#define CPI_CFG_LL_FRAMESLICE_LLY_MASK             (0xFFFF0000U)
#define CPI_CFG_LL_FRAMESLICE_LLY_SHIFT            (16U)
#define CPI_CFG_LL_FRAMESLICE_LLY(x)               (((uint32_t)(x) << CPI_CFG_LL_FRAMESLICE_LLY_SHIFT) & CPI_CFG_LL_FRAMESLICE_LLY_MASK)

/* CFG_UR - CPI upper right comer configuration register */

#define CPI_CFG_UR_FRAMESLICE_URX_MASK             (0xFFFFU)
#define CPI_CFG_UR_FRAMESLICE_URX_SHIFT            (0U)
#define CPI_CFG_UR_FRAMESLICE_URX(x)               (((uint32_t)(x) << CPI_CFG_UR_FRAMESLICE_URX_SHIFT) & CPI_CFG_UR_FRAMESLICE_URX_MASK)
#define CPI_CFG_UR_FRAMESLICE_URY_MASK             (0xFFFF0000U)
#define CPI_CFG_UR_FRAMESLICE_URY_SHIFT            (16U)
#define CPI_CFG_UR_FRAMESLICE_URY(x)               (((uint32_t)(x) << CPI_CFG_UR_FRAMESLICE_URY_SHIFT) & CPI_CFG_UR_FRAMESLICE_URY_MASK)

/* CFG_SIZE - CPI horizontal resolution configuration register */

#define CPI_CFG_SIZE_MASK                          (0xFFFF0000U)
#define CPI_CFG_SIZE_SHIFT                         (16U)
#define CPI_CFG_SIZE(x)                            (((uint32_t)(x) << CPI_CFG_SIZE_SHIFT) & CPI_CFG_SIZE_MASK)

/* CFG_FILTER - CPI RGB coefficients  configuration register */

#define CPI_CFG_FILTER_B_COEFF_MASK                (0xFFU)
#define CPI_CFG_FILTER_B_COEFF_SHIFT               (0U)
#define CPI_CFG_FILTER_B_COEFF(x)                  (((uint32_t)(x) << CPI_CFG_FILTER_B_COEFF_SHIFT) & CPI_CFG_FILTER_B_COEFF_MASK)
#define CPI_CFG_FILTER_G_COEFF_MASK                (0xFF00U)
#define CPI_CFG_FILTER_G_COEFF_SHIFT               (8U)
#define CPI_CFG_FILTER_G_COEFF(x)                  (((uint32_t)(x) << CPI_CFG_FILTER_G_COEFF_SHIFT) & CPI_CFG_FILTER_G_COEFF_MASK)
#define CPI_CFG_FILTER_R_COEFF_MASK                (0xFF0000U)
#define CPI_CFG_FILTER_R_COEFF_SHIFT               (16U)
#define CPI_CFG_FILTER_R_COEFF(x)                  (((uint32_t)(x) << CPI_CFG_FILTER_R_COEFF_SHIFT) & CPI_CFG_FILTER_R_COEFF_MASK)

/* SOC_CTRL - Registers Layout Typedef */

typedef struct
{
  volatile  uint32_t INFO;                /* SOC_CTRL INFO register */
  volatile  uint32_t _reserved0[2];       /* reserved */
  volatile  uint32_t CLUSTER_ISO;         /* SOC_CTRL Cluster Isolate register */
  volatile  uint32_t _reserved1[23];      /* reserved */
  volatile  uint32_t CLUSTER_BUSY;        /* SOC_CTRL Busy register */
  volatile  uint32_t CLUSTER_BYPASS;      /* SOC_CTRL Cluster PMU bypass register */
  volatile  uint32_t JTAG_REG;            /* SOC_CTRL Jtag register */
  volatile  uint32_t L2_SLEEP;            /* SOC_CTRL L2 memory sleep register */
  volatile  uint32_t SLEEP_CTRL;          /* SOC_CTRL Slepp control register */
  volatile  uint32_t CLKDIV0;             /* SOC_CTRL Slepp control register */
  volatile  uint32_t CLKDIV1;             /* SOC_CTRL Slepp control register */
  volatile  uint32_t CLKDIV2;             /* SOC_CTRL Slepp control register */
  volatile  uint32_t CLKDIV3;             /* SOC_CTRL Slepp control register */
  volatile  uint32_t CLKDIV4;             /* SOC_CTRL Slepp control register */
  volatile  uint32_t _reserved2[3];       /* reserved */
  volatile  uint32_t CORE_STATUS;         /* SOC_CTRL Slepp control register */
  volatile  uint32_t CORE_STATUS_EOC;     /* SOC_CTRL Slepp control register */
} soc_strl_reg_t;

#define SOC_CTRL_BASE                                (0x1A104000)
#define SOC_CTRL                                     ((soc_strl_reg_t *)SOC_CTRL_BASE)

/* CLUSTER_BYPASS - SOC_CTRL information register */

#define SOC_CTRL_CLUSTER_BYPASS_BYP_POW_MASK          (0x1U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_POW_SHIFT         (0U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_POW(x)            (((uint32_t)(x) /* << SOC_CTRL_CLUSTER_BYPASS_BYP_POW_SHIFT*/) & SOC_CTRL_CLUSTER_BYPASS_BYP_POW_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_BYP_CFG_MASK          (0x2U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_CFG_SHIFT         (1U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_CFG(x)            (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_BYP_CFG_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_BYP_CFG_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_POW_MASK              (0x8U)
#define SOC_CTRL_CLUSTER_BYPASS_POW_SHIFT             (3U)
#define SOC_CTRL_CLUSTER_BYPASS_POW(x)                (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_POW_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_POW_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_CUURENT_SET_MASK      (0x70U)
#define SOC_CTRL_CLUSTER_BYPASS_CUURENT_SET_SHIFT     (4U)
#define SOC_CTRL_CLUSTER_BYPASS_CUURENT_SET(x)        (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_CUURENT_SET_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_CUURENT_SET_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_DELAY_MASK            (0x180U)
#define SOC_CTRL_CLUSTER_BYPASS_DELAY_SHIFT           (7U)
#define SOC_CTRL_CLUSTER_BYPASS_DELAY(x)              (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_DELAY_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_DELAY_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_BYP_CLK_MASK          (0x200U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_CLK_SHIFT         (9U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_CLK(x)            (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_BYP_CLK_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_BYP_CLK_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_MASK         (0x400U)
#define SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_SHIFT        (10U)
#define SOC_CTRL_CLUSTER_BYPASS_CLK_GATE(x)           (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_MASK)
#define READ_SOC_CTRL_CLUSTER_BYPASS_CLK_GATE(x)      (((uint32_t)(x) & SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_MASK) >> SOC_CTRL_CLUSTER_BYPASS_CLK_GATE_SHIFT)

#define SOC_CTRL_CLUSTER_BYPASS_FLL_PWD_MASK          (0x800U)
#define SOC_CTRL_CLUSTER_BYPASS_FLL_PWD_SHIFT         (11U)
#define SOC_CTRL_CLUSTER_BYPASS_FLL_PWD(x)            (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_FLL_PWD_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_FLL_PWD_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_FLL_RET_MASK          (0x1000U)
#define SOC_CTRL_CLUSTER_BYPASS_FLL_RET_SHIFT         (12U)
#define SOC_CTRL_CLUSTER_BYPASS_FLL_RET(x)            (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_FLL_RET_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_FLL_RET_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_RESET_MASK            (0x2000U)
#define SOC_CTRL_CLUSTER_BYPASS_RESET_SHIFT           (13U)
#define SOC_CTRL_CLUSTER_BYPASS_RESET(x)              (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_RESET_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_RESET_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_BYP_ISO_MASK          (0x4000U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_ISO_SHIFT         (14U)
#define SOC_CTRL_CLUSTER_BYPASS_BYP_ISO(x)            (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_BYP_ISO_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_BYP_ISO_MASK)

#define SOC_CTRL_CLUSTER_BYPASS_PW_ISO_MASK           (0x8000U)
#define SOC_CTRL_CLUSTER_BYPASS_PW_ISO_SHIFT          (15U)
#define SOC_CTRL_CLUSTER_BYPASS_PW_ISO(x)             (((uint32_t)(x) << SOC_CTRL_CLUSTER_BYPASS_PW_ISO_SHIFT) & SOC_CTRL_CLUSTER_BYPASS_PW_ISO_MASK)
#define READ_SOC_CTRL_CLUSTER_BYPASS_PW_ISO(x)        (((uint32_t)(x) & SOC_CTRL_CLUSTER_BYPASS_PW_ISO_MASK) >> SOC_CTRL_CLUSTER_BYPASS_PW_ISO_SHIFT)

/* STATUS - SOC_CTRL status register */

#define SOC_CTRL_CORE_STATUS_EOC_MASK                 (0x80000000U)
#define SOC_CTRL_CORE_STATUS_EOC_SHIFT                (31U)
#define SOC_CTRL_CORE_STATUS_EOC(x)                   (((uint32_t)(x) << SOC_CTRL_CORE_STATUS_EOC_SHIFT) & SOC_CTRL_CORE_STATUS_EOC_MASK)

/* PMU - General Register Layout Typedef */

typedef struct
{
  volatile uint32_t RAR_DCDC;        /* CTRL control register */
  volatile uint32_t SLEEP_CTRL;      /* CTRL sleep control register */
  volatile uint32_t FORCE;           /* CTRL register */
} soc_ctrl_pmu_reg_t;

#define PMU_CTRL_BASE                               (SOC_CTRL_BASE + 0x0100u)
#define PMU_CTRL                                    ((soc_ctrl_pmu_reg_t *)PMU_CTRL_BASE)

/* RAR_DCDC - PMU control register */

#define PMU_CTRL_RAR_DCDC_NV_MASK         (0x1FU)
#define PMU_CTRL_RAR_DCDC_NV_SHIFT        (0U)

#define PMU_CTRL_RAR_DCDC_NV(x)           (((uint32_t)(x) /* << PMU_CTRL_RAR_DCDC_NV_SHIFT */) & PMU_CTRL_RAR_DCDC_NV_MASK)

#define READ_PMU_CTRL_RAR_DCDC_NV(x)      (((uint32_t)(x) & PMU_CTRL_RAR_DCDC_NV_MASK) /* >> PMU_CTRL_RAR_DCDC_NV_SHIFT */)

#define PMU_CTRL_RAR_DCDC_MV_MASK         (0x1F00U)
#define PMU_CTRL_RAR_DCDC_MV_SHIFT        (8U)
#define PMU_CTRL_RAR_DCDC_MV(x)           (((uint32_t)(x) << PMU_CTRL_RAR_DCDC_MV_SHIFT) & PMU_CTRL_RAR_DCDC_MV_MASK)
#define READ_PMU_CTRL_RAR_DCDC_MV(x)      (((uint32_t)(x) & PMU_CTRL_RAR_DCDC_MV_MASK) >> PMU_CTRL_RAR_DCDC_MV_SHIFT)
#define PMU_CTRL_RAR_DCDC_LV_MASK         (0x1F0000U)
#define PMU_CTRL_RAR_DCDC_LV_SHIFT        (16U)
#define PMU_CTRL_RAR_DCDC_LV(x)           (((uint32_t)(x) << PMU_CTRL_RAR_DCDC_LV_SHIFT) & PMU_CTRL_RAR_DCDC_LV_MASK)
#define READ_PMU_CTRL_RAR_DCDC_LV(x)      (((uint32_t)(x) & PMU_CTRL_RAR_DCDC_LV_MASK) >> PMU_CTRL_RAR_DCDC_LV_SHIFT)
#define PMU_CTRL_RAR_DCDC_RV_MASK         (0x1F000000U)
#define PMU_CTRL_RAR_DCDC_RV_SHIFT        (24U)
#define PMU_CTRL_RAR_DCDC_RV(x)           (((uint32_t)(x) << PMU_CTRL_RAR_DCDC_RV_SHIFT) & PMU_CTRL_RAR_DCDC_RV_MASK)
#define READ_PMU_CTRL_RAR_DCDC_RV(x)      (((uint32_t)(x) & PMU_CTRL_RAR_DCDC_RV_MASK) >> PMU_CTRL_RAR_DCDC_RV_SHIFT)

/* SLEEP_CTRL - PMU control register */

#define PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_MASK         (0xFU)
#define PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_SHIFT        (0U)

#define PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET(x)           (((uint32_t)(x) /* << PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_SHIFT */) & PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_MASK)

#define READ_PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET(x)      (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_MASK) /* >> PMU_CTRL_SLEEP_CTRL_CFG_MEM_RET_SHIFT */)

#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_MASK     (0x10U)
#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_SHIFT    (4U)
#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET(x)       (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_SHIFT) & PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET(x)  (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_MASK) >> PMU_CTRL_SLEEP_CTRL_CFG_FLL_SOC_RET_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_MASK  (0x20U)
#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_SHIFT (5U)
#define PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET(x)    (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_SHIFT) & PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET(x)   (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_MASK) >> PMU_CTRL_SLEEP_CTRL_CFG_FLL_CLUSTER_RET_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_MASK      (0x7C0U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_SHIFT     (6U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL(x)        (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_SHIFT) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL(x)   (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_MASK) >> PMU_CTRL_SLEEP_CTRL_EXT_WAKE_SEL_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_MASK     (0x1800U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_SHIFT    (11U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE(x)       (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_SHIFT) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE(x)  (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_MASK) >> PMU_CTRL_SLEEP_CTRL_EXT_WAKE_TYPE_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_MASK       (0x2000U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_SHIFT      (13U)
#define PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN(x)         (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_SHIFT) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN(x)    (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_MASK) >> PMU_CTRL_SLEEP_CTRL_EXT_WAKE_EN_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_WAKEUP_MASK            (0xC000U)
#define PMU_CTRL_SLEEP_CTRL_WAKEUP_SHIFT           (14U)
#define PMU_CTRL_SLEEP_CTRL_WAKEUP(x)              (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_WAKEUP_SHIFT) & PMU_CTRL_SLEEP_CTRL_WAKEUP_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_WAKEUP(x)         (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_WAKEUP_MASK) >> PMU_CTRL_SLEEP_CTRL_WAKEUP_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_BOOT_L2_MASK           (0x10000U)
#define PMU_CTRL_SLEEP_CTRL_BOOT_L2_SHIFT          (16U)
#define PMU_CTRL_SLEEP_CTRL_BOOT_L2(x)             (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_BOOT_L2_SHIFT) & PMU_CTRL_SLEEP_CTRL_BOOT_L2_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_BOOT_L2(x)        (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_BOOT_L2_MASK) >> PMU_CTRL_SLEEP_CTRL_BOOT_L2_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_REBOOT_MASK            (0xC0000U)
#define PMU_CTRL_SLEEP_CTRL_REBOOT_SHIFT           (18U)
#define PMU_CTRL_SLEEP_CTRL_REBOOT(x)              (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_REBOOT_SHIFT) & PMU_CTRL_SLEEP_CTRL_REBOOT_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_REBOOT(x)         (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_REBOOT_MASK) >> PMU_CTRL_SLEEP_CTRL_REBOOT_SHIFT)

#define PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_MASK      (0x100000U)
#define PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_SHIFT     (20U)
#define PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP(x)        (((uint32_t)(x) << PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_SHIFT) & PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_MASK)
#define READ_PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP(x)   (((uint32_t)(x) & PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_MASK) >> PMU_CTRL_SLEEP_CTRL_CLUSTER_WAKEUP_SHIFT)

/* FORCE - PMU control register */

#define PMU_CTRL_FORCE_MEM_RET_MASK                (0xFU)
#define PMU_CTRL_FORCE_MEM_RET_SHIFT               (0U)
#define PMU_CTRL_FORCE_MEM_RET(x)                  (((uint32_t)(x) /* << PMU_CTRL_FORCE_MEM_RET_SHIFT*/) & PMU_CTRL_FORCE_MEM_RET_MASK)

#define PMU_CTRL_FORCE_MEM_PWD_MASK                (0xF0U)
#define PMU_CTRL_FORCE_MEM_PWD_SHIFT               (4U)
#define PMU_CTRL_FORCE_MEM_PWD(x)                  (((uint32_t)(x) << PMU_CTRL_FORCE_MEM_PWD_SHIFT) & PMU_CTRL_FORCE_MEM_PWD_MASK)

#define PMU_CTRL_FORCE_FLL_CLUSTER_RET_MASK        (0x100U)
#define PMU_CTRL_FORCE_FLL_CLUSTER_RET_SHIFT       (8U)
#define PMU_CTRL_FORCE_FLL_CLUSTER_RET(x)          (((uint32_t)(x) << PMU_CTRL_FORCE_FLL_CLUSTER_RET_SHIFT) & PMU_CTRL_FORCE_FLL_CLUSTER_RET_MASK)

#define PMU_CTRL_FORCE_FLL_CLUSTER_PWD_MASK        (0x200U)
#define PMU_CTRL_FORCE_FLL_CLUSTER_PWD_SHIFT       (9U)
#define PMU_CTRL_FORCE_FLL_CLUSTER_PWD(x)          (((uint32_t)(x) << PMU_CTRL_FORCE_FLL_CLUSTER_PWD_SHIFT) & PMU_CTRL_FORCE_FLL_CLUSTER_PWD_MASK)

/* PORT - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t PADFUN[4];          /* PORT pad function register 0 */
  volatile  uint32_t SLEEP_PADCFG[4];    /* PORT sleep pad configuration register 0 */
  volatile  uint32_t PAD_SLEEP;          /* PORT pad sleep register */
  volatile  uint32_t _reserved0[7];      /* reserved */
  volatile  uint32_t PADCFG[16];         /* PORT pad configuration register 0 */
} port_reg_t;

#define PORTA_BASE                              (SOC_CTRL_BASE + 0x0140u)
#define PORTA                                   ((port_reg_t *)PORTA_BASE)

#define GPIO_NUM                                 32

#define PORT_PADCFG_PULL_EN_MASK                 (0x1U)
#define PORT_PADCFG_PULL_EN_SHIFT                (0U)
#define PORT_PADCFG_PULL_EN(x)                   (((uint32_t)(x) << PORT_PADCFG_PULL_EN_SHIFT) & PORT_PADCFG_PULL_EN_MASK)

#define PORT_PADCFG_DRIVE_STRENGTH_MASK          (0x2U)
#define PORT_PADCFG_DRIVE_STRENGTH_SHIFT         (1U)
#define PORT_PADCFG_DRIVE_STRENGTH(x)            (((uint32_t)(x) << PORT_PADCFG_DRIVE_STRENGTH_SHIFT) & PORT_PADCFG_DRIVE_STRENGTH_MASK)

#define PORT_PADFUN_MUX_MASK                     (0x3U)
#define PORT_PADFUN_MUX_SHIFT                    (0U)
#define PORT_PADFUN_MUX(x)                       (((uint32_t)(x) << PORT_PADFUN_MUX_SHIFT) & PORT_PADFUN_MUX_MASK)

/* IO_ISO - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t GPIO_ISO;       /* GPIO power domains isolation */
  volatile  uint32_t CAM_ISO;        /* Cemera power domains isolation */
  volatile  uint32_t LVDS_ISO;       /* LVDS power domains isolation */
} io_iso_reg_t;

#define IO_ISO_BASE                               (SOC_CTRL_BASE + 0x01C0u)
#define IO_ISO                                    ((io_iso_reg_t *)IO_ISO_BASE)

#define IO_ISO_GPIO_ISO_MASK                 (0x1U)
#define IO_ISO_GPIO_ISO_SHIFT                (0U)
#define IO_ISO_GPIO_ISO(x)                   (((uint32_t)(x) /* << IO_ISO_GPIO_ISO_SHIFT */) & IO_ISO_GPIO_ISO_MASK)

#define IO_ISO_CAM_ISO_MASK                 (0x1U)
#define IO_ISO_CAM_ISO_SHIFT                (0U)
#define IO_ISO_CAM_ISO(x)                   (((uint32_t)(x) /* << IO_ISO_CAM_ISO_SHIFT */) & IO_ISO_CAM_ISO_MASK)

#define IO_ISO_LVDS_ISO_MASK                 (0x1U)
#define IO_ISO_LVDS_ISO_SHIFT                (0U)
#define IO_ISO_LVDS_ISO(x)                   (((uint32_t)(x) /* << IO_ISO_LVDS_ISO_SHIFT */) & IO_ISO_LVDS_ISO_MASK)

/* PWM - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t EVENT_CFG;       /* event configuration register */
  volatile  uint32_t CH_EN;           /* channel enable register */
} pwm_ctrl_reg_t;

#define PWM_CTRL_BASE                                (SOC_PERI_BASE + 0x05100u)
#define PWM_CTRL                                     ((pwm_ctrl_reg_t *)PWM_CTRL_BASE)

/* Set Event. */

#define PWM_CTRL_EVENT_TIMER_CHAN_SET_MASK               (0xFFFFU)
#define PWM_CTRL_EVENT_TIMER_CHAN_SET_SHIFT(x)           ((uint32_t)(x))
#define PWM_CTRL_EVENT_TIMER_CHAN_SET(tim, chan, evt)    (((uint32_t)((uint32_t)tim << 2 | (uint32_t)chan) << PWM_CTRL_EVENT_TIMER_CHAN_SET_SHIFT(evt << 2)) & PWM_CTRL_EVENT_TIMER_CHAN_SET_MASK)

/* Enable Event. */

#define PWM_CTRL_EVENT_TIMER_ENA_MASK                    (0xF0000U)
#define PWM_CTRL_EVENT_TIMER_ENA_SHIFT                   (16)
#define PWM_CTRL_EVENT_TIMER_ENA(x)                      (((uint32_t)(x) << PWM_CTRL_EVENT_TIMER_ENA_SHIFT) & PWM_CTRL_EVENT_TIMER_ENA_MASK)

/* Timer enable. */

#define PWM_CTRL_CG_ENA_MASK                             (0xFU)
#define PWM_CTRL_CG_ENA_SHIFT                            (0)
#define PWM_CTRL_CG_ENA(x)                               (((uint32_t)(x) << PWM_CTRL_CG_ENA_SHIFT) & PWM_CTRL_CG_ENA_MASK)

/* ADV_TIMER - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t CMD;              /* control register */
  volatile  uint32_t CFG;              /* configuration register */
  volatile  uint32_t TH;               /* threshold register */
  volatile  uint32_t CH_TH[4];         /* Channles' threshold register */
  volatile  uint32_t CH_LUT[4];        /* Channles' LUT register */
  volatile  uint32_t COUNTER;          /* Counter register */
} pwm_reg_t;

#define PWM0_BASE                               (SOC_PERI_BASE + 0x05000u)
#define PWM0                                    ((pwm_reg_t *)PWM0_BASE)
#define PWM1_BASE                               (PWM0_BASE + 0x40u)
#define PWM1                                    ((pwm_reg_t *)PWM1_BASE)
#define PWM2_BASE                               (PWM1_BASE + 0x40u)
#define PWM2                                    ((pwm_reg_t *)PWM2_BASE)
#define PWM3_BASE                               (PWM2_BASE + 0x40u)
#define PWM3                                    ((pwm_reg_t *)PWM3_BASE)

/* Send command. */

#define PWM_CMD_MASK                            (0x1FU)
#define PWM_CMD_SHIFT                           (0)
#define PWM_CMD(x)                              (((uint32_t)(x) << PWM_CMD_SHIFT) & PWM_CMD_MASK)

/* Timer config. */

#define PWM_CONFIG_INPUT_SRC_MASK               (0xFFU)
#define PWM_CONFIG_INPUT_SRC_SHIFT              (0)
#define PWM_CONFIG_INPUT_SRC(x)                 (((uint32_t)(x) << PWM_CONFIG_INPUT_SRC_SHIFT) & PWM_CONFIG_INPUT_SRC_MASK)

#define PWM_CONFIG_INPUT_MODE_MASK              (0x700U)
#define PWM_CONFIG_INPUT_MODE_SHIFT             (8)
#define PWM_CONFIG_INPUT_MODE(x)                (((uint32_t)(x) << PWM_CONFIG_INPUT_MODE_SHIFT) & PWM_CONFIG_INPUT_MODE_MASK)

#define PWM_CONFIG_CLKSEL_MASK                  (0x800U)
#define PWM_CONFIG_CLKSEL_SHIFT                 (11)
#define PWM_CONFIG_CLKSEL(x)                    (((uint32_t)(x) << PWM_CONFIG_CLKSEL_SHIFT) & PWM_CONFIG_CLKSEL_MASK)

#define PWM_CONFIG_UPDOWNSEL_MASK               (0x1000U)
#define PWM_CONFIG_UPDOWNSEL_SHIFT              (12)
#define PWM_CONFIG_UPDOWNSEL(x)                 (((uint32_t)(x) << PWM_CONFIG_UPDOWNSEL_SHIFT) & PWM_CONFIG_UPDOWNSEL_MASK)

#define PWM_CONFIG_PRESCALE_MASK                (0xFF0000U)
#define PWM_CONFIG_PRESCALE_SHIFT               (16)
#define PWM_CONFIG_PRESCALE(x)                  (((uint32_t)(x) << PWM_CONFIG_PRESCALE_SHIFT) & PWM_CONFIG_PRESCALE_MASK)

/* Channel config. */

#define PWM_THRESHOLD_LOW_MASK                  (0xFFFFU)
#define PWM_THRESHOLD_LOW_SHIFT                 (0)
#define PWM_THRESHOLD_LOW(x)                    (((uint32_t)(x) << PWM_THRESHOLD_LOW_SHIFT) & PWM_THRESHOLD_LOW_MASK)

#define PWM_THRESHOLD_HIGH_MASK                 (0xFFFF0000U)
#define PWM_THRESHOLD_HIGH_SHIFT                (16)
#define PWM_THRESHOLD_HIGH(x)                   (((uint32_t)(x) << PWM_THRESHOLD_HIGH_SHIFT) & PWM_THRESHOLD_HIGH_MASK)

/* Channel config. */

#define PWM_CHANNEL_CONFIG_THRESHOLD_MASK       (0xFFFFU)
#define PWM_CHANNEL_CONFIG_THRESHOLD_SHIFT      (0)
#define PWM_CHANNEL_CONFIG_THRESHOLD(x)         (((uint32_t)(x) << PWM_CHANNEL_CONFIG_THRESHOLD_SHIFT) & PWM_CHANNEL_CONFIG_THRESHOLD_MASK)

#define PWM_CHANNEL_CONFIG_MODE_MASK            (0x70000U)
#define PWM_CHANNEL_CONFIG_MODE_SHIFT           (16)
#define PWM_CHANNEL_CONFIG_MODE(x)              (((uint32_t)(x) << PWM_CHANNEL_CONFIG_MODE_SHIFT) & PWM_CHANNEL_CONFIG_MODE_MASK)

/* SOCEU - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t EVENT;           /* event register */
  volatile  uint32_t FC_MASK_MSB;     /* fc mask MSB register */
  volatile  uint32_t FC_MASK_LSB;     /* fc mask LSB register */
  volatile  uint32_t CL_MASK_MSB;     /* cluster mask MSB register */
  volatile  uint32_t CL_MASK_LSB;     /* cluster mask LSB register */
  volatile  uint32_t PR_MASK_MSB;     /* propagate mask MSB register */
  volatile  uint32_t PR_MASK_LSB;     /* propagate mask LSB register */
  volatile  uint32_t ERR_MASK_MSB;    /* error mask MSB register */
  volatile  uint32_t ERR_MASK_LSB;    /* error mask LSB register */
  volatile  uint32_t TIMER_SEL_HI;    /* timer high register */
  volatile  uint32_t TIMER_SEL_LO;    /* timer low register */
} soceu_reg_t;

#define SOCEU_BASE                               (SOC_PERI_BASE + 0x06000u)
#define SOCEU                                    ((soceu_reg_t *)SOCEU_BASE)

/* The SOC events number */

#define SOC_EVENTS_NUM              0x08
#define EU_EVT_GETCLUSTERBASE(coreId)     (0x00200800u + (coreId << 6))

/* PMU - General Register Layout Typedef */

typedef struct
{
  volatile uint32_t PCTRL;           /* PMU DLC control register */
  volatile uint32_t PRDATA;          /* PMU DLC data register */
  volatile uint32_t DLC_SR;          /* PMU DLC register */
  volatile uint32_t DLC_IMR;         /* PMU DLC register */
  volatile uint32_t DLC_IFR;         /* PMU DLC register */
  volatile uint32_t DLC_IOIFR;       /* PMU DLC register */
  volatile uint32_t DLC_IDIFR;       /* PMU DLC register */
  volatile uint32_t DLC_IMCIFR;      /* PMU DLC register */
} pmu_dlc_reg_t;

#define PMU_DLC_BASE                                (SOC_PERI_BASE + 0x7000u)
#define PMU_DLC                                     ((pmu_dlc_reg_t *)PMU_DLC_BASE)

/* PCTRL - PMU DLC PICL control register */

#define PMU_DLC_PCTRL_START_MASK              (0x1U)
#define PMU_DLC_PCTRL_START_SHIFT             (0U)
#define PMU_DLC_PCTRL_START(x)                (((uint32_t)(x) /* << PMU_DLC_PCTRL_START_SHIFT */) & PMU_DLC_PCTRL_START_MASK)
#define PMU_DLC_PCTRL_PADDR_MASK              (0x7FFEU)
#define PMU_DLC_PCTRL_PADDR_SHIFT             (1U)
#define PMU_DLC_PCTRL_PADDR(x)                (((uint32_t)(x) << PMU_DLC_PCTRL_PADDR_SHIFT) & PMU_DLC_PCTRL_PADDR_MASK)
#define PMU_DLC_PCTRL_DIR_MASK                (0x8000U)
#define PMU_DLC_PCTRL_DIR_SHIFT               (15U)
#define PMU_DLC_PCTRL_DIR(x)                  (((uint32_t)(x) << PMU_DLC_PCTRL_DIR_SHIFT) & PMU_DLC_PCTRL_DIR_MASK)
#define PMU_DLC_PCTRL_PWDATA_MASK             (0xFFFF0000U)
#define PMU_DLC_PCTRL_PWDATA_SHIFT            (16U)
#define PMU_DLC_PCTRL_PWDATA(x)               (((uint32_t)(x) << PMU_DLC_PCTRL_PWDATA_SHIFT) & PMU_DLC_PCTRL_PWDATA_MASK)

/* PRDATA - PMU DLC PICL data read register */

#define PMU_DLC_PRDATA_PRDATA_MASK            (0xFFU)
#define PMU_DLC_PRDATA_PRDATA_SHIFT           (0U)
#define PMU_DLC_PRDATA_PRDATA(x)              (((uint32_t)(x) /* << PMU_DLC_PRDATA_PRDATA_SHIFT */) & PMU_DLC_PRDATA_PRDATA_MASK)

/* SR - PMU DLC DLC Status register */

#define PMU_DLC_SR_PICL_BUSY_MASK             (0x1U)
#define PMU_DLC_SR_PICL_BUSY_SHIFT            (0U)
#define PMU_DLC_SR_PICL_BUSY(x)               (((uint32_t)(x) /* << PMU_DLC_SR_PICL_BUSY_SHIFT */) & PMU_DLC_SR_PICL_BUSY_MASK)
#define PMU_DLC_SR_SCU_BUSY_MASK              (0x2U)
#define PMU_DLC_SR_SCU_BUSY_SHIFT             (1U)
#define PMU_DLC_SR_SCU_BUSY(x)                (((uint32_t)(x) << PMU_DLC_SR_SCU_BUSY_SHIFT) & PMU_DLC_SR_SCU_BUSY_MASK)

/* IMR - PMU DLC Interrupt mask register */

#define PMU_DLC_IMR_ICU_OK_MASK_MASK          (0x1U)
#define PMU_DLC_IMR_ICU_OK_MASK_SHIFT         (0U)
#define PMU_DLC_IMR_ICU_OK_MASK(x)            (((uint32_t)(x) /* << PMU_DLC_IMR_ICU_OK_MASK_SHIFT */) & PMU_DLC_IMR_ICU_OK_MASK_MASK)
#define PMU_DLC_IMR_ICU_DELAYED_MASK_MASK     (0x2U)
#define PMU_DLC_IMR_ICU_DELAYED_MASK_SHIFT    (1U)
#define PMU_DLC_IMR_ICU_DELAYED_MASK(x)       (((uint32_t)(x) << PMU_DLC_IMR_ICU_DELAYED_MASK_SHIFT) & PMU_DLC_IMR_ICU_DELAYED_MASK_MASK)
#define PMU_DLC_IMR_ICU_MODE_CHANGED_MASK_MASK     (0x4U)
#define PMU_DLC_IMR_ICU_MODE_CHANGED_MASK_SHIFT    (2U)
#define PMU_DLC_IMR_ICU_MODE_CHANGED_MASK(x)       (((uint32_t)(x) << PMU_DLC_IMR_ICU_MODE_CHANGED_MASK_SHIFT) & PMU_DLC_IMR_ICU_MODE_CHANGED_MASK_MASK)
#define PMU_DLC_IMR_PICL_OK_MASK_MASK         (0x8U)
#define PMU_DLC_IMR_PICL_OK_MASK_SHIFT        (3U)
#define PMU_DLC_IMR_PICL_OK_MASK(x)           (((uint32_t)(x) << PMU_DLC_IMR_PICL_OK_MASK_SHIFT) & PMU_DLC_IMR_PICL_OK_MASK_MASK)
#define PMU_DLC_IMR_SCU_OK_MASK_MASK          (0x10U)
#define PMU_DLC_IMR_SCU_OK_MASK_SHIFT         (4U)
#define PMU_DLC_IMR_SCU_OK_MASK(x)            (((uint32_t)(x) << PMU_DLC_IMR_SCU_OK_MASK_SHIFT) & PMU_DLC_IMR_SCU_OK_MASK_MASK)

/* IFR - PMU DLC Interrupt flag register */

#define PMU_DLC_IFR_ICU_OK_FLAG_MASK          (0x1U)
#define PMU_DLC_IFR_ICU_OK_FLAG_SHIFT         (0U)
#define PMU_DLC_IFR_ICU_OK_FLAG(x)            (((uint32_t)(x) /* << PMU_DLC_IFR_ICU_OK_FLAG_SHIFT */) & PMU_DLC_IFR_ICU_OK_FLAG_MASK)
#define PMU_DLC_IFR_ICU_DELAYED_FLAG_MASK     (0x2U)
#define PMU_DLC_IFR_ICU_DELAYED_FLAG_SHIFT    (1U)
#define PMU_DLC_IFR_ICU_DELAYED_FLAG(x)       (((uint32_t)(x) << PMU_DLC_IFR_ICU_DELAYED_FLAG_SHIFT) & PMU_DLC_IFR_ICU_DELAYED_FLAG_MASK)
#define PMU_DLC_IFR_ICU_MODE_CHANGED_FLAG_MASK     (0x4U)
#define PMU_DLC_IFR_ICU_MODE_CHANGED_FLAG_SHIFT    (2U)
#define PMU_DLC_IFR_ICU_MODE_CHANGED_FLAG(x)       (((uint32_t)(x) << PMU_DLC_IFR_ICU_MODE_CHANGED_FLAG_SHIFT) & PMU_DLC_IFR_ICU_MODE_CHANGED_FLAG_MASK)
#define PMU_DLC_IFR_PICL_OK_FLAG_MASK         (0x8U)
#define PMU_DLC_IFR_PICL_OK_FLAG_SHIFT        (3U)
#define PMU_DLC_IFR_PICL_OK_FLAG(x)           (((uint32_t)(x) << PMU_DLC_IFR_PICL_OK_FLAG_SHIFT) & PMU_DLC_IFR_PICL_OK_FLAG_MASK)
#define PMU_DLC_IFR_SCU_OK_FLAG_MASK          (0x10U)
#define PMU_DLC_IFR_SCU_OK_FLAG_SHIFT         (4U)
#define PMU_DLC_IFR_SCU_OK_FLAG(x)            (((uint32_t)(x) << PMU_DLC_IFR_SCU_OK_FLAG_SHIFT) & PMU_DLC_IFR_SCU_OK_FLAG_MASK)

/* IOIFR - PMU DLC icu_ok interrupt flag register */

#define PMU_DLC_IOIFR_ICU_OK_FLAG_MASK          (0xFFFFFFFEU)
#define PMU_DLC_IOIFR_ICU_OK_FLAG_SHIFT         (1U)
#define PMU_DLC_IOIFR_ICU_OK_FLAG(x)            (((uint32_t)(x) << PMU_DLC_IOIFR_ICU_OK_FLAG_SHIFT) & PMU_DLC_IOIFR_ICU_OK_FLAG_MASK)

/* IDIFR - PMU DLC icu_delayed interrupt flag register */

#define PMU_DLC_IDIFR_ICU_DELAYED_FLAG_MASK     (0xFFFFFFFEU)
#define PMU_DLC_IDIFR_ICU_DELAYED_FLAG_SHIFT    (1U)
#define PMU_DLC_IDIFR_ICU_DELAYED_FLAG(x)       (((uint32_t)(x) << PMU_DLC_IDIFR_ICU_DELAYED_FLAG_SHIFT) & PMU_DLC_IDIFR_ICU_DELAYED_FLAG_MASK)

/* IMCIFR - PMU DLC icu_mode changed interrupt flag register */

#define PMU_DLC_IMCIFR_ICU_MODE_CHANGED_FLAG_MASK     (0xFFFFFFFEU)
#define PMU_DLC_IMCIFR_ICU_MODE_CHANGED_FLAG_SHIFT    (1U)
#define PMU_DLC_IMCIFR_ICU_MODE_CHANGED_FLAG(x)       (((uint32_t)(x) << PMU_DLC_IMCIFR_ICU_MODE_CHANGED_FLAG_SHIFT) & PMU_DLC_IMCIFR_ICU_MODE_CHANGED_FLAG_MASK)

/* PCTRL_PADDR The address to write in the DLC_PADDR register is
 * CHIP_SEL_ADDR[4:0] concatenated with REG_ADDR[4:0].
 */

#define PMU_DLC_PICL_REG_ADDR_MASK          (0x1FU)
#define PMU_DLC_PICL_REG_ADDR_SHIFT         (0U)
#define PMU_DLC_PICL_REG_ADDR(x)            (((uint32_t)(x) /* << PMU_DLC_PICL_REG_ADDR_SHIFT */) & PMU_DLC_PICL_REG_ADDR_MASK)
#define PMU_DLC_PICL_CHIP_SEL_ADDR_MASK     (0x3E0U)
#define PMU_DLC_PICL_CHIP_SEL_ADDR_SHIFT    (5U)
#define PMU_DLC_PICL_CHIP_SEL_ADDR(x)       (((uint32_t)(x) << PMU_DLC_PICL_CHIP_SEL_ADDR_SHIFT) & PMU_DLC_PICL_CHIP_SEL_ADDR_MASK)

/* CHIP_SEL_ADDR[4:0] */

#define  PICL_WIU_ADDR         0x00
#define  PICL_ICU_ADDR         0x01

/* REG_ADDR[4:0] */

#define  WIU_ISPMR_0           (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x00))
#define  WIU_ISPMR_1           (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x01))
#define  WIU_IFR_0             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x02))
#define  WIU_IFR_1             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x03))
#define  WIU_ICR_0             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x04))
#define  WIU_ICR_1             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x05))
#define  WIU_ICR_2             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x06))
#define  WIU_ICR_3             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x07))
#define  WIU_ICR_4             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x08))
#define  WIU_ICR_5             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x09))
#define  WIU_ICR_6             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0A))
#define  WIU_ICR_7             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0B))
#define  WIU_ICR_8             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0C))
#define  WIU_ICR_9             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0D))
#define  WIU_ICR_10            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0E))
#define  WIU_ICR_11            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x0F))
#define  WIU_ICR_12            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x10))
#define  WIU_ICR_13            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x11))
#define  WIU_ICR_14            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x12))
#define  WIU_ICR_15            (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_WIU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x13))

/* REG_ADDR[4:0] */

#define  ICU_CR                (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_ICU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x00))
#define  ICU_MR                (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_ICU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x01))
#define  ICU_ISMR              (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_ICU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x02))
#define  ICU_DMR_0             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_ICU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x03))
#define  ICU_DMA_1             (PMU_DLC_PICL_CHIP_SEL_ADDR(PICL_ICU_ADDR) | PMU_DLC_PICL_REG_ADDR(0x04))

/* RTC_APB - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t STATUS;       /* RTC_APB_Status register */
  volatile  uint32_t REQUEST;      /* RTC_APB_Request register */
  volatile  uint32_t DATA;         /* RTC_APB_Data register */
  volatile  uint32_t _reserved;    /* reserved */
  volatile  uint32_t IRQ_CTRL;     /* RTC_APB_IRQ_Control register */
  volatile  uint32_t IRQ_MASK;     /* RTC_APB_IRQ_Mask register */
  volatile  uint32_t IRQ_FLAG;     /* RTC_APB_IRQ_Flag register */
} rtc_apb_reg_t;

#define RTC_APB_BASE                               (SOC_PERI_BASE + 0x08000u)
#define RTC_APB                                    ((rtc_apb_reg_t *)RTC_APB_BASE)

/* STATUS - RTC_APB STATUS register */

#define RTC_APB_STATUS_IRQ_EN_MASK                     (0x3FU)
#define RTC_APB_STATUS_IRQ_EN_SHIFT                    (0U)
#define RTC_APB_STATUS_IRQ_EN(x)                       (((uint32_t)(x)/* << RTC_APB_STATUS_IRQ_EN_SHIFT*/) & RTC_APB_STATUS_IRQ_EN_MASK)

/* REQUEST - RTC_APB REQUEST Access register */

#define RTC_APB_REQUEST_ACCESS_ADDR_MASK               (0x3FU)
#define RTC_APB_REQUEST_ACCESS_ADDR_SHIFT              (0U)
#define RTC_APB_REQUEST_ACCESS_ADDR(x)                 (((uint32_t)(x)/* << RTC_APB_REQUEST_ACCESS_ADDR_SHIFT*/) & RTC_APB_REQUEST_ACCESS_ADDR_MASK)
#define RTC_APB_REQUEST_ACCESS_RW_MASK                 (0x10000U)
#define RTC_APB_REQUEST_ACCESS_RW_SHIFT                (16U)
#define RTC_APB_REQUEST_ACCESS_RW(x)                   (((uint32_t)(x) << RTC_APB_REQUEST_ACCESS_RW_SHIFT) & RTC_APB_REQUEST_ACCESS_RW_MASK)

/* IRQ_FLAG - RTC_APB IRQ_FLAG Access register */

#define RTC_APB_IRQ_FLAG_READ_MASK                     (0x1U)
#define RTC_APB_IRQ_FLAG_READ_SHIFT                    (0U)
#define RTC_APB_IRQ_FLAG_READ(x)                       (((uint32_t)(x)/* << RTC_APB_IRQ_FLAG_READ_SHIFT*/) & RTC_APB_IRQ_FLAG_READ_MASK)
#define RTC_APB_IRQ_FLAG_WRITE_MASK                    (0x2U)
#define RTC_APB_IRQ_FLAG_WRITE_SHIFT                   (1U)
#define RTC_APB_IRQ_FLAG_WRITE(x)                      (((uint32_t)(x) << RTC_APB_IRQ_FLAG_WRITE_SHIFT) & RTC_APB_IRQ_FLAG_WRITE_MASK)

/* Bit field of RTC indirect Access Register */

#define RTC_STATUS_ADDR                     0x00
#define RTC_CTRL_ADDR                       0x01
#define RTC_CLK_CTRL_ADDR                   0x02
#define RTC_IRQ_CTRL_ADDR                   0x08
#define RTC_IRQ_MASK_ADDR                   0x09
#define RTC_IRQ_FLAG_ADDR                   0x0A
#define RTC_CALENDAR_CTRL_ADDR              0x10
#define RTC_CALENDAR_TIME_ADDR              0x12
#define RTC_CALENDAR_DATE_ADDR              0x13
#define RTC_ALARM_CTRL_ADDR                 0x18
#define RTC_ALARM_TIME_ADDR                 0x1A
#define RTC_ALARM_DATE_ADDR                 0x1B
#define RTC_TIMER_CTRL_ADDR                 0x20
#define RTC_TIMER_INIT_ADDR                 0x21
#define RTC_TIMER_VALUE_ADDR                0x22
#define RTC_CLKIN_DIV_ADDR                  0x28
#define RTC_REF_CLK_CONF_ADDR               0x2A
#define RTC_TEST_ADDR                       0x30

/* SR - RTC Status register */

#define RTC_SR_INT_RTC_MASK                 (0x1U)
#define RTC_SR_INT_RTC_SHIFT                (0U)
#define RTC_SR_INT_RTC(x)                   (((uint32_t)(x)/* << RTC_SR_INT_RTC_SHIFT*/) & RTC_SR_INT_RTC_MASK)

/* CR - RTC Control register */

#define RTC_CR_STANDBY_MASK                  (0x1U)
#define RTC_CR_STANDBY_SHIFT                 (0U)
#define RTC_CR_STANDBY(x)                    (((uint32_t)(x)/* << RTC_CR_STANDBY_SHIFT*/) & RTC_CR_STANDBY_MASK)
#define RTC_CR_CALIBRATION_EN_MASK           (0x10U)
#define RTC_CR_CALIBRATION_EN_SHIFT          (4U)
#define RTC_CR_CALIBRATION_EN(x)             (((uint32_t)(x) << RTC_CR_CALIBRATION_EN_SHIFT) & RTC_CR_CALIBRATION_EN_MASK)
#define RTC_CR_SOFT_RST_MASK                 (0x100U)
#define RTC_CR_SOFT_RST_SHIFT                (8U)
#define RTC_CR_SOFT_RST(x)                   (((uint32_t)(x) << RTC_CR_SOFT_RST_SHIFT) & RTC_CR_SOFT_RST_MASK)

/* CCR - RTC Clock Control register */

#define RTC_CCR_CKOUT_STANDBY_MASK           (0x1U)
#define RTC_CCR_CKOUT_STANDBY_SHIFT          (0U)
#define RTC_CCR_CKOUT_STANDBY(x)             (((uint32_t)(x)/* << RTC_CCR_CKOUT_STANDBY_SHIFT*/) & RTC_CCR_CKOUT_STANDBY_MASK)
#define RTC_CCR_DIV_AUTOCAL_MASK             (0x1000U)
#define RTC_CCR_DIV_AUTOCAL_SHIFT            (12U)
#define RTC_CCR_DIV_AUTOCAL(x)               (((uint32_t)(x) << RTC_CCR_DIV_AUTOCAL_SHIFT) & RTC_CCR_DIV_AUTOCAL_MASK)
#define RTC_CCR_DIV_COMP_MASK                (0x1F0000U)
#define RTC_CCR_DIV_COMP_SHIFT               (16U)
#define RTC_CCR_DIV_COMP(x)                  (((uint32_t)(x) << RTC_CCR_DIV_COMP_SHIFT) & RTC_CCR_DIV_COMP_MASK)

/* ICR - RTC IRQ Control register
 *
 * 00  INT_RTC high;
 * 01  INT_RTC low;
 * 10; INT_RTC high pulse with duration of 1 CKIN cycle
 * 11; INT_RTC low pulse with duration of 1 CKIN cycle
 */

#define RTC_ICR_FORM_MASK                    (0x3U)
#define RTC_ICR_FORM_SHIFT                   (0U)
#define RTC_ICR_FORM(x)                      (((uint32_t)(x)/* << RTC_ICR_FORM_SHIFT*/) & RTC_ICR_FORM_MASK)

/* IMR - RTC IRQ MASK register */

#define RTC_IMR_ALARM_MASK                   (0x1U)
#define RTC_IMR_ALARM_SHIFT                  (0U)
#define RTC_IMR_ALARM(x)                     (((uint32_t)(x)/* << RTC_IMR_ALARM_SHIFT*/) & RTC_IMR_ALARM_MASK)
#define RTC_IMR_TIMER_MASK                   (0x10U)
#define RTC_IMR_TIMER_SHIFT                  (4U)
#define RTC_IMR_TIMER(x)                     (((uint32_t)(x) << RTC_IMR_TIMER_SHIFT) & RTC_IMR_TIMER_MASK)
#define RTC_IMR_CALIBRATION_MASK             (0x1000U)
#define RTC_IMR_CALIBRATION_SHIFT            (12U)
#define RTC_IMR_CALIBRATION(x)               (((uint32_t)(x) << RTC_IMR_CALIBRATION_SHIFT) & RTC_IMR_CALIBRATION_MASK)

/* IFR - RTC IRQ Flag register */

#define RTC_IFR_ALARM_MASK                   (0x1U)
#define RTC_IFR_ALARM_SHIFT                  (0U)
#define RTC_IFR_ALARM(x)                     (((uint32_t)(x)/* << RTC_IFR_ALARM_SHIFT*/) & RTC_IFR_ALARM_MASK)
#define RTC_IFR_TIMER_MASK                   (0x10U)
#define RTC_IFR_TIMER_SHIFT                  (4U)
#define RTC_IFR_TIMER(x)                     (((uint32_t)(x) << RTC_IFR_TIMER_SHIFT) & RTC_IFR_TIMER_MASK)
#define RTC_IFR_CALIBRATION_MASK             (0x1000U)
#define RTC_IFR_CALIBRATION_SHIFT            (12U)
#define RTC_IFR_CALIBRATION(x)               (((uint32_t)(x) << RTC_IFR_CALIBRATION_SHIFT) & RTC_IFR_CALIBRATION_MASK)

/* CALENDAR CTRL - RTC CALENDAR Control register */

#define RTC_CALENDAR_CTRL_STANDBY_MASK       (0x1U)
#define RTC_CALENDAR_CTRL_STANDBY_SHIFT      (0U)
#define RTC_CALENDAR_CTRL_STANDBY(x)         (((uint32_t)(x)/* << RTC_CALENDAR_CTRL_STANDBY_SHIFT*/) & RTC_CALENDAR_CTRL_STANDBY_MASK)

/* ALARM_CTRL - RTC Alarm control register */

#define RTC_ALARM_CTRL_STANDBY_MASK           (0x1U)
#define RTC_ALARM_CTRL_STANDBY_SHIFT          (0U)
#define RTC_ALARM_CTRL_STANDBY(x)             (((uint32_t)(x)/* << RTC_ALARM_CTRL_STANDBY_SHIFT*/) & RTC_ALARM_CTRL_STANDBY_MASK)
#define RTC_ALARM_CTRL_MODE_MASK              (0x10U)
#define RTC_ALARM_CTRL_MODE_SHIFT             (4U)
#define RTC_ALARM_CTRL_MODE(x)                (((uint32_t)(x) << RTC_ALARM_CTRL_MODE_SHIFT) & RTC_ALARM_CTRL_MODE_MASK)
#define RTC_ALARM_CTRL_CONFIG_MASK            (0xF0000U)
#define RTC_ALARM_CTRL_CONFIG_SHIFT           (16U)
#define RTC_ALARM_CTRL_CONFIG(x)              (((uint32_t)(x) << RTC_ALARM_CTRL_CONFIG_SHIFT) & RTC_ALARM_CTRL_CONFIG_MASK)

/* TIMER - RTC Count down register */

#define RTC_TIMER_STANDBY_MASK                (0x1U)
#define RTC_TIMER_STANDBY_SHIFT               (0U)
#define RTC_TIMER_STANDBY(x)                  (((uint32_t)(x)/* << RTC_TIMER_STANDBY_SHIFT*/) & RTC_TIMER_STANDBY_MASK)
#define RTC_TIMER_MODE_MASK                   (0x10U)
#define RTC_TIMER_MODE_SHIFT                  (4U)
#define RTC_TIMER_MODE(x)                     (((uint32_t)(x) << RTC_TIMER_MODE_SHIFT) & RTC_TIMER_MODE_MASK)

/* CLKIN_DIV - RTC Clock in divider register */

#define RTC_CLKIN_DIV_VAL_MASK                (0xFFFFU)
#define RTC_CLKIN_DIV_VAL_SHIFT               (0U)
#define RTC_CLKIN_DIV_VAL(x)                  (((uint32_t)(x)/* << RTC_CLKIN_DIV_VAL_SHIFT*/) & RTC_CLKIN_DIV_VAL_MASK)

/* CKREF_CONF - RTC Reference Clock configuration */

#define RTC_CKREF_CONF_VAL_MASK               (0x3FFFFFU)
#define RTC_CKREF_CONF_VAL_SHIFT              (0U)
#define RTC_CKREF_CONF_VAL(x)                 (((uint32_t)(x)/* << RTC_CKREF_CONF_VAL_SHIFT*/) & RTC_CKREF_CONF_VAL_MASK)

/* EFUSE_CTRL - Register Layout Typedef */

typedef struct
{
  volatile  uint32_t CMD;       /* EFUSE_Control register */
  volatile  uint32_t CFG;       /* EFUSE_Control register */
} efuse_ctrl_reg_t;

#define EFUSE_CTRL_BASE                               (SOC_PERI_BASE + 0x09000u)
#define EFUSE_CTRL                                    ((efuse_ctrl_reg_t *)EFUSE_CTRL_BASE)

#define    EFUSE_CTRL_CMD_READ       0x1
#define    EFUSE_CTRL_CMD_WRITE      0x2
#define    EFUSE_CTRL_CMD_SLEEP      0x4

/* EFUSE_REGS - Registers Layout Typedef */

typedef struct
{
  volatile  uint32_t INFO;                    /**< EFUSE INFO register, offset: 0x000 */
  volatile  uint32_t INFO2;                   /**< EFUSE_INFO2 register, offset: 0x004 */
  volatile  uint32_t AES_KEY[16];             /**< EFUSE_AES_KEY registers, offset: 0x008 */
  volatile  uint32_t AES_IV[8];               /**< EFUSE_AES_IV registers, offset: 0x048 */
  volatile  uint32_t WAIT_XTAL_DELTA_LSB;     /**< EFUSE_WAIT_XTAL_DELTA_LSB register, offset: 0x068 */
  volatile  uint32_t WAIT_XTAL_DELTA_MSB;     /**< EFUSE_WAIT_XTAL_DELTA_MSB register, offset: 0x06C */
  volatile  uint32_t WAIT_XTAL_MIN;           /**< EFUSE_WAIT_XTAL_MIN registers, offset: 0x070 */
  volatile  uint32_t WAIT_XTAL_MAX;           /**< EFUSE_WAIT_XTAL_MAX registers, offset: 0x074 */
} efuse_regs_reg_t;

#define EFUSE_REGS_BASE                                (SOC_PERI_BASE + 0x09200u)
#define EFUSE_REGS                                     ((efuse_regs_reg_t *)EFUSE_REGS_BASE)

/* INFO - EFUSE information register */

#define EFUSE_INFO_PLT_MASK                           (0x07U)
#define EFUSE_INFO_PLT_SHIFT                          (0U)
#define EFUSE_INFO_PLT(x)                             (((uint32_t)(x) << EFUSE_INFO_PLT_SHIFT) & EFUSE_INFO_PLT_MASK)

#define EFUSE_INFO_BOOT_MASK                          (0x38U)
#define EFUSE_INFO_BOOT_SHIFT                         (3U)
#define EFUSE_INFO_BOOT(x)                            (((uint32_t)(x) << EFUSE_INFO_BOOT_SHIFT) & EFUSE_INFO_BOOT_MASK)

#define EFUSE_INFO_ENCRYPTED_MASK                     (0x40U)
#define EFUSE_INFO_ENCRYPTED_SHIFT                    (6U)
#define EFUSE_INFO_ENCRYPTED(x)                       (((uint32_t)(x) << EFUSE_INFO_ENCRYPTED_SHIFT) & EFUSE_INFO_ENCRYPTED_MASK)

#define EFUSE_INFO_WAIT_XTAL_MASK                     (0x80U)
#define EFUSE_INFO_WAIT_XTAL_SHIFT                    (7U)
#define EFUSE_INFO_WAIT_XTAL(x)                       (((uint32_t)(x) << EFUSE_INFO_WAIT_XTAL_SHIFT) & EFUSE_INFO_WAIT_XTAL_MASK)

/* FC_STDOUT - Registers Layout Typedef */

typedef struct
{
  volatile  uint32_t PUTC[16];      /* FC_STDOUT INFO register, offset: 0x000 */
} fc_stdout_reg_t;

#define FC_STDOUT_BASE                                (SOC_PERI_BASE + 0x10000u + (32 << 7))
#define FC_STDOUT                                     ((fc_stdout_reg_t *)FC_STDOUT_BASE)

#ifdef FEATURE_CLUSTER
/* CLUSTER_STDOUT - Registers Layout Typedef */

typedef struct
{
  volatile  uint32_t PUTC[16];       /* CLUSTER_STDOUT INFO register, offset: 0x000 */
} cluster_stdout_reg_t;

#define CLUSTER_STDOUT_BASE                                (SOC_PERI_BASE + 0x10000u)
#define CLUSTER_STDOUT                                     ((cluster_stdout_reg_t *)CLUSTER_STDOUT_BASE)

/* HWCE - Registers Layout Typedef */

typedef struct
{
  volatile  uint32_t HWCE_TRIGGER_REG;              /* Trigger register */
  volatile  uint32_t HWCE_ACQUIRE_REG;              /* Acquire register */
  volatile  uint32_t HWCE_FINISHED_REG;             /* Finished register */
  volatile  uint32_t HWCE_STATUS_REG;               /* Status register */
  volatile  uint32_t HWCE_RUNNING_JOB_REG;          /* Running Job register */
  volatile  uint32_t HWCE_SOFT_CLEAR_REG;           /* Soft_Clear register */
  volatile  uint32_t _reserved0[2];                 /* Non used registers */
  volatile  uint32_t HWCE_GEN_CONFIG0_REG;          /* Gen_Config0 register */
  volatile  uint32_t HWCE_GEN_CONFIG1_REG;          /* Gen_Config1 register */
  volatile  uint32_t _reserved1[6];                 /* unused registers */
  volatile  uint32_t HWCE_Y_TRANS_SIZE_REG;         /* Y_Trans_Size register */
  volatile  uint32_t HWCE_Y_LINE_STRIDE_LENGTH_REG; /* Y_Line_Stride_Length register */
  volatile  uint32_t HWCE_Y_FEAT_STRIDE_LENGTH_REG; /* Y_Feat_Stride_Length register */
  volatile  uint32_t HWCE_Y_OUT_3_REG;              /* Y_Out_3 register */
  volatile  uint32_t HWCE_Y_OUT_2_REG;              /* Y_Out_2 register */
  volatile  uint32_t HWCE_Y_OUT_1_REG;              /* Y_Out_1 register */
  volatile  uint32_t HWCE_Y_OUT_0_REG;              /* Y_Out_0 register */
  volatile  uint32_t HWCE_Y_IN_3_REG;               /* Y_In_3 register */
  volatile  uint32_t HWCE_Y_IN_2_REG;               /* Y_In_2 register */
  volatile  uint32_t HWCE_Y_IN_1_REG;               /* Y_In_1 register */
  volatile  uint32_t HWCE_Y_IN_0_REG;               /* Y_In_0 register */
  volatile  uint32_t HWCE_X_TRANS_SIZE_REG;         /* X_Trans_Size register */
  volatile  uint32_t HWCE_X_LINE_STRIDE_LENGTH_REG; /* X_Line_Stride_Length register */
  volatile  uint32_t HWCE_X_FEAT_STRIDE_LENGTH_REG; /* X_Feat_Stride_Length register */
  volatile  uint32_t HWCE_X_IN_REG;                 /* X_In register */
  volatile  uint32_t HWCE_W_REG;                    /* W register */
  volatile  uint32_t HWCE_JOB_CONFIG0_REG;          /* Job_Config0 register */
  volatile  uint32_t HWCE_JOB_CONFIG1_REG;          /* Job_Config1 register */
} hwce_reg_t;

#define HWCE_BASE                                (CORE_PERI_BASE + 0x00001000)
#define HWCE                                     ((hwce_reg_t *) HWCE_BASE)

/* Internal registers */

#define HWCE_TRIGGER              (0x00)
#define HWCE_ACQUIRE              (0x04)
#define HWCE_FINISHED             (0x08)
#define HWCE_STATUS               (0x0C)
#define HWCE_RUNNING_JOB          (0x10)
#define HWCE_SOFT_CLEAR           (0x14)
#define HWCE_GEN_CONFIG0          (0x20)
#define HWCE_GEN_CONFIG1          (0x24)

/* Configuration registers */

#define HWCE_Y_TRANS_SIZE         (0x40)
#define HWCE_Y_LINE_STRIDE_LENGTH (0x44)
#define HWCE_Y_FEAT_STRIDE_LENGTH (0x48)
#define HWCE_Y_OUT_3_BASE_ADDR    (0x4C)
#define HWCE_Y_OUT_2_BASE_ADDR    (0x50)
#define HWCE_Y_OUT_1_BASE_ADDR    (0x54)
#define HWCE_Y_OUT_0_BASE_ADDR    (0x58)
#define HWCE_Y_IN_3_BASE_ADDR     (0x5C)
#define HWCE_Y_IN_2_BASE_ADDR     (0x60)
#define HWCE_Y_IN_1_BASE_ADDR     (0x64)
#define HWCE_Y_IN_0_BASE_ADDR     (0x68)
#define HWCE_X_TRANS_SIZE         (0x6C)
#define HWCE_X_LINE_STRIDE_LENGTH (0x70)
#define HWCE_X_FEAT_STRIDE_LENGTH (0x74)
#define HWCE_X_IN_BASE_ADDR       (0x78)
#define HWCE_W_BASE_ADDR          (0x7C)
#define HWCE_JOB_CONFIG0          (0x80)
#define HWCE_JOB_CONFIG1          (0x84)

#define HWCE_NB_IO_REGS           (18)

#define HWCE_ACQUIRE_CONTEXT_COPY (-3)
#define HWCE_ACQUIRE_LOCKED       (-2)
#define HWCE_ACQUIRE_QUEUE_FULL   (-1)
#define HWCE_ACQUIRE_READY        (0)

#define HWCE_GEN_CONFIG0_WSTRIDE(x)           ((x) >> 16)
#define HWCE_GEN_CONFIG0_NCP(x)               (((x) >> 13) & 0x1)
#define HWCE_GEN_CONFIG0_CONV(x)              (((x) >> 11) & 0x3)
#define HWCE_GEN_CONFIG0_VECT(x)              (((x) >> 9) & 0x3)
#define HWCE_GEN_CONFIG0_UNS(x)               (((x) >> 8) & 1)
#define HWCE_GEN_CONFIG0_NY(x)                (((x) >> 7) & 1)
#define HWCE_GEN_CONFIG0_NF(x)                (((x) >> 6) & 1)
#define HWCE_GEN_CONFIG0_QF(x)                ((x) & 0x3f)

#define HWCE_GEN_CONFIG0_CONV_5x5 (0)
#define HWCE_GEN_CONFIG0_CONV_3x3 (1)
#define HWCE_GEN_CONFIG0_CONV_4x7 (2)

#define HWCE_GEN_CONFIG0_VECT_1   (0)
#define HWCE_GEN_CONFIG0_VECT_2   (1)
#define HWCE_GEN_CONFIG0_VECT_4   (2)

#define HWCE_GEN_CONFIG1_PIXSHIFTR(x)         (((x) >> 16) & 0x1F)
#define HWCE_GEN_CONFIG1_PIXMODE(x)           (((x) >> 8) & 0x3)
#define HWCE_GEN_CONFIG1_PIXSHIFTL(x)         (((x) >> 0) & 0x1F)

#define HWCE_JOB_CONFIG0_NOYCONST(x)          ((x) >> 16)
#define HWCE_JOB_CONFIG0_LBUFLEN(x)           ((x) & 0x3ff)

#define HWCE_JOB_CONFIG1_LO(x)                (((x) >> 24) & 0x1)
#define HWCE_JOB_CONFIG1_WIF(x)               (((x) >> 16) & 0x3F)
#define HWCE_JOB_CONFIG1_WOF(x)               (((x) >> 8) & 0x1F)
#define HWCE_JOB_CONFIG1_VECT_DISABLE_MASK(x) (((x) >> 0) & 0xF)

#define HWCE_JOB_STRIDE(x)                    ((x) >> 16)
#define HWCE_JOB_LENGTH(x)                    ((x) & 0xffff)

#endif

#endif /* __ARCH_RISC_V_SRC_GAP8_GAP8_H */
