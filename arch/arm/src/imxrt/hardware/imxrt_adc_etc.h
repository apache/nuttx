/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_adc_etc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_ETC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_ETC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRIG_OFFSET   0x28
#define CHAIN_OFFSET  0x4
#define RESULT_OFFSET 0x4

/* Register Offsets *********************************************************/

#define IMXRT_ADC_ETC_CTRL_OFFSET           0x0000  /* Global control register */
#define IMXRT_ADC_ETC_DONE01_IRQ_OFFSET     0x0004  /* ETC DONE 0 and DONE 1 IRQ state register */
#define IMXRT_ADC_ETC_DONE2_ERR_IRQ_OFFSET  0x0008  /* ETC DONE 2 and DONE ERR IRQ state register */
#define IMXRT_ADC_ETC_DMA_CTRL_OFFSET       0x000c  /* DMA control register */
#define IMXRT_ADC_ETC_TRIG_CTRL_OFFSET      0x0010  /* TRIG control register */
#define IMXRT_ADC_ETC_TRIG_COUNTER_OFFSET   0x0014  /* TRIG counter register */
#define IMXRT_ADC_ETC_TRIG_CHAIN_OFFSET     0x0018  /* TRIG chain register */
#define IMXRT_ADC_ETC_TRIG_RESULT_OFFSET    0x0028  /* TRIG result data register */

/* Register Bit Definitions *************************************************/

/* ADC_ETC global control register */

#define ADC_ETC_CTRL_TRIG_EN_SHIFT          (0)         /* Bits 0-7:   TRIG enable */
#define ADC_ETC_CTRL_TRIG_EN_MASK           (0xff << ADC_ETC_CTRL_TRIG_EN_SHIFT)
#define ADC_ETC_CTRL_TRIG_EN(n)             (((uint32_t)(n) << ADC_ETC_CTRL_TRIG_EN_SHIFT) & ADC_ETC_CTRL_TRIG_EN_MASK)
#define ADC_ETC_CTRL_EXT0_TRIG_EN           (1 << 8)    /* Bit  8:     TSC0 TRIG enable */
#define ADC_ETC_CTRL_EXT0_TRIG_PR_SHIFT     (9)         /* Bits 9-11:  TSC0 trigger priority */
#define ADC_ETC_CTRL_EXT0_TRIG_PR_MASK      (0x7 << ADC_ETC_CTRL_EXT0_TRIG_PR_SHIFT)
#define ADC_ETC_CTRL_EXT0_TRIG_PR(n)        (((uint32_t)(n) << ADC_ETC_CTRL_EXT0_TRIG_PR_SHIFT) & ADC_ETC_CTRL_EXT0_TRIG_PR_MASK)
#define ADC_ETC_CTRL_EXT1_TRIG_EN           (1 << 12)   /* Bit  12:    TSC1 TRIG enable */
#define ADC_ETC_CTRL_EXT1_TRIG_PR_SHIFT     (13)        /* Bits 13-15: TSC1 trigger priority */
#define ADC_ETC_CTRL_EXT1_TRIG_PR_MASK      (0x7 << ADC_ETC_CTRL_EXT1_TRIG_PR_SHIFT)
#define ADC_ETC_CTRL_EXT1_TRIG_PR(n)        (((uint32_t)(n) << ADC_ETC_CTRL_EXT1_TRIG_PR_SHIFT) & ADC_ETC_CTRL_EXT1_TRIG_PR_MASK)
#define ADC_ETC_CTRL_PRE_DIVIDER_SHIFT      (16)        /* Bits 16-23: Pre divider for trig delay and interval */
#define ADC_ETC_CTRL_PRE_DIVIDER_MASK       (0xff << ADC_ETC_CTRL_PRE_DIVIDER_SHIFT)
#define ADC_ETC_CTRL_PRE_DIVIDER(n)         (((uint32_t)(n) << ADC_ETC_CTRL_PRE_DIVIDER_SHIFT) & ADC_ETC_CTRL_PRE_DIVIDER_MASK)
                                                        /* Bits 24-28: Reserved */
#define ADC_ETC_CTRL_DMA_MODE_SEL           (1 << 29)   /* Bit  29:    Trig DMA with pulsed signal */
#define ADC_ETC_CTRL_TSC_BYPASS             (1 << 30)   /* Bit  30:    TSC is bypassed to ADC2 */
#define ADC_ETC_CTRL_SOFTRST                (1 << 31)   /* Bit  31:    Software reset */

/* ADC_ETC DONE 0 and DONE 1 IRQ state register */

#define ADC_ETC_DONE01_IRQ_TRIG0_DONE0      (1 << 0)    /* Bit  0:     TRIG0 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG1_DONE0      (1 << 1)    /* Bit  1:     TRIG1 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG2_DONE0      (1 << 2)    /* Bit  2:     TRIG2 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG3_DONE0      (1 << 3)    /* Bit  3:     TRIG3 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG4_DONE0      (1 << 4)    /* Bit  4:     TRIG4 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG5_DONE0      (1 << 5)    /* Bit  5:     TRIG5 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG6_DONE0      (1 << 6)    /* Bit  6:     TRIG6 done0 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG7_DONE0      (1 << 7)    /* Bit  7:     TRIG7 done0 interrupt detection */
                                                        /* Bits 8-15:  Reserved */
#define ADC_ETC_DONE01_IRQ_TRIG0_DONE1      (1 << 16)   /* Bit  16:    TRIG0 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG1_DONE1      (1 << 17)   /* Bit  17:    TRIG1 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG2_DONE1      (1 << 18)   /* Bit  18:    TRIG2 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG3_DONE1      (1 << 19)   /* Bit  19:    TRIG3 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG4_DONE1      (1 << 20)   /* Bit  20:    TRIG4 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG5_DONE1      (1 << 21)   /* Bit  21:    TRIG5 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG6_DONE1      (1 << 22)   /* Bit  22:    TRIG6 done1 interrupt detection */
#define ADC_ETC_DONE01_IRQ_TRIG7_DONE1      (1 << 23)   /* Bit  23:     TRIG7 done1 interrupt detection */
                                                        /* Bits 24-31: Reserved */

/* ADC_ETC DONE 2 and DONE ERR IRQ state register */

#define ADC_ETC_DONE2_ERR_IRQ_TRIG0_DONE2   (1 << 0)    /* Bit  0:     TRIG0 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG1_DONE2   (1 << 1)    /* Bit  1:     TRIG1 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG2_DONE2   (1 << 2)    /* Bit  2:     TRIG2 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG3_DONE2   (1 << 3)    /* Bit  3:     TRIG3 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG4_DONE2   (1 << 4)    /* Bit  4:     TRIG4 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG5_DONE2   (1 << 5)    /* Bit  5:     TRIG5 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG6_DONE2   (1 << 6)    /* Bit  6:     TRIG6 done2 interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG7_DONE2   (1 << 7)    /* Bit  7:     TRIG7 done2 interrupt detection */
                                                        /* Bits 8-15:  Reserved */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG0_ERR     (1 << 16)   /* Bit  16:    TRIG0 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG1_ERR     (1 << 17)   /* Bit  17:    TRIG1 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG2_ERR     (1 << 18)   /* Bit  18:    TRIG2 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG3_ERR     (1 << 19)   /* Bit  19:    TRIG3 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG4_ERR     (1 << 20)   /* Bit  20:    TRIG4 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG5_ERR     (1 << 21)   /* Bit  21:    TRIG5 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG6_ERR     (1 << 22)   /* Bit  22:    TRIG6 error interrupt detection */
#define ADC_ETC_DONE2_ERR_IRQ_TRIG7_ERR     (1 << 23)   /* Bit  23:    TRIG7 error interrupt detection */
                                                        /* Bits 24-31: Reserved */

/* ADC_ETC DMA control register */

#define ADC_ETC_DMA_CTRL_TRIG0_EN           (1 << 0)    /* Bit  0:     TRIG0 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG1_EN           (1 << 1)    /* Bit  1:     TRIG1 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG2_EN           (1 << 2)    /* Bit  2:     TRIG2 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG3_EN           (1 << 3)    /* Bit  3:     TRIG3 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG4_EN           (1 << 4)    /* Bit  4:     TRIG4 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG5_EN           (1 << 5)    /* Bit  5:     TRIG5 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG6_EN           (1 << 6)    /* Bit  6:     TRIG6 enable dma request */
#define ADC_ETC_DMA_CTRL_TRIG7_EN           (1 << 7)    /* Bit  7:     TRIG7 enable dma request */
                                                        /* Bits 8-15:  Reserved */
#define ADC_ETC_DMA_CTRL_TRIG0_REQ          (1 << 16)   /* Bit  16:    TRIG0 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG1_REQ          (1 << 17)   /* Bit  17:    TRIG1 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG2_REQ          (1 << 18)   /* Bit  18:    TRIG2 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG3_REQ          (1 << 19)   /* Bit  19:    TRIG3 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG4_REQ          (1 << 20)   /* Bit  20:    TRIG4 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG5_REQ          (1 << 21)   /* Bit  21:    TRIG5 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG6_REQ          (1 << 22)   /* Bit  22:    TRIG6 done DMA request detection */
#define ADC_ETC_DMA_CTRL_TRIG7_REQ          (1 << 23)   /* Bit  23:    TRIG7 done DMA request detection */
                                                        /* Bits 24-31: Reserved */

/* ADC_ECT TRIG control register */

#define ADC_ETC_TRIG_CTRL_SW                (1 << 0)    /* Bit  0:     Sotware write 1 as the TRIGGER */
                                                        /* Bits 1-3:   Reserved */
#define ADC_ETC_TRIG_CTRL_TRIG_MODE         (1 << 4)    /* Bit  4:     Sotware trigger select */
                                                        /* Bits 5-7:   Reserved */
#define ADC_ETC_TRIG_CTRL_TRIG_CHAIN_SHIFT  (8)         /* Bits 8-11:  TRIG chain lenght to the ADC */
#define ADC_ETC_TRIG_CTRL_TRIG_CHAIN_MASK   (0x7 << ADC_ETC_TRIG_CTRL_TRIG_CHAIN_SHIFT)
#define ADC_ETC_TRIG_CTRL_TRIG_CHAIN(n)     (((uint32_t)(n) << ADC_ETC_TRIG_CTRL_TRIG_CHAIN_SHIFT) & ADC_ETC_TRIG_CTRL_TRIG_CHAIN_MASK)
                                                        /* Bit  11:    Reserved */
#define ADC_ETC_TRIG_CTRL_TRIG_PR_SHIFT     (12)        /* Bits 12-14: External trigger priority */
#define ADC_ETC_TRIG_CTRL_TRIG_PR_MASK      (0x7 << ADC_ETC_TRIG_CTRL_TRIG_PR_SHIFT)
#define ADC_ETC_TRIG_CTRL_TRIG_PR(n)        (((uint32_t)(n) << ADC_ETC_TRIG_CTRL_TRIG_PR_SHIFT) & ADC_ETC_TRIG_CTRL_TRIG_PR_MASK)
                                                        /* Bit  15:    Reserved */
#define ADC_ETC_TRIG_CTRL_SYNC_MODE         (1 << 16)   /* Bit  16:    Sync mode enable */
                                                        /* Bits 17-31: Reserved */

/* ADC_ETC TRIG counter register */

#define ADC_ETC_TRIG_COUNTER_INIT_DELAY_SHIFT      (0)     /* Bits 0-15:  Initial delay counter */
#define ADC_ETC_TRIG_COUNTER_INIT_DELAY_MASK       (0xffff << ADC_ETC_TRIG_COUNTER_INIT_DELAY_SHIFT)
#define ADC_ETC_TRIG_COUNTER_INIT_DELAY(n)         (((uint32_t)(n) << ADC_ETC_TRIG_COUNTER_INIT_DELAY_SHIFT) & ADC_ETC_TRIG_COUNTER_INIT_DELAY_MASK)
#define ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT_SHIFT (16)    /* Bits 16-31: Sampling interval counter */
#define ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT_MASK  (0xffff << ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT_SHIFT)
#define ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT(n)    (((uint32_t)(n) << ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT_SHIFT) & ADC_ETC_TRIG_COUNTER_INIT_SAMPLE_INT_MASK)

/* ADC_ETC TRIG chain register */

#define ADC_ETC_TRIG_CHAIN_CSEL0_SHIFT    (0)         /* Bits 0-3:   ADC channel selection */
#define ADC_ETC_TRIG_CHAIN_CSEL0_MASK     (0xf << ADC_ETC_TRIG_CHAIN_CSEL0_SHIFT)
#define ADC_ETC_TRIG_CHAIN_CSEL0(n)       (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_CSEL0_SHIFT) & ADC_ETC_TRIG_CHAIN_CSEL0_MASK)
#define ADC_ETC_TRIG_CHAIN_HWTS0_SHIFT    (4)         /* Bits 4-11:  ADC HW trigger selection */
#define ADC_ETC_TRIG_CHAIN_HTWS0_MASK     (0xff << ADC_ETC_TRIG_CHAIN_HWTS0_SHIFT)
#define ADC_ETC_TRIG_CHAIN_HWTS0(n)       (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_HWTS0_SHIFT) & ADC_ETC_TRIG_CHAIN_HTWS0_MASK)
#define ADC_ETC_TRIG_CHAIN_B2B0           (1 << 12)   /* Bit  12:    Enable B2B */
#define ADC_ETC_TRIG_CHAIN_IE0_SHIFT      (13)        /* Bits 13-14: Chain0 IE */
#define ADC_ETC_TRIG_CHAIN_IE0_MASK       (0x3 << ADC_ETC_TRIG_CHAIN_IE0_SHIFT)
#define ADC_ETC_TRIG_CHAIN_IE0(n)         (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_IE0_SHIFT) & ADC_ETC_TRIG_CHAIN_IE0_MASK)
                                                      /* Bit  15:    Reserved */
#define ADC_ETC_TRIG_CHAIN_CSEL1_SHIFT    (16)        /* Bits 16-19: ADC channel selection */
#define ADC_ETC_TRIG_CHAIN_CSEL1_MASK     (0xf << ADC_ETC_TRIG_CHAIN_CSEL1_SHIFT)
#define ADC_ETC_TRIG_CHAIN_CSEL1(n)       (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_CSEL1_SHIFT) & ADC_ETC_TRIG_CHAIN_CSEL1_MASK)
#define ADC_ETC_TRIG_CHAIN_HWTS1_SHIFT    (20)        /* Bits 20-27: ADC HW trigger selection */
#define ADC_ETC_TRIG_CHAIN_HTWS1_MASK     (0xff << ADC_ETC_TRIG_CHAIN_HWTS1_SHIFT)
#define ADC_ETC_TRIG_CHAIN_HWTS1(n)       (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_HWTS1_SHIFT) & ADC_ETC_TRIG_CHAIN_HTWS1_MASK)
#define ADC_ETC_TRIG_CHAIN_B2B1           (1 << 28)   /* Bit  28:    Enable B2B */
#define ADC_ETC_TRIG_CHAIN_IE1_SHIFT      (29)        /* Bits 29-30: Chain0 IE */
#define ADC_ETC_TRIG_CHAIN_IE1_MASK       (0x3 << ADC_ETC_TRIG_CHAIN_IE1_SHIFT)
#define ADC_ETC_TRIG_CHAIN_IE1(n)         (((uint32_t)(n) << ADC_ETC_TRIG_CHAIN_IE1_SHIFT) & ADC_ETC_TRIG_CHAIN_IE1_MASK)
                                                      /* Bit  31:    Reserved */

/* ADC_ETC TRIG result data register */

#define ADC_ETC_TRIG_RESULT_DATA0_SHIFT   (0)         /* Bits 0-11:  Result DATA 0 */
#define ADC_ETC_TRIG_RESULT_DATA0_MASK    (0xfff << ADC_ETC_TRIG_RESULT01_DATA0_SHIFT)
#define ADC_ETC_TRIG_RESULT_DATA0(n)      (((uint32_t)(n) << ADC_ETC_TRIG_RESULT01_DATA0_SHIFT) & ADC_ETC_TRIG_RESULT01_DATA0_MASK)
                                                      /* Bits 12-15: Reserved */
#define ADC_ETC_TRIG_RESULT_DATA1_SHIFT   (16)        /* Bits 16-27: Result DATA 1 */
#define ADC_ETC_TRIG_RESULT_DATA1_MASK    (0xfff << ADC_ETC_TRIG_RESULT01_DATA1_SHIFT)
#define ADC_ETC_TRIG_RESULT_DATA1(n)      (((uint32_t)(n) << ADC_ETC_TRIG_RESULT01_DATA1_SHIFT) & ADC_ETC_TRIG_RESULT01_DATA1_MASK)
                                                      /* Bits 28-31: Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_ETC_H */
