/****************************************************************************
 * arch/arm/src/tiva/hardware/tiva_ssi.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_SSI_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_SSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#if TIVA_NSSI > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SSI register offsets *****************************************************/

#define TIVA_SSI_CR0_OFFSET         0x000  /* SSI Control 0 */
#define TIVA_SSI_CR1_OFFSET         0x004  /* SSI Control 1 */
#define TIVA_SSI_DR_OFFSET          0x008  /* SSI Data */
#define TIVA_SSI_SR_OFFSET          0x00c  /* SSI Status */
#define TIVA_SSI_CPSR_OFFSET        0x010  /* SSI Clock Prescale */
#define TIVA_SSI_IM_OFFSET          0x014  /* SSI Interrupt Mask */
#define TIVA_SSI_RIS_OFFSET         0x018  /* SSI Raw Interrupt Status */
#define TIVA_SSI_MIS_OFFSET         0x01c  /* SSI Masked Interrupt Status */
#define TIVA_SSI_ICR_OFFSET         0x020  /* SSI Interrupt Clear */
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_SSI_DMACR_OFFSET     0x024  /* SSI DMA Control */

#else /* if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TIVA) */
#  define TIVA_SSI_PERIPHID4_OFFSET 0xfd0  /* SSI Peripheral Identification 4 */
#  define TIVA_SSI_PERIPHID5_OFFSET 0xfd4  /* SSI Peripheral Identification 5 */
#  define TIVA_SSI_PERIPHID6_OFFSET 0xfd8  /* SSI Peripheral Identification 6 */
#  define TIVA_SSI_PERIPHID7_OFFSET 0xfdc  /* SSI Peripheral Identification 7 */
#  define TIVA_SSI_PERIPHID0_OFFSET 0xfe0  /* SSI Peripheral Identification 0 */
#  define TIVA_SSI_PERIPHID1_OFFSET 0xfe4  /* SSI Peripheral Identification 1 */
#  define TIVA_SSI_PERIPHID2_OFFSET 0xfe8  /* SSI Peripheral Identification 2 */
#  define TIVA_SSI_PERIPHID3_OFFSET 0xfec  /* SSI Peripheral Identification 3 */
#  define TIVA_SSI_PCELLID0_OFFSET  0xff0  /* SSI PrimeCell Identification 0 */
#  define TIVA_SSI_PCELLID1_OFFSET  0xff4  /* SSI PrimeCell Identification 1 */
#  define TIVA_SSI_PCELLID2_OFFSET  0xff8  /* SSI PrimeCell Identification 2 */
#  define TIVA_SSI_PCELLID3_OFFSET  0xffc  /* SSI PrimeCell Identification 3 */
#endif

/* SSI register addresses ***************************************************/

#define TIVA_SSI0_CR0               (TIVA_SSI0_BASE + TIVA_SSI_CR0_OFFSET)
#define TIVA_SSI0_CR1               (TIVA_SSI0_BASE + TIVA_SSI_CR1_OFFSET)
#define TIVA_SSI0_DR                (TIVA_SSI0_BASE + TIVA_SSI_DR_OFFSET)
#define TIVA_SSI0_SR                (TIVA_SSI0_BASE + TIVA_SSI_SR_OFFSET)
#define TIVA_SSI0_CPSR              (TIVA_SSI0_BASE + TIVA_SSI_CPSR_OFFSET)
#define TIVA_SSI0_IM                (TIVA_SSI0_BASE + TIVA_SSI_IM_OFFSET)
#define TIVA_SSI0_RIS               (TIVA_SSI0_BASE + TIVA_SSI_RIS_OFFSET)
#define TIVA_SSI0_MIS               (TIVA_SSI0_BASE + TIVA_SSI_MIS_OFFSET)
#define TIVA_SSI0_ICR               (TIVA_SSI0_BASE + TIVA_SSI_ICR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_SSI0_DMACR           (TIVA_SSI0_BASE + TIVA_SSI_DMACR_OFFSET)
#else /* if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TIVA) */
#  define TIVA_SSI0_PERIPHID4       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID4_OFFSET)
#  define TIVA_SSI0_PERIPHID5       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID5_OFFSET)
#  define TIVA_SSI0_PERIPHID6       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID6_OFFSET)
#  define TIVA_SSI0_PERIPHID7       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID7_OFFSET)
#  define TIVA_SSI0_PERIPHID0       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID0_OFFSET)
#  define TIVA_SSI0_PERIPHID1       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID1_OFFSET)
#  define TIVA_SSI0_PERIPHID2       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID2_OFFSET)
#  define TIVA_SSI0_PERIPHID3       (TIVA_SSI0_BASE + TIVA_SSI_PERIPHID3_OFFSET)
#  define TIVA_SSI0_PCELLID0        (TIVA_SSI0_BASE + TIVA_SSI_PCELLID0_OFFSET)
#  define TIVA_SSI0_PCELLID1        (TIVA_SSI0_BASE + TIVA_SSI_PCELLID1_OFFSET)
#  define TIVA_SSI0_PCELLID2        (TIVA_SSI0_BASE + TIVA_SSI_PCELLID2_OFFSET)
#  define TIVA_SSI0_PCELLID3        (TIVA_SSI0_BASE + TIVA_SSI_PCELLID3_OFFSET)
#endif

#if TIVA_NSSI > 1
#define TIVA_SSI1_CR0               (TIVA_SSI1_BASE + TIVA_SSI_CR0_OFFSET)
#define TIVA_SSI1_CR1               (TIVA_SSI1_BASE + TIVA_SSI_CR1_OFFSET)
#define TIVA_SSI1_DR                (TIVA_SSI1_BASE + TIVA_SSI_DR_OFFSET)
#define TIVA_SSI1_SR                (TIVA_SSI1_BASE + TIVA_SSI_SR_OFFSET)
#define TIVA_SSI1_CPSR              (TIVA_SSI1_BASE + TIVA_SSI_CPSR_OFFSET)
#define TIVA_SSI1_IM                (TIVA_SSI1_BASE + TIVA_SSI_IM_OFFSET)
#define TIVA_SSI1_RIS               (TIVA_SSI1_BASE + TIVA_SSI_RIS_OFFSET)
#define TIVA_SSI1_MIS               (TIVA_SSI1_BASE + TIVA_SSI_MIS_OFFSET)
#define TIVA_SSI1_ICR               (TIVA_SSI1_BASE + TIVA_SSI_ICR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_SSI1_DMACR           (TIVA_SSI1_BASE + TIVA_SSI_DMACR_OFFSET)
#else /* if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TIVA) */
#  define TIVA_SSI1_PERIPHID4       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID4_OFFSET)
#  define TIVA_SSI1_PERIPHID5       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID5_OFFSET)
#  define TIVA_SSI1_PERIPHID6       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID6_OFFSET)
#  define TIVA_SSI1_PERIPHID7       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID7_OFFSET)
#  define TIVA_SSI1_PERIPHID0       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID0_OFFSET)
#  define TIVA_SSI1_PERIPHID1       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID1_OFFSET)
#  define TIVA_SSI1_PERIPHID2       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID2_OFFSET)
#  define TIVA_SSI1_PERIPHID3       (TIVA_SSI1_BASE + TIVA_SSI_PERIPHID3_OFFSET)
#  define TIVA_SSI1_PCELLID0        (TIVA_SSI1_BASE + TIVA_SSI_PCELLID0_OFFSET)
#  define TIVA_SSI1_PCELLID1        (TIVA_SSI1_BASE + TIVA_SSI_PCELLID1_OFFSET)
#  define TIVA_SSI1_PCELLID2        (TIVA_SSI1_BASE + TIVA_SSI_PCELLID2_OFFSET)
#  define TIVA_SSI1_PCELLID3        (TIVA_SSI1_BASE + TIVA_SSI_PCELLID3_OFFSET)
#endif

#define TIVA_SSI_BASE(n)            (TIVA_SSI0_BASE + (n)*0x01000)

#define TIVA_SSI_CR0(n)             (TIVA_SSI_BASE(n) + TIVA_SSI_CR0_OFFSET)
#define TIVA_SSI_CR1(n)             (TIVA_SSI_BASE(n) + TIVA_SSI_CR1_OFFSET)
#define TIVA_SSI_DR(n)              (TIVA_SSI_BASE(n) + TIVA_SSI_DR_OFFSET)
#define TIVA_SSI_SR(n)              (TIVA_SSI_BASE(n) + TIVA_SSI_SR_OFFSET)
#define TIVA_SSI_CPSR(n)            (TIVA_SSI_BASE(n) + TIVA_SSI_CPSR_OFFSET)
#define TIVA_SSI_IM(n)              (TIVA_SSI_BASE(n) + TIVA_SSI_IM_OFFSET)
#define TIVA_SSI_RIS(n)             (TIVA_SSI_BASE(n) + TIVA_SSI_RIS_OFFSET)
#define TIVA_SSI_MIS(n)             (TIVA_SSI_BASE(n) + TIVA_SSI_MIS_OFFSET)
#define TIVA_SSI_ICR(n)             (TIVA_SSI_BASE(n) + TIVA_SSI_ICR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  define TIVA_SSI_DMACR(n)         (TIVA_SSI_BASE(n) + TIVA_SSI_DMACR_OFFSET)
#else /* if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TIVA) */
#  define TIVA_SSI_PERIPHID4(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID4_OFFSET)
#  define TIVA_SSI_PERIPHID5(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID5_OFFSET)
#  define TIVA_SSI_PERIPHID6(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID6_OFFSET)
#  define TIVA_SSI_PERIPHID7(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID7_OFFSET)
#  define TIVA_SSI_PERIPHID0(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID0_OFFSET)
#  define TIVA_SSI_PERIPHID1(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID1_OFFSET)
#  define TIVA_SSI_PERIPHID2(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID2_OFFSET)
#  define TIVA_SSI_PERIPHID3(n)     (TIVA_SSI_BASE(n) + TIVA_SSI_PERIPHID3_OFFSET)
#  define TIVA_SSI_PCELLID0(n)      (TIVA_SSI_BASE(n) + TIVA_SSI_PCELLID0_OFFSET)
#  define TIVA_SSI_PCELLID1(n)      (TIVA_SSI_BASE(n) + TIVA_SSI_PCELLID1_OFFSET)
#  define TIVA_SSI_PCELLID2(n)      (TIVA_SSI_BASE(n) + TIVA_SSI_PCELLID2_OFFSET)
#  define TIVA_SSI_PCELLID3(n)      (TIVA_SSI_BASE(n) + TIVA_SSI_PCELLID3_OFFSET)
#endif
#endif /* TIVA_NSSI > 1 */

/* SSI register bit defitiions **********************************************/

/* SSI Control 0 (SSICR0), offset 0x000 */

#define SSI_CR0_DSS_SHIFT           0                            /* Bits 3-0: SSI Data Size Select */
#define SSI_CR0_DSS_MASK            (0x0f << SSI_CR0_DSS_SHIFT)
#define   SSI_CR0_DSS(n)            ((n-1) << SSI_CR0_DSS_SHIFT) /* n={4,5,..16} */
#define SSI_CR0_FRF_SHIFT           4                            /* Bits 5-4: SSI Frame Format Select */
#define SSI_CR0_FRF_MASK            (3 << SSI_CR0_FRF_SHIFT)
#define   SSI_CR0_FRF_SPI           (0 << SSI_CR0_FRF_SHIFT)     /* Freescale SPI format */
#define   SSI_CR0_FRF_SSFF          (1 << SSI_CR0_FRF_SHIFT)     /* TI synchronous serial fram format */
#define   SSI_CR0_FRF_UWIRE         (2 << SSI_CR0_FRF_SHIFT)     /* MICROWIRE frame format */
  #define SSI_CR0_SPO               (1 << 6)                     /* Bit 6:  SSI Serial Clock Polarity */
#define SSI_CR0_SPH                 (1 << 7)                     /* Bit 7:  SSI Serial Clock Phase */
#define SSI_CR0_SCR_SHIFT           8                            /* Bits 15-8: SSI Serial Clock Rate */
#define SSI_CR0_SCR_MASK            (0xff << SSI_CR0_SCR_SHIFT)

/* SSI Control 1 (SSICR1), offset 0x004 */

#define SSI_CR1_LBM                 (1 << 0)  /* Bit 0:  SSI Loopback Mode */
#define SSI_CR1_SSE                 (1 << 1)  /* Bit 1:  SSI Synchronous Serial Port Enable */
#define SSI_CR1_MS                  (1 << 2)  /* Bit 2:  SSI Master/Slave Select slave */
#define SSI_CR1_SOD                 (1 << 3)  /* Bit 3:  SSI Slave Mode Output Disable */

/* SSI Data (SSIDR), offset 0x008 */

#define SSI_DR_MASK                 0xffff    /* Bits 15-0: SSI data */

/* SSI Status (SSISR), offset 0x00c */

#define SSI_SR_TFE                  (1 << 0)  /* Bit 0:  SSI Transmit FIFO Empty */
#define SSI_SR_TNF                  (1 << 1)  /* Bit 1:  SSI Transmit FIFO Not Full */
#define SSI_SR_RNE                  (1 << 2)  /* Bit 2:  SSI Receive FIFO Not Empty */
#define SSI_SR_RFF                  (1 << 3)  /* Bit 3:  SSI Receive FIFO Full */
#define SSI_SR_BSY                  (1 << 4)  /* Bit 4:  SSI Busy Bit */

/* SSI Clock Prescale (SSICPSR), offset 0x010 */

#define SSI_CPSR_DIV_MASK           0xff      /* Bits 7-0: SSI Clock Prescale Divisor */

/* SSI Interrupt Mask (SSIIM), offset 0x014 */

#define SSI_IM_ROR                  (1 << 0)  /* Bit 0:  SSI Receive Overrun Interrupt Mask */
#define SSI_IM_RT                   (1 << 1)  /* Bit 1:  SSI Receive Time-Out Interrupt Mask */
#define SSI_IM_RX                   (1 << 2)  /* Bit 2:  SSI Receive FIFO Interrupt Mask */
#define SSI_IM_TX                   (1 << 3)  /* Bit 3:  SSI Transmit FIFO Interrupt Mask */

/* SSI Raw Interrupt Status (SSIRIS), offset 0x018 */

#define SSI_RIS_ROR                 (1 << 0)  /* Bit 0:  SSI Receive Overrun Raw Interrupt Status */
#define SSI_RIS_RT                  (1 << 1)  /* Bit 1:  SSI Receive Time-Out Raw Interrupt Status */
#define SSI_RIS_RX                  (1 << 2)  /* Bit 2:  SSI Receive FIFO Raw Interrupt Status */
#define SSI_RIS_TX                  (1 << 3)  /* Bit 3:  SSI Transmit FIFO Raw Interrupt Status */

/* SSI Masked Interrupt Status (SSIMIS), offset 0x01c */

#define SSI_MIS_ROR                 (1 << 0)  /* Bit 0:  SSI Receive Overrun Masked Interrupt Status */
#define SSI_MIS_RT                  (1 << 1)  /* Bit 1:  SSI Receive Time-Out Masked Interrupt Status */
#define SSI_MIS_RX                  (1 << 2)  /* Bit 2:  SSI Receive FIFO Masked Interrupt Status */
#define SSI_MIS_TX                  (1 << 3)  /* Bit 3:  SSI Transmit FIFO Masked Interrupt Status */

/* SSI Interrupt Clear (SSIICR), offset 0x020 */

#define SSI_ICR_ROR                 (1 << 0)  /* Bit 0: SSI Receive Overrun Interrupt Clear */
#define SSI_ICR_RT                  (1 << 1)  /* Bit 1: SSI Receive Time-Out Interrupt Clear */

#if defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
/* SSI DMA Control */

#  define SSI_DMACR_RXDMAE          (1 << 0)  /* Bit 0: Receive DMA enable */
#  define SSI_DMACR_TXDMAE          (1 << 1)  /* Bit 1: Transmit DMA enable */
#endif

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TIVA)
/* SSI Peripheral Identification n (SSIPERIPHIDn), offset 0xfd0-0xfec */

#  define SSI_PERIPHID_MASK         0xff      /* Bits 7-0: SSI Peripheral ID n */

/* SSI PrimeCell Identification n (SSIPCELLIDn), offset 0xff0-0xffc */

#  define SSI_PCELLID_MASK          0xff      /* Bits 7-0: SSI Prime cell ID */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* TIVA_NSSI > 0 */
#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_SSI_H */
