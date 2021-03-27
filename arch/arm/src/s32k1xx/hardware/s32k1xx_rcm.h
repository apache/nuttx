/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_rcm.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RCM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCM Register Offsets *****************************************************/

#define S32K1XX_RCM_VERID_OFFSET    0x0000  /* Version ID Register */
#define S32K1XX_RCM_PARAM_OFFSET    0x0004  /* Parameter Register */
#define S32K1XX_RCM_SRS_OFFSET      0x0008  /* System Reset Status Register */
#define S32K1XX_RCM_RPC_OFFSET      0x000c  /* Reset Pin Control register */
#define S32K1XX_RCM_SSRS_OFFSET     0x0018  /* Sticky System Reset Status Register */
#define S32K1XX_RCM_SRIE_OFFSET     0x001c  /* System Reset Interrupt Enable Register */

/* RCM Register Addresses ***************************************************/

#define S32K1XX_RCM_VERID           (S32K1XX_RCM_BASE + S32K1XX_RCM_VERID_OFFSET)
#define S32K1XX_RCM_PARAM           (S32K1XX_RCM_BASE + S32K1XX_RCM_PARAM_OFFSET)
#define S32K1XX_RCM_SRS             (S32K1XX_RCM_BASE + S32K1XX_RCM_SRS_OFFSET)
#define S32K1XX_RCM_RPC             (S32K1XX_RCM_BASE + S32K1XX_RCM_RPC_OFFSET)
#define S32K1XX_RCM_SSRS            (S32K1XX_RCM_BASE + S32K1XX_RCM_SSRS_OFFSET)
#define S32K1XX_RCM_SRIE            (S32K1XX_RCM_BASE + S32K1XX_RCM_SRIE_OFFSET)

/* RCM Register Bitfield Definitions ****************************************/

/* Version ID Register */

#define RCM_VERID_FEATURE_SHIFT     (0)       /* Bits 0-15:  Feature Specification Number */
#define RCM_VERID_FEATURE_MASK      (0xffff << RCM_VERID_FEATURE_SHIFT)
#  define RCM_VERID_FEATURE_STD     (3 << RCM_VERID_FEATURE_SHIFT) /* Standard feature set*/

#define RCM_VERID_MINOR_SHIFT       (16)      /* Bits 16-23:  Minor Version Number */
#define RCM_VERID_MINOR_MASK        (0xff << RCM_VERID_MINOR_SHIFT)
#define RCM_VERID_MAJOR_SHIFT       (24)      /* Bits 24-31: Major Version Number */
#define RCM_VERID_MAJOR_MASK        (0xff << RCM_VERID_MAJOR_SHIFT)

/* Parameter Register */

#define RCM_PARAM_EWAKEUP           (1 << 0)  /* Bit 0:  Existence of SRS[WAKEUP] status indication */
#define RCM_PARAM_ELVD              (1 << 1)  /* Bit 1:  Existence of SRS[LVD] status indication */
#define RCM_PARAM_ELOC              (1 << 2)  /* Bit 2:  Existence of SRS[LOC] status indication */
#define RCM_PARAM_ELOL              (1 << 3)  /* Bit 3:  Existence of SRS[LOL] status indication */
#define RCM_PARAM_ECMU_LOC          (1 << 4)  /* Bit 4:  Existence of SRS[CMU_LOC] status indication */
#define RCM_PARAM_EWDOG             (1 << 5)  /* Bit 5:  Existence of SRS[WDOG] status indication */
#define RCM_PARAM_EPIN              (1 << 6)  /* Bit 6:  Existence of SRS[PIN] status indication */
#define RCM_PARAM_EPOR              (1 << 7)  /* Bit 7:  Existence of SRS[POR] status indication */
#define RCM_PARAM_EJTAG             (1 << 8)  /* Bit 8:  Existence of SRS[JTAG] status indication */
#define RCM_PARAM_ELOCKUP           (1 << 9)  /* Bit 9:  Existence of SRS[LOCKUP] status indication */
#define RCM_PARAM_ESW               (1 << 10) /* Bit 10: Existence of SRS[SW] status indication */
#define RCM_PARAM_EMDM_AP           (1 << 11) /* Bit 11: Existence of SRS[MDM_AP] status indication */
#define RCM_PARAM_ESACKERR          (1 << 13) /* Bit 13: Existence of SRS[SACKERR] status indication */
#define RCM_PARAM_ETAMPER           (1 << 15) /* Bit 15: Existence of SRS[TAMPER] status indication */
#define RCM_PARAM_ECORE1            (1 << 16) /* Bit 16: Existence of SRS[CORE1] status indication */

/* System Reset Status Register */

#define RCM_SRS_LVD                 (1 << 1)  /* Bit 1:  Low-Voltage Detect Reset or High-Voltage Detect Reset */
#define RCM_SRS_LOC                 (1 << 2)  /* Bit 2:  Loss-of-Clock Reset */
#define RCM_SRS_LOL                 (1 << 3)  /* Bit 3:  Loss-of-Lock Reset */
#define RCM_SRS_CMU_LOC             (1 << 4)  /* Bit 4:  CMU Loss-of-Clock Reset */
#define RCM_SRS_WDOG                (1 << 5)  /* Bit 5:  Watchdog */
#define RCM_SRS_PIN                 (1 << 6)  /* Bit 6:  External Reset Pin */
#define RCM_SRS_POR                 (1 << 7)  /* Bit 7:  Power-On Reset */
#define RCM_SRS_JTAG                (1 << 8)  /* Bit 8:  JTAG generated reset */
#define RCM_SRS_LOCKUP              (1 << 9)  /* Bit 9:  Core Lockup */
#define RCM_SRS_SW                  (1 << 10) /* Bit 10: Software */
#define RCM_SRS_MDM_AP              (1 << 11) /* Bit 11: MDM-AP System Reset Request */
#define RCM_SRS_SACKERR             (1 << 13) /* Bit 13: Stop Acknowledge Error */

/* Reset Pin Control register */

#define RCM_RPC_RSTFLTSRW_SHIFT     (0)       /* Bits 0-1: Reset Pin Filter Select in Run and Wait Modes */
#define RCM_RPC_RSTFLTSRW_MASK      (3 << RCM_RPC_RSTFLTSRW_SHIFT)
#  define RCM_RPC_RSTFLTSRW_DISABLE (0 << RCM_RPC_RSTFLTSRW_SHIFT) /* All filtering disabled */
#  define RCM_RPC_RSTFLTSRW_BUSCLK  (1 << RCM_RPC_RSTFLTSRW_SHIFT) /* Bus clock filter enabled for normal operation */
#  define RCM_RPC_RSTFLTSRW_LPOCLK  (2 << RCM_RPC_RSTFLTSRW_SHIFT) /* LPO clock filter enabled for normal operation */

#define RCM_RPC_RSTFLTSS            (1 << 2)  /* Bit 2:  Reset Pin Filter Select in Stop Mode */
#define RCM_RPC_RSTFLTSEL_SHIFT     (8)       /* Bits 8-12: Reset Pin Filter Bus Clock Select */
#define RCM_RPC_RSTFLTSEL_MASK      (31 << RCM_RPC_RSTFLTSEL_SHIFT)
#  define RCM_RPC_RSTFLTSEL(n)      ((uint32_t)(n) << RCM_RPC_RSTFLTSEL_SHIFT)

/* Sticky System Reset Status Register */

#define RCM_SSRS_LVD                (1 << 1)  /* Bit 1:  Sticky Low-Voltage Detect Reset */
#define RCM_SSRS_LOC                (1 << 2)  /* Bit 2:  Sticky Loss-of-Clock Reset */
#define RCM_SSRS_LOL                (1 << 3)  /* Bit 3:  Sticky Loss-of-Lock Reset */
#define RCM_SSRS_CMU_LOC            (1 << 4)  /* Bit 4:  Sticky CMU Loss-of-Clock Reset */
#define RCM_SSRS_WDOG               (1 << 5)  /* Bit 5:  Sticky Watchdog */
#define RCM_SSRS_PIN                (1 << 6)  /* Bit 6:  Sticky External Reset Pin */
#define RCM_SSRS_POR                (1 << 7)  /* Bit 7:  Sticky Power-On Reset */
#define RCM_SSRS_JTAG               (1 << 8)  /* Bit 8:  Sticky JTAG generated reset */
#define RCM_SSRS_LOCKUP             (1 << 9)  /* Bit 9:  Sticky Core Lockup */
#define RCM_SSRS_SW                 (1 << 10) /* Bit 10: Sticky Software */
#define RCM_SSRS_MDM_AP             (1 << 11) /* Bit 11: Sticky MDM-AP System Reset Request */
#define RCM_SSRS_SACKERR            (1 << 13) /* Bit 13: Sticky Stop Acknowledge Error */

/* System Reset Interrupt Enable Register */

#define RCM_SRIE_DELAY_SHIFT        (0)       /* Bits 0-1:  Reset Delay Time */
#define RCM_SRIE_DELAY_MASK         (3 << RCM_SRIE_DELAY_SHIFT)
#  define RCM_SRIE_DELAY_10         (0 << RCM_SRIE_DELAY_SHIFT) /* 10 LPO cycles */
#  define RCM_SRIE_DELAY_34         (1 << RCM_SRIE_DELAY_SHIFT) /* 34 LPO cycles */
#  define RCM_SRIE_DELAY_130        (2 << RCM_SRIE_DELAY_SHIFT) /* 130 LPO cycles */
#  define RCM_SRIE_DELAY_514        (3 << RCM_SRIE_DELAY_SHIFT) /* 514 LPO cycles */

#define RCM_SRIE_LOC                (1 << 2)  /* Bit 2:  Loss-of-Clock Interrupt */
#define RCM_SRIE_LOL                (1 << 3)  /* Bit 3:  Loss-of-Lock Interrupt */
#define RCM_SRIE_CMU_LOC            (1 << 4)  /* Bit 4:  CMU Loss-of-Clock Reset Interrupt */
#define RCM_SRIE_WDOG               (1 << 5)  /* Bit 5:  Watchdog Interrupt */
#define RCM_SRIE_PIN                (1 << 6)  /* Bit 6:  External Reset Pin Interrupt */
#define RCM_SRIE_GIE                (1 << 7)  /* Bit 7:  Global Interrupt Enable */
#define RCM_SRIE_JTAG               (1 << 8)  /* Bit 8:  JTAG generated reset */
#define RCM_SRIE_LOCKUP             (1 << 9)  /* Bit 9:  Core Lockup Interrupt */
#define RCM_SRIE_SW                 (1 << 10) /* Bit 10: Software Interrupt */
#define RCM_SRIE_MDM_AP             (1 << 11) /* Bit 11: MDM-AP System Reset Request */
#define RCM_SRIE_SACKERR            (1 << 13) /* Bit 13: Stop Acknowledge Error  */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RCM_H */
