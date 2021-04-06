/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_flexspi.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXSPI_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* FLEXSPI - Register Layout Typedef */

struct flexspi_type_s
{
  volatile uint32_t MCR0;                              /* Module Control Register 0, offset: 0x0 */
  volatile uint32_t MCR1;                              /* Module Control Register 1, offset: 0x4 */
  volatile uint32_t MCR2;                              /* Module Control Register 2, offset: 0x8 */
  volatile uint32_t AHBCR;                             /* AHB Bus Control Register, offset: 0xc */
  volatile uint32_t INTEN;                             /* Interrupt Enable Register, offset: 0x10 */
  volatile uint32_t INTR;                              /* Interrupt Register, offset: 0x14 */
  volatile uint32_t LUTKEY;                            /* LUT Key Register, offset: 0x18 */
  volatile uint32_t LUTCR;                             /* LUT Control Register, offset: 0x1c */
  volatile uint32_t AHBRXBUFCR0[4];                    /* AHB RX Buffer 0 Control Register 0..AHB RX Buffer 3 Control Register 0, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_0[48];
  volatile uint32_t FLSHCR0[4];                        /* Flash A1 Control Register 0..Flash B2 Control Register 0, array offset: 0x60, array step: 0x4 */
  volatile uint32_t FLSHCR1[4];                        /* Flash A1 Control Register 1..Flash B2 Control Register 1, array offset: 0x70, array step: 0x4 */
  volatile uint32_t FLSHCR2[4];                        /* Flash A1 Control Register 2..Flash B2 Control Register 2, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_1[4];
  volatile uint32_t FLSHCR4;                           /* Flash Control Register 4, offset: 0x94 */
       uint8_t RESERVED_2[8];
  volatile uint32_t IPCR0;                             /* IP Control Register 0, offset: 0xa0 */
  volatile uint32_t IPCR1;                             /* IP Control Register 1, offset: 0xa4 */
       uint8_t RESERVED_3[8];
  volatile uint32_t IPCMD;                             /* IP Command Register, offset: 0xb0 */
       uint8_t RESERVED_4[4];
  volatile uint32_t IPRXFCR;                           /* IP RX FIFO Control Register, offset: 0xb8 */
  volatile uint32_t IPTXFCR;                           /* IP TX FIFO Control Register, offset: 0xbc */
  volatile uint32_t DLLCR[2];                          /* DLL Control Register 0, array offset: 0xc0, array step: 0x4 */
       uint8_t RESERVED_5[24];
  volatile  uint32_t STS0;                              /* Status Register 0, offset: 0xe0 */
  volatile  uint32_t STS1;                              /* Status Register 1, offset: 0xe4 */
  volatile  uint32_t STS2;                              /* Status Register 2, offset: 0xe8 */
  volatile  uint32_t AHBSPNDSTS;                        /* AHB Suspend Status Register, offset: 0xec */
  volatile  uint32_t IPRXFSTS;                          /* IP RX FIFO Status Register, offset: 0xf0 */
  volatile  uint32_t IPTXFSTS;                          /* IP TX FIFO Status Register, offset: 0xf4 */
       uint8_t RESERVED_6[8];
  volatile  uint32_t RFDR[32];                          /* IP RX FIFO Data Register 0..IP RX FIFO Data Register 31, array offset: 0x100, array step: 0x4 */
  volatile  uint32_t TFDR[32];                          /* IP TX FIFO Data Register 0..IP TX FIFO Data Register 31, array offset: 0x180, array step: 0x4 */
  volatile uint32_t LUT[64];                            /* LUT 0..LUT 63, array offset: 0x200, array step: 0x4 */
};

/* MCR0 - Module Control Register 0 */

#define FLEXSPI_MCR0_SWRESET_MASK                (0x1u)
#define FLEXSPI_MCR0_SWRESET_SHIFT               (0u)

/* SWRESET - Software Reset */

#define FLEXSPI_MCR0_SWRESET(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_SWRESET_SHIFT)) & FLEXSPI_MCR0_SWRESET_MASK)
#define FLEXSPI_MCR0_MDIS_MASK                   (0x2u)
#define FLEXSPI_MCR0_MDIS_SHIFT                  (1u)

/* MDIS - Module Disable */

#define FLEXSPI_MCR0_MDIS(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_MDIS_SHIFT)) & FLEXSPI_MCR0_MDIS_MASK)
#define FLEXSPI_MCR0_RXCLKSRC_MASK               (0x30u)
#define FLEXSPI_MCR0_RXCLKSRC_SHIFT              (4u)

#define FLEXSPI_MCR0_RXCLKSRC(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_RXCLKSRC_SHIFT)) & FLEXSPI_MCR0_RXCLKSRC_MASK)
#define FLEXSPI_MCR0_ARDFEN_MASK                 (0x40u)
#define FLEXSPI_MCR0_ARDFEN_SHIFT                (6u)

#define FLEXSPI_MCR0_ARDFEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_ARDFEN_SHIFT)) & FLEXSPI_MCR0_ARDFEN_MASK)
#define FLEXSPI_MCR0_ATDFEN_MASK                 (0x80u)
#define FLEXSPI_MCR0_ATDFEN_SHIFT                (7u)

#define FLEXSPI_MCR0_ATDFEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_ATDFEN_SHIFT)) & FLEXSPI_MCR0_ATDFEN_MASK)
#define FLEXSPI_MCR0_HSEN_MASK                   (0x800u)
#define FLEXSPI_MCR0_HSEN_SHIFT                  (11u)

#define FLEXSPI_MCR0_HSEN(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_HSEN_SHIFT)) & FLEXSPI_MCR0_HSEN_MASK)
#define FLEXSPI_MCR0_DOZEEN_MASK                 (0x1000u)
#define FLEXSPI_MCR0_DOZEEN_SHIFT                (12u)

#define FLEXSPI_MCR0_DOZEEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_DOZEEN_SHIFT)) & FLEXSPI_MCR0_DOZEEN_MASK)
#define FLEXSPI_MCR0_COMBINATIONEN_MASK          (0x2000u)
#define FLEXSPI_MCR0_COMBINATIONEN_SHIFT         (13u)

#define FLEXSPI_MCR0_COMBINATIONEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_COMBINATIONEN_SHIFT)) & FLEXSPI_MCR0_COMBINATIONEN_MASK)
#define FLEXSPI_MCR0_SCKFREERUNEN_MASK           (0x4000u)
#define FLEXSPI_MCR0_SCKFREERUNEN_SHIFT          (14u)

#define FLEXSPI_MCR0_SCKFREERUNEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_SCKFREERUNEN_SHIFT)) & FLEXSPI_MCR0_SCKFREERUNEN_MASK)
#define FLEXSPI_MCR0_IPGRANTWAIT_MASK            (0xff0000u)
#define FLEXSPI_MCR0_IPGRANTWAIT_SHIFT           (16u)

#define FLEXSPI_MCR0_IPGRANTWAIT(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_IPGRANTWAIT_SHIFT)) & FLEXSPI_MCR0_IPGRANTWAIT_MASK)
#define FLEXSPI_MCR0_AHBGRANTWAIT_MASK           (0xff000000u)
#define FLEXSPI_MCR0_AHBGRANTWAIT_SHIFT          (24u)

#define FLEXSPI_MCR0_AHBGRANTWAIT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_AHBGRANTWAIT_SHIFT)) & FLEXSPI_MCR0_AHBGRANTWAIT_MASK)

#define FLEXSPI_MCR1_AHBBUSWAIT_MASK             (0xffffu)
#define FLEXSPI_MCR1_AHBBUSWAIT_SHIFT            (0u)
#define FLEXSPI_MCR1_AHBBUSWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR1_AHBBUSWAIT_SHIFT)) & FLEXSPI_MCR1_AHBBUSWAIT_MASK)
#define FLEXSPI_MCR1_SEQWAIT_MASK                (0xffff0000u)
#define FLEXSPI_MCR1_SEQWAIT_SHIFT               (16u)
#define FLEXSPI_MCR1_SEQWAIT(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR1_SEQWAIT_SHIFT)) & FLEXSPI_MCR1_SEQWAIT_MASK)

#define FLEXSPI_MCR2_CLRAHBBUFOPT_MASK           (0x800u)
#define FLEXSPI_MCR2_CLRAHBBUFOPT_SHIFT          (11u)

#define FLEXSPI_MCR2_CLRAHBBUFOPT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_CLRAHBBUFOPT_SHIFT)) & FLEXSPI_MCR2_CLRAHBBUFOPT_MASK)
#define FLEXSPI_MCR2_CLRLEARNPHASE_MASK          (0x4000u)
#define FLEXSPI_MCR2_CLRLEARNPHASE_SHIFT         (14u)

#define FLEXSPI_MCR2_CLRLEARNPHASE(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_CLRLEARNPHASE_SHIFT)) & FLEXSPI_MCR2_CLRLEARNPHASE_MASK)
#define FLEXSPI_MCR2_SAMEDEVICEEN_MASK           (0x8000u)
#define FLEXSPI_MCR2_SAMEDEVICEEN_SHIFT          (15u)

#define FLEXSPI_MCR2_SAMEDEVICEEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_SAMEDEVICEEN_SHIFT)) & FLEXSPI_MCR2_SAMEDEVICEEN_MASK)
#define FLEXSPI_MCR2_SCKBDIFFOPT_MASK            (0x80000u)
#define FLEXSPI_MCR2_SCKBDIFFOPT_SHIFT           (19u)

#define FLEXSPI_MCR2_SCKBDIFFOPT(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_SCKBDIFFOPT_SHIFT)) & FLEXSPI_MCR2_SCKBDIFFOPT_MASK)
#define FLEXSPI_MCR2_RESUMEWAIT_MASK             (0xff000000u)
#define FLEXSPI_MCR2_RESUMEWAIT_SHIFT            (24u)

#define FLEXSPI_MCR2_RESUMEWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_RESUMEWAIT_SHIFT)) & FLEXSPI_MCR2_RESUMEWAIT_MASK)

#define FLEXSPI_AHBCR_APAREN_MASK                (0x1u)
#define FLEXSPI_AHBCR_APAREN_SHIFT               (0u)

#define FLEXSPI_AHBCR_APAREN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_APAREN_SHIFT)) & FLEXSPI_AHBCR_APAREN_MASK)
#define FLEXSPI_AHBCR_CACHABLEEN_MASK            (0x8u)
#define FLEXSPI_AHBCR_CACHABLEEN_SHIFT           (3u)

#define FLEXSPI_AHBCR_CACHABLEEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_CACHABLEEN_SHIFT)) & FLEXSPI_AHBCR_CACHABLEEN_MASK)
#define FLEXSPI_AHBCR_BUFFERABLEEN_MASK          (0x10u)
#define FLEXSPI_AHBCR_BUFFERABLEEN_SHIFT         (4u)

#define FLEXSPI_AHBCR_BUFFERABLEEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_BUFFERABLEEN_SHIFT)) & FLEXSPI_AHBCR_BUFFERABLEEN_MASK)
#define FLEXSPI_AHBCR_PREFETCHEN_MASK            (0x20u)
#define FLEXSPI_AHBCR_PREFETCHEN_SHIFT           (5u)

#define FLEXSPI_AHBCR_PREFETCHEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_PREFETCHEN_SHIFT)) & FLEXSPI_AHBCR_PREFETCHEN_MASK)
#define FLEXSPI_AHBCR_READADDROPT_MASK           (0x40u)
#define FLEXSPI_AHBCR_READADDROPT_SHIFT          (6u)

#define FLEXSPI_AHBCR_READADDROPT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_READADDROPT_SHIFT)) & FLEXSPI_AHBCR_READADDROPT_MASK)

#define FLEXSPI_INTEN_IPCMDDONEEN_MASK           (0x1u)
#define FLEXSPI_INTEN_IPCMDDONEEN_SHIFT          (0u)

#define FLEXSPI_INTEN_IPCMDDONEEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDDONEEN_SHIFT)) & FLEXSPI_INTEN_IPCMDDONEEN_MASK)
#define FLEXSPI_INTEN_IPCMDGEEN_MASK             (0x2u)
#define FLEXSPI_INTEN_IPCMDGEEN_SHIFT            (1u)

#define FLEXSPI_INTEN_IPCMDGEEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDGEEN_SHIFT)) & FLEXSPI_INTEN_IPCMDGEEN_MASK)
#define FLEXSPI_INTEN_AHBCMDGEEN_MASK            (0x4u)
#define FLEXSPI_INTEN_AHBCMDGEEN_SHIFT           (2u)

#define FLEXSPI_INTEN_AHBCMDGEEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBCMDGEEN_SHIFT)) & FLEXSPI_INTEN_AHBCMDGEEN_MASK)
#define FLEXSPI_INTEN_IPCMDERREN_MASK            (0x8u)
#define FLEXSPI_INTEN_IPCMDERREN_SHIFT           (3u)

#define FLEXSPI_INTEN_IPCMDERREN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDERREN_SHIFT)) & FLEXSPI_INTEN_IPCMDERREN_MASK)
#define FLEXSPI_INTEN_AHBCMDERREN_MASK           (0x10u)
#define FLEXSPI_INTEN_AHBCMDERREN_SHIFT          (4u)

#define FLEXSPI_INTEN_AHBCMDERREN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBCMDERREN_SHIFT)) & FLEXSPI_INTEN_AHBCMDERREN_MASK)
#define FLEXSPI_INTEN_IPRXWAEN_MASK              (0x20u)
#define FLEXSPI_INTEN_IPRXWAEN_SHIFT             (5u)

#define FLEXSPI_INTEN_IPRXWAEN(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPRXWAEN_SHIFT)) & FLEXSPI_INTEN_IPRXWAEN_MASK)
#define FLEXSPI_INTEN_IPTXWEEN_MASK              (0x40u)
#define FLEXSPI_INTEN_IPTXWEEN_SHIFT             (6u)

#define FLEXSPI_INTEN_IPTXWEEN(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPTXWEEN_SHIFT)) & FLEXSPI_INTEN_IPTXWEEN_MASK)
#define FLEXSPI_INTEN_SCKSTOPBYRDEN_MASK         (0x100u)
#define FLEXSPI_INTEN_SCKSTOPBYRDEN_SHIFT        (8u)

#define FLEXSPI_INTEN_SCKSTOPBYRDEN(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SCKSTOPBYRDEN_SHIFT)) & FLEXSPI_INTEN_SCKSTOPBYRDEN_MASK)
#define FLEXSPI_INTEN_SCKSTOPBYWREN_MASK         (0x200u)
#define FLEXSPI_INTEN_SCKSTOPBYWREN_SHIFT        (9u)

#define FLEXSPI_INTEN_SCKSTOPBYWREN(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SCKSTOPBYWREN_SHIFT)) & FLEXSPI_INTEN_SCKSTOPBYWREN_MASK)
#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN_MASK       (0x400u)
#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN_SHIFT      (10u)

#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN(x)         (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBBUSTIMEOUTEN_SHIFT)) & FLEXSPI_INTEN_AHBBUSTIMEOUTEN_MASK)
#define FLEXSPI_INTEN_SEQTIMEOUTEN_MASK          (0x800u)
#define FLEXSPI_INTEN_SEQTIMEOUTEN_SHIFT         (11u)

#define FLEXSPI_INTEN_SEQTIMEOUTEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SEQTIMEOUTEN_SHIFT)) & FLEXSPI_INTEN_SEQTIMEOUTEN_MASK)

#define FLEXSPI_INTR_IPCMDDONE_MASK              (0x1u)
#define FLEXSPI_INTR_IPCMDDONE_SHIFT             (0u)

#define FLEXSPI_INTR_IPCMDDONE(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDDONE_SHIFT)) & FLEXSPI_INTR_IPCMDDONE_MASK)
#define FLEXSPI_INTR_IPCMDGE_MASK                (0x2u)
#define FLEXSPI_INTR_IPCMDGE_SHIFT               (1u)

#define FLEXSPI_INTR_IPCMDGE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDGE_SHIFT)) & FLEXSPI_INTR_IPCMDGE_MASK)
#define FLEXSPI_INTR_AHBCMDGE_MASK               (0x4u)
#define FLEXSPI_INTR_AHBCMDGE_SHIFT              (2u)

#define FLEXSPI_INTR_AHBCMDGE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBCMDGE_SHIFT)) & FLEXSPI_INTR_AHBCMDGE_MASK)
#define FLEXSPI_INTR_IPCMDERR_MASK               (0x8u)
#define FLEXSPI_INTR_IPCMDERR_SHIFT              (3u)

#define FLEXSPI_INTR_IPCMDERR(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDERR_SHIFT)) & FLEXSPI_INTR_IPCMDERR_MASK)
#define FLEXSPI_INTR_AHBCMDERR_MASK              (0x10u)
#define FLEXSPI_INTR_AHBCMDERR_SHIFT             (4u)

#define FLEXSPI_INTR_AHBCMDERR(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBCMDERR_SHIFT)) & FLEXSPI_INTR_AHBCMDERR_MASK)
#define FLEXSPI_INTR_IPRXWA_MASK                 (0x20u)
#define FLEXSPI_INTR_IPRXWA_SHIFT                (5u)

#define FLEXSPI_INTR_IPRXWA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPRXWA_SHIFT)) & FLEXSPI_INTR_IPRXWA_MASK)
#define FLEXSPI_INTR_IPTXWE_MASK                 (0x40u)
#define FLEXSPI_INTR_IPTXWE_SHIFT                (6u)

#define FLEXSPI_INTR_IPTXWE(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPTXWE_SHIFT)) & FLEXSPI_INTR_IPTXWE_MASK)
#define FLEXSPI_INTR_SCKSTOPBYRD_MASK            (0x100u)
#define FLEXSPI_INTR_SCKSTOPBYRD_SHIFT           (8u)

#define FLEXSPI_INTR_SCKSTOPBYRD(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SCKSTOPBYRD_SHIFT)) & FLEXSPI_INTR_SCKSTOPBYRD_MASK)
#define FLEXSPI_INTR_SCKSTOPBYWR_MASK            (0x200u)
#define FLEXSPI_INTR_SCKSTOPBYWR_SHIFT           (9u)

#define FLEXSPI_INTR_SCKSTOPBYWR(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SCKSTOPBYWR_SHIFT)) & FLEXSPI_INTR_SCKSTOPBYWR_MASK)
#define FLEXSPI_INTR_AHBBUSTIMEOUT_MASK          (0x400u)
#define FLEXSPI_INTR_AHBBUSTIMEOUT_SHIFT         (10u)

#define FLEXSPI_INTR_AHBBUSTIMEOUT(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBBUSTIMEOUT_SHIFT)) & FLEXSPI_INTR_AHBBUSTIMEOUT_MASK)
#define FLEXSPI_INTR_SEQTIMEOUT_MASK             (0x800u)
#define FLEXSPI_INTR_SEQTIMEOUT_SHIFT            (11u)

#define FLEXSPI_INTR_SEQTIMEOUT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SEQTIMEOUT_SHIFT)) & FLEXSPI_INTR_SEQTIMEOUT_MASK)

#define FLEXSPI_LUTKEY_KEY_MASK                  (0xffffffffu)
#define FLEXSPI_LUTKEY_KEY_SHIFT                 (0u)

#define FLEXSPI_LUTKEY_KEY(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTKEY_KEY_SHIFT)) & FLEXSPI_LUTKEY_KEY_MASK)

#define FLEXSPI_LUTCR_LOCK_MASK                  (0x1u)
#define FLEXSPI_LUTCR_LOCK_SHIFT                 (0u)

#define FLEXSPI_LUTCR_LOCK(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTCR_LOCK_SHIFT)) & FLEXSPI_LUTCR_LOCK_MASK)
#define FLEXSPI_LUTCR_UNLOCK_MASK                (0x2u)
#define FLEXSPI_LUTCR_UNLOCK_SHIFT               (1u)

#define FLEXSPI_LUTCR_UNLOCK(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTCR_UNLOCK_SHIFT)) & FLEXSPI_LUTCR_UNLOCK_MASK)

#define FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK           (0xffu)
#define FLEXSPI_AHBRXBUFCR0_BUFSZ_SHIFT          (0u)

#define FLEXSPI_AHBRXBUFCR0_BUFSZ(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_BUFSZ_SHIFT)) & FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK)
#define FLEXSPI_AHBRXBUFCR0_MSTRID_MASK          (0xf0000u)
#define FLEXSPI_AHBRXBUFCR0_MSTRID_SHIFT         (16u)

#define FLEXSPI_AHBRXBUFCR0_MSTRID(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_MSTRID_SHIFT)) & FLEXSPI_AHBRXBUFCR0_MSTRID_MASK)
#define FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK        (0x3000000u)
#define FLEXSPI_AHBRXBUFCR0_PRIORITY_SHIFT       (24u)

#define FLEXSPI_AHBRXBUFCR0_PRIORITY(x)          (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_PRIORITY_SHIFT)) & FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK)
#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK      (0x80000000u)
#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN_SHIFT     (31u)

#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN(x)        (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_PREFETCHEN_SHIFT)) & FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK)

#define FLEXSPI_AHBRXBUFCR0_COUNT                (4u)

#define FLEXSPI_FLSHCR0_FLSHSZ_MASK              (0x7fffffu)
#define FLEXSPI_FLSHCR0_FLSHSZ_SHIFT             (0u)

#define FLEXSPI_FLSHCR0_FLSHSZ(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR0_FLSHSZ_SHIFT)) & FLEXSPI_FLSHCR0_FLSHSZ_MASK)

#define FLEXSPI_FLSHCR0_COUNT                    (4u)

#define FLEXSPI_FLSHCR1_TCSS_MASK                (0x1fu)
#define FLEXSPI_FLSHCR1_TCSS_SHIFT               (0u)

#define FLEXSPI_FLSHCR1_TCSS(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_TCSS_SHIFT)) & FLEXSPI_FLSHCR1_TCSS_MASK)
#define FLEXSPI_FLSHCR1_TCSH_MASK                (0x3e0u)
#define FLEXSPI_FLSHCR1_TCSH_SHIFT               (5u)

#define FLEXSPI_FLSHCR1_TCSH(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_TCSH_SHIFT)) & FLEXSPI_FLSHCR1_TCSH_MASK)
#define FLEXSPI_FLSHCR1_WA_MASK                  (0x400u)
#define FLEXSPI_FLSHCR1_WA_SHIFT                 (10u)

#define FLEXSPI_FLSHCR1_WA(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_WA_SHIFT)) & FLEXSPI_FLSHCR1_WA_MASK)
#define FLEXSPI_FLSHCR1_CAS_MASK                 (0x7800u)
#define FLEXSPI_FLSHCR1_CAS_SHIFT                (11u)

#define FLEXSPI_FLSHCR1_CAS(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CAS_SHIFT)) & FLEXSPI_FLSHCR1_CAS_MASK)
#define FLEXSPI_FLSHCR1_CSINTERVALUNIT_MASK      (0x8000u)
#define FLEXSPI_FLSHCR1_CSINTERVALUNIT_SHIFT     (15u)

#define FLEXSPI_FLSHCR1_CSINTERVALUNIT(x)        (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CSINTERVALUNIT_SHIFT)) & FLEXSPI_FLSHCR1_CSINTERVALUNIT_MASK)
#define FLEXSPI_FLSHCR1_CSINTERVAL_MASK          (0xffff0000u)
#define FLEXSPI_FLSHCR1_CSINTERVAL_SHIFT         (16u)

#define FLEXSPI_FLSHCR1_CSINTERVAL(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CSINTERVAL_SHIFT)) & FLEXSPI_FLSHCR1_CSINTERVAL_MASK)

#define FLEXSPI_FLSHCR1_COUNT                    (4u)

#define FLEXSPI_FLSHCR2_ARDSEQID_MASK            (0xfu)
#define FLEXSPI_FLSHCR2_ARDSEQID_SHIFT           (0u)

#define FLEXSPI_FLSHCR2_ARDSEQID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_ARDSEQID_SHIFT)) & FLEXSPI_FLSHCR2_ARDSEQID_MASK)
#define FLEXSPI_FLSHCR2_ARDSEQNUM_MASK           (0xe0u)
#define FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT          (5u)

#define FLEXSPI_FLSHCR2_ARDSEQNUM(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT)) & FLEXSPI_FLSHCR2_ARDSEQNUM_MASK)
#define FLEXSPI_FLSHCR2_AWRSEQID_MASK            (0xf00u)
#define FLEXSPI_FLSHCR2_AWRSEQID_SHIFT           (8u)

#define FLEXSPI_FLSHCR2_AWRSEQID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRSEQID_SHIFT)) & FLEXSPI_FLSHCR2_AWRSEQID_MASK)
#define FLEXSPI_FLSHCR2_AWRSEQNUM_MASK           (0xe000u)
#define FLEXSPI_FLSHCR2_AWRSEQNUM_SHIFT          (13u)

#define FLEXSPI_FLSHCR2_AWRSEQNUM(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRSEQNUM_SHIFT)) & FLEXSPI_FLSHCR2_AWRSEQNUM_MASK)
#define FLEXSPI_FLSHCR2_AWRWAIT_MASK             (0xfff0000u)
#define FLEXSPI_FLSHCR2_AWRWAIT_SHIFT            (16u)
#define FLEXSPI_FLSHCR2_AWRWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRWAIT_SHIFT)) & FLEXSPI_FLSHCR2_AWRWAIT_MASK)
#define FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK         (0x70000000u)
#define FLEXSPI_FLSHCR2_AWRWAITUNIT_SHIFT        (28u)

#define FLEXSPI_FLSHCR2_AWRWAITUNIT(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRWAITUNIT_SHIFT)) & FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK)
#define FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK         (0x80000000u)
#define FLEXSPI_FLSHCR2_CLRINSTRPTR_SHIFT        (31u)

#define FLEXSPI_FLSHCR2_CLRINSTRPTR(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_CLRINSTRPTR_SHIFT)) & FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK)

#define FLEXSPI_FLSHCR2_COUNT                    (4u)

#define FLEXSPI_FLSHCR4_WMOPT1_MASK              (0x1u)
#define FLEXSPI_FLSHCR4_WMOPT1_SHIFT             (0u)

#define FLEXSPI_FLSHCR4_WMOPT1(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMOPT1_SHIFT)) & FLEXSPI_FLSHCR4_WMOPT1_MASK)
#define FLEXSPI_FLSHCR4_WMENA_MASK               (0x4u)
#define FLEXSPI_FLSHCR4_WMENA_SHIFT              (2u)

#define FLEXSPI_FLSHCR4_WMENA(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMENA_SHIFT)) & FLEXSPI_FLSHCR4_WMENA_MASK)
#define FLEXSPI_FLSHCR4_WMENB_MASK               (0x8u)
#define FLEXSPI_FLSHCR4_WMENB_SHIFT              (3u)

#define FLEXSPI_FLSHCR4_WMENB(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMENB_SHIFT)) & FLEXSPI_FLSHCR4_WMENB_MASK)

#define FLEXSPI_IPCR0_SFAR_MASK                  (0xffffffffu)
#define FLEXSPI_IPCR0_SFAR_SHIFT                 (0u)

#define FLEXSPI_IPCR0_SFAR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR0_SFAR_SHIFT)) & FLEXSPI_IPCR0_SFAR_MASK)

#define FLEXSPI_IPCR1_IDATSZ_MASK                (0xffffu)
#define FLEXSPI_IPCR1_IDATSZ_SHIFT               (0u)

#define FLEXSPI_IPCR1_IDATSZ(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_IDATSZ_SHIFT)) & FLEXSPI_IPCR1_IDATSZ_MASK)
#define FLEXSPI_IPCR1_ISEQID_MASK                (0xf0000u)
#define FLEXSPI_IPCR1_ISEQID_SHIFT               (16u)

#define FLEXSPI_IPCR1_ISEQID(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_ISEQID_SHIFT)) & FLEXSPI_IPCR1_ISEQID_MASK)
#define FLEXSPI_IPCR1_ISEQNUM_MASK               (0x7000000u)
#define FLEXSPI_IPCR1_ISEQNUM_SHIFT              (24u)

#define FLEXSPI_IPCR1_ISEQNUM(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_ISEQNUM_SHIFT)) & FLEXSPI_IPCR1_ISEQNUM_MASK)
#define FLEXSPI_IPCR1_IPAREN_MASK                (0x80000000u)
#define FLEXSPI_IPCR1_IPAREN_SHIFT               (31u)

#define FLEXSPI_IPCR1_IPAREN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_IPAREN_SHIFT)) & FLEXSPI_IPCR1_IPAREN_MASK)

#define FLEXSPI_IPCMD_TRG_MASK                   (0x1u)
#define FLEXSPI_IPCMD_TRG_SHIFT                  (0u)

#define FLEXSPI_IPCMD_TRG(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCMD_TRG_SHIFT)) & FLEXSPI_IPCMD_TRG_MASK)

#define FLEXSPI_IPRXFCR_CLRIPRXF_MASK            (0x1u)
#define FLEXSPI_IPRXFCR_CLRIPRXF_SHIFT           (0u)

#define FLEXSPI_IPRXFCR_CLRIPRXF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_CLRIPRXF_SHIFT)) & FLEXSPI_IPRXFCR_CLRIPRXF_MASK)
#define FLEXSPI_IPRXFCR_RXDMAEN_MASK             (0x2u)
#define FLEXSPI_IPRXFCR_RXDMAEN_SHIFT            (1u)

#define FLEXSPI_IPRXFCR_RXDMAEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_RXDMAEN_SHIFT)) & FLEXSPI_IPRXFCR_RXDMAEN_MASK)
#define FLEXSPI_IPRXFCR_RXWMRK_MASK              (0x3cu)
#define FLEXSPI_IPRXFCR_RXWMRK_SHIFT             (2u)

#define FLEXSPI_IPRXFCR_RXWMRK(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_RXWMRK_SHIFT)) & FLEXSPI_IPRXFCR_RXWMRK_MASK)

#define FLEXSPI_IPTXFCR_CLRIPTXF_MASK            (0x1u)
#define FLEXSPI_IPTXFCR_CLRIPTXF_SHIFT           (0u)

#define FLEXSPI_IPTXFCR_CLRIPTXF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_CLRIPTXF_SHIFT)) & FLEXSPI_IPTXFCR_CLRIPTXF_MASK)
#define FLEXSPI_IPTXFCR_TXDMAEN_MASK             (0x2u)
#define FLEXSPI_IPTXFCR_TXDMAEN_SHIFT            (1u)

#define FLEXSPI_IPTXFCR_TXDMAEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_TXDMAEN_SHIFT)) & FLEXSPI_IPTXFCR_TXDMAEN_MASK)
#define FLEXSPI_IPTXFCR_TXWMRK_MASK              (0x3cu)
#define FLEXSPI_IPTXFCR_TXWMRK_SHIFT             (2u)

#define FLEXSPI_IPTXFCR_TXWMRK(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_TXWMRK_SHIFT)) & FLEXSPI_IPTXFCR_TXWMRK_MASK)

#define FLEXSPI_DLLCR_DLLEN_MASK                 (0x1u)
#define FLEXSPI_DLLCR_DLLEN_SHIFT                (0u)

#define FLEXSPI_DLLCR_DLLEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_DLLEN_SHIFT)) & FLEXSPI_DLLCR_DLLEN_MASK)
#define FLEXSPI_DLLCR_DLLRESET_MASK              (0x2u)
#define FLEXSPI_DLLCR_DLLRESET_SHIFT             (1u)

#define FLEXSPI_DLLCR_DLLRESET(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_DLLRESET_SHIFT)) & FLEXSPI_DLLCR_DLLRESET_MASK)
#define FLEXSPI_DLLCR_SLVDLYTARGET_MASK          (0x78u)
#define FLEXSPI_DLLCR_SLVDLYTARGET_SHIFT         (3u)

#define FLEXSPI_DLLCR_SLVDLYTARGET(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_SLVDLYTARGET_SHIFT)) & FLEXSPI_DLLCR_SLVDLYTARGET_MASK)
#define FLEXSPI_DLLCR_OVRDEN_MASK                (0x100u)
#define FLEXSPI_DLLCR_OVRDEN_SHIFT               (8u)

#define FLEXSPI_DLLCR_OVRDEN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_OVRDEN_SHIFT)) & FLEXSPI_DLLCR_OVRDEN_MASK)
#define FLEXSPI_DLLCR_OVRDVAL_MASK               (0x7e00u)
#define FLEXSPI_DLLCR_OVRDVAL_SHIFT              (9u)

#define FLEXSPI_DLLCR_OVRDVAL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_OVRDVAL_SHIFT)) & FLEXSPI_DLLCR_OVRDVAL_MASK)

#define FLEXSPI_DLLCR_COUNT                      (2u)

#define FLEXSPI_STS0_SEQIDLE_MASK                (0x1u)
#define FLEXSPI_STS0_SEQIDLE_SHIFT               (0u)

#define FLEXSPI_STS0_SEQIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_SEQIDLE_SHIFT)) & FLEXSPI_STS0_SEQIDLE_MASK)
#define FLEXSPI_STS0_ARBIDLE_MASK                (0x2u)
#define FLEXSPI_STS0_ARBIDLE_SHIFT               (1u)

#define FLEXSPI_STS0_ARBIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_ARBIDLE_SHIFT)) & FLEXSPI_STS0_ARBIDLE_MASK)
#define FLEXSPI_STS0_ARBCMDSRC_MASK              (0xcu)
#define FLEXSPI_STS0_ARBCMDSRC_SHIFT             (2u)

#define FLEXSPI_STS0_ARBCMDSRC(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_ARBCMDSRC_SHIFT)) & FLEXSPI_STS0_ARBCMDSRC_MASK)

#define FLEXSPI_STS1_AHBCMDERRID_MASK            (0xfu)
#define FLEXSPI_STS1_AHBCMDERRID_SHIFT           (0u)

#define FLEXSPI_STS1_AHBCMDERRID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_AHBCMDERRID_SHIFT)) & FLEXSPI_STS1_AHBCMDERRID_MASK)
#define FLEXSPI_STS1_AHBCMDERRCODE_MASK          (0xf00u)
#define FLEXSPI_STS1_AHBCMDERRCODE_SHIFT         (8u)

#define FLEXSPI_STS1_AHBCMDERRCODE(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_AHBCMDERRCODE_SHIFT)) & FLEXSPI_STS1_AHBCMDERRCODE_MASK)
#define FLEXSPI_STS1_IPCMDERRID_MASK             (0xf0000u)
#define FLEXSPI_STS1_IPCMDERRID_SHIFT            (16u)

#define FLEXSPI_STS1_IPCMDERRID(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_IPCMDERRID_SHIFT)) & FLEXSPI_STS1_IPCMDERRID_MASK)
#define FLEXSPI_STS1_IPCMDERRCODE_MASK           (0xf000000u)
#define FLEXSPI_STS1_IPCMDERRCODE_SHIFT          (24u)

#define FLEXSPI_STS1_IPCMDERRCODE(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_IPCMDERRCODE_SHIFT)) & FLEXSPI_STS1_IPCMDERRCODE_MASK)

#define FLEXSPI_STS2_ASLVLOCK_MASK               (0x1u)
#define FLEXSPI_STS2_ASLVLOCK_SHIFT              (0u)

#define FLEXSPI_STS2_ASLVLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_ASLVLOCK_SHIFT)) & FLEXSPI_STS2_ASLVLOCK_MASK)
#define FLEXSPI_STS2_AREFLOCK_MASK               (0x2u)
#define FLEXSPI_STS2_AREFLOCK_SHIFT              (1u)

#define FLEXSPI_STS2_AREFLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_AREFLOCK_SHIFT)) & FLEXSPI_STS2_AREFLOCK_MASK)
#define FLEXSPI_STS2_ASLVSEL_MASK                (0xfcu)
#define FLEXSPI_STS2_ASLVSEL_SHIFT               (2u)

#define FLEXSPI_STS2_ASLVSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_ASLVSEL_SHIFT)) & FLEXSPI_STS2_ASLVSEL_MASK)
#define FLEXSPI_STS2_AREFSEL_MASK                (0x3f00u)
#define FLEXSPI_STS2_AREFSEL_SHIFT               (8u)

#define FLEXSPI_STS2_AREFSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_AREFSEL_SHIFT)) & FLEXSPI_STS2_AREFSEL_MASK)
#define FLEXSPI_STS2_BSLVLOCK_MASK               (0x10000u)
#define FLEXSPI_STS2_BSLVLOCK_SHIFT              (16u)

#define FLEXSPI_STS2_BSLVLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BSLVLOCK_SHIFT)) & FLEXSPI_STS2_BSLVLOCK_MASK)
#define FLEXSPI_STS2_BREFLOCK_MASK               (0x20000u)
#define FLEXSPI_STS2_BREFLOCK_SHIFT              (17u)

#define FLEXSPI_STS2_BREFLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BREFLOCK_SHIFT)) & FLEXSPI_STS2_BREFLOCK_MASK)
#define FLEXSPI_STS2_BSLVSEL_MASK                (0xfc0000u)
#define FLEXSPI_STS2_BSLVSEL_SHIFT               (18u)

#define FLEXSPI_STS2_BSLVSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BSLVSEL_SHIFT)) & FLEXSPI_STS2_BSLVSEL_MASK)
#define FLEXSPI_STS2_BREFSEL_MASK                (0x3f000000u)
#define FLEXSPI_STS2_BREFSEL_SHIFT               (24u)

#define FLEXSPI_STS2_BREFSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BREFSEL_SHIFT)) & FLEXSPI_STS2_BREFSEL_MASK)

#define FLEXSPI_AHBSPNDSTS_ACTIVE_MASK           (0x1u)
#define FLEXSPI_AHBSPNDSTS_ACTIVE_SHIFT          (0u)

#define FLEXSPI_AHBSPNDSTS_ACTIVE(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_ACTIVE_SHIFT)) & FLEXSPI_AHBSPNDSTS_ACTIVE_MASK)
#define FLEXSPI_AHBSPNDSTS_BUFID_MASK            (0xeU)
#define FLEXSPI_AHBSPNDSTS_BUFID_SHIFT           (1u)

#define FLEXSPI_AHBSPNDSTS_BUFID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_BUFID_SHIFT)) & FLEXSPI_AHBSPNDSTS_BUFID_MASK)
#define FLEXSPI_AHBSPNDSTS_DATLFT_MASK           (0xffff0000u)
#define FLEXSPI_AHBSPNDSTS_DATLFT_SHIFT          (16u)

#define FLEXSPI_AHBSPNDSTS_DATLFT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_DATLFT_SHIFT)) & FLEXSPI_AHBSPNDSTS_DATLFT_MASK)

#define FLEXSPI_IPRXFSTS_FILL_MASK               (0xffu)
#define FLEXSPI_IPRXFSTS_FILL_SHIFT              (0u)

#define FLEXSPI_IPRXFSTS_FILL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFSTS_FILL_SHIFT)) & FLEXSPI_IPRXFSTS_FILL_MASK)
#define FLEXSPI_IPRXFSTS_RDCNTR_MASK             (0xffff0000u)
#define FLEXSPI_IPRXFSTS_RDCNTR_SHIFT            (16u)

#define FLEXSPI_IPRXFSTS_RDCNTR(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFSTS_RDCNTR_SHIFT)) & FLEXSPI_IPRXFSTS_RDCNTR_MASK)

#define FLEXSPI_IPTXFSTS_FILL_MASK               (0xffu)
#define FLEXSPI_IPTXFSTS_FILL_SHIFT              (0u)

#define FLEXSPI_IPTXFSTS_FILL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFSTS_FILL_SHIFT)) & FLEXSPI_IPTXFSTS_FILL_MASK)
#define FLEXSPI_IPTXFSTS_WRCNTR_MASK             (0xffff0000u)
#define FLEXSPI_IPTXFSTS_WRCNTR_SHIFT            (16u)

#define FLEXSPI_IPTXFSTS_WRCNTR(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFSTS_WRCNTR_SHIFT)) & FLEXSPI_IPTXFSTS_WRCNTR_MASK)

#define FLEXSPI_RFDR_RXDATA_MASK                 (0xffffffffu)
#define FLEXSPI_RFDR_RXDATA_SHIFT                (0u)

#define FLEXSPI_RFDR_RXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_RFDR_RXDATA_SHIFT)) & FLEXSPI_RFDR_RXDATA_MASK)

#define FLEXSPI_RFDR_COUNT                       (32u)

#define FLEXSPI_TFDR_TXDATA_MASK                 (0xffffffffu)
#define FLEXSPI_TFDR_TXDATA_SHIFT                (0u)

#define FLEXSPI_TFDR_TXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_TFDR_TXDATA_SHIFT)) & FLEXSPI_TFDR_TXDATA_MASK)

#define FLEXSPI_TFDR_COUNT                       (32u)

#define FLEXSPI_LUT_COUNT                        (64u)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXSPI_H */
