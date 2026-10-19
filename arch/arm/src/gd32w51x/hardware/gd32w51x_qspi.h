/****************************************************************************
 * arch/arm/src/gd32w51x/hardware/gd32w51x_qspi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_QSPI_H
#define __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32w51x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets (non-secure register set) */

#define GD32W51X_QSPI_CTL_OFFSET     0x0000  /* Control */
#define GD32W51X_QSPI_DCFG_OFFSET    0x0004  /* Device configuration */
#define GD32W51X_QSPI_STAT_OFFSET    0x0008  /* Status */
#define GD32W51X_QSPI_STATC_OFFSET   0x000c  /* Status clear */
#define GD32W51X_QSPI_DTLEN_OFFSET   0x0010  /* Data length (n - 1) */
#define GD32W51X_QSPI_TCFG_OFFSET    0x0014  /* Transfer configuration */
#define GD32W51X_QSPI_ADDR_OFFSET    0x0018  /* Address */
#define GD32W51X_QSPI_ALTE_OFFSET    0x001c  /* Alternate bytes */
#define GD32W51X_QSPI_DATA_OFFSET    0x0020  /* Data */

/* Register addresses */

#define GD32W51X_QSPI_CTL     (GD32W51X_QSPI_BASE + GD32W51X_QSPI_CTL_OFFSET)
#define GD32W51X_QSPI_DCFG    (GD32W51X_QSPI_BASE + GD32W51X_QSPI_DCFG_OFFSET)
#define GD32W51X_QSPI_STAT    (GD32W51X_QSPI_BASE + GD32W51X_QSPI_STAT_OFFSET)
#define GD32W51X_QSPI_STATC \
  (GD32W51X_QSPI_BASE + GD32W51X_QSPI_STATC_OFFSET)
#define GD32W51X_QSPI_DTLEN \
  (GD32W51X_QSPI_BASE + GD32W51X_QSPI_DTLEN_OFFSET)
#define GD32W51X_QSPI_TCFG    (GD32W51X_QSPI_BASE + GD32W51X_QSPI_TCFG_OFFSET)
#define GD32W51X_QSPI_ADDR    (GD32W51X_QSPI_BASE + GD32W51X_QSPI_ADDR_OFFSET)
#define GD32W51X_QSPI_DATA    (GD32W51X_QSPI_BASE + GD32W51X_QSPI_DATA_OFFSET)

/* QSPI_CTL */

#define QSPI_CTL_QSPIEN              (1 << 0)   /* Enable */
#define QSPI_CTL_ABORT               (1 << 1)   /* Abort request */
#define QSPI_CTL_SSAMPLE_SHIFT       (4)        /* Sample shift */
#define QSPI_CTL_SSAMPLE_MASK        (3 << QSPI_CTL_SSAMPLE_SHIFT)
#define QSPI_CTL_FTL_SHIFT           (8)        /* FIFO threshold (n - 1) */
#define QSPI_CTL_FTL_MASK            (15 << QSPI_CTL_FTL_SHIFT)
#define QSPI_CTL_BUSY                (1 << 14)  /* Busy */
#define QSPI_CTL_PSC_SHIFT           (24)       /* Clock prescaler (n - 1) */
#define QSPI_CTL_PSC_MASK            (0xff << QSPI_CTL_PSC_SHIFT)

/* QSPI_DCFG */

#define QSPI_DCFG_CKMOD              (1 << 0)   /* SCK high when idle */
#define QSPI_DCFG_CSHC_SHIFT         (8)        /* CS high cycles (n - 1) */
#define QSPI_DCFG_CSHC_MASK          (7 << QSPI_DCFG_CSHC_SHIFT)
#define QSPI_DCFG_FMSZ_SHIFT         (16)       /* Flash size: 2^(n + 1) */
#define QSPI_DCFG_FMSZ_MASK          (0x1f << QSPI_DCFG_FMSZ_SHIFT)

/* QSPI_STAT and QSPI_STATC */

#define QSPI_STAT_TERR               (1 << 0)   /* Transfer error */
#define QSPI_STAT_TC                 (1 << 1)   /* Transfer complete */
#define QSPI_STAT_FT                 (1 << 2)   /* FIFO threshold */
#define QSPI_STAT_SM                 (1 << 3)   /* Status match */
#define QSPI_STAT_TMOUT              (1 << 4)   /* Timeout */
#define QSPI_STAT_WS                 (1 << 5)   /* Wrong start sequence */

#define QSPI_STATC_TERRC             (1 << 0)
#define QSPI_STATC_TCC               (1 << 1)
#define QSPI_STATC_SMC               (1 << 3)
#define QSPI_STATC_TMOUTC            (1 << 4)
#define QSPI_STATC_WSC               (1 << 5)

/* QSPI_TCFG */

#define QSPI_TCFG_INSTRUCTION_SHIFT  (0)
#define QSPI_TCFG_IMOD_SHIFT         (8)        /* Instruction lines */
#define QSPI_TCFG_ADDRMOD_SHIFT      (10)       /* Address lines */
#define QSPI_TCFG_ADDRSZ_SHIFT       (12)       /* Address size (bytes - 1) */
#define QSPI_TCFG_ALTEMOD_SHIFT      (14)       /* Alternate byte lines */
#define QSPI_TCFG_DUMYC_SHIFT        (18)       /* Dummy cycles */
#define QSPI_TCFG_DUMYC_MASK         (0x1f << QSPI_TCFG_DUMYC_SHIFT)
#define QSPI_TCFG_DATAMOD_SHIFT      (24)       /* Data lines */
#define QSPI_TCFG_FMOD_SHIFT         (26)       /* Functional mode */

#define QSPI_LINES_NONE              (0)
#define QSPI_LINES_1                 (1)
#define QSPI_LINES_2                 (2)
#define QSPI_LINES_4                 (3)

#define QSPI_FMOD_INDIRECT_WRITE     (0)
#define QSPI_FMOD_INDIRECT_READ      (1)
#define QSPI_FMOD_AUTOPOLLING        (2)
#define QSPI_FMOD_MEMORY_MAPPED      (3)

#endif /* __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_QSPI_H */
