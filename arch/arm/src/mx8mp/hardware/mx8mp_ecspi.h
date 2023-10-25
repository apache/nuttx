/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_ecspi.h
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

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_ECSPI_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_ECSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ECSPI Register Offsets ***************************************************/

#define RXDATA_OFFSET     0x0000
#define TXDATA_OFFSET     0x0004
#define CONREG_OFFSET     0x0008
#define CONFIGREG_OFFSET  0x000c
#define INTREG_OFFSET     0x0010
#define DMAREG_OFFSET     0x0014
#define STATREG_OFFSET    0x0018
#define PERIODREG_OFFSET  0x001C
#define TESTREG_OFFSET    0x0020
#define MSGDATA_OFFSET    0x0040

/* ECSPI Register Bit Definitions *******************************************/

#define CONREG_BURST_LENGTH       (20)
#define CONREG_CHANNEL_SELECT     (18)
#define CONREG_PRE_DIVIDER        (12)
#define CONREG_POST_DIVIDER       (8)
#define CONREG_CHANNEL_MODE       (4)
#define CONREG_SMC                (1 << 3)
#define CONREG_XCH                (1 << 2)
#define CONREG_HT                 (1 << 1)
#define CONREG_EN                 (1 << 0)

#define CONFIGREG_HT_LENGTH       (24)
#define CONFIGREG_SLCK_CTL        (20)
#define CONFIGREG_DATA_CTL        (16)
#define CONFIGREG_SS_POL          (12)
#define CONFIGREG_SS_CTL          (8)
#define CONFIGREG_SCLK_POL        (4)
#define CONFIGREG_SCLK_PHA        (0)

#define INTREG_TCEN               (1 << 7)
#define INTREG_ROEN               (1 << 6)
#define INTREG_RFEN               (1 << 5)
#define INTREG_RDREN              (1 << 4)
#define INTREG_RREN               (1 << 3)
#define INTREG_TFEN               (1 << 2)
#define INTREG_TDREN              (1 << 1)
#define INTREG_TEEN               (1 << 0)

#define DMAREG_RXTDEN             (1 << 31)
#define DMAREG_RX_DMA_LENGTH      (24)
#define DMAREG_RXDEN              (1 << 23)
#define DMAREG_RX_THRESHOLD       (16)
#define DMAREG_TEDEN              (1 << 7)
#define DMAREG_TX_THRESHOLD       (0)

#define STATREG_TC                (1 << 7)
#define STATREG_RO                (1 << 6)
#define STATREG_RF                (1 << 5)
#define STATREG_RDR               (1 << 4)
#define STATREG_RR                (1 << 3)
#define STATREG_TF                (1 << 2)
#define STATREG_TDR               (1 << 1)
#define STATREG_TE                (1 << 0)

#define PERIODREG_CSD_CTL         (16)
#define PERIODREG_CSRC            (1 << 15)
#define PERIODREG_SAMPLE_PERIOD   (0)

#define TESTREG_LBC               (1 << 31)
#define TESTREG_RXCNT             (8)
#define TESTREG_TXCNT             (0)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_ECSPI_H */
