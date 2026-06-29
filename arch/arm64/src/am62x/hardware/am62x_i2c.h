/****************************************************************************
 * arch/arm64/src/am62x/hardware/am62x_i2c.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_I2C_H
#define __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/am62x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM62X_I2C_REVNB_LO_OFFSET       0x0000
#define AM62X_I2C_REVNB_HI_OFFSET       0x0004
#define AM62X_I2C_SYSC_OFFSET           0x0010
#define AM62X_I2C_IRQSTATUS_RAW_OFFSET  0x0024
#define AM62X_I2C_IRQSTATUS_OFFSET      0x0028
#define AM62X_I2C_IRQENABLE_SET_OFFSET  0x002c
#define AM62X_I2C_IRQENABLE_CLR_OFFSET  0x0030
#define AM62X_I2C_WE_OFFSET             0x0034
#define AM62X_I2C_SYSS_OFFSET           0x0090
#define AM62X_I2C_BUF_OFFSET            0x0094
#define AM62X_I2C_CNT_OFFSET            0x0098
#define AM62X_I2C_DATA_OFFSET           0x009c
#define AM62X_I2C_CON_OFFSET            0x00a4
#define AM62X_I2C_OA_OFFSET             0x00a8
#define AM62X_I2C_SA_OFFSET             0x00ac
#define AM62X_I2C_PSC_OFFSET            0x00b0
#define AM62X_I2C_SCLL_OFFSET           0x00b4
#define AM62X_I2C_SCLH_OFFSET           0x00b8
#define AM62X_I2C_SYSTEST_OFFSET        0x00bc
#define AM62X_I2C_BUFSTAT_OFFSET        0x00c0

#define I2C_SYSC_AUTOIDLE               (1 << 0)
#define I2C_SYSC_SRST                   (1 << 1)
#define I2C_SYSS_RST_DONE               (1 << 0)

#define I2C_IRQ_AL                      (1 << 0)
#define I2C_IRQ_NACK                    (1 << 1)
#define I2C_IRQ_ARDY                    (1 << 2)
#define I2C_IRQ_RRDY                    (1 << 3)
#define I2C_IRQ_XRDY                    (1 << 4)
#define I2C_IRQ_GC                      (1 << 5)
#define I2C_IRQ_STC                     (1 << 6)
#define I2C_IRQ_AERR                    (1 << 7)
#define I2C_IRQ_BF                      (1 << 8)
#define I2C_IRQ_AAS                     (1 << 9)
#define I2C_IRQ_XUDF                    (1 << 10)
#define I2C_IRQ_ROVR                    (1 << 11)
#define I2C_IRQ_BB                      (1 << 12)
#define I2C_IRQ_RDR                     (1 << 13)
#define I2C_IRQ_XDR                     (1 << 14)

#define I2C_IRQ_ERRORMASK               (I2C_IRQ_AL | I2C_IRQ_NACK | \
                                         I2C_IRQ_AERR | I2C_IRQ_XUDF | \
                                         I2C_IRQ_ROVR)

#define I2C_IRQ_CLEARMASK               (I2C_IRQ_AL | I2C_IRQ_NACK | \
                                         I2C_IRQ_ARDY | I2C_IRQ_RRDY | \
                                         I2C_IRQ_XRDY | I2C_IRQ_GC | \
                                         I2C_IRQ_STC | I2C_IRQ_AERR | \
                                         I2C_IRQ_BF | I2C_IRQ_AAS | \
                                         I2C_IRQ_XUDF | I2C_IRQ_ROVR | \
                                         I2C_IRQ_RDR | I2C_IRQ_XDR)

#define I2C_DATA_MASK                   0xff

#define I2C_CON_STT                     (1 << 0)
#define I2C_CON_STP                     (1 << 1)
#define I2C_CON_XSA                     (1 << 8)
#define I2C_CON_TRX                     (1 << 9)
#define I2C_CON_MST                     (1 << 10)
#define I2C_CON_EN                      (1 << 15)

#define I2C_SYSTEST_FREE                (1 << 14)

#endif /* __ARCH_ARM64_SRC_AM62X_HARDWARE_AM62X_I2C_H */
