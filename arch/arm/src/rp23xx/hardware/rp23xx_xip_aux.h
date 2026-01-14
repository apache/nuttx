/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_xip_aux.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_AUX_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_AUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_XIP_AUX_STREAM_OFFSET        0x00000000
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_OFFSET 0x00000004
#define RP23XX_XIP_AUX_QMI_DIRECT_RX_OFFSET 0x00000008

/* Register definitions *****************************************************/

#define RP23XX_XIP_AUX_STREAM         (RP23XX_XIP_AUX_BASE + RP23XX_XIP_AUX_STREAM_OFFSET)
#define RP23XX_XIP_AUX_QMI_DIRECT_TX  (RP23XX_XIP_AUX_BASE + RP23XX_XIP_AUX_QMI_DIRECT_TX_OFFSET)
#define RP23XX_XIP_AUX_QMI_DIRECT_RX  (RP23XX_XIP_AUX_BASE + RP23XX_XIP_AUX_QMI_DIRECT_RX_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_XIP_AUX_STREAM_MASK                  0xffffffff
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_MASK           0x001fffff
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_NOPUSH_MASK    0x00100000
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_OE_MASK        0x00080000
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_DWIDTH_MASK    0x00040000
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_IWIDTH_MASK    0x00030000
#define RP23XX_XIP_AUX_QMI_DIRECT_TX_DATA_MASK      0x0000ffff
#define RP23XX_XIP_AUX_QMI_DIRECT_RX_MASK           0x0000ffff

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_XIP_AUX_H */
