/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_ge2d.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_GE2D_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_GE2D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GE2D_INTR_ENABLE                     (CXD56_GE2D_BASE+0x00)
#define GE2D_INTR_STAT                       (CXD56_GE2D_BASE+0x04)
#define GE2D_ADDRESS_DESCRIPTOR_START        (CXD56_GE2D_BASE+0x08)
#define GE2D_STATUS                          (CXD56_GE2D_BASE+0x0c) /* Read */
#define GE2D_CMD_DESCRIPTOR                  (CXD56_GE2D_BASE+0x0c) /* Write */
#define GE2D_STAT_NORMAL_DESCRIPTOR_ADDRESS  (CXD56_GE2D_BASE+0x10)
#define GE2D_STAT_CURRENT_DESCRIPTOR_ADDRESS (CXD56_GE2D_BASE+0x14)
#define GE2D_AHB_BURST_MODE                  (CXD56_GE2D_BASE+0x40)

/* Interrupt bits */

#define GE2D_INTR_WR_ERR   (1 << 17)
#define GE2D_INTR_RD_ERR   (1 << 16)
#define GE2D_INTR_DSD      (1 << 8)
#define GE2D_INTR_NDE      (1 << 3)
#define GE2D_INTR_NDB      (1 << 2)
#define GE2D_INTR_NDF      (1 << 1)
#define GE2D_INTR_HPU      (1 << 0)

#define GE2D_INTR_ALL (GE2D_INTR_WR_ERR | GE2D_INTR_RD_ERR | \
                       GE2D_INTR_DSD | GE2D_INTR_NDE | GE2D_INTR_NDB | \
                       GE2D_INTR_NDF | GE2D_INTR_HPU)

/* Status bits */

#define GE2D_STAT_ISER     (1 << 24)
#define GE2D_STAT_NDCR     (1 << 8)
#define GE2D_STAT_SREQ     (1 << 2)
#define GE2D_STAT_PREQ     (1 << 1)
#define GE2D_STAT_NREQ     (1 << 0)

/* Running control */

#define GE2D_NOP   0
#define GE2D_EXEC  1
#define GE2D_STOP  3

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GE2D_H */
