/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_cisif.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CISIF_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CISIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Common Register Offsets */

#define CISIF_INTR_STAT      (0x0000)
#define CISIF_INTR_ENABLE    (0x0004)
#define CISIF_INTR_DISABLE   (0x0008)
#define CISIF_INTR_CLEAR     (0x000C)
#define CISIF_DIN_ENABLE     (0x0020)
#define CISIF_CIS_SIZE       (0x0024)
#define CISIF_ACT_POS        (0x0028)
#define CISIF_ACT_SIZE       (0x002C)
#define CISIF_MODE           (0x0030)
#define CISIF_ILCODE         (0x0034)
#define CISIF_FORMAT         (0x0038)
#define CISIF_POL            (0x003C)
#define CISIF_YCC_START_ADDR (0x0040)
#define CISIF_YCC_DAREA_SIZE (0x0044)
#define CISIF_YCC_NSTRG_SIZE (0x0048)
#define CISIF_YCC_DSTRG_CONT (0x004C)
#define CISIF_YCC_DREAD_CONT (0x0050)
#define CISIF_JPG_START_ADDR (0x0060)
#define CISIF_JPG_DAREA_SIZE (0x0064)
#define CISIF_JPG_NSTRG_SIZE (0x0068)
#define CISIF_JPG_DSTRG_CONT (0x006C)
#define CISIF_JPG_DREAD_CONT (0x0070)
#define CISIF_EXE_CMD        (0x0080)
#define CISIF_NSTANDBY       (0x0090)
#define CISIF_NRST           (0x0094)
#define CISIF_TEST_INT       (0x00F0)

#define JPG_ERR_STATUS_INT   (1<<28)
#define JPG_MEM_OVF_INT      (1<<27)
#define JPG_FIFO_OVF_INT     (1<<26)
#define JPG_AXI_TRERR_INT    (1<<25)
#define JPG_MARKER_ERR_INT   (1<<24)
#define YCC_MEM_OVF_INT      (1<<21)
#define YCC_FIFO_OVF_INT     (1<<20)
#define YCC_AXI_TRERR_INT    (1<<19)
#define YCC_MARKER_ERR_INT   (1<<18)
#define SIZE_UNDER_INT       (1<<17)
#define SIZE_OVER_INT        (1<<16)
#define VLATCH_INT           (1<<15)
#define JPG_DAREA_END_INT    (1<<12)
#define JPG_NSTORAGE_INT     (1<<11)
#define JPG_AXI_TRDN_INT     (1<<10)
#define YCC_DAREA_END_INT    (1<<9)
#define YCC_NSTORAGE_INT     (1<<8)
#define YCC_AXI_TRDN_INT     (1<<7)
#define JPG_VACT_END_INT     (1<<6)
#define YCC_VACT_END_INT     (1<<5)
#define SOI_INT              (1<<4)
#define EOI_INT              (1<<3)
#define SOY_INT              (1<<2)
#define EOY_INT              (1<<1)
#define VS_INT               (1<<0)

#define ALL_CLEAR_INT        (0xFFFFFFFF)
#define ALL_DISABLE_INT      (0x01ffffff)

#define MODE_YUV_TRS_EN      (0x00000004)
#define MODE_JPG_TRS_EN      (0x00000109)
#define MODE_INTLEV_TRS_EN   (0x0000010E)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CISIF_H */
