/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_dma.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_DMA_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL602_DMA_INTSTATUS_OFFSET          0x000000  /* IntStatus */
#define BL602_DMA_INTTCSTATUS_OFFSET        0x000004  /* IntTCStatus */
#define BL602_DMA_INTTCCLEAR_OFFSET         0x000008  /* IntTCClear */
#define BL602_DMA_INTERRORSTATUS_OFFSET     0x00000c  /* IntErrorStatus */
#define BL602_DMA_INTERRCLR_OFFSET          0x000010  /* IntErrClr */
#define BL602_DMA_RAWINTTCSTATUS_OFFSET     0x000014  /* RawIntTCStatus */
#define BL602_DMA_RAWINTERRORSTATUS_OFFSET  0x000018  /* RawIntErrorStatus */
#define BL602_DMA_ENBLDCHNS_OFFSET          0x00001c  /* EnbldChns */
#define BL602_DMA_SOFTBREQ_OFFSET           0x000020  /* SoftBReq */
#define BL602_DMA_SOFTSREQ_OFFSET           0x000024  /* SoftSReq */
#define BL602_DMA_SOFTLBREQ_OFFSET          0x000028  /* SoftLBReq */
#define BL602_DMA_SOFTLSREQ_OFFSET          0x00002c  /* SoftLSReq */
#define BL602_DMA_TOP_CONFIG_OFFSET         0x000030  /* Top_Config */
#define BL602_DMA_SYNC_OFFSET               0x000034  /* Sync */
#define BL602_DMA_C0SRCADDR_OFFSET          0x000100  /* C0SrcAddr */
#define BL602_DMA_C0DSTADDR_OFFSET          0x000104  /* C0DstAddr */
#define BL602_DMA_C0LLI_OFFSET              0x000108  /* C0LLI */
#define BL602_DMA_C0CONTROL_OFFSET          0x00010c  /* C0Control */
#define BL602_DMA_C0CONFIG_OFFSET           0x000110  /* C0Config */
#define BL602_DMA_C1SRCADDR_OFFSET          0x000200  /* C1SrcAddr */
#define BL602_DMA_C1DSTADDR_OFFSET          0x000204  /* C1DstAddr */
#define BL602_DMA_C1LLI_OFFSET              0x000208  /* C1LLI */
#define BL602_DMA_C1CONTROL_OFFSET          0x00020c  /* C1Control */
#define BL602_DMA_C1CONFIG_OFFSET           0x000210  /* C1Config */
#define BL602_DMA_C2SRCADDR_OFFSET          0x000300  /* C2SrcAddr */
#define BL602_DMA_C2DSTADDR_OFFSET          0x000304  /* C2DstAddr */
#define BL602_DMA_C2LLI_OFFSET              0x000308  /* C2LLI */
#define BL602_DMA_C2CONTROL_OFFSET          0x00030c  /* C2Control */
#define BL602_DMA_C2CONFIG_OFFSET           0x000310  /* C2Config */
#define BL602_DMA_C3SRCADDR_OFFSET          0x000400  /* C3SrcAddr */
#define BL602_DMA_C3DSTADDR_OFFSET          0x000404  /* C3DstAddr */
#define BL602_DMA_C3LLI_OFFSET              0x000408  /* C3LLI */
#define BL602_DMA_C3CONTROL_OFFSET          0x00040c  /* C3Control */
#define BL602_DMA_C3CONFIG_OFFSET           0x000410  /* C3Config */

/* Register definitions *****************************************************/

#define BL602_DMA_INTSTATUS          (BL602_DMA_BASE + BL602_DMA_INTSTATUS_OFFSET)
#define BL602_DMA_INTTCSTATUS        (BL602_DMA_BASE + BL602_DMA_INTTCSTATUS_OFFSET)
#define BL602_DMA_INTTCCLEAR         (BL602_DMA_BASE + BL602_DMA_INTTCCLEAR_OFFSET)
#define BL602_DMA_INTERRORSTATUS     (BL602_DMA_BASE + BL602_DMA_INTERRORSTATUS_OFFSET)
#define BL602_DMA_INTERRCLR          (BL602_DMA_BASE + BL602_DMA_INTERRCLR_OFFSET)
#define BL602_DMA_RAWINTTCSTATUS     (BL602_DMA_BASE + BL602_DMA_RAWINTTCSTATUS_OFFSET)
#define BL602_DMA_RAWINTERRORSTATUS  (BL602_DMA_BASE + BL602_DMA_RAWINTERRORSTATUS_OFFSET)
#define BL602_DMA_ENBLDCHNS          (BL602_DMA_BASE + BL602_DMA_ENBLDCHNS_OFFSET)
#define BL602_DMA_SOFTBREQ           (BL602_DMA_BASE + BL602_DMA_SOFTBREQ_OFFSET)
#define BL602_DMA_SOFTSREQ           (BL602_DMA_BASE + BL602_DMA_SOFTSREQ_OFFSET)
#define BL602_DMA_SOFTLBREQ          (BL602_DMA_BASE + BL602_DMA_SOFTLBREQ_OFFSET)
#define BL602_DMA_SOFTLSREQ          (BL602_DMA_BASE + BL602_DMA_SOFTLSREQ_OFFSET)
#define BL602_DMA_TOP_CONFIG         (BL602_DMA_BASE + BL602_DMA_TOP_CONFIG_OFFSET)
#define BL602_DMA_SYNC               (BL602_DMA_BASE + BL602_DMA_SYNC_OFFSET)
#define BL602_DMA_C0SRCADDR          (BL602_DMA_BASE + BL602_DMA_C0SRCADDR_OFFSET)
#define BL602_DMA_C0DSTADDR          (BL602_DMA_BASE + BL602_DMA_C0DSTADDR_OFFSET)
#define BL602_DMA_C0LLI              (BL602_DMA_BASE + BL602_DMA_C0LLI_OFFSET)
#define BL602_DMA_C0CONTROL          (BL602_DMA_BASE + BL602_DMA_C0CONTROL_OFFSET)
#define BL602_DMA_C0CONFIG           (BL602_DMA_BASE + BL602_DMA_C0CONFIG_OFFSET)
#define BL602_DMA_C1SRCADDR          (BL602_DMA_BASE + BL602_DMA_C1SRCADDR_OFFSET)
#define BL602_DMA_C1DSTADDR          (BL602_DMA_BASE + BL602_DMA_C1DSTADDR_OFFSET)
#define BL602_DMA_C1LLI              (BL602_DMA_BASE + BL602_DMA_C1LLI_OFFSET)
#define BL602_DMA_C1CONTROL          (BL602_DMA_BASE + BL602_DMA_C1CONTROL_OFFSET)
#define BL602_DMA_C1CONFIG           (BL602_DMA_BASE + BL602_DMA_C1CONFIG_OFFSET)
#define BL602_DMA_C2SRCADDR          (BL602_DMA_BASE + BL602_DMA_C2SRCADDR_OFFSET)
#define BL602_DMA_C2DSTADDR          (BL602_DMA_BASE + BL602_DMA_C2DSTADDR_OFFSET)
#define BL602_DMA_C2LLI              (BL602_DMA_BASE + BL602_DMA_C2LLI_OFFSET)
#define BL602_DMA_C2CONTROL          (BL602_DMA_BASE + BL602_DMA_C2CONTROL_OFFSET)
#define BL602_DMA_C2CONFIG           (BL602_DMA_BASE + BL602_DMA_C2CONFIG_OFFSET)
#define BL602_DMA_C3SRCADDR          (BL602_DMA_BASE + BL602_DMA_C3SRCADDR_OFFSET)
#define BL602_DMA_C3DSTADDR          (BL602_DMA_BASE + BL602_DMA_C3DSTADDR_OFFSET)
#define BL602_DMA_C3LLI              (BL602_DMA_BASE + BL602_DMA_C3LLI_OFFSET)
#define BL602_DMA_C3CONTROL          (BL602_DMA_BASE + BL602_DMA_C3CONTROL_OFFSET)
#define BL602_DMA_C3CONFIG           (BL602_DMA_BASE + BL602_DMA_C3CONFIG_OFFSET)

/* Register bit definitions *************************************************/

#define DMA_INTSTATUS_MASK              (0xff)

#define DMA_INTTCSTATUS_MASK            (0xff)

#define DMA_INTTCCLEAR_MASK             (0xff)

#define DMA_INTERRORSTATUS_MASK         (0xff)

#define DMA_INTERRCLR_MASK              (0xff)

#define DMA_RAWINTTCSTATUS_MASK         (0xff)

#define DMA_RAWINTERRORSTATUS_MASK      (0xff)

#define DMA_ENBLDCHNS_MASK              (0xff)

#define DMA_TOP_CONFIG_M                              (1 << 1)
#define DMA_TOP_CONFIG_E                              (1 << 0)

#define DMA_C0CONTROL_I                               (1 << 31)
#define DMA_C0CONTROL_PROT_SHIFT                      (28)
#define DMA_C0CONTROL_PROT_MASK                       (0x07 << DMA_C0CONTROL_PROT_SHIFT)
#define DMA_C0CONTROL_DI                              (1 << 27)
#define DMA_C0CONTROL_SI                              (1 << 26)
#define DMA_C0CONTROL_SLARGERD                        (1 << 24)
#define DMA_C0CONTROL_DWIDTH_SHIFT                    (21)
#define DMA_C0CONTROL_DWIDTH_MASK                     (0x07 << DMA_C0CONTROL_DWIDTH_SHIFT)
#define DMA_C0CONTROL_SWIDTH_SHIFT                    (18)
#define DMA_C0CONTROL_SWIDTH_MASK                     (0x07 << DMA_C0CONTROL_SWIDTH_SHIFT)
#define DMA_C0CONTROL_DBSIZE_SHIFT                    (15)
#define DMA_C0CONTROL_DBSIZE_MASK                     (0x07 << DMA_C0CONTROL_DBSIZE_SHIFT)
#define DMA_C0CONTROL_SBSIZE_SHIFT                    (12)
#define DMA_C0CONTROL_SBSIZE_MASK                     (0x07 << DMA_C0CONTROL_SBSIZE_SHIFT)
#define DMA_C0CONTROL_TRANSFERSIZE_MASK               (0xfff)

#define DMA_C0CONFIG_LLICOUNTER_SHIFT                 (20)
#define DMA_C0CONFIG_LLICOUNTER_MASK                  (0x3ff << DMA_C0CONFIG_LLICOUNTER_SHIFT)
#define DMA_C0CONFIG_H                                (1 << 18)
#define DMA_C0CONFIG_A                                (1 << 17)
#define DMA_C0CONFIG_L                                (1 << 16)
#define DMA_C0CONFIG_ITC                              (1 << 15)
#define DMA_C0CONFIG_IE                               (1 << 14)
#define DMA_C0CONFIG_FLOWCNTRL_SHIFT                  (11)
#define DMA_C0CONFIG_FLOWCNTRL_MASK                   (0x07 << DMA_C0CONFIG_FLOWCNTRL_SHIFT)
#define DMA_C0CONFIG_DSTPERIPHERAL_SHIFT              (6)
#define DMA_C0CONFIG_DSTPERIPHERAL_MASK               (0x1f << DMA_C0CONFIG_DSTPERIPHERAL_SHIFT)
#define DMA_C0CONFIG_SRCPERIPHERAL_SHIFT              (1)
#define DMA_C0CONFIG_SRCPERIPHERAL_MASK               (0x1f << DMA_C0CONFIG_SRCPERIPHERAL_SHIFT)
#define DMA_C0CONFIG_E                                (1 << 0)

#define DMA_C1LLI_LLI_SHIFT                           (2)
#define DMA_C1LLI_LLI_MASK                            (0x3fffffff << DMA_C1LLI_LLI_SHIFT)

#define DMA_C1CONTROL_I                               (1 << 31)
#define DMA_C1CONTROL_PROT_SHIFT                      (28)
#define DMA_C1CONTROL_PROT_MASK                       (0x07 << DMA_C1CONTROL_PROT_SHIFT)
#define DMA_C1CONTROL_DI                              (1 << 27)
#define DMA_C1CONTROL_SI                              (1 << 26)
#define DMA_C1CONTROL_DWIDTH_SHIFT                    (21)
#define DMA_C1CONTROL_DWIDTH_MASK                     (0x07 << DMA_C1CONTROL_DWIDTH_SHIFT)
#define DMA_C1CONTROL_SWIDTH_SHIFT                    (18)
#define DMA_C1CONTROL_SWIDTH_MASK                     (0x07 << DMA_C1CONTROL_SWIDTH_SHIFT)
#define DMA_C1CONTROL_DBSIZE_SHIFT                    (15)
#define DMA_C1CONTROL_DBSIZE_MASK                     (0x07 << DMA_C1CONTROL_DBSIZE_SHIFT)
#define DMA_C1CONTROL_SBSIZE_SHIFT                    (12)
#define DMA_C1CONTROL_SBSIZE_MASK                     (0x07 << DMA_C1CONTROL_SBSIZE_SHIFT)
#define DMA_C1CONTROL_TRANSFERSIZE_MASK               (0xfff)

#define DMA_C1CONFIG_H                                (1 << 18)
#define DMA_C1CONFIG_A                                (1 << 17)
#define DMA_C1CONFIG_L                                (1 << 16)
#define DMA_C1CONFIG_ITC                              (1 << 15)
#define DMA_C1CONFIG_IE                               (1 << 14)
#define DMA_C1CONFIG_FLOWCNTRL_SHIFT                  (11)
#define DMA_C1CONFIG_FLOWCNTRL_MASK                   (0x07 << DMA_C1CONFIG_FLOWCNTRL_SHIFT)
#define DMA_C1CONFIG_DSTPERIPHERAL_SHIFT              (6)
#define DMA_C1CONFIG_DSTPERIPHERAL_MASK               (0x1f << DMA_C1CONFIG_DSTPERIPHERAL_SHIFT)
#define DMA_C1CONFIG_SRCPERIPHERAL_SHIFT              (1)
#define DMA_C1CONFIG_SRCPERIPHERAL_MASK               (0x1f << DMA_C1CONFIG_SRCPERIPHERAL_SHIFT)
#define DMA_C1CONFIG_E                                (1 << 0)

#define DMA_C2LLI_LLI_SHIFT                           (2)
#define DMA_C2LLI_LLI_MASK                            (0x3fffffff << DMA_C2LLI_LLI_SHIFT)

#define DMA_C2CONTROL_I                               (1 << 31)
#define DMA_C2CONTROL_PROT_SHIFT                      (28)
#define DMA_C2CONTROL_PROT_MASK                       (0x07 << DMA_C2CONTROL_PROT_SHIFT)
#define DMA_C2CONTROL_DI                              (1 << 27)
#define DMA_C2CONTROL_SI                              (1 << 26)
#define DMA_C2CONTROL_DWIDTH_SHIFT                    (21)
#define DMA_C2CONTROL_DWIDTH_MASK                     (0x07 << DMA_C2CONTROL_DWIDTH_SHIFT)
#define DMA_C2CONTROL_SWIDTH_SHIFT                    (18)
#define DMA_C2CONTROL_SWIDTH_MASK                     (0x07 << DMA_C2CONTROL_SWIDTH_SHIFT)
#define DMA_C2CONTROL_DBSIZE_SHIFT                    (15)
#define DMA_C2CONTROL_DBSIZE_MASK                     (0x07 << DMA_C2CONTROL_DBSIZE_SHIFT)
#define DMA_C2CONTROL_SBSIZE_SHIFT                    (12)
#define DMA_C2CONTROL_SBSIZE_MASK                     (0x07 << DMA_C2CONTROL_SBSIZE_SHIFT)
#define DMA_C2CONTROL_TRANSFERSIZE_MASK               (0xfff)

#define DMA_C2CONFIG_H                                (1 << 18)
#define DMA_C2CONFIG_A                                (1 << 17)
#define DMA_C2CONFIG_L                                (1 << 16)
#define DMA_C2CONFIG_ITC                              (1 << 15)
#define DMA_C2CONFIG_IE                               (1 << 14)
#define DMA_C2CONFIG_FLOWCNTRL_SHIFT                  (11)
#define DMA_C2CONFIG_FLOWCNTRL_MASK                   (0x07 << DMA_C2CONFIG_FLOWCNTRL_SHIFT)
#define DMA_C2CONFIG_DSTPERIPHERAL_SHIFT              (6)
#define DMA_C2CONFIG_DSTPERIPHERAL_MASK               (0x1f << DMA_C2CONFIG_DSTPERIPHERAL_SHIFT)
#define DMA_C2CONFIG_SRCPERIPHERAL_SHIFT              (1)
#define DMA_C2CONFIG_SRCPERIPHERAL_MASK               (0x1f << DMA_C2CONFIG_SRCPERIPHERAL_SHIFT)
#define DMA_C2CONFIG_E                                (1 << 0)

#define DMA_C3LLI_LLI_SHIFT                           (2)
#define DMA_C3LLI_LLI_MASK                            (0x3fffffff << DMA_C3LLI_LLI_SHIFT)

#define DMA_C3CONTROL_I                               (1 << 31)
#define DMA_C3CONTROL_PROT_SHIFT                      (28)
#define DMA_C3CONTROL_PROT_MASK                       (0x07 << DMA_C3CONTROL_PROT_SHIFT)
#define DMA_C3CONTROL_DI                              (1 << 27)
#define DMA_C3CONTROL_SI                              (1 << 26)
#define DMA_C3CONTROL_DWIDTH_SHIFT                    (21)
#define DMA_C3CONTROL_DWIDTH_MASK                     (0x07 << DMA_C3CONTROL_DWIDTH_SHIFT)
#define DMA_C3CONTROL_SWIDTH_SHIFT                    (18)
#define DMA_C3CONTROL_SWIDTH_MASK                     (0x07 << DMA_C3CONTROL_SWIDTH_SHIFT)
#define DMA_C3CONTROL_DBSIZE_SHIFT                    (15)
#define DMA_C3CONTROL_DBSIZE_MASK                     (0x07 << DMA_C3CONTROL_DBSIZE_SHIFT)
#define DMA_C3CONTROL_SBSIZE_SHIFT                    (12)
#define DMA_C3CONTROL_SBSIZE_MASK                     (0x07 << DMA_C3CONTROL_SBSIZE_SHIFT)
#define DMA_C3CONTROL_TRANSFERSIZE_MASK               (0xfff)

#define DMA_C3CONFIG_H                                (1 << 18)
#define DMA_C3CONFIG_A                                (1 << 17)
#define DMA_C3CONFIG_L                                (1 << 16)
#define DMA_C3CONFIG_ITC                              (1 << 15)
#define DMA_C3CONFIG_IE                               (1 << 14)
#define DMA_C3CONFIG_FLOWCNTRL_SHIFT                  (11)
#define DMA_C3CONFIG_FLOWCNTRL_MASK                   (0x07 << DMA_C3CONFIG_FLOWCNTRL_SHIFT)
#define DMA_C3CONFIG_DSTPERIPHERAL_SHIFT              (6)
#define DMA_C3CONFIG_DSTPERIPHERAL_MASK               (0x1f << DMA_C3CONFIG_DSTPERIPHERAL_SHIFT)
#define DMA_C3CONFIG_SRCPERIPHERAL_SHIFT              (1)
#define DMA_C3CONFIG_SRCPERIPHERAL_MASK               (0x1f << DMA_C3CONFIG_SRCPERIPHERAL_SHIFT)
#define DMA_C3CONFIG_E                                (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_DMA_H */
