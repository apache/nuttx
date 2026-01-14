/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_fmu.h
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

#ifndef ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FMU_H
#define ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/nxxx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXXX_FMU_FSTAT  (NXXX_FMU0_BASE + 0x0000)       /* Flash Status Register */
#define NXXX_FMU_FCNFG  (NXXX_FMU0_BASE + 0x0004)       /* Flash Configuration Register */
#define NXXX_FMU_FCTRL  (NXXX_FMU0_BASE + 0x0008)       /* Flash Control Register */
#define NXXX_FMU_FCCOB0 (NXXX_FMU0_BASE + 0x0010)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB1 (NXXX_FMU0_BASE + 0x0014)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB2 (NXXX_FMU0_BASE + 0x0018)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB3 (NXXX_FMU0_BASE + 0x001C)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB4 (NXXX_FMU0_BASE + 0x0020)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB5 (NXXX_FMU0_BASE + 0x0024)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB6 (NXXX_FMU0_BASE + 0x0028)       /* Flash Common Command Object Registers */
#define NXXX_FMU_FCCOB7 (NXXX_FMU0_BASE + 0x002C)       /* Flash Common Command Object Registers */

/* Flash Status Register (FSTAT) */

#define FMU_FSTAT_FAIL_SHIFT                     (0)
#define FMU_FSTAT_FAIL_MASK                      (0x01 << FMU_FSTAT_FAIL_SHIFT)
#define FMU_FSTAT_FAIL(x)                        (((x) << FMU_FSTAT_FAIL_SHIFT) & FMU_FSTAT_FAIL_MASK)

#define FMU_FSTAT_CMDABT_SHIFT                   (2)
#define FMU_FSTAT_CMDABT_MASK                    (0x01 << FMU_FSTAT_CMDABT_SHIFT)
#define FMU_FSTAT_CMDABT(x)                      (((x) << FMU_FSTAT_CMDABT_SHIFT) & FMU_FSTAT_CMDABT_MASK)

#define FMU_FSTAT_PVIOL_SHIFT                    (4)
#define FMU_FSTAT_PVIOL_MASK                     (0x01 << FMU_FSTAT_PVIOL_SHIFT)
#define FMU_FSTAT_PVIOL(x)                       (((x) << FMU_FSTAT_PVIOL_SHIFT) & FMU_FSTAT_PVIOL_MASK)

#define FMU_FSTAT_ACCERR_SHIFT                   (5)
#define FMU_FSTAT_ACCERR_MASK                    (0x01 << FMU_FSTAT_ACCERR_SHIFT)
#define FMU_FSTAT_ACCERR(x)                      (((x) << FMU_FSTAT_ACCERR_SHIFT) & FMU_FSTAT_ACCERR_MASK)

#define FMU_FSTAT_CWSABT_SHIFT                   (6)
#define FMU_FSTAT_CWSABT_MASK                    (0x01 << FMU_FSTAT_CWSABT_SHIFT)
#define FMU_FSTAT_CWSABT(x)                      (((x) << FMU_FSTAT_CWSABT_SHIFT) & FMU_FSTAT_CWSABT_MASK)

#define FMU_FSTAT_CCIF_SHIFT                     (7)
#define FMU_FSTAT_CCIF_MASK                      (0x01)
#define FMU_FSTAT_CCIF(x)                        (((x) << FMU_FSTAT_CCIF_SHIFT) & FMU_FSTAT_CCIF_MASK)

#define FMU_FSTAT_CMDPRT_SHIFT                   (8)
#define FMU_FSTAT_CMDPRT_MASK                    (0x03 << FMU_FSTAT_CMDPRT_SHIFT)
#define FMU_FSTAT_CMDPRT(x)                      (((x) << FMU_FSTAT_CMDPRT_SHIFT) & FMU_FSTAT_CMDPRT_MASK)

#define FMU_FSTAT_CMDP_SHIFT                     (11)
#define FMU_FSTAT_CMDP_MASK                      (0x01 << FMU_FSTAT_CMDP_SHIFT)
#define FMU_FSTAT_CMDP(x)                        (((x) << FMU_FSTAT_CMDP_SHIFT) & FMU_FSTAT_CMDP_MASK)

#define FMU_FSTAT_CMDDID_SHIFT                   (12)
#define FMU_FSTAT_CMDDID_MASK                    (0x0f << FMU_FSTAT_CMDDID_SHIFT)
#define FMU_FSTAT_CMDDID(x)                      (((x) << FMU_FSTAT_CMDDID_SHIFT) & FMU_FSTAT_CMDDID_MASK)

#define FMU_FSTAT_DFDIF_SHIFT                    (16)
#define FMU_FSTAT_DFDIF_MASK                     (0x01 << FMU_FSTAT_DFDIF_SHIFT)
#define FMU_FSTAT_DFDIF(x)                       (((x) << FMU_FSTAT_DFDIF_SHIFT) & FMU_FSTAT_DFDIF_MASK)

#define FMU_FSTAT_SALV_USED_SHIFT                (17)
#define FMU_FSTAT_SALV_USED_MASK                 (0x01 << FMU_FSTAT_SALV_USED_SHIFT)
#define FMU_FSTAT_SALV_USED(x)                   (((x) << FMU_FSTAT_SALV_USED_SHIFT) & FMU_FSTAT_SALV_USED_MASK)

#define FMU_FSTAT_PEWEN_SHIFT                    (24)
#define FMU_FSTAT_PEWEN_MASK                     (0x03 << FMU_FSTAT_PEWEN_SHIFT)
#define FMU_FSTAT_PEWEN(x)                       (((x) << FMU_FSTAT_PEWEN_SHIFT) & FMU_FSTAT_PEWEN_MASK)

#define FMU_FSTAT_PERDY_SHIFT                    (31)
#define FMU_FSTAT_PERDY_MASK                     (0x01 << FMU_FSTAT_PERDY_SHIFT)
#define FMU_FSTAT_PERDY(x)                       (((x) << FMU_FSTAT_PERDY_SHIFT) & FMU_FSTAT_PERDY_MASK)

/* Flash Configuration Register (FCNFG) */

#define FMU_FCNFG_CCIE_SHIFT                     (7)
#define FMU_FCNFG_CCIE_MASK                      (0x01 << FMU_FCNFG_CCIE_SHIFT)
#define FMU_FCNFG_CCIE(x)                        (((x) << FMU_FCNFG_CCIE_SHIFT) & FMU_FCNFG_CCIE_MASK)

#define FMU_FCNFG_ERSREQ_SHIFT                   (8)
#define FMU_FCNFG_ERSREQ_MASK                    (0x01 << FMU_FCNFG_ERSREQ_SHIFT)
#define FMU_FCNFG_ERSREQ(x)                      (((x) << FMU_FCNFG_ERSREQ_SHIFT) & FMU_FCNFG_ERSREQ_MASK)

#define FMU_FCNFG_DFDIE_SHIFT                    (16)
#define FMU_FCNFG_DFDIE_MASK                     (0x01 << FMU_FCNFG_DFDIE_SHIFT)
#define FMU_FCNFG_DFDIE(x)                       (((x) << FMU_FCNFG_DFDIE_SHIFT) & FMU_FCNFG_DFDIE_MASK)

#define FMU_FCNFG_ERSIEN0_SHIFT                  (24)
#define FMU_FCNFG_ERSIEN0_MASK                   (0x0f << FMU_FCNFG_ERSIEN0_SHIFT)
#define FMU_FCNFG_ERSIEN0(x)                     (((x) << FMU_FCNFG_ERSIEN0_SHIFT) & FMU_FCNFG_ERSIEN0_MASK)

#define FMU_FCNFG_ERSIEN1_SHIFT                  (28)
#define FMU_FCNFG_ERSIEN1_MASK                   (0x0f << FMU_FCNFG_ERSIEN1_SHIFT)
#define FMU_FCNFG_ERSIEN1(x)                     (((x) << FMU_FCNFG_ERSIEN1_SHIFT) & FMU_FCNFG_ERSIEN1_MASK)

/* Flash Control Register (FCTRL) */

#define FMU_FCTRL_RWSC_SHIFT                     (0)
#define FMU_FCTRL_RWSC_MASK                      (0x0f << FMU_FCTRL_RWSC_SHIFT)
#define FMU_FCTRL_RWSC(x)                        (((x) << FMU_FCTRL_RWSC_SHIFT) & FMU_FCTRL_RWSC_MASK)

#define FMU_FCTRL_FDFD_SHIFT                     (16)
#define FMU_FCTRL_FDFD_MASK                      (0x01)
#define FMU_FCTRL_FDFD(x)                        (((x) << FMU_FCTRL_FDFD_SHIFT) & FMU_FCTRL_FDFD_MASK)

#define FMU_FCTRL_ABTREQ_SHIFT                   (24)
#define FMU_FCTRL_ABTREQ_MASK                    (0x01)
#define FMU_FCTRL_ABTREQ(x)                      (((x) << FMU_FCTRL_ABTREQ_SHIFT) & FMU_FCTRL_ABTREQ_MASK)

/* Flash Common Command Object Registers (FCCOB) */

#define FMU_FCCOB_CCOBn_SHIFT                    (0)
#define FMU_FCCOB_CCOBn_MASK                     (0xffffffff << FMU_FCCOB_CCOBn_SHIFT)
#define FMU_FCCOB_CCOBn(x)                       (((x) << FMU_FCCOB_CCOBn_SHIFT) & FMU_FCCOB_CCOBn_MASK)

#endif /* ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_FMU_H */
