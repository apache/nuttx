/****************************************************************************
 * drivers/coresight/coresight_tmc_core.h
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

#ifndef __DRIVERS_CORESIGHT_CORESIGHT_TMC_CORE_H
#define __DRIVERS_CORESIGHT_CORESIGHT_TMC_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/bits.h>
#include <nuttx/coresight/coresight_tmc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TMC_RSZ               0x004
#define TMC_STS               0x00c
#define TMC_RRD               0x010
#define TMC_RRP               0x014
#define TMC_RWP               0x018
#define TMC_TRG               0x01c
#define TMC_CTL               0x020
#define TMC_RWD               0x024
#define TMC_MODE              0x028
#define TMC_LBUFLEVEL         0x02c
#define TMC_CBUFLEVEL         0x030
#define TMC_BUFWM             0x034
#define TMC_RRPHI             0x038
#define TMC_RWPHI             0x03c
#define TMC_AXICTL            0x110
#define TMC_DBALO             0x118
#define TMC_DBAHI             0x11c
#define TMC_FFSR              0x300
#define TMC_FFCR              0x304
#define TMC_PSCR              0x308
#define TMC_ITMISCOP0         0xee0
#define TMC_ITTRFLIN          0xee8
#define TMC_ITATBDATA0        0xeec
#define TMC_ITATBCTR2         0xef0
#define TMC_ITATBCTR1         0xef4
#define TMC_ITATBCTR0         0xef8
#define TMC_AUTHSTATUS        0xfb8

/* TMC_AUTHSTATUS - 0xfb8 */

#define TMC_AUTH_NSID_MASK    GENMASK(1, 0)
#define TMC_NSID_EN           0x03

/* TMC_CTL - 0x020 */

#define TMC_CTL_CAPT_EN       BIT(0)

/* TMC_STS - 0x00C */

#define TMC_STS_FULL          BIT(0)
#define TMC_STS_TRIGGERED     BIT(1)
#define TMC_STS_TMCREADY      BIT(2)
#define TMC_STS_MEMERR        BIT(5)

/* TMC_AXICTL - 0x110
 *
 * TMC AXICTL format for SoC-400
 *  Bits [0-1]  : ProtCtrlBit0-1
 *  Bits [2-5]  : CacheCtrlBits 0-3 (AXCACHE)
 *  Bit  6      : Reserved
 *  Bit  7      : ScatterGatherMode
 *  Bits [8-11] : WrBurstLen
 *  Bits [12-31]  : Reserved.
 * TMC AXICTL format for SoC-600, as above except:
 *  Bits [2-5     : AXI WCACHE
 *  Bits [16-19]  : AXI RCACHE
 *  Bits [20-31]  : Reserved
 */

#define TMC_AXICTL_CLEAR_MASK   0xfbf
#define TMC_AXICTL_ARCACHE_MASK (0xf << 16)

#define TMC_AXICTL_PROT_CTL_B0  BIT(0)
#define TMC_AXICTL_PROT_CTL_B1  BIT(1)
#define TMC_AXICTL_SCT_GAT_MODE BIT(7)
#define TMC_AXICTL_WR_BURST(v)  (((v) & 0xf) << 8)
#define TMC_AXICTL_WR_BURST_16  0xf

/* Write-back Read and Write-allocate */

#define TMC_AXICTL_AXCACHE_OS   (0xf << 2)
#define TMC_AXICTL_ARCACHE_OS   (0xf << 16)

/* TMC_FFCR - 0x304 */

#define TMC_FFCR_EN_FMT         BIT(0)
#define TMC_FFCR_EN_TI          BIT(1)
#define TMC_FFCR_FON_FLIN       BIT(4)
#define TMC_FFCR_FON_TRIG_EVT   BIT(5)
#define TMC_FFCR_FON_MAN        BIT(6)
#define TMC_FFCR_TRIGON_TRIGIN  BIT(8)
#define TMC_FFCR_STOP_ON_FLUSH  BIT(12)

#define TMC_DEVID_AXIAW_VALID   BIT(16)
#define TMC_DEVID_NOSCAT        BIT(24)
#define TMC_DEVID_AXIAW_SHIFT   17
#define TMC_DEVID_AXIAW_MASK    0x7f

#define TMC_MAX_NAME_LEN        32

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum tmc_mode_e
{
  TMC_MODE_CIRCULAR_BUFFER,
  TMC_MODE_SOFTWARE_FIFO,
  TMC_MODE_HARDWARE_FIFO,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tmc_etf_register
 ****************************************************************************/

int tmc_etf_register(FAR struct coresight_tmc_dev_s *tmcdev,
                     FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: tmc_etr_register
 ****************************************************************************/

int tmc_etr_register(FAR struct coresight_tmc_dev_s *tmcdev,
                     FAR const struct coresight_desc_s *desc);

/****************************************************************************
 * Name: tmc_etf_unregister
 ****************************************************************************/

void tmc_etf_unregister(FAR struct coresight_tmc_dev_s * tmcdev);

/****************************************************************************
 * Name: tmc_etr_unregister
 ****************************************************************************/

void tmc_etr_unregister(FAR struct coresight_tmc_dev_s * tmcdev);

#endif  //__DRIVERS_CORESIGHT_CORESIGHT_TMC_CORE_H
