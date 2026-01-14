/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_coresight_trace.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CORESIGHT_TRACE_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CORESIGHT_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_CORESIGHT_TRACE_CTRL_STATUS_OFFSET          0x00000000
#define RP23XX_CORESIGHT_TRACE_TRACE_CAPTURE_FIFO_OFFSET   0x00000004

/* Register definitions *****************************************************/

#define RP23XX_CORESIGHT_TRACE_CTRL_STATUS          (RP23XX_CORESIGHT_TRACE_BASE + RP23XX_CORESIGHT_TRACE_CTRL_STATUS_OFFSET)
#define RP23XX_CORESIGHT_TRACE_TRACE_CAPTURE_FIFO   (RP23XX_CORESIGHT_TRACE_BASE + RP23XX_CORESIGHT_TRACE_TRACE_CAPTURE_FIFO_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_CORESIGHT_TRACE_CTRL_STATUS_MASK                     (0x00000003)
#define RP23XX_CORESIGHT_TRACE_CTRL_STATUS_TRACE_CAPTURE_FIFO_OVERFLOW  (1 << 1)
#define RP23XX_CORESIGHT_TRACE_CTRL_STATUS_TRACE_CAPTURE_FIFO_FLUSH     (1 << 0)
#define RP23XX_CORESIGHT_TRACE_TRACE_CAPTURE_FIFO_MASK              (0xffffffff)
#define RP23XX_CORESIGHT_TRACE_TRACE_CAPTURE_FIFO_RDATA_MASK        (0xffffffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_CORESIGHT_TRACE_H */
