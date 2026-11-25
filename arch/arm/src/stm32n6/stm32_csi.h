/****************************************************************************
 * arch/arm/src/stm32n6/stm32_csi.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_CSI_H
#define __ARCH_ARM_SRC_STM32N6_STM32_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_csi_start
 *
 * Description:
 *   Configure the CSI-2 host and its D-PHY and start virtual channel 0.
 *   Every data type on virtual channel 0 is forwarded to the DCMIPP.  It
 *   sleeps and must be called from a task.
 *
 * Input Parameters:
 *   lanes   - Number of data lanes (1 or 2)
 *   bitrate - Bit rate per lane in Mbit/s (80 to 2500)
 *   bpp     - Bits per pixel of the stream (8, 10, 12 or 14)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_csi_start(unsigned int lanes, unsigned int bitrate,
                    unsigned int bpp);

/****************************************************************************
 * Name: stm32_csi_errors
 *
 * Description:
 *   Return and clear the receive errors (packet and D-PHY lane errors)
 *   seen since the last call.
 *
 ****************************************************************************/

bool stm32_csi_errors(void);

/****************************************************************************
 * Name: stm32_csi_stop
 *
 * Description:
 *   Stop virtual channel 0, power down the D-PHY and disable the CSI-2
 *   host.  It sleeps and must be called from a task.
 *
 ****************************************************************************/

void stm32_csi_stop(void);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_CSI_H */
