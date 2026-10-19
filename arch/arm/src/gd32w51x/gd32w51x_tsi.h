/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_tsi.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_TSI_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_TSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/gd32w51x_tsi.h"

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_tsi_initialize
 *
 * Description:
 *   Enable the touch sensing interface in software trigger mode.  The
 *   board configures the pins (alternate function, open drain for the
 *   sampling capacitors and push-pull for the electrodes).
 *
 * Input Parameters:
 *   samplepins - The pins wired to a sampling capacitor, a mask of
 *                GD32W51X_TSI_PIN(group, pin), at most one per group
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gd32_tsi_initialize(uint32_t samplepins);

/****************************************************************************
 * Name: gd32_tsi_acquire
 *
 * Description:
 *   Run one charge transfer sequence on an electrode and return the number
 *   of transfer cycles that were needed to charge the sampling capacitor of
 *   its group.  The count drops when the electrode is touched.
 *
 * Input Parameters:
 *   group - The group of the electrode (0..2)
 *   pin   - The pin of the electrode in its group (0..3)
 *   count - The location to return the cycle count
 *
 * Returned Value:
 *   Zero (OK) on success; -ETIMEDOUT if the sequence did not complete or
 *   -ERANGE if the maximum cycle number was reached.
 *
 ****************************************************************************/

int gd32_tsi_acquire(int group, int pin, FAR uint16_t *count);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_TSI_H */
