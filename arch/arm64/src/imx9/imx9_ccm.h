/****************************************************************************
 * arch/arm64/src/imx9/imx9_ccm.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_CCM_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Name: imx9_ccm_configure_root_clock
 *
 * Description:
 *   Change root clock source and divider. Leaves the clock running state
 *   unaltered.
 *
 * Input Parameters:
 *   root - The root clock index.
 *   src  - The root clock MUX source.
 *   div  - The root clock divider.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_configure_root_clock(int root, int src, uint32_t div);

/****************************************************************************
 * Name: imx9_ccm_root_clock_on
 *
 * Description:
 *   Enable / disable root clock.
 *
 * Input Parameters:
 *   root    - The root clock index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_root_clock_on(int root, bool enabled);

/****************************************************************************
 * Name: imx9_ccm_gate_on
 *
 * Description:
 *   Enable / disable clock.
 *
 * Input Parameters:
 *   gate    - The clock gate index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_gate_on(int gate, bool enabled);

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_CCM_H */
