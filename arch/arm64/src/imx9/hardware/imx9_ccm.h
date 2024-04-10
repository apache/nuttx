/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_ccm.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_CCM_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx9_memorymap.h"

#if defined(CONFIG_ARCH_CHIP_IMX93)
#  include "hardware/imx93/imx93_ccm.h"
#  include "hardware/imx93/imx93_pll.h"
#else
#  error Unrecognized i.MX9 architecture
#endif

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_CCM_H_ */
