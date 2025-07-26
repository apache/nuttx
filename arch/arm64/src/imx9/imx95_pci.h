/****************************************************************************
 * arch/arm64/src/imx9/imx95_pci.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX95_PCIE_H
#define __ARCH_ARM64_SRC_IMX9_IMX95_PCIE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/imx95_pcie.h"

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: imx95_pcie_init
 *
 * Description:
 *   Init imx95 pcie
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *  0 if success, a negated errno value if failed
 *
 ****************************************************************************/

int imx95_pcie_init(void);

/****************************************************************************
 * Name: imx95_pcie_uninit
 *
 * Description:
 *   Init imx95 pcie
 *
 * Input Parameters:
 *   imx95_pci  - The imx95 pcie controller
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void imx95_pcie_uninit(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX95_PCIE_H */
