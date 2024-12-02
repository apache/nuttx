/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexspi_nor.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_FLEXSPI_NOR_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_FLEXSPI_NOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_IMX9_FLEXSPI_NOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_flexspi_nor_initialize
 *
 * Description:
 *   Initialize a NOR FLASH on FlexSPI interface
 *
 * Input Parameters:
 *   intf: FlexSPI interface number
 *
 * Returned Value:
 *   Pointer to an mtd device, NULL on any error
 *
 ****************************************************************************/

struct mtd_dev_s *imx9_flexspi_nor_initialize(int intf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMX9_FLEXSPI_NOR */
#endif /* __ARCH_ARM_SRC_IMX9_IMX9_FLEXSPI_NOR_H */
