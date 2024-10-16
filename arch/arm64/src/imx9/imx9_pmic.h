/****************************************************************************
 * arch/arm64/src/imx9/imx9_pmic.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include "arm64_internal.h"
#include "hardware/imx9_memorymap.h"

/****************************************************************************
 * Public Function Prototypes
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
 * Name: imx9_pmic_getreset_reason
 *
 * Description:
 *  Read reset reason from pmic via i2c
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   uin32_t, reset reason
 *
 ****************************************************************************/

uint32_t imx9_pmic_getreset_reason(void);

/****************************************************************************
 * Name: imx9_pmic_reset
 *
 * Description:
 *  Reset entire Soc
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_pmic_reset(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif

#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_PMIC_H */
