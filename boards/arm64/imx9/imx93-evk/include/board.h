/****************************************************************************
 * boards/arm64/imx9/imx93-evk/include/board.h
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

#ifndef __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H
#define __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FLEXIO to PWM pin muxings */

/* EVK signals
 * GPIO_IO04 -> FLEXIO1_04
 * GPIO_IO05 -> FLEXIO1_05
 * GPIO_IO06 -> FLEXIO1_06
 * GPIO_IO07 -> FLEXIO1_07
 */

#define FLEXIO1_PWM0_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO04_FLEXIO1_FLEXIO04, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM1_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO05_FLEXIO1_FLEXIO05, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM2_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO06_FLEXIO1_FLEXIO06, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)
#define FLEXIO1_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO07_FLEXIO1_FLEXIO07, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/* TPM3 ch3 to PWM pin GPIO_IO24 muxing */

#define TPM3_PWM3_MUX IOMUX_CFG(IOMUXC_PAD_GPIO_IO24_TPM3_CH3, IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6, 0)

/****************************************************************************
 * Public Data
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
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_IMX9_IMX93_EVK_INCLUDE_BOARD_H */
