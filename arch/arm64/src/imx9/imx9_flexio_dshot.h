/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexio_dshot.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_DSHOT_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_DSHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/timers/dshot.h>

#ifdef CONFIG_IMX9_FLEXIO_DSHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* There are 8 shifters and timers per one FlexIO IP */

#if CONFIG_DSHOT_NCHANNELS > 8
#  error Driver supports max 8 channels per flexio (CONFIG_DSHOT_NCHANNELS)
#endif

#define DSHOT_MAX_CHANNELS CONFIG_DSHOT_NCHANNELS

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Which FlexIO instance backs this DShot device */

typedef enum
{
  DSHOT_FLEXIO1 = 0,
  DSHOT_FLEXIO2 = 1,
} flexio_dshot_id_t;

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
 * Name: imx9_flexio_dshot_init
 *
 * Description:
 *   Initialise a FlexIO-backed DShot lower-half instance.
 *
 *   Channel count and pin mapping are taken from Kconfig
 *   (CONFIG_IMX9_FLEXIOx_DSHOT_NCHANNELS,
 *   CONFIG_IMX9_FLEXIOx_DSHOT_CHANNEL_PINS). These use the same FlexIO
 *   output numbers as the FlexIO PWM driver. The corresponding board
 *   IOMUXC macros must be provided, and the driver applies the required
 *   pinmux configuration during setup.
 *
 * Input Parameters:
 *   id      - Which FlexIO instance to use (DSHOT_FLEXIO1 / DSHOT_FLEXIO2)
 *
 * Returned Value:
 *   A valid pointer to struct dshot_lowerhalf_s on success; NULL on failure.
 *
 ****************************************************************************/

struct dshot_lowerhalf_s *imx9_flexio_dshot_init(flexio_dshot_id_t id);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMX9_FLEXIO_DSHOT */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_DSHOT_H */
