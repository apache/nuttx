/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_fs26.h
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

/* Copyright 2022 NXP
 * This FS26 driver is intended for ENGINEERING DEVELOPMENT OR EVALUATION
 * PURPOSES ONLY.  It is provided as an example to disable the FS26 watchdog
 * functionality for development on the S32K3XX platform.  Please refer to
 * the datasheets and application hints provided on NXP.com to implement
 * full functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"

#include "chip.h"

#include "s32k3xx_pin.h"
#include "hardware/s32k3xx_pinmux.h"
#include "hardware/s32k3xx_lpspi.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_lpspi.h"

#include <arch/board/board.h>

#if defined(CONFIG_S32K3XX_FS26)

/****************************************************************************
 * Data Types
 ****************************************************************************/

struct fs26_dev_s
{
  struct spi_dev_s *spi;           /* Saved SPI driver instance */
  uint8_t           device_status; /* Latest General device status */
};

typedef enum
{
    FS26_WD_DISABLED   = 0U, /* Watchdog refresh disabled */
    FS26_WD_SIMPLE     = 1U, /* Simple watchdog refresh */
    FS26_WD_CHALLENGER = 2U  /* Challenger watchdog refresh */
} fs26_watchdog_type;

void fs26_initialize(struct spi_dev_s *spi);

#endif /* CONFIG_S32K3XX_FS26 */
