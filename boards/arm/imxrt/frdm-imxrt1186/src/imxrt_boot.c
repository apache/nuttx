/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_start.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Initialize on-chip RAM.
 *
 ****************************************************************************/

void imxrt_ocram_initialize(void)
{
}

/****************************************************************************
 * Name: imxrt_flexram_partition
 *
 * Description:
 *   Configure the FlexRAM partition.
 *
 ****************************************************************************/

void imxrt_flexram_partition(void)
{
}

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   Perform early board initialization.
 *
 ****************************************************************************/

void imxrt_boardinitialize(void)
{
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
extern int imxrt_bringup(void);

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Perform late board initialization.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  imxrt_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
