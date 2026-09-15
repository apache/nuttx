/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_bootloader.c
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

#include <stdlib.h>
#include <debug.h>

#include "imxrt118x_start_cm7.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define M7_ENTRY  0x28080000u

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Release the Cortex-M7 and then exit.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  if (imxrt118x_release_cm7(M7_ENTRY) < 0)
    {
      syslog(LOG_ERR, "bootloader: releasing M7 failed\n");
      return EXIT_FAILURE;
    }

  return 0;
}
