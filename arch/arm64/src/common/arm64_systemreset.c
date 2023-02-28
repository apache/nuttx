/****************************************************************************
 * arch/arm64/src/common/arm64_systemreset.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "arm64_internal.h"
#include "arm64_cpu_psci.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal, arm64 reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  int ret;

  /* Set up for the system reset */

  ret = psci_cpu_reset();
  if (ret)
    {
      sinfo("Failed to reset CPU, error code: %d\n", ret);
    }

  /* Wait for the reset */

  for (; ; );
}
