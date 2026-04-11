/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf53_qencoder.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/sensors/qencoder.h>

#include "nrf53_qdec.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct nrf53_qeconfig_s g_qe0_config =
{
  .sample_period   = 0,
  .report_period   = 0,
  .enable_debounce = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nrf53_qencoder_initialize(int devno)
{
  struct qe_lowerhalf_s *lower;
  char devpath[12];
  int ret;

  sninfo("Initializing /dev/qe%d\n", devno);

  lower = nrf53_qeinitialize(devno, &g_qe0_config);
  if (lower == NULL)
    {
      snerr("ERROR: nrf53_qeinitialize failed\n");
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno);

  ret = qe_register(devpath, lower);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  return OK;
}
