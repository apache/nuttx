/****************************************************************************
 * boards/risc-v/mpfs/common/src/mpfs_ihc.c
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

#include <debug.h>
#include <errno.h>

#include "mpfs_ihc.h"
#include "board_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_board_ihc_init
 *
 * Description:
 *   Starts the Inter-Hart Communication (IHC) driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate any failure.
 *
 ****************************************************************************/

int mpfs_board_ihc_init(void)
{
  int ret = 0;

  /* With OpenSBI, initilization comes via mpfs_opensbi.c, not here */

#ifndef CONFIG_MPFS_OPENSBI

  ret = mpfs_ihc_init();

  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the IHC driver: %d\n",
             ret);
    }

#endif

  return ret;
}
