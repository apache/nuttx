/****************************************************************************
 * libs/libc/environ/env_clearenv.c
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

#include <nuttx/environ.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clearenv
 *
 * Description:
 *   The clearenv() function clears the environment of all name-value pairs
 *   and sets the value of the external variable environ to NULL.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int clearenv(void)
{
  FAR struct task_info_s *info = task_get_info();

  nxrmutex_lock(&info->ta_lock);
  env_release(info);
  nxrmutex_unlock(&info->ta_lock);
  return OK;
}
