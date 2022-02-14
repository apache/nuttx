/****************************************************************************
 * drivers/power/pm_governor.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/power/pm.h>

#include "pm.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_set_governor
 *
 * Description:
 *   This function set the domain with assigned governor
 *
 * Input Parameters:
 *   domain        - The PM domain to Set
 *   gov           - The governor to use
 *
 * Returned Value:
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

int pm_set_governor(int domain, FAR const struct pm_governor_s *gov)
{
  if (gov == NULL)
    {
      return -EINVAL;
    }

  if (g_pmglobals.domain[domain].governor &&
      g_pmglobals.domain[domain].governor->deinitialize)
    {
      g_pmglobals.domain[domain].governor->deinitialize();
    }

  g_pmglobals.domain[domain].governor = gov;

  if (g_pmglobals.domain[domain].governor->initialize)
    {
      g_pmglobals.domain[domain].governor->initialize();
    }

  return 0;
}
