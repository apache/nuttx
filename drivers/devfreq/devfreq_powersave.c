/****************************************************************************
 * drivers/devfreq/devfreq_powersave.c
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

#include <nuttx/devfreq.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t devfreq_powersave_limit(FAR struct devfreq_s *devfreq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct devfreq_governor_s g_devfreq_gov_powersave =
{
  .name   = "powersave",
  .limit = devfreq_powersave_limit,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t devfreq_powersave_limit(FAR struct devfreq_s *devfreq)
{
  return devfreq->min;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct devfreq_governor_s *devfreq_powersave(void)
{
  return &g_devfreq_gov_powersave;
}
