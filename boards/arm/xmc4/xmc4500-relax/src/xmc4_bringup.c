/****************************************************************************
 * boards/arm/xmc4/xmc4500-relax/src/xmc4_bringup.c
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
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int xmc4_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_SENSORS_MAX6675
  ret = xmc4_max6675initialize("/dev/temp0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR:  stm32_max6675initialize failed: %d\n", ret);
    }
#endif

  return ret;
}
