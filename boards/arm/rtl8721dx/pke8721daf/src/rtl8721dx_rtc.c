/****************************************************************************
 * boards/arm/rtl8721dx/pke8721daf/src/rtl8721dx_rtc.c
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

#include <syslog.h>
#include <errno.h>

#include "ameba_rtc.h"
#include "rtl8721dx_pke8721daf.h"

#ifdef CONFIG_AMEBA_RTC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721dx_rtc_initialize
 *
 * Description:
 *   Bring up the on-chip RTC and register it at /dev/rtc0.  The RTC has no
 *   board wiring (it is an internal clock), so this simply defers to the
 *   shared driver.
 *
 ****************************************************************************/

int rtl8721dx_rtc_initialize(void)
{
  int ret;

  ret = ameba_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: ameba_rtc_initialize(/dev/rtc0) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_AMEBA_RTC */
