/****************************************************************************
 * libs/libc/syslog/lib_syslog_none.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog/vsyslog
 *
 * Description:
 *   These are empty syslog function used when CONFIG_SYSLOG_NONE option
 *   is selected. This way we can completely remove the syslog logic and its
 *   associated strings from the image.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void syslog(int priority, FAR const IPTR char *fmt, ...)
{
  UNUSED(priority);
  UNUSED(fmt);
}

void vsyslog(int priority, FAR const IPTR char *fmt, va_list ap)
{
  UNUSED(priority);
  UNUSED(fmt);
  UNUSED(ap);
}
