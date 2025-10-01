/****************************************************************************
 * drivers/syslog/vsyslog_lowout.c
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/clock.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_vsyslog_lowout
 *
 * Description:
 *   This function is an extension of nx_vsyslog(). It handles the case where
 *   the user wants to log messages only to the console in a simple,
 *   synchronous manner for debugging purposes.
 *
 ****************************************************************************/

int nx_vsyslog_lowout(FAR const IPTR char *fmt, FAR va_list *ap)
{
  struct lib_sysloglowoutstream_s stream;
  int ret = 0;

  /* Wrap the low-level output in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_sysloglowoutstream(&stream);

  /* Generate the output */

  ret += lib_vsprintf_internal(&stream.common, fmt, *ap);
  if (stream.last_ch != '\n')
    {
      lib_stream_putc(&stream.common, '\n');
      ret++;
    }

  return ret;
}
