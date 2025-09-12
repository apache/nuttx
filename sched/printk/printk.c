/****************************************************************************
 * sched/printk/printk.c
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

#include "printk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxprintk
 *
 * Description:
 *   This routine provides low-level, internal formatted output similar to
 *   printf().  The output is sent to the system console using up_putc().
 *
 * Input Parameters:
 *   fmt - The printf-style format string
 *   ... - The variable argument list providing values referenced in fmt
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications.
 *   The return value is the number of characters actually printed.
 *
 ****************************************************************************/

int nxprintk(const char *fmt, ...)
{
  va_list ap;
  char buf[CONFIG_PRINTK_BUFFER_MAXLEN];
  int len;
  char *p;

  va_start(ap, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (len < 0)
    {
      return 0;
    }

  if (len >= sizeof(buf))
    {
      len = sizeof(buf) - 1;
    }

  for (p = buf; *p; p++)
    {
      if (*p == '\n')
        {
          up_putc('\r');
        }

      up_putc(*p);
    }

  return len;
}
