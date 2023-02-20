/****************************************************************************
 * drivers/note/note_initialize.c
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

#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/note/notectl_driver.h>
#include <nuttx/segger/sysview.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_initialize
 *
 * Description:
 *   Register sched note related drivers at /dev folder that can be used by
 *   an application to read or filter the note data.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int note_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_DRIVERS_NOTERAM
  ret = noteram_register();
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTECTL
  ret = notectl_register();
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_SEGGER_SYSVIEW
  ret = note_sysview_initialize();
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}
