/****************************************************************************
 * drivers/note/note_initialize.c
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

#include <debug.h>

#include <nuttx/instrument.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/note/notectl_driver.h>
#include <nuttx/note/notesnap_driver.h>
#include <nuttx/note/notestream_driver.h>
#include <nuttx/segger/note_rtt.h>
#include <nuttx/segger/sysview.h>

#include "noterpmsg.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FUNCTION
static void note_driver_instrument_enter(FAR void *this_fn,
            FAR void *call_site, FAR void *arg) noinstrument_function;
static void note_driver_instrument_leave(FAR void *this_fn,
            FAR void *call_site, FAR void *arg) noinstrument_function;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FUNCTION
static void note_driver_instrument_enter(FAR void *this_fn,
                                         FAR void *call_site,
                                         FAR void *arg)
{
  sched_note_event_ip(NOTE_TAG_ALWAYS, (uintptr_t)this_fn,
                      NOTE_DUMP_BEGIN, NULL, 0);
}

static void note_driver_instrument_leave(FAR void *this_fn,
                                         FAR void *call_site,
                                         FAR void *arg)
{
  sched_note_event_ip(NOTE_TAG_ALWAYS, (uintptr_t)this_fn,
                      NOTE_DUMP_END, NULL, 0);
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_FUNCTION
static struct instrument_s g_note_instrument =
{
  .enter = note_driver_instrument_enter,
  .leave = note_driver_instrument_leave,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_early_initialize
 *
 * Description:
 *   Registers note drivers early, without depending on system features
 *   such as heap memory.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int note_early_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_SEGGER_SYSVIEW
  ret = note_sysview_initialize();
  if (ret < 0)
    {
      serr("note_sysview_initialize failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTESNAP
  ret = notesnap_register();
  if (ret < 0)
    {
      serr("notesnap_register failed %d\n", ret);
      return ret;
    }
#endif

  return ret;
}

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
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int note_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_SCHED_INSTRUMENTATION_FUNCTION
  instrument_register(&g_note_instrument);
#endif

#ifdef CONFIG_DRIVERS_NOTERAM
  ret = noteram_register();
  if (ret < 0)
    {
      serr("noteram_register failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTEFILE
  ret = notefile_register(CONFIG_DRIVERS_NOTEFILE_PATH);
  if (ret < 0)
    {
      serr("notefile_register failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_NOTE_RTT
  ret = notertt_register();
  if (ret < 0)
    {
      serr("notertt_register failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTECTL
  ret = notectl_register();
  if (ret < 0)
    {
      serr("notectl_register failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTERPMSG_SERVER
  ret = noterpmsg_server_init();
  if (ret < 0)
    {
      serr("noterpmsg_server_init failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRIVERS_NOTERPMSG
  ret = noterpmsg_init();
  if (ret < 0)
    {
      serr("noterpmsg_init failed %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
