/****************************************************************************
 * drivers/trace32/note_fdx.c
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

#include <nuttx/note/note_driver.h>
#include <nuttx/spinlock.h>

#include "t32fdx/t32fdx.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct notefdx_s
{
  struct note_driver_s driver;
  FAR void *channel;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notefdx_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

T32_Fdx_DefineChannel(g_fdx_note_buffer, CONFIG_TRACE32_FDX_NOTE_BUFSIZE);

static const struct note_driver_ops_s g_notefdx_ops =
{
  notefdx_add,
};

struct notefdx_s g_notefdx =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "fdx",
    {
      {
        CONFIG_TRACE32_FDX_NOTE_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_TRACE32_FDX_NOTE_CPUSET
#  endif
      },
    },
#endif
    &g_notefdx_ops
  },
  NULL,
  SP_UNLOCKED
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notefdx_add
 *
 * Description:
 *   Put the variable length note to fdx
 *
 * Input Parameters:
 *   buf    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void notefdx_add(FAR struct note_driver_s *drv,
                        FAR const void *buf, size_t notelen)
{
  FAR struct notefdx_s *note = (FAR struct notefdx_s *)drv;

  if (note->channel != NULL)
    {
      irqstate_t flags = spin_lock_irqsave(&note->lock);

      T32_Fdx_SendPoll(note->channel, (FAR void *)buf, notelen);
      spin_unlock_irqrestore(&note->lock, flags);
    }
}

/****************************************************************************
 * Name: notefdx_register
 *
 * Description:
 *   Register a trace32 fdx driver using note_driver_register
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int notefdx_register(void)
{
  T32_Fdx_InitChannel(g_fdx_note_buffer);
  T32_Fdx_EnableChannel(g_fdx_note_buffer);
  g_notefdx.channel = &g_fdx_note_buffer;
  return note_driver_register(&g_notefdx.driver);
}

/****************************************************************************
 * Name: T32_Fdx_GetTimebase
 *
 * Description:
 *   Get the timebase for the FDX driver
 *
 * Returned Value:
 *   The timebase for the FDX driver
 *
 * Note:
 *   This function is used to get the timebase for the FDX driver.
 *   It is called by t32fdx.c
 *
 ****************************************************************************/

unsigned long T32_Fdx_GetTimebase(void)
{
  return perf_gettime();
}
