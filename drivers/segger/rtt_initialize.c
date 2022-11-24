/****************************************************************************
 * drivers/segger/rtt_initialize.c
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

#include <assert.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/segger/rtt.h>

#include <SEGGER_RTT.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtt_channel_initialize
 ****************************************************************************/

void rtt_channel_initialize(int channel, FAR char * name,
                            size_t upsize, size_t downsize)
{
  FAR char *upbuf = (FAR char *)kmm_malloc(upsize);
  FAR char *downbuf = (FAR char *)kmm_malloc(downsize);
  DEBUGASSERT(upbuf);
  DEBUGASSERT(downbuf);

  SEGGER_RTT_ConfigUpBuffer(channel, name, upbuf,
                            upsize, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SEGGER_RTT_ConfigDownBuffer(channel, name, downbuf,
                              downsize, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}
