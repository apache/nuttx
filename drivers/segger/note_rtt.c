/****************************************************************************
 * drivers/segger/note_rtt.c
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
#include <nuttx/segger/note_rtt.h>
#include <nuttx/segger/rtt.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct notertt_s
{
  struct note_driver_s driver;
  struct lib_rttoutstream_s stream;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notertt_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_notertt_ops =
{
  notertt_add,
};

struct notertt_s g_notertt =
{
  {
    &g_notertt_ops
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notertt_add
 *
 * Description:
 *   Put the variable length note to rttoutstream
 *
 * Input Parameters:
 *   buf    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void notertt_add(FAR struct note_driver_s *drv,
                        FAR const void *buf, size_t notelen)
{
  FAR struct notertt_s *note = (FAR struct notertt_s *)drv;
  lib_stream_puts(&note->stream, buf, notelen);
}

/****************************************************************************
 * Name: notertt_register
 *
 * Description:
 *   Register a serial driver using note_driver_register
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int notertt_register(void)
{
  lib_rttoutstream_open(&g_notertt.stream,
                        CONFIG_NOTE_RTT_CHANNEL,
                        CONFIG_NOTE_RTT_BUFFER_SIZE_UP);
  return note_driver_register(&g_notertt.driver);
}
