/****************************************************************************
 * drivers/note/notestream_driver.c
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

#include <stdint.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/note/notestream_driver.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTEFILE
struct notestream_file_s
{
  struct notestream_driver_s driver;
  struct lib_fileoutstream_s filestream;
  struct file file;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notestream_add(FAR struct note_driver_s *drv,
                           FAR const void *note, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_notestream_ops =
{
  notestream_add
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTELOWEROUT
struct notestream_driver_s g_notestream_lowerout =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "lowerout",
    {
      {
        CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
      },
    },
#endif
    &g_notestream_ops
  },
  &g_lowoutstream
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void notestream_add(FAR struct note_driver_s *drv,
                           FAR const void *note, size_t len)
{
  FAR struct notestream_driver_s *drivers =
      (FAR struct notestream_driver_s *)drv;
  lib_stream_puts(drivers->stream, note, len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTEFILE
int notefile_register(FAR const char *filename)
{
  FAR struct notestream_file_s *notefile;
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  size_t len = strlen(filename) + 1;
#else
  size_t len = 0;
#endif
  int ret;

  notefile = kmm_zalloc(sizeof(struct notestream_file_s) + len);
  if (notefile == NULL)
    {
      return -ENOMEM;
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
  memcpy(notefile + 1, filename, len);
  notefile->driver.driver.name  = (FAR const char *)(notefile + 1);
  notefile->driver.driver.filter.mode.flag =
                      CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE;

#  ifdef CONFIG_SMP
  notefile->driver.driver.filter.mode.cpuset =
                      CONFIG_SCHED_INSTRUMENTATION_CPUSET;
#  endif

#endif

  notefile->driver.stream = &notefile->filestream.common;
  ret = file_open(&notefile->file, filename, O_WRONLY);
  if (ret < 0)
    {
      kmm_free(notefile);
      return ret;
    }

  notefile->driver.driver.ops = &g_notestream_ops;
  lib_fileoutstream(&notefile->filestream, &notefile->file);
  return note_driver_register(&notefile->driver.driver);
}
#endif

