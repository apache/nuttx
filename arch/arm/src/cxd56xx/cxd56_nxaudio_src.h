/****************************************************************************
 * drivers/audio/cxd56_src.h
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

#ifndef __DRIVERS_AUDIO_CXD56_SRC_H
#define __DRIVERS_AUDIO_CXD56_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/samplerate.h>
#include <nuttx/config.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_AUDIO_SRC_STACKSIZE
#  define CONFIG_CXD56_AUDIO_SRC_STACKSIZE  768
#endif

#ifndef CONFIG_CXD56_SRC_MSG_PRIO
#  define CONFIG_CXD56_SRC_MSG_PRIO  1
#endif

#define AUDIO_APB_SRC_FINAL  (1 << 4) /* Last buffer in SRC processing */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int cxd56_src_init(FAR struct cxd56_dev_s *dev, FAR struct dq_queue_s *inq,
                   FAR struct dq_queue_s *outq);
int cxd56_src_deinit(void);
int cxd56_src_enqueue(FAR struct ap_buffer_s *apb);
int cxd56_src_stop(void);

#endif /* CONFIG_AUDIO */

#endif /* __DRIVERS_AUDIO_CXD56_SRC_H */
