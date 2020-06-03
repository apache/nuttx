/****************************************************************************
 * include/nuttx/audio/cs4344.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_CS4344_H
#define __INCLUDE_NUTTX_AUDIO_CS4344_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_CS4344

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_CS4344 - Enables CS4344 support
 * CONFIG_CS4344_INFLIGHT - Maximum number of buffers that the CS4344
 *   driver will send to the I2S driver before any have completed.
 * CONFIG_CS4344_MSG_PRIO - Priority of messages sent to the CS4344
 *   worker thread.
 * CONFIG_CS4344_BUFFER_SIZE - Preferred buffer size
 * CONFIG_CS4344_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_CS4344_WORKER_STACKSIZE - Stack size to use when creating the the
 *   CS4344 worker thread.
 * CONFIG_CS4344_REGDUMP - Enable logic to dump all CS4344 registers to
 *   the SYSLOG device.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the CS4344 driver
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error CONFIG_SCHED_WORKQUEUE is required by the CS4344 driver
#endif

/* Default configuration values */

#ifndef CONFIG_CS4344_INFLIGHT
#  define CONFIG_CS4344_INFLIGHT          2
#endif

#if CONFIG_CS4344_INFLIGHT > 255
#  error CONFIG_CS4344_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_CS4344_MSG_PRIO
#  define CONFIG_CS4344_MSG_PRIO          1
#endif

#ifndef CONFIG_CS4344_BUFFER_SIZE
#  define CONFIG_CS4344_BUFFER_SIZE       8192
#endif

#ifndef CONFIG_CS4344_NUM_BUFFERS
#  define CONFIG_CS4344_NUM_BUFFERS       4
#endif

#ifndef CONFIG_CS4344_WORKER_STACKSIZE
#  define CONFIG_CS4344_WORKER_STACKSIZE  768
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cs4344_initialize
 *
 * Description:
 *   Initialize the CS4344 device.
 *
 * Input Parameters:
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the CS4344 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2s_dev_s;         /* Forward reference. Defined in include/nuttx/audio/i2s.h */
struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s * cs4344_initialize(FAR struct i2s_dev_s *i2s);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_CS4344 */
#endif /* __INCLUDE_NUTTX_AUDIO_CS4344_H */
