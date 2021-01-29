/****************************************************************************
 * include/nuttx/audio/audio_null.h
 * A do-nothinig audio device driver to simplify testing of audio decoders.
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

#ifndef __INCLUDE_NUTTX_AUDIO_AUDIO_NULL_H
#define __INCLUDE_NUTTX_AUDIO_AUDIO_NULL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_NULL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_NULL - Enabled NULL audio device support
 * CONFIG_AUDIO_NULL_MSG_PRIO - Priority of messages sent to the NULL audio
 *   device worker   thread.
 * CONFIG_AUDIO_NULL_BUFFER_SIZE - Preferred buffer size
 * CONFIG_AUDIO_NULL_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_AUDIO_NULL_WORKER_STACKSIZE - Stack size to use when creating the
 *   NULL audio device worker thread.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

/* Default configuration values */

#ifndef CONFIG_AUDIO_NULL_MSG_PRIO
#  define CONFIG_AUDIO_NULL_MSG_PRIO          1
#endif

#ifndef CONFIG_AUDIO_NULL_BUFFER_SIZE
#  define CONFIG_AUDIO_NULL_BUFFER_SIZE       8192
#endif

#ifndef CONFIG_AUDIO_NULL_NUM_BUFFERS
#  define CONFIG_AUDIO_NULL_NUM_BUFFERS       4
#endif

#ifndef CONFIG_AUDIO_NULL_WORKER_STACKSIZE
#  define CONFIG_AUDIO_NULL_WORKER_STACKSIZE  768
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
 * Name: audio_null_initialize
 *
 * Description:
 *   Initialize the null audio device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the NULL audio device is returned
 *   on success; NULL is returned on failure.
 *
 ****************************************************************************/

struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *audio_null_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_NULL */
#endif /* __INCLUDE_NUTTX_AUDIO_AUDIO_NULL_H */
