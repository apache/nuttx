/****************************************************************************
 * include/nuttx/audio/audio_null.h
 * A do-nothinig audio device driver to simplify testing of audio decoders.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
