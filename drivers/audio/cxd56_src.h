/****************************************************************************
 * drivers/audio/cxd56_src.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __DRIVERS_AUDIO_CXD56_SRC_H
#define __DRIVERS_AUDIO_CXD56_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/audio/audio.h>
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

#ifndef SRC_SINC_BEST_QUALITY
#  define SRC_SINC_BEST_QUALITY     0
#endif

#ifndef SRC_SINC_MEDIUM_QUALITY
#  define SRC_SINC_MEDIUM_QUALITY   1
#endif

#ifndef SRC_SINC_FASTEST
#  define SRC_SINC_FASTEST          2
#endif

#ifndef SRC_ZERO_ORDER_HOLD
#  define SRC_ZERO_ORDER_HOLD       3
#endif

#ifndef SRC_LINEAR
#  define SRC_LINEAR                4
#endif

#define AUDIO_APB_SRC_FINAL  (1 << 4) /* Last buffer in SRC processing */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct SRC_STATE_TAG SRC_STATE;

typedef struct
{
  const float *data_in;
  float       *data_out;
  long        input_frames;
  long        output_frames;
  long        input_frames_used;
  long        output_frames_gen;
  int         end_of_input;
  double      src_ratio;
} SRC_DATA;

SRC_STATE *src_new (int converter_type, int channels, int *error);

SRC_STATE *src_delete (SRC_STATE *state);

int src_process (SRC_STATE *state, SRC_DATA *data);

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
