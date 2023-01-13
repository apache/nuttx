/****************************************************************************
 * arch/sim/src/sim/posix/sim_offload.h
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

#ifndef __ARCH_SIM_SRC_POSIX_SIM_OFFLOAD_H
#define __ARCH_SIM_SRC_POSIX_SIM_OFFLOAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUDCODEC_DEC          0x01
#define AUDCODEC_ENC          0x10

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct sim_codec_ops_s
{
  uint8_t format;
  uint8_t flags;

  /* init codec handle */

  void *(*init)(struct audio_info_s *info);

  /* return how much samples return from deocde.
   * or encoder needed.
   * */

  int (*get_samples)(void *handle);

  /* perform dec or enc on [in] data with [insize] bytes.
   * [out] is pcm data with [outsize] when decode,
   * [out] is compress data with [outsize] when encode.
   * return: < 0 means failed. >= 0 means size of source data consumed.
   */

  int (*process)(void *handle, uint8_t *in, uint32_t insize,
                 uint8_t **out, unsigned int *outsize);

  /* uninit codec handle */

  void (*uninit)(void *handle);
} sim_codec_ops_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const sim_codec_ops_s g_codec_ops[];

#endif /* __ARCH_SIM_SRC_POSIX_SIM_OFFLOAD_H */
