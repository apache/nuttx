/****************************************************************************
 * include/nuttx/audio/cxd56.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_CXD56_H
#define __INCLUDE_NUTTX_AUDIO_CXD56_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#if defined(CONFIG_AUDIO_CXD56) || defined(CONFIG_CXD56_AUDIO)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cxd56_lower_s
{
  int dma_handle;
};

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

struct audio_lowerhalf_s;

FAR struct audio_lowerhalf_s *
  cxd56_initialize(FAR const struct cxd56_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_CXD56_AUDIO */
#endif /* __INCLUDE_NUTTX_AUDIO_CXD56_H */
