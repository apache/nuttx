/****************************************************************************
 * include/nuttx/audio/audio_comp.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_AUDIO_COMP_H
#define __INCLUDE_NUTTX_AUDIO_AUDIO_COMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_AUDIO_COMP
#include <nuttx/audio/audio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: audio_comp_initialize
 *
 * Description:
 *   Initialize the composite audio device.
 *
 * Input Parameters:
 *   name - The name of the audio device.
 *   ...  - The list of the lower half audio driver.
 *
 * Returned Value:
 *   struct audio_lowerhalf_s* on success; NULL on failure.
 *
 * Note
 *   The variable argument list must be NULL terminated.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_comp_initialize(FAR const char *name,
                                                    ...);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_COMP */
#endif /* __INCLUDE_NUTTX_AUDIO_AUDIO_COMP_H */
