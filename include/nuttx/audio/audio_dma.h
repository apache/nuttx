/****************************************************************************
 * include/nuttx/audio/audio_dma.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_AUDIO_DMA_H
#define __INCLUDE_NUTTX_AUDIO_AUDIO_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>
#include <nuttx/dma/dma.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct audio_lowerhalf_s *audio_dma_initialize(struct dma_dev_s *dma_dev,
                                               uint8_t chan_num,
                                               bool playback,
                                               uint8_t fifo_width,
                                               uintptr_t fifo_addr);
#endif
