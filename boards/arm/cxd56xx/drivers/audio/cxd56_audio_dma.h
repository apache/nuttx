/****************************************************************************
 * boards/arm/cxd56xx/drivers/audio/cxd56_audio_dma.h
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

#ifndef __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_DMA_H
#define __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

CXD56_AUDIO_ECODE cxd56_audio_dma_get_handle(cxd56_audio_dma_path_t path,
                                             cxd56_audio_dma_t *handle);
CXD56_AUDIO_ECODE cxd56_audio_dma_free_handle(cxd56_audio_dma_t handle);
CXD56_AUDIO_ECODE cxd56_audio_dma_init(cxd56_audio_dma_t handle,
                                       cxd56_audio_samp_fmt_t fmt,
                                       uint8_t *ch_num);
CXD56_AUDIO_ECODE cxd56_audio_dma_set_cb(cxd56_audio_dma_t handle,
                                         cxd56_audio_dma_cb_t cb);
CXD56_AUDIO_ECODE cxd56_audio_dma_get_mstate(cxd56_audio_dma_t handle,
                                      cxd56_audio_dma_mstate_t *state);
CXD56_AUDIO_ECODE cxd56_audio_dma_en_dmaint(void);
CXD56_AUDIO_ECODE cxd56_audio_dma_dis_dmaint(void);
CXD56_AUDIO_ECODE cxd56_audio_dma_start(cxd56_audio_dma_t handle,
                                        uint32_t addr,
                                        uint32_t sample);
CXD56_AUDIO_ECODE cxd56_audio_dma_stop(cxd56_audio_dma_t handle);
void cxd56_audio_dma_int_handler(void);

#endif /* __BOARDS_ARM_CXD56XX_DRIVERS_AUDIO_CXD56_AUDIO_DMA_H */
