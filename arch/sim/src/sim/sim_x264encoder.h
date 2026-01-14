/****************************************************************************
 * arch/sim/src/sim/sim_x264encoder.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_SIM_SRC_SIM_SIM_X264ENCODER_H
#define __ARCH_SIM_SRC_SIM_SIM_X264ENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct x264_wrapper_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct x264_wrapper_s *x264_wrapper_open(void);
int x264_wrapper_close(struct x264_wrapper_s *encoder);
int x264_wrapper_streamon(struct x264_wrapper_s *encoder,
                          int width, int height, int fps, int bframe);
int x264_wrapper_streamoff(struct x264_wrapper_s *encoder);
int x264_wrapper_enqueue(struct x264_wrapper_s *encoder,
                         uint8_t *data, uint32_t size, int64_t pts);
int x264_wrapper_dequeue(struct x264_wrapper_s *encoder,
                         uint8_t *data, uint32_t *size,
                         int64_t *pts, uint32_t *flags);

#endif /* __ARCH_SIM_SRC_SIM_SIM_X264ENCODER_H */
