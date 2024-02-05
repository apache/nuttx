/****************************************************************************
 * arch/sim/src/sim/sim_hostencoder.h
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

#ifndef __ARCH_SIM_SRC_SIM_SIM_HOSTENCODER_H
#define __ARCH_SIM_SRC_SIM_SIM_HOSTENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct host_encoder_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct host_encoder_s *host_encoder_open(void);
int host_encoder_close(struct host_encoder_s *encoder);
int host_encoder_streamon(struct host_encoder_s *encoder,
                          int width, int height, int fps, int bframe);
int host_encoder_streamoff(struct host_encoder_s *encoder);
int host_encoder_enqueue(struct host_encoder_s *encoder,
                         uint8_t *data, uint32_t size, int64_t pts);
int host_encoder_dequeue(struct host_encoder_s *encoder,
                         uint8_t *data, uint32_t *size,
                         int64_t *pts, uint32_t *flags);

#endif /* __ARCH_SIM_SRC_SIM_SIM_HOSTENCODER_H */
