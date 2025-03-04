/****************************************************************************
 * arch/sim/src/sim/sim_openh264dec.h
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

#ifndef __ARCH_SIM_SRC_SIM_SIM_OPENH264DEC_H
#define __ARCH_SIM_SRC_SIM_SIM_OPENH264DEC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct openh264_decoder_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct openh264_decoder_s *openh264_decoder_open(void);
int openh264_decoder_close(struct openh264_decoder_s *decoder);
int openh264_decoder_streamon(struct openh264_decoder_s *decoder);
int openh264_decoder_streamoff(struct openh264_decoder_s *decoder);
int openh264_decoder_enqueue(struct openh264_decoder_s *decoder,
                             void *data, int64_t pts, int size);
int openh264_decoder_dequeue(struct openh264_decoder_s *decoder,
                             void *data, int64_t *pts, uint32_t *size);

#endif /* __ARCH_SIM_SRC_SIM_SIM_OPENH264DEC_H */

