/****************************************************************************
 * arch/sim/src/sim/sim_hostvideo.h
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

#ifndef __ARCH_SIM_SRC_SIM_SIM_HOSTVIDEO_H
#define __ARCH_SIM_SRC_SIM_SIM_HOSTVIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct host_video_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool host_video_is_available(const char *host_video_dev_path);
struct host_video_dev_s *host_video_init(const char *host_video_dev_path);
int host_video_uninit(struct host_video_dev_s *vdev);
int host_video_start_capture(struct host_video_dev_s *vdev);
int host_video_stop_capture(struct host_video_dev_s *vdev);
int host_video_dqbuf(struct host_video_dev_s *vdev, uint8_t *addr,
                     uint32_t size);
int host_video_set_fmt(struct host_video_dev_s *vdev,
                       uint16_t width, uint16_t height, uint32_t fmt,
                       uint32_t denom, uint32_t numer);
int host_video_try_fmt(struct host_video_dev_s *vdev,
                       uint16_t width, uint16_t height, uint32_t fmt,
                       uint32_t denom, uint32_t numer);

#endif /* __ARCH_SIM_SRC_SIM_SIM_HOSTVIDEO_H */
