/****************************************************************************
 * arch/sim/src/sim/up_video_host.h
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

#ifndef __ARCH_SIM_SRC_SIM_UP_VIDEO_HOST_H
#define __ARCH_SIM_SRC_SIM_UP_VIDEO_HOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <linux/videodev2.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool video_host_is_available(const char *video_host_dev_path);
int video_host_init(const char *video_host_dev_path);
int video_host_uninit(void);
int video_host_data_init(void);
int video_host_start_capture(int reqbuf_count);
int video_host_stop_capture(void);
int video_host_dq_buf(uint8_t **addr, struct timeval *ts);
int video_host_enq_buf(uint8_t *addr, uint32_t size);
int video_host_set_buf(uint8_t *addr, uint32_t size);
int video_host_set_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer);
int video_host_try_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer);

#endif /* __ARCH_SIM_SRC_SIM_UP_VIDEO_HOST_H */
