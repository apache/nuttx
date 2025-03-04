# ##############################################################################
# drivers/video/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# Include video drivers

if(CONFIG_DRIVERS_VIDEO)

  set(SRCS)

  if(CONFIG_VIDEO_FB)
    list(APPEND SRCS fb.c)
  endif()

  if(CONFIG_VIDEO_STREAM)
    list(APPEND SRCS v4l2_core.c video_framebuff.c v4l2_cap.c v4l2_m2m.c)
  endif()

  # These video drivers depend on I2C support

  if(CONFIG_I2C)

    if(CONFIG_VIDEO_ISX012)
      list(APPEND SRCS isx012.c)
    endif()

    if(CONFIG_VIDEO_ISX019)
      list(APPEND SRCS isx019.c)
    endif()

    if(CONFIG_VIDEO_OV2640)
      list(APPEND SRCS ov2640.c)
    endif()

  endif()

  if(CONFIG_GOLDFISH_FB)
    list(APPEND SRCS goldfish_fb.c)
  endif()

  if(CONFIG_GOLDFISH_GPU_FB)
    list(APPEND SRCS goldfish_gpu_fb.c)
  endif()

  if(CONFIG_GOLDFISH_CAMERA)
    list(APPEND SRCS goldfish_camera.c)
  endif()
  # These video drivers depend on SPI support

  if(CONFIG_SPI)

    if(CONFIG_VIDEO_MAX7456)
      list(APPEND SRCS max7456.c)
    endif()

  endif()

  target_sources(drivers PRIVATE ${SRCS})

  nuttx_add_subdirectory()
endif()
