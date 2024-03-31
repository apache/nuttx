# ##############################################################################
# drivers/virtio/CMakeLists.txt
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
set(SRCS)

if(CONFIG_DRIVERS_VIRTIO)
  list(APPEND SRCS virtio.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_MMIO)
  list(APPEND SRCS virtio-mmio.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_BLK)
  list(APPEND SRCS virtio-blk.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_GPU)
  list(APPEND SRCS virtio-gpu.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_INPUT)
  list(APPEND SRCS virtio-input.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_NET)
  list(APPEND SRCS virtio-net.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_RNG)
  list(APPEND SRCS virtio-rng.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_RPMB)
  list(APPEND SRCS virtio-rpmb.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_SERIAL)
  list(APPEND SRCS virtio-serial.c)
endif()

if(CONFIG_DRIVERS_VIRTIO_SOUND)
  list(APPEND SRCS virtio-snd.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
target_include_directories(drivers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
