# ##############################################################################
# boards/arm/cxd56xx/drivers/audio/CMakeLists.txt
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

if(CONFIG_CXD56_AUDIO)
  set(SRCS
      cxd56_audio.c
      cxd56_audio_config.c
      cxd56_audio_analog.c
      cxd56_audio_power.c
      cxd56_audio_filter.c
      cxd56_audio_mic.c
      cxd56_audio_volume.c
      cxd56_audio_digital.c
      cxd56_audio_beep.c
      cxd56_audio_irq.c
      cxd56_audio_dma.c
      cxd56_audio_ac_reg.c
      cxd56_audio_bca_reg.c
      cxd56_audio_aca.c
      cxd56_audio_pin.c)

  target_sources(drivers PRIVATE ${SRCS})
  target_include_directories(drivers PRIVATE ${NUTTX_CHIP_ABS_DIR})
endif()
