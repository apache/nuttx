# ##############################################################################
# drivers/wireless/CMakeLists.txt
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

if(CONFIG_DRIVERS_WIRELESS)
  set(SRCS)

  nuttx_add_subdirectory()

  # Include wireless drivers

  if(CONFIG_WL_CC1101)
    list(APPEND SRCS cc1101.c ISM1_868MHzGFSK100kbps.c ISM2_905MHzGFSK250kbps.c
         ISM2_433MHzMSK500kbps.c)
  endif()

  if(CONFIG_WL_GS2200M)
    list(APPEND SRCS gs2200m.c)
  endif()

  if(CONFIG_WL_NRF24L01)
    list(APPEND SRCS nrf24l01.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
