# ##############################################################################
# drivers/coresight/CMakeLists.txt
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

if(CONFIG_CORESIGHT)
  set(SRCS coresight_core.c coresight_common.c)

  if(CONFIG_CORESIGHT_ETB)
    list(APPEND SRCS coresight_etb.c)
  endif()

  if(CONFIG_CORESIGHT_ETM_VERSION STREQUAL "v3")
    list(APPEND SRCS coresight_etm3.c)
  endif()

  if(CONFIG_CORESIGHT_ETM_VERSION STREQUAL "v4")
    list(APPEND SRCS coresight_etm4.c)
  endif()

  if(CONFIG_CORESIGHT_FUNNEL)
    list(APPEND SRCS coresight_funnel.c)
  endif()

  if(CONFIG_CORESIGHT_REPLICATOR)
    list(APPEND SRCS coresight_replicator.c)
  endif()

  if(CONFIG_CORESIGHT_STM)
    list(APPEND SRCS coresight_stm.c)
  endif()

  if(CONFIG_CORESIGHT_TMC)
    list(APPEND SRCS coresight_tmc_core.c coresight_tmc_etf.c
         coresight_tmc_etr.c)
  endif()

  if(CONFIG_CORESIGHT_TPIU)
    list(APPEND SRCS coresight_tpiu.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
