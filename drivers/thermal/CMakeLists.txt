# ##############################################################################
# drivers/thermal/CMakeLists.txt
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

# Include thermal sources

if(CONFIG_THERMAL)
  list(APPEND SRCS thermal_core.c)
endif()

if(CONFIG_THERMAL_GOVERNOR_STEP_WISE)
  list(APPEND SRCS thermal_step_wise.c)
endif()

if(CONFIG_THERMAL_CDEV_CPUFREQ)
  list(APPEND SRCS thermal_cpufreq_cooling.c)
endif()

if(CONFIG_THERMAL_PROCFS)
  list(APPEND SRCS thermal_procfs.c)
endif()

if(CONFIG_THERMAL_DUMMY)
  list(APPEND SRCS thermal_dummy.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
