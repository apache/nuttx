# ##############################################################################
# arch/sim/src/sim/macos/CMakeLists.txt
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

add_library(sim_avf_backend OBJECT EXCLUDE_FROM_ALL
                                   sim_host_avfoundation_backend.m)

set_property(TARGET sim_avf_backend PROPERTY INCLUDE_DIRECTORIES "")
set_property(TARGET sim_avf_backend PROPERTY COMPILE_OPTIONS "")
set_property(TARGET sim_avf_backend PROPERTY COMPILE_DEFINITIONS "")
set_property(TARGET sim_avf_backend PROPERTY INTERFACE_INCLUDE_DIRECTORIES "")
set_property(TARGET sim_avf_backend PROPERTY INTERFACE_COMPILE_OPTIONS "")
set_property(TARGET sim_avf_backend PROPERTY INTERFACE_COMPILE_DEFINITIONS "")

target_include_directories(sim_avf_backend PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(sim_avf_backend
                       PRIVATE $<$<COMPILE_LANGUAGE:OBJC>:-fobjc-arc>)
