# ##############################################################################
# sched/module/CMakeLists.txt
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

if(CONFIG_MODULE)
  set(SRCS mod_insmod.c mod_rmmod.c mod_modsym.c mod_modhandle.c)

  if(CONFIG_FS_PROCFS AND NOT CONFIG_FS_PROCFS_EXCLUDE_MODULE)
    list(APPEND SRCS mod_procfs.c)
  endif()

  target_sources(sched PRIVATE ${SRCS})
endif()
