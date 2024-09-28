# ##############################################################################
# drivers/power/pm/CMakeLists.txt
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

# Include power management sources

set(SRCS)

if(CONFIG_PM)

  list(
    APPEND
    SRCS
    pm_initialize.c
    pm_activity.c
    pm_changestate.c
    pm_checkstate.c
    pm_register.c
    pm_unregister.c
    pm_autoupdate.c
    pm_governor.c
    pm_lock.c
    pm_idle.c)

  if(CONFIG_PM_PROCFS)
    list(APPEND SRCS pm_procfs.c)
  endif()

  # Governor implementations

  if(CONFIG_PM_GOVERNOR_STABILITY)

    list(APPEND SRCS stability_governor.c)

  endif()

  if(CONFIG_PM_GOVERNOR_ACTIVITY)

    list(APPEND SRCS activity_governor.c)

  endif()

  if(CONFIG_PM_GOVERNOR_GREEDY)

    list(APPEND SRCS greedy_governor.c)

  endif()

  if(CONFIG_PM_RUNTIME)

    list(APPEND SRCS pm_runtime.c)

  endif()

endif()

target_sources(drivers PRIVATE ${SRCS})
