# ##############################################################################
# sched/signal/CMakeLists.txt
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

if(NOT CONFIG_DISABLE_ALL_SIGNALS)
  set(SRCS
      sig_procmask.c
      sig_suspend.c
      sig_kill.c
      sig_tgkill.c
      sig_queue.c
      sig_waitinfo.c
      sig_timedwait.c
      sig_lowest.c
      sig_notification.c
      sig_dispatch.c
      sig_pause.c
      sig_nanosleep.c
      sig_usleep.c
      sig_sleep.c
      sig_ppoll.c
      sig_pselect.c)
endif()

if(CONFIG_ENABLE_ALL_SIGNALS)
  list(
    APPEND
    SRCS
    sig_action.c
    sig_allocpendingsigaction.c
    sig_cleanup.c
    sig_deliver.c
    sig_findaction.c
    sig_initialize.c
    sig_pending.c
    sig_releasependingsigaction.c
    sig_releasependingsignal.c
    sig_removependingsignal.c
    sig_unmaskpendingsignal.c)
endif()

if(CONFIG_SIG_DEFAULT)
  list(APPEND SRCS sig_default.c)
endif()

target_sources(sched PRIVATE ${SRCS})
