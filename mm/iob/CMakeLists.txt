# ##############################################################################
# mm/iob/CMakeLists.txt
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

if(CONFIG_MM_IOB)

  set(SRCS
      iob_add_queue.c
      iob_alloc.c
      iob_alloc_qentry.c
      iob_clone.c
      iob_concat.c
      iob_copyin.c
      iob_copyout.c
      iob_contig.c
      iob_free.c
      iob_free_chain.c
      iob_free_qentry.c
      iob_free_queue.c
      iob_initialize.c
      iob_pack.c
      iob_peek_queue.c
      iob_remove_queue.c
      iob_statistics.c
      iob_trimhead.c
      iob_trimhead_queue.c
      iob_trimtail.c
      iob_navail.c
      iob_free_queue_qentry.c
      iob_tailroom.c
      iob_get_queue_info.c
      iob_reserve.c
      iob_update_pktlen.c
      iob_count.c)

  if(CONFIG_IOB_NOTIFIER)
    list(APPEND SRCS iob_notifier.c)
  endif()

  if(CONFIG_DEBUG_FEATURES)
    list(APPEND SRCS iob_dump.c)
  endif()

  if(CONFIG_IOB_SECTION)
    target_compile_options(mm PRIVATE -DIOB_SECTION=CONFIG_IOB_SECTION)
  endif()
  target_sources(mm PRIVATE ${SRCS})

endif()
