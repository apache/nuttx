# ##############################################################################
# mm/kmm_heap/CMakeLists.txt
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

# Kernel heap allocator

if(CONFIG_MM_KERNEL_HEAP)

  set(SRCS
      kmm_initialize.c
      kmm_addregion.c
      kmm_malloc_size.c
      kmm_brkaddr.c
      kmm_calloc.c
      kmm_extend.c
      kmm_free.c
      kmm_mallinfo.c
      kmm_malloc.c
      kmm_memalign.c
      kmm_realloc.c
      kmm_zalloc.c
      kmm_heapmember.c
      kmm_memdump.c)

  if(CONFIG_DEBUG_MM)
    list(APPEND SRCS kmm_checkcorruption.c)
  endif()

  target_sources(mm PRIVATE ${SRCS})

endif()
