# ##############################################################################
# binfmt/libelf/CMakeLists.txt
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

if(CONFIG_ELF)
  set(SRCS)

  # ELF library

  list(
    APPEND
    SRCS
    libelf_bind.c
    libelf_init.c
    libelf_addrenv.c
    libelf_iobuffer.c
    libelf_load.c
    libelf_read.c
    libelf_sections.c
    libelf_symbols.c
    libelf_uninit.c
    libelf_unload.c
    libelf_verify.c)

  if(CONFIG_ELF_COREDUMP)
    list(APPEND SRCS libelf_coredump.c)
    target_include_directories(binfmt PRIVATE ${NUTTX_DIR}/sched)
  endif()

  if(CONFIG_BINFMT_CONSTRUCTORS)
    list(APPEND SRCS libelf_ctors.c libelf_dtors.c)
  endif()

  target_sources(binfmt PRIVATE ${SRCS})
endif()
