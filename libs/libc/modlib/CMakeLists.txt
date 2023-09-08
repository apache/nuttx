# ##############################################################################
# libs/libc/modlib/CMakeLists.txt
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

if(CONFIG_LIBC_MODLIB)

  list(
    APPEND
    SRCS
    modlib_bind.c
    modlib_depend.c
    modlib_init.c
    modlib_iobuffer.c
    modlib_load.c
    modlib_loadhdrs.c
    modlib_read.c
    modlib_registry.c
    modlib_sections.c
    modlib_symbols.c
    modlib_symtab.c
    modlib_uninit.c
    modlib_unload.c
    modlib_verify.c)

  list(APPEND SRCS modlib_globals.S)

  target_sources(c PRIVATE ${SRCS})

endif()
