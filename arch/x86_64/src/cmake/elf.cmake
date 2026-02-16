# ##############################################################################
# arch/x86_64/src/cmake/elf.cmake
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

# Loadable and ELF module settings

nuttx_elf_compile_options(-fvisibility=hidden -fno-pic -mcmodel=large)

nuttx_mod_compile_options(-fvisibility=hidden)

nuttx_elf_link_options(-r)

nuttx_mod_link_options(-r)

nuttx_elf_link_options_ifdef(CONFIG_DEBUG_OPT_UNUSED_SECTIONS --gc-sections)

nuttx_elf_link_options(-e __start)
