# ##############################################################################
# arch/arm/src/cmake/elf.cmake
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

nuttx_elf_compile_options(-fvisibility=hidden -mlong-calls)

nuttx_mod_compile_options(-fvisibility=hidden -mlong-calls)

nuttx_elf_compile_options_ifdef(CONFIG_UNWINDER_ARM -fno-unwind-tables
                                -fno-asynchronous-unwind-tables)

if(CONFIG_FDPIC)

  # An FDPIC module is a shared object whose two segments the loader places
  # independently.  The stock compiler emits correct FDPIC objects for both C
  # and C++, so only the link needs the arm-uclinuxfdpiceabi linker: the stock
  # one carries the armelf emulation alone and would turn every import into a
  # jump slot where the ABI wants a function descriptor.

  if(NOT FDPIC_CROSSDEV)
    set(FDPIC_CROSSDEV arm-uclinuxfdpiceabi-)
  endif()

  set(CMAKE_ELF_LD
      "${FDPIC_CROSSDEV}ld"
      CACHE INTERNAL "Linker for FDPIC modules")

  nuttx_elf_compile_options(-mfdpic -fPIC -Wa,--noexecstack)

  nuttx_elf_link_options(-m armelf_linux_fdpiceabi -shared -z now)

elseif(CONFIG_PIC)

  # An ELF module needs r9 as its PIC base, so it must not also have the
  # register fixed: GCC rejects that pair with "unable to use 'r9' for PIC
  # register".  This mirrors CELFFLAGS in common/Toolchain.defs, which filters
  # --fixed-r9 back out of the inherited CFLAGS for the same reason.

  nuttx_elf_compile_options(-mpic-register=r9)

  nuttx_elf_link_options(--unresolved-symbols=ignore-in-object-files
                         --emit-relocs)

endif()

# Not with CONFIG_PIC: there the module is linked as an executable, which is
# what common/Toolchain.defs does too.

if(CONFIG_BINFMT_ELF_RELOCATABLE AND NOT CONFIG_PIC)
  nuttx_elf_link_options(-r)
endif()

nuttx_mod_link_options(-r)

nuttx_elf_link_options_ifdef(CONFIG_BUILD_KERNEL -Bstatic)

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  if("${CMAKE_LD}" MATCHES "gcc$")
    nuttx_elf_link_options(-Wl,--gc-sections)
  else()
    nuttx_elf_link_options(--gc-sections)
  endif()
endif()

nuttx_elf_link_options(-e _start)
