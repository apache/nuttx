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

  # Say which linker is missing rather than failing later with a command that
  # cannot be run.

  find_program(FDPIC_LD "${FDPIC_CROSSDEV}ld")

  if(NOT FDPIC_LD)
    message(
      FATAL_ERROR
        "CONFIG_FDPIC needs ${FDPIC_CROSSDEV}ld, which is not on PATH. "
        "It is in the NuttX CI image, and tools/ci/docker/linux/Dockerfile "
        "shows how it is built.  Set FDPIC_CROSSDEV to use a different prefix")
  endif()

  set(CMAKE_ELF_LD
      "${FDPIC_LD}"
      CACHE INTERNAL "Linker for FDPIC modules")

  # GCC before 14 does not pass --fdpic to the assembler for -mfdpic, and the
  # assembler then rejects the FDPIC relocations, so pass it here.
  #
  # With -mlong-calls, GCC turns a call in tail position into a branch to the
  # function descriptor rather than through it, which faults.  Keep it from
  # making tail calls.

  set(FDPIC_FLAGS -mfdpic -fPIC -Wa,--noexecstack -Wa,--fdpic
                  -fno-optimize-sibling-calls)

  nuttx_elf_compile_options(${FDPIC_FLAGS})

  nuttx_elf_link_options(-m armelf_linux_fdpiceabi -shared -z now)

  # A shared library is an FDPIC shared object too, as LDMODULEFLAGS makes it in
  # common/Toolchain.defs

  nuttx_mod_compile_options(${FDPIC_FLAGS})

  nuttx_mod_link_options(-m armelf_linux_fdpiceabi -shared -z now)

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

if(NOT CONFIG_FDPIC)
  nuttx_mod_link_options(-r)
endif()

nuttx_elf_link_options_ifdef(CONFIG_BUILD_KERNEL -Bstatic)

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  if("${CMAKE_LD}" MATCHES "gcc$")
    nuttx_elf_link_options(-Wl,--gc-sections)
  else()
    nuttx_elf_link_options(--gc-sections)
  endif()
endif()

nuttx_elf_link_options(-e _start)
