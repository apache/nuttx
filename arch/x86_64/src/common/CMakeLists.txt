# ##############################################################################
# arch/x86_64/src/common/CMakeLists.txt
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

set(SRCS
    x86_64_allocateheap.c
    x86_64_copystate.c
    x86_64_exit.c
    x86_64_getintstack.c
    x86_64_mdelay.c
    x86_64_initialize.c
    x86_64_modifyreg8.c
    x86_64_modifyreg16.c
    x86_64_modifyreg32.c
    x86_64_nputs.c
    x86_64_switchcontext.c
    x86_64_udelay.c
    x86_64_tcbinfo.c)

if(CONFIG_PCI)
  list(APPEND SRCS x86_64_pci.c)
endif()

if(CONFIG_ARCH_X86_64_ACPI)
  list(APPEND SRCS x86_64_acpi.c)
endif()

target_sources(arch PRIVATE ${SRCS})
