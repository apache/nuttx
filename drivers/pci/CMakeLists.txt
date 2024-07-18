# ##############################################################################
# drivers/pci/CMakeLists.txt
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

if(CONFIG_PCI)

  set(SRCS pci.c pci_ecam.c pci_drivers.c)

  if(CONFIG_PCI_QEMU_TEST)
    list(APPEND SRCS pci_qemu_test.c)
  endif()

  if(CONFIG_PCI_QEMU_EDU)
    list(APPEND SRCS pci_qemu_edu.c)
  endif()

  if(CONFIG_PCI_IVSHMEM)
    list(APPEND SRCS pci_ivshmem.c)
  endif()

  if(CONFIG_PCI_UIO_IVSHMEM)
    list(APPEND SRCS pci_uio_ivshmem.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})

endif() # CONFIG_PCI

if(CONFIG_PCI_ENDPOINT)

  set(SRCS pci_epc.c pci_epc_mem.c pci_epf.c)

  if(CONFIG_PCI_QEMU_EPC)
    list(APPEND SRCS pci_qemu_epc.c)
  endif() # CONFIG_PCI_QEMU_EPC

  if(CONFIG_PCI_EPF_TEST)
    list(APPEND SRCS pci_epf_test.c)
  endif() # CONFIG_PCI_EPF_TEST

  if(CONFIG_PCI_EP_TEST)
    list(APPEND SRCS pci_ep_test.c)
  endif() # CONFIG_PCI_ENDPOINT_TEST

  target_sources(drivers PRIVATE ${SRCS})

endif() # CONFIG_PCI_ENDPOINT
