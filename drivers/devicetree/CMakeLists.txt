# ##############################################################################
# drivers/devicetree/CMakeLists.txt
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
if(CONFIG_DEVICE_TREE)
  set(SRCS fdt.c)

  if(CONFIG_PCI)
    list(APPEND SRCS fdt_pci.c)
  endif()

  if(CONFIG_DRIVERS_VIRTIO_MMIO)
    list(APPEND SRCS fdt_virtio_mmio.c)
  endif()

  if(CONFIG_MTD_CFI)
    list(APPEND SRCS fdt_cfi.c)
  endif()

  if(CONFIG_IVSHMEM_FLAT)
    list(APPEND SRCS fdt_ivshmem_flat.c)
  endif()

  if(CONFIG_TIMER_SP804 AND CONFIG_TIMER)
    list(APPEND SRCS fdt_sp804.c)
  endif()
  if(CONFIG_TIMER_IMX_GPT)
    list(APPEND SRCS fdt_imx_gpt.c)
  endif()

  if(CONFIG_WATCHDOG_CMSDK)
    list(APPEND SRCS fdt_cmsdk_wdt.c)
  endif()

  if(CONFIG_QEMU_FWCFG)
    list(APPEND SRCS fdt_qemu_fwcfg.c)
  endif()

  if(CONFIG_PWM_SIFIVE)
    list(APPEND SRCS fdt_sifive_pwm.c)
  endif()

  if(CONFIG_IOEXPANDER_PL061)
    list(APPEND SRCS fdt_pl061.c)
  endif()

  if(CONFIG_CAN_CTUCANFD)
    list(APPEND SRCS fdt_ctucanfd.c)
  endif()

  target_include_directories(drivers
                             PRIVATE ${NUTTX_DIR}/libs/libc/fdt/dtc/libfdt)
  target_sources(drivers PRIVATE ${SRCS})
endif()
