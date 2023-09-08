# ##############################################################################
# boards/arm/stm32/omnibusf4/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_spi.c stm32_userleds.c)

if(CONFIG_SENSORS_MPU60X0)
  list(APPEND SRCS stm32_mpu6000.c)
endif()

if(CONFIG_VIDEO_MAX7456)
  list(APPEND SRCS stm32_max7456.c)
endif()

if(CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_NETDEVICES)
  list(APPEND SRCS stm32_netinit.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS stm32_reset.c)
  endif()
  if(CONFIG_BOARDCTL_IOCTL)
    list(APPEND SRCS stm32_ioctl.c)
  endif()
endif()

if(CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pm.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_timer.c)
endif()

if(CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
