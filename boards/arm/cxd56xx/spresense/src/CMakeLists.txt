# ##############################################################################
# boards/arm/cxd56xx/spresense/src/CMakeLists.txt
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

set(SRCS)

set(SRCS cxd56_main.c cxd56_clock.c cxd56_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS cxd56_appinit.c)
  list(APPEND SRCS cxd56_power.c)
endif()

if(CONFIG_BOARDCTL_IOCTL)
  list(APPEND SRCS cxd56_ioctl.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS cxd56_leds.c)
elseif(CONFIG_USERLED)
  list(APPEND SRCS cxd56_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS cxd56_buttons.c)
endif()

if(CONFIG_CXD56_GPIO_IRQ)
  list(APPEND SRCS cxd56_gpioif.c)
endif()

if(CONFIG_CXD56_PWM)
  list(APPEND SRCS cxd56_pwm.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS cxd56_spi.c)
endif()

if(CONFIG_CXD56_SDIO)
  list(APPEND SRCS cxd56_sdcard.c)
endif()

if(CONFIG_CXD56_SDCARD_AUTOMOUNT)
  list(APPEND SRCS cxd56_automount.c)
endif()

if(CONFIG_CXD56_GAUGE)
  list(APPEND SRCS cxd56_gauge.c)
endif()

if(CONFIG_CXD56_CHARGER)
  list(APPEND SRCS cxd56_charger.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS cxd56_composite.c)
endif()

if(CONFIG_MODEM_ALT1250)
  list(APPEND SRCS cxd56_alt1250_power.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_CXD56_USE_SYSBUS)
  set(LDSCRIPT ramconfig-new.ld)
else()
  set(LDSCRIPT ramconfig.ld)
endif()

# TODO: make this the default and then allow boards to redefine
set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/${LDSCRIPT}")
