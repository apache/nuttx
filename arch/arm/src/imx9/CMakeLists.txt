# ##############################################################################
# arch/arm/src/imx9/CMakeLists.txt
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

set(SRCS
    imx9_allocateheap.c
    imx9_start.c
    imx9_clockconfig.c
    imx9_gpio.c
    imx9_iomuxc.c
    imx9_irq.c
    imx9_timerisr.c
    imx9_idle.c)

if(CONFIG_IMX9_SCMI)
  list(APPEND SRCS imx9_scmi.c)
  # NXP SDK SCMI interface for pinctrl and clocking
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS imx9_rsctable.c imx9_rptun.c)
endif()

if(CONFIG_IMX9_MU)
  list(APPEND SRCS imx9_mu.c)
endif()

if(CONFIG_ARM_MPU)
  list(APPEND SRCS imx9_mpuinit.c)
endif()

if(CONFIG_IMX9_FLEXCAN)
  list(APPEND SRCS imx9_flexcan.c)
endif()

if(CONFIG_IMX9_LPUART)
  list(APPEND SRCS imx9_lpuart.c imx9_lowputc.c)
endif()

if(CONFIG_IMX9_GPIO_IRQ)
  list(APPEND SRCS imx9_gpioirq.c)
endif()

if(CONFIG_IMX9_FLEXIO_PWM)
  list(APPEND SRCS imx9_flexio_pwm.c)
endif()

if(CONFIG_IMX9_TPM_PWM)
  list(APPEND SRCS imx9_tpm_pwm.c)
endif()

if(CONFIG_IMX9_USBDEV)
  list(APPEND SRCS imx9_usbdev.c)
endif()

if(CONFIG_IMX9_LPI2C)
  list(APPEND SRCS imx9_lpi2c.c)
endif()

if(CONFIG_IMX9_LPSPI)
  list(APPEND SRCS imx9_lpspi.c)
endif()

if(CONFIG_IMX9_EDMA)
  list(APPEND SRCS imx9_edma.c)
endif()

target_sources(arch PRIVATE ${SRCS})
