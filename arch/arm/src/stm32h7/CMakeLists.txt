############################################################################
# arch/arm/src/stm32h7/CMakeLists.txt
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Required STM32H7 files

set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS stm32_allocateheap.c stm32_exti_gpio.c stm32_gpio.c stm32_irq.c)
list(APPEND CHIP_CSRCS stm32_start.c stm32_rcc.c stm32_lowputc.c stm32_serial.c)

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_timerisr.c)
endif()

if("${CONFIG_STM32H7_DMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_dma.c)
endif()

if("${CONFIG_STM32H7_I2C}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_i2c.c)
endif()

if("${CONFIG_STM32H7_SPI}" STREQUAL "y")
	list(APPEND CHIP_CSRCS stm32_spi.c)
endif()

set_parent_srcs(${CHIP_CSRCS})
