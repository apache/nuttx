############################################################################
# drivers/analog/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

set(SRCS)

if (CONFIG_DAC)
  # Include the common DAC character driver
  list(APPEND SRCS dac.c)

  # Include DAC device drivers

  if (CONFIG_DAC_AD5410)
    list(APPEND SRCS ad5410.c)
  endif()

  if (CONFIG_DAC7571)
    list(APPEND SRCS dac7571.c)
  endif()

  if (CONFIG_DAC7554)
    list(APPEND SRCS dac7554.c)
  endif()
endif()

# Check for COMP devices

if (CONFIG_COMP)
  # Include the common COMP character driver
  list(APPEND SRCS comp.c)
endif()

# Check for OPAMP devices

if (CONFIG_OPAMP)
  # Include the common OPAMP character driver
  list(APPEND SRCS opamp.c)
endif()

# Check for ADC devices

if (CONFIG_ADC)
  # Include the common ADC character driver
  list(APPEND SRCS adc.c)

  # Amplifiers/multiplexers

  if (CONFIG_ADC_PGA11X)
    list(APPEND SRCS pga11x.c)
  endif()

  # Include ADC device drivers

  if (CONFIG_ADC_ADS1242)
    list(APPEND SRCS ads1242.c)
  endif()

  if (CONFIG_ADC_ADS125X)
    list(APPEND SRCS ads1255.c)
  endif()

  if (CONFIG_ADC_ADS7828)
    list(APPEND SRCS ads7828.c)
  endif()

  if (CONFIG_ADC_MAX1161X)
    list(APPEND SRCS max1161x.c)
  endif()

  if (CONFIG_ADC_LTC1867L)
    list(APPEND SRCS ltc1867l.c)
  endif()
endif()

if (CONFIG_LMP92001)
  list(APPEND SRCS lmp92001.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
