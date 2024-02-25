# ##############################################################################
# drivers/analog/CMakeLists.txt
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

if(CONFIG_DAC)
  # Include the common DAC character driver
  list(APPEND SRCS dac.c)

  # Include DAC device drivers

  if(CONFIG_DAC_AD5410)
    list(APPEND SRCS ad5410.c)
  endif()

  if(CONFIG_DAC7571)
    list(APPEND SRCS dac7571.c)
  endif()

  if(CONFIG_DAC7554)
    list(APPEND SRCS dac7554.c)
  endif()

  if(CONFIG_MCP48XX)
    list(APPEND SRCS mcp48xx.c)
  endif()

endif()

# Check for COMP devices

if(CONFIG_COMP)
  # Include the common COMP character driver
  list(APPEND SRCS comp.c)
endif()

# Check for OPAMP devices

if(CONFIG_OPAMP)
  # Include the common OPAMP character driver
  list(APPEND SRCS opamp.c)
endif()

# Check for ADC devices

if(CONFIG_ADC)
  # Include the common ADC character driver
  list(APPEND SRCS adc.c)

  # Amplifiers/multiplexers

  if(CONFIG_ADC_PGA11X)
    list(APPEND SRCS pga11x.c)
  endif()

  # Include ADC device drivers

  if(CONFIG_ADC_ADS1242)
    list(APPEND SRCS ads1242.c)
  endif()

  if(CONFIG_ADC_ADS125X)
    list(APPEND SRCS ads1255.c)
  endif()

  if(CONFIG_ADC_ADS7828)
    list(APPEND SRCS ads7828.c)
  endif()

  if(CONFIG_ADC_MAX1161X)
    list(APPEND SRCS max1161x.c)
  endif()

  if(CONFIG_ADC_LTC1867L)
    list(APPEND SRCS ltc1867l.c)
  endif()

  if(CONFIG_ADC_HX711)
    list(APPEND SRCS hx711.c)
  endif()
endif()

if(CONFIG_LMP92001)
  list(APPEND SRCS lmp92001.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
