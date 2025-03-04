# ##############################################################################
# drivers/power/battery/CMakeLists.txt
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

# Include power management sources

set(SRCS)

# Add battery charger drivers

if(CONFIG_BATTERY_CHARGER)

  list(APPEND SRCS battery_charger.c)

  # Add the MCP73871 battery charger driver

  if(CONFIG_MCP73871)
    list(APPEND SRCS mcp73871.c)
  endif()

  # Add the BQ2425x I2C-based battery charger driver

  if(CONFIG_I2C_BQ2425X)
    list(APPEND SRCS bq2425x.c)
  endif()

  # Add the BQ2429x I2C-based battery charger driver

  if(CONFIG_I2C_BQ2429X)
    list(APPEND SRCS bq2429x.c)
  endif()

  # Add the axp202 I2C-based battery charger driver

  if(CONFIG_I2C_AXP202)
    list(APPEND SRCS axp202.c)
  endif()

endif()

# Add battery gauge drivers

if(CONFIG_BATTERY_GAUGE)

  list(APPEND SRCS battery_gauge.c)

  # Add the MAX1704x I2C-based battery gauge driver

  if(CONFIG_I2C_MAX1704X)
    list(APPEND SRCS max1704x.c)
  endif()

  # Add the bq27426 I2C-based battery gauge driver

  if(CONFIG_BQ27426)
    list(APPEND SRCS bq27426.c)
  endif()

endif()

# Add battery monitor drivers

if(CONFIG_BATTERY_MONITOR)
  list(APPEND SRCS battery_monitor.c)
  # Add I2C-based battery monitor drivers

  # Add the BQ769x0 I2C-based battery monitor driver
  if(CONFIG_I2C_BQ769X0)
    list(APPEND SRCS bq769x0.c)
  endif()
endif()

if(CONFIG_GOLDFISH_BATTERY)
  list(APPEND SRCS goldfish_battery.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
