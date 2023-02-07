# ##############################################################################
# drivers/power/CMakeLists.txt
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

if(CONFIG_PM)

  list(
    APPEND
    SRCS
    pm_initialize.c
    pm_activity.c
    pm_changestate.c
    pm_checkstate.c
    pm_register.c
    pm_unregister.c
    pm_autoupdate.c
    pm_governor.c
    pm_lock.c)

  if(CONFIG_PM_PROCFS)
    list(APPEND SRCS pm_procfs.c)
  endif()

  # Governor implementations

  if(CONFIG_PM_GOVERNOR_ACTIVITY)

    list(APPEND SRCS activity_governor.c)

  endif()

  if(CONFIG_PM_GOVERNOR_GREEDY)

    list(APPEND SRCS greedy_governor.c)

  endif()

endif()

# Add switched-mode power supply support

if(CONFIG_DRIVERS_SMPS)

  list(APPEND SRCS smps.c)

endif()

# Add powerled support

if(CONFIG_DRIVERS_POWERLED)

  list(APPEND SRCS powerled.c)

endif()

if(CONFIG_REGULATOR)

  list(APPEND SRCS regulator.c)

endif()

if(CONFIG_REGULATOR_RPMSG)

  list(APPEND SRCS regulator_rpmsg.c)

endif()

if(CONFIG_REGULATOR_GPIO)

  list(APPEND SRCS regulator_gpio.c)

endif()

# Add battery charger drivers

if(CONFIG_BATTERY_CHARGER)

  list(APPEND SRCS battery_charger.c)

  # Add the MCP73871 battery charger driver

  if(CONFIG_MCP73871)
    list(APPEND SRCS mcp73871.c)
  endif()

  # Add I2C-based battery charger drivers

  if(CONFIG_I2C)

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

endif()

# Add battery gauge drivers

if(CONFIG_BATTERY_GAUGE)

  list(APPEND SRCS battery_gauge.c)

  # Add I2C-based battery gauge drivers

  if(CONFIG_I2C)

    # Add the MAX1704x I2C-based battery gauge driver

    if(CONFIG_I2C_MAX1704X)
      list(APPEND SRCS max1704x.c)
    endif()

    # Add the bq27426 I2C-based battery gauge driver

    if(CONFIG_BQ27426)
      list(APPEND SRCS bq27426.c)
    endif()

  endif()

endif()

# Add battery monitor drivers

if(CONFIG_BATTERY_MONITOR)
  list(APPEND SRCS battery_monitor.c)
  # Add I2C-based battery monitor drivers

  if(CONFIG_I2C)
    # Add the BQ769x0 I2C-based battery monitor driver
    if(CONFIG_I2C_BQ769X0)
      list(APPEND SRCS bq769x0.c)
    endif()
  endif()
endif()

target_sources(drivers PRIVATE ${SRCS})
