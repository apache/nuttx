############################################################################
# drivers/power/CMakeLists.txt
#
#   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

# Include power management sources

if (CONFIG_PM)
  list(APPEND SRCS
    pm_initialize.c pm_activity.c pm_changestate.c pm_checkstate.c
    pm_register.c pm_unregister.c)

  # Governor implementations

  if (CONFIG_PM_GOVERNOR_ACTIVITY)
    list(APPEND SRCS activity_governor.c)
  endif()

  if (CONFIG_PM_GOVERNOR_GREEDY)
    list(APPEND SRCS greedy_governor.c)
  endif()
endif()

# Add switched-mode power supply support

if (CONFIG_DRIVERS_SMPS)
  list(APPEND SRCS smps.c)
endif()

# Add powerled support

if (CONFIG_DRIVERS_POWERLED)
  list(APPEND SRCS powerled.c)
endif()

# Add motor driver

if (CONFIG_DRIVERS_MOTOR)
  list(APPEND SRCS motor.c)
endif()

# Add battery charger drivers

if (CONFIG_BATTERY_CHARGER)
  list(APPEND SRCS battery_charger.c)

  # Add the MCP73871 battery charger driver
  if (CONFIG_MCP73871)
    list(APPEND SRCS mcp73871.c)
  endif()

  # Add I2C-based battery charger drivers
  if (CONFIG_I2C)
    # Add the BQ2425x I2C-based battery charger driver

    if (CONFIG_I2C_BQ2425X)
      list(APPEND SRCS bq2425x.c)
    endif()

    # Add the BQ2429x I2C-based battery charger driver
    if (CONFIG_I2C_BQ2429X)
      list(APPEND SRCS bq2429x.c)
    endif()
  endif()
endif()

# Add battery gauge drivers

if (CONFIG_BATTERY_GAUGE)
  list(APPEND SRCS battery_gauge.c)

  # Add I2C-based battery gauge drivers

  if (CONFIG_I2C)
    # Add the MAX1704x I2C-based battery gauge driver

    if (CONFIG_I2C_MAX1704X)
      list(APPEND SRCS max1704x.c)
    endif()

    # Add the bq27426 I2C-based battery gauge driver

    if (CONFIG_BQ27426)
      list(APPEND SRCS bq27426.c)
    endif()
  endif()
endif()

# Add battery monitor drivers

if (CONFIG_BATTERY_MONITOR)
  list(APPEND SRCS battery_monitor.c)

  # Add I2C-based battery monitor drivers

  if (CONFIG_I2C)
    # Add the BQ769x0 I2C-based battery monitor driver
    if (CONFIG_I2C_BQ769X0)
      list(APPEND SRCS bq769x0.c)
    endif()
  endif()
endif()

target_sources(drivers PRIVATE ${SRCS})
