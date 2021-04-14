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

set(CSRCS)

# Include power management sources

if("${CONFIG_PM}" STREQUAL "y")

	list(APPEND CSRCS pm_activity.c pm_changestate.c pm_checkstate.c pm_initialize.c)
	list(APPEND CSRCS pm_register.c pm_unregister.c pm_update.c)

	# Include power management in the build

	set_parent_srcs(${CSRCS})

endif()

# Add switched-mode power supply support

if("${CONFIG_DRIVERS_SMPS}" STREQUAL "y")

	list(APPEND CSRCS smps.c)
	
	set_parent_srcs(${CSRCS})

endif()

# Add powerled support

if("${CONFIG_DRIVERS_POWERLED}" STREQUAL "y")

	list(APPEND CSRCS powerled.c)

endif()

# Add motor driver

if("${CONFIG_DRIVERS_MOTOR}" STREQUAL "y")

	list(APPEND CSRCS motor.c)

endif()

# Add battery charger drivers

if("${CONFIG_BATTERY_CHARGER}" STREQUAL "y")

	list(APPEND CSRCS battery_charger.c)
	
	# Add I2C-based battery charger drivers
	
	if("${CONFIG_I2C}" STREQUAL "y")
	
		# Add the BQ2425x I2C-based battery charger driver
		
		if("${CONFIG_I2C_BQ2425X}" STREQUAL "y")
			list(APPEND CSRCS bq2425x.c)
		endif()
		
		# Add the BQ2429x I2C-based battery charger driver
		
		if("${CONFIG_I2C_BQ2429X}" STREQUAL "y")
			list(APPEND CSRCS bq2429x.c)
		endif()
	
	endif()

endif()

# Add battery gauge drivers

if("${CONFIG_BATTERY_GAUGE}" STREQUAL "y")

	list(APPEND CSRCS battery_gauge.c)
	
	# Add I2C-based battery gauge drivers
	
	if("${CONFIG_I2C}" STREQUAL "y")
	
		# Add the MAX1704x I2C-based battery guage driver
		
		if("${CONFIG_I2C_MAX1704X}" STREQUAL "y")
			list(APPEND CSRCS max1704x.c)
		endif()
	
	endif()

endif()

# Include power management in the build

set_parent_srcs(${CSRCS})
