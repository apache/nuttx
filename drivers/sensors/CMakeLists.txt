############################################################################
# drivers/sensors/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2015-2018 Gregory Nutt. All rights reserved.
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

# Include sensor drivers

if("${CONFIG_SENSORS}" STREQUAL "y")

	set(CSRCS)

	if("${CONFIG_SENSORS_HCSR04}" STREQUAL "y")
		list(APPEND CSRCS hc_sr04.c)
	endif()
	
	if("${CONFIG_SENSORS_ADXL345}" STREQUAL "y")
		list(APPEND CSRCS adxl345_base.c)
	endif()
	
	if("${CONFIG_SENSORS_DHTXX}" STREQUAL "y")
		list(APPEND CSRCS dhtxx.c)
	endif()
	
	# These drivers depend on I2C support
	
	if("${CONFIG_I2C}" STREQUAL "y")
	
		if("${CONFIG_SENSORS_APDS9960}" STREQUAL "y")
			list(APPEND CSRCS apds9960.c)
		endif()
		
		if("${CONFIG_SENSORS_AS5048B}" STREQUAL "y")
			list(APPEND CSRCS as5048b.c)
		endif()
		
		if("${CONFIG_SENSORS_KXTJ9}" STREQUAL "y")
			list(APPEND CSRCS kxtj9.c)
		endif()
		
		if("${CONFIG_SENSORS_LIS2DH}" STREQUAL "y")
			list(APPEND CSRCS lis2dh.c)
		endif()
		
		if("${CONFIG_LIS331DL}" STREQUAL "y")
			list(APPEND CSRCS lis331dl.c)
		endif()
		
		if("${CONFIG_SENSORS_LSM303AGR}" STREQUAL "y")
			list(APPEND CSRCS lsm303agr.c)
		endif()
		
		if("${CONFIG_SENSORS_LSM6DSL}" STREQUAL "y")
			list(APPEND CSRCS lsm6dsl.c)
		endif()
		
		if("${CONFIG_SENSORS_LSM9DS1}" STREQUAL "y")
			list(APPEND CSRCS lsm9ds1.c)
		endif()
		
		if("${CONFIG_SENSORS_LPS25H}" STREQUAL "y")
			list(APPEND CSRCS lps25h.c)
		endif()
		
		if("${CONFIG_ADXL345_I2C}" STREQUAL "y")
			list(APPEND CSRCS adxl345_i2c.c)
		endif()
		
		if("${CONFIG_SENSORS_BH1750FVI}" STREQUAL "y")
			list(APPEND CSRCS bh1750fvi.c)
		endif()
		
		if("${CONFIG_SENSORS_BMG160}" STREQUAL "y")
			list(APPEND CSRCS bmg160.c)
		endif()
		
		if("${CONFIG_SENSORS_BMP180}" STREQUAL "y")
			list(APPEND CSRCS bmp180.c)
		endif()
		
		if("${CONFIG_SENSORS_HTS221}" STREQUAL "y")
			list(APPEND CSRCS hts221.c)
		endif()
		
		if("${CONFIG_LM75_I2C}" STREQUAL "y")
			list(APPEND CSRCS lm75.c)
		endif()
		
		if("${CONFIG_SENSORS_LM92}" STREQUAL "y")
			list(APPEND CSRCS lm92.c)
		endif()
		
		if("${CONFIG_SENSORS_MAX44009}" STREQUAL "y")
			list(APPEND CSRCS max44009.c)
		endif()
		
		if("${CONFIG_SENSORS_MB7040}" STREQUAL "y")
			list(APPEND CSRCS mb7040.c)
		endif()
		
		if("${CONFIG_SENSORS_MCP9844}" STREQUAL "y")
			list(APPEND CSRCS mcp9844.c)
		endif()
		
		if("${CONFIG_SENSORS_MLX90393}" STREQUAL "y")
			list(APPEND CSRCS mlx90393.c)
		endif()
		
		if("${CONFIG_SENSORS_MLX90614}" STREQUAL "y")
			list(APPEND CSRCS mlx90614.c)
		endif()
		
		if("${CONFIG_SENSORS_MS58XX}" STREQUAL "y")
			list(APPEND CSRCS ms58xx.c)
		endif()
		
		if("${CONFIG_SENSORS_LTC4151}" STREQUAL "y")
			list(APPEND CSRCS ltc4151.c)
		endif()
		
		if("${CONFIG_SENSORS_INA219}" STREQUAL "y")
			list(APPEND CSRCS ina219.c)
		endif()
		
		if("${CONFIG_SENSORS_INA226}" STREQUAL "y")
			list(APPEND CSRCS ina226.c)
		endif()
		
		if("${CONFIG_SENSORS_INA3221}" STREQUAL "y")
			list(APPEND CSRCS ina3221.c)
		endif()
		
		if("${CONFIG_SENSORS_SHT21}" STREQUAL "y")
			list(APPEND CSRCS sht21.c)
		endif()
		
		if("${CONFIG_SENSORS_T67XX}" STREQUAL "y")
			list(APPEND CSRCS t67xx.c)
		endif()
	
	endif() # CONFIG_I2C
	
	# These drivers depend on SPI support
	
	if("${CONFIG_SPI}" STREQUAL "y")
		if("${CONFIG_ADXL345_SPI}" STREQUAL "y")
			list(APPEND CSRCS adxl345_spi.c)
		endif()
		
		if("${CONFIG_SENSORS_ADXL372}" STREQUAL "y")
			list(APPEND CSRCS adxl372.c)
		endif()
		
		if("${CONFIG_LIS3DSH}" STREQUAL "y")
			list(APPEND CSRCS lis3dsh.c)
		endif()
		
		if("${CONFIG_LIS3DH}" STREQUAL "y")
			list(APPEND CSRCS lis3dh.c)
		endif()
		
		if("${CONFIG_SENSORS_MAX31855}" STREQUAL "y")
			list(APPEND CSRCS max31855.c)
		endif()
		
		if("${CONFIG_SENSORS_MAX6675}" STREQUAL "y")
			list(APPEND CSRCS max6675.c)
		endif()
		
		if("${CONFIG_SENSORS_MPL115A}" STREQUAL "y")
			list(APPEND CSRCS mpl115a.c)
		endif()
		
		if("${CONFIG_SENSORS_LIS3MDL}" STREQUAL "y")
			list(APPEND CSRCS lis3mdl.c)
		endif()
		
		if("${CONFIG_SENSORS_LSM330SPI}" STREQUAL "y")
			list(APPEND CSRCS lsm330_spi.c)
		endif()
		
		if("${CONFIG_SENSORS_L3GD20}" STREQUAL "y")
			list(APPEND CSRCS l3gd20.c)
		endif()
	
	endif() # CONFIG_SPI
	
	# Quadrature encoder upper half
	
	if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
		list(APPEND CSRCS qencoder.c)
	endif()
	
	# Vishay VEML6070
	
	if("${CONFIG_SENSORS_VEML6070}" STREQUAL "y")
		list(APPEND CSRCS veml6070.c)
	endif()
	
	# Sensixs XEN1210
	
	if("${CONFIG_SENSORS_XEN1210}" STREQUAL "y")
		list(APPEND CSRCS xen1210.c)
	endif()
	
	# Zero Cross upper half
	
	if("${CONFIG_SENSORS_ZEROCROSS}" STREQUAL "y")
		list(APPEND CSRCS zerocross.c)
	endif()
	
	# Include sensor driver build support
	
	set_parent_srcs(${CSRCS})

endif() # CONFIG_SENSORS
