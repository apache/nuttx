# ##############################################################################
# boards/arm/cxd56xx/drivers/sensors/CMakeLists.txt
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

set(SRCS)

if(CONFIG_SENSORS_AK09912_SCU)
  list(APPEND SRC ak09912_scu.c)
endif()

if(CONFIG_SENSORS_APDS9930_SCU)
  list(APPEND SRC apds9930_scu.c)
endif()

if(CONFIG_SENSORS_BH1721FVC_SCU)
  list(APPEND SRC bh1721fvc_scu.c)
endif()

if(CONFIG_SENSORS_BH1745NUC_SCU)
  list(APPEND SRC bh1745nuc_scu.c)
endif()

if(CONFIG_SENSORS_BM1383GLV_SCU)
  list(APPEND SRC bm1383glv_scu.c)
endif()

if(CONFIG_SENSORS_BM1422GMV_SCU)
  list(APPEND SRC bm1422gmv_scu.c)
endif()

if(CONFIG_SENSORS_BMI160_SCU)
  list(APPEND SRC bmi160_scu.c)
endif()

if(CONFIG_SENSORS_BMP280_SCU)
  list(APPEND SRC bmp280_scu.c)
endif()

if(CONFIG_SENSORS_KX022_SCU)
  list(APPEND SRC kx022_scu.c)
endif()

if(CONFIG_SENSORS_LT1PA01_SCU)
  list(APPEND SRC lt1pa01_scu.c)
endif()

if(CONFIG_SENSORS_RPR0521RS_SCU)
  list(APPEND SRC rpr0521rs_scu.c)
endif()

if(CONFIG_SENSORS_CXD5610_GNSS)
  list(APPEND SRC cxd5610_gnss.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
