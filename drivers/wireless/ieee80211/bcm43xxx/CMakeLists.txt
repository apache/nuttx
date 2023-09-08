# ##############################################################################
# drivers/wireless/ieee80211/bcm43xxx/CMakeLists.txt
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
if(CONFIG_IEEE80211_BROADCOM_FULLMAC)
  set(SRCS)

  list(
    APPEND
    SRCS
    bcmf_driver.c
    bcmf_cdc.c
    bcmf_bdc.c
    bcmf_utils.c
    bcmf_netdev.c)

  if(CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
    list(APPEND SRCS mmc_sdio.c bcmf_sdio.c bcmf_core.c bcmf_sdpcm.c)
  endif()

  if(CONFIG_IEEE80211_BROADCOM_BCM43362)
    list(APPEND SRCS bcmf_chip_43362.c)
  endif()

  if(CONFIG_IEEE80211_BROADCOM_BCM43438)
    list(APPEND SRCS bcmf_chip_43438.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
