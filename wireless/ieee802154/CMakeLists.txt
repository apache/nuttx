# ##############################################################################
# wireless/ieee802154/CMakeLists.txt
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

if(CONFIG_WIRELESS_IEEE802154)
  set(SRCS ieee802154_primitive.c)

  if(CONFIG_IEEE802154_MAC)

    list(
      APPEND
      SRCS
      mac802154.c
      mac802154_assoc.c
      mac802154_disassoc.c
      mac802154_bind.c
      mac802154_data.c
      mac802154_get_mhrlen.c
      mac802154_getset.c
      mac802154_gts.c
      mac802154_ioctl.c
      mac802154_orphan.c
      mac802154_poll.c
      mac802154_purge.c
      mac802154_reset.c
      mac802154_rxenable.c
      mac802154_scan.c
      mac802154_start.c
      mac802154_sync.c)

    # Include wireless devices build support

    if(CONFIG_IEEE802154_MACDEV)
      list(APPEND SRCS mac802154_device.c)
    endif()

    if(CONFIG_IEEE802154_NETDEV)
      list(APPEND SRCS mac802154_netdev.c)
    endif()
  endif()

  if(CONFIG_IEEE802154_LOOPBACK)
    list(APPEND SRCS mac802154_loopback.c)
  endif()

  target_sources(wireless PRIVATE ${SRCS})
endif()
