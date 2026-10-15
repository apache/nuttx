############################################################################
# net/bt/host/CMakeLists.txt
#
# Copyright (c) ByteDance Ltd. and/or its affiliates.
#
# Author: chao an <anchao.archer@bytedance.com>
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

############################################################################
# net/bt/host/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT_HOST)
  # === Core sources (compiled unconditionally for BT_HCI_HOST) ===
  set(HOST_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/uuid.c
    ${CMAKE_CURRENT_LIST_DIR}/addr.c
    ${CMAKE_CURRENT_LIST_DIR}/buf.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_core.c
    ${CMAKE_CURRENT_LIST_DIR}/hci_common.c
    ${CMAKE_CURRENT_LIST_DIR}/id.c
    ${CMAKE_CURRENT_LIST_DIR}/data.c
  )

  # === Conditional sources ===
  if(CONFIG_BT_BROADCASTER)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/adv.c)
  endif()

  if(CONFIG_BT_OBSERVER)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/scan.c)
  endif()

  if(CONFIG_BT_HOST_CRYPTO)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/crypto_psa.c)
  endif()

  if(CONFIG_BT_ECC)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/ecc.c)
  endif()

  if(CONFIG_BT_HOST_CCM)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/aes_ccm.c)
  endif()

  if(CONFIG_BT_LONG_WQ)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/long_wq.c)
  endif()

  if(CONFIG_BT_SETTINGS)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/settings.c)
  endif()

  if(CONFIG_BT_MONITOR)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/monitor.c)
  endif()

  if(CONFIG_BT_HCI_RAW)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/hci_raw.c)
  endif()

  # Connection-related sources (CONFIG_BT_CONN or CONFIG_BT_ISO)
  if(CONFIG_BT_CONN OR CONFIG_BT_ISO)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/conn.c)

    if(CONFIG_BT_CONN)
      list(APPEND HOST_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/l2cap.c
        ${CMAKE_CURRENT_LIST_DIR}/att.c
        ${CMAKE_CURRENT_LIST_DIR}/gatt.c
      )

      if(CONFIG_BT_GATT_GAP_SVC_VALIDATE)
        list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/gatt_gap_svc_validate.c)
      endif()

      if(CONFIG_BT_SMP)
        list(APPEND HOST_SRCS
          ${CMAKE_CURRENT_LIST_DIR}/smp.c
          ${CMAKE_CURRENT_LIST_DIR}/keys.c
        )
      else()
        list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/smp_null.c)
      endif()
    endif()

    if(CONFIG_BT_ISO)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/iso.c)
    endif()

    if(CONFIG_BT_CHANNEL_SOUNDING)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/cs.c)
    endif()

    if(CONFIG_BT_DF)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/direction.c)
    endif()
  endif()

  # === Classic BR/EDR sources ===
  if(CONFIG_BT_CLASSIC)
    list(APPEND HOST_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/classic/br.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/conn_br.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/keys_br.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/l2cap_br.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/sdp.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/ssp.c
      ${CMAKE_CURRENT_LIST_DIR}/classic/sco.c
    )

    if(CONFIG_BT_A2DP)
      list(APPEND HOST_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/classic/a2dp.c
        ${CMAKE_CURRENT_LIST_DIR}/classic/a2dp_codec_sbc.c
      )
    endif()

    if(CONFIG_BT_AVDTP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/avdtp.c)
    endif()

    if(CONFIG_BT_AVRCP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/avrcp.c)
    endif()

    if(CONFIG_BT_AVCTP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/avctp.c)
    endif()

    if(CONFIG_BT_RFCOMM)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/rfcomm.c)
    endif()

    if(CONFIG_BT_BIP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/bip.c)
    endif()

    if(CONFIG_BT_MAP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/map.c)
    endif()

    if(CONFIG_BT_HFP_HF)
      list(APPEND HOST_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/classic/hfp_hf.c
        ${CMAKE_CURRENT_LIST_DIR}/classic/at.c
      )
    endif()

    if(CONFIG_BT_HFP_AG)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/hfp_ag.c)
    endif()

    if(CONFIG_BT_GOEP)
      list(APPEND HOST_SRCS
        ${CMAKE_CURRENT_LIST_DIR}/classic/obex.c
        ${CMAKE_CURRENT_LIST_DIR}/classic/goep.c
      )
    endif()

    if(CONFIG_BT_DID)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/did.c)
    endif()

    if(CONFIG_BT_HID_DEVICE)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/hid_device.c)
    endif()

    if(CONFIG_BT_HID_HOST)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/hid_host.c)
    endif()

    if(CONFIG_BT_AVRCP_CT_COVER_ART OR CONFIG_BT_AVRCP_TG_COVER_ART)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/avrcp_cover_art.c)
    endif()

    if(CONFIG_BT_PBAP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/pbap.c)
    endif()
  endif()

  # === Shell sources ===
  if(CONFIG_BT_SHELL)
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/bt.c)

    if(CONFIG_BT_CONN)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/gatt.c)
    endif()

    if(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/l2cap.c)
    endif()

    if(CONFIG_BT_ISO)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/iso.c)
    endif()

    if(CONFIG_BT_CHANNEL_SOUNDING)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cs.c)
    endif()

    # Classic shell
    list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/bredr.c)

    if(CONFIG_BT_RFCOMM)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/spp.c)
    endif()

    if(CONFIG_BT_A2DP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/a2dp.c)
    endif()

    if(CONFIG_BT_AVRCP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/avrcp.c)
    endif()

    if(CONFIG_BT_HFP_HF OR CONFIG_BT_HFP_AG)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/hfp.c)
    endif()

    if(CONFIG_BT_GOEP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/goep.c)
    endif()

    if(CONFIG_BT_BIP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/bip.c)
    endif()

    if(CONFIG_BT_HID_DEVICE)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/hid_device.c)
    endif()

    if(CONFIG_BT_HID_HOST)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/hid_host.c)
    endif()

    if(CONFIG_BT_AVRCP_TG_COVER_ART OR CONFIG_BT_AVRCP_CT_COVER_ART)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/avrcp_cover_art.c)
    endif()

    if(CONFIG_BT_PBAP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/pbap.c)
    endif()

    if(CONFIG_BT_MAP)
      list(APPEND HOST_SRCS ${CMAKE_CURRENT_LIST_DIR}/classic/shell/map.c)
    endif()
  endif()

  target_sources(net PRIVATE ${HOST_SRCS})

  # Include paths
  target_include_directories(net PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/classic
    ${CMAKE_CURRENT_LIST_DIR}/..
  )
endif()
