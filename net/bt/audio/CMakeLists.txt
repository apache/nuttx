############################################################################
# net/bt/audio/CMakeLists.txt
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
# net/bt/audio/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
############################################################################

if(CONFIG_BT_AUDIO)
  set(BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/audio.c)

  if(CONFIG_BT_VOCS OR CONFIG_BT_VOCS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/vocs.c)
  endif()
  if(CONFIG_BT_VOCS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/vocs_client.c)
  endif()

  if(CONFIG_BT_AICS OR CONFIG_BT_AICS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/aics.c)
  endif()
  if(CONFIG_BT_AICS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/aics_client.c)
  endif()

  if(CONFIG_BT_VCP_VOL_REND)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/vcp_vol_rend.c)
  endif()
  if(CONFIG_BT_VCP_VOL_CTLR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/vcp_vol_ctlr.c)
  endif()

  if(CONFIG_BT_MICP_MIC_DEV)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/micp_mic_dev.c)
  endif()
  if(CONFIG_BT_MICP_MIC_CTLR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/micp_mic_ctlr.c)
  endif()

  if(CONFIG_BT_CONN)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/ccid.c)
  endif()

  if(CONFIG_BT_CSIP_SET_MEMBER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/csip_set_member.c)
  endif()
  if(CONFIG_BT_CSIP_SET_COORDINATOR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/csip_set_coordinator.c)
  endif()
  if(CONFIG_BT_CSIP_SET_MEMBER OR CONFIG_BT_CSIP_SET_COORDINATOR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/csip_crypto.c)
  endif()

  if(CONFIG_BT_TBS)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/tbs.c)
  endif()
  if(CONFIG_BT_TBS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/tbs_client.c)
  endif()
  if(CONFIG_BT_MCC)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/mcc.c)
  endif()
  if(CONFIG_BT_MCS)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/mcs.c)
  endif()
  if(CONFIG_BT_MPL)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/mpl.c)
  endif()
  if(CONFIG_MCTL)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/media_proxy.c)
  endif()

  if(CONFIG_BT_ASCS)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/ascs.c)
  endif()
  if(CONFIG_BT_PACS)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/pacs.c)
  endif()
  if(CONFIG_BT_BAP_STREAM)
    list(APPEND BT_AUDIO_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/bap_stream.c
      ${CMAKE_CURRENT_LIST_DIR}/codec.c
      ${CMAKE_CURRENT_LIST_DIR}/bap_iso.c
    )
  endif()
  if(CONFIG_BT_BAP_BASE)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_base.c)
  endif()
  if(CONFIG_BT_BAP_UNICAST_SERVER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_unicast_server.c)
  endif()
  if(CONFIG_BT_BAP_UNICAST_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_unicast_client.c)
  endif()
  if(CONFIG_BT_BAP_BROADCAST_SOURCE)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_broadcast_source.c)
  endif()
  if(CONFIG_BT_BAP_BROADCAST_SINK)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_broadcast_sink.c)
  endif()
  if(CONFIG_BT_BAP_SCAN_DELEGATOR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_scan_delegator.c)
  endif()
  if(CONFIG_BT_BAP_BROADCAST_ASSISTANT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/bap_broadcast_assistant.c)
  endif()
  if(CONFIG_BT_CCP_CALL_CONTROL_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/ccp_call_control_client.c)
  endif()
  if(CONFIG_BT_CCP_CALL_CONTROL_SERVER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/ccp_call_control_server.c)
  endif()
  if(CONFIG_BT_HAS)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/has.c)
  endif()
  if(CONFIG_BT_HAS_CLIENT)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/has_client.c)
  endif()
  if(CONFIG_BT_CAP)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_stream.c)
  endif()
  if(CONFIG_BT_CAP_ACCEPTOR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_acceptor.c)
  endif()
  if(CONFIG_BT_CAP_INITIATOR)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_initiator.c)
  endif()
  if(CONFIG_BT_CAP_COMMANDER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_commander.c)
  endif()
  if(CONFIG_BT_CAP_HANDOVER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_handover.c)
  endif()
  if(CONFIG_BT_CAP_INITIATOR_UNICAST OR CONFIG_BT_CAP_COMMANDER)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/cap_common.c)
  endif()
  if(CONFIG_BT_TMAP)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/tmap.c)
  endif()
  if(CONFIG_BT_GMAP)
    list(APPEND BT_AUDIO_SRCS
      ${CMAKE_CURRENT_LIST_DIR}/gmap_client.c
      ${CMAKE_CURRENT_LIST_DIR}/gmap_server.c
    )
  endif()
  if(CONFIG_BT_PBP)
    list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/pbp.c)
  endif()

  if(CONFIG_BT_SHELL)
    if(CONFIG_BT_CCP_CALL_CONTROL_SERVER)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/ccp_call_control_server.c)
    endif()
    if(CONFIG_BT_CCP_CALL_CONTROL_CLIENT)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/ccp_call_control_client.c)
    endif()
    if(CONFIG_BT_VCP_VOL_REND)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/vcp_vol_rend.c)
    endif()
    if(CONFIG_BT_VCP_VOL_CTLR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/vcp_vol_ctlr.c)
    endif()
    if(CONFIG_BT_MICP_MIC_DEV)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/micp_mic_dev.c)
    endif()
    if(CONFIG_BT_MICP_MIC_CTLR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/micp_mic_ctlr.c)
    endif()
    if(CONFIG_BT_CSIP_SET_MEMBER)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/csip_set_member.c)
    endif()
    if(CONFIG_BT_CSIP_SET_COORDINATOR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/csip_set_coordinator.c)
    endif()
    if(CONFIG_BT_TBS)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/tbs.c)
    endif()
    if(CONFIG_BT_TBS_CLIENT)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/tbs_client.c)
    endif()
    if(CONFIG_BT_MPL)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/mpl.c)
    endif()
    if(CONFIG_BT_MCC)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/mcc.c)
    endif()
    if(CONFIG_BT_MCS)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/media_controller.c)
    endif()
    if(CONFIG_BT_HAS_PRESET_SUPPORT)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/has.c)
    endif()
    if(CONFIG_BT_CAP_ACCEPTOR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cap_acceptor.c)
    endif()
    if(CONFIG_BT_CAP_INITIATOR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cap_initiator.c)
    endif()
    if(CONFIG_BT_CAP_COMMANDER)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cap_commander.c)
    endif()
    if(CONFIG_BT_CAP_HANDOVER)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/cap_handover.c)
    endif()
    if(CONFIG_BT_HAS_CLIENT)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/has_client.c)
    endif()
    if(CONFIG_BT_TMAP)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/tmap.c)
    endif()
    if(CONFIG_BT_GMAP)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/gmap.c)
    endif()
    if(CONFIG_BT_BAP_STREAM)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/bap.c)
    endif()
    if(CONFIG_LIBLC3 AND CONFIG_USBD_AUDIO2_CLASS)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/bap_usb.c)
    endif()
    if(CONFIG_BT_BAP_SCAN_DELEGATOR)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/bap_scan_delegator.c)
    endif()
    if(CONFIG_BT_BAP_BROADCAST_ASSISTANT)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/bap_broadcast_assistant.c)
    endif()
    if(CONFIG_BT_PBP)
      list(APPEND BT_AUDIO_SRCS ${CMAKE_CURRENT_LIST_DIR}/shell/pbp.c)
    endif()
  endif()

  target_sources(net PRIVATE ${BT_AUDIO_SRCS})
  target_include_directories(net PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/shell
  )
endif()
