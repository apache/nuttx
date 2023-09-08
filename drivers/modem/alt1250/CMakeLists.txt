# ##############################################################################
# drivers/modem/alt1250/CMakeLists.txt
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

if(CONFIG_MODEM_ALT1250)
  set(SRCS
      alt1250.c
      altcom_pkt.c
      altcom_hdlr.c
      altcom_hdlr_pdn.c
      altcom_hdlr_radio.c
      altcom_hdlr_net.c
      altcom_hdlr_psave.c
      altcom_hdlr_sim.c
      altcom_hdlr_pin.c
      altcom_hdlr_socket.c
      altcom_hdlr_sms.c
      altcom_hdlr_firmware.c
      altcom_hdlr_log.c
      altcom_hdlr_other.c
      altcom_lwm2m_hdlr.c
      altcom_errno.c
      altmdm.c
      altmdm_event.c
      altmdm_spi.c
      altmdm_timer.c)

  target_sources(drivers PRIVATE ${SRCS})
endif()
