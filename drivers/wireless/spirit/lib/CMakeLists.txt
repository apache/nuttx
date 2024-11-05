# ##############################################################################
# drivers/wireless/spirit/lib/CMakeLists.txt
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
target_sources(
  drivers
  PRIVATE spirit_radio.c
          spirit_pktbasic.c
          spirit_pktcommon.c
          spirit_pktmbus.c
          spirit_pktstack.c
          spirit_directrf.c
          spirit_qi.c
          spirit_calibration.c
          spirit_management.c
          spirit_aes.c
          spirit_csma.c
          spirit_linearfifo.c
          spirit_irq.c
          spirit_timer.c
          spirit_gpio.c
          spirit_general.c
          spirit_spi.c)
