############################################################################
# net/bt/services/bas/Kconfig.bas
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

# Bluetooth GATT Battery service

# Copyright (c) 2018 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

config BT_BAS
	bool "GATT Battery service"

config BT_BAS_BLS
	bool "Battery Level Status"
	help
	  Enable this option to include Battery Level Status Characteristic.

if BT_BAS_BLS

config BT_BAS_BLS_IDENTIFIER_PRESENT
	bool "Battery Level Identifier Present"
	help
	  Enable this option if the Battery Level Identifier is present.

config BT_BAS_BLS_BATTERY_LEVEL_PRESENT
	bool "Battery Level Present"
	help
	  Enable this option if the Battery Level is present.

config BT_BAS_BLS_ADDITIONAL_STATUS_PRESENT
	bool "Additional Battery Status Present"
	help
	  Enable this option if Additional Battery Status information is present.

config BT_BAS_BCS
	bool "Battery Critical Status"
	help
	  Enable this option to include Battery Critical Status Characteristic.
endif
