############################################################################
# net/bt/services/Kconfig.cts
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

# Copyright (c) 2024 Croxel Inc.
# SPDX-License-Identifier: Apache-2.0

config BT_CTS
	bool "GATT Current Time service"

if BT_CTS

config BT_CTS_HELPER_API
	bool "Helper APIs to encode and decode CTS formatted time"

endif
