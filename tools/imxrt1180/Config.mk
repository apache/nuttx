############################################################################
# tools/imxrt1180/Config.mk
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

# Resolves the location of the pinned NXP EdgeLock Enclave firmware AHAB
# container.  When CONFIG_IMXRT_ELE_FW_DOWNLOAD is enabled, also defines a
# Make target that downloads it into the shared build cache.

ifeq ($(CONFIG_IMXRT_ELE_FW),y)

ELE_FW_URL_BASE = https://raw.githubusercontent.com/nxp-mcuxpresso/mcux-sdk/6f3fd257cdcf978a4d26e7d6e9eed9240037422b/firmware/edgelock
ELE_FW_NAME = mxrt1180b0-ahab-container.img
ELE_FW_PATH := $(strip $(subst ",,$(CONFIG_IMXRT_ELE_FW_PATH)))
IMXRT_ELE_FW_ABS := $(if $(filter /%,$(ELE_FW_PATH)),$(ELE_FW_PATH),$(TOPDIR)/$(ELE_FW_PATH))

ifeq ($(CONFIG_IMXRT_ELE_FW_DOWNLOAD),y)
$(IMXRT_ELE_FW_ABS):
	$(Q) mkdir -p $(dir $@)
	$(call DOWNLOAD,$(ELE_FW_URL_BASE),$(ELE_FW_NAME),$@)
else
$(IMXRT_ELE_FW_ABS):
	$(Q) test -f $@ || \
	  { echo "error: ELE firmware not found at $@"; \
	    echo "       (CONFIG_IMXRT_ELE_FW_DOWNLOAD is disabled, so it must be provided)"; \
	    exit 1; }
endif

endif
