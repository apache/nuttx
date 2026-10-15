############################################################################
# net/bt/audio/Kconfig.mpl
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

# Bluetooth Audio - Media player configuration options

#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: Apache-2.0
#

config BT_MPL
	bool "Support for media player"
	help
	  Enables support for media player
	  Note that the provided media player is a sample that only provides a
	  mock-up with no actual media being played.
	  For real media playback, the sample must be extended, hooked up to a
	  real media player or replaced with a real media player.

if BT_MPL

config BT_MPL_MEDIA_PLAYER_NAME
	string "Media Player Name"
	default "Player0"
	help
	  Use this option to set the name of the media player.

config BT_MPL_MEDIA_PLAYER_NAME_MAX
	int "Max length of media player name"
	default 20
	range 1 255
	help
	  Sets the maximum number of bytes (including the null termination) of
	  the name of the media player.

config BT_MPL_ICON_URL
	string "Media player Icon URL"
	default "http://server.some.where/path/icon.png"
	help
	  Use this option to set the URL of the Media Player Icon.

config BT_MPL_ICON_URL_MAX
	int "Max length of media player icon URL"
	default 40
	range 1 255
	help
	  Sets the maximum number of bytes (including the null termination) of
	  the media player icon URL.

config BT_MPL_TRACK_TITLE_MAX
	int "Max length of the title of a track"
	default 40
	range 1 255
	help
	  Sets the maximum number of bytes (including the null termination) of
	  the title of any track in the media player.

config BT_MPL_SEGMENT_NAME_MAX
	int "Max length of the name of a track segment"
	default 25
	range 1 255
	help
	  Sets the maximum number of bytes (including the null termination)
	  of the name of any track segment in the media player.

config BT_MPL_GROUP_TITLE_MAX
	int "Max length of the title of a group of tracks"
	default BT_MPL_TRACK_TITLE_MAX
	range 1 255
	help
	  Sets the maximum number of bytes (including the null termination) of
	  the title of any group in the media player.

config BT_MPL_OBJECTS
	bool "Support for media player objects"
	depends on BT_OTS
	# TODO: Temporarily depends also on BT_MCS, to avoid issues with the
	# bt_mcs_get_ots() call
	depends on BT_MCS
	help
	  Enables support for objects in the media player
	  Objects are used to give/get more information about e.g. media tracks.
	  Requires the Object Transfer Service

if BT_MPL_OBJECTS

config BT_MPL_MAX_OBJ_SIZE
	int "Total memory size to use for storing the content of objects"
	default 127
	range 0 65536
	help
	  Sets the total memory size (in octets) to use for storing the content of objects.
	  This is used for the total memory pool buffer size from which memory
	  is allocated when sending object content.

config BT_MPL_ICON_BITMAP_SIZE
	int "Media player Icon bitmap object size"
	default 127
	help
	  This option sets the maximum size (in octets) of the icon object.

config BT_MPL_TRACK_MAX_SIZE
	int "Maximum size for a track object"
	default 127
	help
	  This option sets the maximum size (in octets) of a track object.

endif # BT_MPL_OBJECTS

endif # BT_MPL
