# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

string(TOUPPER ${CONFIG_CAMERA} CONFIG_CAMERA_UPPER)
string(TOLOWER ${CONFIG_CAMERA} CONFIG_CAMERA_LOWER)
if((NOT ${CONFIG_CAMERA_LOWER} STREQUAL "ov7725") AND (NOT ${CONFIG_CAMERA_LOWER} STREQUAL "ov5640"))
message(FATAL_ERROR "${CONFIG_CAMERA} is not supported, only \"ov7725\" or \"ov5640\" is supported")
else()
sdk_compile_definitions(-DCONFIG_CAMERA_${CONFIG_CAMERA_UPPER}=1)
sdk_inc(.)
add_subdirectory(${CONFIG_CAMERA_LOWER})
endif()

