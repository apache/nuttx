# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

string(TOUPPER ${CONFIG_CODEC} CONFIG_CODEC_UPPER)
string(TOLOWER ${CONFIG_CODEC} CONFIG_CODEC_LOWER)
if((NOT ${CONFIG_CODEC_LOWER} STREQUAL "sgtl5000") AND (NOT ${CONFIG_CAMERA_LOWER} STREQUAL "wm8960"))
message(FATAL_ERROR "${CONFIG_CODEC} is not supported, only \"sgtl5000\" or \"wm8960\" is supported")
else()
sdk_compile_definitions(-DCONFIG_CODEC_${CONFIG_CODEC_UPPER}=1)
add_subdirectory(${CONFIG_CODEC_LOWER})
endif()

