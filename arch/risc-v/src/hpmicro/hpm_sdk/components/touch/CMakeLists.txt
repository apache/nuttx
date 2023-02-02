# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

string(TOUPPER ${CONFIG_TOUCH} CONFIG_TOUCH_UPPER)
string(TOLOWER ${CONFIG_TOUCH} CONFIG_TOUCH_LOWER)
if((NOT ${CONFIG_TOUCH_LOWER} STREQUAL "ft5406") AND (NOT ${CONFIG_TOUCH_LOWER} STREQUAL "gt911"))
message(FATAL_ERROR "${CONFIG_TOUCH} is not supported, only \"gt911\" or \"ft5406\" is supported")
else()
sdk_compile_definitions(-DCONFIG_TOUCH_${CONFIG_TOUCH_UPPER}=1)
sdk_inc(.)
add_subdirectory(${CONFIG_TOUCH_LOWER})
endif()

