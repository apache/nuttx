# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(${HPM_SOC})
sdk_inc(ip)
add_subdirectory(${HPM_SOC})

if(NOT "${SOC_LINKER_SCRIPT}" STREQUAL "")
    set(LINKER_SCRIPT ${SOC_LINKER_SCRIPT} PARENT_SCOPE)
endif()
