# Copyright (c) 2021-2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(toolchains)

if(NOT DEFINED USE_CUSTOM_STARTUP)
    sdk_gcc_src(toolchains/gcc/start.S)
    sdk_ses_src(toolchains/segger/startup.s)
endif()

sdk_src(
    toolchains/reset.c
    toolchains/trap.c
    system.c
)

sdk_gcc_src(toolchains/gcc/initfini.c)

# soc drivers
sdk_src (
    hpm_sysctl_drv.c
    hpm_l1c_drv.c
    hpm_clock_drv.c
    hpm_otp_drv.c
)

if(${INCLUDE_BOOTHEADER})
    sdk_inc(boot)
    sdk_src(boot/hpm_bootheader.c)
endif()

sdk_nds_compile_options(-mcpu=d45)

set(SOC_LINKER_SCRIPT "" PARENT_SCOPE)
if(NOT DEFINED USE_CUSTOM_LINKER)
    if(${LINK_TO_FLASH})
        if(${INCLUDE_BOOTHEADER})
            if(${FLASH_XIP})
                set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/flash_xip.ld PARENT_SCOPE)
            else()
                if(${FLASH_SDRAM_XIP})
                    set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/flash_sdram_xip.ld PARENT_SCOPE)
                endif()
            endif()
        else()
            if(${FLASH_UF2})
                set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/flash_uf2.ld PARENT_SCOPE)
            else()
                if(${FLASH_SDRAM_UF2})
                    set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/flash_sdram_uf2.ld PARENT_SCOPE)
                else()
                    set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/flash.ld PARENT_SCOPE)
                endif()
            endif()
        endif()
    else()
        if(BUILD_FOR_SECONDARY_CORE)
            set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/ram_core1.ld PARENT_SCOPE)
        else()
            set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/toolchains/gcc/ram.ld PARENT_SCOPE)
        endif()
    endif()
endif()
