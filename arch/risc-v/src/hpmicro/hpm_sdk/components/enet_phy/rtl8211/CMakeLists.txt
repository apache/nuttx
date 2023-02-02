# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_compile_definitions(-DRGMII=1)
sdk_compile_definitions(-D__USE_RTL8211=1)

sdk_inc(.)
sdk_inc(../)
sdk_src(hpm_rtl8211.c)
