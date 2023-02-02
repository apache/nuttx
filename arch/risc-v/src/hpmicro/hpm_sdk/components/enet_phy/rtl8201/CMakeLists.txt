# Copyright (c) 2021-2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_compile_definitions(-DRGMII=0)
sdk_compile_definitions(-D__USE_RTL8201=1)

sdk_inc(.)
sdk_inc(../)
sdk_src(hpm_rtl8201.c)
