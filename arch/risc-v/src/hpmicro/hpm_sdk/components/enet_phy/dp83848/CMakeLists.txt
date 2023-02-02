# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_compile_definitions(-DRGMII=0)
sdk_compile_definitions(-D__USE_DP83848=1)

sdk_inc(.)
sdk_inc(../)
sdk_src(hpm_dp83848.c)
