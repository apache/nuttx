# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)
sdk_src(hpm_ft5406.c)
sdk_src_ifdef(CONFIG_HPM_TOUCH hpm_touch_ft5406.c)
