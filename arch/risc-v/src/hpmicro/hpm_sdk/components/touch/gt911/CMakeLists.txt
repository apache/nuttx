# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)
sdk_src(hpm_gt911.c)
sdk_src_ifdef(CONFIG_HPM_TOUCH hpm_touch_gt911.c)
