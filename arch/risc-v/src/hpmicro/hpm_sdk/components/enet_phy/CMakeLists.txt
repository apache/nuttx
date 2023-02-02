# Copyright (c) 2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)

add_subdirectory_ifdef(CONFIG_ENET_PHY_RTL8211 rtl8211)
add_subdirectory_ifdef(CONFIG_ENET_PHY_RTL8201 rtl8201)
add_subdirectory_ifdef(CONFIG_ENET_PHY_DP83867 dp83867)
add_subdirectory_ifdef(CONFIG_ENET_PHY_DP83848 dp83848)