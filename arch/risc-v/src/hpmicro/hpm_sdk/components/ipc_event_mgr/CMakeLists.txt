# Copyright (c) 2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

sdk_inc(.)
sdk_src(hpm_ipc_event_mgr.c)

add_subdirectory_ifdef(CONFIG_IPC_EVENT_MGR_MBX mbx)
