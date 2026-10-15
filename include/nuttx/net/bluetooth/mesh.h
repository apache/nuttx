/****************************************************************************
 * include/nuttx/net/bluetooth/mesh.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Bluetooth Mesh Profile APIs.
 */

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <nuttx/net/bluetooth/compat/net_buf.h>

#include <nuttx/net/bluetooth/mesh/keys.h>
#include <nuttx/net/bluetooth/mesh/msg.h>
#include <nuttx/net/bluetooth/mesh/access.h>
#include <nuttx/net/bluetooth/mesh/main.h>
#include <nuttx/net/bluetooth/mesh/cfg.h>
#include <nuttx/net/bluetooth/mesh/cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/health_srv.h>
#include <nuttx/net/bluetooth/mesh/blob_srv.h>
#include <nuttx/net/bluetooth/mesh/cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/health_cli.h>
#include <nuttx/net/bluetooth/mesh/blob_cli.h>
#include <nuttx/net/bluetooth/mesh/blob_io_flash.h>
#include <nuttx/net/bluetooth/mesh/priv_beacon_srv.h>
#include <nuttx/net/bluetooth/mesh/priv_beacon_cli.h>
#include <nuttx/net/bluetooth/mesh/dfu_srv.h>
#include <nuttx/net/bluetooth/mesh/dfd_srv.h>
#include <nuttx/net/bluetooth/mesh/dfu_cli.h>
#include <nuttx/net/bluetooth/mesh/dfu_metadata.h>
#include <nuttx/net/bluetooth/mesh/proxy.h>
#include <nuttx/net/bluetooth/mesh/heartbeat.h>
#include <nuttx/net/bluetooth/mesh/cdb.h>
#include <nuttx/net/bluetooth/mesh/rpr_cli.h>
#include <nuttx/net/bluetooth/mesh/rpr_srv.h>
#include <nuttx/net/bluetooth/mesh/sar_cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/sar_cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/op_agg_srv.h>
#include <nuttx/net/bluetooth/mesh/op_agg_cli.h>
#include <nuttx/net/bluetooth/mesh/large_comp_data_srv.h>
#include <nuttx/net/bluetooth/mesh/large_comp_data_cli.h>
#include <nuttx/net/bluetooth/mesh/od_priv_proxy_srv.h>
#include <nuttx/net/bluetooth/mesh/od_priv_proxy_cli.h>
#include <nuttx/net/bluetooth/mesh/sol_pdu_rpl_srv.h>
#include <nuttx/net/bluetooth/mesh/sol_pdu_rpl_cli.h>
#include <nuttx/net/bluetooth/mesh/brg_cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/brg_cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/statistic.h>

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H */
