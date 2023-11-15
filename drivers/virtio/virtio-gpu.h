/****************************************************************************
 * drivers/virtio/virtio-gpu.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_VIRTIO_VIRTIO_GPU_H
#define __DRIVERS_VIRTIO_VIRTIO_GPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/config.h>
#include <sys/types.h>

#ifdef CONFIG_DRIVERS_VIRTIO_GPU

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Definitions from viogpu.h in OpenBSD src */

#define VIRTIO_GPU_F_VIRGL               (1u << 0)
#define VIRTIO_GPU_F_EDID                (1u << 1)
#define VIRTIO_GPU_F_RESOURCE_UUID       (1u << 2)
#define VIRTIO_GPU_F_RESOURCE_BLOB       (1u << 3)

enum virtio_gpu_ctrl_type
{
  VIRTIO_GPU_UNDEFINED = 0,

  /* 2d commands */

  VIRTIO_GPU_CMD_GET_DISPLAY_INFO = 0x0100,
  VIRTIO_GPU_CMD_RESOURCE_CREATE_2D,
  VIRTIO_GPU_CMD_RESOURCE_UNREF,
  VIRTIO_GPU_CMD_SET_SCANOUT,
  VIRTIO_GPU_CMD_RESOURCE_FLUSH,
  VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D,
  VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING,
  VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING,
  VIRTIO_GPU_CMD_GET_CAPSET_INFO,
  VIRTIO_GPU_CMD_GET_CAPSET,
  VIRTIO_GPU_CMD_GET_EDID,
  VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID,
  VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB,
  VIRTIO_GPU_CMD_SET_SCANOUT_BLOB,

  /* 3d commands */

  VIRTIO_GPU_CMD_CTX_CREATE = 0x0200,
  VIRTIO_GPU_CMD_CTX_DESTROY,
  VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE,
  VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE,
  VIRTIO_GPU_CMD_RESOURCE_CREATE_3D,
  VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D,
  VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D,
  VIRTIO_GPU_CMD_SUBMIT_3D,
  VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB,
  VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB,

  /* Cursor commands */

  VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
  VIRTIO_GPU_CMD_MOVE_CURSOR,

  /* Success responses */

  VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
  VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
  VIRTIO_GPU_RESP_OK_CAPSET_INFO,
  VIRTIO_GPU_RESP_OK_CAPSET,
  VIRTIO_GPU_RESP_OK_EDID,
  VIRTIO_GPU_RESP_OK_RESOURCE_UUID,
  VIRTIO_GPU_RESP_OK_MAP_INFO,

  /* Error responses */

  VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
  VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
  VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
  VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
  VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
  VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,
};

enum virtio_gpu_shm_id
{
  VIRTIO_GPU_SHM_ID_UNDEFINED = 0,
  VIRTIO_GPU_SHM_ID_HOST_VISIBLE
};

#define VIRTIO_GPU_FLAG_FENCE         (1 << 0)
#define VIRTIO_GPU_FLAG_INFO_RING_IDX (1 << 1)

begin_packed_struct struct virtio_gpu_ctrl_hdr
{
  uint32_t type;
  uint32_t flags;
  uint64_t fence_id;
  uint32_t ctx_id;
  uint8_t  ring_idx;
  uint8_t  padding[3];
} end_packed_struct;

/* Data passed in the cursor vq */

begin_packed_struct struct virtio_gpu_cursor_pos
{
  uint32_t scanout_id;
  uint32_t x;
  uint32_t y;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_UPDATE_CURSOR, VIRTIO_GPU_CMD_MOVE_CURSOR */

begin_packed_struct struct virtio_gpu_update_cursor
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_cursor_pos pos;    /* Update & move */
  uint32_t resource_id;                /* Update only */
  uint32_t hot_x;                      /* Update only */
  uint32_t hot_y;                      /* Update only */
  uint32_t padding;
} end_packed_struct;

/* Data passed in the control vq, 2d related */

begin_packed_struct struct virtio_gpu_rect
{
  uint32_t x;
  uint32_t y;
  uint32_t width;
  uint32_t height;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_UNREF */

begin_packed_struct struct virtio_gpu_resource_unref
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_2D: create a 2d resource with a format */

begin_packed_struct struct virtio_gpu_resource_create_2d
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t format;
  uint32_t width;
  uint32_t height;
} end_packed_struct;

/* VIRTIO_GPU_CMD_SET_SCANOUT */

begin_packed_struct struct virtio_gpu_set_scanout
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_rect r;
  uint32_t scanout_id;
  uint32_t resource_id;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_FLUSH */

begin_packed_struct struct virtio_gpu_resource_flush
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_rect r;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D: simple transfer to_host */

begin_packed_struct struct virtio_gpu_transfer_to_host_2d
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_rect r;
  uint64_t offset;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING */

begin_packed_struct struct virtio_gpu_mem_entry
{
  uint64_t addr;
  uint32_t length;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_gpu_resource_attach_backing
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t nr_entries;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING */

begin_packed_struct struct virtio_gpu_resource_detach_backing
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_DISPLAY_INFO */

#define VIRTIO_GPU_MAX_SCANOUTS 16

begin_packed_struct struct virtio_gpu_resp_display_info
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_display_one
    {
      struct virtio_gpu_rect r;
      uint32_t enabled;
      uint32_t flags;
    }
    pmodes[VIRTIO_GPU_MAX_SCANOUTS];
} end_packed_struct;

/* Data passed in the control vq, 3d related */

begin_packed_struct struct virtio_gpu_box
{
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t w;
  uint32_t h;
  uint32_t d;
} end_packed_struct;

/* VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D, VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D */

begin_packed_struct struct virtio_gpu_transfer_host_3d
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_box box;
  uint64_t offset;
  uint32_t resource_id;
  uint32_t level;
  uint32_t stride;
  uint32_t layer_stride;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_3D */

#define VIRTIO_GPU_RESOURCE_FLAG_Y_0_TOP (1 << 0)

begin_packed_struct struct virtio_gpu_resource_create_3d
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t target;
  uint32_t format;
  uint32_t bind;
  uint32_t width;
  uint32_t height;
  uint32_t depth;
  uint32_t array_size;
  uint32_t last_level;
  uint32_t nr_samples;
  uint32_t flags;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_CTX_CREATE */

begin_packed_struct struct virtio_gpu_ctx_create
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t nlen;
  uint32_t padding;
  char debug_name[64];
} end_packed_struct;

/* VIRTIO_GPU_CMD_CTX_DESTROY */

begin_packed_struct struct virtio_gpu_ctx_destroy
{
  struct virtio_gpu_ctrl_hdr hdr;
} end_packed_struct;

/* VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE, VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE */

begin_packed_struct struct virtio_gpu_ctx_resource
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_SUBMIT_3D */

begin_packed_struct struct virtio_gpu_cmd_submit
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t size;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_GET_CAPSET_INFO */

#define VIRTIO_GPU_CAPSET_VIRGL  1
#define VIRTIO_GPU_CAPSET_VIRGL2 2

begin_packed_struct struct virtio_gpu_get_capset_info
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t capset_index;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_CAPSET_INFO */

begin_packed_struct struct virtio_gpu_resp_capset_info
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t capset_id;
  uint32_t capset_max_version;
  uint32_t capset_max_size;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_GET_CAPSET */

begin_packed_struct struct virtio_gpu_get_capset
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t capset_id;
  uint32_t capset_version;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_CAPSET */

begin_packed_struct struct virtio_gpu_resp_capset
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint8_t capset_data[];
} end_packed_struct;

/* VIRTIO_GPU_CMD_GET_EDID */

begin_packed_struct struct virtio_gpu_cmd_get_edid
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t scanout;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_EDID */

begin_packed_struct struct virtio_gpu_resp_edid
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t size;
  uint32_t padding;
  uint8_t edid[1024];
} end_packed_struct;

#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)

begin_packed_struct struct virtio_gpu_config
{
  uint32_t events_read;
  uint32_t events_clear;
  uint32_t num_scanouts;
  uint32_t num_capsets;
} end_packed_struct;

/* simple formats for fbcon/X use */

enum virtio_gpu_formats
{
  VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM  = 1,
  VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM  = 2,
  VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM  = 3,
  VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM  = 4,

  VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM  = 67,
  VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM  = 68,

  VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM  = 121,
  VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM  = 134,
};

/* VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID */

begin_packed_struct struct virtio_gpu_resource_assign_uuid
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_RESOURCE_UUID */

begin_packed_struct struct virtio_gpu_resp_resource_uuid
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint8_t uuid[16];
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB */

#define VIRTIO_GPU_BLOB_MEM_GUEST             0x0001
#define VIRTIO_GPU_BLOB_MEM_HOST3D            0x0002
#define VIRTIO_GPU_BLOB_MEM_HOST3D_GUEST      0x0003

#define VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE     0x0001
#define VIRTIO_GPU_BLOB_FLAG_USE_SHAREABLE    0x0002
#define VIRTIO_GPU_BLOB_FLAG_USE_CROSS_DEVICE 0x0004

begin_packed_struct struct virtio_gpu_resource_create_blob
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;

  /* Zero is invalid blob mem */

  uint32_t blob_mem;
  uint32_t blob_flags;
  uint32_t nr_entries;
  uint64_t blob_id;
  uint64_t size;

  /* sizeof(nr_entries * virtio_gpu_mem_entry) bytes follow */
} end_packed_struct;

/* VIRTIO_GPU_CMD_SET_SCANOUT_BLOB */

begin_packed_struct struct virtio_gpu_set_scanout_blob
{
  struct virtio_gpu_ctrl_hdr hdr;
  struct virtio_gpu_rect r;
  uint32_t scanout_id;
  uint32_t resource_id;
  uint32_t width;
  uint32_t height;
  uint32_t format;
  uint32_t padding;
  uint32_t strides[4];
  uint32_t offsets[4];
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB */

begin_packed_struct struct virtio_gpu_resource_map_blob
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
  uint64_t offset;
} end_packed_struct;

/* VIRTIO_GPU_RESP_OK_MAP_INFO */

#define VIRTIO_GPU_MAP_CACHE_MASK     0x0f
#define VIRTIO_GPU_MAP_CACHE_NONE     0x00
#define VIRTIO_GPU_MAP_CACHE_CACHED   0x01
#define VIRTIO_GPU_MAP_CACHE_UNCACHED 0x02
#define VIRTIO_GPU_MAP_CACHE_WC       0x03

begin_packed_struct struct virtio_gpu_resp_map_info
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t map_info;
  uint32_t padding;
} end_packed_struct;

/* VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB */

begin_packed_struct struct virtio_gpu_resource_unmap_blob
{
  struct virtio_gpu_ctrl_hdr hdr;
  uint32_t resource_id;
  uint32_t padding;
} end_packed_struct;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int virtio_register_gpu_driver(void);

int virtio_gpu_fb_register(int display);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_GPU */
#endif /* __DRIVERS_VIRTIO_VIRTIO_GPU_H */
