/****************************************************************************
 * drivers/usbhost/usbhost_uvc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __DRIVERS_USBHOST_USBHOST_UVC_H
#define __DRIVERS_USBHOST_USBHOST_UVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include "uvc_video.h"
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UVC_MAX_FORMATS         16  /* Maximum supported formats */
#define UVC_MAX_FRAMES          16  /* Maximum supported frames */
#define UVC_MAX_ALTS            16  /* Maximum alternate settings tracked */
#define UVC_MAX_DEVICES         4
#define UVC_MAX_FRAME_INTERVALS 8

/* ISO pipeline slot states */

#define UVC_ISO_SLOT_FREE       0
#define UVC_ISO_SLOT_SUBMITTED  1
#define UVC_ISO_SLOT_COMPLETED  2

#define UVC_CTRL_CACHE_MAX      16

/* ISO pipeline depth (number of concurrent transfers) */

#define UVC_ISO_PIPELINE_DEPTH  CONFIG_USBHOST_UVC_ISO_PIPELINE_DEPTH
#define UVC_PACKETS_PER_SLOT    CONFIG_USBHOST_UVC_PACKETS_PER_SLOT

/****************************************************************************
 * Name: struct uvc_frame_info_s
 *
 * Description:
 *   UVC frame information structure
 *
 ****************************************************************************/

struct uvc_frame_info_s
{
  uint8_t frame_index;      /* Frame index in format descriptor */
  uint16_t width;           /* Frame width in pixels */
  uint16_t height;          /* Frame height in pixels */
  uint32_t interval;        /* Default frame interval in 100ns units */

  /* 0=continuous, N=discrete count */

  uint8_t num_interval;

  /* Discrete interval list (100ns units) */

  uint32_t intervals[UVC_MAX_FRAME_INTERVALS];
};

/****************************************************************************
 * Name: struct uvc_format_info_s
 *
 * Description:
 *   UVC format information structure
 *
 ****************************************************************************/

struct uvc_format_info_s
{
  uint8_t format_index;
  uint8_t num_frames;
  uint32_t pixel_format_fourcc;
  struct uvc_frame_info_s frames[UVC_MAX_FRAMES];
};

/****************************************************************************
 * Name: struct uvc_format_list_s
 *
 * Description:
 *   UVC format list structure
 *
 ****************************************************************************/

struct uvc_format_list_s
{
  uint8_t num_formats;
  struct uvc_format_info_s formats[UVC_MAX_FORMATS];
};

/* Per-altstream parameters for the VS interface */

struct uvc_stream_alt_info_s
{
  uint8_t alt;
  uint16_t max_payload;
  uint16_t raw_mxpacketsize;
  uint8_t interval;
};

/****************************************************************************
 * Name: struct uvc_priv_s
 *
 * Description:
 *   UVC USB host class private data structure
 *
 ****************************************************************************/

struct uvc_priv_s
{
  struct usbhost_class_s usbclass;
  atomic_t disconnected;
  atomic_t crefs;

  FAR struct uvc_dev_s *dev;
  int minor;

  uint8_t ctrl_iface;
  uint8_t pu_id;
  uint8_t ct_id;

  uint32_t pu_bmcontrols;
  uint32_t ct_bmcontrols;

  uint8_t stream_iface;
  uint8_t stream_altsetting;

  uint8_t isoc_ep_addr;
  uint16_t max_pkt_size;
  usbhost_ep_t isoin;

  uint8_t stream_alt_count;
  struct uvc_stream_alt_info_s stream_alts[UVC_MAX_ALTS];

  struct uvc_format_list_s format_list;

  struct work_s disconnect_work;
};

struct uvc_ctrl_cache_s
{
  bool queried;
  bool supported;
  uint8_t ctrl_type;
  int64_t minimum;
  int64_t maximum;
  int64_t step;
  int64_t default_value;
};

struct uvc_dev_s;

struct uvc_iso_slot_s
{
  FAR struct uvc_dev_s *dev;
  FAR uint8_t *buffer;
  size_t buflen;
  ssize_t result;
  atomic_t state;
  uint8_t index;
  uint16_t pkt_idx;
};

struct uvc_iso_pipe_s
{
  struct uvc_iso_slot_s slots[UVC_ISO_PIPELINE_DEPTH];
  sem_t completion_sem;
  uint8_t num_slots;
  uint16_t packets_per_xfer;
  uint16_t max_packet;
};

/****************************************************************************
 * Name: struct uvc_dev_s
 *
 * Description:
 *   UVC device state structure
 *
 ****************************************************************************/

struct uvc_dev_s
{
  struct imgdata_s imgdata;
  struct imgsensor_s imgsensor;

  FAR struct uvc_priv_s *device;

  spinlock_t buffer_lock;

  FAR uint8_t *buffer_addr;
  size_t buffer_size;

  uint8_t current_format_idx;
  uint8_t current_frame_idx;

  imgdata_capture_t capture_callback;
  FAR void *capture_arg;

  pid_t capture_tid;
  atomic_t thread_running;
  sem_t capture_exit_sem;

  /* Per-device format arrays (dynamically allocated) */

  FAR struct v4l2_fmtdesc     *fmtdescs;
  FAR struct v4l2_frmsizeenum *frmsizes;
  FAR struct v4l2_frmivalenum *frmintervals;

  /* Recyclable device number from g_devinuse bitmap */

  int minor;

  struct uvc_iso_pipe_s iso_pipe;

  FAR uint8_t *iso_frame_dest;
  size_t iso_frame_dest_size;
  size_t iso_frame_offset;
  bool iso_frame_started;
  bool iso_have_prev_fid;
  uint8_t iso_prev_fid;
  bool iso_frame_error;
  size_t iso_frame_complete_size;

  struct uvc_ctrl_cache_s ctrl_cache[UVC_CTRL_CACHE_MAX];
};

#endif /* __DRIVERS_USBHOST_USBHOST_UVC_H */
