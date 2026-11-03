/****************************************************************************
 * drivers/usbhost/usbhost_uvc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/videoio.h>
#include <time.h>
#include <unistd.h>
#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/video/v4l2_cap.h>

#include "usbhost_uvc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define imgsensor_to_uvc_dev(s) container_of(s, struct uvc_dev_s, imgsensor)

#define UVC_MAX_DEVNO                  32
#define UVC_DEFAULT_FPS                30
#define UVC_FRAME_COMPLETE_THRESHOLD   95  /* percent */
#define UVC_ISO_WAIT_TIMEOUT_SEC       1
#define UVC_SLOT_WAIT_TIMEOUT_SEC      2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uvc_ctrl_map_s
{
  uint32_t id;
  uint8_t  unit_type;
  uint8_t  selector;
  uint8_t  size;
  bool     is_signed;
  uint8_t  ctrl_bit;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uvc_imgdata_init(FAR struct imgdata_s *data);
static int uvc_imgdata_uninit(FAR struct imgdata_s *data);
static int uvc_imgdata_set_buf(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               FAR uint8_t *addr, uint32_t size);
static int
uvc_imgdata_validate_frame_setting(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts,
                                   FAR imgdata_interval_t *interval);
static int uvc_imgdata_start_capture(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval,
                                     FAR imgdata_capture_t callback,
                                     FAR void *arg);
static int uvc_imgdata_stop_capture(FAR struct imgdata_s *data);
static void uvc_iso_restart(FAR struct uvc_dev_s *dev);
static bool uvc_imgsensor_is_available(FAR struct imgsensor_s *sensor);
static int uvc_imgsensor_init(FAR struct imgsensor_s *sensor);
static int uvc_imgsensor_uninit(FAR struct imgsensor_s *sensor);
static const char *
uvc_imgsensor_get_driver_name(FAR struct imgsensor_s *sensor);
static int
uvc_imgsensor_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     uint8_t nr_datafmts,
                                     FAR imgsensor_format_t *datafmts,
                                     FAR imgsensor_interval_t *interval);
static int uvc_imgsensor_start_capture(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_datafmts,
                                       FAR imgsensor_format_t *datafmts,
                                       FAR imgsensor_interval_t *interval);
static int uvc_imgsensor_stop_capture(FAR struct imgsensor_s *sensor,
                                      imgsensor_stream_type_t type);
static int
uvc_imgsensor_get_frame_interval(FAR struct imgsensor_s *sensor,
                                 imgsensor_stream_type_t type,
                                 FAR imgsensor_interval_t *interval);
static int
uvc_imgsensor_get_supported_value(FAR struct imgsensor_s *sensor,
                                  uint32_t id,
                                  FAR imgsensor_supported_value_t *value);
static int uvc_imgsensor_get_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   FAR imgsensor_value_t *value);
static int uvc_imgsensor_set_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   imgsensor_value_t value);
static FAR struct usbhost_class_s *
uvc_create(FAR struct usbhost_hubport_s *hport,
           FAR const struct usbhost_id_s *id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct usbhost_id_s g_uvc_id[1] =
{
  {
    UVC_DEVICE_CLASS,                      /* base     */
    UVC_SUBCLASS_INTERFACE_COLLECTION,     /* subclass */
    0,                                     /* protocol */
    0,                                     /* vid      */
    0                                      /* pid      */
  },
};

static struct usbhost_registry_s g_uvc_registry =
{
  NULL,               /* flink  */
  uvc_create,         /* create */
  1,                  /* nids   */
  g_uvc_id            /* id[]   */
};

static atomic_t g_uvc_devinuse;

static const struct uvc_ctrl_map_s g_uvc_ctrl_map[] =
{
  { IMGSENSOR_ID_BRIGHTNESS,
    UVC_CTRL_UNIT_PU, UVC_PU_BRIGHTNESS_CONTROL,
    2, true, UVC_PU_BRIGHTNESS_BIT },
  { IMGSENSOR_ID_CONTRAST,
    UVC_CTRL_UNIT_PU, UVC_PU_CONTRAST_CONTROL,
    2, false, UVC_PU_CONTRAST_BIT },
  { IMGSENSOR_ID_SATURATION,
    UVC_CTRL_UNIT_PU, UVC_PU_SATURATION_CONTROL,
    2, false, UVC_PU_SATURATION_BIT },
  { IMGSENSOR_ID_HUE,
    UVC_CTRL_UNIT_PU, UVC_PU_HUE_CONTROL,
    2, true, UVC_PU_HUE_BIT },
  { IMGSENSOR_ID_AUTO_WHITE_BALANCE,
    UVC_CTRL_UNIT_PU,
    UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL,
    1, false, UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_BIT },
  { IMGSENSOR_ID_GAMMA,
    UVC_CTRL_UNIT_PU, UVC_PU_GAMMA_CONTROL,
    2, false, UVC_PU_GAMMA_BIT },
  { IMGSENSOR_ID_SHARPNESS,
    UVC_CTRL_UNIT_PU, UVC_PU_SHARPNESS_CONTROL,
    2, false, UVC_PU_SHARPNESS_BIT },
  { IMGSENSOR_ID_EXPOSURE_AUTO,
    UVC_CTRL_UNIT_CT, UVC_CT_AE_MODE_CONTROL,
    1, false, UVC_CT_AE_MODE_BIT },
  { IMGSENSOR_ID_EXPOSURE_ABSOLUTE,
    UVC_CTRL_UNIT_CT,
    UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL,
    4, false, UVC_CT_EXPOSURE_TIME_ABSOLUTE_BIT },
  { IMGSENSOR_ID_FOCUS_ABSOLUTE,
    UVC_CTRL_UNIT_CT, UVC_CT_FOCUS_ABSOLUTE_CONTROL,
    2, false, UVC_CT_FOCUS_ABSOLUTE_BIT },
  { IMGSENSOR_ID_FOCUS_AUTO,
    UVC_CTRL_UNIT_CT, UVC_CT_FOCUS_AUTO_CONTROL,
    1, false, UVC_CT_FOCUS_AUTO_BIT },
  { IMGSENSOR_ID_ZOOM_ABSOLUTE,
    UVC_CTRL_UNIT_CT, UVC_CT_ZOOM_ABSOLUTE_CONTROL,
    2, false, UVC_CT_ZOOM_ABSOLUTE_BIT },
};

static const struct imgdata_ops_s g_uvc_imgdata_ops =
{
  .init                     = uvc_imgdata_init,
  .uninit                   = uvc_imgdata_uninit,
  .set_buf                  = uvc_imgdata_set_buf,
  .validate_frame_setting   = uvc_imgdata_validate_frame_setting,
  .start_capture            = uvc_imgdata_start_capture,
  .stop_capture             = uvc_imgdata_stop_capture,
};

static const struct imgsensor_ops_s g_uvc_imgsensor_ops =
{
  .is_available             = uvc_imgsensor_is_available,
  .init                     = uvc_imgsensor_init,
  .uninit                   = uvc_imgsensor_uninit,
  .get_driver_name          = uvc_imgsensor_get_driver_name,
  .validate_frame_setting   = uvc_imgsensor_validate_frame_setting,
  .start_capture            = uvc_imgsensor_start_capture,
  .stop_capture             = uvc_imgsensor_stop_capture,
  .get_frame_interval       = uvc_imgsensor_get_frame_interval,
  .get_supported_value      = uvc_imgsensor_get_supported_value,
  .get_value                = uvc_imgsensor_get_value,
  .set_value                = uvc_imgsensor_set_value,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t uvc_getle16(FAR const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

static inline uint32_t uvc_getle32(FAR const uint8_t *val)
{
  return (uint32_t)uvc_getle16(&val[2]) << 16 |
         (uint32_t)uvc_getle16(val);
}

static void uvc_putle16(FAR uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff;
  dest[1] = val >> 8;
}

static void uvc_putle32(FAR uint8_t *dest, uint32_t val)
{
  dest[0] = val & 0xff;
  dest[1] = (val >> 8) & 0xff;
  dest[2] = (val >> 16) & 0xff;
  dest[3] = (val >> 24) & 0xff;
}

static FAR const char *uvc_fourcc_to_string(uint32_t fourcc)
{
  switch (fourcc)
    {
      case V4L2_PIX_FMT_YUYV:
        return "YUYV";

      case V4L2_PIX_FMT_UYVY:
        return "UYVY";

      case V4L2_PIX_FMT_NV12:
        return "NV12";

      case V4L2_PIX_FMT_YVU420:
        return "YV12";

      case V4L2_PIX_FMT_YUV420:
        return "I420";

      case V4L2_PIX_FMT_M420:
        return "M420";

      case V4L2_PIX_FMT_GREY:
        return "GREY";

      case V4L2_PIX_FMT_RGB565:
        return "RGB565";

      case V4L2_PIX_FMT_BGR24:
        return "BGR24";

      case V4L2_PIX_FMT_MJPEG:
        return "MJPEG";

      case V4L2_PIX_FMT_JPEG:
        return "JPEG";

      case V4L2_PIX_FMT_H264:
        return "H264";

      case V4L2_PIX_FMT_HEVC:
        return "HEVC";

      default:
        return "UNKNOWN";
    }
}

static uint32_t uvc_convert_pixelformat(uint32_t imgdata_format)
{
  switch (imgdata_format)
    {
      case IMGDATA_PIX_FMT_YUYV:
        return V4L2_PIX_FMT_YUYV;

      case IMGDATA_PIX_FMT_UYVY:
        return V4L2_PIX_FMT_UYVY;

      case IMGDATA_PIX_FMT_JPEG:
        return V4L2_PIX_FMT_JPEG;

      case IMGDATA_PIX_FMT_NV12:
        return V4L2_PIX_FMT_NV12;

      case IMGDATA_PIX_FMT_YUV420P:
        return V4L2_PIX_FMT_YUV420;

      case IMGDATA_PIX_FMT_RGB565:
        return V4L2_PIX_FMT_RGB565;

      default:
        return imgdata_format;
    }
}

static bool uvc_is_compressed_format(uint32_t fourcc)
{
  return fourcc == V4L2_PIX_FMT_JPEG ||
         fourcc == V4L2_PIX_FMT_MJPEG ||
         fourcc == V4L2_PIX_FMT_H264 ||
         fourcc == V4L2_PIX_FMT_HEVC;
}

static int uvc_alloc_devno(void)
{
  int devno;

  for (devno = 0; devno < UVC_MAX_DEVNO; devno++)
    {
      uint32_t bitno = 1 << devno;

      if (!(atomic_or(&g_uvc_devinuse, bitno) & bitno))
        {
          return devno;
        }
    }

  return -ENOMEM;
}

static void uvc_free_devno(int devno)
{
  if (devno >= 0 && devno < UVC_MAX_DEVNO)
    {
      atomic_and(&g_uvc_devinuse, ~(1 << devno));
    }
}

/****************************************************************************
 * Name: uvc_priv_put
 *
 * Description:
 *   Drop one reference to the class instance, freeing it and the device it
 *   owns once the last holder is gone.
 *
 * Input Parameters:
 *   priv - The class instance to release a reference on
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void uvc_priv_put(FAR struct uvc_priv_s *priv)
{
  if (atomic_sub(&priv->crefs, 1) != 1)
    {
      return;
    }

  if (priv->dev != NULL)
    {
      kmm_free(priv->dev);
    }

  kmm_free(priv);
}

static void uvc_init_ctrlreq(FAR struct usb_ctrlreq_s *req,
                             uint8_t type, uint8_t req_code,
                             uint16_t value, uint16_t index,
                             uint16_t len)
{
  memset(req, 0, sizeof(*req));
  req->type = type;
  req->req  = req_code;
  uvc_putle16(req->value, value);
  uvc_putle16(req->index, index);
  uvc_putle16(req->len, len);
}

/* Only the low 4 bytes of bmControls are kept; every control in
 * g_uvc_ctrl_map lives within the first 32 bits. avail bounds the read
 * against the remaining descriptor length.
 */

static uint32_t uvc_read_bmcontrols(FAR const uint8_t *p, int n, int avail)
{
  uint32_t bm = 0;
  int      i;

  if (n > avail)
    {
      n = avail;
    }

  for (i = 0; i < n && i < 4; i++)
    {
      bm |= (uint32_t)p[i] << (8 * i);
    }

  return bm;
}

static bool uvc_ctrl_present(FAR struct uvc_priv_s *priv,
                             FAR const struct uvc_ctrl_map_s *map)
{
  uint32_t bm = map->unit_type == UVC_CTRL_UNIT_CT ?
                priv->ct_bmcontrols : priv->pu_bmcontrols;

  return (bm >> map->ctrl_bit) & 1;
}

static FAR const struct uvc_ctrl_map_s *
uvc_find_ctrl_map(uint32_t id, FAR int *idx)
{
  uint8_t i;

  for (i = 0; i < nitems(g_uvc_ctrl_map); i++)
    {
      if (g_uvc_ctrl_map[i].id == id)
        {
          if (idx)
            {
              *idx = i;
            }

          return &g_uvc_ctrl_map[i];
        }
    }

  return NULL;
}

static int uvc_ctrl_query(FAR struct uvc_dev_s *dev,
                          FAR const struct uvc_ctrl_map_s *map,
                          uint8_t request,
                          FAR void *data)
{
  FAR struct uvc_priv_s        *priv = dev->device;
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  struct usb_ctrlreq_s          req;
  uint8_t                       unit_id;
  uint8_t                       reqtype;

  /* The cleanup worker hands the pipes back as soon as the device is gone,
   * so a control transfer from here on can only fail in the lower half.
   */

  if (atomic_read(&priv->disconnected))
    {
      return -ENODEV;
    }

  unit_id = map->unit_type == UVC_CTRL_UNIT_CT ?
    priv->ct_id : priv->pu_id;

  if (request & USB_DIR_IN)
    {
      reqtype = USB_REQ_DIR_IN |
                USB_REQ_TYPE_CLASS |
                USB_REQ_RECIPIENT_INTERFACE;
    }
  else
    {
      reqtype = USB_REQ_DIR_OUT |
                USB_REQ_TYPE_CLASS |
                USB_REQ_RECIPIENT_INTERFACE;
    }

  uvc_init_ctrlreq(&req, reqtype, request,
                   map->selector << 8,
                   unit_id << 8 | priv->ctrl_iface,
                   map->size);

  if (request & USB_DIR_IN)
    {
      return DRVR_CTRLIN(hport->drvr, hport->ep0, &req, data);
    }

  return DRVR_CTRLOUT(hport->drvr, hport->ep0, &req, data);
}

static int32_t uvc_ctrl_read_value(FAR const struct uvc_ctrl_map_s *map,
                                   FAR const uint8_t *data)
{
  int32_t val = 0;

  switch (map->size)
    {
      case 1:
        val = data[0];
        if (map->is_signed && (val & 0x80))
          {
            val |= 0xffffff00;
          }

        break;

      case 2:
        val = uvc_getle16(data);
        if (map->is_signed && (val & 0x8000))
          {
            val |= 0xffff0000;
          }

        break;

      case 4:
        val = uvc_getle32(data);
        break;
    }

  return val;
}

static void uvc_ctrl_write_value(FAR const struct uvc_ctrl_map_s *map,
                                 FAR uint8_t *data, int32_t val)
{
  switch (map->size)
    {
      case 1:
        data[0] = (uint8_t)(val & 0xff);
        break;

      case 2:
        uvc_putle16(data, (uint16_t)(val & 0xffff));
        break;

      case 4:
        uvc_putle32(data, (uint32_t)val);
        break;
    }
}

static uint32_t uvc_calculate_fps(FAR const imgdata_interval_t *interval,
                                  uint32_t default_interval)
{
  if (interval && interval->denominator > 0 && interval->numerator > 0)
    {
      return interval->denominator / interval->numerator;
    }

  return default_interval > 0 ? (10000000 / default_interval) : 30;
}

/* Clamp requested interval to nearest supported value from
 * frame descriptor's discrete interval list.
 * Returns the clamped interval in 100ns units.
 */

static uint32_t
uvc_try_frame_interval(FAR const struct uvc_frame_info_s *frame,
                       uint32_t requested_interval)
{
  uint32_t best;
  uint32_t best_diff;
  uint8_t  i;

  if (frame->num_interval == 0)
    {
      /* Continuous: return requested interval clamped to descriptor range */

      return requested_interval;
    }

  /* Find nearest supported interval (minimum
   * absolute difference). Works regardless of
   * whether the list is sorted.
   */

  best = frame->intervals[0];
  best_diff = abs((int32_t)(requested_interval - best));

  if (best_diff == 0)
    {
      return best;
    }

  for (i = 1; i < frame->num_interval; i++)
    {
      uint32_t diff = abs((int32_t)(requested_interval -
                                    frame->intervals[i]));

      if (diff == 0)
        {
          best = frame->intervals[i];
          break;
        }

      if (diff < best_diff)
        {
          best_diff = diff;
          best = frame->intervals[i];
        }
    }

  uinfo("UVC: clamp interval %" PRIu32 " -> %" PRIu32 "\n",
        requested_interval, best);
  return best;
}

static int uvc_execute_probe_commit(FAR struct usbhost_hubport_s *hport,
                                    uint8_t vs_intf, uint8_t format_idx,
                                    uint8_t frame_idx, uint32_t fps,
                                    FAR uint8_t *ctrlreq)
{
  struct usb_ctrlreq_s req;
  uint32_t             interval_100ns = fps > 0 ? (10000000u / fps) : 0;
  int                  ret;

  uinfo("UVC: probe fmt=%d frm=%d fps=%" PRIu32
        " intv=%" PRIu32 "\n",
        format_idx, frame_idx, fps, interval_100ns);

  /* Build probe data from scratch instead of GET_CUR.
   * Some devices STALL GET_CUR when no prior SET_CUR has been issued
   * or when the VS interface is at alt-setting 0.
   * Per UVC spec, the host may SET_CUR with desired fields and zeros
   * elsewhere; the device negotiates and returns the result via GET_CUR.
   */

  memset(ctrlreq, 0, UVC_PROBE_COMMIT_SIZE);

  /* Byte 2: bFormatIndex, Byte 3: bFrameIndex */

  ctrlreq[2] = format_idx;
  ctrlreq[3] = frame_idx;

  /* Bytes 4-7: dwFrameInterval (100ns units) */

  if (interval_100ns > 0)
    {
      uvc_putle32(&ctrlreq[4], interval_100ns);
    }

  /* VS_PROBE_CONTROL SET_CUR */

  uvc_init_ctrlreq(&req,
                   USB_REQ_DIR_OUT |
                   USB_REQ_TYPE_CLASS |
                   USB_REQ_RECIPIENT_INTERFACE,
                   UVC_REQUEST_SET_CUR,
                   0x0100, vs_intf, UVC_PROBE_COMMIT_SIZE);

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0,
                     (FAR const struct usb_ctrlreq_s *)&req, ctrlreq);
  if (ret < 0)
    {
      uerr("ERROR: VS_PROBE_CONTROL SET_CUR failed: %d\n", ret);
      return ret;
    }

  /* VS_PROBE_CONTROL GET_CUR -- read back negotiated parameters */

  uvc_init_ctrlreq(&req,
                   USB_REQ_DIR_IN |
                   USB_REQ_TYPE_CLASS |
                   USB_REQ_RECIPIENT_INTERFACE,
                   UVC_REQUEST_GET_CUR,
                   0x0100, vs_intf, UVC_PROBE_COMMIT_SIZE);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0,
                    (FAR const struct usb_ctrlreq_s *)&req,
                    ctrlreq);
  if (ret < 0)
    {
      uerr("ERROR: VS_PROBE_CONTROL GET_CUR failed: %d\n", ret);
      return ret;
    }

  if (ctrlreq[2] != format_idx || ctrlreq[3] != frame_idx)
    {
      uerr("ERROR: Probe mismatch: want %d/%d got %d/%d\n",
           format_idx, frame_idx, ctrlreq[2], ctrlreq[3]);
      return -EINVAL;
    }

  uinfo("UVC: probe verified OK\n");

  /* VS_COMMIT_CONTROL SET_CUR -- commit the probed parameters.
   * The probe GET_CUR result already contains the negotiated
   * format/frame/interval, use it directly as commit data.
   */

  uvc_init_ctrlreq(&req,
                   USB_REQ_DIR_OUT |
                   USB_REQ_TYPE_CLASS |
                   USB_REQ_RECIPIENT_INTERFACE,
                   UVC_REQUEST_SET_CUR,
                   0x0200, vs_intf, UVC_PROBE_COMMIT_SIZE);

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0,
                     (FAR const struct usb_ctrlreq_s *)&req, ctrlreq);
  if (ret < 0)
    {
      uerr("ERROR: VS_COMMIT_CONTROL SET_CUR failed: %d\n", ret);
      return ret;
    }

  uinfo("UVC: commit OK\n");

  return OK;
}

static int uvc_set_interface_alt(FAR struct usbhost_hubport_s *hport,
                                 FAR struct uvc_priv_s *priv,
                                 FAR uint8_t *ctrlreq)
{
  struct usbhost_epdesc_s ep_desc;
  struct usb_ctrlreq_s    req;
  uint32_t                required_payload = uvc_getle32(&ctrlreq[22]);
  uint16_t                best_payload;
  uint16_t                maxp;
  uint8_t                 desired_alt      = 1;
  uint8_t                 i;
  int                     ret;

  uinfo("UVC: required payload=%" PRIu32 "\n", required_payload);

  /* Find best alternate setting */

  if (priv->stream_alt_count > 0)
    {
      FAR struct uvc_stream_alt_info_s *ai;
      uint16_t payload;

      best_payload = 0xffff;
      for (i = 0; i < priv->stream_alt_count; i++)
        {
          ai = &priv->stream_alts[i];
          payload = ai->max_payload;

          if (payload >= required_payload && payload < best_payload)
            {
              best_payload = payload;
              desired_alt  = ai->alt;
            }
        }

      if (best_payload == 0xffff)
        {
          /* Fallback to maximum payload */

          maxp = 0;
          for (i = 0; i < priv->stream_alt_count; i++)
            {
              if (priv->stream_alts[i].max_payload > maxp)
                {
                  maxp        = priv->stream_alts[i].max_payload;
                  desired_alt = priv->stream_alts[i].alt;
                }
            }

          uwarn("UVC: no alt fits, using max alt=%d payload=%u\n",
                desired_alt, maxp);
        }
      else
        {
          uinfo("UVC: selected alt=%d payload=%u\n",
                desired_alt, best_payload);
        }
    }

  /* Set interface alternate setting */

  uvc_init_ctrlreq(&req,
                   USB_REQ_DIR_OUT |
                   USB_REQ_TYPE_STANDARD |
                   USB_REQ_RECIPIENT_INTERFACE,
                   USB_REQ_SETINTERFACE,
                   desired_alt,
                   priv->stream_iface, 0);

  ret = DRVR_CTRLOUT(hport->drvr, hport->ep0,
                     (FAR const struct usb_ctrlreq_s *)&req, NULL);
  if (ret < 0)
    {
      uerr("ERROR: SET_INTERFACE failed: %d\n", ret);
      return ret;
    }

  /* Update device state */

  priv->stream_altsetting = desired_alt;

  if (priv->stream_alt_count > 0)
    {
      for (i = 0; i < priv->stream_alt_count; i++)
        {
          if (priv->stream_alts[i].alt == desired_alt)
            {
              priv->max_pkt_size = priv->stream_alts[i].max_payload;
              uinfo("UVC: max_pkt=%u alt=%u\n",
                    priv->max_pkt_size, desired_alt);

              /* Re-allocate ISO endpoint with the correct max packet
               * size for this alt setting. epinfo->maxpacket was set
               * during initial epalloc and may not match the current alt.
               */

              if (priv->isoin)
                {
                  ep_desc.hport         = hport;
                  ep_desc.addr          = priv->isoc_ep_addr;
                  ep_desc.in            = 1;
                  ep_desc.xfrtype       = USB_EP_ATTR_XFER_ISOC;
                  ep_desc.interval      = priv->stream_alts[i].interval;
                  ep_desc.mxpacketsize  =
                    priv->stream_alts[i].raw_mxpacketsize;

                  DRVR_EPFREE(hport->drvr, priv->isoin);
                  priv->isoin = NULL;

                  ret = DRVR_EPALLOC(hport->drvr, &ep_desc, &priv->isoin);
                  if (ret < 0)
                    {
                      uerr("ERROR: ISO ep realloc: %d\n", ret);
                      return ret;
                    }
                }

              break;
            }
        }
    }

  /* Verify interface setting */

  uvc_init_ctrlreq(&req,
                   USB_REQ_DIR_IN |
                   USB_REQ_TYPE_STANDARD |
                   USB_REQ_RECIPIENT_INTERFACE,
                   USB_REQ_GETINTERFACE,
                   0, priv->stream_iface, 1);

  ret = DRVR_CTRLIN(hport->drvr, hport->ep0,
                    (FAR const struct usb_ctrlreq_s *)&req, ctrlreq);
  if (ret < 0)
    {
      uerr("ERROR: GET_INTERFACE failed: %d\n", ret);
      return ret;
    }

  if (ctrlreq[0] != desired_alt)
    {
      uerr("ERROR: alt mismatch: want %d got %d\n", desired_alt, ctrlreq[0]);
      return -EIO;
    }

  return OK;
}

static int uvc_set_format(FAR struct uvc_priv_s *priv,
                          uint8_t vs_intf, uint8_t format_idx,
                          uint8_t frame_idx,
                          FAR const imgdata_interval_t *interval)
{
  FAR struct uvc_frame_info_s  *frame            = NULL;
  FAR struct usbhost_hubport_s *hport            = priv->usbclass.hport;
  FAR uint8_t                  *ctrlreq;
  uint32_t                      default_interval = 0;
  uint32_t                      requested_intv;
  uint32_t                      clamped_intv;
  uint32_t                      fps;
  size_t                        maxlen;
  uint8_t                       array_idx;
  int                           ret;

  uinfo("UVC: set_format vs=%d fmt=%d frm=%d\n",
        vs_intf, format_idx, frame_idx);

  /* Allocate control request buffer */

  ret = DRVR_ALLOC(hport->drvr, &ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC ctrlreq: %d\n", ret);
      return ret;
    }

  if (maxlen < UVC_PROBE_COMMIT_SIZE)
    {
      uerr("ERROR: DRVR_ALLOC buffer too small: %zu < %d\n",
           maxlen, UVC_PROBE_COMMIT_SIZE);
      DRVR_FREE(hport->drvr, ctrlreq);
      return -ENOMEM;
    }

  /* Calculate frame rate with interval clamping.
   * format_idx is 1-based UVC index; find matching
   * 0-based array entry for frame descriptor lookup.
   */

  for (array_idx = 0; array_idx < priv->format_list.num_formats; array_idx++)
    {
      if (priv->format_list.formats[array_idx].format_index == format_idx)
        {
          if (frame_idx > 0 &&
              frame_idx <= priv->format_list.formats[array_idx].num_frames)
            {
              frame =
                &priv->format_list.formats[array_idx].frames[frame_idx - 1];
              default_interval = frame->interval;
            }

          break;
        }
    }

  fps = uvc_calculate_fps(interval, default_interval);

  /* Clamp interval to device-supported discrete values */

  if (frame)
    {
      uinfo("UVC: frame num_interval=%d intv=%" PRIu32 "\n",
            frame->num_interval, frame->interval);
    }

  if (frame && frame->num_interval > 0 && fps > 0)
    {
      requested_intv = 10000000u / fps;
      clamped_intv = uvc_try_frame_interval(frame, requested_intv);

      uinfo("UVC: requested_intv=%" PRIu32 " clamped=%" PRIu32 "\n",
            requested_intv, clamped_intv);

      if (clamped_intv != requested_intv)
        {
          fps = 10000000u / clamped_intv;
          uinfo("UVC: clamped fps to %" PRIu32 "\n", fps);
        }
    }

  /* Execute probe and commit sequence */

  ret = uvc_execute_probe_commit(hport, vs_intf, format_idx, frame_idx, fps,
                                 ctrlreq);
  if (ret == OK)
    {
      /* Set interface alternate setting */

      ret = uvc_set_interface_alt(hport, priv, ctrlreq);
    }

  DRVR_FREE(hport->drvr, ctrlreq);
  return ret;
}

static int uvc_find_format_match(FAR struct uvc_dev_s *dev,
                                 uint32_t width, uint32_t height,
                                 uint32_t pixelformat, int *frame_idx)
{
  FAR struct uvc_priv_s *priv = dev->device;
  uint8_t                i;
  uint8_t                j;

  if (atomic_read(&priv->disconnected))
    {
      return -ENODEV;
    }

  uinfo("UVC: find format %" PRIu32 "x%" PRIu32
        " pix=0x%08" PRIx32 "\n",
        width, height, pixelformat);

  /* Only exact match (resolution + pixel format) - no fallback to avoid
   * corruption
   */

  for (i = 0; i < priv->format_list.num_formats; i++)
    {
      struct uvc_format_info_s *fmt = &priv->format_list.formats[i];

      if (fmt->pixel_format_fourcc == pixelformat)
        {
          for (j = 0; j < fmt->num_frames; j++)
            {
              struct uvc_frame_info_s *frame = &fmt->frames[j];

              if (frame->width == width && frame->height == height)
                {
                  *frame_idx = j;   /* Set frame index */

                  uinfo("UVC: match fmt=%d frm=%d\n", i, j);
                  return i;   /* Return format index */
                }
            }
        }
    }

  uerr("UVC: no match for %" PRIu32 "x%" PRIu32
       " 0x%08" PRIx32 " (%s)\n",
       width, height, pixelformat, uvc_fourcc_to_string(pixelformat));
  for (i = 0; i < priv->format_list.num_formats; i++)
    {
      struct uvc_format_info_s *fmt = &priv->format_list.formats[i];

      for (j = 0; j < fmt->num_frames; j++)
        {
          struct uvc_frame_info_s *frame = &fmt->frames[j];

          uerr("  fmt%d: %dx%d %s\n", i, frame->width, frame->height,
               uvc_fourcc_to_string(fmt->pixel_format_fourcc));
        }
    }

  return -ENOENT;
}

static int uvc_validate_format_params(FAR struct uvc_priv_s *priv,
                                      uint8_t format_idx, uint8_t frame_idx)
{
  if (format_idx >= priv->format_list.num_formats)
    {
      uerr("UVC: bad format_idx %d >= %d\n",
           format_idx, priv->format_list.num_formats);
      return -EINVAL;
    }

  struct uvc_format_info_s *fmt = &priv->format_list.formats[format_idx];

  if (frame_idx >= fmt->num_frames)
    {
      uerr("UVC: bad frame_idx %d >= %d\n", frame_idx, fmt->num_frames);
      return -EINVAL;
    }

  return OK;
}

static int uvc_applyformat(FAR struct uvc_dev_s *dev,
                           int format_idx, int frame_idx,
                           FAR const imgdata_interval_t *req_inter)
{
  FAR struct uvc_priv_s        *priv = dev->device;
  FAR struct uvc_format_info_s *fmt;
  FAR struct uvc_frame_info_s  *frame;
  int                           ret;

  /* Validate format parameters */

  ret = uvc_validate_format_params(priv, format_idx, frame_idx);
  if (ret != OK)
    {
      return ret;
    }

  fmt   = &priv->format_list.formats[format_idx];
  frame = &fmt->frames[frame_idx];

  /* Apply the format with requested frame rate */

  ret = uvc_set_format(priv, priv->stream_iface, fmt->format_index,
                       frame->frame_index, req_inter);
  if (ret != OK)
    {
      uerr("UVC: set format failed: %d\n", ret);

      /* Fallback: Try with format default frame rate */

      if (req_inter && req_inter->denominator > 0)
        {
          uint32_t format_fps = uvc_calculate_fps(NULL, frame->interval);

          uinfo("UVC: fallback to default %" PRIu32 " fps\n",
                format_fps);

          ret = uvc_set_format(priv, priv->stream_iface,
                               fmt->format_index, frame->frame_index, NULL);
          if (ret != OK)
            {
              uerr("UVC: default fps also failed: %d\n", ret);
              return ret;
            }
        }
      else
        {
          return ret;
        }
    }

  dev->current_format_idx = format_idx;
  dev->current_frame_idx  = frame_idx;

  uinfo("UVC: applied fmt=%d frm=%d\n",
        fmt->format_index, frame->frame_index);

  return OK;
}

static void uvc_free_formats(FAR struct uvc_dev_s *dev)
{
  if (dev->fmtdescs)
    {
      kmm_free(dev->fmtdescs);
      dev->fmtdescs = NULL;
    }

  if (dev->frmsizes)
    {
      kmm_free(dev->frmsizes);
      dev->frmsizes = NULL;
    }

  if (dev->frmintervals)
    {
      kmm_free(dev->frmintervals);
      dev->frmintervals = NULL;
    }

  dev->imgsensor.fmtdescs      = NULL;
  dev->imgsensor.frmsizes      = NULL;
  dev->imgsensor.frmintervals  = NULL;
}

static int uvc_imgsensor_init(FAR struct imgsensor_s *sensor)
{
  FAR struct uvc_dev_s   *dev  = imgsensor_to_uvc_dev(sensor);
  FAR struct uvc_priv_s  *priv = dev->device;
  FAR const char         *desc;
  uint8_t                 num_formats;
  int                     total_frames = 0;
  uint8_t                 fmt_idx      = 0;
  uint8_t                 size_idx     = 0;
  uint8_t                 interval_idx = 0;
  uint8_t                 i;
  uint8_t                 j;

  if (atomic_read(&priv->disconnected))
    {
      return -ENODEV;
    }

  num_formats = priv->format_list.num_formats;

  uinfo("UVC: setup imgsensor from %d formats\n", num_formats);

  /* Free any previous allocation */

  uvc_free_formats(dev);

  /* Count total frames across all formats */

  for (i = 0; i < num_formats; i++)
    {
      total_frames += priv->format_list.formats[i].num_frames;
    }

  /* Allocate per-device format arrays */

  dev->fmtdescs = kmm_zalloc(num_formats * sizeof(struct v4l2_fmtdesc));
  dev->frmsizes = kmm_zalloc(total_frames * sizeof(struct v4l2_frmsizeenum));
  dev->frmintervals =
    kmm_zalloc(total_frames * sizeof(struct v4l2_frmivalenum));

  if (!dev->fmtdescs || !dev->frmsizes || !dev->frmintervals)
    {
      uerr("UVC: format alloc failed\n");
      uvc_free_formats(dev);
      return -ENOMEM;
    }

  for (i = 0; i < num_formats; i++)
    {
      FAR struct uvc_format_info_s *fmt = &priv->format_list.formats[i];

      dev->fmtdescs[fmt_idx].index       = fmt_idx;
      dev->fmtdescs[fmt_idx].type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      dev->fmtdescs[fmt_idx].flags       = 0;
      dev->fmtdescs[fmt_idx].pixelformat = fmt->pixel_format_fourcc;

      desc = uvc_fourcc_to_string(fmt->pixel_format_fourcc);
      strlcpy(dev->fmtdescs[fmt_idx].description, desc,
              sizeof(dev->fmtdescs[fmt_idx].description));

      for (j = 0; j < fmt->num_frames; j++)
        {
          FAR struct uvc_frame_info_s *frame = &fmt->frames[j];

          dev->frmsizes[size_idx].index = size_idx;
          dev->frmsizes[size_idx].type = V4L2_FRMSIZE_TYPE_DISCRETE;
          dev->frmsizes[size_idx].pixel_format =
            fmt->pixel_format_fourcc;
          dev->frmsizes[size_idx].discrete.width = frame->width;
          dev->frmsizes[size_idx].discrete.height = frame->height;

          dev->frmintervals[interval_idx].index = interval_idx;
          dev->frmintervals[interval_idx].pixel_format =
            fmt->pixel_format_fourcc;
          dev->frmintervals[interval_idx].width = frame->width;
          dev->frmintervals[interval_idx].height = frame->height;
          dev->frmintervals[interval_idx].type = V4L2_FRMIVAL_TYPE_DISCRETE;

          if (frame->interval > 0)
            {
              dev->frmintervals[interval_idx].discrete.numerator = 1;
              dev->frmintervals[interval_idx].discrete.denominator =
                10000000 / frame->interval;
            }
          else
            {
              dev->frmintervals[interval_idx].discrete.numerator = 1;
              dev->frmintervals[interval_idx].discrete.denominator =
                UVC_DEFAULT_FPS;
            }

          size_idx++;
          interval_idx++;
        }

      fmt_idx++;
    }

  sensor->fmtdescs_num     = num_formats;
  sensor->fmtdescs         = dev->fmtdescs;
  sensor->frmsizes_num     = size_idx;
  sensor->frmsizes         = dev->frmsizes;
  sensor->frmintervals_num = interval_idx;
  sensor->frmintervals     = dev->frmintervals;

  uinfo("UVC: %d fmts, %d sizes, %d intervals\n",
        sensor->fmtdescs_num,
        sensor->frmsizes_num,
        sensor->frmintervals_num);

  return OK;
}

static int uvc_imgdata_init(FAR struct imgdata_s *data)
{
  FAR struct uvc_dev_s *dev = (FAR struct uvc_dev_s *)data;

  uinfo("UVC: imgdata init\n");

  if (atomic_read(&dev->device->disconnected))
    {
      return -ENODEV;
    }

  return OK;
}

static int uvc_imgdata_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct uvc_dev_s  *dev = (FAR struct uvc_dev_s *)data;
  FAR struct uvc_priv_s *priv;

  uinfo("UVC: stop capture\n");

  if (!atomic_xchg(&dev->thread_running, false))
    {
      return OK;
    }

  /* Cancel pending ISO transfers so the thread can exit */

  priv = dev->device;
  if (priv && priv->isoin)
    {
      FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;

      if (hport && hport->drvr)
        {
          DRVR_CANCEL(hport->drvr, priv->isoin);
        }
    }

  if (dev->capture_tid > 0)
    {
      nxsem_wait_uninterruptible(&dev->capture_exit_sem);
      nxsem_destroy(&dev->capture_exit_sem);
      dev->capture_tid = 0;
    }

  dev->capture_callback = NULL;
  dev->capture_arg = NULL;

  /* Frame buffer is freed by capture_tid cleanup */

  return OK;
}

static int uvc_imgdata_uninit(FAR struct imgdata_s *data)
{
  FAR struct uvc_dev_s *dev = (FAR struct uvc_dev_s *)data;

  if (atomic_read(&dev->thread_running))
    {
      uvc_imgdata_stop_capture(data);
    }

  uvc_free_formats(dev);

  /* In a normal close dev stays alive for the next open, so only hand the
   * reference back once the device is gone for good.
   */

  if (atomic_read(&dev->device->disconnected))
    {
      uvc_priv_put(dev->device);
    }

  return OK;
}

static int uvc_imgdata_set_buf(FAR struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               FAR imgdata_format_t *datafmts,
                               FAR uint8_t *addr, uint32_t size)
{
  FAR struct uvc_dev_s *dev = (FAR struct uvc_dev_s *)data;
  irqstate_t            flags;

  /* NULL addr with size==0 is valid: enters drop-frame mode.
   * Reject mismatched states: addr without size, or size without addr.
   */

  if ((addr != NULL) != (size != 0))
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&dev->buffer_lock);
  dev->buffer_addr = addr;
  dev->buffer_size = size;
  spin_unlock_irqrestore(&dev->buffer_lock, flags);

  return OK;
}

static int
uvc_imgdata_validate_frame_setting(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts,
                                   FAR imgdata_interval_t *interval)
{
  FAR struct uvc_dev_s *dev = (FAR struct uvc_dev_s *)data;
  uint32_t              converted_pixelformat;
  int                   format_idx;
  int                   frame_idx;

  UNUSED(interval);

  if (!datafmts || nr_datafmts == 0)
    {
      return -EINVAL;
    }

  converted_pixelformat = uvc_convert_pixelformat(datafmts[0].pixelformat);

  format_idx = uvc_find_format_match(dev, datafmts[0].width,
                                     datafmts[0].height,
                                     converted_pixelformat, &frame_idx);
  if (format_idx < 0)
    {
      uerr("UVC: format %dx%d (0x%08" PRIx32 ") unsupported\n",
           datafmts[0].width, datafmts[0].height, converted_pixelformat);
      return -EINVAL;
    }

  uinfo("UVC: format match idx=%d frm=%d\n", format_idx, frame_idx);

  /* Validate only checks that the format+resolution exists in the
   * device descriptor.  Do NOT send USB probe/commit here -- that
   * belongs in set_format (S_FMT / start_capture), not try_format.
   * Sending probe during validate can fail because the device may
   * not be ready (alt-setting 0) and it changes device state.
   */

  return OK;
}

static bool uvc_deliver_frame(FAR struct uvc_dev_s *dev,
                              size_t data_size,
                              FAR struct timeval *timestamp)
{
  FAR struct uvc_priv_s *priv = dev->device;

  if (data_size == 0 || data_size > dev->iso_frame_dest_size)
    {
      return false;
    }

  /* For compressed formats, validate JPEG SOI/EOI markers.
   * Reference: original patch checked SOI in deliver_frame.
   */

  if (priv && dev->current_format_idx < priv->format_list.num_formats)
    {
      if (uvc_is_compressed_format(
            priv->format_list.formats[dev->current_format_idx].
              pixel_format_fourcc))
        {
          uint8_t   *data = dev->iso_frame_dest;
          size_t    sz    = data_size;

          /* Must start with JPEG SOI (FF D8) */

          if (sz < 2 || data[0] != 0xff || data[1] != 0xd8)
            {
              return false;
            }

          /* The device pads the last payload to an even length with a zero
           * byte, so a JPEG body of odd length ends in FF D9 00.  UVC
           * carries no length field to tell the pad from data, which leaves
           * the EOI as the only completeness signal available here.
           */

          if (sz >= 5 && data[sz - 1] == 0x00 &&
              data[sz - 3] == 0xff && data[sz - 2] == 0xd9)
            {
              sz--;
            }

          if (sz < 4 || data[sz - 2] != 0xff || data[sz - 1] != 0xd9)
            {
              return false;
            }
        }
    }

  if (dev->capture_callback)
    {
      int ret = dev->capture_callback(0, data_size, timestamp,
                                      dev->capture_arg);

      if (ret == OK)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: uvc_iso_complete_frame
 *
 * Description:
 *   Validate and signal a completed frame for delivery.
 *   Reference: Linux uvc_video_validate_buffer + uvc_video_next_buffers
 *
 *   Compressed formats (MJPEG, H.264, H.265): any size is valid.
 *   Uncompressed formats (YUYV): need >= 95% of expected frame size.
 *
 ****************************************************************************/

static void uvc_iso_complete_frame(FAR struct uvc_dev_s *dev)
{
  FAR struct uvc_priv_s *priv          = dev->device;
  bool                   is_compressed = false;

  if (priv && dev->current_format_idx < priv->format_list.num_formats)
    {
      is_compressed = uvc_is_compressed_format(
        priv->format_list.formats[dev->current_format_idx].
          pixel_format_fourcc);
    }

  /* Compressed: any EOF frame is valid (variable size).
   * Uncompressed: need >= 95% of expected frame size.
   */

  if (is_compressed ||
      dev->iso_frame_offset >=
        (dev->iso_frame_dest_size * UVC_FRAME_COMPLETE_THRESHOLD / 100))
    {
      /* For compressed formats, validate JPEG SOI marker */

      if (is_compressed && dev->iso_frame_offset >= 2)
        {
          FAR uint8_t *buf = dev->iso_frame_dest;

          if (buf[0] != 0xff || buf[1] != 0xd8)
            {
              return;
            }
        }

      dev->iso_frame_complete_size = dev->iso_frame_offset;
    }
}

/****************************************************************************
 * Name: uvc_iso_deliver_completed_frame
 *
 * Description:
 *   Deliver the assembled frame, or report the drop when no buffer was
 *   latched. The drop report is what asks the upper half for the next
 *   buffer, so it must not be skipped.
 *
 ****************************************************************************/

static void uvc_iso_deliver_completed_frame(FAR struct uvc_dev_s *dev)
{
  struct timeval ts;

  gettimeofday(&ts, NULL);

  if (dev->iso_frame_dest == NULL)
    {
      if (dev->capture_callback)
        {
          dev->capture_callback(0, 0, &ts, dev->capture_arg);
        }
    }
  else if (dev->iso_frame_complete_size != 0)
    {
      uvc_deliver_frame(dev, dev->iso_frame_complete_size, &ts);
    }

  dev->iso_frame_complete_size = 0;
}

/****************************************************************************
 * Name: uvc_iso_frame_begin
 *
 * Description:
 *   Latch the buffer the next frame assembles into, binding one frame to
 *   one buffer. A NULL latch means none was free.
 *
 ****************************************************************************/

static void uvc_iso_frame_begin(FAR struct uvc_dev_s *dev)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->buffer_lock);
  dev->iso_frame_dest      = dev->buffer_addr;
  dev->iso_frame_dest_size = dev->buffer_size;
  spin_unlock_irqrestore(&dev->buffer_lock, flags);

  dev->iso_frame_offset  = 0;
  dev->iso_frame_started = true;
  dev->iso_frame_error   = false;
}

/****************************************************************************
 * Name: uvc_iso_decode_packet
 *
 * Description:
 *   Decode a single ISO packet. Called from per-packet callback
 *   with actual_length. Uses slot->pkt_idx to locate the packet
 *   data in the max_packet-stride buffer.
 *
 *   Reference: Linux uvc_video_decode_isoc() in uvc_video.c
 *
 ****************************************************************************/

static void uvc_iso_decode_packet(FAR struct uvc_dev_s *dev,
                                  FAR struct uvc_iso_slot_s *slot,
                                  uint32_t actual_len)
{
  FAR struct uvc_iso_pipe_s *pipe    = &dev->iso_pipe;
  uint16_t                   max_pkt = pipe->max_packet;
  size_t                     offset  = slot->pkt_idx * max_pkt;
  FAR uint8_t               *pkt_buf = slot->buffer + offset;
  uint8_t                    header_len;
  uint8_t                    bm;
  bool                       fid;
  bool                       eof;
  bool                       err;
  size_t                     data_len;

  if (actual_len < 2 || actual_len > max_pkt)
    {
      return;
    }

  header_len = pkt_buf[0];
  if (header_len < 2 || header_len > actual_len)
    {
      return;
    }

  bm = pkt_buf[1];
  fid = (bm & 0x01) != 0;
  eof = (bm & 0x02) != 0;
  err = (bm & 0x40) != 0;

  if (err)
    {
      dev->iso_frame_error = true;
    }

  /* FID tracking and frame boundary detection */

  if (!dev->iso_have_prev_fid)
    {
      dev->iso_prev_fid = fid;
      dev->iso_have_prev_fid = true;
      uvc_iso_frame_begin(dev);
    }
  else if (fid != dev->iso_prev_fid)
    {
      if (dev->iso_frame_started && dev->iso_frame_offset > 0)
        {
          /* Complete the previous frame on FID flip.
           * For uncompressed: deliver partial frame (few missing lines).
           * For compressed: also attempt delivery - if EOF was lost but
           * data is valid (has SOI), complete_frame will accept it.
           * This matches Linux UVC behavior (uvc_video_decode_start).
           */

          if (!dev->iso_frame_error && dev->iso_frame_dest != NULL)
            {
              uvc_iso_complete_frame(dev);
            }

          uvc_iso_deliver_completed_frame(dev);
        }

      dev->iso_prev_fid = fid;
      uvc_iso_frame_begin(dev);
    }

  if (!dev->iso_frame_started)
    {
      return;
    }

  data_len = actual_len - header_len;
  if (data_len > 0)
    {
      if (dev->iso_frame_dest == NULL)
        {
          dev->iso_frame_offset += data_len;
        }
      else if (dev->iso_frame_offset + data_len <=
               dev->iso_frame_dest_size)
        {
          memcpy(dev->iso_frame_dest + dev->iso_frame_offset,
                 pkt_buf + header_len, data_len);
          dev->iso_frame_offset += data_len;
        }
      else
        {
          dev->iso_frame_error = true;
          dev->iso_frame_started = false;
        }
    }

  /* EOF handling */

  if (eof && dev->iso_frame_started && dev->iso_frame_offset > 0)
    {
      if (!dev->iso_frame_error && dev->iso_frame_dest != NULL)
        {
          uvc_iso_complete_frame(dev);
        }

      uvc_iso_deliver_completed_frame(dev);

      dev->iso_frame_started = false;
      dev->iso_frame_offset = 0;
      dev->iso_frame_error = false;
    }
}

/****************************************************************************
 * Name: uvc_iso_slot_callback
 *
 * Description:
 *   ISO slot completion callback. Called from HCD
 *   context (work queue). Marks slot COMPLETED and
 *   posts semaphore to wake capture_tid.
 *   capture_tid is the sole submitter of slots.
 *
 ****************************************************************************/

static void uvc_iso_slot_callback(FAR void *arg, ssize_t result)
{
  FAR struct uvc_iso_slot_s *slot = (FAR struct uvc_iso_slot_s *)arg;
  FAR struct uvc_dev_s      *dev;
  FAR struct uvc_iso_pipe_s *pipe;

  /* If dev was detached (timeout during cleanup), the capture
   * thread gave up waiting. Free the buffer ourselves and bail.
   */

  if (!slot->dev)
    {
      if (slot->buffer)
        {
          kmm_free(slot->buffer);
          slot->buffer = NULL;
        }

      return;
    }

  dev = slot->dev;
  pipe = &dev->iso_pipe;

  /* Ignore late/duplicate completions after the slot was already
   * retired (error short-circuit or stream stop).
   */

  if (atomic_read(&slot->state) != UVC_ISO_SLOT_SUBMITTED)
    {
      return;
    }

  if (result < 0)
    {
      slot->result = result;
      atomic_set(&slot->state, UVC_ISO_SLOT_COMPLETED);
      nxsem_post(&pipe->completion_sem);
      return;
    }

  uvc_iso_decode_packet(dev, slot, result);
  slot->pkt_idx++;

  if (slot->pkt_idx >= pipe->packets_per_xfer)
    {
      slot->result = 0;
      atomic_set(&slot->state, UVC_ISO_SLOT_COMPLETED);
      nxsem_post(&pipe->completion_sem);
    }
}

/****************************************************************************
 * Name: uvc_iso_submit_slot
 *
 * Description:
 *   Submit a FREE slot to HCD via DRVR_ASYNCH.
 *
 ****************************************************************************/

static int uvc_iso_submit_slot(FAR struct uvc_dev_s *dev,
                               FAR struct uvc_iso_slot_s *slot)
{
  FAR struct uvc_priv_s        *priv  = dev->device;
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  int                           ret;

  slot->pkt_idx = 0;
  slot->result = 0;

  /* Arm before submit: a completion that lands before the SUBMITTED mark
   * is dropped and the slot never finishes.
   */

  atomic_set_release(&slot->state, UVC_ISO_SLOT_SUBMITTED);

  ret = DRVR_ASYNCH(hport->drvr,
                    priv->isoin,
                    slot->buffer,
                    slot->buflen,
                    uvc_iso_slot_callback,
                    slot);
  if (ret < 0)
    {
      atomic_set(&slot->state, UVC_ISO_SLOT_FREE);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: uvc_iso_pipe_init
 *
 * Description:
 *   Initialize the ISO pipeline: calculate packet count, allocate
 *   per-slot buffers, init completion semaphore.
 *
 ****************************************************************************/

static int uvc_iso_pipe_init(FAR struct uvc_dev_s *dev)
{
  FAR struct uvc_priv_s     *priv = dev->device;
  FAR struct uvc_iso_pipe_s *pipe = &dev->iso_pipe;
  size_t                     xfer_buf_size;
  uint8_t                    i;
  uint8_t                    j;

  pipe->max_packet = priv->max_pkt_size;
  if (pipe->max_packet == 0)
    {
      uerr("ERROR: max_packet is zero\n");
      return -EINVAL;
    }

  pipe->num_slots = UVC_ISO_PIPELINE_DEPTH;
  pipe->packets_per_xfer = UVC_PACKETS_PER_SLOT;
  xfer_buf_size = pipe->packets_per_xfer * pipe->max_packet;

  for (i = 0; i < pipe->num_slots; i++)
    {
      pipe->slots[i].buffer = kmm_zalloc(xfer_buf_size);
      if (!pipe->slots[i].buffer)
        {
          for (j = 0; j < i; j++)
            {
              kmm_free(pipe->slots[j].buffer);
              pipe->slots[j].buffer = NULL;
            }

          return -ENOMEM;
        }

      pipe->slots[i].buflen = xfer_buf_size;
      atomic_set(&pipe->slots[i].state, UVC_ISO_SLOT_FREE);
      pipe->slots[i].index = i;
      pipe->slots[i].dev = dev;
      pipe->slots[i].result = 0;
    }

  nxsem_init(&pipe->completion_sem, 0, 0);

  uinfo("UVC: pipe init slots=%d pkts=%d maxpkt=%d bufsz=%zu\n",
        pipe->num_slots, pipe->packets_per_xfer,
        pipe->max_packet, xfer_buf_size);

  return OK;
}

/****************************************************************************
 * Name: uvc_iso_pipe_destroy
 *
 * Description:
 *   Release all pipeline resources.
 *
 ****************************************************************************/

static void uvc_iso_pipe_destroy(FAR struct uvc_dev_s *dev)
{
  FAR struct uvc_iso_pipe_s *pipe = &dev->iso_pipe;
  uint8_t                    i;

  for (i = 0; i < pipe->num_slots; i++)
    {
      /* Skip slots handed off to late callback (dev == NULL).
       * The callback will free the buffer when it arrives.
       */

      if (!pipe->slots[i].dev)
        {
          continue;
        }

      if (pipe->slots[i].buffer)
        {
          kmm_free(pipe->slots[i].buffer);
          pipe->slots[i].buffer = NULL;
        }
    }

  nxsem_destroy(&pipe->completion_sem);
  pipe->num_slots = 0;
}

static int uvc_capture_thread(int argc, FAR char *argv[])
{
  FAR struct uvc_dev_s      *dev = (FAR struct uvc_dev_s *)
                                    ((uintptr_t)strtoul(argv[1], NULL, 0));
  FAR struct uvc_priv_s     *priv = dev->device;
  FAR struct uvc_iso_pipe_s *pipe = &dev->iso_pipe;
  struct timespec            ts;
  int                        submitted = 0;
  int                        ret;
  uint8_t                    i;

  DEBUGASSERT(dev->buffer_addr && dev->buffer_size > 0);

  /* Initialize frame assembly state */

  dev->iso_frame_dest = NULL;
  dev->iso_frame_dest_size = 0;
  dev->iso_frame_offset = 0;
  dev->iso_frame_started = false;
  dev->iso_have_prev_fid = false;
  dev->iso_prev_fid = 0;
  dev->iso_frame_error = false;
  dev->iso_frame_complete_size = 0;

  /* Initialize pipeline */

  ret = uvc_iso_pipe_init(dev);
  if (ret < 0)
    {
      uerr("UVC: pipe init failed: %d\n", ret);
      atomic_set(&dev->thread_running, false);
      nxsem_post(&dev->capture_exit_sem);
      return ret;
    }

  uinfo("UVC: pipe ready slots=%d pkts=%u maxpkt=%u\n",
        pipe->num_slots, pipe->packets_per_xfer, pipe->max_packet);

  /* Submit all slots */

  for (i = 0; i < pipe->num_slots; i++)
    {
      ret = uvc_iso_submit_slot(dev, &pipe->slots[i]);
      if (ret < 0)
        {
          uerr("UVC: slot %d submit failed: %d\n", i, ret);
        }
      else
        {
          submitted++;
        }
    }

  if (submitted == 0)
    {
      uerr("UVC: all slots failed, exiting\n");
      goto cleanup;
    }

  /* Main loop */

  while (atomic_read(&dev->thread_running) &&
         !atomic_read(&priv->disconnected))
    {
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += UVC_ISO_WAIT_TIMEOUT_SEC;

      ret = nxsem_timedwait(&pipe->completion_sem, &ts);
      if (ret < 0)
        {
          if (ret == -ETIMEDOUT || ret == -EINTR)
            {
              continue;
            }

          break;
        }

      /* Scan for completed slots */

      for (i = 0; i < pipe->num_slots; i++)
        {
          FAR struct uvc_iso_slot_s *slot = &pipe->slots[i];

          if (atomic_read(&slot->state) != UVC_ISO_SLOT_COMPLETED)
            {
              continue;
            }

          /* Decoding already happened inline via per-packet
           * callbacks in uvc_iso_decode_packet.
           */

          if (!atomic_read(&dev->thread_running) ||
              atomic_read(&priv->disconnected))
            {
              goto cleanup;
            }

          /* Transport error (NO_FREE_QTD, TIMEOUT, STALL, ...).
           * The HAL has already stopped the EP.  -ESHUTDOWN/-ENODEV
           * outside of a restart means the stream is gone for good;
           * anything else is recoverable -- drop the frame and
           * restart the stream automatically.
           */

          if (slot->result < 0)
            {
              if (slot->result == -ESHUTDOWN || slot->result == -ENODEV)
                {
                  goto cleanup;
                }

              uwarn("UVC: xfer error %d, restarting\n",
                    (int)slot->result);

              if (dev->capture_callback)
                {
                  struct timeval tv;

                  gettimeofday(&tv, NULL);
                  dev->capture_callback(0, 0, &tv, dev->capture_arg);
                }

              uvc_iso_restart(dev);
              break;
            }

          /* Resubmit slot */

          atomic_set(&slot->state, UVC_ISO_SLOT_FREE);
          ret = uvc_iso_submit_slot(dev, slot);
          if (ret < 0)
            {
              uwarn("UVC: slot %d resubmit failed: %d\n", i, ret);
              if (ret == -ESHUTDOWN || ret == -ENODEV)
                {
                  goto cleanup;
                }
            }
        }

      /* Check if any slot is still active (SUBMITTED or COMPLETED).
       * If all slots are FREE, pipeline is dead -- break
       * to avoid infinite idle loop.  A COMPLETED slot still has its
       * completion posted and is resubmitted on the next pass; slots
       * finish from the callback, so all of them can be COMPLETED here
       * when this thread was preempted since the scan.
       */

      for (i = 0; i < pipe->num_slots; i++)
        {
          if (atomic_read(&pipe->slots[i].state) != UVC_ISO_SLOT_FREE)
            {
              break;
            }
        }

      if (i == pipe->num_slots)
        {
          uerr("UVC: no active slots, exiting\n");
          break;
        }
    }

cleanup:

  /* Signal pipeline shutdown and cancel in-flight
   * transfers. DRVR_CANCEL triggers the callback
   * which posts the completion semaphore, so we
   * can wait on it instead of polling with usleep.
   */

  if (priv->isoin && priv->usbclass.hport &&
      priv->usbclass.hport->drvr)
    {
      DRVR_CANCEL(priv->usbclass.hport->drvr, priv->isoin);
    }

  /* Wait for each in-flight slot to actually
   * reach COMPLETED. The shared semaphore may
   * carry stale counts from the main loop, so a
   * single sem_wait per slot is not sufficient --
   * re-check the slot state after every wake.
   */

  for (i = 0; i < pipe->num_slots; i++)
    {
      struct timespec cts;

      clock_gettime(CLOCK_REALTIME, &cts);
      cts.tv_sec += UVC_SLOT_WAIT_TIMEOUT_SEC;

      while (atomic_read(&pipe->slots[i].state) == UVC_ISO_SLOT_SUBMITTED)
        {
          ret = nxsem_timedwait(&pipe->completion_sem, &cts);
          if (ret == -ETIMEDOUT)
            {
              uerr("UVC: slot %d stuck, defer free\n", i);

              /* Detach slot from dev. When the late
               * callback arrives, it sees dev==NULL,
               * frees the buffer itself, then returns.
               */

              pipe->slots[i].dev = NULL;
              break;
            }
        }
    }

  uvc_iso_pipe_destroy(dev);

  dev->iso_frame_dest = NULL;
  dev->iso_frame_dest_size = 0;

  nxsem_post(&dev->capture_exit_sem);
  return OK;
}

/****************************************************************************
 * Name: uvc_iso_restart
 *
 * Description:
 *   Recover from a transport error (NO_FREE_QTD, TIMEOUT, STALL, ...).
 *   The endpoint is already idle at this point, so cancel the in-flight
 *   slots, reset the frame assembly state and resubmit; the first
 *   submission after the stop restarts the data stream.  Called from
 *   capture_tid only.
 *
 ****************************************************************************/

static void uvc_iso_restart(FAR struct uvc_dev_s *dev)
{
  FAR struct uvc_priv_s *priv = dev->device;
  FAR struct uvc_iso_pipe_s *pipe = &dev->iso_pipe;
  int ret;
  int i;

  uinfo("UVC: restarting ISO stream\n");

  /* Stop the stream.  Completes all in-flight slots with -ESHUTDOWN. */

  if (priv->isoin && priv->usbclass.hport &&
      priv->usbclass.hport->drvr)
    {
      DRVR_CANCEL(priv->usbclass.hport->drvr, priv->isoin);
    }

  for (i = 0; i < pipe->num_slots; i++)
    {
      struct timespec cts;

      clock_gettime(CLOCK_REALTIME, &cts);
      cts.tv_sec += UVC_SLOT_WAIT_TIMEOUT_SEC;

      while (atomic_read(&pipe->slots[i].state) == UVC_ISO_SLOT_SUBMITTED)
        {
          ret = nxsem_timedwait(&pipe->completion_sem, &cts);
          if (ret == -ETIMEDOUT)
            {
              uerr("UVC: restart slot %d stuck, defer free\n", i);
              pipe->slots[i].dev = NULL;
              break;
            }
        }
    }

  dev->iso_frame_dest = NULL;
  dev->iso_frame_dest_size = 0;
  dev->iso_frame_offset = 0;
  dev->iso_frame_started = false;
  dev->iso_have_prev_fid = false;
  dev->iso_frame_error = false;
  dev->iso_frame_complete_size = 0;

  for (i = 0; i < pipe->num_slots; i++)
    {
      if (!pipe->slots[i].dev || !pipe->slots[i].buffer)
        {
          continue;
        }

      atomic_set(&pipe->slots[i].state, UVC_ISO_SLOT_FREE);
      ret = uvc_iso_submit_slot(dev, &pipe->slots[i]);
      if (ret < 0)
        {
          uerr("UVC: restart slot %d submit failed: %d\n", i, ret);
        }
    }
}

static int uvc_imgdata_start_capture(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval,
                                     FAR imgdata_capture_t callback,
                                     FAR void *arg)
{
  FAR struct uvc_dev_s  *dev  = (FAR struct uvc_dev_s *)data;
  FAR struct uvc_priv_s *priv = dev->device;
  char                   tid_buf[16];
  FAR char              *targv[2];
  int                    ret;

  uinfo("UVC: start capture\n");

  if (atomic_read(&priv->disconnected))
    {
      return -ENODEV;
    }

  if (atomic_xchg(&dev->thread_running, true))
    {
      return -EBUSY;
    }

  if (datafmts && nr_datafmts > 0)
    {
      imgdata_interval_t default_interval;
      FAR imgdata_interval_t *use_interval;
      uint32_t converted_pixelformat;
      int format_idx;
      int frame_idx;

      default_interval.numerator   = 1;
      default_interval.denominator = UVC_DEFAULT_FPS;

      /* Use the provided interval or default to 30 fps */

      use_interval = interval ? interval : &default_interval;

      /* Convert the pixel format like validate_frame_setting does */

      converted_pixelformat =
        uvc_convert_pixelformat(datafmts[0].pixelformat);

      /* Find the format match to get the indices */

      format_idx = uvc_find_format_match(dev, datafmts[0].width,
                                         datafmts[0].height,
                                         converted_pixelformat,
                                         &frame_idx);
      if (format_idx < 0)
        {
          int fi;

          uerr("UVC: unsupported: %dx%d 0x%08" PRIx32 " (%s)\n",
               datafmts[0].width, datafmts[0].height,
               converted_pixelformat,
               uvc_fourcc_to_string(converted_pixelformat));

          for (fi = 0;
               fi < priv->format_list.num_formats;
               fi++)
            {
              struct uvc_format_info_s *f =
                &priv->format_list.formats[fi];
              int fri;

              for (fri = 0;
                   fri < f->num_frames;
                   fri++)
                {
                  uerr("  %s %dx%d\n",
                       uvc_fourcc_to_string(f->pixel_format_fourcc),
                       f->frames[fri].width,
                       f->frames[fri].height);
                }
            }

          atomic_set(&dev->thread_running, false);
          return -ENOTSUP;
        }

      if (priv->stream_altsetting == 0 ||
          dev->current_format_idx != format_idx ||
          dev->current_frame_idx != frame_idx)
        {
          uinfo("UVC: apply fmt %d/%d -> %d/%d alt=%d\n",
                dev->current_format_idx,
                dev->current_frame_idx,
                format_idx, frame_idx,
                priv->stream_altsetting);

          /* Set the format on the UVC device */

          ret = uvc_applyformat(dev, format_idx, frame_idx, use_interval);
          if (ret < 0)
            {
              uerr("UVC: set format failed: %d\n", ret);
              atomic_set(&dev->thread_running, false);
              return ret;
            }
        }
    }

  /* Store callback.
   * Note: callback(addr=0, size=0, ...) means "frame dropped" --
   * no valid data is available (drop-frame mode).
   */

  dev->capture_callback = callback;
  dev->capture_arg      = arg;

  if (!priv->isoin)
    {
      atomic_set(&dev->thread_running, false);
      return -ENODEV;
    }

  snprintf(tid_buf, sizeof(tid_buf), "%p", dev);
  targv[0] = tid_buf;
  targv[1] = NULL;

  nxsem_init(&dev->capture_exit_sem, 0, 0);

  ret = kthread_create("uvc_capture",
                       CONFIG_USBHOST_UVC_THREAD_PRIORITY,
                       CONFIG_USBHOST_UVC_THREAD_STACKSIZE,
                       uvc_capture_thread, targv);

  if (ret < 0)
    {
      nxsem_destroy(&dev->capture_exit_sem);
      atomic_set(&dev->thread_running, false);
      return ret;
    }

  dev->capture_tid = ret;
  return OK;
}

static bool uvc_imgsensor_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct uvc_dev_s *dev = imgsensor_to_uvc_dev(sensor);

  return dev != NULL && dev->device != NULL &&
         !atomic_read(&dev->device->disconnected) &&
         dev->device->isoin != NULL &&
         dev->device->stream_alt_count > 0;
}

static int uvc_imgsensor_uninit(FAR struct imgsensor_s *sensor)
{
  uinfo("UVC: imgsensor uninit\n");
  return OK;
}

static const char *
uvc_imgsensor_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "UVC Camera Driver";
}

static int
uvc_imgsensor_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     uint8_t nr_datafmts,
                                     FAR imgsensor_format_t *datafmts,
                                     FAR imgsensor_interval_t *interval)
{
  FAR struct uvc_dev_s *dev = imgsensor_to_uvc_dev(sensor);
  uint32_t              converted_pixelformat;
  int                   format_idx;
  int                   frame_idx;

  if (!datafmts || nr_datafmts == 0)
    {
      return -EINVAL;
    }

  converted_pixelformat = uvc_convert_pixelformat(datafmts[0].pixelformat);
  format_idx = uvc_find_format_match(dev, datafmts[0].width,
                                     datafmts[0].height,
                                     converted_pixelformat, &frame_idx);
  if (format_idx < 0)
    {
      uerr("UVC: sensor validate failed: %dx%d 0x%08" PRIx32 "\n",
           datafmts[0].width, datafmts[0].height,
           converted_pixelformat);
      return -EINVAL;
    }

  return OK;
}

static int uvc_imgsensor_start_capture(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_datafmts,
                                       FAR imgsensor_format_t *datafmts,
                                       FAR imgsensor_interval_t *interval)
{
  /* Calculate the correct dev pointer from imgsensor pointer */

  FAR struct uvc_dev_s  *dev  = imgsensor_to_uvc_dev(sensor);
  FAR struct uvc_priv_s *priv = dev->device;

  uinfo("UVC: imgsensor start capture\n");

  if (atomic_read(&priv->disconnected))
    {
      return -ENODEV;
    }

  uinfo("UVC: sensor capture started\n");
  return OK;
}

static int uvc_imgsensor_stop_capture(FAR struct imgsensor_s *sensor,
                                      imgsensor_stream_type_t type)
{
  FAR struct uvc_dev_s  *dev  = imgsensor_to_uvc_dev(sensor);
  FAR struct uvc_priv_s *priv = dev->device;

  if (atomic_read(&priv->disconnected))
    {
      return OK;
    }

  uinfo("UVC: stopping stream, altsetting=%d\n", priv->stream_altsetting);
  if (priv->stream_altsetting > 0)
    {
      FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;

      if (hport && hport->drvr)
        {
          struct usb_ctrlreq_s req;
          int ret;

          uvc_init_ctrlreq(&req,
                           USB_REQ_DIR_OUT |
                           USB_REQ_TYPE_STANDARD |
                           USB_REQ_RECIPIENT_INTERFACE,
                           USB_REQ_SETINTERFACE,
                           0, priv->stream_iface, 0);

          ret = DRVR_CTRLOUT(hport->drvr, hport->ep0, &req, NULL);
          if (ret < 0)
            {
              uerr("UVC: stop sensor failed: %d\n", ret);
              return ret;
            }

          priv->stream_altsetting = 0;
        }
    }

  uinfo("UVC: sensor capture stopped\n");
  return OK;
}

static int
uvc_imgsensor_get_frame_interval(FAR struct imgsensor_s *sensor,
                                 imgsensor_stream_type_t type,
                                 FAR imgsensor_interval_t *interval)
{
  FAR struct uvc_dev_s *dev = imgsensor_to_uvc_dev(sensor);

  if (!interval)
    {
      return -EINVAL;
    }

  FAR struct uvc_priv_s *priv = dev->device;

  if (priv && !atomic_read(&priv->disconnected) &&
      dev->current_format_idx < priv->format_list.num_formats)
    {
      struct uvc_format_info_s *fmt =
        &priv->format_list.formats[dev->current_format_idx];
      struct uvc_frame_info_s *frame = &fmt->frames[dev->current_frame_idx];

      if (frame->interval > 0)
        {
          interval->numerator   = 1;
          interval->denominator = 10000000 / frame->interval;
        }
      else
        {
          interval->numerator   = 1;
          interval->denominator = UVC_DEFAULT_FPS;
        }
    }
  else
    {
      interval->numerator   = 1;
      interval->denominator = UVC_DEFAULT_FPS;
    }

  return OK;
}

static int uvc_query_ctrl_hw(FAR struct uvc_dev_s *dev,
                             FAR const struct uvc_ctrl_map_s *map,
                             uint32_t id,
                             FAR struct uvc_ctrl_cache_s *cache)
{
  uint8_t data[4];
  int     ret;

  if (map->size > sizeof(data))
    {
      return -EINVAL;
    }

  cache->queried = true;

  /* A control not advertised in bmControls is unsupported; avoid probing it
   * (some devices answer GET but STALL SET_CUR). Matches Linux uvcvideo.
   */

  if (!uvc_ctrl_present(dev->device, map))
    {
      cache->supported     = false;
      cache->ctrl_type     = (map->size == 1) ?
        IMGSENSOR_CTRL_TYPE_BOOLEAN : IMGSENSOR_CTRL_TYPE_INTEGER;
      cache->minimum       = 0;
      cache->maximum       = 0;
      cache->step          = 1;
      cache->default_value = 0;
      return OK;
    }

  /* Boolean controls */

  if (map->size == 1 &&
      (id == IMGSENSOR_ID_AUTO_WHITE_BALANCE ||
       id == IMGSENSOR_ID_FOCUS_AUTO ||
       id == IMGSENSOR_ID_EXPOSURE_AUTO))
    {
      cache->ctrl_type      = IMGSENSOR_CTRL_TYPE_BOOLEAN;
      cache->minimum        = 0;
      cache->maximum        = 1;
      cache->step           = 1;
      cache->default_value  = 0;

      memset(data, 0, sizeof(data));
      ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_DEF, data);
      if (ret < 0)
        {
          cache->supported = false;
          return OK;
        }

      cache->supported = true;
      cache->default_value = uvc_ctrl_read_value(map, data);
      return OK;
    }

  cache->ctrl_type = IMGSENSOR_CTRL_TYPE_INTEGER;

  /* GET_MIN -- if STALL, control unsupported */

  memset(data, 0, sizeof(data));
  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_MIN, data);
  if (ret < 0)
    {
      cache->supported = false;
      cache->minimum = 0;
      cache->maximum = 0;
      cache->step = 1;
      cache->default_value = 0;
      return OK;
    }

  cache->supported = true;
  cache->minimum = uvc_ctrl_read_value(map, data);

  memset(data, 0, sizeof(data));
  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_MAX, data);
  cache->maximum = (ret >= 0) ?
    uvc_ctrl_read_value(map, data) : 255;

  memset(data, 0, sizeof(data));
  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_RES, data);
  cache->step = (ret >= 0) ? uvc_ctrl_read_value(map, data) : 1;
  if (cache->step == 0)
    {
      cache->step = 1;
    }

  memset(data, 0, sizeof(data));
  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_DEF, data);
  cache->default_value = (ret >= 0) ?
    uvc_ctrl_read_value(map, data) : cache->minimum;

  uinfo("UVC: ctrl 0x%" PRIx32 " [%lld,%lld] step=%lld def=%lld\n",
        id, cache->minimum, cache->maximum,
        cache->step, cache->default_value);

  return OK;
}

static int
uvc_imgsensor_get_supported_value(FAR struct imgsensor_s *sensor,
                                  uint32_t id,
                                  FAR imgsensor_supported_value_t *value)
{
  FAR struct uvc_dev_s            *dev = imgsensor_to_uvc_dev(sensor);
  FAR const struct uvc_ctrl_map_s *map;
  FAR struct uvc_ctrl_cache_s     *cache;
  int                              idx;

  map = uvc_find_ctrl_map(id, &idx);
  if (!map || !value)
    {
      return -EINVAL;
    }

  if (idx >= UVC_CTRL_CACHE_MAX)
    {
      return -EINVAL;
    }

  cache = &dev->ctrl_cache[idx];

  if (!cache->queried)
    {
      uvc_query_ctrl_hw(dev, map, id, cache);
    }

  if (!cache->supported)
    {
      return -EINVAL;
    }

  value->type                  = cache->ctrl_type;
  value->u.range.minimum       = cache->minimum;
  value->u.range.maximum       = cache->maximum;
  value->u.range.step          = cache->step;
  value->u.range.default_value = cache->default_value;

  return OK;
}

static int uvc_imgsensor_get_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   FAR imgsensor_value_t *value)
{
  FAR struct uvc_dev_s            *dev = imgsensor_to_uvc_dev(sensor);
  FAR const struct uvc_ctrl_map_s *map;
  uint8_t                          data[4];
  int                              ret;

  map = uvc_find_ctrl_map(id, NULL);
  if (!map || !value)
    {
      return -EINVAL;
    }

  memset(data, 0, sizeof(data));
  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_GET_CUR, data);
  if (ret < 0)
    {
      return ret;
    }

  value->value32 = uvc_ctrl_read_value(map, data);
  return OK;
}

static int uvc_imgsensor_set_value(FAR struct imgsensor_s *sensor,
                                   uint32_t id, uint32_t size,
                                   imgsensor_value_t value)
{
  FAR struct uvc_dev_s            *dev = imgsensor_to_uvc_dev(sensor);
  FAR const struct uvc_ctrl_map_s *map;
  uint8_t                          data[4];
  int                              ret;

  map = uvc_find_ctrl_map(id, NULL);
  if (!map)
    {
      return -EINVAL;
    }

  memset(data, 0, sizeof(data));
  uvc_ctrl_write_value(map, data, value.value32);

  ret = uvc_ctrl_query(dev, map, UVC_REQUEST_SET_CUR, data);
  if (ret < 0)
    {
      uerr("UVC: SET_CUR failed id=0x%" PRIx32 ": %d\n",
           id, ret);
      return ret;
    }

  return OK;
}

static void uvc_device_cleanup_worker(FAR void *arg)
{
  FAR struct uvc_priv_s         *priv  = (FAR struct uvc_priv_s *)arg;
  FAR struct usbhost_hubport_s  *hport = priv->usbclass.hport;
  char                           devpath[32];

  if (priv->isoin && hport && hport->drvr)
    {
      DRVR_CANCEL(hport->drvr, priv->isoin);
    }

  /* dev is absent when connect failed before the node was registered. */

  if (priv->dev != NULL && atomic_read(&priv->dev->thread_running))
    {
      uvc_imgdata_stop_capture(&priv->dev->imgdata);
    }

  /* Hand the USB side back before unregistering the node.  The node can
   * outlive this worker, so late ioctls are expected; with the endpoints
   * already gone they fail out of DRVR_CTRLOUT/CTRLIN rather than reaching
   * a half-torn device.  hport belongs to the adapter and stays valid.
   */

  if (priv->isoin && hport && hport->drvr)
    {
      DRVR_EPFREE(hport->drvr, priv->isoin);
      priv->isoin = NULL;
    }

  if (hport && hport->drvr)
    {
      DRVR_DISCONNECT(hport->drvr, hport);
      hport->funcaddr = 0;
    }

  if (priv->dev != NULL)
    {
      snprintf(devpath, sizeof(devpath), "/dev/video%d", priv->minor);

      /* Hands the v4l2 reference back when the node is not open; the one
       * held here keeps both objects alive until the put below.
       */

      capture_unregister(devpath);
    }

  /* Only once the node is gone: a minor released earlier could be handed to
   * a device registering the same path while this one still occupies it.
   */

  uvc_free_devno(priv->minor);

  /* The hardware is handed back here, but the instance must outlive it
   * while the v4l2 layer still holds dev: its callbacks reach priv through
   * dev->device and rely on disconnected/isoin to refuse further work.
   */

  uinfo("UVC hardware released\n");
  uvc_priv_put(priv);
}

static int uvc_register(FAR struct uvc_priv_s *priv)
{
  FAR struct uvc_dev_s   *dev;
  FAR struct imgsensor_s *sensors[1];
  char                    devpath[32];
  int                     ret;

  snprintf(devpath, sizeof(devpath), "/dev/video%d", priv->minor);

  uinfo("UVC: register devpath=%s\n", devpath);

  dev = kmm_zalloc(sizeof(struct uvc_dev_s));
  if (!dev)
    {
      return -ENOMEM;
    }

  dev->device        = priv;
  priv->dev          = dev;
  dev->minor         = priv->minor;
  dev->imgdata.ops   = &g_uvc_imgdata_ops;
  dev->imgsensor.ops = &g_uvc_imgsensor_ops;

  spin_lock_init(&dev->buffer_lock);

  sensors[0] = &dev->imgsensor;

  ret = capture_register(devpath, &dev->imgdata, sensors, 1);
  if (ret != OK)
    {
      uerr("UVC: capture_register failed: %d\n", ret);
      priv->dev = NULL;
      kmm_free(dev);
      return ret;
    }

  /* The node is live now, so the v4l2 layer owns dev from here on. */

  atomic_add(&priv->crefs, 1);

  uinfo("UVC: registered OK\n");
  return OK;
}

static int uvc_parse_cs_interface(FAR struct uvc_priv_s *priv,
                                  FAR const uint8_t *configdesc,
                                  uint8_t len, uint8_t ifno,
                                  uint8_t subclass,
                                  uint8_t altsetting,
                                  bool in_vs_interface,
                                  FAR struct uvc_format_info_s **format,
                                  FAR struct usbhost_epdesc_s *vs_in_desc)
{
  if (len < 3)
    {
      uerr("ERROR: CS interface descriptor too short\n");
      return OK;
    }

  uint8_t subtype = configdesc[2];

  uinfo("UVC: CS interface subtype: 0x%02x\n", subtype);

  /* Parse VC interface descriptors for PU/CT IDs */

  if (subclass == UVC_SUBCLASS_CONTROL)
    {
      if (subtype == UVC_VC_PROCESSING_UNIT &&
          len >= 8)
        {
          priv->pu_id = configdesc[3];
          priv->ctrl_iface = ifno;

          /* PU descriptor: bControlSize@7, bmControls@8 */

          priv->pu_bmcontrols =
            uvc_read_bmcontrols(&configdesc[8], configdesc[7],
                                len - 8);
          uinfo("UVC: PU id=%d iface=%d bmControls=0x%08"
                PRIx32 "\n",
                priv->pu_id, ifno, priv->pu_bmcontrols);
        }
      else if (subtype == UVC_VC_INPUT_TERMINAL && len >= 8)
        {
          uint16_t term_type = uvc_getle16(&configdesc[4]);

          if (term_type == UVC_ITT_CAMERA)
            {
              priv->ct_id = configdesc[3];
              priv->ctrl_iface = ifno;

              /* Camera Terminal: bControlSize@14, bmControls@15 */

              if (len >= 15)
                {
                  priv->ct_bmcontrols =
                    uvc_read_bmcontrols(&configdesc[15],
                                        configdesc[14],
                                        len - 15);
                }

              uinfo("UVC: CT id=%d iface=%d bmControls=0x%08" PRIx32
                    "\n", priv->ct_id, ifno, priv->ct_bmcontrols);
            }
        }

      return OK;
    }

  if (!in_vs_interface)
    {
      return OK;
    }

  switch (subtype)
    {
      case UVC_VS_INPUT_HEADER:
        {
          uint16_t total_len = uvc_getle16(&configdesc[4]);
          uint8_t  num_fmt   = configdesc[3];
          uint8_t  ep_addr   = configdesc[6];

          if (len < 12)
            {
              uerr("ERROR: VS_INPUT_HEADER descriptor too short\n");
              break;
            }

          uinfo("UVC: VS_INPUT_HEADER ep=0x%02x len=%d nfmt=%d\n",
                ep_addr, total_len, num_fmt);

          /* Initialize format list */

          priv->format_list.num_formats = 0;
          break;
        }

      case UVC_VS_FORMAT_MJPEG:
      case UVC_VS_FORMAT_UNCOMPRESSED:
      case UVC_VS_FORMAT_FRAME_BASED:
        {
          /* Minimum descriptor lengths per UVC spec:
           * MJPEG: 11, Uncompressed: 27, Frame-Based: 28
           */

          uint8_t min_len;

          if (subtype == UVC_VS_FORMAT_MJPEG)
            {
              min_len = 11;
            }
          else if (subtype == UVC_VS_FORMAT_FRAME_BASED)
            {
              min_len = 28;
            }
          else
            {
              min_len = 27;
            }

          if (len < min_len)
            {
              uerr("ERROR: Format descriptor too short\n");
              break;
            }

          uint8_t idx = configdesc[3];

          if (priv->format_list.num_formats < UVC_MAX_FORMATS)
            {
              struct uvc_format_info_s *fmt =
                &priv->format_list.formats[
                  priv->format_list.num_formats++];

              fmt->format_index = idx;
              fmt->num_frames   = 0;

              /* For uncompressed formats, parse the GUID to determine
               * pixel format
               */

              if (subtype == UVC_VS_FORMAT_UNCOMPRESSED &&
                  len >= 27)
                {
                  /* GUID is at offset 5-20 in the descriptor */

                  const uint8_t *guid = &configdesc[5];

                  /* YUY2 GUID: 59555932-0000-0010-8000-00AA00389B71 */

                  static const uint8_t yuy2_guid[16] =
                  {
                    0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* UYVY GUID: 59565955-0000-0010-8000-00AA00389B71 */

                  static const uint8_t uyvy_guid[16] =
                  {
                    0x55, 0x59, 0x56, 0x59, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* NV12 GUID: 3231564E-0000-0010-8000-00AA00389B71 */

                  static const uint8_t nv12_guid[16] =
                  {
                    0x4e, 0x56, 0x31, 0x32, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* YV12 GUID: 32315659-0000-0010-8000-00AA00389B71 */

                  static const uint8_t yv12_guid[16] =
                  {
                    0x59, 0x56, 0x31, 0x32, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* I420 GUID: 30323449-0000-0010-8000-00AA00389B71 */

                  static const uint8_t i420_guid[16] =
                  {
                    0x49, 0x34, 0x32, 0x30, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* M420 GUID: 3032344d-0000-0010-8000-00AA00389B71 */

                  static const uint8_t m420_guid[16] =
                  {
                    0x4d, 0x34, 0x32, 0x30, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* Y800 GUID: 30303859-0000-0010-8000-00AA00389B71 */

                  static const uint8_t y800_guid[16] =
                  {
                    0x59, 0x38, 0x30, 0x30, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* Y8 GUID: 20203859-0000-0010-8000-00AA00389B71 */

                  static const uint8_t y8_guid[16] =
                  {
                    0x59, 0x38, 0x20, 0x20, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* RGBP GUID: 50424752-0000-0010-8000-00AA00389B71 */

                  static const uint8_t rgbp_guid[16] =
                  {
                    0x52, 0x47, 0x42, 0x50, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* BGR3 GUID (BGR24):
                   * e436eb7d-524f-11ce-9f53-0020af0ba770
                   */

                  static const uint8_t bgr3_guid[16] =
                  {
                    0x7d, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11,
                    0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70
                  };

                  if (memcmp(guid, yuy2_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_YUYV;
                      uinfo("UVC: Format %d: YUY2 (YUYV)\n", idx);
                    }
                  else if (memcmp(guid, uyvy_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_UYVY;
                      uinfo("UVC: Format %d: UYVY\n", idx);
                    }
                  else if (memcmp(guid, nv12_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_NV12;
                      uinfo("UVC: Format %d: NV12\n", idx);
                    }
                  else if (memcmp(guid, yv12_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_YVU420;
                      uinfo("UVC: Format %d: YV12\n", idx);
                    }
                  else if (memcmp(guid, i420_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_YUV420;
                      uinfo("UVC: Format %d: I420\n", idx);
                    }
                  else if (memcmp(guid, m420_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_M420;
                      uinfo("UVC: Format %d: M420\n", idx);
                    }
                  else if (memcmp(guid, y800_guid, 16) == 0 ||
                           memcmp(guid, y8_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_GREY;
                      uinfo("UVC: Format %d: GREY\n", idx);
                    }
                  else if (memcmp(guid, rgbp_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_RGB565;
                      uinfo("UVC: Format %d: RGB565\n", idx);
                    }
                  else if (memcmp(guid, bgr3_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_BGR24;
                      uinfo("UVC: Format %d: BGR24\n", idx);
                    }
                  else
                    {
                      uerr("UVC: fmt %d: unknown GUID "
                           "%02x%02x%02x%02x"
                           "%02x%02x%02x%02x"
                           "%02x%02x%02x%02x"
                           "%02x%02x%02x%02x,"
                           " skipping format\n",
                           idx,
                           guid[0],  guid[1],  guid[2],  guid[3],
                           guid[4],  guid[5],  guid[6],  guid[7],
                           guid[8],  guid[9],  guid[10], guid[11],
                           guid[12], guid[13], guid[14], guid[15]);

                      /* Skip unknown format to avoid
                       * silent data corruption.
                       */

                      priv->format_list.num_formats--;
                      *format = NULL;
                    }
                }
              else if (subtype == UVC_VS_FORMAT_MJPEG)
                {
                  /* MJPEG format.
                   * NuttX V4L2 upper layer (v4l2_cap.c
                   * convert_to_imgdatafmt)
                   * only recognizes V4L2_PIX_FMT_JPEG, not
                   * V4L2_PIX_FMT_MJPEG.
                   * Using MJPEG here causes the upper layer
                   * to mismap the format
                   * to IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG,
                   * which then fails format
                   * matching in start_capture. Use JPEG for
                   * compatibility.
                   */

                  fmt->pixel_format_fourcc = V4L2_PIX_FMT_JPEG;
                }
              else if (subtype == UVC_VS_FORMAT_FRAME_BASED &&
                       len >= 28)
                {
                  /* Frame-Based format: H.264, H.265, VP8, etc.
                   * GUID at offset 5-20, same as Uncompressed.
                   * bVariableSize at offset 27 indicates compression.
                   * Reference: Linux uvc_parse_format (uvc_driver.c)
                   */

                  const uint8_t *guid = &configdesc[5];

                  /* H.264 GUID: 48323634-0000-0010-8000-00AA00389B71 */

                  static const uint8_t h264_guid[16] =
                  {
                    0x48, 0x32, 0x36, 0x34, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* H.265/HEVC GUID:
                   * 48323635-0000-0010-8000-00AA00389B71
                   */

                  static const uint8_t h265_guid[16] =
                  {
                    0x48, 0x32, 0x36, 0x35, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  /* HEVC GUID:
                   * 48455643-0000-0010-8000-00AA00389B71
                   */

                  static const uint8_t hevc_guid[16] =
                  {
                    0x48, 0x45, 0x56, 0x43, 0x00, 0x00, 0x10, 0x00,
                    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
                  };

                  if (memcmp(guid, h264_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_H264;
                      uinfo("UVC: Format %d: H.264\n", idx);
                    }
                  else if (memcmp(guid, h265_guid, 16) == 0 ||
                           memcmp(guid, hevc_guid, 16) == 0)
                    {
                      fmt->pixel_format_fourcc = V4L2_PIX_FMT_HEVC;
                      uinfo("UVC: Format %d: H.265/HEVC\n", idx);
                    }
                  else
                    {
                      /* Unknown Frame-Based GUID: construct fourcc from
                       * first 4 bytes
                       */

                      uinfo("UVC: fmt %d: unknown FB GUID "
                            "%02x%02x%02x%02x\n",
                            idx, guid[0], guid[1],
                            guid[2], guid[3]);
                      fmt->pixel_format_fourcc = v4l2_fourcc(guid[0],
                                                             guid[1],
                                                             guid[2],
                                                             guid[3]);
                    }
                }

              *format = fmt;
              uinfo("UVC: fmt %d type=0x%02x "
                    "fourcc=0x%08" PRIx32 "\n",
                    idx, subtype,
                    fmt->pixel_format_fourcc);
            }
          else
            {
              uerr("ERROR: Too many formats\n");
              *format = NULL;
            }

          break;
        }

      case UVC_VS_FRAME_MJPEG:
      case UVC_VS_FRAME_UNCOMPRESSED:
      case UVC_VS_FRAME_FRAME_BASED:
        {
          /* Minimum frame descriptor lengths per UVC spec:
           * Uncompressed/MJPEG: 30, Frame-Based: 26
           */

          uint8_t min_frame_len =
            (subtype == UVC_VS_FRAME_FRAME_BASED) ? 26 : 30;

          if (len < min_frame_len)
            {
              uerr("ERROR: Frame descriptor too short\n");
              break;
            }

          if (*format == NULL)
            {
              uerr("ERROR: Frame descriptor without format\n");
              break;
            }

          if ((*format)->num_frames < UVC_MAX_FRAMES)
            {
              struct uvc_frame_info_s *frame =
                &(*format)->frames[(*format)->num_frames++];
              uint8_t intv_offset;
              uint8_t type_offset;

              frame->frame_index = configdesc[3];
              frame->width       = uvc_getle16(&configdesc[5]);
              frame->height      = uvc_getle16(&configdesc[7]);

              /* dwDefaultFrameInterval and
               * bFrameIntervalType offsets differ:
               * Frame-Based:      dw at 17, type at 21
               * Uncompressed/MJPEG: dw at 21, type at 25
               */

              if (subtype == UVC_VS_FRAME_FRAME_BASED)
                {
                  intv_offset = 17;
                  type_offset = 21;
                }
              else
                {
                  intv_offset = 21;
                  type_offset = 25;
                }

              frame->interval =
                uvc_getle32(&configdesc[intv_offset]);
              frame->num_interval = configdesc[type_offset];

              /* Parse discrete interval list */

              if (frame->num_interval > 0 &&
                  frame->num_interval <= UVC_MAX_FRAME_INTERVALS &&
                  (uint32_t)(type_offset + 1) +
                      frame->num_interval * 4 <= len)
                {
                  uint8_t k;
                  uint8_t list_offset = type_offset + 1;

                  for (k = 0; k < frame->num_interval; k++)
                    {
                      frame->intervals[k] =
                        uvc_getle32(&configdesc[list_offset +
                                                k * 4]);
                    }

                  uinfo("UVC: Frame %d: %dx%d, "
                        "interval=%" PRIu32 ", discrete=%d\n",
                        frame->frame_index,
                        frame->width,
                        frame->height,
                        frame->interval,
                        frame->num_interval);
                }
              else
                {
                  uinfo("UVC: Frame %d: %dx%d, "
                        "interval=%" PRIu32 " (continuous)\n",
                        frame->frame_index,
                        frame->width,
                        frame->height,
                        frame->interval);
                }
            }
          else
            {
              uerr("ERROR: Too many frames for format\n");
            }

          break;
        }

      default:
        uinfo("UVC: Unknown CS interface  subtype: 0x%02x\n",
              subtype);
        break;
    }

  return OK;
  return OK;
}

/****************************************************************************
 * Name: uvc_parse_cfgdesc
 *
 * Description:
 *   Parse UVC configuration descriptor
 *
 ****************************************************************************/

static int uvc_parse_cfgdesc(FAR struct uvc_priv_s *priv,
                             FAR const uint8_t *configdesc,
                             int desclen)
{
  FAR struct uvc_format_info_s *format          = NULL;
  FAR struct usbhost_hubport_s *hport           = priv->usbclass.hport;
  FAR struct usb_cfgdesc_s     *cfgdesc;
  FAR struct usb_desc_s        *desc;
  struct usbhost_epdesc_s       vs_in_desc;
  int                           remaining;
  uint8_t                       ifno            = 0xff;
  uint8_t                       subclass        = 0;
  uint8_t                       altsetting      = 0;
  bool                          iso_found       = false;
  bool                          in_vs_interface = false;
  bool                          vs_iface_found  = false;
  int                           ret;

  memset(&vs_in_desc, 0, sizeof(vs_in_desc));
  priv->stream_alt_count = 0;

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      uerr("ERROR: Not a configuration descriptor\n");
      return -EINVAL;
    }

  remaining = uvc_getle16(cfgdesc->totallen);
  if (remaining > desclen)
    {
      uerr("ERROR: Descriptor length mismatch\n");
      return -EINVAL;
    }

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  uinfo("UVC: Parsing configuration descriptor, total length: %d\n",
        remaining);

  /* Parse all descriptors */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      desc = (FAR struct usb_desc_s *)configdesc;

      /* Validate descriptor length */

      if (desc->len > remaining || desc->len < sizeof(struct usb_desc_s))
        {
          uerr("ERROR: Invalid descriptor length: %d, remaining: %d\n",
               desc->len, remaining);
          break;
        }

      uinfo("UVC: Descriptor type=0x%02x, len=%d\n", desc->type, desc->len);
      switch (desc->type)
        {
          case USB_DESC_TYPE_INTERFACE:
            {
              FAR struct usb_ifdesc_s *ifdesc =
                (FAR struct usb_ifdesc_s *)configdesc;

              if (remaining < USB_SIZEOF_IFDESC)
                {
                  uerr("ERROR: Interface descriptor too short\n");
                  break;
                }

              ifno         = ifdesc->ifno;
              subclass   = ifdesc->subclass;
              altsetting = ifdesc->alt;
              uinfo("UVC: Interface %d, alt %d, class 0x%x, subclass 0x%x\n",
                    ifno, altsetting, ifdesc->classid, subclass);

              /* Handle Video Control interface */

              if (ifdesc->classid == UVC_DEVICE_CLASS &&
                  subclass == UVC_SUBCLASS_CONTROL)
                {
                  in_vs_interface = false;
                  uinfo("UVC: Video Control iface %d\n", ifno);
                }

              /* Handle Video Streaming interface */

              else if (ifdesc->classid == UVC_DEVICE_CLASS &&
                       subclass == UVC_SUBCLASS_STREAMING)
                {
                  priv->stream_iface = ifno;
                  in_vs_interface    = true;
                  vs_iface_found     = true;
                  uinfo("UVC: Found VS interface %d, alt %d\n",
                        ifno, altsetting);
                }

              /* Non-video interface */

              else
                {
                  in_vs_interface = false;
                  uinfo("UVC: Non-video interface, skipping\n");
                }
            }
            break;

          case USB_DESC_TYPE_ENDPOINT:
            {
              FAR struct usb_epdesc_s *epdesc =
                (FAR struct usb_epdesc_s *)configdesc;

              if (remaining < USB_SIZEOF_EPDESC)
                {
                  uerr("ERROR: Endpoint descriptor too short\n");
                  break;
                }

              if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                  USB_EP_ATTR_XFER_ISOC &&
                  in_vs_interface && altsetting >= 1)
                {
                  if (USB_ISEPIN(epdesc->addr))
                    {
                      uint16_t wmax = uvc_getle16(epdesc->mxpacketsize);
                      uint16_t maxp = wmax & 0x07ff;
                      uint8_t  txns = ((wmax >> 11) & 0x03) + 1;
                      uint16_t altmxp = maxp * txns;

                      if (priv->stream_alt_count < UVC_MAX_ALTS)
                        {
                          struct uvc_stream_alt_info_s *ai =
                            &priv->stream_alts[priv->stream_alt_count++];

                          ai->alt = altsetting;
                          ai->max_payload = altmxp;
                          ai->raw_mxpacketsize = wmax;
                          ai->interval = epdesc->interval;
                          uinfo("UVC: alt=%u payload=%u\n",
                                ai->alt, ai->max_payload);
                        }

                      if (!iso_found)
                        {
                          iso_found = true;
                          vs_in_desc.hport = hport;
                          vs_in_desc.addr = epdesc->addr;
                          vs_in_desc.in = 1;
                          vs_in_desc.xfrtype = USB_EP_ATTR_XFER_ISOC;
                          vs_in_desc.interval = epdesc->interval;
                          vs_in_desc.mxpacketsize = wmax;
                          priv->isoc_ep_addr = epdesc->addr;
                          priv->max_pkt_size = altmxp;
                        }
                      else if (altmxp > priv->max_pkt_size)
                        {
                          vs_in_desc.mxpacketsize = wmax;
                          priv->max_pkt_size = altmxp;
                        }

                      uinfo("UVC:VS IN ep=0x%02x alt=%d intv=%d maxpkt=%d\n",
                            epdesc->addr, altsetting, epdesc->interval,
                            altmxp);
                    }
                }
            }
            break;

          case USB_DESC_TYPE_CSINTERFACE:
            ret = uvc_parse_cs_interface(priv, configdesc, desc->len,
                                         ifno, subclass, altsetting,
                                         in_vs_interface, &format,
                                         &vs_in_desc);
            if (ret < 0)
              {
                return ret;
              }

            break;

          default:
            break;
        }

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Validate that we found required interfaces */

  if (!vs_iface_found)
    {
      uerr("ERROR: No Video Streaming interface found\n");
      return -EINVAL;
    }

  if (!iso_found)
    {
      uerr("ERROR: No isochronous IN endpoint found\n");
      return -EINVAL;
    }

  /* Allocate the isochronous endpoint */

  ret = DRVR_EPALLOC(hport->drvr, &vs_in_desc, &priv->isoin);
  if (ret < 0)
    {
      uerr("ERROR: ISO IN epalloc failed: %d\n", ret);
      return ret;
    }

  uinfo("UVC: cfgdesc OK stream=%d fmts=%d\n",
        priv->stream_iface, priv->format_list.num_formats);
  return OK;
}

static int uvc_disconnected(struct usbhost_class_s *hubclass)
{
  FAR struct uvc_priv_s *priv = (FAR struct uvc_priv_s *)hubclass;

  uinfo("UVC device disconnecting\n");

  atomic_set(&priv->disconnected, true);

  work_queue(HPWORK, &priv->disconnect_work, uvc_device_cleanup_worker, priv,
             0);

  return OK;
}

static int uvc_connect(FAR struct usbhost_class_s *hubclass,
                       FAR const uint8_t *configdesc, int desclen)
{
  FAR struct uvc_priv_s *priv = (FAR struct uvc_priv_s *)hubclass;
  int                    ret;

  uinfo("UVC: connecting, desclen=%d\n", desclen);

  if (atomic_read(&priv->disconnected))
    {
      uerr("Device marked as disconnected\n");
      return -ENODEV;
    }

  ret = uvc_parse_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: Failed to parse config descriptor:"
           " %d\n", ret);
      return ret;
    }

  uinfo("UVC: cfgdesc parsed OK\n");

  ret = uvc_register(priv);
  if (ret != OK)
    {
      uerr("ERROR: register UVC failed: %d\n", ret);
      return ret;
    }

  uinfo("UVC: registered OK\n");
  return OK;
}

static FAR struct usbhost_class_s *
uvc_create(FAR struct usbhost_hubport_s *hport,
           FAR const struct usbhost_id_s *id)
{
  FAR struct uvc_priv_s *priv;

  UNUSED(id);

  priv = kmm_zalloc(sizeof(struct uvc_priv_s));
  if (!priv)
    {
      return NULL;
    }

  priv->usbclass.hport        = hport;
  priv->usbclass.connect      = uvc_connect;
  priv->usbclass.disconnected = uvc_disconnected;

  atomic_set(&priv->crefs, 1);

  priv->minor = uvc_alloc_devno();
  if (priv->minor < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return &priv->usbclass;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uvc_initialize
 *
 * Description:
 *   Initialize the USB priv class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_uvc_initialize(void)
{
  /* Advertise our availability to support priv (usb camera) devices */

  return usbhost_registerclass(&g_uvc_registry);
}
