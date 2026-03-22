/****************************************************************************
 * include/nuttx/usb/uvc.h
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

#ifndef __INCLUDE_NUTTX_USB_UVC_H
#define __INCLUDE_NUTTX_USB_UVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UVC device descriptor counts - used by both standalone and composite */

#define USBUVC_NINTERFACES        2     /* VC + VS */
#define USBUVC_NUM_EPS            1     /* Bulk IN only */
#define USBUVC_EP_BULKIN_IDX      0
#define USBUVC_NSTRIDS            2     /* VC iInterface + VS iInterface */
#define USBUVC_NCONFIGS           1

/* UVC Class-Specific Descriptor Types (USB Video Class 1.1, Table A-4) */

#define UVC_CS_UNDEFINED                  0x20
#define UVC_CS_DEVICE                     0x21
#define UVC_CS_CONFIGURATION              0x22
#define UVC_CS_STRING                     0x23
#define UVC_CS_INTERFACE                  0x24
#define UVC_CS_ENDPOINT                   0x25

/* UVC Interface Subclass Codes (Table A-2) */

#define UVC_SC_UNDEFINED                  0x00
#define UVC_SC_VIDEOCONTROL               0x01
#define UVC_SC_VIDEOSTREAMING             0x02
#define UVC_SC_VIDEO_INTERFACE_COLLECTION 0x03

/* UVC Interface Protocol Codes (Table A-3) */

#define UVC_PC_PROTOCOL_UNDEFINED         0x00
#define UVC_PC_PROTOCOL_15                0x01

/* UVC VideoControl Interface Descriptor Subtypes (Table A-5) */

#define UVC_VC_DESCRIPTOR_UNDEFINED       0x00
#define UVC_VC_HEADER                     0x01
#define UVC_VC_INPUT_TERMINAL             0x02
#define UVC_VC_OUTPUT_TERMINAL            0x03
#define UVC_VC_SELECTOR_UNIT              0x04
#define UVC_VC_PROCESSING_UNIT            0x05
#define UVC_VC_EXTENSION_UNIT            0x06

/* UVC VideoStreaming Interface Descriptor Subtypes (Table A-6) */

#define UVC_VS_UNDEFINED                  0x00
#define UVC_VS_INPUT_HEADER               0x01
#define UVC_VS_OUTPUT_HEADER              0x02
#define UVC_VS_STILL_IMAGE_FRAME          0x03
#define UVC_VS_FORMAT_UNCOMPRESSED        0x04
#define UVC_VS_FRAME_UNCOMPRESSED         0x05
#define UVC_VS_FORMAT_MJPEG               0x06
#define UVC_VS_FRAME_MJPEG                0x07
#define UVC_VS_COLOR_FORMAT               0x0d

/* UVC Terminal Types (Table B-1, B-2) */

#define UVC_TT_VENDOR_SPECIFIC            0x0100
#define UVC_TT_STREAMING                  0x0101
#define UVC_ITT_VENDOR_SPECIFIC           0x0200
#define UVC_ITT_CAMERA                    0x0201
#define UVC_ITT_MEDIA_TRANSPORT_INPUT     0x0202
#define UVC_OTT_VENDOR_SPECIFIC           0x0300
#define UVC_OTT_DISPLAY                   0x0301
#define UVC_OTT_MEDIA_TRANSPORT_OUTPUT    0x0302

/* UVC Class-Specific Request Codes (Table A-8) */

#define UVC_RC_UNDEFINED                  0x00
#define UVC_SET_CUR                       0x01
#define UVC_GET_CUR                       0x81
#define UVC_GET_MIN                       0x82
#define UVC_GET_MAX                       0x83
#define UVC_GET_RES                       0x84
#define UVC_GET_LEN                       0x85
#define UVC_GET_INFO                      0x86
#define UVC_GET_DEF                       0x87

/* UVC VideoStreaming Interface Control Selectors (Table A-11) */

#define UVC_VS_CONTROL_UNDEFINED          0x00
#define UVC_VS_PROBE_CONTROL              0x01
#define UVC_VS_COMMIT_CONTROL             0x02

/* UVC Payload Header bits (Table 2-6) */

#define UVC_STREAM_FID                    (1 << 0)
#define UVC_STREAM_EOF                    (1 << 1)
#define UVC_STREAM_PTS                    (1 << 2)
#define UVC_STREAM_SCR                    (1 << 3)
#define UVC_STREAM_RES                    (1 << 4)
#define UVC_STREAM_STI                    (1 << 5)
#define UVC_STREAM_ERR                    (1 << 6)
#define UVC_STREAM_EOH                    (1 << 7)

/* UVC Payload Header Length (minimum, no PTS/SCR) */

#define UVC_PAYLOAD_HEADER_LEN            2

/* GUID for YUY2 (MEDIASUBTYPE_YUY2) — we use this for uncompressed
 * Note: For RGB565 there is no standard UVC GUID. We advertise YUY2
 * and do a trivial RGB565→YUY2 conversion in the app, OR we define
 * a custom GUID. For Phase 1, we use YUY2 which is universally supported.
 */

#define UVC_GUID_FORMAT_YUY2 \
  { 'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00, \
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* UVC Video Probe and Commit Controls (Table 4-75) */

begin_packed_struct struct uvc_streaming_control_s
{
  uint16_t bmhint;
  uint8_t  bformatindex;
  uint8_t  bframeindex;
  uint32_t dwframeinterval;
  uint16_t wkeyframerate;
  uint16_t wpframerate;
  uint16_t wcompquality;
  uint16_t wcompwindowsize;
  uint16_t wdelay;
  uint32_t dwmaxvideoframesize;
  uint32_t dwmaxpayloadtransfersize;
  uint32_t dwclockfrequency;
  uint8_t  bmframinginfo;
  uint8_t  bpreferedversion;
  uint8_t  bminversion;
  uint8_t  bmaxversion;
} end_packed_struct;

#define UVC_PROBE_COMMIT_SIZE  sizeof(struct uvc_streaming_control_s)

/* UVC gadget initialization parameters.
 * The application queries the video device at runtime and passes
 * these to usbdev_uvc_initialize() so that USB descriptors match
 * the actual sensor capabilities.
 */

struct uvc_params_s
{
  uint16_t width;                 /* Frame width in pixels */
  uint16_t height;                /* Frame height in pixels */
  uint8_t  fps;                   /* Frames per second */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: usbdev_uvc_initialize
 *
 * Description:
 *   Initialize the USB Video Class device driver.
 *
 * Input Parameters:
 *   params - Video parameters (width, height, fps) queried from the
 *            camera sensor.  If NULL, defaults to 320x240 @ 5fps.
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.
 *
 ****************************************************************************/

FAR void *usbdev_uvc_initialize(FAR const struct uvc_params_s *params);

/****************************************************************************
 * Name: usbdev_uvc_uninitialize
 *
 * Description:
 *   Uninitialize the USB Video Class device driver.
 *
 ****************************************************************************/

void usbdev_uvc_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: usbdev_uvc_classobject
 *
 * Description:
 *   Create a UVC class driver instance for composite device usage.
 *   Called by the composite framework (or board code) to instantiate
 *   the UVC class driver with assigned interface/string/endpoint bases.
 *
 * Input Parameters:
 *   minor   - Device minor number (unused, pass 0).
 *   devinfo - USB device info with ifnobase, strbase, epno[] assigned
 *             by the composite framework.
 *   params  - Video parameters (width, height, fps).  NULL = defaults.
 *   classdev - Location to return the class driver instance.
 *
 * Returned Value:
 *   OK on success; negative errno on failure.
 *
 ****************************************************************************/

int usbdev_uvc_classobject(int minor,
                           FAR struct usbdev_devinfo_s *devinfo,
                           FAR const struct uvc_params_s *params,
                           FAR struct usbdevclass_driver_s **classdev);

/****************************************************************************
 * Name: usbdev_uvc_classuninitialize
 *
 * Description:
 *   Uninitialize a UVC class driver instance created by classobject.
 *
 ****************************************************************************/

void usbdev_uvc_classuninitialize(
    FAR struct usbdevclass_driver_s *classdev);

/****************************************************************************
 * Name: usbdev_uvc_get_composite_devdesc
 *
 * Description:
 *   Fill in a composite_devdesc_s structure for the UVC gadget.
 *   Board code calls this, then sets ifnobase/strbase/epno[] before
 *   passing to composite_initialize().
 *
 *   Note: classobject/uninitialize are left NULL because UVC needs
 *   extra params (uvc_params_s).  Board code must set them.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_COMPOSITE
void usbdev_uvc_get_composite_devdesc(
    FAR struct composite_devdesc_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_UVC_H */
