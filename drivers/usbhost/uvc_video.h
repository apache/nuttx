/****************************************************************************
 * drivers/usbhost/uvc_video.h
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

#ifndef __INCLUDE_NUTTX_USB_VIDEO_H
#define __INCLUDE_NUTTX_USB_VIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/usb/usb.h>

#define UVC_DEVICE_CLASS                    USB_CLASS_VIDEO
#define UVC_DEVICE_SUBCLASS                 0x01
#define UVC_DEVICE_PROTOCOL                 0x00

#define UVC_SUBCLASS_CONTROL                0x01
#define UVC_SUBCLASS_STREAMING              0x02
#define UVC_SUBCLASS_INTERFACE_COLLECTION   0x03

#define UVC_PROTOCOL_UNDEF                  0x00
#define UVC_PROTOCOL_15                     0x01

#define UVC_CS_INTERFACE                    0x24
#define UVC_CS_ENDPOINT                     0x25

#define UVC_VC_DESCRIPTOR_UNDEFINED         0x00
#define UVC_VC_HEADER                       0x01
#define UVC_VC_INPUT_TERMINAL               0x02
#define UVC_VC_OUTPUT_TERMINAL              0x03
#define UVC_VC_SELECTOR_UNIT                0x04
#define UVC_VC_PROCESSING_UNIT              0x05
#define UVC_VC_EXTENSION_UNIT               0x06

#define UVC_VS_DESCRIPTOR_UNDEFINED         0x00
#define UVC_VS_INPUT_HEADER                 0x01
#define UVC_VS_OUTPUT_HEADER                0x02
#define UVC_VS_STILL_IMAGE_FRAME            0x03
#define UVC_VS_FORMAT_UNCOMPRESSED          0x04
#define UVC_VS_FRAME_UNCOMPRESSED           0x05
#define UVC_VS_FORMAT_MJPEG                 0x06
#define UVC_VS_FRAME_MJPEG                  0x07
#define UVC_VS_FORMAT_MPEG2TS               0x0a
#define UVC_VS_FORMAT_DV                    0x0c
#define UVC_VS_COLORFORMAT                  0x0d
#define UVC_VS_FORMAT_FRAME_BASED           0x10
#define UVC_VS_FRAME_FRAME_BASED            0x11
#define UVC_VS_FORMAT_STREAM_BASED          0x12

#define UVC_EP_UNDEFINED                    0x00
#define UVC_EP_GENERAL                      0x01
#define UVC_EP_ENDPOINT                     0x02
#define UVC_EP_INTERRUPT                    0x03

#define UVC_FORMAT_TYPE_UNDEFINED           0x00
#define UVC_FORMAT_TYPE_I                   0x01
#define UVC_FORMAT_TYPE_II                  0x02
#define UVC_FORMAT_TYPE_III                 0x03
#define UVC_FORMAT_TYPE_IV                  0x04

#define UVC_FRAME_TYPE_UNDEFINED            0x00
#define UVC_FRAME_TYPE_I                    0x01
#define UVC_FRAME_TYPE_P                    0x02
#define UVC_FRAME_TYPE_B                    0x03

#define UVC_REQUEST_UNDEFINED               0x00
#define UVC_REQUEST_SET_CUR                 0x01
#define UVC_REQUEST_GET_CUR                 0x81
#define UVC_REQUEST_GET_MIN                 0x82
#define UVC_REQUEST_GET_MAX                 0x83
#define UVC_REQUEST_GET_RES                 0x84
#define UVC_REQUEST_GET_LEN                 0x85
#define UVC_REQUEST_GET_INFO                0x86
#define UVC_REQUEST_GET_DEF                 0x87

#define UVC_TT_STREAMING                    0x0101
#define UVC_ITT_CAMERA                      0x0201
#define UVC_ITT_MEDIA_TRANSPORT_INPUT       0x0202
#define UVC_OTT_DISPLAY                     0x0301
#define UVC_OTT_MEDIA_TRANSPORT_OUTPUT      0x0302

#define UVC_VS_PROBE_CONTROL                0x01
#define UVC_VS_COMMIT_CONTROL               0x02

#define UVC_REQ_GET_CUR                     0x81
#define UVC_REQ_SET_CUR                     0x01

/* Camera Terminal Control Selectors (A.9.4) */

#define UVC_CT_SCANNING_MODE_CONTROL            0x01
#define UVC_CT_AE_MODE_CONTROL                  0x02
#define UVC_CT_AE_PRIORITY_CONTROL              0x03
#define UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL   0x04
#define UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL   0x05
#define UVC_CT_FOCUS_ABSOLUTE_CONTROL           0x06
#define UVC_CT_FOCUS_RELATIVE_CONTROL           0x07
#define UVC_CT_FOCUS_AUTO_CONTROL               0x08
#define UVC_CT_IRIS_ABSOLUTE_CONTROL            0x09
#define UVC_CT_IRIS_RELATIVE_CONTROL            0x0a
#define UVC_CT_ZOOM_ABSOLUTE_CONTROL            0x0b
#define UVC_CT_ZOOM_RELATIVE_CONTROL            0x0c
#define UVC_CT_PANTILT_ABSOLUTE_CONTROL         0x0d
#define UVC_CT_PANTILT_RELATIVE_CONTROL         0x0e
#define UVC_CT_ROLL_ABSOLUTE_CONTROL            0x0f
#define UVC_CT_ROLL_RELATIVE_CONTROL            0x10
#define UVC_CT_PRIVACY_CONTROL                  0x11

/* Processing Unit Control Selectors (A.9.5) */

#define UVC_PU_BACKLIGHT_COMPENSATION_CONTROL          0x01
#define UVC_PU_BRIGHTNESS_CONTROL                      0x02
#define UVC_PU_CONTRAST_CONTROL                        0x03
#define UVC_PU_GAIN_CONTROL                            0x04
#define UVC_PU_POWER_LINE_FREQUENCY_CONTROL            0x05
#define UVC_PU_HUE_CONTROL                             0x06
#define UVC_PU_SATURATION_CONTROL                      0x07
#define UVC_PU_SHARPNESS_CONTROL                       0x08
#define UVC_PU_GAMMA_CONTROL                           0x09
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL       0x0a
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL  0x0b
#define UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL         0x0c
#define UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL    0x0d
#define UVC_PU_DIGITAL_MULTIPLIER_CONTROL              0x0e
#define UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL        0x0f
#define UVC_PU_HUE_AUTO_CONTROL                        0x10
#define UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL           0x11
#define UVC_PU_ANALOG_LOCK_STATUS_CONTROL              0x12

/* Camera Terminal bmControls bit positions (UVC 1.5 Table 3-6). Not the
 * selectors above: the spec leaves gaps (Zoom Abs is selector 0x0b, bit 9).
 */

#define UVC_CT_SCANNING_MODE_BIT            0
#define UVC_CT_AE_MODE_BIT                  1
#define UVC_CT_AE_PRIORITY_BIT              2
#define UVC_CT_EXPOSURE_TIME_ABSOLUTE_BIT   3
#define UVC_CT_EXPOSURE_TIME_RELATIVE_BIT   4
#define UVC_CT_FOCUS_ABSOLUTE_BIT           5
#define UVC_CT_FOCUS_RELATIVE_BIT           6
#define UVC_CT_IRIS_ABSOLUTE_BIT            7
#define UVC_CT_IRIS_RELATIVE_BIT            8
#define UVC_CT_ZOOM_ABSOLUTE_BIT            9
#define UVC_CT_ZOOM_RELATIVE_BIT            10
#define UVC_CT_PANTILT_ABSOLUTE_BIT         11
#define UVC_CT_PANTILT_RELATIVE_BIT         12
#define UVC_CT_ROLL_ABSOLUTE_BIT            13
#define UVC_CT_ROLL_RELATIVE_BIT            14
#define UVC_CT_FOCUS_AUTO_BIT               17
#define UVC_CT_PRIVACY_BIT                  18

/* Processing Unit bmControls bit positions (UVC 1.5 Table 3-8). */

#define UVC_PU_BRIGHTNESS_BIT                      0
#define UVC_PU_CONTRAST_BIT                        1
#define UVC_PU_HUE_BIT                             2
#define UVC_PU_SATURATION_BIT                      3
#define UVC_PU_SHARPNESS_BIT                       4
#define UVC_PU_GAMMA_BIT                           5
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_BIT       6
#define UVC_PU_WHITE_BALANCE_COMPONENT_BIT         7
#define UVC_PU_BACKLIGHT_COMPENSATION_BIT          8
#define UVC_PU_GAIN_BIT                            9
#define UVC_PU_POWER_LINE_FREQUENCY_BIT            10
#define UVC_PU_HUE_AUTO_BIT                        11
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_BIT  12
#define UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_BIT    13
#define UVC_PU_DIGITAL_MULTIPLIER_BIT              14
#define UVC_PU_DIGITAL_MULTIPLIER_LIMIT_BIT        15
#define UVC_PU_ANALOG_VIDEO_STANDARD_BIT           16
#define UVC_PU_ANALOG_LOCK_STATUS_BIT              17

/* UVC control unit types */

#define UVC_CTRL_UNIT_PU            0
#define UVC_CTRL_UNIT_CT            1

/* UVC VS_PROBE_COMMIT_CONTROL payload size (UVC 1.5 Table 4-47) */

#define UVC_PROBE_COMMIT_SIZE       26

#endif /* __INCLUDE_NUTTX_USB_VIDEO_H */
