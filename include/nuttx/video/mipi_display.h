/****************************************************************************
 * include/nuttx/video/mipi_display.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing software
 * distributed under the License is distributed on an "AS IS" BASIS WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_MIPI_DISPLAY_H
#define __INCLUDE_NUTTX_VIDEO_MIPI_DISPLAY_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSI Processor-to-Peripheral transaction types */

#define MIPI_DSI_VSYNC_START                          0x01
#define MIPI_DSI_VSYNC_END                            0x11
#define MIPI_DSI_HSYNC_START                          0x21
#define MIPI_DSI_HSYNC_END                            0x31
#define MIPI_DSI_COMPRESSION_MODE                     0x07
#define MIPI_DSI_END_OF_TRANSMISSION                  0x08
#define MIPI_DSI_COLOR_MODE_OFF                       0x02
#define MIPI_DSI_COLOR_MODE_ON                        0x12
#define MIPI_DSI_SHUTDOWN_PERIPHERAL                  0x22
#define MIPI_DSI_TURN_ON_PERIPHERAL                   0x32
#define MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM          0x03
#define MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM          0x13
#define MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM          0x23
#define MIPI_DSI_GENERIC_READ_0_PARAM                 0x04
#define MIPI_DSI_GENERIC_READ_1_PARAM                 0x14
#define MIPI_DSI_GENERIC_READ_2_PARAM                 0x24
#define MIPI_DSI_DCS_SHORT_WRITE_0_PARAM              0x05
#define MIPI_DSI_DCS_SHORT_WRITE_1_PARAM              0x15
#define MIPI_DSI_DCS_READ_0_PARAM                     0x06
#define MIPI_DSI_EXECUTE_QUEUE                        0x16
#define MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE       0x37
#define MIPI_DSI_NULL_PACKET                          0x09
#define MIPI_DSI_BLANKING_PACKET                      0x19
#define MIPI_DSI_LONG_GENERIC_WRITE                   0x29
#define MIPI_DSI_DCS_LONG_WRITE                       0x39
#define MIPI_DSI_PICTURE_PARAMETER_SET                0x0a
#define MIPI_DSI_COMPRESSED_PIXEL_STREAM              0x0b
#define MIPI_DSI_LOOSELY_PACKED_PIXEL_STREAM_YCBCR20  0x0c
#define MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR24          0x1c
#define MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR16          0x2c
#define MIPI_DSI_PACKED_PIXEL_STREAM_RGB30            0x0d
#define MIPI_DSI_PACKED_PIXEL_STREAM_RGB36            0x1d
#define MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR12          0x3d
#define MIPI_DSI_PACKED_PIXEL_STREAM_RGB16            0x0e
#define MIPI_DSI_PACKED_PIXEL_STREAM_RGB18            0x1e
#define MIPI_DSI_PIXEL_STREAM_3BYTE_RGB18             0x2e
#define MIPI_DSI_PACKED_PIXEL_STREAM_RGB24            0x3e

/* DSI Peripheral-to-Processor transaction data type */

#define MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT      0x02
#define MIPI_DSI_RX_END_OF_TRANSMISSION               0x08
#define MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE 0x11
#define MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE 0x12
#define MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE        0x1a
#define MIPI_DSI_RX_DCS_LONG_READ_RESPONSE            0x1c
#define MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE     0x21
#define MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE     0x22

/* DCS commands */

#define MIPI_DCS_NOP                      0x00
#define MIPI_DCS_SOFT_RESET               0x01
#define MIPI_DCS_GET_COMPRESSION_MODE     0x03
#define MIPI_DCS_GET_DISPLAY_ID           0x04
#define MIPI_DCS_GET_ERROR_COUNT_ON_DSI   0x05
#define MIPI_DCS_GET_RED_CHANNEL          0x06
#define MIPI_DCS_GET_GREEN_CHANNEL        0x07
#define MIPI_DCS_GET_BLUE_CHANNEL         0x08
#define MIPI_DCS_GET_DISPLAY_STATUS       0x09
#define MIPI_DCS_GET_POWER_MODE           0x0a
#define MIPI_DCS_GET_ADDRESS_MODE         0x0b
#define MIPI_DCS_GET_PIXEL_FORMAT         0x0c
#define MIPI_DCS_GET_DISPLAY_MODE         0x0d
#define MIPI_DCS_GET_SIGNAL_MODE          0x0e
#define MIPI_DCS_GET_DIAGNOSTIC_RESULT    0x0f
#define MIPI_DCS_ENTER_SLEEP_MODE         0x10
#define MIPI_DCS_EXIT_SLEEP_MODE          0x11
#define MIPI_DCS_ENTER_PARTIAL_MODE       0x12
#define MIPI_DCS_ENTER_NORMAL_MODE        0x13
#define MIPI_DCS_GET_IMAGE_CHECKSUM_RGB   0x14
#define MIPI_DCS_GET_IMAGE_CHECKSUM_CT    0x15
#define MIPI_DCS_EXIT_INVERT_MODE         0x20
#define MIPI_DCS_ENTER_INVERT_MODE        0x21
#define MIPI_DCS_SET_GAMMA_CURVE          0x26
#define MIPI_DCS_SET_DISPLAY_OFF          0x28
#define MIPI_DCS_SET_DISPLAY_ON           0x29
#define MIPI_DCS_SET_COLUMN_ADDRESS       0x2a
#define MIPI_DCS_SET_PAGE_ADDRESS         0x2b
#define MIPI_DCS_WRITE_MEMORY_START       0x2c
#define MIPI_DCS_WRITE_LUT                0x2d
#define MIPI_DCS_READ_MEMORY_START        0x2e
#define MIPI_DCS_SET_PARTIAL_ROWS         0x30    /* MIPI DCS 1.02 -
                                                   * MIPI_DCS_SET_PARTIAL_AREA
                                                   * before that */
#define MIPI_DCS_SET_PARTIAL_COLUMNS      0x31
#define MIPI_DCS_SET_SCROLL_AREA          0x33
#define MIPI_DCS_SET_TEAR_OFF             0x34
#define MIPI_DCS_SET_TEAR_ON              0x35
#define MIPI_DCS_SET_ADDRESS_MODE         0x36
#define MIPI_DCS_SET_SCROLL_START         0x37
#define MIPI_DCS_EXIT_IDLE_MODE           0x38
#define MIPI_DCS_ENTER_IDLE_MODE          0x39
#define MIPI_DCS_SET_PIXEL_FORMAT         0x3a
#define MIPI_DCS_WRITE_MEMORY_CONTINUE    0x3c
#define MIPI_DCS_SET_3D_CONTROL           0x3d
#define MIPI_DCS_READ_MEMORY_CONTINUE     0x3e
#define MIPI_DCS_GET_3D_CONTROL           0x3f
#define MIPI_DCS_SET_VSYNC_TIMING         0x40
#define MIPI_DCS_SET_TEAR_SCANLINE        0x44
#define MIPI_DCS_GET_SCANLINE             0x45
#define MIPI_DCS_SET_DISPLAY_BRIGHTNESS   0x51    /* MIPI DCS 1.3 */
#define MIPI_DCS_GET_DISPLAY_BRIGHTNESS   0x52    /* MIPI DCS 1.3 */
#define MIPI_DCS_WRITE_CONTROL_DISPLAY    0x53    /* MIPI DCS 1.3 */
#define MIPI_DCS_GET_CONTROL_DISPLAY      0x54    /* MIPI DCS 1.3 */
#define MIPI_DCS_WRITE_POWER_SAVE         0x55    /* MIPI DCS 1.3 */
#define MIPI_DCS_GET_POWER_SAVE           0x56    /* MIPI DCS 1.3 */
#define MIPI_DCS_SET_CABC_MIN_BRIGHTNESS  0x5e    /* MIPI DCS 1.3 */
#define MIPI_DCS_GET_CABC_MIN_BRIGHTNESS  0x5f    /* MIPI DCS 1.3 */
#define MIPI_DCS_READ_DDB_START           0xa1
#define MIPI_DCS_READ_PPS_START           0xa2
#define MIPI_DCS_READ_DDB_CONTINUE        0xa8
#define MIPI_DCS_READ_PPS_CONTINUE        0xa9

/* DCS pixel formats */

#define MIPI_DCS_PIXEL_FMT_24BIT  7
#define MIPI_DCS_PIXEL_FMT_18BIT  6
#define MIPI_DCS_PIXEL_FMT_16BIT  5
#define MIPI_DCS_PIXEL_FMT_12BIT  3
#define MIPI_DCS_PIXEL_FMT_8BIT   2
#define MIPI_DCS_PIXEL_FMT_3BIT   1

#endif /* __INCLUDE_NUTTX_VIDEO_MIPI_DISPLAY_H */
