/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi_packet.c
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

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/video/mipi_dsi.h>
#include <nuttx/video/mipi_display.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_pixel_format_to_bpp
 *
 * Description:
 *   Obtain the number of bits per pixel for any given pixel format defined
 *   by the MIPI DSI specification
 *
 * Input Parameters:
 *   fmt - MIPI DSI pixel format
 *
 * Returned Value:
 *   The number of bits per pixel of the given pixel format.
 *
 ****************************************************************************/

int mipi_dsi_pixel_format_to_bpp(uint8_t fmt)
{
  switch (fmt)
    {
    case MIPI_DSI_FMT_RGB888:
    case MIPI_DSI_FMT_RGB666:
      return 24;
    case MIPI_DSI_FMT_RGB666_PACKED:
      return 18;
    case MIPI_DSI_FMT_RGB565:
      return 16;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: mipi_dsi_packet_format_is_short
 *
 * Description:
 *   Check if a packet is of the short format
 *
 * Input Parameters:
 *   type - MIPI DSI data type of the packet
 *
 * Returned Value:
 *   True if the packet for the given data type is a short packet, false
 *   otherwise.
 *
 ****************************************************************************/

bool mipi_dsi_packet_format_is_short(uint8_t type)
{
  switch (type)
    {
    case MIPI_DSI_VSYNC_START:
    case MIPI_DSI_VSYNC_END:
    case MIPI_DSI_HSYNC_START:
    case MIPI_DSI_HSYNC_END:
    case MIPI_DSI_COMPRESSION_MODE:
    case MIPI_DSI_END_OF_TRANSMISSION:
    case MIPI_DSI_COLOR_MODE_OFF:
    case MIPI_DSI_COLOR_MODE_ON:
    case MIPI_DSI_SHUTDOWN_PERIPHERAL:
    case MIPI_DSI_TURN_ON_PERIPHERAL:
    case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
    case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
    case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
    case MIPI_DSI_GENERIC_READ_0_PARAM:
    case MIPI_DSI_GENERIC_READ_1_PARAM:
    case MIPI_DSI_GENERIC_READ_2_PARAM:
    case MIPI_DSI_DCS_SHORT_WRITE_0_PARAM:
    case MIPI_DSI_DCS_SHORT_WRITE_1_PARAM:
    case MIPI_DSI_DCS_READ_0_PARAM:
    case MIPI_DSI_EXECUTE_QUEUE:
    case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: mipi_dsi_packet_format_is_long
 *
 * Description:
 *   Check if a packet is of the long format
 *
 * Input Parameters:
 *   type - MIPI DSI data type of the packet
 *
 * Returned Value:
 *   True if the packet for the given data type is a long packet, false
 *   otherwise.
 *
 ****************************************************************************/

bool mipi_dsi_packet_format_is_long(uint8_t type)
{
  switch (type)
    {
    case MIPI_DSI_NULL_PACKET:
    case MIPI_DSI_BLANKING_PACKET:
    case MIPI_DSI_LONG_GENERIC_WRITE:
    case MIPI_DSI_DCS_LONG_WRITE:
    case MIPI_DSI_PICTURE_PARAMETER_SET:
    case MIPI_DSI_COMPRESSED_PIXEL_STREAM:
    case MIPI_DSI_LOOSELY_PACKED_PIXEL_STREAM_YCBCR20:
    case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR24:
    case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR16:
    case MIPI_DSI_PACKED_PIXEL_STREAM_RGB30:
    case MIPI_DSI_PACKED_PIXEL_STREAM_RGB36:
    case MIPI_DSI_PACKED_PIXEL_STREAM_YCBCR12:
    case MIPI_DSI_PACKED_PIXEL_STREAM_RGB16:
    case MIPI_DSI_PACKED_PIXEL_STREAM_RGB18:
    case MIPI_DSI_PIXEL_STREAM_3BYTE_RGB18:
    case MIPI_DSI_PACKED_PIXEL_STREAM_RGB24:
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: mipi_dsi_create_packet
 *
 * Description:
 *   Create a packet from a message according to the DSI protocol
 *
 * Input Parameters:
 *   packet - Pointer to a DSI packet structure
 *   msg    - Message to translate into a packet
 *
 * Returned Value:
 *   Return: 0 on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_create_packet(FAR struct mipi_dsi_packet *packet,
                           FAR const struct mipi_dsi_msg *msg)
{
  DEBUGASSERT(packet != NULL);
  DEBUGASSERT(msg != NULL);

  /* do some minimum sanity checking */

  if (!mipi_dsi_packet_format_is_short(msg->type) &&
      !mipi_dsi_packet_format_is_long(msg->type))
    {
      return -EINVAL;
    }

  if (msg->channel > 3)
    {
      return -EINVAL;
    }

  memset(packet, 0, sizeof(*packet));
  packet->header[0] = (msg->channel << 6) | (msg->type & 0x3f);

  /* TODO: compute ECC if hardware support is not available */

  /* Long write packets contain the word count in header bytes 1 and 2.
   * The payload follows the header and is word count bytes long.
   *
   * Short write packets encode up to two parameters in header bytes 1
   * and 2.
   */

  if (mipi_dsi_packet_format_is_long(msg->type))
    {
      packet->header[1] = (msg->tx_len >> 0) & 0xff;
      packet->header[2] = (msg->tx_len >> 8) & 0xff;

      packet->payload_length = msg->tx_len;
      packet->payload = msg->tx_buf;
    }
  else
    {
      FAR const uint8_t *tx = msg->tx_buf;

      packet->header[1] = (msg->tx_len > 0) ? tx[0] : 0;
      packet->header[2] = (msg->tx_len > 1) ? tx[1] : 0;
    }

  packet->size = sizeof(packet->header) + packet->payload_length;

  return OK;
}
