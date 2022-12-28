/****************************************************************************
 * drivers/video/mipidsi/mipi_dsi_device.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <nuttx/video/mipi_display.h>
#include <nuttx/kmalloc.h>

#include "mipi_dsi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_transfer
 *
 * Description:
 *   Transfer message to display modules
 *
 * Input Parameters:
 *   device - DSI peripheral
 *   msg - Message to transfer
 *
 * Returned Value:
 *   The number of bytes successfully transfered or a negative error code on
 *   failure.
 *
 ****************************************************************************/

ssize_t mipi_dsi_transfer(FAR struct mipi_dsi_device *device,
                          FAR struct mipi_dsi_msg *msg)
{
  FAR const struct mipi_dsi_host_ops *ops = device->host->ops;

  if (ops == NULL || ops->transfer == NULL)
    {
      return -ENOSYS;
    }

  if (device->mode_flags & MIPI_DSI_MODE_LPM)
    {
      msg->flags |= MIPI_DSI_MSG_USE_LPM;
    }

  if (device->mode_flags & MIPI_DSI_MODE_AFTER_FRAME)
    {
      msg->flags |= MIPI_DSI_MSG_AFTER_FRAME;
    }

  return ops->transfer(device->host, msg);
}

/****************************************************************************
 * Name: mipi_dsi_attach
 *
 * Description:
 *   Attach a DSI device to its DSI host
 *
 * Input Parameters:
 *   device - DSI peripheral
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_attach(FAR struct mipi_dsi_device *device)
{
  FAR const struct mipi_dsi_host_ops *ops = device->host->ops;

  if (ops == NULL || ops->attach == NULL)
    {
      return -ENOSYS;
    }

  return ops->attach(device->host, device);
}

/****************************************************************************
 * Name: mipi_dsi_detach
 *
 * Description:
 *   Detach a DSI device from its DSI host
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_detach(FAR struct mipi_dsi_device *device)
{
  FAR const struct mipi_dsi_host_ops *ops = device->host->ops;

  if (ops == NULL || ops->detach == NULL)
    {
      return -ENOSYS;
    }

  return ops->detach(device->host, device);
}

/****************************************************************************
 * Name: mipi_dsi_shutdown_peripheral
 *
 * Description:
 *   Send a Shutdown Peripheral command to the device device
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_shutdown_peripheral(FAR struct mipi_dsi_device *device)
{
  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .type = MIPI_DSI_SHUTDOWN_PERIPHERAL,
    .tx_buf = (uint8_t [2]) { 0, 0 },
    .tx_len = 2,
  };

  int ret = mipi_dsi_transfer(device, &msg);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: mipi_dsi_turn_on_peripheral
 *
 * Description:
 *   Send a Turn On Peripheral command to the device device
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_turn_on_peripheral(FAR struct mipi_dsi_device *device)
{
  int ret;

  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .type = MIPI_DSI_TURN_ON_PERIPHERAL,
    .tx_buf = (uint8_t [2]) { 0, 0 },
    .tx_len = 2,
  };

  ret = mipi_dsi_transfer(device, &msg);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: mipi_dsi_set_maximum_return_packet_size
 *
 * Description:
 *   Specify the maximum size of the payload in a long packet transmitted
 *   from the peripheral back to the device host processor
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   value - The maximum size of the payload
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_set_maximum_return_packet_size(
                          FAR struct mipi_dsi_device *device, uint16_t value)
{
  int ret;

  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
    .tx_len = 2,
    .tx_buf = (uint8_t [2]) { value & 0xff, value >> 8 },
  };

  ret = mipi_dsi_transfer(device, &msg);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: mipi_dsi_compression_mode
 *
 * Description:
 *   Enable / disable DSC on the peripheral. Enable or disable Display Stream
 *   Compression on the peripheral using the default Picture Parameter Set
 *   and VESA DSC 1.1 algorithm.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   enable - Whether to enable or disable the DSC
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_compression_mode(FAR struct mipi_dsi_device *device,
                              bool enable)
{
  /* Note: Needs updating for non-default PPS or algorithm */

  int ret;

  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .type = MIPI_DSI_COMPRESSION_MODE,
    .tx_len = 2,
    .tx_buf = (uint8_t [2]) { enable & 0xff, 0 },
  };

  ret = mipi_dsi_transfer(device, &msg);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: mipi_dsi_generic_write
 *
 * Description:
 *   Transmit data using a generic write packet
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   payload - buffer containing the payload
 *   size - size of the payload
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_generic_write(FAR struct mipi_dsi_device *device,
                           FAR const void *payload,
                           size_t size)
{
  int ret;

  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .tx_buf = payload,
    .tx_len = size
  };

  switch (size)
    {
    case 0:
      msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM;
      break;

    case 1:
      msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM;
      break;

    case 2:
      msg.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM;
      break;

    default:
      msg.type = MIPI_DSI_LONG_GENERIC_WRITE;
      break;
    }

  ret = mipi_dsi_transfer(device, &msg);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: mipi_dsi_generic_read
 *
 * Description:
 *   Receive data using a generic read packet.
 *   This function will automatically choose the right data type depending on
 *   the number of parameters passed in.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   params - buffer containing the request parameters
 *   num_params - number of request parameters
 *   data - buffer in which to return the received data
 *   size - size of receive buffer
 *
 * Returned Value:
 *   The number of bytes successfully read or a negative error code on
 *   failure.
 *
 ****************************************************************************/

ssize_t mipi_dsi_generic_read(FAR struct mipi_dsi_device *device,
                              FAR const void *params,
                              size_t num_params,
                              FAR void *data,
                              size_t size)
{
  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .tx_len = num_params,
    .tx_buf = params,
    .rx_len = size,
    .rx_buf = data
  };

  switch (num_params)
    {
    case 0:
      msg.type = MIPI_DSI_GENERIC_READ_0_PARAM;
      break;
    case 1:
      msg.type = MIPI_DSI_GENERIC_READ_1_PARAM;
      break;
    case 2:
      msg.type = MIPI_DSI_GENERIC_READ_2_PARAM;
      break;
    default:
      return -EINVAL;
    }

  return mipi_dsi_transfer(device, &msg);
}

/****************************************************************************
 * Name: mipi_dsi_dcs_write_buffer
 *
 * Description:
 *   Transmit a DCS command with payload.
 *   This function will automatically choose the right data type depending on
 *   the command payload length.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   data - buffer containing data to be transmitted
 *   len - size of transmission buffer
 *
 * Returned Value:
 *   The number of bytes successfully transmitted or a negative error
 *   code on failure.
 *
 ****************************************************************************/

ssize_t mipi_dsi_dcs_write_buffer(FAR struct mipi_dsi_device *device,
                                  FAR const void *data,
                                  size_t len)
{
  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .tx_buf = data,
    .tx_len = len
  };

  switch (len)
    {
    case 0:
      return -EINVAL;
    case 1:
      msg.type = MIPI_DSI_DCS_SHORT_WRITE_0_PARAM;
      break;
    case 2:
      msg.type = MIPI_DSI_DCS_SHORT_WRITE_1_PARAM;
      break;
    default:
      msg.type = MIPI_DSI_DCS_LONG_WRITE;
      break;
    }

  return mipi_dsi_transfer(device, &msg);
}

/****************************************************************************
 * Name: mipi_dsi_dcs_write
 *
 * Description:
 *   Send DCS write command.
 *   This function will automatically choose the right data type depending on
 *   the command payload length.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   cmd - DCS command
 *   data - buffer containing the command payload
 *   len - command payload length
 *
 * Returned Value:
 *   The number of bytes successfully transmitted or a negative error
 *   code on failure.
 *
 ****************************************************************************/

ssize_t mipi_dsi_dcs_write(FAR struct mipi_dsi_device *device,
                           uint8_t cmd,
                           FAR const void *data,
                           size_t len)
{
  ssize_t ret;
  uint8_t stack_tx[8];
  FAR uint8_t *tx = stack_tx;

  if (len > sizeof(stack_tx) - 1)
    {
      tx = kmm_malloc(len + 1);
      if (!tx)
        {
          return -ENOMEM;
        }
    }

  /* concatenate the DCS command byte and the payload */

  tx[0] = cmd;
  if (data)
    {
      memcpy(&tx[1], data, len);
    }

  ret = mipi_dsi_dcs_write_buffer(device, tx, len + 1);

  if (tx != stack_tx)
    {
      kmm_free(tx);
    }

  return ret;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_read
 *
 * Description:
 *   Send DCS read request command.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   cmd - DCS command
 *   data - buffer in which to receive data
 *   len - size of receive buffer
 *
 * Returned Value:
 *   The number of bytes read or a negative error code on failure.
 *
 ****************************************************************************/

ssize_t mipi_dsi_dcs_read(FAR struct mipi_dsi_device *device,
                          uint8_t cmd,
                          FAR void *data,
                          size_t len)
{
  struct mipi_dsi_msg msg =
  {
    .channel = device->channel,
    .type = MIPI_DSI_DCS_READ_0_PARAM,
    .tx_buf = &cmd,
    .tx_len = 1,
    .rx_buf = data,
    .rx_len = len
  };

  return mipi_dsi_transfer(device, &msg);
}

/****************************************************************************
 * Name: mipi_dsi_dcs_nop
 *
 * Description:
 *   Send DCS nop packet
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_nop(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_NOP, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_soft_reset
 *
 * Description:
 *   Send a software reset of the display module
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_soft_reset(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SOFT_RESET, NULL, 0);

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_get_power_mode
 *
 * Description:
 *   Query the display module's current power mode
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   mode - return location of the current power mode
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_get_power_mode(FAR struct mipi_dsi_device *device,
                                FAR uint8_t *mode)
{
  int ret;

  ret = mipi_dsi_dcs_read(device, MIPI_DCS_GET_POWER_MODE, mode,
                          sizeof(*mode));
  if (ret <= 0)
    {
      if (ret == 0)
        {
          ret = -ENODATA;
        }

      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_get_pixel_format
 *
 * Description:
 *   Gets the pixel format for the RGB image data used by the display module.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   format - return location of the pixel format
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_get_pixel_format(FAR struct mipi_dsi_device *device,
                                  FAR uint8_t *format)
{
  int ret;

  ret = mipi_dsi_dcs_read(device, MIPI_DCS_GET_PIXEL_FORMAT, format,
                          sizeof(*format));
  if (ret <= 0)
    {
      if (ret == 0)
        {
          ret = -ENODATA;
        }

      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_enter_sleep_mode
 *
 * Description:
 *   Send a Enter Sleep Mode command to display module.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_enter_sleep_mode(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_ENTER_SLEEP_MODE, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_exit_sleep_mode
 *
 * Description:
 *   Send a Exit Sleep Mode command to display module.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_exit_sleep_mode(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_display_off
 *
 * Description:
 *   Send a Display OFF command to display module. Stop displaying the image
 *   data on the display device.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_display_off(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_display_on
 *
 * Description:
 *   Send a Display On command to display module. Start displaying the image
 *   data on the display device.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_display_on(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_column_address
 *
 * Description:
 *   Define the column extent of the frame memory accessed by the host
 *   processor
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   start - first column address of frame memory
 *   end - last column address of frame memory
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_column_address(FAR struct mipi_dsi_device *device,
                                    uint16_t start,
                                    uint16_t end)
{
  uint8_t payload[4] =
  {
    start >> 8,
    start & 0xff,
    end >> 8,
    end & 0xff
  };

  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_COLUMN_ADDRESS, payload,
                           sizeof(payload));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_page_address
 *
 * Description:
 *   Define the page extent of the frame memory accessed by the host
 *   processor
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   start - first page address of frame memory
 *   end - last page address of frame memory
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_page_address(FAR struct mipi_dsi_device *device,
                                  uint16_t start,
                                  uint16_t end)
{
  uint8_t payload[4] =
  {
    start >> 8,
    start & 0xff,
    end >> 8,
    end & 0xff
  };

  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_PAGE_ADDRESS, payload,
                           sizeof(payload));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_tear_off
 *
 * Description:
 *   Turn off the display module's Tearing Effect output signal on the TE
 *   signal line
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_tear_off(FAR struct mipi_dsi_device *device)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_TEAR_OFF, NULL, 0);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_tear_on
 *
 * Description:
 *   Turn on the display module's Tearing Effect output signal on the TE
 *   signal line.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   mode - Tearing Mode, MIPI_DSI_DCS_TEAR_MODE_VBLANK or
 *          MIPI_DSI_DCS_TEAR_MODE_VHBLANK
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_tear_on(FAR struct mipi_dsi_device *device,
                             uint8_t mode)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_TEAR_ON, &mode,
                           sizeof(mode));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_pixel_format
 *
 * Description:
 *   Sets the pixel format for the RGB image data used by the display module.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   format - pixel format
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_pixel_format(FAR struct mipi_dsi_device *device,
                                  uint8_t format)
{
  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_PIXEL_FORMAT, &format,
                           sizeof(format));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_tear_scanline
 *
 * Description:
 *   Set the scanline to use as trigger for the Tearing Effect output signal
 *   of the display module.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   scanline - scanline to use as trigger
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_tear_scanline(FAR struct mipi_dsi_device *device,
                                   uint16_t scanline)
{
  uint8_t payload[2] =
  {
    scanline >> 8,
    scanline & 0xff
  };

  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_TEAR_SCANLINE, payload,
                           sizeof(payload));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_set_display_brightness
 *
 * Description:
 *   Sets the brightness value of the display.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   brightness - brightness value
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_set_display_brightness(FAR struct mipi_dsi_device *device,
                                        uint16_t brightness)
{
  uint8_t payload[2] =
  {
    brightness & 0xff,
    brightness >> 8
  };

  int ret;

  ret = mipi_dsi_dcs_write(device, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
                           payload, sizeof(payload));
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: mipi_dsi_dcs_get_display_brightness
 *
 * Description:
 *   Gets the current brightness value of the display.
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *   brightness - brightness value
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_dcs_get_display_brightness(FAR struct mipi_dsi_device *device,
                                        FAR uint16_t *brightness)
{
  int ret;

  ret = mipi_dsi_dcs_read(device, MIPI_DCS_GET_DISPLAY_BRIGHTNESS,
                          brightness, sizeof(*brightness));
  if (ret <= 0)
    {
      if (ret == 0)
        {
          ret = -ENODATA;
        }

      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: mipi_dsi_device_register
 *
 * Description:
 *   Register mipi dsi device, if defined CONFIG_MIPI_DSI_DRIVER, will create
 *   character device at /dev/dsiN/dev-xxx.
 *
 * Input Parameters:
 *   host - An instance of the dsi host
 *   name - The name of the dsi device
 *   channel - The channel used by dsi device
 *
 * Returned Value:
 *   struct mipi_dsi_device* if the driver was successfully register; NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

FAR struct mipi_dsi_device *
mipi_dsi_device_register(FAR struct mipi_dsi_host *host,
                         FAR const char *name, int channel)
{
  FAR struct mipi_dsi_device *dev;
#ifdef CONFIG_MIPI_DSI_DRIVER
  int ret;
#endif

  DEBUGASSERT(host != NULL && name != NULL);

  if (channel > 3)
    {
      verr("invalid virtual channel: %u\n", channel);
      return NULL;
    }

  dev = kmm_zalloc(sizeof(struct mipi_dsi_device));
  if (dev)
    {
      dev->host = host;
      dev->channel = channel;
      snprintf(dev->name, sizeof(dev->name), "%s", name);
#ifdef CONFIG_MIPI_DSI_DRIVER
      ret = mipi_dsi_device_driver_register(dev);
      if (ret < 0)
        {
          kmm_free(dev);
          dev = NULL;
        }
#endif // CONFIG_MIPI_DSI_DRIVER
    }

  return dev;
}
