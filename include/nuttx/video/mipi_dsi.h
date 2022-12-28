/****************************************************************************
 * include/nuttx/video/mipi_dsi.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_MIPI_DSI_H
#define __INCLUDE_NUTTX_VIDEO_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

#include <stdbool.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define cmd for dsi device */

#define MIPIDSI_GETDEVLANES     _MIPIDSIIOC(1)
#define MIPIDSI_GETDEVFMT       _MIPIDSIIOC(2)
#define MIPIDSI_GETDEVMODE      _MIPIDSIIOC(3)
#define MIPIDSI_GETDEVHSRATE    _MIPIDSIIOC(4)
#define MIPIDSI_GETDEVLPRATE    _MIPIDSIIOC(5)

/* Define cmd for dsi host */

#define MIPIDSI_TRANSFER        _MIPIDSIIOC(6)

/* Define msg flags */

#define MIPI_DSI_MSG_REQ_ACK      (1 << 0)   /* Request ACK from peripheral */
#define MIPI_DSI_MSG_USE_LPM      (1 << 1)   /* Use Low Power Mode to
                                                * transmit message */
#define MIPI_DSI_MSG_AFTER_FRAME  (1 << 2)   /* Transmit message after frame */

/* Tearing Effect Output Line mode */

#define MIPI_DSI_DCS_TEAR_MODE_VBLANK  0  /* The TE output line consists of
                                           * V-Blanking information only */
#define MIPI_DSI_DCS_TEAR_MODE_VHBLANK 1  /* The TE output line consists of
                                           * both V-Blanking and H-Blanking
                                           * information */

/* Define pixel color format */

#define MIPI_DSI_FMT_RGB888         0
#define MIPI_DSI_FMT_RGB666         1
#define MIPI_DSI_FMT_RGB666_PACKED  2
#define MIPI_DSI_FMT_RGB565         3

/* Indicates the status of register 0Ah */

#define MIPI_DSI_DCS_POWER_MODE_DISPLAY (1 << 2)
#define MIPI_DSI_DCS_POWER_MODE_NORMAL  (1 << 3)
#define MIPI_DSI_DCS_POWER_MODE_SLEEP   (1 << 4)
#define MIPI_DSI_DCS_POWER_MODE_PARTIAL (1 << 5)
#define MIPI_DSI_DCS_POWER_MODE_IDLE    (1 << 6)

/* DSI mode flags define */

#define MIPI_DSI_MODE_VIDEO            (1 << 0)  /* Video mode */
#define MIPI_DSI_MODE_VIDEO_BURST      (1 << 1)  /* Video burst mode */
#define MIPI_DSI_MODE_VIDEO_SYNC_PULSE (1 << 2)  /* Video pulse mode */
#define MIPI_DSI_MODE_VIDEO_AUTO_VERT  (1 << 3)  /* Enable auto vertical
                                                  * count mode */
#define MIPI_DSI_MODE_VIDEO_HSE        (1 << 4)  /* Enable hsync-end packets
                                                  * in vsync-pulse and
                                                  * v-porch area */
#define MIPI_DSI_MODE_VIDEO_NO_HFP     (1 << 5)  /* Disable hfront-porch area */
#define MIPI_DSI_MODE_VIDEO_NO_HBP     (1 << 6)  /* Disable hback-porch area */
#define MIPI_DSI_MODE_VIDEO_NO_HSA     (1 << 7)  /* Disable hsync-active area */
#define MIPI_DSI_MODE_VSYNC_FLUSH      (1 << 8)  /* Flush display FIFO on vsync
                                                  * pulse */
#define MIPI_DSI_MODE_NO_EOT_PACKET    (1 << 9)  /* Disable EoT packets in HS
                                                  * mode */
#define MIPI_DSI_CLOCK_NON_CONTINUOUS  (1 << 10) /* Device supports
                                                  * non-continuous clock
                                                  * behavior (DSI spec 5.6.1) */
#define MIPI_DSI_MODE_LPM              (1 << 11) /* Transmit data in low
                                                  * power */
#define MIPI_DSI_MODE_AFTER_FRAME      (1 << 12) /* Transmit data after frame
                                                  * transfer done */
#define MIPI_DSI_HS_PKT_END_ALIGNED    (1 << 13) /* Transmit data ending at
                                                  * the same time for all
                                                  * lanes within one hsync */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mipi_dsi_host;
struct mipi_dsi_device;

/* Read/write DSI buffer */

struct mipi_dsi_msg
{
  uint8_t channel;          /* Virtual channel id */
  uint8_t type;             /* Payload data type */
  uint8_t flags;            /* Flags controlling this message transmission */

  size_t tx_len;            /* Length of tx_buf */
  FAR const void *tx_buf;   /* Data to be written */

  size_t rx_len;            /* Lenght of rx_buf */
  FAR void *rx_buf;         /* Data to be read, or NULL */
};

/* Represents a MIPI DSI packet in protocol format */

struct mipi_dsi_packet
{
  size_t size;                 /* Size (in bytes) of the packet */
  uint8_t header[4];           /* The four bytes that make up the header
                                * (Data ID, Word Count or Packet Data,
                                * and ECC) */
  size_t payload_length;       /* Number of bytes in the payload */
  FAR const uint8_t *payload;  /* A pointer to a buffer containing
                                * the payload */
};

/* DSI bus operations
 *
 * DSI packets transmitted by .transfer() are passed in as mipi_dsi_msg
 * structures. This structure contains information about the type of packet
 * being transmitted as well as the transmit and receive buffers. When an
 * error is encountered during transmission, this function will return a
 * negative error code. On success it shall return the number of bytes
 * transmitted for write packets or the number of bytes received for read
 * packets.
 *
 * Note that typically DSI packet transmission is atomic, so the .transfer()
 * function will seldomly return anything other than the number of bytes
 * contained in the transmit buffer on success.
 *
 * Also note that those callbacks can be called no matter the state the
 * host is in. Drivers that need the underlying device to be powered to
 * perform these operations will first need to make sure it's been
 * properly enabled.
 */

struct mipi_dsi_host_ops
{
  /* Attach DSI device to DSI host */

  CODE int (*attach)(FAR struct mipi_dsi_host *host,
                     FAR struct mipi_dsi_device *device);

  /* Detach DSI device from DSI host */

  CODE int (*detach)(FAR struct mipi_dsi_host *host,
                     FAR struct mipi_dsi_device *device);

  /* Transmit a DSI packet */

  CODE ssize_t (*transfer)(FAR struct mipi_dsi_host *host,
                           FAR const struct mipi_dsi_msg *msg);
};

/* Dsi host structure */

struct mipi_dsi_host
{
  int bus;
  FAR const struct mipi_dsi_host_ops *ops;
};

/* Dsi peripheral device structure panel dsi device */

struct mipi_dsi_device
{
  FAR struct mipi_dsi_host *host;     /* DSI host for this peripheral,
                                       * almost on the panel */
  char name[32];                      /* DSI peripheral chip type */
  uint16_t channel;                   /* Vitural channel assigned to this */
  uint16_t lanes;                     /* Number of active data lanes */
  uint8_t  format;                    /* Pixel formal */
  uint32_t mode_flags;                /* Dsi operate mode flag */
  uint32_t hs_rate;                   /* Maximum lane frequency for high speed
                                       * mode in hertz, this shoud be set
                                       * to the real limits of the hardware. */
  uint32_t lp_rate;                   /* Maximum lane frequency for low power
                                       * mode in hertz,  this shoud be set
                                       * to the real limits of the hardware. */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

bool mipi_dsi_packet_format_is_short(uint8_t type);

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

bool mipi_dsi_packet_format_is_long(uint8_t type);

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
                           FAR const struct mipi_dsi_msg *msg);

/****************************************************************************
 * Name: mipi_dsi_host_register
 *
 * Description:
 *   Register mipi dsi host, if defined CONFIG_MIPI_DSI_DRIVER, will create
 *   character device at /dev.
 *
 * Input Parameters:
 *   host - An instance of the dsi host
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int mipi_dsi_host_register(FAR struct mipi_dsi_host *host);

/****************************************************************************
 * Name: mipi_dsi_host_get
 *
 * Description:
 *   Find host in list by bus number. Lcd driver can get host by this
 *   interface to register dsi device.
 *
 * Input Parameters:
 *   bus - The dsi host bus number.
 *
 * Returned Value:
 *   struct mipi_dsi_host pointer if the host was successfully registered;
 *   NULL pointer is returned on any failure.
 *
 ****************************************************************************/

FAR struct mipi_dsi_host *mipi_dsi_host_get(int bus);

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

int mipi_dsi_pixel_format_to_bpp(uint8_t fmt);

/****************************************************************************
 * Name: mipi_dsi_device_register_full
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
                         FAR const char *name, int channel);

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

int mipi_dsi_attach(FAR struct mipi_dsi_device *device);

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

int mipi_dsi_detach(FAR struct mipi_dsi_device *device);

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
                          FAR struct mipi_dsi_msg *msg);

/****************************************************************************
 * Name: mipi_dsi_shutdown_peripheral
 *
 * Description:
 *   Send a Shutdown Peripheral command to the dsi device
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_shutdown_peripheral(FAR struct mipi_dsi_device *device);

/****************************************************************************
 * Name: mipi_dsi_turn_on_peripheral
 *
 * Description:
 *   Send a Turn On Peripheral command to the dsi device
 *
 * Input Parameters:
 *   device - DSI peripheral device
 *
 * Returned Value:
 *   OK on success or a negative error code on failure.
 *
 ****************************************************************************/

int mipi_dsi_turn_on_peripheral(FAR struct mipi_dsi_device *device);

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
                        FAR struct mipi_dsi_device *device, uint16_t value);

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
                              bool enable);

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
                           FAR const void *payload, size_t size);

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
                              FAR const void *params, size_t num_params,
                              FAR void *data, size_t size);

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
                                  FAR const void *data, size_t len);

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

ssize_t mipi_dsi_dcs_write(FAR struct mipi_dsi_device *device, uint8_t cmd,
                           FAR const void *data, size_t len);

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

ssize_t mipi_dsi_dcs_read(FAR struct mipi_dsi_device *device, uint8_t cmd,
                          FAR void *data, size_t len);

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

int mipi_dsi_dcs_nop(FAR struct mipi_dsi_device *device);

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

int mipi_dsi_dcs_soft_reset(FAR struct mipi_dsi_device *device);

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
                                FAR uint8_t *mode);

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
                                  FAR uint8_t *format);

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

int mipi_dsi_dcs_enter_sleep_mode(FAR struct mipi_dsi_device *device);

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

int mipi_dsi_dcs_exit_sleep_mode(FAR struct mipi_dsi_device *device);

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

int mipi_dsi_dcs_set_display_off(FAR struct mipi_dsi_device *device);

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

int mipi_dsi_dcs_set_display_on(FAR struct mipi_dsi_device *device);

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
                                    uint16_t start, uint16_t end);

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
                                  uint16_t start, uint16_t end);

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

int mipi_dsi_dcs_set_tear_off(FAR struct mipi_dsi_device *device);

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
                             uint8_t mode);

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
                                  uint8_t format);

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
                                   uint16_t scanline);

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
                                        uint16_t brightness);

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
                                        FAR uint16_t *brightness);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_VIDEO_MIPI_DSI_H */
