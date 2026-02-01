/****************************************************************************
 * drivers/lcd/st7796.c
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
 * http://www.apache.org/licenses/LICENSE-2.0
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/lcd/st7796.h>

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be enabled for ST7796"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7796 Commands - Standard */

#define ST7796_NOP              0x00  /* NOP */
#define ST7796_SWRESET          0x01  /* Software Reset */
#define ST7796_RDDID            0x04  /* Read Display ID */
#define ST7796_RDDST            0x09  /* Read Display Status */
#define ST7796_SLPIN            0x10  /* Sleep In */
#define ST7796_SLPOUT           0x11  /* Sleep Out */
#define ST7796_PTLON            0x12  /* Partial Display Mode On */
#define ST7796_NORON            0x13  /* Normal Display Mode On */
#define ST7796_RDIMGFMT         0x0a  /* Read Display Image Format */
#define ST7796_RDSELFDIAG       0x0f  /* Read Display Self-Diagnostic Result */
#define ST7796_INVOFF           0x20  /* Display Inversion Off */
#define ST7796_INVON            0x21  /* Display Inversion On */
#define ST7796_GAMMASET         0x26  /* Gamma Set */
#define ST7796_DISPOFF          0x28  /* Display Off */
#define ST7796_DISPON           0x29  /* Display On */
#define ST7796_CASET            0x2a  /* Column Address Set */
#define ST7796_RASET            0x2b  /* Row Address Set */
#define ST7796_RAMWR            0x2c  /* Memory Write */
#define ST7796_RAMRD            0x2e  /* Memory Read */
#define ST7796_PTLAR            0x30  /* Partial Area */
#define ST7796_VSCRDEF          0x33  /* Vertical Scrolling Definition */
#define ST7796_TEOFF            0x34  /* Tearing Effect Line Off */
#define ST7796_TEON             0x35  /* Tearing Effect Line On */
#define ST7796_MADCTL           0x36  /* Memory Access Control */
#define ST7796_VSCRSADD         0x37  /* Vertical Scrolling Start Address */
#define ST7796_PIXFMT           0x3a  /* Pixel Format Set */
#define ST7796_WRDISPBRIGHT     0x51  /* Write Display Brightness */
#define ST7796_RDDISPBRIGHT     0x52  /* Read Display Brightness */
#define ST7796_WRCTRLD          0x53  /* Write Control Display */
#define ST7796_RDCTRLD          0x54  /* Read Control Display */
#define ST7796_WRCABC           0x55  /* Write Content Adaptive Brightness */
#define ST7796_RDCABC           0x56  /* Read Content Adaptive Brightness */
#define ST7796_WRCABCMIN        0x5e  /* Write CABC Minimum Brightness */
#define ST7796_RDCABCMIN        0x5f  /* Read CABC Minimum Brightness */

/* ST7796 Commands - Extended */

#define ST7796_INVCTR           0xb4  /* Display Inversion Control */
#define ST7796_DFC              0xb6  /* Display Function Control */
#define ST7796_PWCTRL1          0xc0  /* Power Control 1 */
#define ST7796_PWCTRL2          0xc1  /* Power Control 2 */
#define ST7796_PWCTRL3          0xc2  /* Power Control 3 */
#define ST7796_PWCTRL4          0xc3  /* Power Control 4 */
#define ST7796_PWCTRL5          0xc4  /* Power Control 5 */
#define ST7796_VCOM             0xc5  /* VCOM Control */
#define ST7796_PWCTRL6          0xc6  /* Power Control 6 */
#define ST7796_GAMMAPOS         0xe0  /* Positive Gamma Correction */
#define ST7796_GAMMANEG         0xe1  /* Negative Gamma Correction */
#define ST7796_DOCA             0xe9  /* Set DDB Write Address */
#define ST7796_CSCON            0xf0  /* Command Set Control */

/* ST7796 MADCTL bits */

#define ST7796_MADCTL_MY        0x80  /* Row Address Order */
#define ST7796_MADCTL_MX        0x40  /* Column Address Order */
#define ST7796_MADCTL_MV        0x20  /* Row/Column Exchange */
#define ST7796_MADCTL_ML        0x10  /* Vertical Refresh Order */
#define ST7796_MADCTL_BGR       0x08  /* BGR color filter panel */
#define ST7796_MADCTL_MH        0x04  /* Horizontal Refresh Order */

/* Force SPI MODE 0 (CPOL=0, CPHA=0) - Standard for ST7796 */

#define ST7796_SPIMODE SPIDEV_MODE0

/* Bytes per pixel based on BPP */

#define ST7796_BYTESPP(bpp)     (((bpp) == 18) ? 3 : 2)

/* Color format based on BPP */

#define ST7796_COLORFMT(bpp)    (((bpp) == 18) ? FB_FMT_RGB24 : \
                                 FB_FMT_RGB16_565)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Command sequence entry for initialization */

struct st7796_cmd_s
{
  uint8_t cmd;              /* Command byte */
  FAR const uint8_t *data;  /* Parameter data (NULL if no params) */
  uint8_t len;              /* Number of parameter bytes */
  uint16_t delay_ms;        /* Delay after command in milliseconds */
};

struct st7796_dev_s
{
  struct fb_vtable_s vtable;
  FAR struct spi_dev_s *spi;
  FAR uint8_t *fbmem;
  FAR uint16_t *swap_buf;             /* Persistent buffer for byte-swapping */
  struct st7796_config_s config;      /* Board-provided configuration */
  uint16_t rotation;                  /* Current rotation in degrees */
  bool power;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int st7796_getvideoinfo(FAR struct fb_vtable_s *vtable,
                               FAR struct fb_videoinfo_s *vinfo);
static int st7796_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                               FAR struct fb_planeinfo_s *pinfo);
static int st7796_updatearea(FAR struct fb_vtable_s *vtable,
                             FAR const struct fb_area_s *area);
static int st7796_ioctl(FAR struct fb_vtable_s *vtable, int cmd,
                        unsigned long arg);
static void st7796_select(FAR struct st7796_dev_s *priv);
static void st7796_deselect(FAR struct st7796_dev_s *priv);
static void st7796_sendcmd(FAR struct st7796_dev_s *priv, uint8_t cmd);
static void st7796_senddata(FAR struct st7796_dev_s *priv,
                            FAR const uint8_t *data, size_t len);
static void st7796_send_sequence(FAR struct st7796_dev_s *priv,
                                 FAR const struct st7796_cmd_s *seq,
                                 size_t count);
static void st7796_setarea(FAR struct st7796_dev_s *priv,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1);
static void st7796_apply_rotation(FAR struct st7796_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct st7796_dev_s g_st7796dev;

/* Initialization sequence data arrays
 *
 * ST7796S Register Configuration Reference:
 *
 * CSCON (Command Set Control):
 *   0xc3, 0x96  - Enable extended command access (unlock)
 *   0x3c, 0x69  - Disable extended command access (lock)
 *
 * PIXFMT (Pixel Format):
 *   0x55 - RGB565 16bpp (0101 0101b: 16-bit RGB + 16-bit MCU)
 *
 * INVCTR (Display Inversion):
 *   0x01 - Column inversion mode
 *
 * DFC (Display Function Control):
 *   0x80 - Source output scan direction
 *   0x02 - Gate output scan direction
 *   0x3b - Number of lines (59 * 8 = 472 lines)
 *
 * CMD 0xE8 (Undocumented):
 *   Internal timing adjustment, from manufacturer reference
 *
 * PWCTRL2/3 (Power Control):
 *   0x06 - VGH/VGL voltage setting
 *   0xa7 - VCOM voltage adjustment
 *
 * VCOM:
 *   0x18 - VCOM voltage for contrast tuning
 *
 * GAMMAPOS/GAMMANEG:
 *   14-byte gamma correction curves for color linearity
 *
 * MADCTL values provided by board configuration
 */

static const uint8_t g_cscon1_data[] =
{
  0xc3
};

static const uint8_t g_cscon2_data[] =
{
  0x96
};

static const uint8_t g_pixfmt_data[] =
{
  0x55
};

static const uint8_t g_invctr_data[] =
{
  0x01
};

static const uint8_t g_dfc_data[] =
{
  0x80, 0x02, 0x3b
};

static const uint8_t g_cmd_e8_data[] =
{
  0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33
};

static const uint8_t g_pwctrl2_data[] =
{
  0x06
};

static const uint8_t g_pwctrl3_data[] =
{
  0xa7
};

static const uint8_t g_vcom_data[] =
{
  0x18
};

static const uint8_t g_gammapos_data[] =
{
  0xf0, 0x09, 0x0b, 0x06, 0x04, 0x15,
  0x2f, 0x54, 0x42, 0x3c, 0x17, 0x14,
  0x18, 0x1b
};

static const uint8_t g_gammaneg_data[] =
{
  0xf0, 0x09, 0x0b, 0x06, 0x04, 0x03,
  0x2d, 0x43, 0x42, 0x3b, 0x16, 0x14,
  0x17, 0x1b
};

static const uint8_t g_cscon3_data[] =
{
  0x3c
};

static const uint8_t g_cscon4_data[] =
{
  0x69
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7796_select
 *
 * Description:
 *   Select the SPI device, locking the bus and configuring SPI parameters.
 *   This function locks the SPI bus, sets the SPI mode, bit width, and
 *   frequency, then asserts the chip select signal.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_select(FAR struct st7796_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, ST7796_SPIMODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, priv->config.frequency);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: st7796_deselect
 *
 * Description:
 *   Deselect the SPI device, releasing the chip select and unlocking the
 *   SPI bus for other devices.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_deselect(FAR struct st7796_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: st7796_sendcmd
 *
 * Description:
 *   Send a command byte to the ST7796 display controller. The D/C (Data/
 *   Command) line is set to command mode before transmission.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *   cmd  - Command byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_sendcmd(FAR struct st7796_dev_s *priv, uint8_t cmd)
{
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(priv->spi, cmd);
}

/****************************************************************************
 * Name: st7796_senddata
 *
 * Description:
 *   Send data bytes to the ST7796 display controller. The D/C (Data/
 *   Command) line is set to data mode before transmission.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *   data - Pointer to the data buffer to send
 *   len  - Number of bytes to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_senddata(FAR struct st7796_dev_s *priv,
                            FAR const uint8_t *data, size_t len)
{
  if (len > 0 && data != NULL)
    {
      SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
      SPI_SNDBLOCK(priv->spi, data, len);
    }
}

/****************************************************************************
 * Name: st7796_send_sequence
 *
 * Description:
 *   Send a sequence of commands and data to the ST7796 display controller.
 *   Each entry in the sequence contains a command, optional data, and an
 *   optional delay. This function is typically used for display
 *   initialization.
 *
 * Input Parameters:
 *   priv  - Reference to the ST7796 device structure
 *   seq   - Pointer to an array of command/data sequence entries
 *   count - Number of entries in the sequence array
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_send_sequence(FAR struct st7796_dev_s *priv,
                                 FAR const struct st7796_cmd_s *seq,
                                 size_t count)
{
  size_t i;

  lcdinfo("ST7796: Sending initialization sequence (%zu commands)\n", count);

  for (i = 0; i < count; i++)
    {
      st7796_sendcmd(priv, seq[i].cmd);

      if (seq[i].data != NULL && seq[i].len > 0)
        {
          st7796_senddata(priv, seq[i].data, seq[i].len);
        }

      if (seq[i].delay_ms > 0)
        {
          nxsig_usleep(seq[i].delay_ms * 1000);
        }
    }

  lcdinfo("ST7796: Initialization sequence complete\n");
}

/****************************************************************************
 * Name: st7796_setarea
 *
 * Description:
 *   Set the active drawing area on the ST7796 display. This defines the
 *   rectangular region where subsequent pixel data will be written. The
 *   function sends CASET (Column Address Set) and RASET (Row Address Set)
 *   commands to define the window boundaries.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *   x0   - Start column (left edge)
 *   y0   - Start row (top edge)
 *   x1   - End column (right edge, inclusive)
 *   y1   - End row (bottom edge, inclusive)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_setarea(FAR struct st7796_dev_s *priv,
                           uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
  uint8_t data[4];

  st7796_sendcmd(priv, ST7796_CASET);
  data[0] = (x0 >> 8) & 0xff;
  data[1] = x0 & 0xff;
  data[2] = (x1 >> 8) & 0xff;
  data[3] = x1 & 0xff;
  st7796_senddata(priv, data, 4);

  st7796_sendcmd(priv, ST7796_RASET);
  data[0] = (y0 >> 8) & 0xff;
  data[1] = y0 & 0xff;
  data[2] = (y1 >> 8) & 0xff;
  data[3] = y1 & 0xff;
  st7796_senddata(priv, data, 4);
}

/****************************************************************************
 * Name: st7796_apply_rotation
 *
 * Description:
 *   Apply rotation by modifying the MADCTL register. This function
 *   calculates the appropriate MADCTL value based on the base orientation
 *   and the requested rotation angle, then sends it to the display.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void st7796_apply_rotation(FAR struct st7796_dev_s *priv)
{
  uint8_t madctl;
  uint8_t data[1];

  /* Start with base MADCTL from config */

  madctl = priv->config.madctl;

  /* Apply rotation by modifying MX/MY bits */

  switch (priv->rotation)
    {
      case 0:

      /* No rotation - use base MADCTL as-is */

        break;

      case 90:

      /* 90 degrees: swap MV and toggle MX */

        madctl ^= (ST7796_MADCTL_MV | ST7796_MADCTL_MX);
        break;

      case 180:

      /* 180 degrees: toggle both MX and MY */

        madctl ^= (ST7796_MADCTL_MX | ST7796_MADCTL_MY);
        break;

      case 270:

      /* 270 degrees: swap MV and toggle MY */

        madctl ^= (ST7796_MADCTL_MV | ST7796_MADCTL_MY);
        break;

      default:
        lcdwarn("ST7796: Invalid rotation %d, using 0\n", priv->rotation);
        priv->rotation = 0;
        break;
    }

  /* Send MADCTL command */

  data[0] = madctl;

  st7796_select(priv);
  st7796_sendcmd(priv, ST7796_MADCTL);
  st7796_senddata(priv, data, 1);
  st7796_deselect(priv);

  lcdinfo("ST7796: Applied rotation %d (MADCTL=0x%02x)\n",
          priv->rotation, madctl);
}

/****************************************************************************
 * Name: st7796_getvideoinfo
 *
 * Description:
 *   Get information about the video controller and the configuration of
 *   the video plane. This is part of the framebuffer interface required
 *   by NuttX.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *   vinfo  - Pointer to the video info structure to be filled
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int st7796_getvideoinfo(FAR struct fb_vtable_s *vtable,
                               FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct st7796_dev_s *priv = (FAR struct st7796_dev_s *)vtable;

  lcdinfo("ST7796: getvideoinfo\n");

  vinfo->fmt     = ST7796_COLORFMT(priv->config.bpp);
  vinfo->xres    = priv->config.xres;
  vinfo->yres    = priv->config.yres;
  vinfo->nplanes = 1;

  return OK;
}

/****************************************************************************
 * Name: st7796_getplaneinfo
 *
 * Description:
 *   Get information about the framebuffer plane. Returns details about
 *   the framebuffer memory, stride, bits per pixel, and virtual
 *   resolution. This is part of the framebuffer interface required by
 *   NuttX.
 *
 * Input Parameters:
 *   vtable  - Reference to the framebuffer virtual table
 *   planeno - Plane number (must be 0 for this single-plane display)
 *   pinfo   - Pointer to the plane info structure to be filled
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int st7796_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                               FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct st7796_dev_s *priv = (FAR struct st7796_dev_s *)vtable;
  size_t fbsize;
  uint8_t bytespp;

  lcdinfo("ST7796: getplaneinfo - plane %d\n", planeno);

  bytespp = ST7796_BYTESPP(priv->config.bpp);
  fbsize = priv->config.xres * priv->config.yres * bytespp;

  pinfo->fbmem        = priv->fbmem;
  pinfo->fblen        = fbsize;
  pinfo->stride       = priv->config.xres * bytespp;
  pinfo->bpp          = priv->config.bpp;
  pinfo->xres_virtual = priv->config.xres;
  pinfo->yres_virtual = priv->config.yres;
  pinfo->xoffset      = 0;
  pinfo->yoffset      = 0;

  return OK;
}

/****************************************************************************
 * Name: st7796_updatearea
 *
 * Description:
 *   Update a rectangular area of the display from the framebuffer. This
 *   function transfers pixel data from the in-memory framebuffer to the
 *   ST7796 display controller via SPI. The pixel data is byte-swapped
 *   from little-endian (CPU native) to big-endian (display native) format
 *   during transfer.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *   area   - Pointer to structure describing the rectangular area to
 *            update (x, y, width, height)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int st7796_updatearea(FAR struct fb_vtable_s *vtable,
                             FAR const struct fb_area_s *area)
{
  FAR struct st7796_dev_s *priv = (FAR struct st7796_dev_s *)vtable;
  FAR uint16_t *src_fbptr;
  size_t row_size_pixels;
  uint8_t bytespp;
  int row;
  int i;

  lcdinfo("ST7796: updatearea - x=%d y=%d w=%d h=%d\n",
          area->x, area->y, area->w, area->h);

  bytespp = ST7796_BYTESPP(priv->config.bpp);

  st7796_select(priv);
  st7796_setarea(priv, area->x, area->y,
                 area->x + area->w - 1,
                 area->y + area->h - 1);

  st7796_sendcmd(priv, ST7796_RAMWR);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  row_size_pixels = area->w;
  src_fbptr = (FAR uint16_t *)
              (priv->fbmem +
               (area->y * priv->config.xres + area->x) * bytespp);

  for (row = 0; row < area->h; row++)
    {
      for (i = 0; i < row_size_pixels; i++)
        {
          uint16_t pixel = src_fbptr[i];
          priv->swap_buf[i] = (pixel << 8) | (pixel >> 8);
        }

      SPI_SNDBLOCK(priv->spi, (FAR const uint8_t *)priv->swap_buf,
                   row_size_pixels * bytespp);

      src_fbptr += priv->config.xres;
    }

  st7796_deselect(priv);

  return OK;
}

/****************************************************************************
 * Name: st7796_ioctl
 *
 * Description:
 *   Handle framebuffer ioctl commands. Supports rotation control via
 *   FBIOSET_ROTATION and FBIOGET_ROTATION commands.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *   cmd    - ioctl command
 *   arg    - ioctl argument
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int st7796_ioctl(FAR struct fb_vtable_s *vtable, int cmd,
                        unsigned long arg)
{
  FAR struct st7796_dev_s *priv = (FAR struct st7796_dev_s *)vtable;
  int ret = OK;

  switch (cmd)
    {
      case FBIOSET_ROTATION:
        {
          uint16_t rotation = (uint16_t)arg;

          /* Validate rotation value */

          if (rotation != 0 && rotation != 90 &&
              rotation != 180 && rotation != 270)
            {
              lcderr("ERROR: Invalid rotation %d\n", rotation);
              ret = -EINVAL;
              break;
            }

          priv->rotation = rotation;
          st7796_apply_rotation(priv);
        }
        break;

      case FBIOGET_ROTATION:
        {
          FAR uint16_t *rotation = (FAR uint16_t *)arg;

          if (rotation == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *rotation = priv->rotation;
        }
        break;

      case FBIO_UPDATE:
        {
          FAR struct fb_area_s *area = (FAR struct fb_area_s *)arg;
          ret = st7796_updatearea(vtable, area);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: st7796_init_display
 *
 * Description:
 *   Initialize the ST7796 display hardware with the provided configuration.
 *   This sends the initialization sequence and configures MADCTL based on
 *   board-provided settings.
 *
 * Input Parameters:
 *   priv - Reference to the ST7796 device structure
 *
 * Returned Value:
 *   None
 *
 * Timing requirements from ST7796S datasheet section 9.16:
 *
 *   SWRESET: 120ms minimum recovery time (using 150ms for margin)
 *   SLPOUT:  120ms minimum for DC-DC and oscillator stabilization
 *   INVON:   No minimum specified, 10ms for safety
 *   NORON:   No minimum specified, 10ms for safety
 *   DISPON:  Frame memory must be written first, 120ms for stabilization
 *
 * Sequence follows recommended power-on flow:
 *   1. Software reset
 *   2. Exit sleep mode
 *   3. Unlock extended registers (CSCON)
 *   4. Configure display parameters (including board-provided MADCTL)
 *   5. Lock extended registers (CSCON)
 *   6. Enable inversion and normal mode
 *   7. Turn on display
 *
 ****************************************************************************/

static void st7796_init_display(FAR struct st7796_dev_s *priv)
{
  uint8_t madctl_data[1];

  /* MADCTL value from board configuration */

  madctl_data[0] = priv->config.madctl;

  /* Initialization sequence - Part 1: Reset and wake */

  static const struct st7796_cmd_s init_seq_part1[] =
  {
    { ST7796_SWRESET, NULL,           0, 150 },
    { ST7796_SLPOUT,  NULL,           0, 150 },
    { ST7796_CSCON,   g_cscon1_data,  1, 0   },
    { ST7796_CSCON,   g_cscon2_data,  1, 0   },
  };

  /* Initialization sequence - Part 2: Configuration (after MADCTL) */

  static const struct st7796_cmd_s init_seq_part2[] =
  {
    { ST7796_PIXFMT,   g_pixfmt_data,   1,  0   },
    { ST7796_INVCTR,   g_invctr_data,   1,  0   },
    { ST7796_DFC,      g_dfc_data,      3,  0   },
    { 0xe8,            g_cmd_e8_data,   8,  0   },
    { ST7796_PWCTRL2,  g_pwctrl2_data,  1,  0   },
    { ST7796_PWCTRL3,  g_pwctrl3_data,  1,  0   },
    { ST7796_VCOM,     g_vcom_data,     1,  0   },
    { ST7796_GAMMAPOS, g_gammapos_data, 14, 0   },
    { ST7796_GAMMANEG, g_gammaneg_data, 14, 0   },
    { ST7796_CSCON,    g_cscon3_data,   1,  0   },
    { ST7796_CSCON,    g_cscon4_data,   1,  0   },
    { ST7796_INVON,    NULL,            0,  10  },
    { ST7796_NORON,    NULL,            0,  10  },
    { ST7796_DISPON,   NULL,            0,  120 },
  };

  st7796_select(priv);

  /* Send part 1: Reset and unlock */

  st7796_send_sequence(priv, init_seq_part1,
                       sizeof(init_seq_part1) / sizeof(struct st7796_cmd_s));

  /* Send MADCTL with board-provided value */

  st7796_sendcmd(priv, ST7796_MADCTL);
  st7796_senddata(priv, madctl_data, 1);

  /* Send part 2: Configuration and enable */

  st7796_send_sequence(priv, init_seq_part2,
                       sizeof(init_seq_part2) / sizeof(struct st7796_cmd_s));

  st7796_deselect(priv);

  lcdinfo("ST7796: Display initialized (MADCTL=0x%02x)\n",
          priv->config.madctl);

  /* Apply initial rotation if configured */

  if (priv->rotation != 0)
    {
      st7796_apply_rotation(priv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st7796_fbinitialize
 *
 * Description:
 *   Initialize the ST7796 LCD controller and framebuffer driver. This
 *   function allocates the framebuffer memory, initializes the driver
 *   structure, sends the display initialization sequence, and returns
 *   a pointer to the framebuffer virtual table for use with the NuttX
 *   framebuffer interface.
 *
 * Input Parameters:
 *   spi    - Reference to the SPI driver structure to use for communication
 *   config - Board-specific configuration (frequency, resolution, etc.)
 *
 * Returned Value:
 *   On success, a pointer to the framebuffer virtual table is returned.
 *   On failure, NULL is returned.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *st7796_fbinitialize(FAR struct spi_dev_s *spi,
                                            FAR const struct st7796_config_s
                                            *config)
{
  FAR struct st7796_dev_s *priv = &g_st7796dev;
  size_t fbsize;
  uint8_t bytespp;

  DEBUGASSERT(spi != NULL && config != NULL);

  lcdinfo("ST7796: Initializing framebuffer driver\n");
  lcdinfo("ST7796: Resolution: %dx%d @ %d bpp\n",
          config->xres, config->yres, config->bpp);
  lcdinfo("ST7796: SPI Frequency: %lu Hz\n",
          (unsigned long)config->frequency);
  lcdinfo("ST7796: MADCTL: 0x%02x, Rotation: %d\n",
          config->madctl, config->rotation);

  /* Copy configuration */

  memcpy(&priv->config, config, sizeof(struct st7796_config_s));

  /* Set initial rotation from config */

  priv->rotation = config->rotation;

  /* Calculate sizes */

  bytespp = ST7796_BYTESPP(config->bpp);
  fbsize = config->xres * config->yres * bytespp;

  /* Allocate framebuffer memory */

  priv->fbmem = (FAR uint8_t *)kmm_zalloc(fbsize);
  if (priv->fbmem == NULL)
    {
      lcderr("ERROR: Failed to allocate framebuffer (%zu bytes)\n", fbsize);
      return NULL;
    }

  /* Allocate persistent row swap buffer to avoid malloc in updatearea */

  priv->swap_buf = (FAR uint16_t *)kmm_malloc(config->xres * bytespp);
  if (priv->swap_buf == NULL)
    {
      lcderr("ERROR: Failed to allocate swap buffer\n");
      kmm_free(priv->fbmem);
      return NULL;
    }

  /* Initialize driver structure */

  priv->vtable.getvideoinfo = st7796_getvideoinfo;
  priv->vtable.getplaneinfo = st7796_getplaneinfo;
  priv->vtable.updatearea   = st7796_updatearea;
  priv->vtable.ioctl        = st7796_ioctl;
  priv->spi                 = spi;
  priv->power               = false;

  /* Initialize display hardware */

  st7796_init_display(priv);

  priv->power = true;
  lcdinfo("ST7796: Display ready\n");

  return &priv->vtable;
}

/****************************************************************************
 * Name: st7796_setrotation
 *
 * Description:
 *   Set display rotation at runtime. This function can be called directly
 *   by board code or applications that have access to the vtable pointer.
 *
 * Input Parameters:
 *   vtable   - Reference to the framebuffer virtual table
 *   rotation - Rotation angle in degrees (0, 90, 180, or 270)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int st7796_setrotation(FAR struct fb_vtable_s *vtable, uint16_t rotation)
{
  return st7796_ioctl(vtable, FBIOSET_ROTATION, (unsigned long)rotation);
}

/****************************************************************************
 * Name: st7796_getrotation
 *
 * Description:
 *   Get current display rotation.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *
 * Returned Value:
 *   Current rotation in degrees (0, 90, 180, or 270).
 *
 ****************************************************************************/

uint16_t st7796_getrotation(FAR struct fb_vtable_s *vtable)
{
  FAR struct st7796_dev_s *priv = (FAR struct st7796_dev_s *)vtable;
  return priv->rotation;
}
