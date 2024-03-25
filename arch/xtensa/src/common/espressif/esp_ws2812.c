/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_ws2812.c
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

#include <stdlib.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/leds/ws2812.h>
#include <nuttx/rmt/rmt.h>

#include "hal/rmt_types.h"
#include "soc/soc.h"

#include "esp_rmt.h"

#include "esp_ws2812.h"

#ifdef CONFIG_WS2812

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rmt_item32_t rmt_symbol_word_t

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct rgbw_led_s
{
  union
    {
      struct
        {
          uint8_t b;
          uint8_t g;
          uint8_t r;
          uint8_t w;
        };
      uint32_t val;
    };
};

struct esp_ws2812_dev_s
{
  struct rmt_dev_s    *rmt;
  uint8_t             *buf;
  size_t               buflen;
  size_t               open_count;   /* Number of opens on this instance. */
};

/* RMT channel ID */

enum rmt_channel_e
{
  RMT_CHANNEL_0,  /* RMT channel number 0 */
  RMT_CHANNEL_1,  /* RMT channel number 1 */
  RMT_CHANNEL_2,  /* RMT channel number 2 */
  RMT_CHANNEL_3,  /* RMT channel number 3 */
#if SOC_RMT_CHANNELS_PER_GROUP > 4
  RMT_CHANNEL_4,  /* RMT channel number 4 */
  RMT_CHANNEL_5,  /* RMT channel number 5 */
  RMT_CHANNEL_6,  /* RMT channel number 6 */
  RMT_CHANNEL_7,  /* RMT channel number 7 */
#endif
  RMT_CHANNEL_MAX /* Number of RMT channels */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t map_byte_to_words(struct esp_ws2812_dev_s *dev,
                                  uint8_t byte,
                                  uint32_t *dst);
static int map_leds_to_words(struct esp_ws2812_dev_s *dev,
                             struct rgbw_led_s *leds,
                             uint32_t n_leds,
                             uint32_t *dst,
                             bool has_white);
static int esp_open(struct file *filep);
static int esp_close(struct file *filep);
static int esp_write(struct file *filep, const char *data, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if SOC_RMT_CHANNEL_CLK_INDEPENDENT
extern uint32_t g_rmt_source_clock_hz[RMT_CHANNEL_MAX];
#else
extern uint32_t g_rmt_source_clock_hz;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: map_byte_to_words
 *
 * Description:
 *   Maps a byte to a sequence of RMT items. Each bit in the byte is
 *   represented by an RMT item (32-bit value). The function iterates over
 *   each bit in the byte, creating an RMT item for each bit, HIGH or LOW.
 *   The created RMT items are stored in the destination array.
 *
 * Input Parameters:
 *   dev  - Pointer to the RMT-based WS2812 device structure.
 *   byte - The byte to be mapped.
 *   dst  - Destination array for the RMT items.
 *
 * Returned Value:
 *   Number of RMT items mapped.
 *
 ****************************************************************************/

static uint32_t map_byte_to_words(struct esp_ws2812_dev_s *dev,
                                  uint8_t byte,
                                  uint32_t *dst)
{
  uint32_t mapped;
  uint8_t mask;
  uint16_t t0h;
  uint16_t t0l;
  uint16_t t1h;
  uint16_t t1l;
  uint32_t clock_period_ps;
  uint32_t rmt_period_ps;

#if SOC_RMT_CHANNEL_CLK_INDEPENDENT
  clock_period_ps = 1000000000000 / g_rmt_source_clock_hz[dev->rmt->minor];
#else
  clock_period_ps = 1000000000000 / g_rmt_source_clock_hz;
#endif
  rmt_period_ps = clock_period_ps / RMT_DEFAULT_CLK_DIV;

  /* Calculate the RMT period to encode WS2812 frames */

  t0h = ((uint16_t)(350000 / rmt_period_ps));
  t0l = ((uint16_t)(900000 / rmt_period_ps));
  t1h = ((uint16_t)(900000 / rmt_period_ps));
  t1l = ((uint16_t)(350000 / rmt_period_ps));

  mapped = 0;
  mask = 0x80;
  for (int i = 0; i < 8; i++)
    {
      uint32_t word;
      uint8_t bit = (byte & mask);

      mask >>= 1;

      if (bit)
        {
          word = (t1l << 16) | (0x8000 | t1h);
        }
        else
        {
          word = (t0l << 16) | (0x8000 | t0h);
        }

      *dst = word;
      dst++;
      mapped++;
    }

  return mapped;
}

/****************************************************************************
 * Name: map_leds_to_words
 *
 * Description:
 *   Maps an array of LEDs to a sequence of RMT items. Each LED in the array
 *   is represented by a sequence of RMT items, one for each bit in the RGB
 *   (and optionally white) values. Iterates over each LED in the array,
 *   mapping the RGB (and optionally white) values to RMT items using the
 *   map_byte_to_words function. The RMT items are stored in the destination
 *   array.
 *
 * Input Parameters:
 *   dev       - Pointer to the RMT-based WS2812 device structure.
 *   leds      - Pointer to the array of LEDs.
 *   n_leds    - Number of LEDs in the array.
 *   dst       - Destination array for the RMT items.
 *   has_white - Flag indicating if the LEDs include a white component.
 *
 * Returned Value:
 *   Number of RMT items mapped; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int map_leds_to_words(struct esp_ws2812_dev_s *dev,
                             struct rgbw_led_s *leds,
                             uint32_t n_leds,
                             uint32_t *dst,
                             bool has_white)
{
  uint32_t dst_offset;

  if (!dst || !leds)
    {
      return -EINVAL;
    }

  dst_offset = 0;
  for (uint32_t led_idx = 0; led_idx < n_leds; led_idx++)
    {
      dst_offset += map_byte_to_words(dev,
                                      leds[led_idx].g,
                                      dst + dst_offset);
      dst_offset += map_byte_to_words(dev,
                                      leds[led_idx].r,
                                      dst + dst_offset);
      dst_offset += map_byte_to_words(dev,
                                      leds[led_idx].b,
                                      dst + dst_offset);
      if (has_white)
        {
          dst_offset += map_byte_to_words(dev,
                                          leds[led_idx].w,
                                          dst + dst_offset);
        }
    }

  return dst_offset;
}

/****************************************************************************
 * Name: esp_open
 *
 * Description:
 *   This function opens a WS2812 device instance. It locks the device,
 *   checks if the device has already been initialized, and if not, it
 *   allocates and initializes the pixel buffer. It then increases the open
 *   count and unlocks the device.
 *
 * Input Parameters:
 *   filep - Pointer to the file structure.
 *
 * Returned Value:
 *   Returns OK on successful open of the device; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int esp_open(struct file *filep)
{
  struct inode              *inode     = filep->f_inode;
  struct ws2812_dev_s       *dev_data  = inode->i_private;
  struct esp_ws2812_dev_s   *priv;
  uint8_t                    colors;
  irqstate_t                 flags;
  size_t                     buflen;
  int                        i;
  int                        ret;

  priv = (struct esp_ws2812_dev_s *)dev_data->private;

  flags = enter_critical_section();

  if (priv->buf != NULL)
    {
      /* We've already been initialized.  Keep on truckin' */

      ledinfo("esp_ws2812 re-open dev: 0x%p\n", dev_data);

      ret = OK;
      goto post_and_return;
    }

  ledinfo("esp_ws2812 open dev: 0x%p\n", dev_data);

  /* Allocate the pixel buffer */

  if (priv->open_count == 0)
    {
      struct rgbw_led_s *led;

      /* Number of colors of each LED */

      colors = (dev_data->has_white ? 4 : 3);

      /* Each LED color is represented by 8 RMT items + 1 last item. Each RMT
       * item is 32-bit long.
       */

      buflen = (dev_data->nleds * colors * 8 + 1) * sizeof(rmt_item32_t);

      priv->buf = kmm_zalloc(buflen);

      if (priv->buf == NULL)
        {
          lederr("esp_ws2812 open: out of memory\n");

          ret = -ENOMEM;
          goto post_and_return;
        }

      priv->buflen = buflen;

      /* Clear all LEDs in the LED strip */

      led = kmm_zalloc(sizeof(struct rgbw_led_s));

      if (led == NULL)
        {
          lederr("esp_ws2812 open: out of memory\n");

          ret = -ENOMEM;
          goto post_and_return;
        }

      for (i = 0; i < dev_data->nleds; i++)
        {
          map_leds_to_words(priv,
                            led,
                            1,
                            ((uint32_t *)priv->buf + i * colors * 8),
                            dev_data->has_white);
        }

      kmm_free(led);
    }
  else
    {
      ledwarn("esp_ws2812 open: already open\n");
    }

  priv->open_count += 1;

  ret = OK;

post_and_return:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: esp_close
 *
 * Description:
 *   This function closes a previously opened WS2812 device instance. It
 *   locks the device, decreases the open count, and if no other instances
 *   are open, it frees the buffer associated with the device. It then
 *   unlocks the device and returns OK.
 *
 * Input Parameters:
 *   filep - Pointer to the file structure.
 *
 * Returned Value:
 *   Returns OK on successful close of the device; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

static int esp_close(struct file *filep)
{
  struct inode        *inode    = filep->f_inode;
  struct ws2812_dev_s *dev_data = inode->i_private;
  struct esp_ws2812_dev_s *priv;

  priv = (struct esp_ws2812_dev_s *)dev_data->private;

  nxmutex_lock(&dev_data->lock);

  ledinfo("esp_ws2812 close dev: 0x%p\n", dev_data);

  priv->open_count -= 1;

  if (priv->open_count == 0)
    {
      kmm_free(priv->buf);
      priv->buf = NULL;
    }

  nxmutex_unlock(&dev_data->lock);
  return OK;
}

/****************************************************************************
 * Name: esp_write
 *
 * Description:
 *   This function writes the LED data to the WS2812 device. It checks if the
 *   data and length are valid, locks the device, maps the LED data to the
 *   buffer, updates the file position, writes the buffer to the RMT device,
 *   and unlocks the device. It returns the number of LED pixels that had
 *   their values changed.
 *
 * Input Parameters:
 *   filep - Pointer to the file structure.
 *   data  - Pointer to the LED data to be written.
 *   len   - The length of the data to be written.
 *
 * Returned Value:
 *   Returns the number of LED pixels that had their values changed on
 *   successful write; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t esp_write(struct file *filep, const char *data, size_t len)
{
  struct inode              *inode            = filep->f_inode;
  struct ws2812_dev_s       *dev              = inode->i_private;
  struct esp_ws2812_dev_s   *priv             =
      (struct esp_ws2812_dev_s *)dev->private;
  int                        position         = filep->f_pos;
  uint32_t                   n_leds           = len / WS2812_RW_PIXEL_SIZE;
  uint8_t                    colors           = (dev->has_white ? 4 : 3);
  uint8_t                   *bp               = priv->buf + position;
  int                        rmt_bytes;
  int                        n_leds_written;
  int                        ret;

  /* Check if LED data isn't NULL */

  if (data == NULL)
    {
      lederr("esp_ws2812 write failed: NULL data\n");
      set_errno(EINVAL);
      return 0;
    }

  /* Check if the number of LEDs to be updated is valid */

  if (n_leds > dev->nleds)
    {
      lederr("esp_ws2812 write failed: invalid len for the LEDs buffer\n");
      set_errno(EINVAL);
      return 0;
    }

  nxmutex_lock(&dev->lock);

  if (len > 0)
    {
      /* Check if the lenght to be updated, considering the current position,
       * is valid. The number of LEDs to be updated should, starting from the
       * current offset should be less than the LED strip total length.
       */

      if (((position + len) / WS2812_RW_PIXEL_SIZE) > dev->nleds)
        {
          ledwarn("esp_ws2812 write truncated:\n\t\tLED position: %d\n"
                  "\t\tLED requested to be written: %d\n"
                  "\t\tLED strip LED count: %d\n"
                  "\t\tLED being written: %d\n",
                  position / WS2812_RW_PIXEL_SIZE,
                  n_leds,
                  dev->nleds,
                  dev->nleds - (position / WS2812_RW_PIXEL_SIZE));
          n_leds = dev->nleds - (position / WS2812_RW_PIXEL_SIZE);
        }

      ret = map_leds_to_words(priv,
                              (struct rgbw_led_s *)data,
                              n_leds,
                              (uint32_t *)bp,
                              dev->has_white);
      if (ret < 0)
        {
          lederr("esp_ws2812 write failed: %d\n", ret);
          nxmutex_unlock(&dev->lock);
          set_errno(-ret);
          return ret;
        }

      /* Update the file position: each LED color is represented by 8 RMT
       * items. The position is, then, the number of LEDs to be update times
       * the size of a LED color in bytes.
       */

      position += n_leds * WS2812_RW_PIXEL_SIZE;

      filep->f_pos = position;
    }

  /* Write the buffer to the RMT device */

  rmt_bytes = priv->rmt->ops->write(priv->rmt,
                                    (const char *)priv->buf,
                                    priv->buflen);

  /* n_leds_written is the number of LEDs that had their values changed:
   * Each LED color is represented by 8 RMT items. We also added a last
   * RMT item to the buffer, so we need to subtract 1 from the total number.
   * Finally, we divide by the number of colors to get the number of LEDs.
   */

  n_leds_written = ((rmt_bytes / sizeof(rmt_item32_t)) - 1) / (colors * 8);

  /* Compare n_leds_written with the value representing the full LED strip */

  if (n_leds_written < dev->nleds)
    {
      lederr("esp_ws2812 write failed: %d\n", n_leds_written);
      nxmutex_unlock(&dev->lock);
      set_errno(-EIO);
      return -EIO;
    }

  nxmutex_unlock(&dev->lock);

  /* Return the number of LEDs pixels that had their values changed */

  return n_leds * WS2812_RW_PIXEL_SIZE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ws2812_setup
 *
 * Description:
 *   This function sets up a WS2812 device instance. It allocates memory for
 *   the device structures, initializes the device with the provided
 *   parameters, and registers the device with the system.
 *
 * Input Parameters:
 *   path         - The device path.
 *   rmt          - Pointer to the RMT device structure.
 *   pixel_count  - The number of pixels in the WS2812 strip.
 *   has_white    - Flag indicating if the WS2812 strip includes a white LED.
 *
 * Returned Value:
 *   Returns a pointer to the WS2812 device structure on successful setup;
 *   NULL is returned on any failure, with errno set appropriately.
 *
 ****************************************************************************/

struct ws2812_dev_s *esp_ws2812_setup(const char       *path,
                                      struct rmt_dev_s *rmt,
                                      uint16_t         pixel_count,
                                      bool             has_white)
{
  struct ws2812_dev_s       *dev;
  struct esp_ws2812_dev_s *priv;
  int err;

  /* Allocate struct holding generic WS2812 device data */

  dev = kmm_zalloc(sizeof(struct ws2812_dev_s));

  if (dev == NULL)
    {
      lederr("esp_ws2812 setup: out of memory\n");
      set_errno(ENOMEM);
      return NULL;
    }

  /* Allocate struct holding Espressif's WS2812 (RMT-enabled) device data */

  priv = kmm_zalloc(sizeof(struct esp_ws2812_dev_s));

  if (priv == NULL)
    {
      lederr("esp_ws2812 open: out of memory\n");
      kmm_free(dev);
      set_errno(ENOMEM);
      return NULL;
    }

  priv->rmt = rmt;

  dev->open      = esp_open;
  dev->close     = esp_close;
  dev->write     = esp_write;
  dev->private   = priv;
  dev->clock     = CONFIG_WS2812_FREQUENCY;
  dev->port      = priv->rmt->minor;
  dev->nleds     = pixel_count;
  dev->has_white = has_white;

  nxmutex_init(&dev->lock);

  ledinfo("register dev: 0x%p\n", dev);

  /* Register the WS2812 RGB addressable LED strip device */

  err = ws2812_register(path, dev);

  if (err != OK)
    {
      set_errno(err);
      return NULL;
    }

  return (void *)dev;
}

/****************************************************************************
 * Name: esp_ws2812_release
 *
 * Description:
 *   This function releases a previously opened WS2812 device instance. It
 *   checks if the device is currently open, and if not, it frees the private
 *   data structure and sets the private field of the device to NULL. If the
 *   device is still open, it returns an error.
 *
 * Input Parameters:
 *   driver - Pointer to the instance of the WS2812 device driver to be
 *            released.
 *
 * Returned Value:
 *   Returns OK on successful release of the device; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int esp_ws2812_release(void * driver)
{
  struct ws2812_dev_s *dev = driver;
  struct esp_ws2812_dev_s *priv;
  int ret = OK;

  priv = (struct esp_ws2812_dev_s *)dev->private;

  nxmutex_lock(&dev->lock);

  if (priv->open_count == 0)
    {
      dev->private = NULL;

      nxmutex_unlock(&dev->lock);

      kmm_free(priv);
    }
    else
    {
      ret = -EBUSY;
      nxmutex_unlock(&dev->lock);
    }

  return ret;
}

#endif /* CONFIG_WS2812 */
