/****************************************************************************
 * arch/arm/src/rp2040/rp2040_ws2812.c
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

#include <rp2040_ws2812.h>

#include <stdlib.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/leds/ws2812.h>

#include <rp2040_pio.h>

#ifdef CONFIG_WS2812

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct instance
{
  uint32_t          pio;          /* The pio instance we are using.    */
  uint32_t          pio_location; /* the program location in the pio.  */
  uint32_t          pio_sm;       /* The state machine we are using.   */
  uint8_t          *pixels;       /* Buffer to hold pixels             */
  size_t            open_count;   /* Number of opens on this instance. */
  clock_t           last_dma;     /* when last DMA completed.          */
  int               power_pin;    /* pin for ws2812 power              */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t ws2812_program_instructions[] =
{
  0x6221, /* 0: out  x,  1    side 0 [2]  <-- wrap target */
  0x1123, /* 1: jmp  !x, 3    side 1 [1]                  */
  0x1400, /* 2: jmp  0        side 1 [4]                  */
  0xa442, /* 3: nop           side 0 [4]  --> wrap        */
};

static const struct rp2040_pio_program pio_program =
{
  .instructions = ws2812_program_instructions,
  .length       = 4,
  .origin       = -1,
};

#define ws2812_wrap_target 0
#define ws2812_wrap        3

#define ws2812_T1 2
#define ws2812_T2 5
#define ws2812_T3 3

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dma_complete
 *
 * Description:
 *   Called on completion of the DMA transfer.
 *
 * Input Parameters:
 *   handle - handle to our DMA channel
 *   status - status of the transfer
 *   arg    - Pointer to drivers private data structure.
 *
 ****************************************************************************/

void dma_complete(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct ws2812_dev_s *dev_data = arg;
  struct instance     *priv     = (struct instance *)dev_data->private;

  rp2040_dmafree(handle);

  priv->last_dma = clock_systime_ticks();

  nxsem_post(&dev_data->exclsem);
}

/****************************************************************************
 * Name: update_pixels
 *
 * Description:
 *   This thread manages the actual update of pixels.
 *
 * Input Parameters:
 *   dev_data - Pointer to drivers device data structure.
 *
 ****************************************************************************/

static void update_pixels(struct ws2812_dev_s  *dev_data)
{
  struct instance *priv       = (struct instance *)dev_data->private;
  clock_t          time_delta;
  DMA_HANDLE       dma_handle = rp2040_dmachannel();
  dma_config_t     dma_config =
    {
      .dreq   = rp2040_pio_get_dreq(priv->pio, priv->pio_sm, true),
      .size   = RP2040_DMA_SIZE_WORD,
      .noincr = false
    };

  rp2040_txdmasetup(dma_handle,
                    (uintptr_t) RP2040_PIO_TXF(priv->pio, priv->pio_sm),
                    (uintptr_t) priv->pixels,
                    4 * dev_data->nleds,
                    dma_config);

  /* Make sure at least 50us elapsed since last DMA completed. */

  time_delta =   (clock_systime_ticks() - priv->last_dma)
               * CONFIG_USEC_PER_TICK;

  if (time_delta < 50)
    {
      usleep(50 - time_delta);
    }

  rp2040_dmastart(dma_handle, dma_complete, dev_data);

  /* NOTE: we don't post exclsem here, the dma_complete does that */
}

/****************************************************************************
 * Name: my_open
 *
 * Description:
 *
 * Input Parameters:
 *   dev_data - Pointer to a ws2812_dev_s
 *
 * Returned Value:
 *   A pointer to an internal structure used by rp2040_ws2812
 *
 ****************************************************************************/

static int my_open(struct file *filep)
{
  struct inode         *inode     = filep->f_inode;
  struct ws2812_dev_s  *dev_data  = inode->i_private;
  struct instance      *priv      = (struct instance *)dev_data->private;
  rp2040_pio_sm_config  config;
  int                   divisor;
  int                   ret;

  nxsem_wait(&dev_data->exclsem);

  priv->open_count += 1;

  if (priv->pixels != NULL)
    {
      /* We've already been initialized.  Keep on truckin' */

      ledinfo("rp2040_ws2812 re-open dev: 0x%08lX\n", (uint32_t) dev_data);

      ret = OK;
      goto post_and_return;
    }

  ledinfo("rp2040_ws2812 open dev: 0x%08lX\n", (uint32_t) dev_data);

  /* Allocate the pixel buffer */

  priv->pixels = kmm_zalloc(4 * dev_data->nleds);

  if (priv->pixels == NULL)
    {
      lederr("rp2040_ws2812 open: out of memory\n");

      ret = -ENOMEM;
      goto post_and_return;
    }

  /* ==== Load the pio program ==== */

  /* get pio instance and load program */

  for (priv->pio = 0; priv->pio < RP2040_PIO_NUM; ++priv->pio)
    {
      /* Try to claim a state machine */

      priv->pio_sm = rp2040_pio_claim_unused_sm(priv->pio, false);

      /* If we did not get one try the nest pio block, if any */

      if (priv->pio_sm < 0) continue;

      /* See if we have space in this block to load our program */

      if (rp2040_pio_can_add_program(priv->pio, &pio_program))
        {
          /* Great! load the program and exit the pio choice loop */

          priv->pio_location = rp2040_pio_add_program(priv->pio,
                                                      &pio_program);

          break;
        }

      /* Oops -- no room at the inn!  Release sm and try next pio */

      rp2040_pio_sm_unclaim(priv->pio, priv->pio_sm);
    }

  if (priv->pio >= RP2040_PIO_NUM)
    {
      kmm_free(priv->pixels);

      ret = -ENOMEM;
      goto post_and_return;
    }

  /* ==== configure the pio state machine ==== */

  /* Configure our pin as used by PIO for output */

  rp2040_pio_gpio_init(priv->pio, dev_data->port);

  rp2040_pio_sm_set_consecutive_pindirs(priv->pio,
                                        priv->pio_sm,
                                        dev_data->port,
                                        1,
                                        true);

  /* Initialize the config structure */

  memset(&config, 1, sizeof(rp2040_pio_sm_config));

  /* Set the clock divisor as appropriate for our system clock speed
   * so the pio clock rus at nine time the requested bit clock rate
   */

  divisor =   ((uint64_t)BOARD_SYS_FREQ << 8)
            / (9 * (uint64_t)dev_data->clock);

  rp2040_sm_config_set_clkdiv_int_frac(&config,
                                       divisor >> 8,
                                       divisor & 0xff);

  /* Set the wrap points as required by the program */

  rp2040_sm_config_set_wrap(&config,
                            priv->pio_location + ws2812_wrap_target,
                            priv->pio_location + ws2812_wrap);

  /* set to shift out 24 or 32 bits depending on has_white. */

  rp2040_sm_config_set_out_shift(&config,
                                 false,
                                 true,
                                 dev_data->has_white ? 32 : 24);

  /* Configure a single mandatory side-set pin */

  rp2040_sm_config_set_sideset(&config, 1, false, false);

  /* Since we don't need an RX fifo, well make a bit TX fifo */

  rp2040_sm_config_set_fifo_join(&config,
                                 RP2040_PIO_FIFO_JOIN_TX);

  /* Configure a single mandatory side-set pin */

  rp2040_sm_config_set_sideset(&config, 1, false, false);

  /* Configure our chosen GPIO pin (in "port") as side-set output */

  rp2040_sm_config_set_sideset_pins(&config, dev_data->port);

  /* Load the configuration into the state machine. */

  rp2040_pio_sm_init(priv->pio,
                     priv->pio_sm,
                     priv->pio_location,
                     &config);

  /* Enable the state machine */

  rp2040_pio_sm_set_enabled(priv->pio, priv->pio_sm, true);

  /* Turn on the power pin if any */

  if (priv->power_pin >= 0)
    {
      rp2040_gpio_init(priv->power_pin);
      rp2040_gpio_setdir(priv->power_pin, true);
      rp2040_gpio_put(priv->power_pin, true);
    }

  ret = OK;

post_and_return:
  nxsem_post(&dev_data->exclsem);

  return ret;
}

/****************************************************************************
 * Name: my_close
 *
 * Description:
 *
 * Input Parameters:
 *   dev_data - Pointer to a ws2812_dev_s
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int my_close(struct file *filep)
{
  struct inode        *inode    = filep->f_inode;
  struct ws2812_dev_s *dev_data = inode->i_private;
  struct instance     *priv     = (struct instance *)dev_data->private;

  nxsem_wait(&dev_data->exclsem);

  ledinfo("rp2040_ws2812 close dev: 0x%08lX\n", (uint32_t) dev_data);

  priv->open_count -= 1;

  if (priv->open_count == 0  &&  priv->power_pin >= 0)
    {
      rp2040_gpio_put(priv->power_pin, false);
    }

  nxsem_post(&dev_data->exclsem);

  return OK;
}

/****************************************************************************
 * Name: my_write
 * Description:
 *   Updates the ws2812s with new data.
 *
 * Input Parameter:
 *   filep    - Pointer system file data
 *   data     - Data to send.
 *   len      - Length of data in bytes.
 *
 * Returned Value:
 *   number of bytes written on success, ERROR if write fails.
 *
 ****************************************************************************/

static ssize_t my_write(struct file *filep,
                        const char  *data,
                        size_t       len)
{
  struct inode        *inode      = filep->f_inode;
  struct ws2812_dev_s *dev_data   = inode->i_private;
  struct instance     *priv       = (struct instance *)dev_data->private;
  int                  position   = filep->f_pos;
  uint8_t             *xfer_p     = priv->pixels + position;
  int                  xfer_index = 0;

  if (data == NULL)
    {
      return 0;
    }

  nxsem_wait(&dev_data->exclsem);

  ledinfo("rp2040_ws2812 write dev: 0x%08lX\n", (uint32_t) dev_data);

  if (len > 0)
    {
      /* Copy the data to the buffer swapping the
       * red and green, since ws2812 use a GRB order
       * instead of RGB
       */

      for (xfer_index = 0; xfer_index < len; xfer_index += 4)
        {
          /* Stop transfer at end of pixel buffer */

          if (position >= (4 * dev_data->nleds))
            {
              ledinfo("rp2040_ws2812 write off end: %d\n", position);
              break;
            }

          /* Copy swapping WWRRGGBB to GGRRBBWW */

    #ifdef CONFIG_BIG_ENDIAN
          xfer_p[3] = *data++;
          xfer_p[1] = *data++;
          xfer_p[0] = *data++;
          xfer_p[2] = *data++;
    #else /* CONFIG_BIG_ENDIAN */
          xfer_p[1] = *data++;
          xfer_p[3] = *data++;
          xfer_p[2] = *data++;
          xfer_p[0] = *data++;
    #endif /* CONFIG_BIG_ENDIAN */

          xfer_p   += 4;
          position += 4;
        }

      filep->f_pos = position;
    }

  update_pixels(dev_data);

  /* NOTE: we don't post exclsem here, so update_pixels must make sure
   *       that happens.
   */

  return xfer_index;
}

/****************************************************************************
 * Name: my_read
 * Description:
 *   Reads data back from the pixel buffer.
 *
 * Input Parameter:
 *   filep    - Pointer system file data
 *   data     - Buffer for return data.
 *   len      - Length of data in bytes.
 *
 * Returned Value:
 *   number of bytes read on success, ERROR if write fails.
 *
 ****************************************************************************/

static ssize_t my_read(struct file *filep,
                       char        *data,
                       size_t       len)
{
  struct inode        *inode      = filep->f_inode;
  struct ws2812_dev_s *dev_data   = inode->i_private;
  struct instance     *priv       = (struct instance *)dev_data->private;
  int                  position   = filep->f_pos;
  uint8_t             *xfer_p     = priv->pixels + position;
  int                  xfer_index = 0;

  if (data == NULL  ||  len == 0)
    {
      return 0;
    }

  nxsem_wait(&dev_data->exclsem);

  /* Copy the data from the buffer swapping the
   * red and green, since ws2812 use a GRB order
   * instead of RGB
   */

  for (xfer_index = 0; xfer_index < len; xfer_index += 4)
    {
      /* Stop transfer at end of pixel buffer */

      if (position >= (4 * dev_data->nleds))
        {
          ledinfo("rp2040_ws2812 read off end: %d\n", position);
          break;
        }

      /* Copy swapping GGRRBBWW to WWRRGGBB  */

  #ifdef CONFIG_BIG_ENDIAN
      *data++ = xfer_p[3];
      *data++ = xfer_p[1];
      *data++ = xfer_p[0];
      *data++ = xfer_p[2];
  #else /* CONFIG_BIG_ENDIAN */
      *data++ = xfer_p[1];
      *data++ = xfer_p[3];
      *data++ = xfer_p[2];
      *data++ = xfer_p[0];
  #endif /* CONFIG_BIG_ENDIAN */

      xfer_p   += 4;
      position += 4;
    }

  filep->f_pos = position;

  nxsem_wait(&dev_data->exclsem);

  return xfer_index;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_ws2812_setup
 *
 * Description:
 *   Initialize and register the ws2812 driver.
 *
 * Input Parameters:
 *   Path to the ws2812 device  (e.g. "/dev/leds0")
 *   Port number for the ws2812 chain
 *   Pin for ws2812 power
 *   The number of pixels in the chain
 *   Whether ws2812s have white LEDs
 *
 * Returned Value:
 *   An opaque pointer that can be passed to rp2040_ws2812_teardown on
 *   success or NULL (with errno set) on failure
 ****************************************************************************/

void * rp2040_ws2812_setup(const char *path,
                           int         port,
                           int         power_pin,
                           uint16_t    pixel_count,
                           bool        has_white)
{
  struct ws2812_dev_s *dev_data;
  struct instance     *priv;
  int err;

  dev_data = kmm_zalloc(sizeof(struct ws2812_dev_s));

  if (dev_data == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  /* Allocate struct holding out persistent data */

  priv = kmm_zalloc(sizeof(struct instance));

  if (priv == NULL)
    {
      lederr("rp2040_ws2812 open: out of memory\n");

      kmm_free(dev_data);
      set_errno(ENOMEM);
      return NULL;
    }

  dev_data->open      = my_open;
  dev_data->close     = my_close;
  dev_data->write     = my_write;
  dev_data->read      = my_read;
  dev_data->port      = port;
  dev_data->nleds     = pixel_count;
  dev_data->clock     = CONFIG_WS2812_FREQUENCY;
  dev_data->private   = priv;

  nxsem_init(&dev_data->exclsem, 0, 1);

  priv->power_pin     = power_pin;

  ledinfo("register dev_data: 0x%08lX\n", (uint32_t) dev_data);

  err = ws2812_register(path, dev_data);

  if (err != OK)
    {
      set_errno(err);
      return NULL;
    }

  return (void *)dev_data;
}

/****************************************************************************
 * Name: rp2040_ws2812_release
 *
 * Description:
 *   This function releases the internal memory structures created when
 *   a driver is opened.  It will fail with an error -EBUSY the driver
 *   is opened.
 *
 * Input Parameters:
 *   driver      - Opaque pointer returned by rp2040_ws2812_setup.
 *
 * Returned Value:
 *   OK on success or an ERROR on failure
 *
 ****************************************************************************/

int rp2040_ws2812_release(void * driver)
{
  struct ws2812_dev_s *dev_data = driver;
  struct instance     *priv     = (struct instance *)dev_data->private;

  int ret = OK;

  nxsem_wait(&dev_data->exclsem);

  if (priv->open_count == 0)
    {
      dev_data->private = NULL;

      rp2040_pio_sm_set_enabled(priv->pio, priv->pio_sm, false);
      rp2040_pio_sm_unclaim(priv->pio, priv->pio_sm);

      nxsem_post(&dev_data->exclsem);

      kmm_free(priv->pixels);
      kmm_free(priv);
    }
    else
    {
      ret = -EBUSY;
      nxsem_post(&dev_data->exclsem);
    }

  return ret;
}

#endif /* CONFIG_WS2812 */
