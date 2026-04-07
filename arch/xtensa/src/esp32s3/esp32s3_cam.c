/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cam.c
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

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/imgdata.h>

#include <arch/board/board.h>

#include "esp32s3_dma.h"
#include "espressif/esp_gpio.h"
#include "espressif/esp_irq.h"

#include "xtensa.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_lcd_cam.h"

#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use PLL=160MHz as clock resource for CAM */

#define ESP32S3_CAM_CLK_SEL       2
#define ESP32S3_CAM_CLK_MHZ       160

/* XCLK = 160 / (6 + 2/3) = 24 MHz */

#define ESP32S3_CAM_CLKM_DIV_NUM  6
#define ESP32S3_CAM_CLKM_DIV_A    3
#define ESP32S3_CAM_CLKM_DIV_B    2

/* DMA buffer configuration */

#define ESP32S3_CAM_DMA_BUFLEN    4096
#define ESP32S3_CAM_DMADESC_NUM   40

/* VSYNC filter threshold */

#define ESP32S3_CAM_VSYNC_FILTER  4

/* GDMA external memory block size setting for PSRAM RX.
 * Change this single macro to switch between 16B / 32B / 64B.
 * ESP32S3_CAM_DMA_ALIGN is the byte alignment derived from it.
 */

#define ESP32S3_CAM_EXT_MEMBLK    ESP32S3_DMA_EXT_MEMBLK_64B

#if ESP32S3_CAM_EXT_MEMBLK == ESP32S3_DMA_EXT_MEMBLK_64B
#  define ESP32S3_CAM_DMA_ALIGN   64
#elif ESP32S3_CAM_EXT_MEMBLK == ESP32S3_DMA_EXT_MEMBLK_32B
#  define ESP32S3_CAM_DMA_ALIGN   32
#else
#  define ESP32S3_CAM_DMA_ALIGN   16
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_cam_s
{
  struct imgdata_s data;

  bool capturing;

  int cpuint;
  uint8_t cpu;
  int32_t dma_channel;

  struct esp32s3_dmadesc_s *dmadesc; /* Heap-allocated DMA descriptors */

  uint8_t *fb;                    /* Frame buffer */
  uint32_t fb_size;               /* Frame buffer size */
  uint32_t fb_pos;                /* Current write position */
  bool fb_allocated;              /* true if driver allocated fb */
  uint8_t vsync_cnt;              /* VSYNC counter for frame sync */

  imgdata_capture_t cb;           /* Capture done callback */
  void *cb_arg;                   /* Callback argument */

  uint16_t width;
  uint16_t height;
  uint32_t pixfmt;

  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_cam_init(struct imgdata_s *data);
static int esp32s3_cam_uninit(struct imgdata_s *data);
static int esp32s3_cam_set_buf(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size);
static int esp32s3_cam_validate_frame_setting(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval);
static int esp32s3_cam_start_capture(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval,
                               imgdata_capture_t callback,
                               void *arg);
static int esp32s3_cam_stop_capture(struct imgdata_s *data);
static void *esp32s3_cam_alloc(struct imgdata_s *data,
                               uint32_t align_size, uint32_t size);
static void esp32s3_cam_free(struct imgdata_s *data, void *addr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_cam_ops =
{
  .init                   = esp32s3_cam_init,
  .uninit                 = esp32s3_cam_uninit,
  .set_buf                = esp32s3_cam_set_buf,
  .validate_frame_setting = esp32s3_cam_validate_frame_setting,
  .start_capture          = esp32s3_cam_start_capture,
  .stop_capture           = esp32s3_cam_stop_capture,
  .alloc                  = esp32s3_cam_alloc,
  .free                   = esp32s3_cam_free,
};

static struct esp32s3_cam_s g_cam_priv =
{
  .data =
    {
      .ops = &g_cam_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cam_interrupt
 *
 * Description:
 *   CAM VSYNC interrupt handler. Called when a frame is complete.
 *
 ****************************************************************************/

static int IRAM_ATTR cam_interrupt(int irq, void *context, void *arg)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)arg;
  uint32_t status = getreg32(LCD_CAM_LC_DMA_INT_ST_REG);

  /* Only handle CAM VSYNC interrupt */

  if (status & LCD_CAM_CAM_VSYNC_INT_ST_M)
    {
      /* Clear interrupt */

      putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M,
               LCD_CAM_LC_DMA_INT_CLR_REG);

      if (priv->capturing)
        {
          priv->vsync_cnt++;

          /* First VSYNC = start of frame (DMA begins filling buffer).
           * Second VSYNC = end of frame (buffer is complete).
           */

          if (priv->vsync_cnt == 1)
            {
              /* Frame capture just started, DMA is now receiving data.
               * Nothing to do here — wait for next VSYNC.
               */
            }
          else if (priv->vsync_cnt >= 2 && priv->cb)
            {
              struct timeval ts;
              uint32_t regval;

              /* Stop capture and DMA before invoking callback.
               * The callback may call set_buf() which rewrites DMA
               * descriptors.  If DMA is still draining the CAM AFIFO
               * it could read half-written descriptors and follow a
               * corrupted pbuf pointer, writing pixel data over
               * unrelated memory (e.g. g_cam_priv.data.ops).
               */

              regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
              regval &= ~LCD_CAM_CAM_START_M;
              putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

              regval = getreg32(LCD_CAM_CAM_CTRL_REG);
              regval |= LCD_CAM_CAM_UPDATE_REG_M;
              putreg32(regval, LCD_CAM_CAM_CTRL_REG);

              /* Stop DMA channel before callback to prevent race */

              esp32s3_dma_reset_channel(priv->dma_channel, false);

              gettimeofday(&ts, NULL);

              /* Notify frame complete */

              priv->cb(0, priv->fb_size, &ts, priv->cb_arg);

              /* Check if callback called stop_capture.  With a
               * single-buffer FIFO the V4L2 layer stops capture
               * inside the callback; restarting DMA after that
               * would run unsynchronized and corrupt memory.
               */

              if (!priv->capturing)
                {
                  priv->vsync_cnt = 0;
                }
              else
                {
                  /* Restart capture for next frame */

                  priv->vsync_cnt = 0;

                  /* DMA channel was already reset before the
                   * callback above.  Reset CAM + AFIFO now.
                   */

                  /* Reset CAM + AFIFO */

                  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
                  regval |= LCD_CAM_CAM_RESET_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
                  regval &= ~LCD_CAM_CAM_RESET_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

                  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
                  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

                  /* Re-set REC_DATA_BYTELEN after reset */

                  regval &= ~LCD_CAM_CAM_REC_DATA_BYTELEN_M;
                  regval |= ((ESP32S3_CAM_DMA_BUFLEN - 1)
                             << LCD_CAM_CAM_REC_DATA_BYTELEN_S);
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

                  /* Reload DMA descriptors */

                  esp32s3_dma_load(priv->dmadesc, priv->dma_channel,
                                   false);
                  esp32s3_dma_enable(priv->dma_channel, false);

                  /* Restart */

                  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
                  regval |= LCD_CAM_CAM_START_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

                  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
                  regval |= LCD_CAM_CAM_UPDATE_REG_M;
                  putreg32(regval, LCD_CAM_CAM_CTRL_REG);
                }
            }
        }
    }

  /* Clear any CAM HS interrupt too */

  if (status & LCD_CAM_CAM_HS_INT_ST_M)
    {
      putreg32(LCD_CAM_CAM_HS_INT_CLR_M,
               LCD_CAM_LC_DMA_INT_CLR_REG);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_cam_gpio_config
 *
 * Description:
 *   Configure GPIO pins for DVP camera interface.
 *
 ****************************************************************************/

static void esp32s3_cam_gpio_config(void)
{
  /* Data pins D0-D7 as input via GPIO matrix */

  esp_configgpio(CONFIG_ESP32S3_CAM_D0_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D0_PIN, CAM_DATA_IN0_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D1_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D1_PIN, CAM_DATA_IN1_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D2_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D2_PIN, CAM_DATA_IN2_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D3_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D3_PIN, CAM_DATA_IN3_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D4_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D4_PIN, CAM_DATA_IN4_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D5_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D5_PIN, CAM_DATA_IN5_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D6_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D6_PIN, CAM_DATA_IN6_IDX, false);

  esp_configgpio(CONFIG_ESP32S3_CAM_D7_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_D7_PIN, CAM_DATA_IN7_IDX, false);

  /* PCLK input */

  esp_configgpio(CONFIG_ESP32S3_CAM_PCLK_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_PCLK_PIN, CAM_PCLK_IDX, false);

  /* VSYNC input */

  esp_configgpio(CONFIG_ESP32S3_CAM_VSYNC_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_VSYNC_PIN, CAM_V_SYNC_IDX, false);

  /* HREF (H_ENABLE) input */

  esp_configgpio(CONFIG_ESP32S3_CAM_HREF_PIN, INPUT);
  esp_gpio_matrix_in(CONFIG_ESP32S3_CAM_HREF_PIN, CAM_H_ENABLE_IDX, false);

  /* XCLK output to sensor */

  esp_configgpio(CONFIG_ESP32S3_CAM_XCLK_PIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32S3_CAM_XCLK_PIN,
                      CAM_CLK_IDX, false, false);
}

/****************************************************************************
 * Name: esp32s3_cam_enableclk
 *
 * Description:
 *   Enable CAM clock and configure XCLK output.
 *
 ****************************************************************************/

static void esp32s3_cam_enableclk(void)
{
  uint32_t regval;

  /* Enable LCD_CAM peripheral clock (shared with LCD) */

  periph_module_enable(PERIPH_LCD_CAM_MODULE);

  /* Configure CAM clock:
   *   CLK_SEL = 3 (PLL 160MHz)
   *   DIV_NUM = 6, DIV_A = 3, DIV_B = 2
   *   XCLK = 160 / (6 + 2/3) = 24 MHz
   */

  regval = (ESP32S3_CAM_CLK_SEL << LCD_CAM_CAM_CLK_SEL_S) |
           (ESP32S3_CAM_CLKM_DIV_A << LCD_CAM_CAM_CLKM_DIV_A_S) |
           (ESP32S3_CAM_CLKM_DIV_B << LCD_CAM_CAM_CLKM_DIV_B_S) |
           (ESP32S3_CAM_CLKM_DIV_NUM << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
           LCD_CAM_CAM_VS_EOF_EN_M |  /* VSYNC triggers DMA EOF */
           (ESP32S3_CAM_VSYNC_FILTER << LCD_CAM_CAM_VSYNC_FILTER_THRES_S);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);
}

/****************************************************************************
 * Name: esp32s3_cam_dmasetup
 *
 * Description:
 *   Configure DMA for camera RX.
 *
 ****************************************************************************/

static int esp32s3_cam_dmasetup(struct esp32s3_cam_s *priv)
{
  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          1, 10, true);
  if (priv->dma_channel < 0)
    {
      snerr("ERROR: Failed to allocate GDMA channel\n");
      return -ENOMEM;
    }

  esp32s3_dma_set_ext_memblk(priv->dma_channel,
                             false,
                             ESP32S3_CAM_EXT_MEMBLK);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_config
 *
 * Description:
 *   Configure CAM controller registers and interrupts.
 *
 ****************************************************************************/

static int esp32s3_cam_config(struct esp32s3_cam_s *priv)
{
  int ret;
  uint32_t regval;
  irqstate_t flags;

  /* Reset CAM module and AFIFO first */

  regval = LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Configure CAM_CTRL1 after reset:
   *   - VSYNC filter enable
   *   - REC_DATA_BYTELEN = DMA node size - 1 (controls in_suc_eof)
   *   Note: 8-bit DVP sensor outputs BE RGB565 (high byte first).
   *         CAM_BYTE_ORDER only reverses bits within a byte in
   *         1-byte mode, so HW byte swap is not available here.
   *         Application must swap bytes if LE RGB565 is needed.
   */

  regval = LCD_CAM_CAM_VSYNC_FILTER_EN_M |
           ((ESP32S3_CAM_DMA_BUFLEN - 1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Update registers */

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Bypass RGB/YUV conversion (bit=0 means bypass) */

  putreg32(0, LCD_CAM_CAM_RGB_YUV_REG);

  /* Setup DMA */

  if (esp32s3_cam_dmasetup(priv) != OK)
    {
      return -ENOMEM;
    }

  /* Disable all LCD_CAM interrupts and clear pending */

  putreg32(0, LCD_CAM_LC_DMA_INT_ENA_REG);
  putreg32(0xffffffff, LCD_CAM_LC_DMA_INT_CLR_REG);

  /* Setup interrupt with CPU interrupts disabled.
   * Order matters:
   *   1) setup_irq creates cpuint → peripheral IRQ mapping
   *   2) irq_attach registers the handler for that IRQ
   * Since spin_lock_irqsave masks CPU interrupts, even if the
   * peripheral line is asserted after setup_irq, the interrupt
   * won't be serviced until after irq_attach completes.
   */

  flags = spin_lock_irqsave(&priv->lock);

  priv->cpu = this_cpu();

  /* 1) Map peripheral → CPU interrupt (creates IRQ mapping) */

  priv->cpuint = esp_setup_irq(ESP32S3_PERIPH_LCD_CAM,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               cam_interrupt,
                               priv);
  DEBUGASSERT(priv->cpuint >= 0);

  UNUSED(ret);

  /* 3) Clear any spurious interrupt that fired during setup */

  putreg32(0xffffffff, LCD_CAM_LC_DMA_INT_CLR_REG);

  spin_unlock_irqrestore(&priv->lock, flags);

  up_enable_irq(ESP32S3_IRQ_LCD_CAM);

  /* Enable CAM VSYNC interrupt in hardware */

  regval = getreg32(LCD_CAM_LC_DMA_INT_ENA_REG);
  regval |= LCD_CAM_CAM_VSYNC_INT_ENA_M;
  putreg32(regval, LCD_CAM_LC_DMA_INT_ENA_REG);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_init
 ****************************************************************************/

static int esp32s3_cam_init(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;

  priv->capturing = false;
  priv->cb = NULL;
  priv->cb_arg = NULL;
  priv->fb = NULL;
  priv->fb_size = 0;
  priv->fb_pos = 0;
  priv->width = 0;
  priv->height = 0;
  priv->pixfmt = 0;

  spin_lock_init(&priv->lock);

  /* Allocate DMA descriptors from heap so they are isolated from
   * the driver struct.  If GDMA ever follows a stale/corrupted
   * descriptor it will scribble on heap, not on g_cam_priv.
   */

  if (priv->dmadesc == NULL)
    {
      priv->dmadesc = kmm_memalign(4,
                        sizeof(struct esp32s3_dmadesc_s) *
                        ESP32S3_CAM_DMADESC_NUM);
      if (priv->dmadesc == NULL)
        {
          snerr("ERROR: Failed to allocate DMA descriptors\n");
          return -ENOMEM;
        }
    }

  /* Configure GPIO pins */

  esp32s3_cam_gpio_config();

  /* Enable clock and configure XCLK */

  esp32s3_cam_enableclk();

  /* Configure CAM controller */

  return esp32s3_cam_config(priv);
}

/****************************************************************************
 * Name: esp32s3_cam_uninit
 ****************************************************************************/

static int esp32s3_cam_uninit(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;
  irqstate_t flags;

  /* Stop capture */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset CAM module and AFIFO to stop all hardware activity */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Keep XCLK running so the sensor stays accessible via I2C for
   * subsequent re-initialization.  VSYNC generation is already
   * stopped by the CAM_RESET above.  Disable only the CAM_START
   * bit in CAM_CTRL to stop the capture engine while preserving
   * the clock divider configuration.
   */

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval &= ~LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Stop and release DMA before tearing down the CPU-level IRQ.
   * esp32s3_dma_release() detaches the GDMA channel interrupt;
   * if DMA is still active it may fire after detach -> irq_unexpected.
   */

  if (priv->dma_channel >= 0)
    {
      esp32s3_dma_reset_channel(priv->dma_channel, false);
      esp32s3_dma_release(priv->dma_channel);
      priv->dma_channel = -1;
    }

  /* Mask CPU interrupts so no new peripheral interrupt can be
   * delivered between clearing the pending flag and detaching
   * the handler.  XCLK is still running (kept for I2C access),
   * so a VSYNC edge that arrived before the CAM_RESET could
   * still be latched in the interrupt controller.
   */

  flags = spin_lock_irqsave(&priv->lock);

  putreg32(0, LCD_CAM_LC_DMA_INT_ENA_REG);
  putreg32(0xffffffff, LCD_CAM_LC_DMA_INT_CLR_REG);

  up_disable_irq(ESP32S3_IRQ_LCD_CAM);
  esp_teardown_irq(ESP32S3_PERIPH_LCD_CAM, priv->cpuint);

  spin_unlock_irqrestore(&priv->lock, flags);

  /* Free DMA descriptors */

  if (priv->dmadesc)
    {
      kmm_free(priv->dmadesc);
      priv->dmadesc = NULL;
    }

  /* Free frame buffer only if driver allocated it */

  if (priv->fb && priv->fb_allocated)
    {
      kmm_free(priv->fb);
    }

  priv->fb = NULL;
  priv->fb_allocated = false;

  priv->capturing = false;

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_set_buf
 ****************************************************************************/

static int esp32s3_cam_set_buf(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               uint8_t *addr, uint32_t size)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;

  /* Store negotiated format from upper layer */

  priv->width  = datafmts[IMGDATA_FMT_MAIN].width;
  priv->height = datafmts[IMGDATA_FMT_MAIN].height;
  priv->pixfmt = datafmts[IMGDATA_FMT_MAIN].pixelformat;

  if (addr != NULL && size > 0)
    {
      priv->fb = addr;
      priv->fb_size = size;
      priv->fb_allocated = false;
    }
  else
    {
      /* Allocate frame buffer in PSRAM if available.
       * 8-bit DVP formats (RGB565, YUV422) are all 2 bytes per pixel.
       */

      priv->fb_size = priv->width * priv->height * 2;
      priv->fb = kmm_memalign(ESP32S3_CAM_DMA_ALIGN, priv->fb_size);
      if (!priv->fb)
        {
          snerr("ERROR: Failed to allocate frame buffer\n");
          return -ENOMEM;
        }

      priv->fb_allocated = true;
    }

  memset(priv->fb, 0, priv->fb_size);

  /* Setup DMA descriptors for RX into frame buffer */

  esp32s3_dma_setup(priv->dmadesc,
                    ESP32S3_CAM_DMADESC_NUM,
                    priv->fb,
                    priv->fb_size,
                    false,  /* RX */
                    priv->dma_channel);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_alloc
 *
 * Description:
 *   Allocate frame buffer memory with GDMA-required alignment.
 *   GDMA with EXT_MEMBLK_64B needs 64-byte aligned addresses
 *   for PSRAM access.
 *
 ****************************************************************************/

static void *esp32s3_cam_alloc(struct imgdata_s *data,
                               uint32_t align_size, uint32_t size)
{
  return kmm_memalign(ESP32S3_CAM_DMA_ALIGN, size);
}

/****************************************************************************
 * Name: esp32s3_cam_free
 ****************************************************************************/

static void esp32s3_cam_free(struct imgdata_s *data, void *addr)
{
  kmm_free(addr);
}

/****************************************************************************
 * Name: esp32s3_cam_validate_frame_setting
 ****************************************************************************/

static int esp32s3_cam_validate_frame_setting(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval)
{
  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  /* Pixel format is validated by the sensor driver (imgsensor);
   * imgdata only checks that width and height are non-zero.
   */

  if (datafmts[IMGDATA_FMT_MAIN].width == 0 ||
      datafmts[IMGDATA_FMT_MAIN].height == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_start_capture
 ****************************************************************************/

static int esp32s3_cam_start_capture(struct imgdata_s *data,
                               uint8_t nr_datafmts,
                               imgdata_format_t *datafmts,
                               imgdata_interval_t *interval,
                               imgdata_capture_t callback,
                               void *arg)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;

  if (priv->capturing)
    {
      return -EBUSY;
    }

  if (!priv->fb)
    {
      return -EINVAL;
    }

  priv->cb = callback;
  priv->cb_arg = arg;
  priv->fb_pos = 0;
  priv->vsync_cnt = 0;

  /* Stop CAM first */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset CAM module + AFIFO (match esp32-camera ll_cam_start) */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset DMA channel */

  esp32s3_dma_reset_channel(priv->dma_channel, false);

  /* Re-set REC_DATA_BYTELEN after reset (reference does this) */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_REC_DATA_BYTELEN_M;
  regval |= ((ESP32S3_CAM_DMA_BUFLEN - 1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Load DMA descriptors */

  esp32s3_dma_load(priv->dmadesc, priv->dma_channel, false);
  esp32s3_dma_enable(priv->dma_channel, false);

  /* Start capture */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Update registers */

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  priv->capturing = true;

  return OK;
}

/****************************************************************************
 * Name: esp32s3_cam_stop_capture
 ****************************************************************************/

static int esp32s3_cam_stop_capture(struct imgdata_s *data)
{
  struct esp32s3_cam_s *priv = (struct esp32s3_cam_s *)data;
  uint32_t regval;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  /* Mark not capturing first so ISR won't process further VSYNCs */

  priv->capturing = false;
  priv->cb = NULL;
  priv->cb_arg = NULL;

  /* Stop capture engine */

  regval = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset CAM + AFIFO to fully quiesce hardware */

  regval |= LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval |= LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  regval = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Reset DMA channel to abort any in-flight transfer */

  esp32s3_dma_reset_channel(priv->dma_channel, false);

  /* Clear any pending VSYNC interrupt */

  putreg32(LCD_CAM_CAM_VSYNC_INT_CLR_M,
           LCD_CAM_LC_DMA_INT_CLR_REG);

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_cam_initialize
 *
 * Description:
 *   Initialize the ESP32-S3 CAM imgdata driver.
 *   Starts XCLK output for sensor communication.
 *
 * Returned Value:
 *   Pointer to imgdata_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgdata_s *esp32s3_cam_initialize(void)
{
  /* Start XCLK output to sensor — must be running before
   * sensor I2C communication (e.g. gc0308_initialize).
   */

  esp_configgpio(CONFIG_ESP32S3_CAM_XCLK_PIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32S3_CAM_XCLK_PIN,
                      CAM_CLK_IDX, false, false);
  esp32s3_cam_enableclk();

  return &g_cam_priv.data;
}
