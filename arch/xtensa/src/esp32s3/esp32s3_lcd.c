/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_lcd.c
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
#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_gpio.h"
#include "esp32s3_dma.h"
#include "esp32s3_irq.h"

#include "xtensa.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_lcd_cam.h"

#include "periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

#ifdef CONFIG_ESP32S3_LCD_DATA_16BIT
#  define ESP32S3_LCD_DATA_WIDTH  2
#  define ESP32S3_LCD_DATA_BPP    16
#else
#  error "Configure LCD data width is not supported"
#endif

#if CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240
#  if (CONFIG_ESP32S3_LCD_CLOCK_MHZ % 3) == 0
    /* Use PLL=240MHz as clock resource */
#    define ESP32S3_LCD_CLK_SEL   2
#    define ESP32S3_LCD_CLK_MHZ   240
#  else
    /* Use PLL=160MHz as clock resource */
#    define ESP32S3_LCD_CLK_SEL   3
#    define ESP32S3_LCD_CLK_MHZ   160
#  endif
#else
  /* Use PLL=160MHz as clock resource */
#  define ESP32S3_LCD_CLK_SEL     3
#  define ESP32S3_LCD_CLK_MHZ     160
#endif

/* Total Pins */

#define ESP32S3_LCD_PINS          20

/* LCD configuration parameters */

#define ESP32S3_LCD_CLK_N         (ESP32S3_LCD_CLK_MHZ / \
                                   CONFIG_ESP32S3_LCD_CLOCK_MHZ)
#define ESP32S3_LCD_CLK_RES       (ESP32S3_LCD_CLK_MHZ % \
                                   CONFIG_ESP32S3_LCD_CLOCK_MHZ)

#define ESP32S3_LCD_VT_HIGHT      (CONFIG_ESP32S3_LCD_VRES + \
                                   CONFIG_ESP32S3_LCD_VBACKPORCH + \
                                   CONFIG_ESP32S3_LCD_VFRONTPORCH + \
                                   CONFIG_ESP32S3_LCD_VPULSEWIDTH - 1)

#define ESP32S3_LCD_HT_WIDTH      (CONFIG_ESP32S3_LCD_HRES + \
                                   CONFIG_ESP32S3_LCD_HBACKPORCH + \
                                   CONFIG_ESP32S3_LCD_HFRONTPORCH + \
                                   CONFIG_ESP32S3_LCD_HPULSEWIDTH - 1)

#define ESP32S3_LCD_VA_HIGHT      (CONFIG_ESP32S3_LCD_VRES - 1)

#define ESP32S3_LCD_HA_WIDTH      (CONFIG_ESP32S3_LCD_HRES - 1)

#define ESP32S3_LCD_VA_FRONT      (CONFIG_ESP32S3_LCD_VBACKPORCH + \
                                   CONFIG_ESP32S3_LCD_VPULSEWIDTH - 1)

#define ESP32S3_LCD_HB_FRONT      (CONFIG_ESP32S3_LCD_HBACKPORCH + \
                                   CONFIG_ESP32S3_LCD_HPULSEWIDTH -1)

#define ESP32S3_LCD_HSYNC_WIDTH   (CONFIG_ESP32S3_LCD_HPULSEWIDTH - 1)

#define ESP32S3_LCD_VSYNC_WIDTH   (CONFIG_ESP32S3_LCD_VPULSEWIDTH - 1)

#define ESP32S3_LCD_COLOR_FMT     (FB_FMT_RGB16_565)
#define ESP32S3_LCD_STRIDE        (CONFIG_ESP32S3_LCD_HRES * \
                                   ESP32S3_LCD_DATA_WIDTH)

/* Display memory buffer and DMA */

#define ESP32S3_LCD_FB_SIZE       (CONFIG_ESP32S3_LCD_HRES * \
                                   CONFIG_ESP32S3_LCD_VRES * \
                                   ESP32S3_LCD_DATA_WIDTH)

#define ESP32S3_LCD_DMADESC_NUM   (ESP32S3_LCD_FB_SIZE / \
                                   ESP32S3_DMA_BUFLEN_MAX + 1)

#define ESP32S3_LCD_LAYERS        CONFIG_ESP32S3_LCD_BUFFER_LAYERS

/* Get current layer pointer */

#define CURRENT_LAYER(p)          (&((p)->layer[(p)->cur_layer]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Pin Configuration */

struct pin_config_s
{
  uint8_t num;
  uint8_t signal;
};

/* Hardware Configuration */

struct esp32s3_lcd_config_s
{
  struct pin_config_s pins_config[ESP32S3_LCD_PINS];
};

/* LCD General Layer information */

struct esp32s3_layer_s
{
  /* DMA descriptor(s) */

  struct esp32s3_dmadesc_s dmadesc[ESP32S3_LCD_DMADESC_NUM];

  /* DMA framebuffer memory */

  uint8_t *framebuffer;
};

/* This structure provides the overall state of the LCD */

struct esp32s3_lcd_s
{
  int ref;

  /* Layer information */

  struct esp32s3_layer_s layer[ESP32S3_LCD_LAYERS];

  uint8_t cur_layer;              /* Current layer number */

  int cpuint;                     /* CPU interrupt assigned to this LCD */
  uint8_t cpu;                    /* CPU ID */
  int32_t dma_channel;            /* DMA channel */

  spinlock_t lock;                /* Device specific lock. */

  /* Debug stuff */

#ifdef CONFIG_ESP32S3_LCD_REGDEBUG
  bool wrlast;            /* True: Last access was a write */
  uintptr_t addrlast;     /* Last address accessed */
  uint32_t vallast;       /* Last value read or written */
  int ntimes;             /* Number of consecutive accesses */
#endif
};

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern void esp_rom_delay_us(uint32_t us);
extern void cache_writeback_addr(void *addr_ptr, uint32_t size);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_ESP32S3_LCD_REGDEBUG
static bool esp32s3_lcd_checkreg(bool wr,
                                 uint32_t regval,
                                 uintptr_t address);
static uint32_t esp32s3_lcd_getreg(uintptr_t addr);
static void esp32s3_lcd_putreg(uintptr_t addr, uint32_t val);
#else
#  define esp32s3_lcd_getreg(addr)      getreg32(addr)
#  define esp32s3_lcd_putreg(addr,val)  putreg32(val,addr)
#endif

/* Frame buffer interface ***************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int esp32s3_lcd_base_getvideoinfo(struct fb_vtable_s *vtable,
                                         struct fb_videoinfo_s *vinfo);
static int esp32s3_lcd_base_getplaneinfo(struct fb_vtable_s *vtable,
                                         int planeno,
                                         struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int esp32s3_lcd_base_updatearea(struct fb_vtable_s *vtable,
                                       const struct fb_area_s *area);
#endif

/* Initialization ***********************************************************/

static void esp32s3_lcd_dmasetup(void);
static void esp32s3_lcd_gpio_config(void);
static void esp32s3_lcd_disable(void);
static void esp32s3_lcd_enable(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes LCD controller configuration */

static const struct esp32s3_lcd_config_s g_lcd_config =
{
  .pins_config =
  {
    { CONFIG_ESP32S3_LCD_PCLK_PIN,   LCD_PCLK_IDX       },
    { CONFIG_ESP32S3_LCD_VSYNC_PIN,  LCD_V_SYNC_IDX     },
    { CONFIG_ESP32S3_LCD_HSYNC_PIN,  LCD_H_SYNC_IDX     },
    { CONFIG_ESP32S3_LCD_HE_PIN,     LCD_H_ENABLE_IDX   },
    { CONFIG_ESP32S3_LCD_DATA0_PIN,  LCD_DATA_OUT0_IDX  },
    { CONFIG_ESP32S3_LCD_DATA1_PIN,  LCD_DATA_OUT1_IDX  },
    { CONFIG_ESP32S3_LCD_DATA2_PIN,  LCD_DATA_OUT2_IDX  },
    { CONFIG_ESP32S3_LCD_DATA3_PIN,  LCD_DATA_OUT3_IDX  },
    { CONFIG_ESP32S3_LCD_DATA4_PIN,  LCD_DATA_OUT4_IDX  },
    { CONFIG_ESP32S3_LCD_DATA5_PIN,  LCD_DATA_OUT5_IDX  },
    { CONFIG_ESP32S3_LCD_DATA6_PIN,  LCD_DATA_OUT6_IDX  },
    { CONFIG_ESP32S3_LCD_DATA7_PIN,  LCD_DATA_OUT7_IDX  },
    { CONFIG_ESP32S3_LCD_DATA8_PIN,  LCD_DATA_OUT8_IDX  },
    { CONFIG_ESP32S3_LCD_DATA9_PIN,  LCD_DATA_OUT9_IDX  },
    { CONFIG_ESP32S3_LCD_DATA10_PIN, LCD_DATA_OUT10_IDX },
    { CONFIG_ESP32S3_LCD_DATA11_PIN, LCD_DATA_OUT11_IDX },
    { CONFIG_ESP32S3_LCD_DATA12_PIN, LCD_DATA_OUT12_IDX },
    { CONFIG_ESP32S3_LCD_DATA13_PIN, LCD_DATA_OUT13_IDX },
    { CONFIG_ESP32S3_LCD_DATA14_PIN, LCD_DATA_OUT14_IDX },
    { CONFIG_ESP32S3_LCD_DATA15_PIN, LCD_DATA_OUT15_IDX }
  }
};

/* This structure provides the overall state of the LCD */

static struct esp32s3_lcd_s g_lcd_priv;

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_base_videoinfo =
{
  .fmt      = ESP32S3_LCD_COLOR_FMT,
  .xres     = CONFIG_ESP32S3_LCD_VRES,
  .yres     = CONFIG_ESP32S3_LCD_HRES,
  .nplanes  = 1
};

/* This structure provides the base layer interface */

static const struct fb_vtable_s g_base_vtable =
{
  .getvideoinfo  = esp32s3_lcd_base_getvideoinfo,
  .getplaneinfo  = esp32s3_lcd_base_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea    = esp32s3_lcd_base_updatearea,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max_common_divisor
 *
 * Description:
 *   Calculate maxium common divisor.
 *
 * Input Parameters:
 *   a - Calculation parameter a
 *   b - Calculation parameter b
 *
 * Returned Value:
 *   Maxium common divisor.
 *
 ****************************************************************************/

static inline uint32_t max_common_divisor(uint32_t a, uint32_t b)
{
  uint32_t c = a % b;

  while (c)
    {
      a = b;
      b = c;
      c = a % b;
    }

  return b;
}

/****************************************************************************
 * Name: esp32s3_lcd_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   wr      - true: write operation; false: read operation
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_LCD_REGDEBUG
static bool esp32s3_lcd_checkreg(bool wr,
                                 uint32_t regval,
                                 uintptr_t address)
{
  struct esp32s3_lcd_s *priv = &g_lcd_priv;

  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lcdinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: esp32s3_lcd_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 * Input Parameters:
 *  address - Regster address
 *
 * Returned Value:
 *  Regster value.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_LCD_REGDEBUG
static uint32_t esp32s3_lcd_getreg(uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (esp32s3_lcd_checkreg(false, regval, address))
    {
      lcdinfo("%" PRIx32 " ->%" PRIx32 "\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: esp32s3_lcd_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 * Input Parameters:
 *  address - Regster address
 *  regval  - Regster value
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_LCD_REGDEBUG
static void esp32s3_lcd_putreg(uintptr_t address, uint32_t regval)
{
  if (esp32s3_lcd_checkreg(true, regval, address))
    {
      lcdinfo("%" PRIx32 " <-%" PRIx32 "\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

/****************************************************************************
 * Name: esp32s3_lcd_base_getvideoinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_VIDEOINFO
 *   Get the videoinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   vinfo  - The videoinfo object
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int esp32s3_lcd_base_getvideoinfo(struct fb_vtable_s *vtable,
                                         struct fb_videoinfo_s *vinfo)
{
  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_base_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: esp32s3_lcd_base_getplaneinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_PLANEINFO
 *   Get the planeinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   pinfo  - the planeinfo object
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int esp32s3_lcd_base_getplaneinfo(struct fb_vtable_s *vtable,
                                         int planeno,
                                         struct fb_planeinfo_s *pinfo)
{
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      struct esp32s3_lcd_s *priv = &g_lcd_priv;
      struct esp32s3_layer_s *layer = CURRENT_LAYER(priv);

      pinfo->display = 0;
      pinfo->fbmem   = (void *)layer->framebuffer;
      pinfo->fblen   = ESP32S3_LCD_FB_SIZE;
      pinfo->stride  = ESP32S3_LCD_STRIDE;
      pinfo->bpp     = ESP32S3_LCD_DATA_BPP;
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: esp32s3_lcd_base_updatearea
 *
 * Description:
 *   Flush data from cache to PSRAM so that LCD DMA can access it.
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   area   - Reference to the overlay area
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_FB_UPDATE
static int esp32s3_lcd_base_updatearea(struct fb_vtable_s *vtable,
                                       const struct fb_area_s *area)
{
  struct esp32s3_lcd_s *priv = &g_lcd_priv;

  cache_writeback_addr(CURRENT_LAYER(priv)->framebuffer,
                       ESP32S3_LCD_FB_SIZE);

  return 0;
}
#endif

/****************************************************************************
 * Name: lcd_interrupt
 *
 * Description:
 *   Start sending next frame to LCD.
 *
 * Input Parameters:
 *   irq     - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg     - Not used
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int IRAM_ATTR lcd_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  struct esp32s3_lcd_s *priv = &g_lcd_priv;
  uint32_t status = esp32s3_lcd_getreg(LCD_CAM_LC_DMA_INT_ST_REG);

  esp32s3_lcd_putreg(LCD_CAM_LC_DMA_INT_CLR_REG, status);
  if (status & LCD_CAM_LCD_VSYNC_INT_ST_M)
    {
      /* Stop TX */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval &= ~LCD_CAM_LCD_START_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_UPDATE_REG_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      /* Clear TX fifo */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_MISC_REG);
      regval |= LCD_CAM_LCD_AFIFO_RESET_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_MISC_REG, regval);

#if ESP32S3_LCD_LAYERS > 1
      priv->cur_layer = (priv->cur_layer + 1) % ESP32S3_LCD_LAYERS;

      esp32s3_dma_load(CURRENT_LAYER(priv)->dmadesc,
                       priv->dma_channel,
                       true);
#endif

#ifndef CONFIG_FB_UPDATE
      /* Write framebuffer data from D-cache to PSRAM */

      cache_writeback_addr(CURRENT_LAYER(priv)->framebuffer,
                           ESP32S3_LCD_FB_SIZE);
#endif

      /* Enable DMA TX */

      esp32s3_dma_enable(priv->dma_channel, true);

      /* Update LCD parameters and start TX */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_UPDATE_REG_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_START_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_lcd_dmasetup
 *
 * Description:
 *   Configure the channel DMA
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_dmasetup(void)
{
  struct esp32s3_lcd_s *priv = &g_lcd_priv;

  esp32s3_dma_init();

  priv->dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          10, 1, true);
  DEBUGASSERT(priv->dma_channel >= 0);
  esp32s3_dma_set_ext_memblk(priv->dma_channel,
                             true,
                             ESP32S3_DMA_EXT_MEMBLK_64B);

  for (int i = 0; i < ESP32S3_LCD_LAYERS; i++)
    {
      struct esp32s3_layer_s *layer = &priv->layer[i];

      layer->framebuffer = memalign(64, ESP32S3_LCD_FB_SIZE);
      DEBUGASSERT(layer->framebuffer != NULL);
      memset(layer->framebuffer, 0, ESP32S3_LCD_FB_SIZE);

      esp32s3_dma_setup(layer->dmadesc,
                        ESP32S3_LCD_DMADESC_NUM,
                        layer->framebuffer,
                        ESP32S3_LCD_FB_SIZE,
                        true, priv->dma_channel);
    }
}

/****************************************************************************
 * Name: esp32s3_lcd_gpio_config
 *
 * Description:
 *   Configure GPIO pins for use with the LCD
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_gpio_config(void)
{
  const struct esp32s3_lcd_config_s *config = &g_lcd_config;

  lcdinfo("Configuring pins\n");

  /* Configure each pin */

  for (int i = 0; i < ESP32S3_LCD_PINS; i++)
    {
      const struct pin_config_s *pins_config = &config->pins_config[i];

      esp32s3_configgpio(pins_config->num, OUTPUT);
      esp32s3_gpio_matrix_out(pins_config->num, pins_config->signal, 0, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_lcd_enableclk
 *
 * Description:
 *   Enable LCD clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_enableclk(void)
{
  uint32_t regval;
  uint32_t clk_a;
  uint32_t clk_b;

#if ESP32S3_LCD_CLK_RES != 0
    uint32_t divisor = max_common_divisor(ESP32S3_LCD_CLK_RES,
                                          CONFIG_ESP32S3_LCD_CLOCK_MHZ);
    clk_b = ESP32S3_LCD_CLK_RES / divisor;
    clk_a = CONFIG_ESP32S3_LCD_CLOCK_MHZ / divisor;

    lcdinfo("divisor=%d\n", divisor);
#else
    clk_b = clk_a = 0;
#endif

  lcdinfo("PCLK=%d/(%d + %d/%d)\n", ESP32S3_LCD_CLK_MHZ,
          ESP32S3_LCD_CLK_N, clk_b, clk_a);

  periph_module_enable(PERIPH_LCD_CAM_MODULE);

  regval = (1 << LCD_CAM_LCD_CLKCNT_N_S) |
           LCD_CAM_CLK_EN_M |
           LCD_CAM_LCD_CLK_EQU_SYSCLK_M |
           (ESP32S3_LCD_CLK_SEL << LCD_CAM_LCD_CLK_SEL_S) |
           (ESP32S3_LCD_CLK_N << LCD_CAM_LCD_CLKM_DIV_NUM_S) |
           (clk_a << LCD_CAM_LCD_CLKM_DIV_A_S) |
           (clk_b << LCD_CAM_LCD_CLKM_DIV_B_S);
  esp32s3_lcd_putreg(LCD_CAM_LCD_CLOCK_REG, regval);
}

/****************************************************************************
 * Name: esp32s3_lcd_config
 *
 * Description:
 *   Configure LCD controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_config(void)
{
  uint32_t regval;
  irqstate_t flags;
  struct esp32s3_lcd_s *priv = &g_lcd_priv;

  /* Enable TX done interrupt */

  regval  = esp32s3_lcd_getreg(LCD_CAM_LC_DMA_INT_ENA_REG);
  regval |= LCD_CAM_LCD_VSYNC_INT_ENA_M;
  esp32s3_lcd_putreg(LCD_CAM_LC_DMA_INT_ENA_REG, regval);

  /* Set LCD screem parameters:
   *    1. RGB mode, ouput VSYNC/HSYNC/DE signal
   *    2. VT height
   *    3. VA height
   *    4. HB front
   *    5. HT width
   *    6. HA width
   *    7. VB front
   *    8. VSYNC width
   *    9. HSYNC width
   */

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_CTRL_REG);
  regval |= LCD_CAM_LCD_RGB_MODE_EN_M |
            (ESP32S3_LCD_VT_HIGHT << LCD_CAM_LCD_VT_HEIGHT_S) |
            (ESP32S3_LCD_VA_HIGHT << LCD_CAM_LCD_VA_HEIGHT_S) |
            (ESP32S3_LCD_HB_FRONT << LCD_CAM_LCD_HB_FRONT_S);
  esp32s3_lcd_putreg(LCD_CAM_LCD_CTRL_REG, regval);

  regval = (ESP32S3_LCD_HT_WIDTH << LCD_CAM_LCD_HT_WIDTH_S) |
           (ESP32S3_LCD_HA_WIDTH << LCD_CAM_LCD_HA_WIDTH_S) |
           (ESP32S3_LCD_VA_FRONT << LCD_CAM_LCD_VB_FRONT_S);
  esp32s3_lcd_putreg(LCD_CAM_LCD_CTRL1_REG, regval);

  regval = (ESP32S3_LCD_HSYNC_WIDTH << LCD_CAM_LCD_HSYNC_WIDTH_S) |
           (ESP32S3_LCD_VSYNC_WIDTH << LCD_CAM_LCD_VSYNC_WIDTH_S) |
           LCD_CAM_LCD_HSYNC_IDLE_POL_M |
           LCD_CAM_LCD_HS_BLANK_EN_M |
           LCD_CAM_LCD_VSYNC_IDLE_POL_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_CTRL2_REG, regval);

  /* Configure output mode:
   *    1. always output
   *    2. 16-bit word
   *    3. LCD mode
   *    4. 3-bit dummy
   */

  regval = LCD_CAM_LCD_ALWAYS_OUT_EN_M |
           LCD_CAM_LCD_2BYTE_EN_M |
           LCD_CAM_LCD_DOUT_M |
           (3 << LCD_CAM_LCD_DUMMY_CYCLELEN_S);
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

  regval = LCD_CAM_LCD_AFIFO_THRESHOLD_NUM_M |
           LCD_CAM_LCD_BK_EN_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_MISC_REG, regval);

  /* Update registers */

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
  regval |= LCD_CAM_LCD_UPDATE_REG_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

  /* Set GDMA */

  esp32s3_lcd_dmasetup();

  /* Configure interrupt */

  regval = LCD_CAM_LCD_VSYNC_INT_ENA_M;
  esp32s3_lcd_putreg(LCD_CAM_LC_DMA_INT_ENA_REG, regval);

  flags = spin_lock_irqsave(&priv->lock);

  priv->cpu = up_cpu_index();
  priv->cpuint = esp32s3_setup_irq(priv->cpu,
                                   ESP32S3_PERIPH_LCD_CAM,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(priv->cpuint >= 0);

  DEBUGASSERT(irq_attach(ESP32S3_IRQ_LCD_CAM, lcd_interrupt, priv) == 0);

  spin_unlock_irqrestore(&priv->lock, flags);

  up_enable_irq(ESP32S3_IRQ_LCD_CAM);
}

/****************************************************************************
 * Name: esp32s3_lcd_enable
 *
 * Description:
 *   Enable LCD display.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_enable(void)
{
  uint32_t regval;
  struct esp32s3_lcd_s *priv = &g_lcd_priv;
  struct esp32s3_layer_s *layer = CURRENT_LAYER(priv);

  esp32s3_dma_load(layer->dmadesc, priv->dma_channel, true);
  esp32s3_dma_enable(priv->dma_channel, true);

  /* Delay 1 microsecond to wait the DMA start */

  esp_rom_delay_us(1);

  /* Update LCD parameters before start */

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
  regval |= LCD_CAM_LCD_UPDATE_REG_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
  regval |= LCD_CAM_LCD_START_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);
}

/****************************************************************************
 * Name: esp32s3_lcd_disable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32s3_lcd_disable(void)
{
  uint32_t regval;

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
  regval &= ~LCD_CAM_LCD_START_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

  /* Update LCD parameters before start */

  regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
  regval |= LCD_CAM_LCD_UPDATE_REG_M;
  esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  lcdinfo("Entry\n");

  if (g_lcd_priv.ref++ != 0)
    {
      return 0;
    }

  DEBUGASSERT(display == 0);

  /* Disable the LCD */

  esp32s3_lcd_disable();

  /* Configure GPIO pins */

  esp32s3_lcd_gpio_config();

  /* Enable the LCD peripheral clock */

  esp32s3_lcd_enableclk();

  /* Configure LCD controller */

  esp32s3_lcd_config();

  /* And turn the LCD on */

  esp32s3_lcd_enable();

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  DEBUGASSERT(display == 0);

  lcdinfo("vplane: %d\n", vplane);
  if (vplane == 0)
    {
      return (struct fb_vtable_s *)&g_base_vtable;
    }
  else
    {
      return NULL;
    }
}
