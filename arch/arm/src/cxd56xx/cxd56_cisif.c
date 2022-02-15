/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cisif.c
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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/chip/cisif.h>
#include <nuttx/video/imgdata.h>
#include "arm_arch.h"

#include "cxd56_clock.h"
#include "hardware/cxd56_cisif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To see the interrupt timing of Vsync */

/* #define CISIF_INTR_TRACE */

/* #define CISIF_DBG_CONTI_CAP */

#define YUV_VSIZE_STEP (1)
#define YUV_HSIZE_STEP (1)
#define YUV_VSIZE_MIN  (64)
#define YUV_HSIZE_MIN  (96)
#define YUV_VSIZE_MAX  (360)
#define YUV_HSIZE_MAX  (480)

#define JPG_VSIZE_STEP (1)
#define JPG_HSIZE_STEP (1)
#define JPG_VSIZE_MIN  (64)
#define JPG_HSIZE_MIN  (96)
#define JPG_VSIZE_MAX  (1944)
#define JPG_HSIZE_MAX  (2592)

#define CISIF_FMT_MIN  (1)
#define CISIF_FMT_MAX  (3)

#define JPG_INT_ALL   (JPG_ERR_STATUS_INT | \
                       JPG_MEM_OVF_INT    | \
                       JPG_FIFO_OVF_INT   | \
                       JPG_AXI_TRERR_INT  | \
                       JPG_MARKER_ERR_INT | \
                       JPG_AXI_TRDN_INT)

#define YCC_INT_ALL   (YCC_MEM_OVF_INT    | \
                       YCC_FIFO_OVF_INT   | \
                       YCC_AXI_TRERR_INT  | \
                       YCC_MARKER_ERR_INT | \
                       SIZE_UNDER_INT     | \
                       SIZE_OVER_INT      | \
                       YCC_AXI_TRDN_INT)

/* YUV data size with frame v * h */

#define YUV_SIZE(v, h) (v * h * 2)

/* Check Buffer address alignment */

#define CISIF_BUFADDR_ALIGNMENT            (32)
#define ILLEGAL_BUFADDR_ALIGNMENT(addr)    (((addr) == NULL) ||  \
                                            (((uint32_t)(addr) % \
                                              CISIF_BUFADDR_ALIGNMENT) != 0))

#ifdef CONFIG_CXD56_CISIF_DEBUG
#define ciferr    _err
#define cifwarn   _warn
#define cifinfo   _info
#else
#define ciferr(x...)
#define cifwarn(x...)
#define cifinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum state_e
{
  STATE_STANDBY,
  STATE_READY,
  STATE_CAPTURE,
};

typedef enum state_e state_t;

typedef void (*intc_func_table)(uint8_t code);

typedef void (*notify_callback_t)(uint8_t code,
                                  uint32_t size,
                                  uint32_t addr);
typedef void (*comp_callback_t)(uint8_t code,
                                uint32_t size,
                                uint32_t addr);

struct cisif_yuv_param_s
{
  uint16_t          hsize;
  uint16_t          vsize;
  uint32_t          notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_yuv_param_s cisif_yuv_param_t;

struct cisif_jpg_param_s
{
  uint32_t notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_jpg_param_s cisif_jpg_param_t;

struct cisif_param_s
{
  uint32_t           format;
  cisif_yuv_param_t  yuv_param;
  cisif_jpg_param_t  jpg_param;
};

typedef struct cisif_param_s cisif_param_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static state_t g_state = STATE_STANDBY;
static uint32_t g_storage_addr = 0;

static notify_callback_t g_jpg_notify_callback_func;
static notify_callback_t g_ycc_notify_callback_func;

static bool     g_jpgint_receive;
static bool     g_errint_receive;

#ifdef CISIF_INTR_TRACE
static uint32_t g_cisif_vint_count = 0;
static uint32_t g_cisif_vint_count_max = 0;
static uint32_t g_cisif_time_start;
static uint32_t g_cisif_time_stop;
#endif

static imgdata_capture_t g_cxd56_cisif_complete_capture;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cisif_vs_int(uint8_t code);
static void cisif_ycc_axi_trdn_int(uint8_t code);
static void cisif_ycc_nstorage_int(uint8_t code);
static void cisif_jpg_axi_trdn_int(uint8_t code);
static void cisif_jpg_nstorage_int(uint8_t code);
static void cisif_ycc_err_int(uint8_t code);
static void cisif_jpg_err_int(uint8_t code);

static void     cisif_reg_write(uint16_t reg, uint32_t val);
static uint32_t cisif_reg_read(uint16_t reg);

static int cisif_check_param(cisif_param_t *p);
static int cisif_set_yuv_param(cisif_yuv_param_t *p);
static int cisif_set_jpg_param(cisif_jpg_param_t *p);

static int cisif_set_yuv_sarea(uint8_t *addr, uint32_t size);
static int cisif_set_jpg_sarea(uint8_t *addr, uint32_t size);
static int cisif_set_intlev_sarea(uint8_t *addr,
                                  uint32_t total_size,
                                  uint32_t yuv_size);
static int cisif_intc_handler(int irq, FAR void *context, FAR void *arg);

/* video image data operations */

static int cxd56_cisif_init(void);
static int cxd56_cisif_uninit(void);
static int cxd56_cisif_validate_frame_setting
             (uint8_t nr_datafmt,
              FAR imgdata_format_t *datafmt,
              FAR imgdata_interval_t *interval);
static int cxd56_cisif_start_capture
             (uint8_t nr_datafmt,
              FAR imgdata_format_t *datafmt,
              FAR imgdata_interval_t *interval,
              imgdata_capture_t callback);
static int cxd56_cisif_stop_capture(void);
static int cxd56_cisif_validate_buf(uint8_t *addr, uint32_t size);
static int cxd56_cisif_set_buf(uint8_t *addr, uint32_t size);

const intc_func_table g_intcomp_func[] =
  {
    cisif_vs_int,            /* VS_INT */
    NULL,                    /* EOY_INT */
    NULL,                    /* SOY_INT */
    NULL,                    /* EOI_INT */
    NULL,                    /* SOI_INT */
    NULL,                    /* YCC_VACT_END_INT */
    NULL,                    /* JPG_VACT_END_INT */
    cisif_ycc_axi_trdn_int,  /* YCC_AXI_TRDN_INT */
    cisif_ycc_nstorage_int,  /* YCC_NSTORAGE_INT */
    NULL,                    /* YCC_DAREA_END_INT */
    cisif_jpg_axi_trdn_int,  /* JPG_AXI_TRDN_INT */
    cisif_jpg_nstorage_int,  /* JPG_NSTORAGE_INT */
    NULL,                    /* JPG_DAREA_END_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    NULL,                    /* VLATCH_INT */
    cisif_ycc_err_int,       /* SIZE_OVER_INT */
    cisif_ycc_err_int,       /* SIZE_UNDER_INT */
    cisif_ycc_err_int,       /* YCC_MARKER_ERR_INT */
    cisif_ycc_err_int,       /* YCC_AXI_TRERR_INT */
    cisif_ycc_err_int,       /* YCC_FIFO_OVF_INT */
    cisif_ycc_err_int,       /* YCC_MEM_OVF_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    cisif_jpg_err_int,       /* JPG_MARKER_ERR_INT */
    cisif_jpg_err_int,       /* JPG_AXI_TRERR_INT */
    cisif_jpg_err_int,       /* JPG_FIFO_OVF_INT */
    cisif_jpg_err_int,       /* JPG_MEM_OVF_INT */
    cisif_jpg_err_int,       /* JPG_ERR_STATUS_INT */
  };

const struct imgdata_ops_s g_cxd56_cisif_ops =
  {
    .init                   = cxd56_cisif_init,
    .uninit                 = cxd56_cisif_uninit,
    .validate_buf           = cxd56_cisif_validate_buf,
    .set_buf                = cxd56_cisif_set_buf,
    .validate_frame_setting = cxd56_cisif_validate_frame_setting,
    .start_capture          = cxd56_cisif_start_capture,
    .stop_capture           = cxd56_cisif_stop_capture,
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CISIF_INTR_TRACE
static uint64_t cisif_get_msec_time(void)
{
  struct timespec tp;

  if (clock_systime_timespec(&tp) < 0)
    {
      return 0;
    }

  return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}

static void cisif_trace_time_start(void)
{
  g_cisif_time_start = (uint32_t)cisif_get_msec_time();
}

static void cisif_trace_time_stop(char *str)
{
  g_cisif_time_stop = (uint32_t)cisif_get_msec_time();
  printf("%s:%d[ms]\n", str, (uint32_t)(g_cisif_time_stop -
                               g_cisif_time_start));
}

void cisif_intrtrace_start(int max)
{
  g_cisif_vint_count_max = max;
  g_cisif_vint_count = 0;
  cisif_trace_time_start();
}
#endif

/****************************************************************************
 * cisif_vs_int
 ****************************************************************************/

static void cisif_vs_int(uint8_t code)
{
#ifdef CISIF_INTR_TRACE
  if (g_cisif_vint_count < g_cisif_vint_count_max)
    {
      cisif_trace_time_stop("cisif_vs_int");
      cisif_trace_time_start();
      g_cisif_vint_count++;
    }
  else
    {
      g_cisif_vint_count_max = 0;
    }
#endif

  switch (g_state)
    {
      case STATE_STANDBY:
        cifinfo("invalid state\n");
        break;

      case STATE_READY:
        g_errint_receive = false;
        break;

      case STATE_CAPTURE:
        g_errint_receive = false;
        break;

      default:
        cifinfo("invalid state\n");
        break;
    }
}

/****************************************************************************
 * cisif_callback_for_intlev
 ****************************************************************************/

static void cisif_callback_for_intlev(uint8_t code)
{
  uint32_t      size;
  uint32_t      yuv_size;

  if (!g_jpgint_receive)
    {
      /* In either YUV or JPEG is not received,
       * wait receiving.
       */

      g_jpgint_receive = true;
      return;
    }

  /* Read received data size */

  yuv_size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  size     = yuv_size + cisif_reg_read(CISIF_JPG_DSTRG_CONT);

  /* Notify and get next addr */

  g_cxd56_cisif_complete_capture(0, size);

  g_jpgint_receive = false;

  cisif_reg_write(CISIF_EXE_CMD, 1);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);

  return;
}

/****************************************************************************
 * cisif_ycc_axi_trdn_int
 ****************************************************************************/

static void cisif_ycc_axi_trdn_int(uint8_t code)
{
  uint32_t size;
  uint32_t cisif_mode;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_axi_trdn_int");
#endif

  if (g_errint_receive)
    {
      /* In error occurred case in the same frame, ignore */

      cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
      return;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);
  if (cisif_mode == MODE_INTLEV_TRS_EN)
    {
      /* In JPEG + YUV format case */

      cisif_callback_for_intlev(code);
    }
  else
    {
      size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
      g_cxd56_cisif_complete_capture(0, size);
      cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
    }
}

/****************************************************************************
 * cisif_ycc_nstorage_int
 ****************************************************************************/

static void cisif_ycc_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_ycc_notify_callback_func(0, size, g_storage_addr);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, size);
}

/****************************************************************************
 * cisif_jpg_axi_trdn_int
 ****************************************************************************/

static void cisif_jpg_axi_trdn_int(uint8_t code)
{
  uint32_t size;
  uint32_t cisif_mode;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_axi_trdn_int");
#endif

  if (g_errint_receive)
    {
      /* In error occurred case in the same frame, ignore */

      cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
      return;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);

  if (cisif_mode == MODE_INTLEV_TRS_EN)
    {
      /* In JPEG + YUV format case */

      cisif_callback_for_intlev(code);
    }
  else
    {
      size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
      g_cxd56_cisif_complete_capture(0, size);
      cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
    }
}

/****************************************************************************
 * cisif_jpg_nstorage_int
 ****************************************************************************/

static void cisif_jpg_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);

  g_jpg_notify_callback_func(0, size, g_storage_addr);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, size);
}

/****************************************************************************
 * cisif_ycc_err_int
 ****************************************************************************/

static void cisif_ycc_err_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_err_int");
#endif

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_cxd56_cisif_complete_capture(code, size);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_jpg_err_int
 ****************************************************************************/

static void cisif_jpg_err_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_err_int");
#endif

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  g_cxd56_cisif_complete_capture(code, size);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_intc_handler
 ****************************************************************************/

static int cisif_intc_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t value;
  uint32_t enable;
  uint8_t  index;

  value = cisif_reg_read(CISIF_INTR_STAT);
  cisif_reg_write(CISIF_INTR_CLEAR, value & ALL_CLEAR_INT);
  cifinfo("int stat %08x\n", value);

  enable = cisif_reg_read(CISIF_INTR_ENABLE);
  value = (value & enable);

  for (index = 0;
       index < sizeof(g_intcomp_func) / sizeof(g_intcomp_func[0]);
       index++)
    {
      if ((value & (1 << index)) != 0)
        {
          g_intcomp_func[index](index);
        }
    }

  return OK;
}

/****************************************************************************
 * cisif_reg_write
 ****************************************************************************/

static void cisif_reg_write(uint16_t reg, uint32_t val)
{
  putreg32(val, CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * cisif_reg_read
 ****************************************************************************/

static uint32_t cisif_reg_read(uint16_t reg)
{
  return getreg32(CXD56_CISIF_BASE + reg);
}

static bool is_uncompressed(uint32_t fmt)
{
  bool ret = false;

  if ((fmt == IMGDATA_PIX_FMT_UYVY) ||
      (fmt == IMGDATA_PIX_FMT_RGB565))
    {
      ret = true;
    }

  return ret;
}

/****************************************************************************
 * cisif_check_param
 ****************************************************************************/

static int cisif_check_param(cisif_param_t *p)
{
  if (p == NULL)
    {
      return -EINVAL;
    }

  switch (p->format)
    {
      case IMGDATA_PIX_FMT_UYVY:
      case IMGDATA_PIX_FMT_RGB565:
      case IMGDATA_PIX_FMT_JPEG:
      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:
        break;

      default:
        return -EINVAL;
    }

  if (p->format != IMGDATA_PIX_FMT_JPEG)
    {
      if (p->yuv_param.hsize < YUV_HSIZE_MIN ||
          p->yuv_param.hsize > YUV_HSIZE_MAX ||
          p->yuv_param.vsize < YUV_VSIZE_MIN ||
          p->yuv_param.vsize > YUV_VSIZE_MAX)
        {
          return -EINVAL;
        }

      if (p->yuv_param.notify_func != NULL)
        {
          if (p->yuv_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  if (!is_uncompressed(p->format))
    {
      if (p->jpg_param.notify_func != NULL)
        {
          if (p->jpg_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * cisif_set_yuvparam
 ****************************************************************************/

static int cisif_set_yuv_param(cisif_yuv_param_t *p)
{
  uint32_t act_size = 0;

  act_size = (p->vsize & 0x1ff) << 16;
  act_size |= p->hsize & 0x1ff;

  cisif_reg_write(CISIF_ACT_SIZE, act_size);
  cisif_reg_write(CISIF_CIS_SIZE, act_size);

  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_NSTRG_SIZE,
                  (p->notify_size & 0xffffffe0));

  g_ycc_notify_callback_func = p->notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_yuvsarea
 ****************************************************************************/

static int cisif_set_yuv_sarea(uint8_t *addr, uint32_t size)
{
  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_DAREA_SIZE, (size & 0xffffffe0));
  cisif_reg_write(CISIF_YCC_START_ADDR, CXD56_PHYSADDR(addr));

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_param
 ****************************************************************************/

static int cisif_set_jpg_param(cisif_jpg_param_t *p)
{
  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_NSTRG_SIZE, (p->notify_size
                                               & 0xffffffe0));

  g_jpg_notify_callback_func = p->notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_jpg_sarea(uint8_t *addr, uint32_t size)
{
  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_DAREA_SIZE, (size & 0xffffffe0));
  cisif_reg_write(CISIF_JPG_START_ADDR, CXD56_PHYSADDR(addr));

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_intlev_sarea(uint8_t *addr,
                                  uint32_t total_size,
                                  uint32_t yuv_size)
{
  if (total_size < yuv_size)
    {
      return -EINVAL;
    }

  /* Set for YUV */

  cisif_set_yuv_sarea(addr, yuv_size);

  /* Set for JPEG */

  cisif_set_jpg_sarea(addr + yuv_size, total_size - yuv_size);

  return OK;
}

/****************************************************************************
 * cisif_chk_jpgfrmsize
 ****************************************************************************/

static int cisif_chk_jpgfrmsize(int w, int h)
{
  if ((w < JPG_HSIZE_MIN) ||
      (w > JPG_HSIZE_MAX))
    {
      return -EINVAL;
    }

  if ((h < JPG_VSIZE_MIN) ||
      (h > JPG_VSIZE_MAX))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * cisif_chk_yuvfrmsize
 ****************************************************************************/

static int cisif_chk_yuvfrmsize(int w, int h)
{
  if ((w < YUV_HSIZE_MIN) ||
      (w > YUV_HSIZE_MAX))
    {
      return -EINVAL;
    }

  if ((h < YUV_VSIZE_MIN) ||
      (h > YUV_VSIZE_MAX))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * cxd56_cisif_init
 ****************************************************************************/

static int cxd56_cisif_init(void)
{
  if (g_state != STATE_STANDBY)
    {
      return -EPERM;
    }

  /* enable CISIF clock */

  cxd56_img_cisif_clock_enable();

  /* disable CISIF interrupt */

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* attach interrupt handler */

  irq_attach(CXD56_IRQ_CISIF, cisif_intc_handler, NULL);

  /* enable CISIF irq  */

  up_enable_irq(CXD56_IRQ_CISIF);

#ifdef CISIF_INTR_TRACE
  cisif_reg_write(CISIF_INTR_ENABLE, VS_INT);
#endif

  g_state = STATE_READY;
  return OK;
}

/****************************************************************************
 * cxd56_cisif_uninit
 ****************************************************************************/

static int cxd56_cisif_uninit(void)
{
  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  /* disable CISIF irq  */

  up_disable_irq(CXD56_IRQ_CISIF);

  /* detach interrupt handler */

  irq_detach(CXD56_IRQ_CISIF);

  /* disable CISIF interrupt */

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* disable CISIF clock */

  cxd56_img_cisif_clock_disable();

  g_state = STATE_STANDBY;
  return OK;
}

/****************************************************************************
 * cxd56_cisif_start_capture
 ****************************************************************************/

static int cxd56_cisif_start_capture
             (uint8_t nr_fmt,
              FAR imgdata_format_t *fmt,
              FAR imgdata_interval_t *interval,
              imgdata_capture_t callback)
{
  cisif_param_t param =
    {
      0
    };

  cisif_yuv_param_t *yuv = &param.yuv_param;
  cisif_jpg_param_t *jpg = &param.jpg_param;

  uint32_t mode;
  uint32_t interrupts = VS_INT;
  int ret;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  param.format = fmt[IMGDATA_FMT_MAIN].pixelformat;
  if (param.format != IMGDATA_PIX_FMT_JPEG)
    {
      if (is_uncompressed(param.format))
        {
          yuv->hsize = fmt[IMGDATA_FMT_MAIN].width;
          yuv->vsize = fmt[IMGDATA_FMT_MAIN].height;
        }
      else
        {
          yuv->hsize = fmt[IMGDATA_FMT_SUB].width;
          yuv->vsize = fmt[IMGDATA_FMT_SUB].height;
        }
    }

  ret = cisif_check_param(&param);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  switch (param.format)
    {
      case IMGDATA_PIX_FMT_UYVY:
      case IMGDATA_PIX_FMT_RGB565:

        cisif_set_yuv_param(yuv);

        mode = MODE_YUV_TRS_EN;
        interrupts |= YCC_INT_ALL;
        break;

      case IMGDATA_PIX_FMT_JPEG:
        cisif_set_jpg_param(jpg);

        mode = MODE_JPG_TRS_EN;
        interrupts |= JPG_INT_ALL;
        break;

      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:
        cisif_set_yuv_param(yuv);
        cisif_set_jpg_param(jpg);

        mode = MODE_INTLEV_TRS_EN;
        interrupts |= YCC_INT_ALL | JPG_INT_ALL;
        g_jpgint_receive = false;
        break;

      default:
        return -EINVAL;
    }

  g_cxd56_cisif_complete_capture = callback;

  g_state = STATE_CAPTURE;

  if (g_ycc_notify_callback_func != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_jpg_notify_callback_func != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_MODE, mode);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

  cisif_reg_write(CISIF_DIN_ENABLE, 1);

  return OK;
}

static int cxd56_cisif_stop_capture(void)
{
  g_state = STATE_READY;
  cisif_reg_write(CISIF_DIN_ENABLE, 0);
  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

static int cxd56_cisif_validate_buf(uint8_t *addr, uint32_t size)
{
  if (ILLEGAL_BUFADDR_ALIGNMENT(addr) ||
      size == 0)
    {
      return -EINVAL;
    }

  return OK;
}

static int cxd56_cisif_set_buf(uint8_t *addr, uint32_t size)
{
  int      ret;
  uint32_t mode;
  uint32_t regval;
  uint16_t w;
  uint16_t h;

  ret = cxd56_cisif_validate_buf(addr, size);
  if (ret != OK)
    {
      return ret;
    }

  mode = cisif_reg_read(CISIF_MODE);

  switch (mode)
    {
      case MODE_YUV_TRS_EN:
        ret = cisif_set_yuv_sarea(addr, size);
        break;

      case MODE_JPG_TRS_EN:
        ret = cisif_set_jpg_sarea(addr, size);
        break;

      default: /* MODE_INTLEV_TRS_EN */

        /* Get YUV frame size information */

        regval =  cisif_reg_read(CISIF_ACT_SIZE);
        h = (regval >> 16) & 0x1ff;
        w = regval & 0x01ff;

        ret = cisif_set_intlev_sarea(addr,
                                     size,
                                     YUV_SIZE(w, h));
        break;
    }

  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_EXE_CMD, 1);
  g_storage_addr = (uint32_t)addr;

  return ret;
}

static int cxd56_cisif_validate_frame_setting
             (uint8_t nr_datafmt,
              FAR imgdata_format_t *datafmt,
              FAR imgdata_interval_t *interval)
{
  int ret = OK;

  if ((nr_datafmt < CISIF_FMT_MIN) || (nr_datafmt > CISIF_FMT_MAX))
    {
      return -EINVAL;
    }

  switch (datafmt[IMGDATA_FMT_MAIN].pixelformat)
    {
      case IMGDATA_PIX_FMT_UYVY:                /* YUV 4:2:2 */
      case IMGDATA_PIX_FMT_RGB565:              /* RGB565 */

        ret = cisif_chk_yuvfrmsize(datafmt[IMGDATA_FMT_MAIN].width,
                                   datafmt[IMGDATA_FMT_MAIN].height);
        break;

      case IMGDATA_PIX_FMT_JPEG:                /* JPEG */

        ret = cisif_chk_jpgfrmsize(datafmt[IMGDATA_FMT_MAIN].width,
                                   datafmt[IMGDATA_FMT_MAIN].height);
        break;

      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:    /* JPEG + YUV 4:2:2 */

        if ((nr_datafmt == 2) &&
            !is_uncompressed(datafmt[IMGDATA_FMT_SUB].pixelformat))
          {
            /* Unsupported pixel format */

            return -EINVAL;
          }

        ret = cisif_chk_jpgfrmsize(datafmt[IMGDATA_FMT_MAIN].width,
                                   datafmt[IMGDATA_FMT_MAIN].height);
        if (ret != OK)
          {
            return ret;
          }

        if (nr_datafmt == 2)
          {
            ret = cisif_chk_yuvfrmsize
                    (datafmt[IMGDATA_FMT_SUB].width,
                     datafmt[IMGDATA_FMT_SUB].height);
          }

        break;

      default: /* Unsupported pixel format */

        return -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * cxd56_cisif_initialize
 ****************************************************************************/

int cxd56_cisif_initialize(void)
{
  imgdata_register(&g_cxd56_cisif_ops);
  return OK;
}

