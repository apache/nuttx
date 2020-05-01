/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cisif.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <nuttx/video/video.h>

#include "arm_arch.h"

#include "cxd56_clock.h"
#include "hardware/cxd56_cisif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To see the interrupt timing of Vsync */

/* #define CISIF_INTR_TRACE */

/* #define CISIF_DBG_CONTI_CAP */

#define YUV_VSIZE_MIN (64)
#define YUV_HSIZE_MIN (96)
#define YUV_VSIZE_MAX (360)
#define YUV_HSIZE_MAX (480)

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static state_t g_state = STATE_STANDBY;
static uint32_t g_storage_addr = 0;

notify_callback_t g_jpg_notify_callback_func;
notify_callback_t g_ycc_notify_callback_func;
comp_callback_t   g_comp_callback_func;

static bool     g_jpgint_receive;
static bool     g_errint_receive;

#ifdef CISIF_INTR_TRACE
static uint32_t g_cisif_vint_count = 0;
static uint32_t g_cisif_vint_count_max = 0;
static uint32_t g_cisif_time_start;
static uint32_t g_cisif_time_stop;
#endif

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
static int cisif_set_yuv_param(cisif_param_t *p);
static int cisif_set_jpg_param(cisif_param_t *p);

static int cisif_check_sarea(void *s);
static int cisif_set_yuv_sarea(void *s);
static int cisif_set_jpg_sarea(void *s);
static int cisif_set_intlev_sarea(void *s, uint32_t yuv_size);

int cisif_intc_handler(int irq, FAR void *context, FAR void *arg);

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CISIF_INTR_TRACE
static uint64_t cisif_get_msec_time(void)
{
  struct timespec tp;

  if (clock_gettime(CLOCK_REALTIME, &tp))
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

  g_comp_callback_func(0, size, g_storage_addr);

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
      g_comp_callback_func(0, size, g_storage_addr);
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
      g_comp_callback_func(0, size, g_storage_addr);
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
  g_comp_callback_func(code, size, g_storage_addr);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_jpg_err_int
 ****************************************************************************/

static void cisif_jpg_err_int(uint8_t code)
{
  uint32_t size;
  uint32_t addr;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_err_int");
#endif

  addr = g_storage_addr;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  g_comp_callback_func(code, size, addr);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_intc_handler
 ****************************************************************************/

int cisif_intc_handler(int irq, FAR void *context, FAR void *arg)
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

/****************************************************************************
 * cisif_check_param
 ****************************************************************************/

static int cisif_check_param(cisif_param_t *p)
{
  if (p == NULL)
    {
      return -EINVAL;
    }

  if (p->comp_func == NULL)
    {
      return -EINVAL;
    }

  switch (p->format)
    {
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_JPEG:
      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        break;

      default:
        return -EINVAL;
    }

  if (p->format != V4L2_PIX_FMT_JPEG)
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

  if (p->format != V4L2_PIX_FMT_UYVY)
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
 * cisif_check_sarea
 ****************************************************************************/

static int cisif_check_sarea(void *s)
{
  if (s == NULL)
    {
      return -EINVAL;
    }

  cisif_sarea_t *ss = (cisif_sarea_t *)s;
  if (ILLEGAL_BUFADDR_ALIGNMENT(ss->strg_addr) ||
      ss->strg_size == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * cisif_set_yuvparam
 ****************************************************************************/

static int cisif_set_yuv_param(cisif_param_t *p)
{
  uint32_t act_size = 0;

  act_size = (p->yuv_param.vsize & 0x1ff) << 16;
  act_size |= p->yuv_param.hsize & 0x1ff;

  cisif_reg_write(CISIF_ACT_SIZE, act_size);
  cisif_reg_write(CISIF_CIS_SIZE, act_size);

  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_NSTRG_SIZE, (p->yuv_param.notify_size
                                                 & 0xffffffe0));

  g_ycc_notify_callback_func = p->yuv_param.notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_yuvsarea
 ****************************************************************************/

static int cisif_set_yuv_sarea(void *s)
{
  cisif_sarea_t *ss = (cisif_sarea_t *)s;

  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_DAREA_SIZE, (ss->strg_size & 0xffffffe0));
  cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)ss->strg_addr);

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_param
 ****************************************************************************/

static int cisif_set_jpg_param(cisif_param_t *p)
{
  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_NSTRG_SIZE, (p->jpg_param.notify_size
                                               & 0xffffffe0));

  g_jpg_notify_callback_func = p->jpg_param.notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_jpg_sarea(void *s)
{
  cisif_sarea_t *ss = (cisif_sarea_t *)s;

  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_DAREA_SIZE, (ss->strg_size & 0xffffffe0));
  cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)ss->strg_addr);

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_intlev_sarea(void *s, uint32_t yuv_size)
{
  cisif_sarea_t *sarea = (cisif_sarea_t *)s;
  cisif_sarea_t sarea_int;

  if (sarea->strg_size < yuv_size)
    {
      return -EINVAL;
    }

  /* Set for YUV */

  sarea_int.strg_addr = sarea->strg_addr;
  sarea_int.strg_size = yuv_size;
  cisif_set_yuv_sarea(&sarea_int);

  /* Set for JPEG */

  sarea_int.strg_addr = sarea->strg_addr + yuv_size;
  sarea_int.strg_size = sarea->strg_size - yuv_size;

  cisif_set_jpg_sarea(&sarea_int);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * cxd56_cisifinit
 ****************************************************************************/

int cxd56_cisifinit(void)
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
 * cxd56_cisiffinalize
 ****************************************************************************/

int cxd56_cisiffinalize(void)
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
 * cxd56_cisifstartcapturing
 ****************************************************************************/

int cxd56_cisifstartcapture(
  cisif_param_t *param,
  cisif_sarea_t *sarea)
{
  uint32_t cisif_mode;
  uint32_t interrupts = VS_INT;
  int ret;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  ret = cisif_check_param(param);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  ret = cisif_check_sarea(sarea);
  if (ret != OK)
    {
      return ret;
    }

  switch (param->format)
    {
      case V4L2_PIX_FMT_UYVY:
        cisif_set_yuv_param(param);
        cisif_set_yuv_sarea(sarea);

        cisif_mode = MODE_YUV_TRS_EN;
        interrupts |= YCC_INT_ALL;
        break;

      case V4L2_PIX_FMT_JPEG:
        cisif_set_jpg_param(param);
        cisif_set_jpg_sarea(sarea);

        cisif_mode = MODE_JPG_TRS_EN;
        interrupts |= JPG_INT_ALL;
        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        cisif_set_yuv_param(param);
        cisif_set_jpg_param(param);

        cisif_set_intlev_sarea(sarea,
                               YUV_SIZE(param->yuv_param.vsize,
                               param->yuv_param.hsize));

        cisif_mode = MODE_INTLEV_TRS_EN;
        interrupts |= YCC_INT_ALL | JPG_INT_ALL;
        g_jpgint_receive = false;
        break;

      default:
        return -EINVAL;
    }

  g_comp_callback_func = param->comp_func;
  g_storage_addr       = (uint32_t)sarea->strg_addr;

  g_state = STATE_CAPTURE;

  if (g_ycc_notify_callback_func != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_jpg_notify_callback_func != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_MODE, cisif_mode);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

  cisif_reg_write(CISIF_DIN_ENABLE, 1);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

int cxd56_cisifstopcapture(void)
{
  g_state = STATE_READY;
  cisif_reg_write(CISIF_DIN_ENABLE, 0);
  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

int cxd56_cisifsetdmabuf(cisif_sarea_t *sarea)
{
  int      ret;
  uint32_t cisif_mode;
  uint32_t yuv_regsize;
  uint32_t yuv_hsize;
  uint32_t yuv_vsize;

  ret = cisif_check_sarea(sarea);
  if (ret != OK)
    {
      return ret;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);

  switch (cisif_mode)
    {
      case MODE_YUV_TRS_EN:
        ret = cisif_set_yuv_sarea(sarea);
        break;

      case MODE_JPG_TRS_EN:
        ret = cisif_set_jpg_sarea(sarea);
        break;

      default: /* MODE_INTLEV_TRS_EN */

        /* Get YUV frame size information */

        yuv_regsize =  cisif_reg_read(CISIF_ACT_SIZE);
        yuv_vsize = (yuv_regsize >> 16) & 0x1ff;
        yuv_hsize = yuv_regsize & 0x01ff;

        ret = cisif_set_intlev_sarea(sarea,
                                     YUV_SIZE(yuv_vsize, yuv_hsize));
        break;
    }

  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_EXE_CMD, 1);
  g_storage_addr = (uint32_t)sarea->strg_addr;

  return ret;
}
