/****************************************************************************
 * arch/arm/src/common/ameba/ameba_adc.c
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

/* NuttX ADC lower half for the Realtek Ameba ADC.  The chip has one SAR
 * converter fed by a channel-switch list; the board says which channels to
 * sample and which analog pad each external channel is wired to, and this
 * driver exposes them as one /dev/adc0 device.  Every ANIOC_TRIGGER samples
 * the whole list, in list order, and hands each raw conversion value to the
 * ADC upper half tagged with the hardware channel number the sample came
 * from.
 *
 * The converter is driven through the SDK fwlib ADC API.  Those routines are
 * marked _LONG_CALL_ but are linked from the fwlib RAM source ameba_adc.c
 * (already compiled into every image for the clock/brown-out calibration),
 * so no extra source is added to the board build for the ADC.  The fwlib API
 * takes no register base: it selects the secure or non-secure ADC alias
 * itself with TrustZone_IsSecure(), so this driver never touches a base
 * address.
 *
 * On amebadplus the SDK deliberately does not enable the ADC hardware
 * software-trigger path (ADC_SWTrigCmd() is compiled out and only logs a
 * "not supported" note).  The supported on-demand read is a bounded burst of
 * the auto channel-switch FIFO: ADC_ReceiveBuf() clears the FIFO, enables
 * the auto channel switch, reads exactly the requested number of words and
 * disables it again.  Reading one word per listed channel therefore yields a
 * single polled sweep of the list -- the on-demand, no-interrupt behaviour a
 * software trigger would give -- which is what the ANIOC_TRIGGER handler
 * does.  Everything runs in task context; the ADC_IRQ line is unused.
 *
 * The chip-specific wiring (channel count, list depth, pad-mux code, clock
 * masks) lives in the per-chip ameba_adc_chip.h.  To keep the vendor headers
 * out of the NuttX include world the few fwlib symbols and constants used
 * here are declared locally rather than pulled in from <ameba_adc.h>.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "ameba_adc.h"
#include "ameba_adc_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE               0x0
#define AMEBA_ENABLE                0x1

/* fwlib ADC operation mode and pad values (from ameba_adc.h / ameba_gpio.h).
 * The auto channel-switch mode is the one ADC_ReceiveBuf() drives; the pad
 * is left un-pulled and its digital input buffer is disabled so the analog
 * front end sees a clean input.
 */

#define AMEBA_ADC_AUTO_MODE         0x1  /* ADC_AUTO_MODE       */
#define AMEBA_ADC_CLKDIV_24         0x3  /* ADC_CLK_DIV_24      */
#define AMEBA_PAD_NOPULL            0x0  /* GPIO_PuPd_NOPULL    */

/* 16-bit conversion word layout: channel id in [19:16], data in [15:0]. */

#define AMEBA_ADC_GET_CHNO(x)       (((x) >> 16) & 0x0f)
#define AMEBA_ADC_GET_DATA(x)       ((x) & 0xffff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ameba_adc_dev_s
{
  const struct adc_callback_s *cb;   /* Upper-half receive callback */
  uint8_t   nchan;                   /* Channels in the switch list */
  bool      enabled;                 /* Converter is powered/running */

  /* Hardware channel number sampled at each list position. */

  uint8_t   chanlist[AMEBA_ADC_MAXLIST];

  /* Analog pad per list position, AMEBA_ADC_PIN_NC for internal channels. */

  uint8_t   pins[AMEBA_ADC_MAXLIST];
};

/* Layout-compatible mirror of the fwlib ADC_InitTypeDef.  The driver only
 * ever writes the leading opmode/cvlistlen/cvlist[16] fields (offsets
 * 0..17), plus -- on amebadplus only -- the ClkDiv byte at offset 18.  That
 * prefix is identical on every current Ameba chip, so those writes always
 * land on the right fields.
 *
 * Everything past offset 18 is deliberately NOT named here: the tail order
 * differs per chip (amebadplus is ClkDiv/RxThresholdLevel/SpecialCh/
 * ChanInType at 28B, amebagreen2 has RxThresholdLevel/SpecialCh/
 * SamplePeriodUs at 22B, RTL8720F adds an ADC_Chan[8] sub-struct at 52B),
 * and giving those bytes amebadplus-specific names would only be correct on
 * one chip and misleading on the others.  The driver never reads or writes
 * them -- it lets ADC_StructInit() fill the WHOLE per-chip struct with the
 * correct defaults for whichever fwlib is linked, then overwrites just the
 * prefix (and ClkDiv).
 *
 * The reserved[] tail exists solely so this stack object is at least as
 * large as the linked chip's real ADC_InitTypeDef -- otherwise
 * ADC_StructInit(), which writes the whole struct, would overflow the
 * stack.  Each chip header supplies the real sizeof through
 * AMEBA_ADC_INIT_SIZE and the static assertion below fails at compile time
 * if a new port forgets it or under-sizes it.  ClkDiv sits at a fixed
 * offset 18 on amebadplus, so it stays named; the reserved run starts right
 * after the named prefix.
 */

/* Byte span of the named prefix (opmode + cvlistlen + cvlist[16] + clkdiv).
 * Single source of truth for where the reserved tail begins; the offsetof
 * assertion below verifies the compiler laid the prefix out with no padding
 * so this really is the prefix size.
 */

#define AMEBA_ADC_PREFIX_SIZE  (1 + 1 + 16 + 1)

struct ameba_adc_init_s
{
  uint8_t  opmode;                   /* ADC_OpMode                          */
  uint8_t  cvlistlen;                /* ADC_CvlistLen                       */
  uint8_t  cvlist[16];               /* ADC_Cvlist[16]                      */
  uint8_t  clkdiv;                   /* ADC_ClkDiv (amebadplus only, @18)   */

  /* Chip-specific tail (RxThreshold/SpecialCh/... ) -- filled by
   * ADC_StructInit(), never touched by this driver.  Sized so the whole
   * mirror is >= the linked chip's ADC_InitTypeDef; ternary keeps it
   * non-empty when a chip's struct is <= the prefix.
   */

  uint8_t  reserved[AMEBA_ADC_INIT_SIZE > AMEBA_ADC_PREFIX_SIZE ?
                    AMEBA_ADC_INIT_SIZE - AMEBA_ADC_PREFIX_SIZE : 1];
};

static_assert(offsetof(struct ameba_adc_init_s, reserved) ==
              AMEBA_ADC_PREFIX_SIZE,
              "ameba_adc_init_s prefix has unexpected padding");

static_assert(sizeof(struct ameba_adc_init_s) >= AMEBA_ADC_INIT_SIZE,
              "ameba_adc_init_s smaller than the fwlib ADC_InitTypeDef");

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib ADC / pin / clock API (linked from fwlib RAM ameba_adc.c and the
 * ROM pinmux table).  None of the ADC calls take a register base -- the
 * fwlib picks the secure/non-secure alias internally.
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void Pinmux_Config(uint8_t pin, uint32_t func);
extern void PAD_PullCtrl(uint8_t pin, uint8_t pulltype);
extern void PAD_SleepPullCtrl(uint8_t pin, uint8_t pulltype);
extern void PAD_InputCtrl(uint8_t pin, uint32_t newstate);
extern void ADC_StructInit(struct ameba_adc_init_s *init);
extern void ADC_Init(struct ameba_adc_init_s *init);
extern void ADC_Cmd(uint32_t newstate);
extern void ADC_ReceiveBuf(uint32_t *buf, uint32_t len);

/* ADC lower-half operations. */

static int  ameba_adc_bind(struct adc_dev_s *dev,
                           const struct adc_callback_s *callback);
static void ameba_adc_reset(struct adc_dev_s *dev);
static int  ameba_adc_setup(struct adc_dev_s *dev);
static void ameba_adc_shutdown(struct adc_dev_s *dev);
static void ameba_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  ameba_adc_ioctl(struct adc_dev_s *dev, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_ameba_adc_ops =
{
  .ao_bind     = ameba_adc_bind,
  .ao_reset    = ameba_adc_reset,
  .ao_setup    = ameba_adc_setup,
  .ao_shutdown = ameba_adc_shutdown,
  .ao_rxint    = ameba_adc_rxint,
  .ao_ioctl    = ameba_adc_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_adc_hwinit
 *
 * Description:
 *   Program the converter's channel-switch list for auto channel-switch
 *   reads.  ADC_Init() itself loads the list from the init struct, so the
 *   whole board list is applied in one call; the converter is left disabled
 *   (setup() powers it on).
 *
 ****************************************************************************/

static void ameba_adc_hwinit(struct ameba_adc_dev_s *priv)
{
  struct ameba_adc_init_s init;
  int i;

  memset(&init, 0, sizeof(init));
  ADC_StructInit(&init);

  init.opmode    = AMEBA_ADC_AUTO_MODE;
  init.cvlistlen = (uint8_t)(priv->nchan - 1);

  /* ClkDiv exists at offset 18 only on amebadplus; on other chips that
   * byte is RxThresholdLevel, so writing it would corrupt an unrelated
   * field.  Guard with the chip macro and keep the SDK default elsewhere.
   */

#ifdef AMEBA_ADC_HAS_CLKDIV
  init.clkdiv    = AMEBA_ADC_CLKDIV_24;
#endif

  for (i = 0; i < priv->nchan; i++)
    {
      init.cvlist[i] = priv->chanlist[i];
    }

  ADC_Init(&init);
}

/****************************************************************************
 * Name: ameba_adc_bind
 ****************************************************************************/

static int ameba_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;

  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ameba_adc_reset
 *
 * Description:
 *   Reload the channel-switch list; called before setup() and on error
 *   recovery.  The peripheral clock and pads are latched at registration.
 *
 ****************************************************************************/

static void ameba_adc_reset(struct adc_dev_s *dev)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;

  ameba_adc_hwinit(priv);
}

/****************************************************************************
 * Name: ameba_adc_setup
 *
 * Description:
 *   Power the converter on when the device is first opened.
 *
 ****************************************************************************/

static int ameba_adc_setup(struct adc_dev_s *dev)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;

  if (!priv->enabled)
    {
      ADC_Cmd(AMEBA_ENABLE);
      priv->enabled = true;
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_adc_shutdown
 *
 * Description:
 *   Power the converter off when the device is closed.
 *
 ****************************************************************************/

static void ameba_adc_shutdown(struct adc_dev_s *dev)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;

  if (priv->enabled)
    {
      ADC_Cmd(AMEBA_DISABLE);
      priv->enabled = false;
    }
}

/****************************************************************************
 * Name: ameba_adc_rxint
 *
 * Description:
 *   No-op: this driver reads by polling the auto channel-switch FIFO on
 *   demand, so there are no RX interrupts to enable or disable.
 *
 ****************************************************************************/

static void ameba_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  UNUSED(dev);
  UNUSED(enable);
}

/****************************************************************************
 * Name: ameba_adc_trigger
 *
 * Description:
 *   Sample the whole channel list once and push each raw value to the upper
 *   half.  ADC_ReceiveBuf() reads one word per listed channel, in list
 *   order, through the auto channel-switch FIFO; the channel number is taken
 *   from each word so the sample is tagged with the channel it came from.
 *
 ****************************************************************************/

static int ameba_adc_trigger(struct adc_dev_s *dev)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;
  uint32_t buf[AMEBA_ADC_MAXLIST];
  int i;

  if (priv->cb == NULL || priv->cb->au_receive == NULL)
    {
      return -EINVAL;
    }

  ADC_ReceiveBuf(buf, priv->nchan);

  for (i = 0; i < priv->nchan; i++)
    {
      uint8_t  ch   = (uint8_t)AMEBA_ADC_GET_CHNO(buf[i]);
      int32_t  data = (int32_t)AMEBA_ADC_GET_DATA(buf[i]);

      priv->cb->au_receive(dev, ch, data);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_adc_ioctl
 ****************************************************************************/

static int ameba_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  struct ameba_adc_dev_s *priv = (struct ameba_adc_dev_s *)dev->ad_priv;
  int ret;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        ret = ameba_adc_trigger(dev);
        break;

      case ANIOC_GET_NCHANNELS:
        ret = priv->nchan;
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_adc_register
 *
 * Description:
 *   See ameba_adc.h.
 *
 ****************************************************************************/

int ameba_adc_register(const char *path, const uint8_t *channels,
                       const uint8_t *pins, unsigned int nchan)
{
  struct ameba_adc_dev_s *priv;
  struct adc_dev_s *dev;
  unsigned int i;
  int ret;

  if (nchan == 0)
    {
      return -EINVAL;
    }

  if (nchan > AMEBA_ADC_MAXLIST)
    {
      nchan = AMEBA_ADC_MAXLIST;
    }

  dev = kmm_zalloc(sizeof(struct adc_dev_s) +
                   sizeof(struct ameba_adc_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  priv = (struct ameba_adc_dev_s *)(dev + 1);
  dev->ad_ops  = &g_ameba_adc_ops;
  dev->ad_priv = priv;

  priv->nchan = (uint8_t)nchan;
  memset(priv->pins, AMEBA_ADC_PIN_NC, sizeof(priv->pins));

  /* Gate the ADC peripheral clock (plus the cap-touch/CTC clock domain on
   * the chips whose header defines it) and turn each external channel's pad
   * into an analog input: route it to the ADC crossbar function, drop its
   * pulls and disable its digital input buffer.  Internal channels have no
   * pad and are skipped.
   */

  RCC_PeriphClockCmd(AMEBA_ADC_APBPERIPH, AMEBA_ADC_APBPERIPH_CLK,
                     AMEBA_ENABLE);
#ifdef AMEBA_ADC_AUXCLK_PERIPH
  RCC_PeriphClockCmd(AMEBA_ADC_AUXCLK_PERIPH, AMEBA_ADC_AUXCLK_CLK,
                     AMEBA_ENABLE);
#endif

  for (i = 0; i < nchan; i++)
    {
      priv->chanlist[i] = channels[i];
      priv->pins[i]     = pins[i];

      if (pins[i] != AMEBA_ADC_PIN_NC)
        {
          Pinmux_Config(pins[i], AMEBA_ADC_PINMUX_FID);
          PAD_PullCtrl(pins[i], AMEBA_PAD_NOPULL);
          PAD_SleepPullCtrl(pins[i], AMEBA_PAD_NOPULL);
          PAD_InputCtrl(pins[i], AMEBA_DISABLE);
        }
    }

  ameba_adc_hwinit(priv);

  ret = adc_register(path, dev);
  if (ret < 0)
    {
      aerr("ERROR: adc_register(%s) failed: %d\n", path, ret);
      kmm_free(dev);
      return ret;
    }

  return OK;
}
