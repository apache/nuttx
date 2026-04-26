/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexio_dshot.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/dshot.h>

#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "imx9_clockconfig.h"
#include "imx9_flexio_dshot.h"
#include "imx9_gpio.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_ccm.h"
#include "hardware/imx9_pinmux.h"
#include "hardware/imx9_flexio.h"

#ifdef CONFIG_IMX9_FLEXIO_DSHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-bit 3-chip symbols for DShot TX */

#define DSHOT_SYMBOL_0             0b100
#define DSHOT_SYMBOL_1             0b110

/* Telemetry request bit in a TX packet */

#define TELEM_BIT                  4

/* Extract the FlexIO output index for channel 'ch' from the packed
 * CONFIG_IMX9_FLEXIOx_DSHOT_CHANNEL_PINS value (one byte per channel).
 */

#define PIN_FOR_CHANNEL(priv, ch)    ((uint8_t)((priv)->pins >> ((ch) * 8)))

/* Maximum flexio root clock frequency acc. to the reference manual */

#define FLEXIO_MAX_ROOT_FREQ 80000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_dshot_channel_s
{
  int8_t shifter;
  int8_t timer;
  int8_t pin;

  uint32_t tx_timctl;
  uint32_t rx_timctl;
  uint32_t tx_shftctl;
  uint32_t rx_shftctl;

  volatile uint32_t curr_timctl;
  volatile struct timespec ts_raw;
  volatile uint32_t rx_raw;
  volatile int state;
};

struct imx9_flexio_dshot_s
{
  const struct dshot_lowerhalf_s lh;

  const flexio_dshot_id_t id;
  const uintptr_t base;
  const int irq;
  const uint64_t pins;
  const int tx_timer;

  uint32_t tx_timcmp;
  uint32_t rx_timcmp;

  bool initialized;
  bool bidir_enabled;
  int nchannels;

  uint32_t active_ch_mask;

  volatile uint32_t timien;
  volatile uint32_t shiftsien;

  spinlock_t     spinlock;

  struct imx9_dshot_channel_s chan[DSHOT_MAX_CHANNELS];
};

enum imx9_dshot_ch_state_e
{
  CH_TX_SEG1 = 0,
  CH_TX_SEG2 = 1,
  CH_RX      = 2,
  CH_DONE    = 3
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int imx9_dshot_setup(struct dshot_lowerhalf_s *dev);
static int imx9_dshot_shutdown(struct dshot_lowerhalf_s *dev);
static int imx9_dshot_configure(struct dshot_lowerhalf_s *dev,
                                const struct dshot_config_s *cfg);
static int imx9_dshot_send_command(struct dshot_lowerhalf_s *dev,
                                   const uint16_t *packets,
                                   uint16_t ch_mask);
static int imx9_dshot_get_raw_telemetry(struct dshot_lowerhalf_s *dev,
                                        struct dshot_raw_telemetry_s *raw,
                                        uint16_t ch_mask);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dshot_ops_s g_imx9_dshot_ops =
{
  .setup             = imx9_dshot_setup,
  .shutdown          = imx9_dshot_shutdown,
  .configure         = imx9_dshot_configure,
  .send_command      = imx9_dshot_send_command,
  .get_raw_telemetry = imx9_dshot_get_raw_telemetry,
  .ioctl             = NULL,
};

static struct imx9_flexio_dshot_s g_dshot_dev[] =
{
#ifdef CONFIG_IMX9_FLEXIO1_DSHOT
  {
    .lh.ops    = &g_imx9_dshot_ops,
    .id        = DSHOT_FLEXIO1,
    .base      = IMX9_FLEXIO1_BASE,
    .irq       = IMX9_IRQ_FLEXIO1,
    .nchannels = CONFIG_IMX9_FLEXIO1_DSHOT_NCHANNELS,
    .pins      = CONFIG_IMX9_FLEXIO1_DSHOT_CHANNEL_PINS,
    .tx_timer  = CONFIG_IMX9_FLEXIO1_DSHOT_TX_TIMER,
    .spinlock  = SP_UNLOCKED,
  },
#endif
#ifdef CONFIG_IMX9_FLEXIO2_DSHOT
  {
    .lh.ops    = &g_imx9_dshot_ops,
    .id        = DSHOT_FLEXIO2,
    .base      = IMX9_FLEXIO2_BASE,
    .irq       = IMX9_IRQ_FLEXIO2,
    .nchannels = CONFIG_IMX9_FLEXIO2_DSHOT_NCHANNELS,
    .pins      = CONFIG_IMX9_FLEXIO2_DSHOT_CHANNEL_PINS,
    .tx_timer  = CONFIG_IMX9_FLEXIO2_DSHOT_TX_TIMER,
    .spinlock  = SP_UNLOCKED,
  },
#endif
};

/* Constant register values for the FlexIO SHIFTCFG, TIMCFG and TIMCTL */

/* Timer control, same for TX and RX apart from trigger control */

static const uint32_t timctl = (FLEXIO_TIMCTL_TRGPOL(1) |
                                FLEXIO_TIMCTL_TRGSRC(1) |
                                FLEXIO_TIMCTL_PINCFG(0) |
                                FLEXIO_TIMCTL_PINSEL(0) |
                                FLEXIO_TIMCTL_PINPOL(0));

/* These are the same for TX and RX */

static const uint32_t shftcfg = (FLEXIO_SHIFTCFG_PWIDTH(0) |
                                 FLEXIO_SHIFTCFG_INSRC(0) |
                                 FLEXIO_SHIFTCFG_SSTOP(1) |
                                 FLEXIO_SHIFTCFG_SSTART(0));

static const uint32_t timcfg = (FLEXIO_TIMCFG_TIMOUT(0) |
                                FLEXIO_TIMCFG_TIMDEC(0) |
                                FLEXIO_TIMCFG_TIMRST(0) |
                                FLEXIO_TIMCFG_TIMDIS(2) |
                                FLEXIO_TIMCFG_TIMENA(2) |
                                FLEXIO_TIMCFG_TSTOP(2) |
                                FLEXIO_TIMCFG_TSTART(0));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t imx9_flexio_getreg(struct imx9_flexio_dshot_s *priv,
                                          int offset)
{
  return getreg32(priv->base + offset);
}

static inline void imx9_flexio_putreg(struct imx9_flexio_dshot_s *priv,
                                      int offset,
                                      uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static inline void imx9_flexio_modifyreg(struct imx9_flexio_dshot_s *priv,
                                         int offset, uint32_t clear,
                                         uint32_t set)
{
  modifyreg32(priv->base + offset, clear, set);
}

static inline void imx9_dshot_get_timestamp(struct timespec *time)
{
#if defined (CONFIG_IMX9_FLEXIO_DSHOT_TS_SYSTIME)
  clock_systime_timespec(time);
#elif defined (CONFIG_IMX9_FLEXIO_DSHOT_TS_BOARDTIME)
  board_systime_timespec(time);
#else /* CONFIG_IMX9_FLEXIO_DSHOT_TS_NONE */
  time->tv_sec = 0;
  time->tv_nsec = 0;
#endif
}

/* Expand each packet bit to 3 bit symbols on the wire */

static uint64_t imx9_dshot_expand_data(uint16_t packet)
{
  uint32_t mask;
  uint64_t expanded = 0;

  for (mask = 0x8000; mask != 0; mask >>= 1)
    {
      expanded <<= 3;
      expanded |= (packet & mask) ? DSHOT_SYMBOL_1 : DSHOT_SYMBOL_0;
    }

  return expanded;
}

static int imx9_dshot_select_clock(struct imx9_flexio_dshot_s *priv,
                                   uint32_t freq, uint32_t telem_freq)
{
  uint32_t div;
  uint32_t clk_hz;
  uint32_t baud_half_tx;
  uint32_t baud_half_rx;
  int clkroot = CCM_CR_FLEXIO1 + priv->id;
  int ret;

  ret = imx9_get_clock(SYS_PLL1PFD0DIV2, &clk_hz);
  if (ret < 0)
    {
      return ret;
    }

  /* Start with minimum divider for the clock that doesn't exceed the maximum
   * allowed root clock frequency
   */

  div = clk_hz / FLEXIO_MAX_ROOT_FREQ;
  if (div == 0)
    {
      /* Set to minimum 1, in case clk_hz < FLEXIO_MAX_ROOT_FREQ */

      div = 1;
    }
  else if (clk_hz / div > FLEXIO_MAX_ROOT_FREQ)
    {
      /* Don't exceed the MAX_ROOT_FREQ due to rounding error */

      div++;
    }

  /* Search for the smallest divider that keeps baud_half in [1, 256]. */

  for (; div <= 255; div++)
    {
      ret = imx9_ccm_configure_root_clock(clkroot, SYS_PLL1PFD0DIV2, div);
      if (ret < 0)
        {
          return ret;
        }

      ret = imx9_get_rootclock(clkroot, &clk_hz);
      if (ret < 0)
        {
          return ret;
        }

      /* Calculate rounded baud rate halves (the result is
       * register write * 2 + 1).
       */

      baud_half_tx = clk_hz / (freq * 3);
      baud_half_tx += baud_half_tx & 1;
      baud_half_tx /= 2;

      baud_half_rx = clk_hz / telem_freq;
      baud_half_rx += baud_half_rx & 1;
      baud_half_rx /= 2;

      if (baud_half_tx > 0 && baud_half_tx <= 256 &&
          baud_half_rx > 0 && baud_half_rx <= 256)
        {
          /* TX uses 8-bit baud mode
           * Configure rx_timcmp to count for 21 bits. The actual receive
           * frame is only 20 bits, but the first transition from high to low
           * is just a start of the frame and needs to be skipped.
           *
           * Configure tx_timcmp to count for 48 / 2.
           * The TX is done in 2 23 bit parts + stop bit (0) for both.
           */

          priv->tx_timcmp = ((23 * 2 - 1) << 8) |
            (baud_half_tx - 1);
          priv->rx_timcmp = ((21 * 2 - 1) << 8) | (baud_half_rx - 1);
          imx9_ccm_gate_on(CCM_LPCG_FLEXIO1 + priv->id, true);
          return OK;
        }
    }

  return -ERANGE;
}

static void imx9_dshot_apply_pinmux(flexio_dshot_id_t id, int channel)
{
  if (id == DSHOT_FLEXIO1)
    {
      switch (channel)
        {
#ifdef FLEXIO1_DSHOT0_MUX
          case 0:
            imx9_iomux_configure(FLEXIO1_DSHOT0_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT1_MUX
          case 1:
            imx9_iomux_configure(FLEXIO1_DSHOT1_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT2_MUX
          case 2:
            imx9_iomux_configure(FLEXIO1_DSHOT2_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT3_MUX
          case 3:
            imx9_iomux_configure(FLEXIO1_DSHOT3_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT4_MUX
          case 4:
            imx9_iomux_configure(FLEXIO1_DSHOT4_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT5_MUX
          case 5:
            imx9_iomux_configure(FLEXIO1_DSHOT5_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT6_MUX
          case 6:
            imx9_iomux_configure(FLEXIO1_DSHOT6_MUX);
            break;
#endif
#ifdef FLEXIO1_DSHOT7_MUX
          case 7:
            imx9_iomux_configure(FLEXIO1_DSHOT7_MUX);
            break;
#endif
          default:
            break;
        }
    }
  else
    {
      switch (channel)
        {
#ifdef FLEXIO2_DSHOT0_MUX
          case 0:
            imx9_iomux_configure(FLEXIO2_DSHOT0_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT1_MUX
          case 1:
            imx9_iomux_configure(FLEXIO2_DSHOT1_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT2_MUX
          case 2:
            imx9_iomux_configure(FLEXIO2_DSHOT2_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT3_MUX
          case 3:
            imx9_iomux_configure(FLEXIO2_DSHOT3_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT4_MUX
          case 4:
            imx9_iomux_configure(FLEXIO2_DSHOT4_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT5_MUX
          case 5:
            imx9_iomux_configure(FLEXIO2_DSHOT5_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT6_MUX
          case 6:
            imx9_iomux_configure(FLEXIO2_DSHOT6_MUX);
            break;
#endif
#ifdef FLEXIO2_DSHOT7_MUX
          case 7:
            imx9_iomux_configure(FLEXIO2_DSHOT7_MUX);
            break;
#endif
          default:
            break;
        }
    }
}

/* Switch channel configuration between RX and TX as needed */

static void imx9_dshot_switch_channel(struct imx9_flexio_dshot_s *priv,
                                      struct imx9_dshot_channel_s *ch,
                                      bool rx)
{
  uint32_t new_shftctl = rx ? ch->rx_shftctl : ch->tx_shftctl;
  uint32_t new_timctl;
  uint32_t new_timcmp;

  /* If using common tx_timer for all channels, only switch between
   * rx and tx for that one. Keep all the other timers in rx setting
   * so that no extra time is spent in register writes.
   */

  if ((priv->tx_timer >= 0 && priv->tx_timer != ch->timer) || rx)
    {
      new_timctl = ch->rx_timctl;
      new_timcmp = priv->rx_timcmp;
    }
  else
    {
      new_timctl = ch->tx_timctl;
      new_timcmp = priv->tx_timcmp;
    }

  /* Disable timer if re-configuration is needed */

  if (ch->curr_timctl != new_timctl)
    {
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(ch->timer),
                         ch->curr_timctl & (~FLEXIO_TIMCTL_TIMOD_MASK));
    }

  /* Shifter needs to write a temporary "2" to PINCFG to avoid
   * spurious transition to low according to the TRM.
   */

  imx9_flexio_putreg(priv,
                     IMX9_FLEXIO_SHIFTCTL_OFFSET(ch->shifter),
                     (ch->tx_shftctl &
                      (~(FLEXIO_SHIFTCTL_SMOD_MASK |
                         FLEXIO_SHIFTCTL_PINCFG_MASK))) |
                     FLEXIO_SHIFTCTL_PINCFG(2));

  /* Reconfigure shifter */

  imx9_flexio_putreg(priv,
                     IMX9_FLEXIO_SHIFTCTL_OFFSET(ch->shifter),
                     new_shftctl);

  /* Reconfigure the timer if needed */

  if (ch->curr_timctl != new_timctl)
    {
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMCMP_OFFSET(ch->timer),
                         new_timcmp);
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(ch->timer),
                         new_timctl);
      ch->curr_timctl = new_timctl;
    }
}

static void imx9_dshot_config_channel(struct imx9_flexio_dshot_s *priv,
                                      struct imx9_dshot_channel_s *ch)
{
  bool common_tx = priv->tx_timer >= 0;

  /* Polarity is inverted for Bidirectional DShot */

  uint32_t pinpol = priv->bidir_enabled ? 1 : 0;

  /* If all channels use the same timer for TX, configure that to shiftctl */

  int tx_timer = common_tx ? priv->tx_timer : ch->timer;

  /* Channel configuration for TX, trigger timer on shift buffer
   * not empty
   */

  ch->tx_shftctl = (FLEXIO_SHIFTCTL_TIMSEL(tx_timer) |
                    FLEXIO_SHIFTCTL_TIMPOL(0) |
                    FLEXIO_SHIFTCTL_PINCFG(3) |
                    FLEXIO_SHIFTCTL_PINSEL(ch->pin) |
                    FLEXIO_SHIFTCTL_PINPOL(pinpol) |
                    FLEXIO_SHIFTCTL_SMOD(2));

  ch->tx_timctl = timctl | FLEXIO_TIMCTL_TRGSEL(4 * ch->shifter + 1);

  /* When using a common TX timer, it is enabled separately in send_command
   * to avoid spurious triggers while shifters are being switched from RX
   * to TX.
   */

  if (!common_tx)
    {
      ch->tx_timctl |= FLEXIO_TIMCTL_TIMOD(1);
    }

  /* Channel configuration for RX, Enable timer on the first bit
   * seen in the shifter input (falling edge on the pin)
   */

  ch->rx_shftctl = (FLEXIO_SHIFTCTL_TIMSEL(ch->timer) |
                    FLEXIO_SHIFTCTL_TIMPOL(1) |
                    FLEXIO_SHIFTCTL_PINCFG(0) |
                    FLEXIO_SHIFTCTL_PINSEL(ch->pin) |
                    FLEXIO_SHIFTCTL_PINPOL(0) |
                    FLEXIO_SHIFTCTL_SMOD(1));

  ch->rx_timctl = timctl | FLEXIO_TIMCTL_TRGSEL(2 * ch->pin) |
                           FLEXIO_TIMCTL_ONETIM(1) |
                           FLEXIO_TIMCTL_TIMOD(1);
}

static int imx9_dshot_configure(struct dshot_lowerhalf_s *dev,
                                const struct dshot_config_s *cfg)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)dev;
  int i;
  int ret = 0;

  /* Verify the inputs; frequencies must be non-zero, active mask
   * must not contain channels which don't exist
   */

  if (cfg == NULL || cfg->freq == 0 || cfg->telem_freq == 0 ||
      (cfg->active_mask & (~((1 << priv->nchannels) - 1))) != 0)
    {
      return -EINVAL;
    }

  ret = imx9_dshot_select_clock(priv, cfg->freq, cfg->telem_freq);
  if (ret < 0)
    {
      return ret;
    }

  priv->bidir_enabled = (cfg->bidir != 0);

  /* Reset and enable flexio, poll for enabled */

  imx9_flexio_putreg(priv, IMX9_FLEXIO_CTRL_OFFSET, FLEXIO_CTRL_SWRST(1));
  imx9_flexio_putreg(priv, IMX9_FLEXIO_CTRL_OFFSET, FLEXIO_CTRL_FLEXEN(1));

  while (imx9_flexio_getreg(priv, IMX9_FLEXIO_CTRL_OFFSET) !=
         FLEXIO_CTRL_FLEXEN_MASK)
    {
    }

  /* Disable all shifter and timer interrupts and clear status */

  imx9_flexio_putreg(priv, IMX9_FLEXIO_SHIFTSIEN_OFFSET, 0);
  imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMIEN_OFFSET, 0);
  imx9_flexio_putreg(priv, IMX9_FLEXIO_SHIFTSTAT_OFFSET, cfg->active_mask);
  priv->timien = 0;
  priv->shiftsien = 0;

  /* Set channel specific configurations */

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      struct imx9_dshot_channel_s *ch = &priv->chan[i];

      ch->state = CH_DONE;
      ch->curr_timctl = 0;

      if ((cfg->active_mask & (1 << (i))) != 0)
        {
          /* Allocate shifters and timers for channels, and initialize
           * state
           */

          ch->shifter = i;
          ch->timer = i;
          ch->pin = PIN_FOR_CHANNEL(priv, i);

          /* Mux the pin */

          imx9_dshot_apply_pinmux(priv->id, i);

          /* Calculate channel specific configurations for shifters and
           * timers
           */

          imx9_dshot_config_channel(priv, ch);

          /* Set the constant configuration values */

          imx9_flexio_putreg(priv,
                             IMX9_FLEXIO_SHIFTCFG_OFFSET(ch->shifter),
                             shftcfg);
          imx9_flexio_putreg(priv,
                             IMX9_FLEXIO_TIMCFG_OFFSET(ch->timer),
                             timcfg);

          /* Set the configuration to TX */

          ch->curr_timctl = 0;
          imx9_dshot_switch_channel(priv, ch, false);
        }
      else
        {
          ch->shifter = -1;
          ch->timer = -1;
          ch->pin = -1;
        }
    }

  if (priv->tx_timer >= 0)
    {
      /* In addition, in case the tx timer is not one of active channels'
       * timers, configure that as well
       */

      imx9_flexio_putreg(priv,
                         IMX9_FLEXIO_TIMCMP_OFFSET(priv->tx_timer),
                         priv->tx_timcmp);
      imx9_flexio_putreg(priv,
                         IMX9_FLEXIO_TIMCFG_OFFSET(priv->tx_timer),
                         timcfg);
    }

  priv->active_ch_mask = cfg->active_mask;

  return OK;
}

static int imx9_dshot_send_command(struct dshot_lowerhalf_s *dev,
                                   const uint16_t *packets,
                                   uint16_t ch_mask)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)dev;
  struct imx9_dshot_channel_s *ch = NULL;
  uint32_t stat;
  uint32_t tx_seg1[DSHOT_MAX_CHANNELS];
  uint32_t tx_seg2[DSHOT_MAX_CHANNELS];
  irqstate_t flags;
  int i;
  uint32_t sft_mask = 0;
  uint32_t tlm_timer_mask = 0;
  uint32_t common_tx_timctl = 0;

  /* Filter out all channels which are not configured */

  ch_mask &= priv->active_ch_mask;

  /* If request contains no active channel data (or there are no active
   * channels), just return
   */

  if (ch_mask == 0)
    {
      return OK;
    }

  /* Loop through the channels, switch each channel to transmit if needed
   * and build the TX packets
   */

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      bool telemetry;
      uint64_t expanded;

      if ((ch_mask & (1 << i)) == 0)
        {
          continue;
        }

      ch = &priv->chan[i];
      telemetry = (packets[i] & (1 << TELEM_BIT)) != 0;

      /* Collect the needed shifters for the requested channels
       * and a bitmap of the timer interrupt enables needed for telemetry
       */

      sft_mask |= 1 << ch->shifter;

      if (priv->bidir_enabled && telemetry)
        {
          tlm_timer_mask |= 1 << ch->timer;
        }

      /* Expand each bit in packet to 3-bit symbols and split it
       * into two 23-bit shift buffer write values. The timer is configured
       * to send 2x23 bits, the last bit is always dropped here, as it is
       * always 0 / STOP.
       */

      expanded = imx9_dshot_expand_data(packets[i]);
      tx_seg1[i] = (uint32_t)((expanded & 0xffffff000000) >> 16);
      tx_seg2[i] = (uint32_t)((expanded & 0xffffff) << 8);
    }

  if (priv->tx_timer >= 0)
    {
      /* If bidirectional telemetry is requested, and we are using a common
       * TX timer, only request interrupts from the single TX channel
       */

      if (tlm_timer_mask)
        {
          tlm_timer_mask = 1 << priv->tx_timer;
        }

      /* Disable the common TX timer just in case; it might get triggered
       * early or it may already be running in some error case. Also set it
       * to trigger on the last TX channel, so that it triggers when all the
       * shifters have been filled in below. Note: ch is left pointing to
       * the last channel after the previous loop.
       */

      DEBUGASSERT(ch != NULL);
      common_tx_timctl = timctl | FLEXIO_TIMCTL_TRGSEL(4 * ch->shifter + 1);
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(priv->tx_timer),
                         common_tx_timctl);
    }

  flags = spin_lock_irqsave(&priv->spinlock);

  if (priv->bidir_enabled)
    {
      /* Disable shifter interrupts, which might be left on */

      priv->shiftsien &= ~sft_mask;
      imx9_flexio_putreg(priv, IMX9_FLEXIO_SHIFTSIEN_OFFSET,
                         priv->shiftsien);

      /* If there was bidirectional telemetry read previously, restore
       * shifter and timer configuration for TX.
       */

      for (i = DSHOT_MAX_CHANNELS - 1; i >= 0; i--)
        {
          if ((ch_mask & (1 << i)) != 0)
            {
              ch = &priv->chan[i];
              imx9_dshot_switch_channel(priv, ch, false);
            }
        }
    }

  if (priv->tx_timer >= 0)
    {
      /* Re-enable the common TX timer.
       */

      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(priv->tx_timer),
                         common_tx_timctl | FLEXIO_TIMCTL_TIMOD(1));
    }

  /* Send out the segment1 for every channel. Don't do anything else
   * in this loop, so that channels start as close to each other
   * as possible timewise. This helps with interrupt load, where interrupts
   * from multiple channels occur at the same time.
   */

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      if ((ch_mask & (1 << i)) != 0)
        {
          bool telemetry = (packets[i] & (1 << TELEM_BIT)) != 0;
          ch = &priv->chan[i];

          /* If telemetry is requested from this channel, set the state to
           * the beginning of the state machine. Otherwise, set it directly
           * to "done".
           */

          ch->state = priv->bidir_enabled && telemetry ? CH_TX_SEG1
                                                       : CH_DONE;

          /* Load seg1 to shifter buffer */

          imx9_flexio_putreg(priv,
                             IMX9_FLEXIO_SHIFTBUFBIS_OFFSET(ch->shifter),
                             tx_seg1[i]);
        }
    }

  /* Poll for the shift buffers to become empty (seg1 starts shifting) */

  do
    {
      stat = imx9_flexio_getreg(priv, IMX9_FLEXIO_SHIFTSTAT_OFFSET);
    }
  while ((stat & sft_mask) != sft_mask);

  /* Segment 1 is now shifting for all channels, shifter buffers are empty */

  if (tlm_timer_mask)
    {
      /* Clear timer status for the timers generating tx interrupts */

      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMSTAT_OFFSET, tlm_timer_mask);

      /* Enable TX timer interrupt(s) if bidirectional DShot telemetry is
       * requested.
       */

      priv->timien |= tlm_timer_mask;
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMIEN_OFFSET, priv->timien);
    }

  /* Load segment 2 */

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      if ((ch_mask & (1 << i)) != 0)
        {
          ch = &priv->chan[i];
          imx9_flexio_putreg(priv,
                             IMX9_FLEXIO_SHIFTBUFBIS_OFFSET(ch->shifter),
                             tx_seg2[i]);
        }
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  return OK;
}

/* For bi-directional DShot, there are 3 interrupts to be handled:
 * 1. timer interrupt in the middle of TX (state CH_TX_SEG1)
 * 2. timer interrupt in the end of TX (state CH_TX_SEG2)
 * 3. shifter interrupt in the end of RX (state CH_RX)
 *
 * First interrupt is useless, but can't really be avoided. On the second
 * one, the shifter/timer pair is configured for RX. On the third one,
 * the RX data is read from the shifter.
 */

static int imx9_dshot_irq_handler(int irq, void *context, void *arg)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)arg;

  uint32_t timstat_clear = 0;
  uint32_t shiftsien_enable = 0;
  uint32_t shiftsien_disable = 0;
  uint32_t timien_disable = 0;
  int i;

  irqstate_t flags = spin_lock_irqsave(&priv->spinlock);
  uint32_t timstat = imx9_flexio_getreg(priv, IMX9_FLEXIO_TIMSTAT_OFFSET)
                       & priv->timien;
  uint32_t shiftstat = imx9_flexio_getreg(priv, IMX9_FLEXIO_SHIFTSTAT_OFFSET)
                         & priv->shiftsien;
  struct timespec ts_raw;
  bool common_tx = priv->tx_timer >= 0;

  /* Get timestamp in case we have shifter interrupts (data is ready) */

  if (shiftstat != 0)
    {
      imx9_dshot_get_timestamp(&ts_raw);
    }

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      struct imx9_dshot_channel_s *ch = &priv->chan[i];
      uint32_t tim_irq;
      uint32_t shift_irq;

      if ((priv->active_ch_mask & (1 << i)) == 0 || ch->state >= CH_DONE)
        {
          /* Channel not active (ch->timer, ch->shifter not valid) or
           * no telemetry requested
           */

          continue;
        }

      tim_irq = common_tx ? timstat & (1 << priv->tx_timer)
                          : timstat & (1 << ch->timer);
      shift_irq = (shiftstat & (1 << ch->shifter)) != 0;

      if (!tim_irq && !shift_irq)
        {
          /* No interrupt for this channel */

          continue;
        }

      if (tim_irq && ch->state == CH_TX_SEG1)
        {
          /* First timer interrupt. Just clear it and wait for the next */

          timstat_clear |= tim_irq;
          ch->state = CH_TX_SEG2;
        }
      else if (tim_irq && ch->state == CH_TX_SEG2)
        {
          /* Second timer interrupt, configure for RX */

          /* If the rx timer and tx timer are not the same, but rx timer is
           * already running, force re-configure it (stop & re-start)
           */

          if (common_tx && priv->tx_timer != ch->timer &&
              (timstat & (1 << ch->timer)) == 0)
            {
              ch->curr_timctl = 0;
            }

          /* Set this channel to RX */

          imx9_dshot_switch_channel(priv, ch, true);

          /* Enable shifter interrupt and disable timer interrupt for this
           * channel
           */

          shiftsien_enable |= 1 << ch->shifter;
          timien_disable |= 1 << ch->timer;

          /* Clear the status for the RX timer to enable the trigger */

          timstat_clear |= 1 << ch->timer;
          ch->state = CH_RX;
        }
      else if (shift_irq && ch->state == CH_RX)
        {
          /* Shifter interrupt, data is ready in bits 19:0 */

          ch->rx_raw = imx9_flexio_getreg(
                         priv, IMX9_FLEXIO_SHIFTBUFBIS_OFFSET(ch->shifter));
          ch->rx_raw &= 0xfffff;
          ch->ts_raw = ts_raw;
          shiftsien_disable |= 1 << ch->shifter;
          ch->state = CH_DONE;
        }
    }

  if (timien_disable != 0)
    {
      /* TX complete */

      /* If there is just one timer used for all channels, modify the
       * disable mask to only contain that timer
       */

      if (common_tx)
        {
          timien_disable = 1 << priv->tx_timer;
        }

      /* Clear shifter status for RX channels */

      imx9_flexio_putreg(priv, IMX9_FLEXIO_SHIFTSTAT_OFFSET,
                         shiftsien_enable);

      /* Disable the handled timer interrupts */

      priv->timien &= ~timien_disable;
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMIEN_OFFSET,
                         priv->timien);
    }

  if (shiftsien_disable != 0 || shiftsien_enable != 0)
    {
      /* TX or RX complete */

      /* Enable shifter irqs for channels just configured for RX, disable
       * for channels which already received data
       */

      priv->shiftsien |= shiftsien_enable;
      priv->shiftsien &= ~shiftsien_disable;
      imx9_flexio_putreg(priv, IMX9_FLEXIO_SHIFTSIEN_OFFSET,
                         priv->shiftsien);
    }

  /* Clear status of the timer interrupts */

  if (timstat_clear != 0)
    {
      imx9_flexio_putreg(priv, IMX9_FLEXIO_TIMSTAT_OFFSET, timstat_clear);
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);

  return OK;
}

static int imx9_dshot_setup(struct dshot_lowerhalf_s *dev)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)dev;
  int ret;

  if (priv->initialized)
    {
      return OK;
    }

  ret = irq_attach(priv->irq, imx9_dshot_irq_handler, priv);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
      priv->initialized = true;
    }

  return ret;
}

static int imx9_dshot_shutdown(struct dshot_lowerhalf_s *dev)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)dev;

  if (!priv->initialized)
    {
      return OK;
    }

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  priv->initialized = false;
  priv->active_ch_mask = 0;

  return OK;
}

static int imx9_dshot_get_raw_telemetry(struct dshot_lowerhalf_s *dev,
                                        struct dshot_raw_telemetry_s *raw,
                                        uint16_t ch_mask)
{
  struct imx9_flexio_dshot_s *priv =
    (struct imx9_flexio_dshot_s *)dev;
  int i;

  if (raw == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < DSHOT_MAX_CHANNELS; i++)
    {
      if ((ch_mask & (1u << i)) != 0)
        {
          irqstate_t flags = spin_lock_irqsave(&priv->spinlock);
          raw[i].raw = priv->chan[i].rx_raw;
          raw[i].timestamp = priv->chan[i].ts_raw;
          spin_unlock_irqrestore(&priv->spinlock, flags);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dshot_lowerhalf_s *imx9_flexio_dshot_init(flexio_dshot_id_t id)
{
  struct imx9_flexio_dshot_s *priv = NULL;
  int i;

  for (i = 0; i < sizeof(g_dshot_dev) / sizeof(g_dshot_dev[0]); i++)
    {
      if (g_dshot_dev[i].id == id)
        {
          priv = &g_dshot_dev[i];
          break;
        }
    }

  if (priv == NULL)
    {
      return NULL;
    }

  return (struct dshot_lowerhalf_s *)&priv->lh;
}

void imx9_flexio_dshot_deinit(flexio_dshot_id_t id)
{
}

#endif /* CONFIG_IMX9_FLEXIO_DSHOT */
