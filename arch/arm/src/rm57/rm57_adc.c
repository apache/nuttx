/****************************************************************************
 * arch/arm/src/rm57/rm57_adc.c
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

/* MibADC driver for the RM57L843 (TRM SPNU562A chapter 22).
 *
 * The device has two ADC modules.  ADC1 serves 32 analog inputs, ADC2 serves
 * 25 of which 16 are the same balls ADC1 sees.  Each module contains three
 * independent conversion groups - the Event group (which the interrupt
 * header names "GROUP0"), Group1 and Group2 - and each group has its own
 * channel selection mask, trigger source, sampling window, results FIFO and
 * VIM interrupt channel.  A group is therefore the natural unit to expose as
 * a NuttX ADC device, and this driver registers one /dev/adcN per enabled
 * group.
 *
 * Two layers of state follow from that split.  A few settings belong to the
 * module and are shared by all three of its groups: the 10/12-bit resolution
 * (ADOPMODECR.10_12_BIT), the ADCLK prescaler, the offset correction value
 * in ADCALR, the parity configuration, and the partitioning of the 64-word
 * results RAM.  Everything else is per group.  struct rm57_adcmod_s holds
 * the former, struct rm57_adcgrp_s the latter, and the module is brought up
 * once, by whichever of its groups is initialized first.
 *
 * Results RAM partitioning deserves a note because it is easy to get wrong.
 * ADBNDCR splits the 64 buffers in *pairs*: the Event group receives
 * 2 x BNDA buffers and Group1 receives 2 x (BNDB - BNDA), with Group2 taking
 * whatever remains.  Because the register is shared, the split is computed
 * from every enabled group of a module at once, in rm57_adc_partition(), and
 * the per-group buffer counts are checked against the 64-word total at
 * compile time.
 *
 * The peripheral clock needs no attention here: rm57_periph_init() in
 * rm57_clockconfig.c powers up every PCR quadrant unconditionally, so both
 * MibADC modules are already clocked when this driver runs.  Likewise the
 * MPU peripheral region already spans both the register frames and the
 * results RAM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_adc.h"
#include "hardware/rm57_sys.h"
#include "rm57_adc.h"
#include "rm57_iomm.h"

#ifdef CONFIG_RM57_ADC_HAVE_DMA
#  include <nuttx/cache.h>
#  include "hardware/rm57_dma.h"
#  include "rm57_dma.h"
#endif

#ifdef CONFIG_RM57_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The ADC prescaler divides VCLK.  ADCLK = VCLK / (PS + 1), and the
 * datasheet (SPNS215, tc(ADCLK) >= 33ns) caps ADCLK at 30MHz.
 */

#define RM57_ADC_VCLK           BOARD_VCLK_FREQUENCY
#define RM57_ADC_CLOCK_MAX      30000000

/* Round the prescaler up so the resulting ADCLK never exceeds the request.
 * ADCLOCKCR.PS is only five bits wide, so a request slow enough to need a
 * larger divider cannot be expressed; that is checked at compile time below
 * rather than being silently truncated by the register write.
 */

#define RM57_ADC_PRESCALER(f)   ((RM57_ADC_VCLK + (f) - 1) / (f) - 1)
#define RM57_ADC_PRESCALER_MAX  (ADC_CLOCKCR_PS_MASK >> ADC_CLOCKCR_PS_SHIFT)

/* Sampling window used for the calibration conversions.  Those are timed by
 * the Event group's ADEVSAMP whatever group the caller was using (TRM
 * SPNU562A section 22.2.6.1.1), and they have to meet the same ~1us minimum
 * sampling time as any other conversion.  The Event group may not even be in
 * the configuration, in which case ADEVSAMP is still at its reset value of
 * zero - a two cycle window, below the hardware's own SW >= 3 floor - so the
 * calibration path programs it itself.
 */

#define RM57_ADC_CAL_CYCLES(ps) (((RM57_ADC_VCLK / ((ps) + 1)) + 999999) / \
                                 1000000)
#define RM57_ADC_CAL_ACQ(ps)    (RM57_ADC_CAL_CYCLES(ps) > \
                                 ADC_SAMP_ACQ_MIN + 2 ? \
                                 RM57_ADC_CAL_CYCLES(ps) - 2 : \
                                 ADC_SAMP_ACQ_MIN)

/* Time to allow after releasing the ADC core from power down before a
 * conversion may be started, td(PU-ADV) of TRM section 22.2.6.3.1.  The
 * enhanced power down mode is not used, so ADPWRUPDLYCTRL does not apply and
 * the wait is the driver's job.
 *
 * ASSUMPTION: SPNS215 does not publish td(PU-ADV) for this device - it is
 * absent from the MibADC timing table - so this is a conservative guess
 * rather than a datasheet number.
 */

#define RM57_ADC_PWRUP_DELAY_US 20

/* D-cache line of the Cortex-R5F, in bytes (ARM DDI 0460, section 7.1) */

#define RM57_ADC_DCACHE_LINE    32

/* Timeout, in polling iterations, for hardware handshakes that have no
 * interrupt to wait on: the calibration conversion (ADCALCR.CAL_ST) and the
 * results RAM auto-initialization (ADBNDEND.BUF_INIT_ACTIVE).
 */

#define RM57_ADC_TIMEOUT        1000000

/* Self test decision thresholds, as a fraction of full scale.  Driving
 * ADREFHI onto a healthy channel through the internal test resistor pulls
 * the reading high, and ADREFLO pulls it low; a channel shorted to one rail
 * refuses to move, and an open channel follows the test source all the way.
 */

#define RM57_ADC_ST_LOW(full)   ((full) / 4)
#define RM57_ADC_ST_HIGH(full)  ((full) - (full) / 4)

/* Per-module configuration, expanded from the Kconfig selections */

#ifdef CONFIG_RM57_ADC1
#  ifdef CONFIG_RM57_ADC1_12BIT
#    define RM57_ADC1_RESOLUTION 12
#  else
#    define RM57_ADC1_RESOLUTION 10
#  endif
#  define RM57_ADC1_PS          RM57_ADC_PRESCALER(CONFIG_RM57_ADC1_CLOCK)
#endif

#ifdef CONFIG_RM57_ADC2
#  ifdef CONFIG_RM57_ADC2_12BIT
#    define RM57_ADC2_RESOLUTION 12
#  else
#    define RM57_ADC2_RESOLUTION 10
#  endif
#  define RM57_ADC2_PS          RM57_ADC_PRESCALER(CONFIG_RM57_ADC2_CLOCK)
#endif

/* Results RAM buffer counts.  A disabled group contributes nothing, and the
 * boundaries are programmed in pairs so every count must be even.
 */

#ifdef CONFIG_RM57_ADC1_EV
#  define RM57_ADC1_EV_BUFS     CONFIG_RM57_ADC1_EV_BUFS
#else
#  define RM57_ADC1_EV_BUFS     0
#endif
#ifdef CONFIG_RM57_ADC1_G1
#  define RM57_ADC1_G1_BUFS     CONFIG_RM57_ADC1_G1_BUFS
#else
#  define RM57_ADC1_G1_BUFS     0
#endif
#ifdef CONFIG_RM57_ADC1_G2
#  define RM57_ADC1_G2_BUFS     CONFIG_RM57_ADC1_G2_BUFS
#else
#  define RM57_ADC1_G2_BUFS     0
#endif

#ifdef CONFIG_RM57_ADC2_EV
#  define RM57_ADC2_EV_BUFS     CONFIG_RM57_ADC2_EV_BUFS
#else
#  define RM57_ADC2_EV_BUFS     0
#endif
#ifdef CONFIG_RM57_ADC2_G1
#  define RM57_ADC2_G1_BUFS     CONFIG_RM57_ADC2_G1_BUFS
#else
#  define RM57_ADC2_G1_BUFS     0
#endif
#ifdef CONFIG_RM57_ADC2_G2
#  define RM57_ADC2_G2_BUFS     CONFIG_RM57_ADC2_G2_BUFS
#else
#  define RM57_ADC2_G2_BUFS     0
#endif

#ifdef CONFIG_RM57_ADC1
static_assert(RM57_ADC1_EV_BUFS + RM57_ADC1_G1_BUFS + RM57_ADC1_G2_BUFS <=
              RM57_ADC_NRESULTBUFS,
              "ADC1 group result buffers exceed the 64-word results RAM");
static_assert(((RM57_ADC1_EV_BUFS | RM57_ADC1_G1_BUFS) & 1) == 0,
              "ADC1 Event and Group1 buffer counts must be even");
static_assert(RM57_ADC_VCLK / (RM57_ADC1_PS + 1) <= RM57_ADC_CLOCK_MAX,
              "ADC1 ADCLK exceeds the 30MHz datasheet maximum");
static_assert(RM57_ADC1_PS <= RM57_ADC_PRESCALER_MAX,
              "ADC1 conversion clock is too slow for the 5-bit\n"
              "ADCLOCKCR.PS field");
#endif

#ifdef CONFIG_RM57_ADC2
static_assert(RM57_ADC2_EV_BUFS + RM57_ADC2_G1_BUFS + RM57_ADC2_G2_BUFS <=
              RM57_ADC_NRESULTBUFS,
              "ADC2 group result buffers exceed the 64-word results RAM");
static_assert(((RM57_ADC2_EV_BUFS | RM57_ADC2_G1_BUFS) & 1) == 0,
              "ADC2 Event and Group1 buffer counts must be even");
static_assert(RM57_ADC_VCLK / (RM57_ADC2_PS + 1) <= RM57_ADC_CLOCK_MAX,
              "ADC2 ADCLK exceeds the 30MHz datasheet maximum");
static_assert(RM57_ADC2_PS <= RM57_ADC_PRESCALER_MAX,
              "ADC2 conversion clock is too slow for the 5-bit\n"
              "ADCLOCKCR.PS field");
#endif

/* Kconfig symbols that are conditional - the per-group flags, and the
 * trigger source which depends on its group being hardware triggered - are
 * only defined when selected.  Give each a zero fallback so the
 * RM57_ADC_GRPDEF() expansion below always sees a value.
 */

#ifndef CONFIG_RM57_ADC1_EV_TRIGSRC
#  define CONFIG_RM57_ADC1_EV_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC1_EV_HWTRIG
#  define CONFIG_RM57_ADC1_EV_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC1_EV_RISING
#  define CONFIG_RM57_ADC1_EV_RISING 0
#endif
#ifndef CONFIG_RM57_ADC1_EV_CONTINUOUS
#  define CONFIG_RM57_ADC1_EV_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC1_EV_FREEZE
#  define CONFIG_RM57_ADC1_EV_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC1_EV_OVRWRITE
#  define CONFIG_RM57_ADC1_EV_OVRWRITE 0
#endif

#ifndef CONFIG_RM57_ADC1_G1_TRIGSRC
#  define CONFIG_RM57_ADC1_G1_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC1_G1_HWTRIG
#  define CONFIG_RM57_ADC1_G1_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC1_G1_RISING
#  define CONFIG_RM57_ADC1_G1_RISING 0
#endif
#ifndef CONFIG_RM57_ADC1_G1_CONTINUOUS
#  define CONFIG_RM57_ADC1_G1_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC1_G1_FREEZE
#  define CONFIG_RM57_ADC1_G1_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC1_G1_OVRWRITE
#  define CONFIG_RM57_ADC1_G1_OVRWRITE 0
#endif

#ifndef CONFIG_RM57_ADC1_G2_TRIGSRC
#  define CONFIG_RM57_ADC1_G2_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC1_G2_HWTRIG
#  define CONFIG_RM57_ADC1_G2_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC1_G2_RISING
#  define CONFIG_RM57_ADC1_G2_RISING 0
#endif
#ifndef CONFIG_RM57_ADC1_G2_CONTINUOUS
#  define CONFIG_RM57_ADC1_G2_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC1_G2_FREEZE
#  define CONFIG_RM57_ADC1_G2_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC1_G2_OVRWRITE
#  define CONFIG_RM57_ADC1_G2_OVRWRITE 0
#endif

#ifndef CONFIG_RM57_ADC2_EV_TRIGSRC
#  define CONFIG_RM57_ADC2_EV_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC2_EV_HWTRIG
#  define CONFIG_RM57_ADC2_EV_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC2_EV_RISING
#  define CONFIG_RM57_ADC2_EV_RISING 0
#endif
#ifndef CONFIG_RM57_ADC2_EV_CONTINUOUS
#  define CONFIG_RM57_ADC2_EV_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC2_EV_FREEZE
#  define CONFIG_RM57_ADC2_EV_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC2_EV_OVRWRITE
#  define CONFIG_RM57_ADC2_EV_OVRWRITE 0
#endif

#ifndef CONFIG_RM57_ADC2_G1_TRIGSRC
#  define CONFIG_RM57_ADC2_G1_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC2_G1_HWTRIG
#  define CONFIG_RM57_ADC2_G1_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC2_G1_RISING
#  define CONFIG_RM57_ADC2_G1_RISING 0
#endif
#ifndef CONFIG_RM57_ADC2_G1_CONTINUOUS
#  define CONFIG_RM57_ADC2_G1_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC2_G1_FREEZE
#  define CONFIG_RM57_ADC2_G1_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC2_G1_OVRWRITE
#  define CONFIG_RM57_ADC2_G1_OVRWRITE 0
#endif

#ifndef CONFIG_RM57_ADC2_G2_TRIGSRC
#  define CONFIG_RM57_ADC2_G2_TRIGSRC 0
#endif
#ifndef CONFIG_RM57_ADC2_G2_HWTRIG
#  define CONFIG_RM57_ADC2_G2_HWTRIG 0
#endif
#ifndef CONFIG_RM57_ADC2_G2_RISING
#  define CONFIG_RM57_ADC2_G2_RISING 0
#endif
#ifndef CONFIG_RM57_ADC2_G2_CONTINUOUS
#  define CONFIG_RM57_ADC2_G2_CONTINUOUS 0
#endif
#ifndef CONFIG_RM57_ADC2_G2_FREEZE
#  define CONFIG_RM57_ADC2_G2_FREEZE 0
#endif
#ifndef CONFIG_RM57_ADC2_G2_OVRWRITE
#  define CONFIG_RM57_ADC2_G2_OVRWRITE 0
#endif
/* System module memory initialization bits for the two results RAMs */

#define RM57_ADC1_MSINENA_BIT   (1 << 8)
#define RM57_ADC2_MSINENA_BIT   (1 << 14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_adcgrp_s;

/* State shared by the three conversion groups of one ADC module */

struct rm57_adcmod_s
{
  uintptr_t base;          /* Register frame base address */
  uintptr_t rambase;       /* Conversion results RAM base address */
  uintptr_t lutbase;       /* Channel select LUT base, 0 if unsupported */
  uint32_t  msinena;       /* SYS MSINENA bit for the results RAM */
  uint8_t   module;        /* RM57_ADC_MODULE1 or RM57_ADC_MODULE2 */
  uint8_t   resolution;    /* 10 or 12 bits */
  uint8_t   prescaler;     /* ADCLOCKCR.PS */
  uint8_t   nchannels;     /* Number of external analog inputs */
  uint32_t  validmask;     /* Channels this module can select */
  uint8_t   refcount;      /* Groups currently set up */
  bool      initialized;   /* Module-level bring-up has run */
  mutex_t   lock;          /* Serializes calibration and self test */

#ifdef CONFIG_RM57_ADC_MAGCOMP
  uint8_t   magcomp_irq;      /* VIM channel of the comparators */
  bool      magcomp_attached; /* Its handler is attached */
  uint32_t  magcomp_events;   /* Threshold crossings seen */
#endif

  struct rm57_adcgrp_s *grp[RM57_ADC_NGROUPS];
};

/* One conversion group, exposed as a single /dev/adcN */

struct rm57_adcgrp_s
{
  struct adc_dev_s dev;    /* Upper half device; ad_priv points back here */
  struct rm57_adcmod_s *mod;
  uint8_t   group;         /* RM57_ADC_GROUP_EV/_G1/_G2 */
  uint8_t   irq;           /* VIM channel for this group */
  bool      setup;         /* ao_setup() has run */
  struct rm57_adc_grpcfg_s cfg;
  const struct adc_callback_s *cb;
  uint32_t  overruns;      /* Results memory overrun count */

  /* One result popped from the FIFO that the upper half would not take.
   * See rm57_adc_drain().
   */

  bool      held;
  uint8_t   heldchan;
  int32_t   heldvalue;

#ifdef CONFIG_RM57_ADC_HAVE_DMA
  bool      usedma;        /* Deliver results by DMA rather than by CPU */
  uint8_t   dmareq;        /* DMA request line for this group */
  uint16_t  dmansamples;   /* Results per half of the DMA ring */
  DMA_HANDLE dma;
  uint32_t *dmabuf;        /* Ring of 2 x dmansamples words */
#endif
};

/* Module state that the calibration and self test sequences have to take
 * over and hand back.  See rm57_adc_calprologue().
 */

struct rm57_adc_calsave_s
{
  uint32_t sel[RM57_ADC_NGROUPS]; /* Each group's channel selection */
  uint32_t evsamp;                /* Event group sampling window */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Upper half ADC operations */

static int  rm57_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback);
static void rm57_adc_reset(struct adc_dev_s *dev);
static int  rm57_adc_setup(struct adc_dev_s *dev);
static void rm57_adc_shutdown(struct adc_dev_s *dev);
static void rm57_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  rm57_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg);

/* Interrupt handling */

static int  rm57_adc_interrupt(int irq, void *context, void *arg);

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
/* Defined below, but called from the module bring-up above it */

static void rm57_adc_tempmux(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_rm57_adc_ops =
{
  .ao_bind     = rm57_adc_bind,
  .ao_reset    = rm57_adc_reset,
  .ao_setup    = rm57_adc_setup,
  .ao_shutdown = rm57_adc_shutdown,
  .ao_rxint    = rm57_adc_rxint,
  .ao_ioctl    = rm57_adc_ioctl,
};

#ifdef CONFIG_RM57_ADC1
static struct rm57_adcmod_s g_adc1 =
{
  .base       = RM57_ADC1_BASE,
  .rambase    = RM57_ADC1_RAM_BASE,
  .lutbase    = RM57_ADC1_LUT_BASE,
  .msinena    = RM57_ADC1_MSINENA_BIT,
  .module     = RM57_ADC_MODULE1,
  .resolution = RM57_ADC1_RESOLUTION,
  .prescaler  = RM57_ADC1_PS,
  .nchannels  = RM57_ADC1_NCHANNELS,
  .validmask  = RM57_ADC1_VALIDMASK,
  .lock       = NXMUTEX_INITIALIZER,
#ifdef CONFIG_RM57_ADC_MAGCOMP
  .magcomp_irq = RM57_REQ_ADC1MAGCOMP,
#endif
};
#endif

#ifdef CONFIG_RM57_ADC2
static struct rm57_adcmod_s g_adc2 =
{
  .base       = RM57_ADC2_BASE,
  .rambase    = RM57_ADC2_RAM_BASE,

  /* ADC2 implements only the sequential channel selection mode, so it has
   * no look-up table (TRM SPNU562A section 22.2.2.2).
   */

  .lutbase    = 0,
  .msinena    = RM57_ADC2_MSINENA_BIT,
  .module     = RM57_ADC_MODULE2,
  .resolution = RM57_ADC2_RESOLUTION,
  .prescaler  = RM57_ADC2_PS,
  .nchannels  = RM57_ADC2_NCHANNELS,
  .validmask  = RM57_ADC2_VALIDMASK,
  .lock       = NXMUTEX_INITIALIZER,
#ifdef CONFIG_RM57_ADC_MAGCOMP
  .magcomp_irq = RM57_REQ_ADC2MAGCOMP,
#endif
};
#endif

/* Group instances.  RM57_ADC_GRPDEF() expands the Kconfig selections for one
 * group into its static configuration; the DMA members only exist when some
 * group in the build actually uses DMA.
 */

#ifdef CONFIG_RM57_ADC_HAVE_DMA
#  define RM57_ADC_DMADEF(m,g)                                              \
     .usedma      = true,                                                   \
     .dmareq      = DMA_REQ_MIBADC##m##_##g,                                \
     .dmansamples = CONFIG_RM57_ADC##m##_##g##_DMA_NSAMPLES,
#  define RM57_ADC_NODMADEF                                                 \
     .usedma      = false,
#else
#  define RM57_ADC_NODMADEF
#endif

#define RM57_ADC_BOOL(x)  ((x) != 0)

#define RM57_ADC_GRPDEF(m, g, grpid, dmadef)                                \
  {                                                                         \
    .dev  = { .ad_ops = &g_rm57_adc_ops },                                  \
    .mod  = &g_adc##m,                                                      \
    .group = (grpid),                                                       \
    .irq  = RM57_REQ_ADC##m##GROUP##grpid,                                  \
    .cfg  =                                                                 \
    {                                                                       \
      .chanmask     = CONFIG_RM57_ADC##m##_##g##_CHANMASK,                  \
      .acq          = CONFIG_RM57_ADC##m##_##g##_ACQ,                       \
      .threshold    = CONFIG_RM57_ADC##m##_##g##_THRESHOLD,                 \
      .trigsrc      = CONFIG_RM57_ADC##m##_##g##_TRIGSRC,                   \
      .hwtrig       = RM57_ADC_BOOL(CONFIG_RM57_ADC##m##_##g##_HWTRIG),     \
      .rising       = RM57_ADC_BOOL(CONFIG_RM57_ADC##m##_##g##_RISING),     \
      .bothedges    = false,                                                \
      .continuous   = RM57_ADC_BOOL(CONFIG_RM57_ADC##m##_##g##_CONTINUOUS), \
      .freeze       = RM57_ADC_BOOL(CONFIG_RM57_ADC##m##_##g##_FREEZE),     \
      .ovrwrite     = RM57_ADC_BOOL(CONFIG_RM57_ADC##m##_##g##_OVRWRITE),   \
      .dischargeen  = false,                                                \
      .dischargecyc = 0,                                                    \
    },                                                                      \
    dmadef                                                                  \
  }

#ifdef CONFIG_RM57_ADC1_EV
#  ifdef CONFIG_RM57_ADC1_EV_DMA
#    define ADC1_EV_DMA RM57_ADC_DMADEF(1, EVT)
#  else
#    define ADC1_EV_DMA RM57_ADC_NODMADEF
#  endif
static struct rm57_adcgrp_s g_adc1_ev =
  RM57_ADC_GRPDEF(1, EV, 0, ADC1_EV_DMA);
#endif

#ifdef CONFIG_RM57_ADC1_G1
#  ifdef CONFIG_RM57_ADC1_G1_DMA
#    define ADC1_G1_DMA RM57_ADC_DMADEF(1, G1)
#  else
#    define ADC1_G1_DMA RM57_ADC_NODMADEF
#  endif
static struct rm57_adcgrp_s g_adc1_g1 =
  RM57_ADC_GRPDEF(1, G1, 1, ADC1_G1_DMA);
#endif

#ifdef CONFIG_RM57_ADC1_G2
#  ifdef CONFIG_RM57_ADC1_G2_DMA
#    define ADC1_G2_DMA RM57_ADC_DMADEF(1, G2)
#  else
#    define ADC1_G2_DMA RM57_ADC_NODMADEF
#  endif
static struct rm57_adcgrp_s g_adc1_g2 =
  RM57_ADC_GRPDEF(1, G2, 2, ADC1_G2_DMA);
#endif

#ifdef CONFIG_RM57_ADC2_EV
static struct rm57_adcgrp_s g_adc2_ev =
  RM57_ADC_GRPDEF(2, EV, 0, RM57_ADC_NODMADEF);
#endif

#ifdef CONFIG_RM57_ADC2_G1
static struct rm57_adcgrp_s g_adc2_g1 =
  RM57_ADC_GRPDEF(2, G1, 1, RM57_ADC_NODMADEF);
#endif

#ifdef CONFIG_RM57_ADC2_G2
static struct rm57_adcgrp_s g_adc2_g2 =
  RM57_ADC_GRPDEF(2, G2, 2, RM57_ADC_NODMADEF);
#endif

/* DMA landing buffers.  Each is one D-cache line aligned so invalidating a
 * buffer cannot disturb anything sharing its line.
 *
 * Two constraints apply to every group's sample count, and neither can be
 * expressed in Kconfig because they relate symbols from different menus:
 *
 *   - It bounds the scratch arrays rm57_adc_dmacallback() puts on the
 *     interrupt stack, which are sized by CONFIG_RM57_ADC_DMA_MAXSAMPLES.
 *   - Each half of the ring is invalidated on its own, so a half has to be
 *     a whole number of D-cache lines.  Otherwise the unaligned edge is
 *     clean-invalidated instead, which writes the CPU's stale copy of that
 *     line back over the half the ADC is still filling.
 */

#define RM57_ADC_DMA_LINEWORDS  (RM57_ADC_DCACHE_LINE / sizeof(uint32_t))

#define RM57_ADC_DMA_ASSERT(n, name)                                        \
  static_assert((n) <= CONFIG_RM57_ADC_DMA_MAXSAMPLES,                      \
                name " DMA samples exceed RM57_ADC_DMA_MAXSAMPLES");        \
  static_assert(((n) % RM57_ADC_DMA_LINEWORDS) == 0,                        \
                name " DMA samples must be a multiple of a D-cache line")

#ifdef CONFIG_RM57_ADC1_EV_DMA
RM57_ADC_DMA_ASSERT(CONFIG_RM57_ADC1_EV_DMA_NSAMPLES, "ADC1 Event group");
static uint32_t g_adc1_ev_dmabuf[2 * CONFIG_RM57_ADC1_EV_DMA_NSAMPLES]
  aligned_data(RM57_ADC_DCACHE_LINE);
#endif
#ifdef CONFIG_RM57_ADC1_G1_DMA
RM57_ADC_DMA_ASSERT(CONFIG_RM57_ADC1_G1_DMA_NSAMPLES, "ADC1 Group1");
static uint32_t g_adc1_g1_dmabuf[2 * CONFIG_RM57_ADC1_G1_DMA_NSAMPLES]
  aligned_data(RM57_ADC_DCACHE_LINE);
#endif
#ifdef CONFIG_RM57_ADC1_G2_DMA
RM57_ADC_DMA_ASSERT(CONFIG_RM57_ADC1_G2_DMA_NSAMPLES, "ADC1 Group2");
static uint32_t g_adc1_g2_dmabuf[2 * CONFIG_RM57_ADC1_G2_DMA_NSAMPLES]
  aligned_data(RM57_ADC_DCACHE_LINE);
#endif

/* Guards module-wide registers that groups modify from different contexts */

static spinlock_t g_adc_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_adc_getreg / rm57_adc_putreg
 *
 * Description:
 *   Read and write one of the owning module's registers by offset.
 *
 ****************************************************************************/

static inline uint32_t rm57_adc_getreg(struct rm57_adcmod_s *mod,
                                       unsigned int offset)
{
  return getreg32(mod->base + offset);
}

static inline void rm57_adc_putreg(struct rm57_adcmod_s *mod,
                                   unsigned int offset, uint32_t value)
{
  putreg32(value, mod->base + offset);
}

/****************************************************************************
 * Name: rm57_adc_modifyreg
 *
 * Description:
 *   Read-modify-write a module register with interrupts disabled.  Several
 *   module-wide registers (ADOPMODECR above all) are touched by more than
 *   one group, so the sequence has to be atomic against both the other
 *   groups' interrupt handlers and other CPUs.
 *
 ****************************************************************************/

static void rm57_adc_modifyreg(struct rm57_adcmod_s *mod,
                               unsigned int offset, uint32_t clrbits,
                               uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  flags  = spin_lock_irqsave(&g_adc_lock);
  regval = getreg32(mod->base + offset);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(regval, mod->base + offset);
  spin_unlock_irqrestore(&g_adc_lock, flags);
}

/****************************************************************************
 * Name: rm57_adc_powerup / rm57_adc_idlepowerdown
 *
 * Description:
 *   Release the ADC core from power down, and put it back once nothing is
 *   using it.  ADOPMODECR.POWERDOWN stops the core outright - the sequencer
 *   keeps its registers but no conversion can complete - so every path that
 *   is about to convert has to come through here, not just ao_setup(): a
 *   diagnostic conversion can perfectly well be the first thing to run after
 *   the last /dev/adcN of a module was closed.
 *
 *   TRM SPNU562A section 22.2.6.3.1 requires td(PU-ADV) between clearing the
 *   bit and starting a conversion, and says in as many words that the wait
 *   is the application's job when the enhanced power down mode is off.
 *
 ****************************************************************************/

static void rm57_adc_powerup(struct rm57_adcmod_s *mod)
{
  if ((rm57_adc_getreg(mod, RM57_ADC_OPMODECR_OFFSET) &
       ADC_OPMODECR_POWERDOWN) == 0)
    {
      return;
    }

  rm57_adc_modifyreg(mod, RM57_ADC_OPMODECR_OFFSET,
                     ADC_OPMODECR_POWERDOWN, ADC_OPMODECR_ADC_EN);
  up_udelay(RM57_ADC_PWRUP_DELAY_US);
}

static void rm57_adc_idlepowerdown(struct rm57_adcmod_s *mod)
{
  if (mod->refcount == 0)
    {
      rm57_adc_modifyreg(mod, RM57_ADC_OPMODECR_OFFSET, 0,
                         ADC_OPMODECR_POWERDOWN);
    }
}

/****************************************************************************
 * Name: rm57_adc_fullscale
 *
 * Description:
 *   Largest conversion result the module can produce in its configured
 *   resolution.
 *
 ****************************************************************************/

static inline uint16_t rm57_adc_fullscale(struct rm57_adcmod_s *mod)
{
  return mod->resolution == 12 ? ADC_CALR_12BIT_MASK : ADC_CALR_10BIT_MASK;
}

/****************************************************************************
 * Name: rm57_adc_ishwtrig
 *
 * Description:
 *   Whether a group starts its conversions from a hardware event.  Group1
 *   and Group2 choose, through MODECR.HW_TRIG, but the Event group has no
 *   such bit - bit 3 is reserved there - because it is always driven by its
 *   trigger source (TRM Figures 22-24 and 22-26).
 *
 ****************************************************************************/

static inline bool rm57_adc_ishwtrig(struct rm57_adcgrp_s *grp)
{
  return grp->group == RM57_ADC_GROUP_EV || grp->cfg.hwtrig;
}

/****************************************************************************
 * Name: rm57_adc_validchan
 *
 * Description:
 *   Whether a module can select one particular channel.  This is a mask
 *   rather than a count because ADC2's selectable set is not contiguous: it
 *   has 25 external inputs at channels 0-24, and then channels 30 and 31
 *   address the on-die temperature sensors, which have no ball at all.
 *
 ****************************************************************************/

static inline bool rm57_adc_validchan(struct rm57_adcmod_s *mod,
                                      uint8_t channel)
{
  return channel < 32 && (mod->validmask & ((uint32_t)1 << channel)) != 0;
}

/****************************************************************************
 * Name: rm57_adc_module
 *
 * Description:
 *   Look up a module by index, or NULL if it is not in this configuration.
 *
 ****************************************************************************/

static struct rm57_adcmod_s *rm57_adc_module(int module)
{
  switch (module)
    {
#ifdef CONFIG_RM57_ADC1
      case RM57_ADC_MODULE1:
        return &g_adc1;
#endif
#ifdef CONFIG_RM57_ADC2
      case RM57_ADC_MODULE2:
        return &g_adc2;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: rm57_adc_group
 *
 * Description:
 *   Look up a conversion group by module and group index, or NULL if that
 *   group is not in this configuration.
 *
 ****************************************************************************/

static struct rm57_adcgrp_s *rm57_adc_group(int module, int group)
{
  if (module == RM57_ADC_MODULE1)
    {
      switch (group)
        {
#ifdef CONFIG_RM57_ADC1_EV
          case RM57_ADC_GROUP_EV:
            return &g_adc1_ev;
#endif
#ifdef CONFIG_RM57_ADC1_G1
          case RM57_ADC_GROUP_G1:
            return &g_adc1_g1;
#endif
#ifdef CONFIG_RM57_ADC1_G2
          case RM57_ADC_GROUP_G2:
            return &g_adc1_g2;
#endif
          default:
            break;
        }
    }
  else if (module == RM57_ADC_MODULE2)
    {
      switch (group)
        {
#ifdef CONFIG_RM57_ADC2_EV
          case RM57_ADC_GROUP_EV:
            return &g_adc2_ev;
#endif
#ifdef CONFIG_RM57_ADC2_G1
          case RM57_ADC_GROUP_G1:
            return &g_adc2_g1;
#endif
#ifdef CONFIG_RM57_ADC2_G2
          case RM57_ADC_GROUP_G2:
            return &g_adc2_g2;
#endif
          default:
            break;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: rm57_adc_nbuffers
 *
 * Description:
 *   Number of results RAM buffers reserved for one conversion group.  The
 *   values come from Kconfig and are validated at compile time above.
 *
 ****************************************************************************/

static uint8_t rm57_adc_nbuffers(int module, int group)
{
  if (module == RM57_ADC_MODULE1)
    {
      switch (group)
        {
          case RM57_ADC_GROUP_EV:
            return RM57_ADC1_EV_BUFS;
          case RM57_ADC_GROUP_G1:
            return RM57_ADC1_G1_BUFS;
          case RM57_ADC_GROUP_G2:
            return RM57_ADC1_G2_BUFS;
          default:
            return 0;
        }
    }
  else
    {
      switch (group)
        {
          case RM57_ADC_GROUP_EV:
            return RM57_ADC2_EV_BUFS;
          case RM57_ADC_GROUP_G1:
            return RM57_ADC2_G1_BUFS;
          case RM57_ADC_GROUP_G2:
            return RM57_ADC2_G2_BUFS;
          default:
            return 0;
        }
    }
}

/****************************************************************************
 * Name: rm57_adc_partition
 *
 * Description:
 *   Program ADBNDCR so the 64-word results RAM is split between the module's
 *   three conversion groups.  The boundaries are expressed in *pairs* of
 *   buffers: the Event group gets 2 x BNDA and Group1 gets 2 x (BNDB-BNDA),
 *   with Group2 taking the remainder.  Because one register covers all three
 *   groups, this is computed for the whole module at once.
 *
 ****************************************************************************/

static void rm57_adc_partition(struct rm57_adcmod_s *mod)
{
  uint32_t evbufs = rm57_adc_nbuffers(mod->module, RM57_ADC_GROUP_EV);
  uint32_t g1bufs = rm57_adc_nbuffers(mod->module, RM57_ADC_GROUP_G1);
  uint32_t bnda   = evbufs / 2;
  uint32_t bndb   = bnda + g1bufs / 2;

  rm57_adc_putreg(mod, RM57_ADC_BNDCR_OFFSET,
                  ADC_BNDCR_BNDA(bnda) | ADC_BNDCR_BNDB(bndb));

  /* BNDEND selects the total memory size and shares its register with the
   * read-only auto-initialization status, so preserve the upper half.
   */

  rm57_adc_modifyreg(mod, RM57_ADC_BNDEND_OFFSET, ADC_BNDEND_MASK,
                     ADC_BNDEND_64WORDS);
}

/****************************************************************************
 * Name: rm57_adc_raminit
 *
 * Description:
 *   Auto-initialize the module's conversion results RAM to zero through the
 *   system module's memory initialization controller, so the parity bits are
 *   consistent before parity checking is switched on (TRM section 22.2.7.1).
 *
 ****************************************************************************/

static void rm57_adc_raminit(struct rm57_adcmod_s *mod)
{
  uint32_t regval;
  int i;

  /* Unlock the global memory initialization key, then start this RAM */

  regval = getreg32(RM57_SYS_MINITGCR) & ~SYS_MINITGCR_MASK;
  putreg32(regval | SYS_MINITGCR_ENABLE, RM57_SYS_MINITGCR);
  putreg32(mod->msinena, RM57_SYS_MSINENA);

  for (i = 0; i < RM57_ADC_TIMEOUT; i++)
    {
      if ((rm57_adc_getreg(mod, RM57_ADC_BNDEND_OFFSET) &
           ADC_BNDEND_BUF_INIT_ACTIVE) == 0)
        {
          break;
        }
    }

  if (i >= RM57_ADC_TIMEOUT)
    {
      aerr("ERROR: ADC%d results RAM auto-init did not complete\n",
           mod->module + 1);
    }

  /* Withdraw the request and re-lock the key, so a stray write cannot
   * restart an initialization behind the driver's back.
   */

  putreg32(0, RM57_SYS_MSINENA);

  regval = getreg32(RM57_SYS_MINITGCR) & ~SYS_MINITGCR_MASK;
  putreg32(regval | SYS_MINITGCR_DISABLE, RM57_SYS_MINITGCR);
}

/****************************************************************************
 * Name: rm57_adc_modinit
 *
 * Description:
 *   Bring one ADC module out of reset and apply the settings its three
 *   conversion groups share.  Runs once per module, driven by whichever of
 *   its groups is initialized first.
 *
 ****************************************************************************/

static void rm57_adc_modinit(struct rm57_adcmod_s *mod)
{
  uint32_t opmode;

  if (mod->initialized)
    {
      return;
    }

  /* Reset the state machines and control registers */

  rm57_adc_putreg(mod, RM57_ADC_RSTCR_OFFSET, ADC_RSTCR_RESET);
  rm57_adc_putreg(mod, RM57_ADC_RSTCR_OFFSET, 0);

  /* Select the resolution before anything else: it changes the layout of the
   * results, of ADCALR and of the magnitude compare registers.
   */

  opmode = ADC_OPMODECR_CHN_TEST_DIS;
  if (mod->resolution == 12)
    {
      opmode |= ADC_OPMODECR_12BIT;
    }

  rm57_adc_putreg(mod, RM57_ADC_OPMODECR_OFFSET, opmode);

  /* ADCLK = VCLK / (PS + 1) */

  rm57_adc_putreg(mod, RM57_ADC_CLOCKCR_OFFSET,
                  ADC_CLOCKCR_PS(mod->prescaler));

  /* Split the results RAM between the groups, then zero it */

  rm57_adc_partition(mod);
  rm57_adc_raminit(mod);

  /* No offset correction until a calibration run supplies one */

  rm57_adc_putreg(mod, RM57_ADC_CALR_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, 0);

  /* Leave the ADEVT pin an input with its pull enabled; a board that wants
   * it as a trigger input or a GPIO reconfigures it afterwards.
   */

  rm57_adc_putreg(mod, RM57_ADC_EVTDIR_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_EVTOUT_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_EVTPDR_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_EVTPSEL_OFFSET, ADC_EVT_BIT);
  rm57_adc_putreg(mod, RM57_ADC_EVTPDIS_OFFSET, 0);

  /* Enable the module and wait for the buffer initialization to settle */

  rm57_adc_modifyreg(mod, RM57_ADC_OPMODECR_OFFSET, 0,
                     ADC_OPMODECR_ADC_EN);

  /* Parity checking is inverted logic: 5h disables it, and that is also the
   * reset value.  Enable it only after the RAM has been zeroed, otherwise
   * the first read of an untouched buffer reports a spurious error.
   */

#ifdef CONFIG_RM57_ADC_PARITY
  rm57_adc_modifyreg(mod, RM57_ADC_PARCR_OFFSET, ADC_PARCR_PARITY_MASK,
                     ADC_PARCR_PARITY_ENABLE);
#else
  rm57_adc_modifyreg(mod, RM57_ADC_PARCR_OFFSET, ADC_PARCR_PARITY_MASK,
                     ADC_PARCR_PARITY_DISABLE);
#endif

  mod->initialized = true;

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
  /* Route the on-die temperature sensors onto their channels.  The power
   * down bit gates all three sensors at once, so this is idempotent and can
   * run from whichever module comes up first.
   */

  rm57_adc_tempmux();
#endif

  ainfo("ADC%d: %d-bit, ADCLK %luHz (VCLK/%d), %d channels\n",
        mod->module + 1, mod->resolution,
        (unsigned long)(RM57_ADC_VCLK / (mod->prescaler + 1)),
        mod->prescaler + 1, mod->nchannels);
}

/****************************************************************************
 * Name: rm57_adc_grpconfig
 *
 * Description:
 *   Apply a group's configuration to the hardware.  The channel selection
 *   register is deliberately left alone: writing a non-zero value there is
 *   what starts a software-triggered conversion, so it is programmed last,
 *   by rm57_adc_start().
 *
 ****************************************************************************/

static void rm57_adc_grpconfig(struct rm57_adcgrp_s *grp)
{
  struct rm57_adcmod_s *mod = grp->mod;
  struct rm57_adc_grpcfg_s *cfg = &grp->cfg;
  int g = grp->group;
  uint32_t modecr;
  uint32_t src;
  uint32_t sampdis;

  /* Conversion mode.  CHID is always requested so the channel number
   * travels with each result and can be handed to the upper half.
   */

  modecr = ADC_MODECR_CHID;

  if (mod->resolution == 12)
    {
      modecr |= ADC_MODECR_DATA_FMT_12BIT;
    }

  if (cfg->continuous)
    {
      modecr |= ADC_MODECR_MODE_CONT;
    }

  if (cfg->freeze)
    {
      modecr |= ADC_MODECR_FRZ;
    }

  if (cfg->ovrwrite)
    {
      modecr |= ADC_MODECR_OVR_RAM_IGN;
    }

  /* Bit 3 is the hardware trigger enable for Group1 and Group2 only; on the
   * Event group it is reserved and must be written as zero.
   */

  if (cfg->hwtrig && g != RM57_ADC_GROUP_EV)
    {
      modecr |= ADC_MODECR_HW_TRIG;
    }

  rm57_adc_putreg(mod, RM57_ADC_MODECR_OFFSET(g), modecr);

  /* Hardware trigger source and edge */

  src = ADC_SRC_SRC(cfg->trigsrc);
  if (cfg->bothedges)
    {
      src |= ADC_SRC_EDG_BOTH;
    }
  else if (cfg->rising)
    {
      src |= ADC_SRC_EDG_SEL;
    }

  rm57_adc_putreg(mod, RM57_ADC_SRC_OFFSET(g), src);

  /* Sampling window: SW = ACQ + 2 ADCLK cycles, minimum 3 */

  rm57_adc_putreg(mod, RM57_ADC_SAMP_OFFSET(g),
                  ADC_SAMP_ACQ(cfg->acq < ADC_SAMP_ACQ_MIN ?
                               ADC_SAMP_ACQ_MIN : cfg->acq));

  /* Optional sample capacitor discharge before each acquisition */

  sampdis = 0;
  if (cfg->dischargeen)
    {
      sampdis = ADC_SAMPDISEN_EN | ADC_SAMPDISEN_CYC(cfg->dischargecyc);
    }

  rm57_adc_putreg(mod, RM57_ADC_SAMPDISEN_OFFSET(g), sampdis);

  /* Threshold counter: how many results accumulate before an interrupt */

  rm57_adc_putreg(mod, RM57_ADC_THRINTCR_OFFSET(g),
                  ADC_THRINTCR_THR(cfg->threshold));

  /* Start from an empty FIFO and no stale flags */

  rm57_adc_putreg(mod, RM57_ADC_FIFORESETCR_OFFSET(g),
                  ADC_FIFORESETCR_RESET);
  rm57_adc_putreg(mod, RM57_ADC_INTFLG_OFFSET(g), ADC_INTFLG_CLRALL);
}

/****************************************************************************
 * Name: rm57_adc_start / rm57_adc_stop
 *
 * Description:
 *   Start or stop conversions on a group.  Writing a non-zero channel mask
 *   to ADxxSEL starts a software-triggered group immediately, or arms a
 *   hardware-triggered one; writing zero stops it.
 *
 ****************************************************************************/

static void rm57_adc_start(struct rm57_adcgrp_s *grp)
{
  rm57_adc_putreg(grp->mod, RM57_ADC_SEL_OFFSET(grp->group),
                  grp->cfg.chanmask);

  /* Writing the channel selection resets this group's results memory
   * pointer and its threshold counter, so the counter has to be reloaded
   * afterwards (TRM SPNU562A section 22.3.32).  The first conversion cannot
   * complete before the next store retires, so nothing is lost in between.
   */

  rm57_adc_putreg(grp->mod, RM57_ADC_THRINTCR_OFFSET(grp->group),
                  ADC_THRINTCR_THR(grp->cfg.threshold));
}

static void rm57_adc_stop(struct rm57_adcgrp_s *grp)
{
  rm57_adc_putreg(grp->mod, RM57_ADC_SEL_OFFSET(grp->group), 0);
}

/****************************************************************************
 * Name: rm57_adc_decode
 *
 * Description:
 *   Split one raw FIFO word into its channel id and conversion result.  The
 *   two fields sit in different places depending on whether the module is
 *   configured for 12- or 10-bit operation (TRM Figures 22-4 and 22-5).
 *
 ****************************************************************************/

static inline void rm57_adc_decode(struct rm57_adcmod_s *mod, uint32_t raw,
                                   uint8_t *channel, int32_t *value)
{
  if (mod->resolution == 12)
    {
      *value   = (int32_t)(raw & ADC_FIFO_12BIT_DR_MASK);
      *channel = (uint8_t)((raw & ADC_FIFO_12BIT_CHID_MASK) >>
                           ADC_FIFO_12BIT_CHID_SHIFT);
    }
  else
    {
      *value   = (int32_t)(raw & ADC_FIFO_10BIT_DR_MASK);
      *channel = (uint8_t)((raw & ADC_FIFO_10BIT_CHID_MASK) >>
                           ADC_FIFO_10BIT_CHID_SHIFT);
    }
}

/****************************************************************************
 * Name: rm57_adc_fifoempty
 *
 * Description:
 *   Whether a word read from a group FIFO carries no data because the FIFO
 *   had already run dry.  Like the result and channel id fields, the flag
 *   moves with the module's resolution.
 *
 ****************************************************************************/

static inline bool rm57_adc_fifoempty(struct rm57_adcmod_s *mod,
                                      uint32_t raw)
{
  uint32_t empty = mod->resolution == 12 ? ADC_FIFO_12BIT_EMPTY :
                                           ADC_FIFO_10BIT_EMPTY;

  return (raw & empty) != 0;
}

/****************************************************************************
 * Name: rm57_adc_drain
 *
 * Description:
 *   Move every unread conversion result from a group's FIFO to the upper
 *   half.  Reading the FIFO pops a result; once it runs dry the hardware
 *   flags the read as empty and that word carries no data.
 *
 *   The loop also stops when the upper half refuses a sample, which it does
 *   when its own FIFO is full.  Leaving the remaining results in the ADC's
 *   memory is the right response: they are still readable, and the threshold
 *   counter keeps them accounted for.  The one already in hand cannot be put
 *   back, though - reading the FIFO popped it - so it is held in the group
 *   and offered again first thing next time, rather than dropped.
 *
 * Returned Value:
 *   The number of results handed to the upper half.
 *
 ****************************************************************************/

static int rm57_adc_drain(struct rm57_adcgrp_s *grp)
{
  struct rm57_adcmod_s *mod = grp->mod;
  uintptr_t fifo = mod->base + RM57_ADC_BUFFER_OFFSET(grp->group);
  int nread = 0;
  int i;

  if (grp->cb == NULL || grp->cb->au_receive == NULL)
    {
      return 0;
    }

  /* Retry the sample a previous drain could not place */

  if (grp->held)
    {
      if (grp->cb->au_receive(&grp->dev, grp->heldchan, grp->heldvalue) < 0)
        {
          return 0;
        }

      grp->held = false;
      nread++;
    }

  /* A group can hold at most the buffers reserved for it, so the loop is
   * bounded even if the empty flag never appears.
   */

  for (i = 0; i < RM57_ADC_NRESULTBUFS; i++)
    {
      uint32_t raw = getreg32(fifo);
      uint8_t channel;
      int32_t value;

      if (rm57_adc_fifoempty(mod, raw))
        {
          break;
        }

      rm57_adc_decode(mod, raw, &channel, &value);

      if (grp->cb->au_receive(&grp->dev, channel, value) < 0)
        {
          grp->held      = true;
          grp->heldchan  = channel;
          grp->heldvalue = value;
          break;
        }

      nread++;
    }

  return nread;
}

/****************************************************************************
 * Name: rm57_adc_interrupt
 *
 * Description:
 *   Handle one conversion group's interrupt.  A group raises this line for
 *   three reasons: its threshold counter reached zero, its results memory
 *   overran, or - in single conversion mode - it finished its channel list.
 *
 ****************************************************************************/

static int rm57_adc_interrupt(int irq, void *context, void *arg)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)arg;
  struct rm57_adcmod_s *mod;
  uint32_t flags;
  bool bycpu;
  int g;

  DEBUGASSERT(grp != NULL);

  mod = grp->mod;
  g   = grp->group;

  /* Whether this handler owns the FIFO.  With DMA moving the results the
   * only interrupt the CPU asks for is the overrun, and reading the FIFO
   * here would race the DMA engine for the same words.
   */

#ifdef CONFIG_RM57_ADC_HAVE_DMA
  bycpu = !grp->usedma;
#else
  bycpu = true;
#endif

  flags = rm57_adc_getreg(mod, RM57_ADC_INTFLG_OFFSET(g));

  /* An overrun means results were produced faster than they were read.  The
   * upper half is reset so the reader restarts from a coherent stream rather
   * than silently consuming a mixture of old and new samples.
   */

  if ((flags & ADC_INTFLG_MEM_OVERRUN) != 0)
    {
      grp->overruns++;
      grp->held = false;

      if (grp->cb != NULL && grp->cb->au_reset != NULL)
        {
          grp->cb->au_reset(&grp->dev);
        }

      /* Release the group's memory so conversions can resume.  Without the
       * FIFO reset the ADC stalls until every stale result is read out.
       * The stream has already been declared broken by au_reset(), so in the
       * DMA case the reset alone is enough - the half-filled ring is dropped
       * with it rather than being raced for.
       */

      if (bycpu)
        {
          rm57_adc_drain(grp);
        }

      rm57_adc_putreg(mod, RM57_ADC_FIFORESETCR_OFFSET(g),
                      ADC_FIFORESETCR_RESET);
    }
  else if (bycpu && (flags & (ADC_INTFLG_THR | ADC_INTFLG_END)) != 0)
    {
      rm57_adc_drain(grp);
    }

  /* Re-arm the threshold counter.  It counts down as results are written and
   * back up as they are read, but a partially drained FIFO would otherwise
   * leave it short of the configured value.
   */

  if ((flags & ADC_INTFLG_THR) != 0)
    {
      rm57_adc_putreg(mod, RM57_ADC_THRINTCR_OFFSET(g),
                      ADC_THRINTCR_THR(grp->cfg.threshold));
    }

  /* Clear the write-1-to-clear flags we acted on */

  rm57_adc_putreg(mod, RM57_ADC_INTFLG_OFFSET(g),
                  flags & ADC_INTFLG_CLRALL);

  /* A single-conversion group stops once its channel list is done.  Restart
   * it so a reader that keeps the device open keeps receiving samples.
   */

  if ((flags & ADC_INTFLG_END) != 0 && !grp->cfg.continuous &&
      !rm57_adc_ishwtrig(grp))
    {
      rm57_adc_start(grp);
    }

  return OK;
}

#ifdef CONFIG_RM57_ADC_HAVE_DMA

/****************************************************************************
 * Name: rm57_adc_dmabuffer
 *
 * Description:
 *   The static DMA landing ring belonging to one group, or NULL if that
 *   group does not use DMA in this configuration.
 *
 ****************************************************************************/

static uint32_t *rm57_adc_dmabuffer(struct rm57_adcgrp_s *grp)
{
#ifdef CONFIG_RM57_ADC1_EV_DMA
  if (grp == &g_adc1_ev)
    {
      return g_adc1_ev_dmabuf;
    }
#endif

#ifdef CONFIG_RM57_ADC1_G1_DMA
  if (grp == &g_adc1_g1)
    {
      return g_adc1_g1_dmabuf;
    }
#endif

#ifdef CONFIG_RM57_ADC1_G2_DMA
  if (grp == &g_adc1_g2)
    {
      return g_adc1_g2_dmabuf;
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: rm57_adc_dmacallback
 *
 * Description:
 *   Deliver one half of the DMA ring to the upper half.  The channel is
 *   configured with auto-initiation and both the half- and block-complete
 *   interrupts armed, so the two halves alternate: HBC signals that the
 *   first half is full while the ADC is filling the second, and BTC the
 *   other way round.
 *
 ****************************************************************************/

static void rm57_adc_dmacallback(DMA_HANDLE handle, uint8_t status,
                                 void *arg)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)arg;
  struct rm57_adcmod_s *mod = grp->mod;
  uint8_t channel[CONFIG_RM57_ADC_DMA_MAXSAMPLES];
  uint32_t data[CONFIG_RM57_ADC_DMA_MAXSAMPLES];
  uint32_t *src;
  size_t count = grp->dmansamples;
  size_t i;

  if ((status & RM57_DMA_STATUS_SUCCESS) == 0)
    {
      return;
    }

  /* HBC means the first half just filled, BTC the second */

  src = grp->dmabuf;
  if ((status & RM57_DMA_STATUS_BTC) != 0)
    {
      src += grp->dmansamples;
    }

  /* The ADC wrote these words behind the D-cache, so drop any stale lines
   * before reading them.
   */

  up_invalidate_dcache((uintptr_t)src,
                       (uintptr_t)src + count * sizeof(uint32_t));

  for (i = 0; i < count; i++)
    {
      int32_t value;

      rm57_adc_decode(mod, src[i], &channel[i], &value);
      data[i] = (uint32_t)value;
    }

  /* A full upper half FIFO is an overrun by another name: the DMA engine is
   * already filling the other half of the ring and nothing can hold these
   * back, so account for what is lost the same way the CPU path does.
   */

  if (grp->cb != NULL && grp->cb->au_receive_batch != NULL)
    {
      if (grp->cb->au_receive_batch(&grp->dev, channel, data, count) < 0)
        {
          grp->overruns++;
        }
    }
  else if (grp->cb != NULL && grp->cb->au_receive != NULL)
    {
      for (i = 0; i < count; i++)
        {
          if (grp->cb->au_receive(&grp->dev, channel[i],
                                  (int32_t)data[i]) < 0)
            {
              grp->overruns++;
              break;
            }
        }
    }
}

/****************************************************************************
 * Name: rm57_adc_dmasetup
 *
 * Description:
 *   Claim a DMA channel for one group and arm it as a two-half ring fed by
 *   that group's results FIFO.  Each DMA request moves one 32-bit result
 *   from the fixed FIFO address into the advancing ring.
 *
 ****************************************************************************/

static int rm57_adc_dmasetup(struct rm57_adcgrp_s *grp)
{
  struct rm57_dmacfg_s cfg;

  grp->dmabuf = rm57_adc_dmabuffer(grp);
  if (grp->dmabuf == NULL)
    {
      return -ENOMEM;
    }

  grp->dma = rm57_dmachannel(grp->dmareq);
  if (grp->dma == NULL)
    {
      aerr("ERROR: No DMA channel for ADC%d group %d\n",
           grp->mod->module + 1, grp->group);
      grp->dmabuf = NULL;
      return -EBUSY;
    }

  /* One element per frame, one frame per request.  TTYPE is left clear,
   * which selects frame triggering: each DMA request transfers a whole
   * frame, and the ADC raises exactly one request per result it writes.  So
   * the frame has to be a single word and the ring has to be counted in
   * frames - the same shape the SCI receive ring uses for its byte-per-
   * request source in rm57_serial.c.  Sizing it the other way round would
   * make one result drain ADxxBUFFER dmansamples times over, and everything
   * after the first read would land in the ring as an empty-FIFO word.
   */

  cfg.saddr     = grp->mod->base + RM57_ADC_BUFFER_OFFSET(grp->group);
  cfg.daddr     = (uint32_t)(uintptr_t)grp->dmabuf;
  cfg.nframes   = 2 * grp->dmansamples;
  cfg.nelems    = 1;
  cfg.chctrl    = DMA_CHCTRL_RES_32BIT | DMA_CHCTRL_WES_32BIT |
                  DMA_CHCTRL_ADDMR_CONST | DMA_CHCTRL_ADDMW_INCR |
                  DMA_CHCTRL_AIM;
  cfg.parassign = DMA_PAR_AB_B_RD_A_WR;

  rm57_dmasetup(grp->dma, &cfg);
  rm57_dmastart(grp->dma, rm57_adc_dmacallback, grp, true);

  /* Ask the ADC for one request per result written to its memory.  Block
   * mode is deliberately not used: its block count shares a field with the
   * threshold counter, and the ring already batches the delivery.
   */

  rm57_adc_putreg(grp->mod, RM57_ADC_DMACR_OFFSET(grp->group),
                  ADC_DMACR_DMA_EN);

  return OK;
}

/****************************************************************************
 * Name: rm57_adc_dmastop
 ****************************************************************************/

static void rm57_adc_dmastop(struct rm57_adcgrp_s *grp)
{
  rm57_adc_putreg(grp->mod, RM57_ADC_DMACR_OFFSET(grp->group), 0);

  if (grp->dma != NULL)
    {
      rm57_dmastop(grp->dma);
      rm57_dmafree(grp->dma);
      grp->dma = NULL;
    }
}

#endif /* CONFIG_RM57_ADC_HAVE_DMA */

/****************************************************************************
 * Name: rm57_adc_calconvert
 *
 * Description:
 *   Run one calibration conversion of the reference voltage selected by
 *   'refsel' and return the raw result.  There is no interrupt for this, so
 *   CAL_ST has to be polled: the hardware holds it set for the duration of
 *   the conversion and clears it when the result lands in ADCALR.
 *
 *   The caller must already hold the module lock and have placed the module
 *   in calibration mode with every channel selection cleared.
 *
 ****************************************************************************/

static int rm57_adc_calconvert(struct rm57_adcmod_s *mod, uint32_t refsel,
                               uint16_t *raw)
{
  int i;

  /* Select the reference, then enable calibration and start the conversion.
   * CAL_EN has to be set before CAL_ST or the start request is ignored.
   */

  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, refsel);
  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, refsel | ADC_CALCR_CAL_EN);
  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET,
                  refsel | ADC_CALCR_CAL_EN | ADC_CALCR_CAL_ST);

  for (i = 0; i < RM57_ADC_TIMEOUT; i++)
    {
      if ((rm57_adc_getreg(mod, RM57_ADC_CALCR_OFFSET) &
           ADC_CALCR_CAL_ST) == 0)
        {
          break;
        }
    }

  if (i >= RM57_ADC_TIMEOUT)
    {
      aerr("ERROR: ADC%d calibration conversion timed out\n",
           mod->module + 1);
      return -ETIMEDOUT;
    }

  /* ADCALR must be read before the next conversion overwrites it */

  *raw = (uint16_t)(rm57_adc_getreg(mod, RM57_ADC_CALR_OFFSET) &
                    rm57_adc_fullscale(mod));
  return OK;
}

/****************************************************************************
 * Name: rm57_adc_calprologue / rm57_adc_calepilogue
 *
 * Description:
 *   Enter and leave the state the calibration and self test sequences need:
 *   the core powered, no group converting, self test off, and the module's
 *   own settings saved so the caller's configuration survives.
 *
 *   ADEVSAMP is part of that saved state even though the Event group may not
 *   be in the configuration at all, because the calibration conversion is
 *   timed by it whichever group the caller was using (TRM SPNU562A section
 *   22.2.6.1.1).  Left at its reset value it gives a two cycle window, short
 *   of both the ~1us minimum sampling time and the hardware's own SW >= 3,
 *   and the offset error computed from it would then be loaded into ADCALR
 *   and added to every subsequent conversion.
 *
 ****************************************************************************/

static void rm57_adc_calprologue(struct rm57_adcmod_s *mod,
                                 struct rm57_adc_calsave_s *saved)
{
  int g;

  rm57_adc_powerup(mod);

  for (g = 0; g < RM57_ADC_NGROUPS; g++)
    {
      saved->sel[g] = rm57_adc_getreg(mod, RM57_ADC_SEL_OFFSET(g));
      rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), 0);
    }

  saved->evsamp = rm57_adc_getreg(mod, RM57_ADC_EVSAMP_OFFSET);
  rm57_adc_putreg(mod, RM57_ADC_EVSAMP_OFFSET,
                  ADC_SAMP_ACQ(RM57_ADC_CAL_ACQ(mod->prescaler)));

  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, 0);
}

static void rm57_adc_calepilogue(struct rm57_adcmod_s *mod,
                                 const struct rm57_adc_calsave_s *saved)
{
  int g;

  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_EVSAMP_OFFSET, saved->evsamp);

  for (g = 0; g < RM57_ADC_NGROUPS; g++)
    {
      rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), saved->sel[g]);

      /* Restarting a group resets its results memory pointer and its
       * threshold counter together (TRM section 22.3.32), so the counter has
       * to be put back too or the group's reader silently loses its
       * interrupt cadence for the rest of its life.
       */

      if (saved->sel[g] != 0 && mod->grp[g] != NULL)
        {
          rm57_adc_putreg(mod, RM57_ADC_THRINTCR_OFFSET(g),
                          ADC_THRINTCR_THR(mod->grp[g]->cfg.threshold));
        }
    }

  rm57_adc_idlepowerdown(mod);
}

/****************************************************************************
 * Name: rm57_adc_calibrate
 *
 * Description:
 *   Offset error calibration, TRM SPNU562A section 22.2.6.1.2.  The four
 *   internal reference voltages are converted in turn; their average should
 *   land on mid scale, and whatever it misses by is the offset error.  The
 *   two's complement of that error goes into ADCALR, which the hardware then
 *   adds to every conversion result.
 *
 ****************************************************************************/

int rm57_adc_calibrate(int module, struct rm57_adc_calib_s *calib)
{
  static const uint32_t refsel[4] =
  {
    ADC_CALCR_REF_MIDHI,
    ADC_CALCR_REF_MIDLO,
    ADC_CALCR_REF_LO,
    ADC_CALCR_REF_HI,
  };

  struct rm57_adcmod_s *mod = rm57_adc_module(module);
  struct rm57_adc_calsave_s saved;
  uint16_t raw[4];
  uint32_t sum = 0;
  int32_t offset;
  int32_t correction;
  uint16_t midscale;
  int ret;
  int i;

  if (mod == NULL || !mod->initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&mod->lock);
  if (ret < 0)
    {
      return ret;
    }

  rm57_adc_calprologue(mod, &saved);

  for (i = 0; i < 4; i++)
    {
      ret = rm57_adc_calconvert(mod, refsel[i], &raw[i]);
      if (ret < 0)
        {
          goto errout;
        }

      sum += raw[i];
    }

  /* Averaging the mid-high, mid-low, low and high references reconstructs
   * mid scale; the difference is the offset the converter is carrying.
   */

  midscale   = rm57_adc_fullscale(mod) >> 1;
  offset     = (int32_t)(sum / 4) - (int32_t)midscale;
  correction = (-offset) & rm57_adc_fullscale(mod);

  rm57_adc_putreg(mod, RM57_ADC_CALR_OFFSET, (uint32_t)correction);

  if (calib != NULL)
    {
      for (i = 0; i < 4; i++)
        {
          calib->raw[i] = raw[i];
        }

      calib->offset     = (int16_t)offset;
      calib->correction = (int16_t)correction;
    }

  ainfo("ADC%d: calibration offset %" PRId32 ", correction %" PRId32 "\n",
        module + 1, offset, correction);

errout:
  rm57_adc_calepilogue(mod, &saved);
  nxmutex_unlock(&mod->lock);
  return ret;
}

/****************************************************************************
 * Name: rm57_adc_midpointcal
 *
 * Description:
 *   Differential mid-point calibration, TRM SPNU562A section 22.2.6.1.3.
 *   The internal bridge resistors are only matched to about 1.5%, which
 *   skews the mid-point reference.  Measuring it twice with the two rails
 *   swapped and averaging cancels that mismatch.
 *
 ****************************************************************************/

int rm57_adc_midpointcal(int module, struct rm57_adc_calib_s *calib)
{
  struct rm57_adcmod_s *mod = rm57_adc_module(module);
  struct rm57_adc_calsave_s saved;
  uint16_t cal1;
  uint16_t cal2;
  int32_t offset;
  int32_t correction;
  uint16_t midscale;
  int ret;

  if (mod == NULL || !mod->initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&mod->lock);
  if (ret < 0)
    {
      return ret;
    }

  rm57_adc_calprologue(mod, &saved);

  ret = rm57_adc_calconvert(mod, ADC_CALCR_REF_MIDHI, &cal1);
  if (ret < 0)
    {
      goto errout;
    }

  ret = rm57_adc_calconvert(mod, ADC_CALCR_REF_MIDLO, &cal2);
  if (ret < 0)
    {
      goto errout;
    }

  midscale   = rm57_adc_fullscale(mod) >> 1;
  offset     = (int32_t)(((uint32_t)cal1 + cal2) / 2) - (int32_t)midscale;
  correction = (-offset) & rm57_adc_fullscale(mod);

  rm57_adc_putreg(mod, RM57_ADC_CALR_OFFSET, (uint32_t)correction);

  if (calib != NULL)
    {
      calib->raw[0]     = cal1;
      calib->raw[1]     = cal2;
      calib->raw[2]     = 0;
      calib->raw[3]     = 0;
      calib->offset     = (int16_t)offset;
      calib->correction = (int16_t)correction;
    }

  ainfo("ADC%d: mid-point offset %" PRId32 ", correction %" PRId32 "\n",
        module + 1, offset, correction);

errout:
  rm57_adc_calepilogue(mod, &saved);
  nxmutex_unlock(&mod->lock);
  return ret;
}

/****************************************************************************
 * Name: rm57_adc_oneshot
 *
 * Description:
 *   Convert one channel a single time and return the raw result.  'calcr' is
 *   written to ADCALCR for the duration, which is how the caller asks for
 *   self test mode; passing zero performs an ordinary conversion.
 *
 *   Group 'g' is borrowed for this.  Its channel selection, mode and
 *   interrupt enables are saved and restored, so the conversion is invisible
 *   to whatever the group was doing.  Silencing the group matters as much as
 *   parking its channel selection, and it takes both halves: masking the
 *   ADC's own interrupt enables stops new requests, and disabling the VIM
 *   channel stops one that was already latched.  Either way through, the
 *   group's handler would otherwise run rm57_adc_drain() and pop this
 *   conversion's result before it could be read.
 *
 *   The group's own buffered results are left alone.  They belong to its
 *   reader and resetting the FIFO to tidy up after this conversion would
 *   punch a silent hole in an unrelated /dev/adcN stream; a single word is
 *   read back out instead, which is exactly what this conversion put there.
 *
 *   The caller must hold the module lock.
 *
 ****************************************************************************/

static int rm57_adc_oneshot(struct rm57_adcmod_s *mod, int g,
                            uint8_t channel, uint32_t calcr, uint16_t *raw)
{
  uintptr_t fifo = mod->base + RM57_ADC_BUFFER_OFFSET(g);
  struct rm57_adcgrp_s *grp = mod->grp[g];
  uint32_t savedsel;
  uint32_t savedmode;
  uint32_t savedint;
  uint32_t word;
  uint8_t chid;
  int32_t value;
  bool masked = false;
  int ret = OK;
  int i;

  rm57_adc_powerup(mod);

  savedsel  = rm57_adc_getreg(mod, RM57_ADC_SEL_OFFSET(g));
  savedmode = rm57_adc_getreg(mod, RM57_ADC_MODECR_OFFSET(g));
  savedint  = rm57_adc_getreg(mod, RM57_ADC_INTENA_OFFSET(g));

  /* Park the group and silence it before taking it over */

  if (grp != NULL && grp->setup)
    {
      up_disable_irq(grp->irq);
      masked = true;
    }

  rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), 0);
  rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(g), 0);

  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, calcr);

  /* Single conversion of just this channel, results tagged with the id.
   * ADxxSR.END is write-1-to-clear and lives in its own register, so clear
   * it explicitly rather than relying on the channel selection write to do
   * it - the poll below is only meaningful if it starts out clear.
   */

  rm57_adc_putreg(mod, RM57_ADC_MODECR_OFFSET(g), ADC_MODECR_CHID);
  rm57_adc_putreg(mod, RM57_ADC_INTFLG_OFFSET(g), ADC_INTFLG_CLRALL);
  rm57_adc_putreg(mod, RM57_ADC_SR_OFFSET(g), ADC_SR_END);
  rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), (uint32_t)1 << channel);

  for (i = 0; i < RM57_ADC_TIMEOUT; i++)
    {
      if ((rm57_adc_getreg(mod, RM57_ADC_SR_OFFSET(g)) & ADC_SR_END) != 0)
        {
          break;
        }
    }

  rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), 0);

  if (i >= RM57_ADC_TIMEOUT)
    {
      aerr("ERROR: ADC%d one-shot conversion timed out\n",
           mod->module + 1);
      ret = -ETIMEDOUT;
    }
  else
    {
      word = getreg32(fifo);
      if (rm57_adc_fifoempty(mod, word))
        {
          ret = -EIO;
        }
      else
        {
          rm57_adc_decode(mod, word, &chid, &value);
          *raw = (uint16_t)value;
        }
    }

  /* Leave nothing of this conversion behind for the group's own reader */

  rm57_adc_putreg(mod, RM57_ADC_CALCR_OFFSET, 0);
  rm57_adc_putreg(mod, RM57_ADC_INTFLG_OFFSET(g), ADC_INTFLG_CLRALL);
  rm57_adc_putreg(mod, RM57_ADC_SR_OFFSET(g), ADC_SR_END);

  rm57_adc_putreg(mod, RM57_ADC_MODECR_OFFSET(g), savedmode);
  rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(g), savedint);
  rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), savedsel);

  if (savedsel != 0)
    {
      /* Restarting the group reset its threshold counter along with its
       * memory pointer, so reload it (TRM section 22.3.32).
       */

      rm57_adc_putreg(mod, RM57_ADC_THRINTCR_OFFSET(g),
                      ADC_THRINTCR_THR(grp != NULL ?
                                       grp->cfg.threshold : 1));
    }

  if (masked)
    {
      up_enable_irq(grp->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: rm57_adc_diaggroup
 *
 * Description:
 *   Pick a conversion group to run a diagnostic conversion in.  Two things
 *   disqualify a group.
 *
 *   It has to own results memory: a group with no buffers reserved has a
 *   zero boundary, and the TRM warns that converting into it makes the ADC
 *   fall back to a 1024-word default region, which would run straight over
 *   the next group's results.
 *
 *   And it has to be startable from software.  The Event group is not - "the
 *   Event Group is always hardware event-triggered" (TRM SPNU562A section
 *   22.2.2), so writing ADEVSEL only arms it and the conversion waits for a
 *   trigger that a diagnostic sequence has no way to produce.  Borrowing it
 *   would spin out the whole poll timeout with the module lock held and then
 *   fail, every time.
 *
 * Returned Value:
 *   A group index, or -ENOSPC if this module has no usable group.
 *
 ****************************************************************************/

static int rm57_adc_diaggroup(struct rm57_adcmod_s *mod)
{
  int g;

  for (g = 0; g < RM57_ADC_NGROUPS; g++)
    {
      if (g != RM57_ADC_GROUP_EV && rm57_adc_nbuffers(mod->module, g) > 0)
        {
          return g;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: rm57_adc_selftest
 *
 * Description:
 *   Detect an open circuit or a short on one analog input, TRM SPNU562A
 *   section 22.2.6.2.  In self test mode the selected reference is driven
 *   onto the input through an internal resistor while the external signal is
 *   still connected, so the two sources fight each other:
 *
 *     - A healthy channel is pulled some way towards each reference but is
 *       held back by whatever is driving it, so both readings stay away from
 *       the rails.
 *     - An open channel has nothing opposing the test source, so it follows
 *       it completely: low with ADREFLO, high with ADREFHI.
 *     - A channel shorted to a rail cannot be moved at all and reads the
 *       same both times.
 *
 ****************************************************************************/

int rm57_adc_selftest(int module, struct rm57_adc_selftest_s *test)
{
  struct rm57_adcmod_s *mod = rm57_adc_module(module);
  struct rm57_adc_calsave_s saved;
  uint16_t full;
  uint16_t lowthr;
  uint16_t highthr;
  int ret;
  int g;

  if (mod == NULL || !mod->initialized || test == NULL)
    {
      return -ENODEV;
    }

  if (!rm57_adc_validchan(mod, test->channel))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&mod->lock);
  if (ret < 0)
    {
      return ret;
    }

  g = rm57_adc_diaggroup(mod);
  if (g < 0)
    {
      nxmutex_unlock(&mod->lock);
      return g;
    }

  /* Self test mode is module-wide: while it is on, every group converts
   * against the internal test source, not its own input.  So unlike a plain
   * one-shot, all three groups have to be parked for the duration, which is
   * what the calibration prologue does.
   */

  rm57_adc_calprologue(mod, &saved);

  ret = rm57_adc_oneshot(mod, g, test->channel, ADC_CALCR_SELF_TEST,
                         &test->lo);
  if (ret < 0)
    {
      goto errout;
    }

  ret = rm57_adc_oneshot(mod, g, test->channel,
                         ADC_CALCR_SELF_TEST | ADC_CALCR_HILO, &test->hi);
  if (ret < 0)
    {
      goto errout;
    }

  full    = rm57_adc_fullscale(mod);
  lowthr  = RM57_ADC_ST_LOW(full);
  highthr = RM57_ADC_ST_HIGH(full);

  if (test->lo <= lowthr && test->hi >= highthr)
    {
      test->status = RM57_ADC_SELFTEST_OPEN;
    }
  else if (test->lo <= lowthr && test->hi <= lowthr)
    {
      test->status = RM57_ADC_SELFTEST_SHORT_GND;
    }
  else if (test->lo >= highthr && test->hi >= highthr)
    {
      test->status = RM57_ADC_SELFTEST_SHORT_PWR;
    }
  else
    {
      test->status = RM57_ADC_SELFTEST_OK;
    }

  ainfo("ADC%d ch%u self test: lo=%u hi=%u status=%u\n",
        module + 1, test->channel, test->lo, test->hi, test->status);

errout:
  rm57_adc_calepilogue(mod, &saved);
  nxmutex_unlock(&mod->lock);
  return ret;
}

#ifdef CONFIG_RM57_ADC_TEMPSENSOR

/****************************************************************************
 * Name: rm57_adc_tempmux
 *
 * Description:
 *   Route the enabled temperature sensors onto their ADC channels.  Two
 *   things have to happen: the sensors have to come out of power down, which
 *   is one bit gating all three, and each sensor has to be selected in place
 *   of the ball it shares its channel with.
 *
 *   This is deliberately driven from the driver rather than from the board's
 *   pin-mux table, because which sensors are wanted is a Kconfig decision: a
 *   board that needs AD1IN[31] as a real analog input just leaves
 *   CONFIG_RM57_ADC_TEMP1 unset and keeps the ball.
 *
 ****************************************************************************/

/* Which module and channel each on-die sensor is multiplexed onto, and which
 * of them this configuration actually asked for.  A sensor that was not
 * selected keeps its ball as an ordinary analog input, so converting its
 * channel would return that ball's voltage - and then scale it with the
 * sensor's calibration, which is a plausible-looking lie.  Hence the mask.
 */

static const uint8_t g_tempmod[RM57_ADC_NTEMPSENSORS] =
{
  RM57_ADC_TEMP1_MODULE, RM57_ADC_TEMP2_MODULE, RM57_ADC_TEMP3_MODULE
};

static const uint8_t g_tempchan[RM57_ADC_NTEMPSENSORS] =
{
  RM57_ADC_TEMP1_CHANNEL, RM57_ADC_TEMP2_CHANNEL, RM57_ADC_TEMP3_CHANNEL
};

static const bool g_tempsel[RM57_ADC_NTEMPSENSORS] =
{
#ifdef CONFIG_RM57_ADC_TEMP1
  true,
#else
  false,
#endif
#ifdef CONFIG_RM57_ADC_TEMP2
  true,
#else
  false,
#endif
#ifdef CONFIG_RM57_ADC_TEMP3
  true,
#else
  false,
#endif
};

static inline bool rm57_adc_tempenabled(int sensor)
{
  return sensor >= 0 && sensor < RM57_ADC_NTEMPSENSORS && g_tempsel[sensor];
}

static void rm57_adc_tempmux(void)
{
  static const struct rm57_pinmux_s g_tempmux[] =
  {
    PINMUX_TEMP_PWRDN_OFF_PIN,
#ifdef CONFIG_RM57_ADC_TEMP1
    PINMUX_TEMP1_ENABLE_ON_PIN,
#endif
#ifdef CONFIG_RM57_ADC_TEMP2
    PINMUX_TEMP2_ENABLE_ON_PIN,
#endif
#ifdef CONFIG_RM57_ADC_TEMP3
    PINMUX_TEMP3_ENABLE_ON_PIN,
#endif
  };

  rm57_iomm_configmultiple(g_tempmux,
                           sizeof(g_tempmux) / sizeof(g_tempmux[0]));
}

/****************************************************************************
 * Name: rm57_adc_tempcal
 *
 * Description:
 *   Read one sensor's factory calibration out of TI OTP and sort it into
 *   ascending temperature.  Each sensor has three (Kelvin, ADC reading)
 *   points recorded during device test, packed as 16-bit fields, so one
 *   32-bit load fetches a whole point.
 *
 *   The sort is not decoration.  Silicon records the points in the order
 *   they were measured - +30C, then -40C, then +125C - so the raw sequence
 *   is not monotonic in either variable and interpolating across it directly
 *   would pick nonsense segments.
 *
 *   Reading OTP needs no flash mode change, but it does need the region to
 *   be mapped strongly-ordered or device rather than normal - otherwise
 *   errata L2FMC#5 returns the instruction opcode in place of the data.
 *   rm57_periph_region() already maps this range strongly-ordered.
 *
 * Returned Value:
 *   True if the calibration looks usable: programmed (unprogrammed OTP reads
 *   as all ones), the temperatures plausible, and the readings strictly
 *   monotonic across them so the interpolation below has a slope to work
 *   with.
 *
 *   Monotonic in either direction, deliberately.  Neither the TRM nor
 *   SPNS215 says whether the sensor's output rises or falls with
 *   temperature - measured silicon rises, at about 6 codes per degree, but
 *   that is one device - and rejecting a falling characteristic would report
 *   it as uncalibrated, which is indistinguishable from blank OTP.
 *
 ****************************************************************************/

static bool rm57_adc_tempcal(int sensor, uint16_t *val, uint16_t *kelvin)
{
  uintptr_t base = RM57_ADC_TEMPCAL_BASE(sensor);
  bool rising;
  int i;
  int j;

  /* Read every point before judging any of them, so that a caller reporting
   * a rejected calibration has the whole thing to show rather than the
   * prefix that happened to pass.
   */

  for (i = 0; i < RM57_ADC_TEMPCAL_NPOINTS; i++)
    {
      uint32_t word = getreg32(base + i * 4);

      kelvin[i] = (uint16_t)(word & RM57_ADC_TEMPCAL_KELVIN_MASK);
      val[i]    = (uint16_t)(word >> RM57_ADC_TEMPCAL_VAL_SHIFT);
    }

  for (i = 0; i < RM57_ADC_TEMPCAL_NPOINTS; i++)
    {
      if (val[i] == RM57_ADC_TEMPCAL_BLANK ||
          kelvin[i] == RM57_ADC_TEMPCAL_BLANK)
        {
          return false;
        }

      /* A "temperature" outside any range this device is tested at means the
       * halves are not where they are expected, or the OTP read did not
       * return OTP at all.
       */

      if (kelvin[i] < RM57_ADC_TEMPCAL_MINK ||
          kelvin[i] > RM57_ADC_TEMPCAL_MAXK)
        {
          return false;
        }
    }

  /* Insertion sort into ascending temperature.  Three points. */

  for (i = 1; i < RM57_ADC_TEMPCAL_NPOINTS; i++)
    {
      uint16_t k = kelvin[i];
      uint16_t v = val[i];

      for (j = i; j > 0 && kelvin[j - 1] > k; j--)
        {
          kelvin[j] = kelvin[j - 1];
          val[j]    = val[j - 1];
        }

      kelvin[j] = k;
      val[j]    = v;
    }

  /* The readings must now be ordered too; a repeated one would divide by
   * zero, and a fold-back would make the segments meaningless.
   */

  rising = val[1] > val[0];

  for (i = 1; i < RM57_ADC_TEMPCAL_NPOINTS; i++)
    {
      if (kelvin[i] == kelvin[i - 1] ||
          (rising ? val[i] <= val[i - 1] : val[i] >= val[i - 1]))
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: rm57_adc_tempconvert
 *
 * Description:
 *   Turn a raw conversion into milli-Celsius using the calibration points,
 *   which rm57_adc_tempcal() has put in ascending temperature order.  The
 *   response is close enough to linear that piecewise interpolation between
 *   the recorded points is the whole story: pick the segment the reading
 *   falls in, or the nearest one and extrapolate along it.  On the device
 *   this was measured on the two segments span -40C to +30C and +30C to
 *   +125C, at roughly six codes per degree.
 *
 *   Note the calibration was taken with ADREFHI at 3.3V.  On a board with a
 *   different reference the readings would need scaling first; LAUNCHXL2
 *   ties ADREFHI to the 3.3V analog supply, so no correction is applied.
 *
 ****************************************************************************/

static int32_t rm57_adc_tempconvert(uint16_t raw, const uint16_t *val,
                                    const uint16_t *kelvin)
{
  bool rising = val[1] > val[0];
  int lo = 0;
  int64_t millik;

  /* Segment 0 covers everything on the cold side of point 1 and segment 1
   * everything on the warm side; readings outside the calibrated span
   * extrapolate along the near segment.  Which side of val[1] "cold" is
   * depends on the direction of the characteristic - see rm57_adc_tempcal().
   */

  if (rising ? raw > val[1] : raw < val[1])
    {
      lo = 1;
    }

  millik = (int64_t)kelvin[lo] * 1000 +
           ((int64_t)(raw - val[lo]) *
            ((int64_t)kelvin[lo + 1] - kelvin[lo]) * 1000) /
           ((int64_t)val[lo + 1] - val[lo]);

  return (int32_t)(millik - RM57_ADC_ZEROC_MILLIK);
}

/****************************************************************************
 * Name: rm57_adc_readtemp
 *
 * Description:
 *   Convert one on-die temperature sensor and report the result.  The
 *   sensor's channel is converted once on a borrowed group, then scaled by
 *   the factory calibration.
 *
 *   If OTP holds no usable calibration the raw reading is still returned,
 *   with 'calibrated' clear and 'millicel' zero - inventing a temperature
 *   from an uncalibrated curve would be worse than admitting there is none.
 *
 ****************************************************************************/

int rm57_adc_readtemp(int sensor, struct rm57_adc_temp_s *temp)
{
  struct rm57_adcmod_s *mod;
  uint16_t val[RM57_ADC_TEMPCAL_NPOINTS];
  uint16_t kelvin[RM57_ADC_TEMPCAL_NPOINTS];
  int ret;
  int g;

  if (temp == NULL || !rm57_adc_tempenabled(sensor))
    {
      return -EINVAL;
    }

  mod = rm57_adc_module(g_tempmod[sensor]);
  if (mod == NULL || !mod->initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&mod->lock);
  if (ret < 0)
    {
      return ret;
    }

  g = rm57_adc_diaggroup(mod);
  if (g < 0)
    {
      nxmutex_unlock(&mod->lock);
      return g;
    }

  /* An ordinary conversion, so calcr is zero: unlike self test, reading a
   * temperature sensor does not disturb the other groups and they are left
   * running.
   */

  ret = rm57_adc_oneshot(mod, g, g_tempchan[sensor], 0, &temp->raw);
  rm57_adc_idlepowerdown(mod);
  nxmutex_unlock(&mod->lock);

  if (ret < 0)
    {
      return ret;
    }

  temp->sensor     = (uint8_t)sensor;
  temp->calibrated = rm57_adc_tempcal(sensor, val, kelvin);
  temp->millicel   = temp->calibrated ?
                     rm57_adc_tempconvert(temp->raw, val, kelvin) : 0;

  return OK;
}

/****************************************************************************
 * Name: rm57_adc_tempcal_points
 *
 * Description:
 *   Hand back one sensor's factory calibration points so a caller can report
 *   them.  This exists for bring-up more than for measurement: when a sensor
 *   comes back uncalibrated there is otherwise no way to tell blank OTP from
 *   a curve the driver rejected, and it is also the port's only read of the
 *   OTP region - so seeing plausible Kelvin values here is what proves that
 *   errata L2FMC#5 is not returning instruction opcodes instead of data.
 *
 ****************************************************************************/

int rm57_adc_tempcal_points(int sensor, uint16_t *val, uint16_t *kelvin)
{
  if (val == NULL || kelvin == NULL || !rm57_adc_tempenabled(sensor))
    {
      return -EINVAL;
    }

  return rm57_adc_tempcal(sensor, val, kelvin) ? OK : -ENODATA;
}

#endif /* CONFIG_RM57_ADC_TEMPSENSOR */

#ifdef CONFIG_RM57_ADC_MAGCOMP

/****************************************************************************
 * Name: rm57_adc_magcomp_interrupt
 *
 * Description:
 *   Service a module's magnitude compare interrupt.  This is its own VIM
 *   channel, separate from the three conversion group lines - 31 for MibADC1
 *   and 59 for MibADC2 (SPNS215 Table 6-39) - so without a handler here the
 *   comparators would set their flag, assert the line and then have nothing
 *   ever acknowledge them.
 *
 *   Reading ADMAGINTOFF both identifies the highest priority pending
 *   comparator and clears its flag, so the loop drains them in priority
 *   order.  The offset register reads zero when none is pending.
 *
 *   What the driver can do with the event is limited: the NuttX ADC upper
 *   half has no notification path for an analog watchdog, so the crossings
 *   are counted and reported through the debug log rather than pushed to the
 *   application.
 *
 ****************************************************************************/

static int rm57_adc_magcomp_interrupt(int irq, void *context, void *arg)
{
  struct rm57_adcmod_s *mod = (struct rm57_adcmod_s *)arg;
  uint32_t offset;
  int i;

  DEBUGASSERT(mod != NULL);

  for (i = 0; i < RM57_ADC_NMAGCOMP; i++)
    {
      offset = rm57_adc_getreg(mod, RM57_ADC_MAGINTOFF_OFFSET) &
               ADC_MAGINTOFF_MASK;
      if (offset == 0)
        {
          break;
        }

      mod->magcomp_events++;

      /* The offset is one-based; the flag has already been cleared by the
       * read, so nothing else is needed to acknowledge it.
       */

      ainfo("ADC%d magnitude comparator %" PRIu32 " tripped\n",
            mod->module + 1, offset);
    }

  return OK;
}

/****************************************************************************
 * Name: rm57_adc_magcomp_attach
 *
 * Description:
 *   Attach the module's magnitude compare interrupt the first time one of
 *   its comparators is armed.
 *
 ****************************************************************************/

static int rm57_adc_magcomp_attach(struct rm57_adcmod_s *mod)
{
  int ret;

  if (mod->magcomp_attached)
    {
      return OK;
    }

  ret = irq_attach(mod->magcomp_irq, rm57_adc_magcomp_interrupt, mod);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach ADC%d magnitude compare IRQ %d: %d\n",
           mod->module + 1, mod->magcomp_irq, ret);
      return ret;
    }

  up_enable_irq(mod->magcomp_irq);
  mod->magcomp_attached = true;
  return OK;
}

/****************************************************************************
 * Name: rm57_adc_magcomp
 *
 * Description:
 *   Configure one of the module's three magnitude comparators.  Each watches
 *   a single channel and raises an interrupt when its conversion result
 *   crosses a threshold - either a fixed value or the latest result of a
 *   second channel.  The field layout differs between the 12- and 10-bit
 *   module configurations (TRM Figures 22-83 and 22-84).
 *
 ****************************************************************************/

static int rm57_adc_magcomp(struct rm57_adcmod_s *mod,
                            const struct rm57_adc_magcomp_s *mc)
{
  uint32_t regval;
  int ret;

  if (mc->index >= RM57_ADC_NMAGCOMP ||
      !rm57_adc_validchan(mod, mc->channel) ||
      (mc->usechannel && !rm57_adc_validchan(mod, mc->cmpchannel)))
    {
      return -EINVAL;
    }

  if (!mc->enable)
    {
      rm57_adc_putreg(mod, RM57_ADC_MAGINTENACLR_OFFSET,
                      ADC_MAGINT(mc->index));
      return OK;
    }

  if (mc->threshold > rm57_adc_fullscale(mod))
    {
      return -EINVAL;
    }

  if (mod->resolution == 12)
    {
      regval = ((uint32_t)mc->channel << ADC_MAGINTCR12_MAG_CHID_SHIFT) |
               ((uint32_t)mc->cmpchannel <<
                ADC_MAGINTCR12_COMP_CHID_SHIFT) |
               ((uint32_t)mc->threshold << ADC_MAGINTCR12_MAG_THR_SHIFT);

      if (mc->greater)
        {
          regval |= ADC_MAGINTCR12_CMP_GE_LT;
        }

      if (mc->usechannel)
        {
          regval |= ADC_MAGINTCR12_CHN_THR_COMP;
        }
    }
  else
    {
      regval = ((uint32_t)mc->channel << ADC_MAGINTCR10_MAG_CHID_SHIFT) |
               ((uint32_t)mc->cmpchannel <<
                ADC_MAGINTCR10_COMP_CHID_SHIFT) |
               ((uint32_t)mc->threshold << ADC_MAGINTCR10_MAG_THR_SHIFT);

      if (mc->greater)
        {
          regval |= ADC_MAGINTCR10_CMP_GE_LT;
        }

      if (mc->usechannel)
        {
          regval |= ADC_MAGINTCR10_CHN_THR_COMP;
        }
    }

  rm57_adc_putreg(mod, RM57_ADC_MAGINTCR_OFFSET(mc->index), regval);

  /* Compare every bit of the result */

  rm57_adc_putreg(mod, RM57_ADC_MAGINTMASK_OFFSET(mc->index),
                  rm57_adc_fullscale(mod));

  /* Clear a stale flag before arming, so the comparator does not fire on a
   * condition that was true before it was configured, and make sure someone
   * is listening before the line can assert.
   */

  rm57_adc_putreg(mod, RM57_ADC_MAGINTFLG_OFFSET, ADC_MAGINT(mc->index));

  ret = rm57_adc_magcomp_attach(mod);
  if (ret < 0)
    {
      return ret;
    }

  rm57_adc_putreg(mod, RM57_ADC_MAGINTENASET_OFFSET,
                  ADC_MAGINT(mc->index));

  return OK;
}

/****************************************************************************
 * Name: rm57_adc_wdog
 *
 * Description:
 *   Back the generic ANIOC_WDOG_UPPER/LOWER commands with the first two
 *   magnitude comparators of the owning module: comparator 0 fires when a
 *   result reaches the upper threshold, comparator 1 when it falls below
 *   the lower one.  The watched channel is the lowest one selected for this
 *   group, which is the only channel the generic interface can name.
 *
 ****************************************************************************/

static int rm57_adc_wdog(struct rm57_adcgrp_s *grp, bool upper,
                         uint16_t threshold)
{
  struct rm57_adc_magcomp_s mc;

  if (grp->cfg.chanmask == 0)
    {
      return -EINVAL;
    }

  mc.index      = upper ? 0 : 1;
  mc.enable     = true;
  mc.channel    = (uint8_t)__builtin_ctz(grp->cfg.chanmask);
  mc.threshold  = threshold;
  mc.greater    = upper;
  mc.usechannel = false;
  mc.cmpchannel = 0;

  return rm57_adc_magcomp(grp->mod, &mc);
}

#endif /* CONFIG_RM57_ADC_MAGCOMP */

#ifdef CONFIG_RM57_ADC_ENHANCED_CHSEL

/****************************************************************************
 * Name: rm57_adc_chnseq
 *
 * Description:
 *   Load a free-order channel conversion sequence into this group's look-up
 *   table and switch the group into the enhanced channel selection mode.
 *   Passing a NULL sequence returns the group to the default sequential
 *   mode, where channels are converted in ascending channel number.
 *
 *   Only ADC1 implements this (TRM section 22.2.2.2).  The table holds 32
 *   entries per group, stacked Event, Group1, Group2.
 *
 ****************************************************************************/

static int rm57_adc_chnseq(struct rm57_adcgrp_s *grp,
                           const struct rm57_adc_chnseq_s *seq)
{
  struct rm57_adcmod_s *mod = grp->mod;
  uintptr_t lut;
  int i;

  if (mod->lutbase == 0)
    {
      return -ENOTSUP;
    }

  if (seq == NULL)
    {
      rm57_adc_putreg(mod, RM57_ADC_CHNSELMODE_OFFSET(grp->group),
                      ADC_CHNSELMODE_SEQUENTIAL);
      return OK;
    }

  if (seq->nentries == 0 || seq->nentries > RM57_ADC_LUT_NENTRIES)
    {
      return -EINVAL;
    }

  for (i = 0; i < seq->nentries; i++)
    {
      if (!rm57_adc_validchan(mod, seq->channel[i]))
        {
          return -EINVAL;
        }
    }

  /* Conversions must be stopped while the table is rewritten */

  rm57_adc_stop(grp);

  lut = mod->lutbase + RM57_ADC_LUT_GROUP_OFFSET(grp->group);
  for (i = 0; i < seq->nentries; i++)
    {
      putreg32(ADC_LUT_INT_CHN(seq->channel[i]) |
               ADC_LUT_EXT_CHN(seq->extsel[i]), lut + i * 4);
    }

  /* MAX_COUNT is the index at which the table wraps.  The TRM recommends
   * one less than a whole number of sub-sequences, which for a table used
   * end to end is simply the last entry.
   */

  rm57_adc_putreg(mod, RM57_ADC_CURRCOUNT_OFFSET(grp->group), 0);
  rm57_adc_putreg(mod, RM57_ADC_MAXCOUNT_OFFSET(grp->group),
                  seq->nentries - 1);

  rm57_adc_putreg(mod, RM57_ADC_CHNSELMODE_OFFSET(grp->group),
                  ADC_CHNSELMODE_ENHANCED);

  return OK;
}

#endif /* CONFIG_RM57_ADC_ENHANCED_CHSEL */

/****************************************************************************
 * Name: rm57_adc_evtpin
 *
 * Description:
 *   Read or drive the module's ADEVT pin when it is not being used as a
 *   conversion trigger.  Note that the pulls are disabled by the hardware
 *   whenever the pin is an output.
 *
 ****************************************************************************/

static int rm57_adc_evtpin(struct rm57_adcmod_s *mod,
                           struct rm57_adc_evtpin_s *pin)
{
  if (pin->output)
    {
      rm57_adc_putreg(mod, RM57_ADC_EVTPDR_OFFSET,
                      pin->opendrain ? ADC_EVT_BIT : 0);
      rm57_adc_putreg(mod, RM57_ADC_EVTDIR_OFFSET, ADC_EVT_BIT);
      rm57_adc_putreg(mod, pin->value ? RM57_ADC_EVTSET_OFFSET :
                                        RM57_ADC_EVTCLR_OFFSET,
                      ADC_EVT_BIT);
    }
  else
    {
      rm57_adc_putreg(mod, RM57_ADC_EVTDIR_OFFSET, 0);
      rm57_adc_putreg(mod, RM57_ADC_EVTPSEL_OFFSET,
                      pin->pullup ? ADC_EVT_BIT : 0);
      rm57_adc_putreg(mod, RM57_ADC_EVTPDIS_OFFSET,
                      pin->pullen ? 0 : ADC_EVT_BIT);

      pin->value = (rm57_adc_getreg(mod, RM57_ADC_EVTIN_OFFSET) &
                    ADC_EVT_BIT) != 0;
    }

  return OK;
}

/****************************************************************************
 * Name: rm57_adc_bind
 *
 * Description:
 *   Record the upper half's callbacks.  Called by adc_register() before any
 *   other operation, so this is the earliest point at which results can be
 *   delivered.
 *
 ****************************************************************************/

static int rm57_adc_bind(struct adc_dev_s *dev,
                         const struct adc_callback_s *callback)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;

  DEBUGASSERT(grp != NULL);
  grp->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: rm57_adc_reset
 *
 * Description:
 *   Return the group to a known idle state: conversions stopped, interrupts
 *   and DMA requests off, FIFO emptied.  Called exactly once, by
 *   adc_register(), before the device can be opened - so it has to be safe
 *   on a module that has not been initialized yet, and it must not deliver
 *   any sample, because the upper half has not allocated its FIFO buffers at
 *   that point.  (The reset the upper half performs on an overrun is its own
 *   au_reset(), which is a different thing entirely.)
 *
 ****************************************************************************/

static void rm57_adc_reset(struct adc_dev_s *dev)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;
  struct rm57_adcmod_s *mod;
  irqstate_t flags;
  int g;

  DEBUGASSERT(grp != NULL);

  mod = grp->mod;
  g   = grp->group;

  flags = enter_critical_section();

  rm57_adc_modinit(mod);

  rm57_adc_putreg(mod, RM57_ADC_SEL_OFFSET(g), 0);
  rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(g), 0);
  rm57_adc_putreg(mod, RM57_ADC_DMACR_OFFSET(g), 0);

  rm57_adc_grpconfig(grp);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_adc_setup
 *
 * Description:
 *   Prepare the group for use on the first open of its character device:
 *   apply its configuration, claim its DMA channel if it has one, and attach
 *   its interrupt.  Interrupts are left disabled at the ADC on return, as
 *   the upper half expects; ao_rxint() turns them on.
 *
 ****************************************************************************/

static int rm57_adc_setup(struct adc_dev_s *dev)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;
  struct rm57_adcmod_s *mod;
  int ret;

  DEBUGASSERT(grp != NULL);

  if (grp->setup)
    {
      return OK;
    }

  mod = grp->mod;

  rm57_adc_modinit(mod);

  /* Bring the core out of power down if a previous shutdown parked it */

  rm57_adc_powerup(mod);

  grp->held = false;
  rm57_adc_grpconfig(grp);

#ifdef CONFIG_RM57_ADC_HAVE_DMA
  if (grp->usedma)
    {
      ret = rm57_adc_dmasetup(grp);
      if (ret < 0)
        {
          rm57_adc_idlepowerdown(mod);
          return ret;
        }
    }
#endif

  ret = irq_attach(grp->irq, rm57_adc_interrupt, grp);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach ADC%d group %d IRQ %d: %d\n",
           mod->module + 1, grp->group, grp->irq, ret);
#ifdef CONFIG_RM57_ADC_HAVE_DMA
      if (grp->usedma)
        {
          rm57_adc_dmastop(grp);
        }
#endif

      rm57_adc_idlepowerdown(mod);
      return ret;
    }

  up_enable_irq(grp->irq);

  mod->refcount++;
  grp->setup = true;

  ainfo("ADC%d group %d ready on IRQ %d\n", mod->module + 1, grp->group,
        grp->irq);
  return OK;
}

/****************************************************************************
 * Name: rm57_adc_shutdown
 *
 * Description:
 *   Undo rm57_adc_setup() when the character device is closed.  When the
 *   last group of a module goes away the ADC core is powered down; the
 *   sequencer's digital logic stays alive so the registers keep their
 *   contents.
 *
 ****************************************************************************/

static void rm57_adc_shutdown(struct adc_dev_s *dev)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;
  struct rm57_adcmod_s *mod;

  DEBUGASSERT(grp != NULL);

  if (!grp->setup)
    {
      return;
    }

  mod = grp->mod;

  up_disable_irq(grp->irq);
  irq_detach(grp->irq);

  rm57_adc_stop(grp);
  rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(grp->group), 0);

#ifdef CONFIG_RM57_ADC_HAVE_DMA
  if (grp->usedma)
    {
      rm57_adc_dmastop(grp);
    }
#endif

  grp->setup = false;
  grp->held  = false;

  if (mod->refcount > 0)
    {
      mod->refcount--;
    }

  rm57_adc_idlepowerdown(mod);
}

/****************************************************************************
 * Name: rm57_adc_rxint
 *
 * Description:
 *   Enable or disable the group's result interrupts, and start or stop
 *   conversions with them.  A group left converting with its interrupt
 *   masked would only fill its results memory and overrun.
 *
 ****************************************************************************/

static void rm57_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;
  struct rm57_adcmod_s *mod;
  irqstate_t flags;
  int g;

  DEBUGASSERT(grp != NULL);

  mod = grp->mod;
  g   = grp->group;

  flags = enter_critical_section();

  if (enable)
    {
      uint32_t intena = ADC_INTENA_OVR;

#ifdef CONFIG_RM57_ADC_HAVE_DMA
      /* With DMA moving the results the CPU only needs to hear about
       * overruns; the threshold and end interrupts would duplicate the
       * DMA callback.
       */

      if (!grp->usedma)
#endif
        {
          intena |= grp->cfg.continuous ? ADC_INTENA_THR : ADC_INTENA_END;
        }

      rm57_adc_putreg(mod, RM57_ADC_INTFLG_OFFSET(g), ADC_INTFLG_CLRALL);
      rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(g), intena);

      rm57_adc_start(grp);
    }
  else
    {
      rm57_adc_stop(grp);
      rm57_adc_putreg(mod, RM57_ADC_INTENA_OFFSET(g), 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rm57_adc_ioctl
 *
 * Description:
 *   Handle the ADC commands the upper half did not consume itself.  That
 *   covers the generic ANIOC_TRIGGER, ANIOC_GET_NCHANNELS and the watchdog
 *   pair, plus the RM57-specific commands declared in rm57_adc.h.
 *
 ****************************************************************************/

/* Commands that reprogram this group's channel selection or mode.  They have
 * to be serialised against the diagnostic sequences, which borrow a group
 * and would otherwise have their conversion redirected mid-flight - or, just
 * as bad, hand back a channel selection they saved before the caller changed
 * it.  The diagnostic commands themselves take the same lock further down,
 * so they must not be listed here.
 */

static bool rm57_adc_ioctl_needslock(int cmd)
{
  switch (cmd)
    {
      case ANIOC_TRIGGER:
      case ANIOC_RM57_START:
      case ANIOC_RM57_STOP:
      case ANIOC_RM57_SET_CHANMASK:
      case ANIOC_RM57_SET_GRPCFG:
#ifdef CONFIG_RM57_ADC_ENHANCED_CHSEL
      case ANIOC_RM57_SET_CHNSEQ:
#endif
        return true;

      default:
        return false;
    }
}

static int rm57_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct rm57_adcgrp_s *grp = (struct rm57_adcgrp_s *)dev->ad_priv;
  struct rm57_adcmod_s *mod;
  bool locked = rm57_adc_ioctl_needslock(cmd);
  int ret = OK;

  DEBUGASSERT(grp != NULL);
  mod = grp->mod;

  if (locked)
    {
      ret = nxmutex_lock(&mod->lock);
      if (ret < 0)
        {
          return ret;
        }

      ret = OK;
    }

  switch (cmd)
    {
      /* Generic commands ***************************************************/

      case ANIOC_TRIGGER:
        {
          /* Kick off one conversion sequence.  For a hardware-triggered
           * group this only arms it; the trigger still has to arrive.
           */

          rm57_adc_start(grp);
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          ret = __builtin_popcount(grp->cfg.chanmask);
        }
        break;

#ifdef CONFIG_RM57_ADC_MAGCOMP
      case ANIOC_WDOG_UPPER:
        {
          ret = rm57_adc_wdog(grp, true, (uint16_t)arg);
        }
        break;

      case ANIOC_WDOG_LOWER:
        {
          ret = rm57_adc_wdog(grp, false, (uint16_t)arg);
        }
        break;
#endif

      /* RM57-specific commands *********************************************/

      case ANIOC_RM57_START:
        {
          rm57_adc_start(grp);
        }
        break;

      case ANIOC_RM57_STOP:
        {
          rm57_adc_stop(grp);
        }
        break;

      case ANIOC_RM57_SET_CHANMASK:
        {
          uint32_t mask = (uint32_t)arg;

          if ((mask & ~mod->validmask) != 0)
            {
              ret = -EINVAL;
              break;
            }

          grp->cfg.chanmask = mask;
          rm57_adc_start(grp);
        }
        break;

      case ANIOC_RM57_SET_GRPCFG:
        {
          const struct rm57_adc_grpcfg_s *cfg =
            (const struct rm57_adc_grpcfg_s *)((uintptr_t)arg);

          if (cfg == NULL)
            {
              ret = -EINVAL;
              break;
            }

          if ((cfg->chanmask & ~mod->validmask) != 0)
            {
              ret = -EINVAL;
              break;
            }

          if (cfg->trigsrc > (ADC_SRC_SRC_MASK >> ADC_SRC_SRC_SHIFT))
            {
              ret = -EINVAL;
              break;
            }

          rm57_adc_stop(grp);
          grp->cfg = *cfg;
          rm57_adc_grpconfig(grp);
        }
        break;

      case ANIOC_RM57_CALIBRATE:
        {
          ret = rm57_adc_calibrate(mod->module,
                  (struct rm57_adc_calib_s *)((uintptr_t)arg));
        }
        break;

      case ANIOC_RM57_MIDPOINTCAL:
        {
          ret = rm57_adc_midpointcal(mod->module,
                  (struct rm57_adc_calib_s *)((uintptr_t)arg));
        }
        break;

      case ANIOC_RM57_GET_OFFSET:
        {
          int32_t *offset = (int32_t *)((uintptr_t)arg);

          if (offset == NULL)
            {
              ret = -EINVAL;
              break;
            }

          /* ADCALR holds the correction as a two's complement value in the
           * module's resolution.  Sign extend it so the caller gets the same
           * signed quantity struct rm57_adc_calib_s reports, rather than a
           * large positive number for every negative correction.
           */

          uint32_t full = rm57_adc_fullscale(mod);
          uint32_t sign = (full >> 1) + 1;
          uint32_t raw  = rm57_adc_getreg(mod, RM57_ADC_CALR_OFFSET) & full;

          *offset = (int32_t)raw - (int32_t)((raw & sign) != 0 ?
                                             full + 1 : 0);
        }
        break;

      case ANIOC_RM57_SET_OFFSET:
        {
          rm57_adc_putreg(mod, RM57_ADC_CALR_OFFSET,
                          (uint32_t)arg & rm57_adc_fullscale(mod));
        }
        break;

      case ANIOC_RM57_SELFTEST:
        {
          ret = rm57_adc_selftest(mod->module,
                  (struct rm57_adc_selftest_s *)((uintptr_t)arg));
        }
        break;

#ifdef CONFIG_RM57_ADC_MAGCOMP
      case ANIOC_RM57_SET_MAGCOMP:
        {
          const struct rm57_adc_magcomp_s *mc =
            (const struct rm57_adc_magcomp_s *)((uintptr_t)arg);

          ret = mc == NULL ? -EINVAL : rm57_adc_magcomp(mod, mc);
        }
        break;
#endif

#ifdef CONFIG_RM57_ADC_ENHANCED_CHSEL
      case ANIOC_RM57_SET_CHNSEQ:
        {
          ret = rm57_adc_chnseq(grp,
                  (const struct rm57_adc_chnseq_s *)((uintptr_t)arg));
        }
        break;
#endif

#ifdef CONFIG_RM57_ADC_TEMPSENSOR
      case ANIOC_RM57_GET_TEMP:
        {
          struct rm57_adc_temp_s *temp =
            (struct rm57_adc_temp_s *)((uintptr_t)arg);

          ret = temp == NULL ? -EINVAL :
                rm57_adc_readtemp(temp->sensor, temp);
        }
        break;
#endif

      case ANIOC_RM57_EVTPIN:
        {
          struct rm57_adc_evtpin_s *pin =
            (struct rm57_adc_evtpin_s *)((uintptr_t)arg);

          ret = pin == NULL ? -EINVAL : rm57_adc_evtpin(mod, pin);
        }
        break;

      default:
        {
          aerr("ERROR: Unrecognized ADC ioctl 0x%04x\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  if (locked)
    {
      nxmutex_unlock(&mod->lock);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_adc_initialize
 *
 * Description:
 *   Initialize one MibADC conversion group and register it with the upper
 *   half ADC driver at 'devpath'.
 *
 ****************************************************************************/

int rm57_adc_initialize(const char *devpath, int module, int group)
{
  struct rm57_adcgrp_s *grp;
  struct rm57_adcmod_s *mod;
  int ret;

  DEBUGASSERT(devpath != NULL);

  if (module < 0 || module >= RM57_ADC_NMODULES ||
      group < 0 || group >= RM57_ADC_NGROUPS)
    {
      return -EINVAL;
    }

  grp = rm57_adc_group(module, group);
  if (grp == NULL)
    {
      aerr("ERROR: ADC%d group %d is not enabled\n", module + 1, group);
      return -ENODEV;
    }

  mod = grp->mod;

  /* Publish the back pointer before adc_register(), which calls straight
   * into ao_bind() and ao_reset().
   */

  grp->dev.ad_priv = grp;
  mod->grp[group]  = grp;

  ret = adc_register(devpath, &grp->dev);
  if (ret < 0)
    {
      aerr("ERROR: adc_register(%s) failed: %d\n", devpath, ret);
      mod->grp[group] = NULL;
      return ret;
    }

#ifdef CONFIG_RM57_ADC_CALIBRATE
  /* Calibrate once the module is up.  A failure here is not fatal: the
   * converter still works, just without offset correction.
   */

  ret = rm57_adc_calibrate(module, NULL);
  if (ret < 0)
    {
      awarn("WARNING: ADC%d calibration failed: %d\n", module + 1, ret);
      ret = OK;
    }
#endif

  ainfo("Registered %s as ADC%d group %d\n", devpath, module + 1, group);
  return ret;
}

#endif /* CONFIG_RM57_ADC */
