/****************************************************************************
 * include/nuttx/timers/imx_gpt.h
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

/* The i.MX GPT (General Purpose Timer) exposes a single free-running
 * counter shared by an Output Compare Unit (OCU) and an Input Capture Unit
 * (ICU).  A single MMIO block and a single interrupt line serve all
 * channels.  This driver implements a shared "core" (imx_gpt.c) that owns
 * the MMIO mapping, counter initialisation and the one and only interrupt
 * service routine, plus "lower-half" adaptations that plug into the generic
 * NuttX ocu (OCU) and capture (ICU) upper-half drivers.
 */

#ifndef __INCLUDE_NUTTX_TIMERS_IMX_GPT_H
#define __INCLUDE_NUTTX_TIMERS_IMX_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/capture.h>
#include <nuttx/timers/ocu.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Channel counts (configurable, bounded by the hardware). */

#ifdef CONFIG_TIMER_IMX_GPT_OCU
#define IMX_GPT_NUM_OCU  CONFIG_TIMER_IMX_GPT_NUM_OCU
#endif
#ifdef CONFIG_TIMER_IMX_GPT_ICU
#define IMX_GPT_NUM_ICU  CONFIG_TIMER_IMX_GPT_NUM_ICU
#endif

/* Register offsets (32-bit, little-endian, 4-byte access). */

#define IMX_GPT_CR       0x00  /* Control register */
#define IMX_GPT_PR       0x04  /* Prescaler register */
#define IMX_GPT_SR       0x08  /* Status register (W1C) */
#define IMX_GPT_IR       0x0c  /* Interrupt enable register */
#define IMX_GPT_OCR1     0x10  /* Output compare register 1 (OCU ch0) */
#define IMX_GPT_OCR2     0x14  /* Output compare register 2 (OCU ch1) */
#define IMX_GPT_OCR3     0x18  /* Output compare register 3 (OCU ch2) */
#define IMX_GPT_ICR1     0x1c  /* Input capture register 1 (ICU ch0) */
#define IMX_GPT_ICR2     0x20  /* Input capture register 2 (ICU ch1) */
#define IMX_GPT_CNT      0x24  /* Counter (free-running, shared time base) */

/* Per-channel register helpers. */

#define IMX_GPT_OCR(ch)  (IMX_GPT_OCR1 + ((ch) * 4)) /* ch = 0..2 */
#define IMX_GPT_ICR(ch)  (IMX_GPT_ICR1 + ((ch) * 4)) /* ch = 0..1 */

/* GPT_CR bit definitions. */

#define GPT_CR_EN           (1 << 0)   /* Counter enable */
#define GPT_CR_ENMOD        (1 << 1)   /* Reset counter when EN 0->1 */
#define GPT_CR_CLKSRC_SHIFT (6)        /* Clock source select */
#define GPT_CR_CLKSRC_MASK  (0x7 << GPT_CR_CLKSRC_SHIFT)
#define GPT_CR_FRR          (1 << 9)   /* Free-run (1) / restart (0) */
#define GPT_CR_SWR          (1 << 15)  /* Software reset */

/* Input capture mode (2 bits/channel): 0=off 1=rising 2=falling 3=both. */

#define GPT_CR_IM_SHIFT(ch) (16 + ((ch) * 2))
#define GPT_CR_IM_MASK(ch)  (0x3 << GPT_CR_IM_SHIFT(ch))
#define GPT_IM_DISABLE      0
#define GPT_IM_RISING       1
#define GPT_IM_FALLING      2
#define GPT_IM_BOTH         3

/* Output compare mode (3 bits/channel):
 * 0=disconnect 1=toggle 2=clear(low) 3=set(high) 4=low-pulse.
 */

#define GPT_CR_OM_SHIFT(ch) (20 + ((ch) * 3))
#define GPT_CR_OM_MASK(ch)  (0x7 << GPT_CR_OM_SHIFT(ch))
#define GPT_OM_DISCONNECT   0
#define GPT_OM_TOGGLE       1
#define GPT_OM_CLEAR        2
#define GPT_OM_SET          3

/* Force output compare (write 1, self-clearing), one bit per OCU channel. */

#define GPT_CR_FO(ch)       (1 << (29 + (ch)))

/* GPT_SR / GPT_IR share the same bit layout. */

#define GPT_SR_OF(ch)       (1 << (ch))        /* OCU compare match, ch 0..2 */
#define GPT_SR_IF(ch)       (1 << (3 + (ch)))  /* ICU edge capture, ch 0..1 */
#define GPT_SR_ROV          (1 << 5)           /* Counter roll-over */

#define GPT_IR_OF(ch)       GPT_SR_OF(ch)
#define GPT_IR_IF(ch)       GPT_SR_IF(ch)
#define GPT_IR_ROV          GPT_SR_ROV

#define GPT_TIMER_MAX       0xffffffffu

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct imx_gpt_dev_s;

/* OCU (output compare) per-channel lower-half. Defined here so the shared
 * core ISR can dispatch directly to it.
 */

#ifdef CONFIG_TIMER_IMX_GPT_OCU
struct imx_gpt_ocu_s
{
  struct ocu_lowerhalf_s     lower;          /* Must be first (cast-compat) */
  FAR struct imx_gpt_dev_s  *dev;            /* Back-pointer to shared core */
  uint8_t                    ch;             /* OCU channel index 0..2 */
  bool                       active;         /* Compare actions enabled */
  bool                       notify_en;      /* HW notification enabled */
  enum ocu_pinaction_e       action;         /* Pin action on match */
  uint32_t                   last_threshold; /* Last programmed OCR value */
  ocu_callback_t             callback;       /* ISR match callback */
  FAR void                  *cbarg;          /* Callback argument */
};
#endif

/* ICU (input capture) per-channel lower-half. */

#ifdef CONFIG_TIMER_IMX_GPT_ICU
struct imx_gpt_icu_s
{
  struct cap_lowerhalf_s     lower;          /* Must be first (cast-compat) */
  FAR struct imx_gpt_dev_s  *dev;            /* Back-pointer to shared core */
  uint8_t                    ch;             /* ICU channel index 0..1 */
  bool                       running;        /* Capture enabled */
  bool                       have_rise;      /* last_rise is valid */
  uint8_t                    level;          /* Current inferred input level */
#ifdef CONFIG_CAPTURE_NOTIFY
  enum cap_type_e            type;           /* Notify edge filter */
  capture_notify_t           cb;             /* Upper-half notify callback */
  FAR void                  *priv;           /* Callback private pointer */
#endif
  uint32_t                   last_rise;      /* Timestamp of last rising edge */
  uint32_t                   period;         /* Ticks between rising edges */
  uint32_t                   high_time;      /* High-level ticks in a period */
  uint32_t                   edges;          /* Accumulated edge count */
};
#endif

/* Shared "core" device: one MMIO block, one IRQ, one counter. */

struct imx_gpt_dev_s
{
  uintptr_t                  base;           /* MMIO base address */
  int                        irq;            /* Interrupt number */
  uint32_t                   freq;           /* Effective tick freq (Hz) */
  spinlock_t                 lock;           /* Protects CR/SR RMW */
  volatile uint32_t          rollover;       /* ROV accounting */
#ifdef CONFIG_TIMER_IMX_GPT_OCU
  FAR struct imx_gpt_ocu_s  *ocu[IMX_GPT_NUM_OCU];
#endif
#ifdef CONFIG_TIMER_IMX_GPT_ICU
  FAR struct imx_gpt_icu_s  *icu[IMX_GPT_NUM_ICU];
#endif
};

/****************************************************************************
 * Inline Register Access Helpers
 ****************************************************************************/

static inline uint32_t imx_gpt_getreg(FAR struct imx_gpt_dev_s *dev,
                                      unsigned int offset)
{
  return *((FAR volatile uint32_t *)(dev->base + offset));
}

static inline void imx_gpt_putreg(FAR struct imx_gpt_dev_s *dev,
                                  unsigned int offset, uint32_t value)
{
  *((FAR volatile uint32_t *)(dev->base + offset)) = value;
}

static inline void imx_gpt_modreg(FAR struct imx_gpt_dev_s *dev,
                                  unsigned int offset, uint32_t clrbits,
                                  uint32_t setbits)
{
  FAR volatile uint32_t *addr =
    (FAR volatile uint32_t *)(dev->base + offset);
  *addr = (*addr & ~clrbits) | setbits;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: imx_gpt_initialize
 *
 * Description:
 *   Initialise a single i.MX GPT instance: map the MMIO block, configure
 *   and start the shared free-running counter, attach the interrupt, and
 *   register the OCU channels as /dev/ocu<ocu_bus + n>.
 *
 * Input Parameters:
 *   base    - MMIO base address of the GPT block (from FDT).
 *   irq     - Interrupt number of the GPT block (from FDT).
 *   freq    - Counter input clock in Hz (from FDT clock-frequency).
 *   ocu_bus - Starting /dev/ocuN index for the OCU channels.
 *   cap_bus - Starting /dev/capN index for the ICU channels (unused until
 *             ICU support is added).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int imx_gpt_initialize(uintptr_t base, int irq, uint32_t freq,
                       int ocu_bus, int cap_bus);

/****************************************************************************
 * Name: imx_gpt_ocu_register
 *
 * Description:
 *   Internal helper implemented by the OCU lower-half source file.  Called
 *   from imx_gpt_initialize() to build and register one channel.  Returns
 *   the allocated channel object (stored by the core in dev->ocu[]) or NULL
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER_IMX_GPT_OCU
FAR struct imx_gpt_ocu_s *
imx_gpt_ocu_register(FAR struct imx_gpt_dev_s *dev, int ch,
                     FAR const char *devpath);
#endif

#ifdef CONFIG_TIMER_IMX_GPT_ICU
FAR struct imx_gpt_icu_s *
imx_gpt_icu_register(FAR struct imx_gpt_dev_s *dev, int ch,
                     FAR const char *devpath);
#endif

/****************************************************************************
 * Name: imx_gpt_ocu_on_match
 *
 * Description:
 *   Per-channel compare-match interrupt handler, implemented by the OCU
 *   lower-half source file and invoked from the shared core ISR when the
 *   associated status bit is set.  Called in interrupt context.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER_IMX_GPT_OCU
void imx_gpt_ocu_on_match(FAR struct imx_gpt_ocu_s *ocu);
#endif
#ifdef CONFIG_TIMER_IMX_GPT_ICU
void imx_gpt_icu_on_edge(FAR struct imx_gpt_icu_s *icu);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_IMX_GPT_H */
