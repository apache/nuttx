/****************************************************************************
 * arch/arm/src/armv7-m/arm_itm_syslog.c
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

#include <nuttx/syslog/syslog.h>
#include <nuttx/compiler.h>

#include "nvic.h"
#include "itm.h"
#include "tpi.h"
#include "dwt.h"
#include "arm_internal.h"
#include "itm_syslog.h"

#ifdef CONFIG_ARMV7M_ITMSYSLOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARMV7M_ITMSYSLOG_SWODIV
#  define CONFIG_ARMV7M_ITMSYSLOG_SWODIV 15
#endif

#if CONFIG_ARMV7M_ITMSYSLOG_SWODIV < 0
#  error CONFIG_ARMV7M_ITMSYSLOG_SWODIV should be at least equal to 1
#endif

/* Use Port #0 at default */

#ifndef CONFIG_ARMV7M_ITMSYSLOG_PORT
#  define CONFIG_ARMV7M_ITMSYSLOG_PORT 0
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SYSLOG channel methods */

static int itm_putc(struct syslog_channel_s *channel, int ch);
static int itm_flush(struct syslog_channel_s *channel);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the ITM SYSLOG channel operations */

static const struct syslog_channel_ops_s g_itm_channel_ops =
{
  .sc_putc  = itm_putc,
  .sc_force = itm_putc,
  .sc_flush = itm_flush,
};

/* This structure describes the ITM SYSLOG channel */

static struct syslog_channel_s g_itm_channel =
{
  .sc_ops   = &g_itm_channel_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: itm_putc
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 ****************************************************************************/

static int itm_putc(struct syslog_channel_s *channel, int ch)
{
  UNUSED(channel);

  /* ITM enabled */

  if ((getreg32(ITM_TCR) & ITM_TCR_ITMENA_MASK) == 0)
    {
      return EOF;
    }

  /* ITM Port "CONFIG_ARMV7M_ITMSYSLOG_PORT" enabled */

  if (getreg32(ITM_TER) & (1 << CONFIG_ARMV7M_ITMSYSLOG_PORT))
    {
      while (getreg32(ITM_PORT(CONFIG_ARMV7M_ITMSYSLOG_PORT)) == 0);
      putreg8((uint8_t)ch, ITM_PORT(CONFIG_ARMV7M_ITMSYSLOG_PORT));
    }

  return ch;
}

/****************************************************************************
 * Name: itm_flush
 *
 * Description:
 *   A dummy FLUSH method
 *
 ****************************************************************************/

static int itm_flush(struct syslog_channel_s *channel)
{
  UNUSED(channel);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: itm_syslog_initialize
 *
 * Description:
 *   Performs ARM-specific initialize for the ITM SYSLOG functions.
 *   Additional, board specific logic may be required to:
 *
 *   - Enable/configured serial wire output pins
 *   - Enable debug clocking.
 *
 *   Those operations must be performed by MCU-specific logic before this
 *   function is called.
 *
 ****************************************************************************/

void itm_syslog_initialize(void)
{
  uint32_t regval;

  /* Enable trace in core debug */

  regval  = getreg32(NVIC_DEMCR);
  regval |= NVIC_DEMCR_TRCENA;
  putreg32(regval, NVIC_DEMCR);

  putreg32(0xc5acce55, ITM_LAR);
  putreg32(0,          ITM_TER);
  putreg32(0,          ITM_TCR);
  putreg32(2,          TPI_SPPR); /* Pin protocol: 2=> Manchester (USART) */

  /* Default 880kbps */

  regval = CONFIG_ARMV7M_ITMSYSLOG_SWODIV - 1;
  putreg32(regval,     TPI_ACPR); /* TRACECLKIN/(ACPR+1) SWO speed */

  putreg32(0,          ITM_TPR);
  putreg32(0x400003fe, DWT_CTRL);
  putreg32(0x0001000d, ITM_TCR);
  putreg32(0x00000100, TPI_FFCR);
  putreg32(0xffffffff, ITM_TER); /* Enable 32 Ports */

  /* Setup the SYSLOG channel */

  syslog_channel(&g_itm_channel);
}

#endif /* CONFIG_ARMV7M_ITMSYSLOG */
