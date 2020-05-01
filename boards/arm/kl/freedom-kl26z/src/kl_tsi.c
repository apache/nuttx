/****************************************************************************
 * boards/arm/kl/freedom-kl26z/src/kl_tsi.c
 *
 *   Copyright (C) 2013 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *           with adaptations from Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 * https://community.freescale.com/community/
 * the-embedded-beat/blog/2012/10/15/
 * using-the-touch-interface-on-the-freescale-freedom-development-platform
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "arm_arch.h"
#include "kl_gpio.h"
#include "hardware/kl_tsi.h"
#include "hardware/kl_pinmux.h"
#include "hardware/kl_sim.h"

#ifdef CONFIG_KL_TSI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The touchpad on the Freedom KL26Z board connects to the MCU via:
 *
 *   PTB16 TWI0_CH9
 *   PTB17 TSI0_CH10
 */

#define NSENSORS 2

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    tsi_calibrate(void);
static ssize_t tsi_read(FAR struct file *filep,
                        FAR char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_defcap[NSENSORS];    /* Default capacitance for each sensor */
static uint16_t g_currdelta = 0;       /* Current delta for the current button */
static uint8_t  g_channel = 0;         /* Current g_channel */

/* Channel assigned to each sensor */

static uint8_t const g_chsensor[NSENSORS] = { 9, 10 };

/* Character driver operations */

static const struct file_operations tsi_ops =
{
  0,           /* open */
  0,           /* close */
  tsi_read,    /* read */
  0,           /* write */
  0,           /* seek */
  0,           /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsi_calibrate
 ****************************************************************************/

static void tsi_calibrate(void)
{
  uint32_t regval;
  int i;

  for (i = 0; i < NSENSORS; i++)
    {
      /* Clear end of scan flag */

      regval  = getreg32(KL_TSI_GENCS);
      regval |= TSI_GENCS_EOSF;
      putreg32(regval, KL_TSI_GENCS);

      /* Scan the next electrode */

      regval = TSI_DATA_TSICH(g_chsensor[i]);
      putreg32(regval, KL_TSI_DATA);

      regval |= TSI_DATA_SWTS;
      putreg32(regval, KL_TSI_DATA);

      /* Wait until the conversion is done */

      while (!(getreg32(KL_TSI_GENCS) & TSI_GENCS_EOSF));

      g_defcap[i] = getreg32(KL_TSI_DATA) & TSI_DATA_TSICNT_MASK;
      iinfo("Sensor %d = %d\n", i + 1, g_defcap[i]);
    }
}

/****************************************************************************
 * Name: tsi_read
 ****************************************************************************/

static ssize_t tsi_read(FAR struct file *filep, FAR char *buf, size_t buflen)
{
  static int deltacap = 0;
  uint32_t regval;

  if (buf == NULL || buflen < 1)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  deltacap = 0;

  /* Clear end of scan flag */

  regval  = getreg32(KL_TSI_GENCS);
  regval |= TSI_GENCS_EOSF;
  putreg32(regval, KL_TSI_GENCS);

  /* Scan the next electrode */

  regval = TSI_DATA_TSICH(g_chsensor[g_channel]);
  putreg32(regval, KL_TSI_DATA);

  regval |= TSI_DATA_SWTS;
  putreg32(regval, KL_TSI_DATA);

  /* Wait until the conversion is done */

  while (!(getreg32(KL_TSI_GENCS) & TSI_GENCS_EOSF));

  /* Compute delta using calibration reference counts */

  deltacap  = getreg32(KL_TSI_DATA) & TSI_DATA_TSICNT_MASK;
  deltacap -= g_defcap[g_channel];

  if (deltacap < 0)
    {
      deltacap = 0;
    }

  g_currdelta = (uint16_t)deltacap;

  iinfo("Delta for g_channel %d = %d\n", g_channel, g_currdelta);

  buf[0] = g_currdelta & 0xff;
  buf[1] = (g_currdelta & 0xff00) >> 8;
  buf[2] = g_channel;

  /* Increment the channel index to sample the next sensor on the next timer
   * that read() is called.
   */

  g_channel++;
  if (g_channel >= NSENSORS)
    {
      g_channel = 0;
    }

  return 3;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_tsi_initialize
 *
 * Description:
 *   Initialize the TSI hardware and interface for the sliders on board the
 *   Freedom KL26Z board.  Register a character driver at /dev/tsi that may
 *   be used to read from each sensor.
 *
 ****************************************************************************/

void kl_tsi_initialize(void)
{
  uint32_t regval;

  /* Enable access to the TSI module */

  regval = getreg32(KL_SIM_SCGC5);
  regval |= SIM_SCGC5_TSI;
  putreg32(regval, KL_SIM_SCGC5);

  /* Configure PINs used on TSI */

  kl_configgpio(PIN_TSI0_CH9);
  kl_configgpio(PIN_TSI0_CH10);

  /* Configure TSI parameter */

  regval = getreg32(KL_TSI_GENCS);
  regval |= TSI_GENCS_MODE_CAPSENSING | TSI_GENCS_REFCHRG_8UA |
            TSI_GENCS_DVOLT_1p03V | TSI_GENCS_EXTCHRG_64A |
            TSI_GENCS_PS_DIV16 | TSI_GENCS_NSCN_TIMES(12) | TSI_GENCS_STPE;
  putreg32(regval, KL_TSI_GENCS);

  /* Only after setting TSI we should enable it */

  regval  = getreg32(KL_TSI_GENCS);
  regval |= TSI_GENCS_TSIEN;
  putreg32(regval, KL_TSI_GENCS);

  /* Calibrate it before to use */

  tsi_calibrate();

  /* And finally register the TSI character driver */

  register_driver("/dev/tsi", &tsi_ops, 0444, NULL);
}

#endif /* CONFIG_KL_TSI */
