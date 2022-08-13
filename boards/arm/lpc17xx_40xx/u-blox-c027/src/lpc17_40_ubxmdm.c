/****************************************************************************
 * boards/arm/lpc17xx_40xx/u-blox-c027/src/lpc17_40_ubxmdm.c
 *
 *   Copyright (C) 2016 Vladimir Komendantskiy. All rights reserved.
 *   Author: Vladimir Komendantskiy <vladimir@moixaenergy.com>
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/modem/u-blox.h>

#include <arch/board/board.h>
#include "lpc17_40_gpio.h"
#include "u-blox-c027.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_U_BLOX_DEBUG
#  define m_err     _err
#  define m_info    _info
#else
#  define m_err(x...)
#  define m_info(x...)
#endif

#define UBXMDM_REGISTER_COUNT                           \
  (sizeof(lpc17_40_ubxmdm_name_pins) /                     \
   sizeof(struct lpc17_40_name_pin))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Core interface pin connections.  These do not include UART or USB pins. */

struct lpc17_40_ubxmdm_pins
{
  lpc17_40_pinset_t ldo_enable;
  lpc17_40_pinset_t power_on_n;
  lpc17_40_pinset_t reset_n;
  lpc17_40_pinset_t shifter_en_n;
  lpc17_40_pinset_t usb_detect;
};

/* This structure type provides the private representation of the
 * "lower-half" driver state.  This type must be coercible to type
 * 'ubxmdm_lower'.
 */

struct lpc17_40_ubxmdm_lower
{
  const struct ubxmdm_ops * ops;  /* Lower half operations */

  /* Private, architecture-specific information. */

  const struct lpc17_40_ubxmdm_pins * pins;
  bool usb_used;
};

/* Pair type for associating a register name to a pin. */

struct lpc17_40_name_pin
{
  const char name[3];
  const lpc17_40_pinset_t pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int lpc17_40_poweron  (struct ubxmdm_lower * lower);
static int lpc17_40_poweroff (struct ubxmdm_lower * lower);
static int lpc17_40_reset    (struct ubxmdm_lower * lower);
static int lpc17_40_getstatus(struct ubxmdm_lower * lower,
                              struct ubxmdm_status * status);
static int lpc17_40_ioctl    (struct ubxmdm_lower * lower,
                              int cmd,
                              unsigned long arg);

/* "Lower half" driver state */

static struct lpc17_40_ubxmdm_lower lpc17_40_ubxmdm_lower;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct lpc17_40_ubxmdm_pins lpc17_40_ubxmdm_pins =
{
  .ldo_enable   = C027_MDMLDOEN,
  .power_on_n   = C027_MDMPWR,
  .reset_n      = C027_MDMRST,
  .shifter_en_n = C027_MDMLVLOE,
  .usb_detect   = C027_MDMUSBDET,
};

static const struct ubxmdm_ops lpc17_40_ubxmdm_ops =
{
  .poweron      = lpc17_40_poweron,
  .poweroff     = lpc17_40_poweroff,
  .reset        = lpc17_40_reset,
  .getstatus    = lpc17_40_getstatus,
  .ioctl        = lpc17_40_ioctl,
};

static const struct lpc17_40_name_pin lpc17_40_ubxmdm_name_pins[] =
{
  {
    {
      'T', 'X', 'D'
    },
    GPIO_UART1_TXD
  },
  {
    {
      'R', 'X', 'D'
    },
    GPIO_UART1_RXD
  },
  {
    {
      'C', 'T', 'S'
    },
    GPIO_UART1_CTS
  },
  {
    {
      'R', 'T', 'S'
    },
    GPIO_UART1_RTS
  },
  {
    {
      'D', 'C', 'D'
    },
    GPIO_UART1_DCD
  },
  {
    {
      'D', 'S', 'R'
    },
    GPIO_UART1_DSR
  },
  {
    {
      'D', 'T', 'R'
    },
    GPIO_UART1_DTR
  },
  {
    {
      'R', 'I', ' '
    },
    GPIO_UART1_RI
  },
  {
    {
      'I', 'O', '1'
    },
    C027_MDMGPIO1
  },
  {
    {
      'R', 'S', 'T'
    },
    C027_MDMRST
  },
  {
    {
      'P', 'W', 'R'
    },
    C027_MDMPWR
  },
  {
    {
      'D', 'E', 'T'
    },
    C027_MDMUSBDET
  },
  {
    {
      'L', 'D', 'O'
    },
    C027_MDMLDOEN
  },
  {
    {
      'L', 'V', 'L'
    },
    C027_MDMLVLOE
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lpc17_40_poweron(struct ubxmdm_lower * lower)
{
  struct lpc17_40_ubxmdm_lower * priv =
    (struct lpc17_40_ubxmdm_lower *) lower;
  lpc17_40_pinset_t usb_detect_val;

  /* Modem not in reset */

  lpc17_40_configgpio(priv->pins->reset_n      | GPIO_VALUE_ONE);

  /* Switch closed to GND */

  lpc17_40_configgpio(priv->pins->power_on_n   | GPIO_VALUE_ZERO);

  /* Min. time for power_on_n being low is 5 ms */

  nxsig_usleep(10 * 1000);

  if (priv->usb_used)
    {
      usb_detect_val = GPIO_VALUE_ONE;  /* USB sense on */
    }
  else
    {
      usb_detect_val = GPIO_VALUE_ZERO; /* USB sense off */
    }

  lpc17_40_configgpio(priv->pins->usb_detect   | usb_detect_val);

  /* LDO enabled */

  lpc17_40_configgpio(priv->pins->ldo_enable   | GPIO_VALUE_ONE);

  /* Delay to obtain correct voltage on shifters */

  nxsig_usleep(1 * 1000);

  /* UART shifter enabled */

  lpc17_40_configgpio(priv->pins->shifter_en_n | GPIO_VALUE_ZERO);

  /* Stop current through switch */

  /* lpc17_40_configgpio(priv->pins->power_on_n  | GPIO_VALUE_ONE); */

  return OK;
}

static int lpc17_40_poweroff(struct ubxmdm_lower * lower)
{
  struct lpc17_40_ubxmdm_lower * priv =
    (struct lpc17_40_ubxmdm_lower *) lower;

  /* LDO disabled */

  lpc17_40_configgpio(priv->pins->ldo_enable   | GPIO_VALUE_ZERO);

  /* Modem not in reset */

  lpc17_40_configgpio(priv->pins->reset_n      | GPIO_VALUE_ONE);

  /* Switch open */

  lpc17_40_configgpio(priv->pins->power_on_n   | GPIO_VALUE_ONE);

  /* UART shifter disabled */

  lpc17_40_configgpio(priv->pins->shifter_en_n | GPIO_VALUE_ONE);

  /* USB sense off */

  lpc17_40_configgpio(priv->pins->usb_detect   | GPIO_VALUE_ZERO);

  return OK;
}

static int lpc17_40_reset(struct ubxmdm_lower * lower)
{
  struct lpc17_40_ubxmdm_lower * priv =
    (struct lpc17_40_ubxmdm_lower *) lower;

  /* Modem in reset */

  lpc17_40_configgpio(priv->pins->reset_n | GPIO_VALUE_ZERO);

  /* The minimum reset_n low time is 50 ms */

  nxsig_usleep(75 * 1000);

  /* Modem not in reset */

  lpc17_40_configgpio(priv->pins->reset_n | GPIO_VALUE_ONE);

  return OK;
}

static int lpc17_40_getstatus(struct ubxmdm_lower * lower,
                              struct ubxmdm_status * status)
{
  struct lpc17_40_ubxmdm_lower * priv =
    (struct lpc17_40_ubxmdm_lower *) lower;
  int i;

  status->on =
    lpc17_40_gpioread(priv->pins->ldo_enable) &&
    lpc17_40_gpioread(priv->pins->reset_n) &&
    !lpc17_40_gpioread(priv->pins->shifter_en_n);

  DEBUGASSERT(status->register_values_size >= UBXMDM_REGISTER_COUNT);
  status->register_values_size = UBXMDM_REGISTER_COUNT;

  for (i = 0; i < UBXMDM_REGISTER_COUNT; i++)
    {
      strncpy(status->register_values[i].name,
              lpc17_40_ubxmdm_name_pins[i].name,
              3);
      status->register_values[i].val =
        lpc17_40_gpioread(lpc17_40_ubxmdm_name_pins[i].pin);
    }

  return OK;
}

static int lpc17_40_ioctl(struct ubxmdm_lower * lower,
                          int cmd, unsigned long arg)
{
  /* No platform-specific IOCTL at the moment. */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_ubxmdm_init
 *
 * Description:
 *   Initialize the modem.  The modem is initialized and
 *   registered at '/dev/ubxmdm'.
 *
 * Input Parameters:
 *   usb_used - enables the USB sense pin if 'true'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_40_ubxmdm_init(bool usb_used)
{
  struct lpc17_40_ubxmdm_lower * priv = &lpc17_40_ubxmdm_lower;

  DEBUGASSERT(priv->ops == NULL && priv->pins == NULL);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops      = &lpc17_40_ubxmdm_ops;
  priv->pins     = &lpc17_40_ubxmdm_pins;
  priv->usb_used = usb_used;

  lpc17_40_poweroff((struct ubxmdm_lower *) priv);

  ubxmdm_register("/dev/ubxmdm", (struct ubxmdm_lower *) priv);
}
