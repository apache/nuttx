/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_pinconfig.c
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_arch.h"

#include "cxd56_pinconfig.h"
#include "hardware/cxd5602_topreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GET_IOVAL_MASK(p) \
  ((p) & (PINCONF_DRIVE_MASK | PINCONF_PULL_MASK | PINCONF_IN_EN_MASK))

/* IOCSYS_IOMD0 */

#define GROUP_I2C4          (0)
#define GROUP_PMIC_INT      (2)
#define GROUP_RTC_IRQ_OUT   (4)
#define GROUP_AP_CLK        (6)
#define GROUP_GNSS_1PPS_OUT (8)
#define GROUP_SPI0A         (12)
#define GROUP_SPI0B         (14)
#define GROUP_SPI1A         (16)
#define GROUP_SPI1B         (18)
#define GROUP_SPI2A         (20)
#define GROUP_SPI2B         (22)
#define GROUP_HIFIRQ        (24)
#define GROUP_HIFEXT        (26)

/* IOCSYS_IOMD1 */

#define GROUP_SEN_IRQ_IN    (8)
#define GROUP_SPI3_CS0_X    (10)
#define GROUP_SPI3_CS1_X    (12)
#define GROUP_SPI3_CS2_X    (14)
#define GROUP_SPI3          (16)
#define GROUP_I2C0          (18)
#define GROUP_PWMA          (20)
#define GROUP_PWMB          (22)

/* IOCAPP_IOMD */

#define GROUP_IS            (0)
#define GROUP_UART2         (2)
#define GROUP_SPI4          (4)
#define GROUP_EMMCA         (6)
#define GROUP_EMMCB         (8)
#define GROUP_SDIOA         (10)
#define GROUP_SDIOB         (12)
#define GROUP_SDIOC         (14)
#define GROUP_SDIOD         (16)
#define GROUP_I2S0          (18)
#define GROUP_I2S1          (20)
#define GROUP_MCLK          (22)
#define GROUP_PDM           (24)
#define GROUP_USBVBUS       (26)

/* DBG_HOSTIF_SEL */

#define LATCH_OFF           (1ul << 0)
#define LATCH_OFF_MASK      (1ul << 0)

/* SYSTEM_CONFIG */

#define SYSTEM_CONFIG_I2C   (0)
#define SYSTEM_CONFIG_UART  (1)
#define SYSTEM_CONFIG_SPI   (2)
#define SYSTEM_CONFIG_MON   (3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_mode_regaddr(uint32_t pin, uint32_t *addr, uint32_t *shift)
{
  DEBUGASSERT(addr && shift);

  if ((pin < PIN_I2C4_BCK) || (PIN_USB_VBUSINT < pin))
      return -EINVAL;

  if (pin <= PIN_HIF_GPIO0)
    {
      if (pin <= PIN_I2C4_BDT)
        {
          *shift = GROUP_I2C4;
        }
      else if (pin <= PIN_PMIC_INT)
        {
          *shift = GROUP_PMIC_INT;
        }
      else if (pin <= PIN_RTC_IRQ_OUT)
        {
          *shift = GROUP_RTC_IRQ_OUT;
        }
      else if (pin <= PIN_AP_CLK)
        {
          *shift = GROUP_AP_CLK;
        }
      else if (pin <= PIN_GNSS_1PPS_OUT)
        {
          *shift = GROUP_GNSS_1PPS_OUT;
        }
      else if (pin <= PIN_SPI0_SCK)
        {
          *shift = GROUP_SPI0A;
        }
      else if (pin <= PIN_SPI0_MISO)
        {
          *shift = GROUP_SPI0B;
        }
      else if (pin <= PIN_SPI1_IO1)
        {
          *shift = GROUP_SPI1A;
        }
      else if (pin <= PIN_SPI1_IO3)
        {
          *shift = GROUP_SPI1B;
        }
      else if (pin <= PIN_SPI2_SCK)
        {
          *shift = GROUP_SPI2A;
        }
      else if (pin <= PIN_SPI2_MISO)
        {
          *shift = GROUP_SPI2B;
        }
      else if (pin <= PIN_HIF_IRQ_OUT)
        {
          *shift = GROUP_HIFIRQ;
        }
      else
        {
          *shift = GROUP_HIFEXT;
        }

      *addr = CXD56_TOPREG_IOCSYS_IOMD0;
    }
  else if (pin <= PIN_PWM3)
    {
      if (pin <= PIN_SEN_IRQ_IN)
        {
          *shift = GROUP_SEN_IRQ_IN;
        }
      else if (pin <= PIN_SPI3_CS0_X)
        {
          *shift = GROUP_SPI3_CS0_X;
        }
      else if (pin <= PIN_SPI3_CS1_X)
        {
          *shift = GROUP_SPI3_CS1_X;
        }
      else if (pin <= PIN_SPI3_CS2_X)
        {
          *shift = GROUP_SPI3_CS2_X;
        }
      else if (pin <= PIN_SPI3_MISO)
        {
          *shift = GROUP_SPI3;
        }
      else if (pin <= PIN_I2C0_BDT)
        {
          *shift = GROUP_I2C0;
        }
      else if (pin <= PIN_PWM1)
        {
          *shift = GROUP_PWMA;
        }
      else
        {
          *shift = GROUP_PWMB;
        }

      *addr = CXD56_TOPREG_IOCSYS_IOMD1;
    }
  else
    {
      if (pin <= PIN_IS_DATA7)
        {
          *shift = GROUP_IS;
        }
      else if (pin <= PIN_UART2_RTS)
        {
          *shift = GROUP_UART2;
        }
      else if (pin <= PIN_SPI4_MISO)
        {
          *shift = GROUP_SPI4;
        }
      else if (pin <= PIN_EMMC_DATA1)
        {
          *shift = GROUP_EMMCA;
        }
      else if (pin <= PIN_EMMC_DATA3)
        {
          *shift = GROUP_EMMCB;
        }
      else if (pin <= PIN_SDIO_DATA3)
        {
          *shift = GROUP_SDIOA;
        }
      else if (pin <= PIN_SDIO_WP)
        {
          *shift = GROUP_SDIOB;
        }
      else if (pin <= PIN_SDIO_DIR1_3)
        {
          *shift = GROUP_SDIOC;
        }
      else if (pin <= PIN_SDIO_CLKI)
        {
          *shift = GROUP_SDIOD;
        }
      else if (pin <= PIN_I2S0_DATA_OUT)
        {
          *shift = GROUP_I2S0;
        }
      else if (pin <= PIN_I2S1_DATA_OUT)
        {
          *shift = GROUP_I2S1;
        }
      else if (pin <= PIN_MCLK)
        {
          *shift = GROUP_MCLK;
        }
      else if (pin <= PIN_PDM_OUT)
        {
          *shift = GROUP_PDM;
        }
      else
        {
          *shift = GROUP_USBVBUS;
        }

      *addr = CXD56_TOPREG_IOCAPP_IOMD;
    }

  return 0;
}

static void set_i2s_output_config(uint32_t pin, uint32_t mode, bool is_slave)
{
  uint32_t mask =
    (PIN_I2S0_BCK == pin) ? (I2S0_BCK | I2S0_LRCK) : (I2S1_BCK | I2S1_LRCK);

  if (is_slave)
    {
      modifyreg32(CXD56_TOPREG_IOOEN_APP, 0, mask);
    }
  else
    {
      modifyreg32(CXD56_TOPREG_IOOEN_APP, mask, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Value:
 *   32-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_pin_config(uint32_t pinconf)
{
  return cxd56_pin_configs(&pinconf, 1);
}

/****************************************************************************
 * Name: cxd56_pin_configs
 *
 * Description:
 *   Configure multiple pins based on bit-encoded description of the pin.
 *
 * Input Value:
 *   pinconfs[] - Array of 32-bit encoded value describing the pin.
 *   n - The number of elements in the array.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_pin_configs(uint32_t pinconfs[], size_t n)
{
  int ret = 0;
  int i;
  uint32_t pin;
  uint32_t mode;
  uint32_t ioreg;
  uint32_t ioval;
  uint32_t modereg;
  uint32_t shift;
  uint32_t oldreg = 0;
  uint32_t oldshift = 0;
  uint32_t latch_endpin = PIN_SPI2_SCK;

  if (SYSTEM_CONFIG_SPI == getreg8(CXD56_TOPREG_SYSTEM_CONFIG))
    {
      latch_endpin = PIN_SPI2_MISO;
    }

  for (i = 0; i < n; i++)
    {
      pin = PINCONF_GET_PIN(pinconfs[i]);
      mode = PINCONF_GET_MODE(pinconfs[i]);
      ioval = GET_IOVAL_MASK(pinconfs[i]);

      DEBUGASSERT((PIN_RTC_CLK_IN <= pin) && (pin <= PIN_USB_VBUSINT));
      DEBUGASSERT((pin <= PIN_GNSS_1PPS_OUT) || (PIN_SPI0_CS_X <= pin));
      DEBUGASSERT((pin <= PIN_HIF_GPIO0) || (PIN_SEN_IRQ_IN <= pin));
      DEBUGASSERT((pin <= PIN_PWM3) || (PIN_IS_CLK <= pin));

      /* Set HostIF latch off */

      if ((PIN_SPI2_CS_X <= pin) && (pin <= latch_endpin))
        {
          modifyreg32(CXD56_TOPREG_DBG_HOSTIF_SEL,
                      LATCH_OFF_MASK,
                      LATCH_OFF);
        }

      /* Set IO cell register */

      ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
      putreg32(ioval, ioreg);

      /* Set i2s output register */

      if (((PIN_I2S0_BCK == pin) || (PIN_I2S1_BCK == pin)) &&
          (PINCONF_MODE1 == mode))
        {
          set_i2s_output_config(pin, mode,
              PINCONF_INPUT_ENABLED(pinconfs[i]));
        }

      ret = get_mode_regaddr(pin, &modereg, &shift);

      if ((!ret) && ((oldreg != modereg) || (oldshift != shift)))
        {
          oldreg = modereg;
          oldshift = shift;

          /* Set alternative mode register */

          modifyreg32(modereg, (0x3 << shift), (mode << shift));
        }
    }

  return 0;
}

/****************************************************************************
 * Name: cxd56_pin_status
 *
 * Description:
 *   Get a pin status.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int cxd56_pin_status(uint32_t pin, cxd56_pin_status_t *stat)
{
  int ret = 0;
  uint32_t ioreg;
  uint32_t ioval;
  uint32_t modereg;
  uint32_t modeval;
  uint32_t shift;

  DEBUGASSERT(stat);

  stat->mode     = -1;
  stat->input_en = -1;
  stat->drive    = -1;
  stat->pull     = -1;

  if (((PIN_GNSS_1PPS_OUT < pin) && (pin < PIN_SPI0_CS_X)) ||
      ((PIN_HIF_GPIO0 < pin) && (pin < PIN_SEN_IRQ_IN)) ||
      ((PIN_PWM3 < pin) && (pin < PIN_IS_CLK)) || (PIN_USB_VBUSINT < pin))
    {
      return -EINVAL;
    }

  ioreg = CXD56_TOPREG_IO_RTC_CLK_IN + (pin * 4);
  ioval = getreg32(ioreg);

  ret = get_mode_regaddr(pin, &modereg, &shift);

  if (!ret)
    {
      modeval = getreg32(modereg);
      modeval = (modeval >> shift) & 0x3;

      stat->mode = modeval;
      stat->input_en = (ioval & PINCONF_IN_EN_MASK);
      stat->drive = (ioval & PINCONF_DRIVE_MASK);
      stat->pull = (ioval & PINCONF_PULL_MASK);
    }

  return ret;
}
