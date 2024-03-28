/****************************************************************************
 * arch/xtensa/src/esp32/esp32_dac.c
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

#include "xtensa.h"
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>
#include <debug.h>
#include "esp32_dac.h"
#include "esp32_rtc_gpio.h"
#include "hardware/esp32_rtc_io.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_sens.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32_DAC0_RTC_IO_CHANNEL RTCIO_GPIO25_CHANNEL
#define ESP32_DAC1_RTC_IO_CHANNEL RTCIO_GPIO26_CHANNEL

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_dac_priv_s
{
  spinlock_t slock;                        /* Device specific lock. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC methods */

static void dac_reset(struct dac_dev_s *dev);
static int  dac_setup(struct dac_dev_s *dev);
static void dac_shutdown(struct dac_dev_s *dev);
static void dac_txint(struct dac_dev_s *dev, bool enable);
static int  dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg);
static int  dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct esp32_dac_priv_s esp32_dac_priv;

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

static struct dac_dev_s g_dac =
{
  .ad_ops  = &g_dacops,                   /* Arch-specific operations */
  .ad_nchannel = 2,                       /* Available number of DAC channels */
  .ad_priv = (void *) (&esp32_dac_priv),  /* Used by the arch-specific logic */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_power
 *
 * Description:
 *   Power ON or OFF both DAC channels
 *
 * Input Parameters:
 *   bool on - true : turn DAC ON; false : turn DAC OFF
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_power(bool on)
{
  if (on)
    {
      modifyreg32(RTC_IO_PAD_DAC1_REG, 0, RTC_IO_PDAC1_DAC_XPD_FORCE |
                  RTC_IO_PDAC1_XPD_DAC);
      modifyreg32(RTC_IO_PAD_DAC2_REG, 0, RTC_IO_PDAC2_DAC_XPD_FORCE |
                  RTC_IO_PDAC2_XPD_DAC);
    }
  else
    {
      modifyreg32(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC_XPD_FORCE |
                  RTC_IO_PDAC1_XPD_DAC, 0);
      modifyreg32(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC_XPD_FORCE |
                  RTC_IO_PDAC2_XPD_DAC, 0);
    }
}

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 *   NOTE:  DAC reset will reset both DAC channels!
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(struct dac_dev_s *dev)
{
  dac_shutdown(dev);
  dac_setup(dev);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(struct dac_dev_s *dev)
{
  irqstate_t flags;
  struct esp32_dac_priv_s *priv = (struct esp32_dac_priv_s *) dev->ad_priv;
  ainfo("DAC starting setup (for both channels)");

  flags = spin_lock_irqsave(&priv->slock);

  /* Initialize RTC GPIO set to RTC Disabled and disable both pull resistors
   * set RTC_IO_PDACn_MUX_SEL to route the pad to RTC block
   * set RTC_IO_PDACn_DRV to 0x2 (which is default anyway)
   *     Note: Drive strength _DRV (you won't find this in TRM)
   *     0: ~5 mA; 1: ~10 mA; 2: ~20 mA; 3: ~40 mA; the default value is 2.
   * Keep other bits 0, especially:
   * RTC_IO_PDACn_FUN_SEL (2 bits) = mode 0 to choose RTC_GPIO function.
   * RTC_IO_PDACn_FUN_IE to disable Input
   * RTC_IO_PDACn_RUE to disable pull up resistor
   * RTC_IO_PDACn_RDE to disable pull down resistor
   *
   * Note: the following 2 bits are setup separately in dac_power as a last
   * operation.
   * set RTC_IO_PDACn_DAC_XPD_FORCE to power up DAC
   * set RTC_IO_PDACn_XPD_DAC to power on DAC
   */

  uint32_t reg_val = RTC_IO_PDAC1_MUX_SEL |
                     0x2 << RTC_IO_PDAC1_DRV_S;

  /* Write the same value to both registers for DAC 1 and DAC 2 */

  putreg32(reg_val, RTC_IO_PAD_DAC1_REG);
  putreg32(reg_val, RTC_IO_PAD_DAC2_REG);

  /* Disable GPIO output by setting bits in "write 1 to clear" reg */

  modifyreg32(RTC_GPIO_ENABLE_W1TC_REG, 0 ,
              (UINT32_C(1) << (ESP32_DAC0_RTC_IO_CHANNEL +
              RTC_GPIO_ENABLE_W1TC_S)) |
              (UINT32_C(1) << (ESP32_DAC1_RTC_IO_CHANNEL +
              RTC_GPIO_ENABLE_W1TC_S)));

  /* Clear bit of PAD_DRIVER to setup "normal" output mode for the
   * corresponding pads
   */

  modifyreg32(RTC_GPIO_PIN6_REG, RTC_GPIO_PIN6_PAD_DRIVER, 0);
  modifyreg32(RTC_GPIO_PIN7_REG, RTC_GPIO_PIN7_PAD_DRIVER, 0);

  dac_power(true);

  dev->ad_ocount += 1;

  spin_unlock_irqrestore(&priv->slock, flags);

  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC.  This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_shutdown(struct dac_dev_s *dev)
{
  esp32_configrtcio(ESP32_DAC0_RTC_IO_CHANNEL, RTC_FUNCTION_DIGITAL);
  esp32_configrtcio(ESP32_DAC1_RTC_IO_CHANNEL, RTC_FUNCTION_DIGITAL);
  dac_power(false);
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX (transmit) interrupts for the DAC device.
 *   This function is intended to control interrupt-driven data transfers.
 *   Enabling TX interrupts allows the DAC device to generate
 *   an interrupt when it is ready to accept new data for transmission.
 *   Disabling TX interrupts would prevent the DAC from generating these
 *   interrupts.
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device.
 *   enable - Set true to enable TX interrupts. set false to disable
 *            TX interrupts.
 *
 * Returned Value:
 *   None
 *
 * Note:
 *   The actual logic for enabling or disabling TX interrupts is not
 *   implemented in this function!
 *
 ****************************************************************************/

static void dac_txint(struct dac_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   Set the DAC (Digital-to-Analog Converter) output.
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device.
 *   msg - A pointer to the DAC message structure. This structure includes
 *         the data to be sent to the DAC and the target DAC channel.
 *         The 'am_data' field of this structure is the actual data to be
 *         written to the DAC, and 'am_channel' determines which DAC channel
 *         (0 or 1) to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure. -EINVAL is
 *   returned if an invalid channel is specified.
 *
 * Note: The dac_msg_s.am_data is treated as 8 bit value i.e. in range
 *       from 0-255 and corresponds to the analog voltage 0~Vref.
 *       The reference voltage 'Vref' here is input from the pin VDD3P3_RTC
 *       which ideally equals to the power supply VDD (3.3V).
 *       The output voltage can be calculated as the following:
 *       out_voltage = 3.3 * digi_val / 255
 *
 ****************************************************************************/

static int dac_send(struct dac_dev_s *dev, struct dac_msg_s *msg)
{
  irqstate_t flags;
  uint8_t value = (uint8_t)(msg->am_data & 0xff);
  uint32_t reg_val;
  struct esp32_dac_priv_s *priv = (struct esp32_dac_priv_s *) dev->ad_priv;

  flags = spin_lock_irqsave(&priv->slock);

  switch (msg->am_channel)
    {
      case 0:
        modifyreg32(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1, 0);
        reg_val = getreg32(RTC_IO_PAD_DAC1_REG);
        reg_val &= ~RTC_IO_PDAC1_DAC_M;
        reg_val |= value << RTC_IO_PDAC1_DAC_S;
        putreg32(reg_val, RTC_IO_PAD_DAC1_REG);
        break;

      case 1:
        modifyreg32(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2, 0);
        reg_val = getreg32(RTC_IO_PAD_DAC2_REG);
        reg_val &= ~RTC_IO_PDAC2_DAC_M;
        reg_val |= value << RTC_IO_PDAC2_DAC_S;
        putreg32(reg_val, RTC_IO_PAD_DAC2_REG);
        break;

      default:
        spin_unlock_irqrestore(&priv->slock, flags);
        return -EINVAL;
    }

  spin_unlock_irqrestore(&priv->slock, flags);

  /* One shot mode does not support interrupts for DAC. The TX Done is
   * signaled to upper half driver directly from this function because the
   * value is used right away.
   */

  dac_txdone(dev);

  return OK;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   All ioctl (input/output control) calls for the DAC device are routed
 *   through this method. This function handles various control commands
 *   for the DAC device. Currently, it returns -ENOTTY for all commands,
 *   indicating that no command is implemented.
 *
 * Input Parameters:
 *   dev - A pointer to the DAC device structure. This structure contains
 *         information about the DAC device, required for handling the ioctl
 *         commands.
 *   cmd - An integer value representing the ioctl command. These commands
 *         are used to perform various control operations on the DAC device.
 *   arg - An unsigned long value representing additional information or
 *         arguments that are relevant to the ioctl command.
 *         The interpretation of this parameter
 *         depends on the specific command.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure. Currently, it
 *   always returns -ENOTTY, indicating that no ioctl commands are supported.
 *
 ****************************************************************************/

static int dac_ioctl(struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_dac_initialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct dac_dev_s *esp32_dac_initialize(void)
{
  return &g_dac;
}
