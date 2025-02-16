/****************************************************************************
 * drivers/wireless/lpwan/sx126x/sx126x.c
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

#include "sx126x.h"

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/ioctl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/endian.h>
#include <unistd.h>
#include <nuttx/wireless/lpwan/sx126x.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Private prototypes for file operations
 ****************************************************************************/

static int sx126x_open(FAR struct file *filep);

static int sx126x_close(FAR struct file *filep);

static ssize_t sx126x_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t buflen);

static ssize_t sx126x_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen);

static int sx126x_ioctl(FAR struct file *filep,
                        int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private data types
 ****************************************************************************/

struct sx126x_dev_s
{
  struct spi_dev_s *spi;
  const struct sx126x_lower_s *lower;
  uint8_t times_opened;
  mutex_t lock;                           /* Only let one user in at a time */
  sem_t rx_sem;
  sem_t tx_sem;
  uint16_t irqbits;

  /* Hardware settings */

  bool invert_iq;

  /* Common settings */

  uint8_t payload_len;
  enum sx126x_packet_type_e packet_type;  /* This will decide what modulation to use */
  uint32_t frequency_hz;
  uint8_t power;
  uint16_t preambles;

  /* LoRa settings */

  enum sx126x_lora_sf_e lora_sf;
  enum sx126x_lora_bw_e lora_bw;
  enum sx126x_lora_cr_e lora_cr;
  bool lora_crc;
  bool lora_fixed_header;
  bool low_datarate_optimization;
  uint8_t syncword[SX126X_REG_SYNCWORD_LEN];

  /* Interrupt handling */

  uint16_t irq_mask;
  struct work_s irq0_work;
};

enum sx126x_cmd_status
{
  SX126X_STATUS_RESERVED, SX126X_STATUS_RFU, SX126X_STATUS_DATA_AVAILABLE,
  SX126X_STATUS_TIMEOUT, SX126X_STATUS_ERROR, SX126X_STATUS_EXECUTE_FAIL,
  SX126X_STATUS_TX_DONE
};

enum sx126x_chip_mode
{
  SX126X_MODE_UNUSED, SX126X_MODE_RFU, SX126X_MODE_STBY_RC,
  SX126X_MODE_STBY_XOSC, SX126X_MODE_FS, SX126X_MODE_RX, SX126X_MODE_TX
};

struct sx126x_status_s
{
  enum sx126x_cmd_status cmd;
  enum sx126x_chip_mode mode;
};

static const struct file_operations sx126x_ops =
{
  sx126x_open,
  sx126x_close,
  sx126x_read,
  sx126x_write,
  NULL,
  sx126x_ioctl,
  NULL,
  NULL
};

/****************************************************************************
 * Globals
 ****************************************************************************/

FAR struct sx126x_dev_s g_sx126x_devices[SX126X_MAX_DEVICES];

/****************************************************************************
 * Private prototypes
 ****************************************************************************/

/* SPI and control **********************************************************/

static void sx126x_command(FAR struct sx126x_dev_s *dev,
                           uint8_t cmd,
                           FAR const uint8_t *params,
                           size_t paramslen,
                           FAR uint8_t *returns);

static void sx126x_reset(FAR struct sx126x_dev_s *dev);

static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                              FAR struct sx126x_status_s *status);

static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev);

static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev);

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length);

/* Operational modes functions **********************************************/

static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                               enum sx126x_standby_mode_e mode);

static void sx126x_set_tx(FAR struct sx126x_dev_s *dev,
                          uint32_t timeout);

static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t timeout);

static void sx126x_set_cad(struct sx126x_dev_s *dev);

static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev);

static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                      enum sx126x_regulator_mode_e mode);

static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                 enum sx126x_device_e model,
                                 uint8_t hpmax,
                                 uint8_t padutycycle);

static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev);

/* DIO and IRQ control functions ********************************************/

static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                      uint16_t irq_mask,
                                      uint16_t dio1_mask,
                                      uint16_t dio2_mask,
                                      uint16_t dio3_mask);

static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                         bool enable);

static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_tcxo_voltage_e voltage,
                                    uint32_t delay);

static void sx126x_get_irq_status(FAR struct sx126x_dev_s *dev,
                                  FAR uint16_t *irqstatus);

static void sx126x_clear_irq_status(FAR struct sx126x_dev_s *dev,
                                    uint16_t clearbits);

/* RF Modulation and Packet-Related Functions *******************************/

static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                          FAR struct
                                          sx126x_packetparams_lora_s *
                                          pktparams);

static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                              FAR struct
                                              sx126x_modparams_lora_s *
                                              modparams);

static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                           uint8_t tx,
                                           uint8_t rx);

static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                 enum sx126x_ramp_time_e ramp_time);

static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                   enum sx126x_packet_type_e type);

static void sx126x_set_rf_frequency(FAR struct sx126x_dev_s *dev,
                                    uint32_t frequency_hz);

/* Communication status information *****************************************/

static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                 FAR int32_t *dbm);

static void sx126x_get_rx_buffer_status(FAR struct sx126x_dev_s *dev,
                                        uint8_t *status,
                                        uint8_t *payload_len,
                                        uint8_t *rx_buff_offset);

/* Registers and buffer *****************************************************/

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length);

static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len);

static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len);

/* Register settings ********************************************************/

static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length);

/* Driver specific **********************************************************/

static int sx126x_init(FAR struct sx126x_dev_s *dev);

static int sx126x_deinit(FAR struct sx126x_dev_s *dev);

static int sx126x_setup_radio(FAR struct sx126x_dev_s *dev);

static void sx126x_set_defaults(FAR struct sx126x_dev_s *dev);

/* Interrupt handlers *******************************************************/

static int sx126x_irq0handler(int irq, FAR void *context, FAR void *arg);

static inline int sx126x_attachirq0(FAR struct sx126x_dev_s *dev, xcpt_t isr,
  FAR void *arg);

static void sx126x_isr0_process(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* File operations **********************************************************/

static int sx126x_open(FAR struct file *filep)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;
  dev = filep->f_inode->i_private;
  wlinfo("Opening SX126x %d", dev->lower->dev_number);

  /* Lock dev */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Only one can open this dev at a time */

  if (dev->times_opened > 0)
    {
      ret = -EBUSY;
      goto exit_err;
    }

  /* Initialize */

  ret = sx126x_init(dev);
  if (ret != 0)
    {
      goto exit_err;
    }

  /* Success */

  dev->times_opened++;
  ret = OK;

exit_err:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int sx126x_close(FAR struct file *filep)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;
  dev = filep->f_inode->i_private;
  wlinfo("Closing SX126x %d", dev->lower->dev_number);

  /* Lock */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      goto exit_err;
    }

  /* De-init */

  ret = sx126x_deinit(dev);
  if (ret != 0)
    {
      goto exit_err;
    }

  /* Success */

  if (dev->times_opened > 0)
    {
      dev->times_opened--; /* Do not let this wrap around. */
      ret = OK;
    }

exit_err:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static ssize_t sx126x_read(FAR struct file *filep,
                           FAR char *buf,
                           size_t buflen)
{
  int ret = 0;
  if (buf == NULL || buflen < 1)
    {
      return -EINVAL;
    }

  /* Get device */

  struct sx126x_dev_s *dev;
  dev = filep->f_inode->i_private;

  nxmutex_lock(&dev->lock);

  printf("Reading\n");

  /* Get header */

  struct sx126x_read_header_s *header = (struct sx126x_read_header_s *)buf;

  /* Pre-RX setup */

  sx126x_spi_lock(dev);
  dev->irq_mask = SX126X_IRQ_RXDONE_MASK | SX126X_IRQ_CRCERR_MASK;
  ret = sx126x_setup_radio(dev);
  if (ret != 0)
    {
      goto sx126x_rx_abort;
    }

  /* RX mode */

  sx126x_set_rx(dev, SX126X_NO_TIMEOUT);
  sx126x_spi_unlock(dev);

  /* Wait for a packet */

  nxsem_wait(&dev->rx_sem);

  /* Get payload */

  uint8_t status = 0;
  uint8_t offset = 0;

  sx126x_spi_lock(dev);
  sx126x_get_rx_buffer_status(dev, &status,
                              &header->payload_length,
                              &offset);
  sx126x_read_buffer(dev, offset, header->payload,
                     header->payload_length);
  sx126x_spi_unlock(dev);

  /* Get CRC check */

  header->crc_error = dev->irqbits & SX126X_IRQ_CRCERR_MASK;

  /* Exit */

  sx126x_rx_abort:

  nxmutex_unlock(&dev->lock);

  return 1;
}

static ssize_t sx126x_write(FAR struct file *filep,
                            FAR const char *buf,
                            size_t buflen)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;
  dev = filep->f_inode->i_private;

  if (buf == NULL || buflen < 1)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  sx126x_spi_lock(dev);

  /* Data */

  dev->payload_len = buflen;
  sx126x_write_buffer(dev, 0, (uint8_t *)buf, buflen);

  /* Pre-TX setup */

  dev->irq_mask = SX126X_IRQ_TXDONE_MASK;
  ret = sx126x_setup_radio(dev);
  if (ret != 0)
    {
      sx126x_spi_unlock(dev);
      goto sx126x_tx_abort;
    }

  /* TX */

  sx126x_set_tx(dev, 0);

  sx126x_spi_unlock(dev);

  /* Wait for transmitting operations to be finished */

  wlinfo("TXing");
  ret = nxsem_wait(&dev->tx_sem);

  sx126x_tx_abort:

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int sx126x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = 0;

  /* Get device */

  struct sx126x_dev_s *dev;
  dev = filep->f_inode->i_private;
  wlinfo("IOCTL cmd %d arg %u SX126x dev_number %d",
    cmd,
    *(FAR uint32_t *)((uintptr_t)arg),
    dev->lower->dev_number);

  /* Lock */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      goto exit_err;
    }

  /* Do thing */

  switch (cmd)
    {
      /* Set radio freq. Takes uint32_t *frequency in Hz */

      case WLIOC_SETRADIOFREQ:
        {
          FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(freq_ptr != NULL);

          dev->frequency_hz = *freq_ptr;
          break;
        }

      /* Get radio freq. Sets uint32_t *frequency in Hz */

      case WLIOC_GETRADIOFREQ:
        {
          FAR uint32_t *freq_ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(freq_ptr != NULL);

           *freq_ptr = dev->frequency_hz;
          break;
        }

      /* Set TX power. arg: Pointer to int8_t power value */

      case WLIOC_SETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          dev->power = *ptr;
          break;
        }

      /* Get current TX power. arg: Pointer to int8_t power value */

      case WLIOC_GETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          *ptr = dev->power;
          break;
        }

      /* TODO: Integration with new common IOCTL API */

      /* Driver specific IOCTL */

      /* Lora config */

      case SX126XIOC_LORACONFIGSET:
        {
          FAR struct sx126x_lora_config_s *ptr =
            (FAR struct sx126x_lora_config_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          /* Modulation params */

          dev->lora_sf = ptr->modulation.spreading_factor;
          dev->lora_bw = ptr->modulation.bandwidth;
          dev->lora_cr = ptr->modulation.coding_rate;
          dev->low_datarate_optimization =
            ptr->modulation.low_datarate_optimization;

          /* Packet params */

          dev->lora_crc = ptr->packet.crc_enable;
          dev->lora_fixed_header =
            ptr->packet.fixed_length_header;
          dev->payload_len = ptr->packet.payload_length;
          dev->invert_iq = ptr->packet.invert_iq;
          dev->preambles = ptr->packet.preambles;

          break;
        }
    }

  /* Success */

  ret = OK;

exit_err:
  nxmutex_unlock(&dev->lock);
  return ret;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step(uint32_t freq_in_hz)
{
  uint32_t  steps_int;
  uint32_t  steps_frac;

  steps_int     = freq_in_hz / SX126X_PLL_STEP_SCALED;
  steps_frac    = freq_in_hz - (steps_int * SX126X_PLL_STEP_SCALED);

  return (steps_int <<
    SX126X_PLL_STEP_SHIFT_AMOUNT) + (((steps_frac <<
    SX126X_PLL_STEP_SHIFT_AMOUNT) + (SX126X_PLL_STEP_SCALED >>
                                     1)) / SX126X_PLL_STEP_SCALED);
}

/* Operational modes functions **********************************************/

static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                               enum sx126x_standby_mode_e mode)
{
  sx126x_command(dev, SX126X_SETSTANDBY, (uint8_t *)&mode,
                 SX126X_SETSTANDBY_PARAMS, NULL);
}

static void sx126x_set_tx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
{
  /* Convert timeout to BE24 */

  timeout = htobe32(timeout << 8);

  sx126x_command(dev, SX126X_SETTX, (uint8_t *)&timeout, SX126X_SETTX_PARAMS,
                 NULL);
}

static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t timeout)
{
  /* Convert timeout to BE24 */

  timeout = htobe32(timeout << 8);

  sx126x_command(dev, SX126X_SETRX, (uint8_t *)&timeout, SX126X_SETRX_PARAMS,
                 NULL);
}

static void sx126x_stop_timer_on_preamble(FAR struct sx126x_dev_s *dev,
                                          bool enable)
{
  sx126x_command(dev, SX126X_STOPTIMERONPREAMBLE, (uint8_t *)&enable,
                 SX126X_STOPTIMERONPREAMBLE_PARAMS, NULL);
}

static void sx126x_set_rx_duty_cycle(FAR struct sx126x_dev_s *dev,
                                     uint32_t rx_period,
                                     uint32_t sleep_period)
{
  uint8_t params[SX126X_SETRXDUTYCYCLE_PARAMS];

  rx_period     = htobe32(rx_period << 8);
  sleep_period  = htobe32(sleep_period << 8);

  memcpy(params + SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAM,
         (uint8_t *)&rx_period,
         SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAMS);
  memcpy(params + SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM,
         (uint8_t *)&sleep_period,
         SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAMS);

  sx126x_command(dev, SX126X_SETRXDUTYCYCLE, params,
                 SX126X_SETRXDUTYCYCLE_PARAMS, NULL);
}

static void sx126x_set_cad(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETCAD, NULL, 0, NULL);
}

static void sx126x_set_tx_continuous_wave(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETTXCONTINUOUSWAVE, NULL, 0, NULL);
}

static void sx126x_set_regulator_mode(FAR struct sx126x_dev_s *dev,
                                      enum sx126x_regulator_mode_e mode)
{
  sx126x_command(dev, SX126X_SETREGULATORMODE, (uint8_t *)&mode,
                 SX126X_SETREGULATORMODE_PARAMS, NULL);
}

/* Caution! Exceeding the limits listed in DS_SX1261/2 V2.1
 * 13.1.14.1 may cause irreversible damage to the device
 */

static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev,
                                 enum sx126x_device_e model, uint8_t hpmax,
                                 uint8_t padutycycle)
{
  uint8_t params[SX126X_SETPACONFIG_PARMS];

  memset(params, 0, SX126X_SETPACONFIG_PARMS);

  params[SX126X_SETPACONFIG_PADUTYCYCLE_PARAM]  = padutycycle;
  params[SX126X_SETPACONFIG_HPMAX_PARAM]        = hpmax;
  params[SX126X_SETPACONFIG_DEVICESEL_PARAM]    = model;
  params[SX126X_SETPACONFIG_PALUT_PARAM]        = 0x01;

  sx126x_command(dev, SX126X_SETPACONFIG, params, SX126X_SETPACONFIG_PARMS,
                 NULL);
}

static void sx126x_set_tx_infinite_preamble(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_SETTXINFINITEPREAMBLE, NULL, 0, NULL);
}

static void sx126x_set_rx_tx_fallback_mode(FAR struct sx126x_dev_s *dev,
                                           enum sx126x_fallback_mode_e
                                           fallback)
{
  sx126x_command(dev, SX126X_SETRXTXFALLBACKMODE, (uint8_t *)&fallback,
                 SX126X_SETRXTXFALLBACKMODE_PARAMS, NULL);
}

/* DIO and IRQ control functions */

static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev,
                                      uint16_t irq_mask, uint16_t dio1_mask,
                                      uint16_t dio2_mask, uint16_t dio3_mask)
{
  irq_mask  = htobe16(irq_mask);
  dio1_mask = htobe16(dio1_mask);
  dio2_mask = htobe16(dio2_mask);
  dio3_mask = htobe16(dio3_mask);

  uint8_t params[SX126X_SETDIOIRQPARAMS_PARAMS];

  memcpy(params + SX126X_SETDIOIRQPARAMS_IRQMASK_PARAM, &irq_mask,
         SX126X_SETDIOIRQPARAMS_IRQMASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAM, &dio1_mask,
         SX126X_SETDIOIRQPARAMS_DIO1MASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAM, &dio2_mask,
         SX126X_SETDIOIRQPARAMS_DIO2MASK_PARAMS);

  memcpy(params + SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAM, &dio3_mask,
         SX126X_SETDIOIRQPARAMS_DIO3MASK_PARAMS);

  sx126x_command(dev, SX126X_SETDIOIRQPARAMS, params,
                 SX126X_SETDIOIRQPARAMS_PARAMS, NULL);
}

static void sx126x_set_dio2_as_rf_switch(FAR struct sx126x_dev_s *dev,
                                         bool enable)
{
  sx126x_command(dev, SX126X_SETDIO2RFSWCTRL, (uint8_t *)&enable,
                 SX126X_SETDIO2RFSWCTRL_PARAMS, NULL);
}

static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev,
                                    enum sx126x_tcxo_voltage_e voltage,
                                    uint32_t delay)
{
  uint8_t params[SX126X_SETDIO3TCXOCTRL_PARAMS];

  params[SX126X_SETDIO3TCXOCTRL_TCXO_V_PARAM] = voltage;

  /* Convert delay to 24 bit and convert to BE */

  delay = htobe32(delay << 8);
  memcpy(params + SX126X_SETDIO3TCXOCTRL_DELAY_PARAM, &delay,
         SX126X_SETDIO3TCXOCTRL_DELAY_PARAMS);

  sx126x_command(dev, SX126X_SETDIO3TCXOCTRL, params,
                 SX126X_SETDIO3TCXOCTRL_PARAMS, NULL);
}

static void sx126x_get_irq_status(FAR struct sx126x_dev_s *dev,
                                  FAR uint16_t *irqstatus)
{
  uint8_t returns[SX126X_GETIRQSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETIRQSTATUS,
                 NULL, SX126X_GETIRQSTATUS_RETURNS,
                 returns);

  uint16_t bits;
  memcpy(&bits, returns +
         SX126X_GETIRQSTATUS_IRQSTATUS_RETURN,
         SX126X_GETIRQSTATUS_IRQSTATUS_RETURNS);

  *irqstatus = be16toh(bits);
}

static void sx126x_clear_irq_status(FAR struct sx126x_dev_s *dev,
                                    uint16_t clearbits)
{
  uint8_t params[SX126X_CLEARIRQSTATUS_PARAMS];

  clearbits = htobe16(clearbits);
  memcpy(params + SX126X_CLEARIRQSTATUS_CLEAR_PARAM,
         &clearbits,
         SX126X_CLEARIRQSTATUS_CLEAR_PARAMS);

  sx126x_command(dev, SX126X_CLEARIRQSTATUS, params,
                 SX126X_CLEARIRQSTATUS_PARAMS,
                 NULL);
}

/* RF Modulation and Packet-Related Functions *******************************/

static void sx126x_set_packet_params_lora(FAR struct sx126x_dev_s *dev,
                                          FAR struct
                                          sx126x_packetparams_lora_s *
                                          pktparams)
{
  uint8_t params[SX126X_SETPACKETPARMS_PARAMS];

  memset(params, 0, SX126X_SETPACKETPARMS_PARAMS);

  uint16_t preambles = htobe16(pktparams->preambles);
  memcpy(params + SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAM, &preambles,
         SX126X_PKTPARAM1_LORA_PREAMBLELEN_PARAMS);

  params[SX126X_PKTPARAM3_LORA_HEADERTYPE_PARAM] =
    pktparams->fixed_length_header;
  params[SX126X_PKTPARAM4_LORA_PAYLOADLEN_PARAM] =
    pktparams->payload_length;
  params[SX126X_PKTPARAM5_LORA_CRCTYPE_PARAM]   = pktparams->crc_enable;
  params[SX126X_PKTPARAM6_LORA_INVERTIQ_PARAM]  = pktparams->invert_iq;

  sx126x_command(dev, SX126X_SETPACKETPARMS, params,
                 SX126X_SETPACKETPARMS_PARAMS, NULL);
}

static void sx126x_set_modulation_params_lora(FAR struct sx126x_dev_s *dev,
                                              FAR struct
                                              sx126x_modparams_lora_s *
                                              modparams)
{
  uint8_t params[SX126X_SETMODULATIONPARAMS_PARAMS];

  memset(params, 0, SX126X_SETMODULATIONPARAMS_PARAMS);

  params[SX126X_MODPARAM1_LORA_SF_PARAM] =
    modparams->spreading_factor;
  params[SX126X_MODPARAM2_LORA_BW_PARAM] =
    modparams->bandwidth;
  params[SX126X_MODPARAM3_LORA_CR_PARAM] =
    modparams->coding_rate;
  params[SX126X_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM] =
    modparams->low_datarate_optimization;

  sx126x_command(dev, SX126X_SETMODULATIONPARAMS, params,
                 SX126X_SETMODULATIONPARAMS_PARAMS, NULL);
}

static void sx126x_set_buffer_base_address(FAR struct sx126x_dev_s *dev,
                                           uint8_t tx, uint8_t rx)
{
  uint8_t params[SX126X_SETBUFFERBASEADDRESS_PARAMS];

  memset(params, 0, SX126X_SETBUFFERBASEADDRESS_PARAMS);

  params[SX126X_SETBUFFERBASEADDRESS_TX_PARAM]  = tx;
  params[SX126X_SETBUFFERBASEADDRESS_RX_PARAM]  = rx;

  sx126x_command(dev, SX126X_SETBUFFERBASEADDRESS, params,
                 SX126X_SETBUFFERBASEADDRESS_PARAMS, NULL);
}

static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev, uint8_t power,
                                 enum sx126x_ramp_time_e ramp_time)
{
  uint8_t params[SX126X_SETTXPARMS_PARAMS];

  memset(params, 0, SX126X_SETTXPARMS_PARAMS);

  params[SX126X_SETTXPARMS_RAMPTIME_PARAM]  = ramp_time;
  params[SX126X_SETTXPARMS_POWER_PARAM]     = power;

  sx126x_command(dev, SX126X_SETTXPARMS, params, SX126X_SETTXPARMS_PARAMS,
                 NULL);
}

static void sx126x_set_packet_type(FAR struct sx126x_dev_s *dev,
                                   enum sx126x_packet_type_e type)
{
  sx126x_command(dev, SX126X_SETPACKETTYPE, (uint8_t *)&type,
                 SX126X_SETPACKETTYPE_PARAMS, NULL);
}

static void sx126x_set_rf_frequency(FAR struct sx126x_dev_s *dev,
                                    uint32_t frequency_hz)
{
  uint32_t corrected_freq =
    sx126x_convert_freq_in_hz_to_pll_step(frequency_hz);

  corrected_freq = htobe32(corrected_freq);

  sx126x_command(dev, SX126X_SETRFFREQUENCY, (uint8_t *)&corrected_freq,
                 SX126X_SETRFFREQUENCY_PARAMS, NULL);
}

static void sx126x_set_lora_symb_num_timout(FAR struct sx126x_dev_s *dev,
                                            uint8_t symbnum)
{
  sx126x_command(dev, SX126X_SETLORASYMBNUMTIMEOUT, &symbnum,
                 SX126X_SETLORASYMBNUMTIMEOUT_PARAMS, NULL);
}

/* Communication status information *****************************************/

static void sx126x_get_status(FAR struct sx126x_dev_s *dev,
                              FAR struct sx126x_status_s *status)
{
  /* NOP param to shift out result */

  uint8_t   parms[1] = {
    0x00
  };

  uint8_t   rets[1];

  /* Get, mask and shift result into readable mode numbers */

  sx126x_command(dev, SX126X_CMD_GETSTATUS, parms, sizeof(parms), rets);
  status->mode = (rets[0] & SX126X_STATUS_CHIPMODE_MASK) >>
                 SX126X_STATUS_CHIPMODE_SHIFT;
  status->cmd = (rets[0] & SX126X_STATUS_CMD_MASK) >>
                SX126X_STATUS_CMD_SHIFT;
}

static void sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev,
                                 FAR int32_t *dbm)
{
  uint8_t rets[SX126X_GETRSSIINST_RETURNS];

  sx126x_command(dev, SX126X_GETRSSIINST, NULL, SX126X_GETRSSIINST_RETURNS,
                 rets);

  /* Calculate dBm from returns */

  int32_t rssi = rets[SX126X_GETRSSIINST_RSSI_RETURN];
  (*dbm) = -rssi / 2.0;
}

static void sx126x_get_rx_buffer_status(FAR struct sx126x_dev_s *dev,
  uint8_t *status,
  uint8_t *payload_len,
  uint8_t *rx_buff_offset)
{
  uint8_t returns[SX126X_GETRXBUFFERSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETRXBUFFERSTATUS,
  NULL,
  SX126X_GETRXBUFFERSTATUS_RETURNS,
  returns);

  *status = returns[SX126X_GETRXBUFFERSTATUS_STATUS_RETURN];
  *payload_len = returns[SX126X_GETRXBUFFERSTATUS_PAYLOAD_LEN_RETURN];
  *rx_buff_offset = returns[SX126X_GETRXBUFFERSTATUS_RX_START_PTR_RETURN];
}

/* Lower hardware control ***************************************************/

static void sx126x_reset(FAR struct sx126x_dev_s *dev)
{
  dev->lower->reset();
}

/* SPI Communication ********************************************************/

static void sx126x_select(FAR struct sx126x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), true);
}

static void sx126x_deselect(FAR struct sx126x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), false);
}

static void sx126x_spi_lock(FAR struct sx126x_dev_s *dev)
{
  struct spi_dev_s *spi = dev->spi;

  SPI_LOCK(spi, true);
  SPI_SETBITS(spi, 8);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, SX126X_SPI_SPEED);
}

static void sx126x_spi_unlock(FAR struct sx126x_dev_s *dev)
{
  SPI_LOCK(dev->spi, false);
}

static void sx126x_command(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                           const FAR uint8_t *params, size_t paramslen,
                           FAR uint8_t *returns)
{
  sx126x_select(dev);

  /* First send the command. This does not return anything.
   * "RFU" according the manual
   */

  SPI_SEND(dev->spi, cmd);

  /* Send all the params and record the returning bytes */

  for (size_t i = 0; i < paramslen; i++)
    {
      uint8_t param = SX126X_NOP;
      if (params != NULL)
        {
          param = params[i];
        }

      uint8_t ret = SPI_SEND(dev->spi, param);

      if (returns != NULL)
        {
          returns[i] = ret;
        }
    }

  sx126x_deselect(dev);
}

/* Registers and buffer *****************************************************/

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  uint8_t *data,
                                  size_t data_length)
{
  sx126x_select(dev);

  /* Send the opcode and address */

  SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);

  /* Send data */

  for (size_t i = 0; i < data_length; i++)
    {
      SPI_SEND(dev->spi, data[i]);
    }

  sx126x_deselect(dev);
}

static void sx126x_read_register(FAR struct sx126x_dev_s *dev,
                                 uint16_t address,
                                 uint8_t *data,
                                 size_t data_length)
{
  sx126x_select(dev);

  /* Send the opcode and address */

  SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);

  /* Send data */

  for (size_t i = 0; i < data_length; i++)
    {
      data[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_deselect(dev);
}

static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *payload,
                                uint8_t len)
{
  sx126x_select(dev);

  /* Command */

  SPI_SEND(dev->spi, SX126X_WRITEBUFFER);

  /* Offset */

  SPI_SEND(dev->spi, offset);

  /* Data */

  for (size_t i = 0; i < len; i++)
    {
      SPI_SEND(dev->spi, payload[i]);
    }

  sx126x_deselect(dev);
}

static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *payload,
                               uint8_t len)
{
  sx126x_select(dev);

  /* Command */

  SPI_SEND(dev->spi, SX126X_READBUFFER);

  /* Offset */

  SPI_SEND(dev->spi, offset);

  /* NOP */

  SPI_SEND(dev->spi, SX126X_NOP);

  /* Data */

  for (size_t i = 0; i < len; i++)
    {
      payload[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_deselect(dev);
}

/* Register settings ********************************************************/

static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev,
                                uint8_t *syncword,
                                uint8_t syncword_length)
{
  if (syncword_length > SX126X_REG_SYNCWORD_LEN)
    {
      syncword_length = SX126X_REG_SYNCWORD_LEN;
      wlerr("Syncword length was limited to the maximum 8 bytes");
    }

  sx126x_write_register(dev, SX126X_REG_SYNCWORD, syncword, syncword_length);
}

/* Driver specific **********************************************************/

static int sx126x_init(FAR struct sx126x_dev_s *dev)
{
  sx126x_reset(dev);
  sx126x_set_defaults(dev);
  return 0;
}

static int sx126x_deinit(FAR struct sx126x_dev_s *dev)
{
  return 0;
}

static void sx126x_set_defaults(FAR struct sx126x_dev_s *dev)
{
  /* Hardware defaults */

  dev->invert_iq = SX126X_DEFAULT_INVERT_IQ;

  /* Common defaults */

  dev->packet_type    = SX126X_DEFAULT_PACKET_TYPE;
  dev->frequency_hz   = SX126X_DEFAULT_FREQ;
  dev->power          = SX126X_DEFAULT_POWER;
  dev->preambles      = SX126X_DEFAULT_LORA_PREAMBLES;

  /* LoRa defaults */

  dev->lora_sf                      = SX126X_DEFAULT_LORA_SF;
  dev->lora_bw                      = SX126X_DEFAULT_LORA_BW;
  dev->lora_cr                      = SX126X_DEFAULT_LORA_CR;
  dev->lora_fixed_header            = SX126X_DEFAULT_LORA_FIXED_HEADER;
  dev->lora_crc                     = SX126X_DEFAULT_LORA_CRC_EN;
  dev->low_datarate_optimization    = SX126X_DEFAULT_LORA_LDO;

  uint8_t newsyncword[] = SX126X_DEFAULT_SYNCWORD;
  memcpy(dev->syncword, newsyncword, sizeof(dev->syncword));

  /* GFSK defaults */
}

static int sx126x_setup_radio(FAR struct sx126x_dev_s *dev)
{
  /* Clear IRQ status */

  sx126x_clear_irq_status(dev, 0xffff);

  /* Set regulator */

  sx126x_set_regulator_mode(dev, dev->lower->regulator_mode);

  /* Set packet type */

  sx126x_set_packet_type(dev, dev->packet_type);

  /* Set RF frequency */

  int illegal_freq = dev->lower->check_frequency(dev->frequency_hz);

  if (illegal_freq)
    {
      wlerr("Board does not support %dHz", dev->frequency_hz);
      return -1;
    }

  sx126x_set_rf_frequency(dev, dev->frequency_hz);

  /* Set PA settings from lower */

  uint8_t hp;
  uint8_t dc;
  enum sx126x_device_e model;
  dev->lower->get_pa_values(&model, &hp, &dc);
  sx126x_set_pa_config(dev, model, hp, dc);

  /* Set TX params */

  dev->lower->limit_tx_power(&dev->power); /* Limited by board */
  sx126x_set_tx_params(dev, dev->power, dev->lower->tx_ramp_time);

  /* Set base */

  sx126x_set_buffer_base_address(dev, 0, 0);

  /* Set params depending on packet type */

  switch (dev->packet_type)
    {
      case (SX126X_PACKETTYPE_LORA):
        {
          /* Mod params */

          struct sx126x_modparams_lora_s modparams = {
            .spreading_factor           = dev->lora_sf,
            .bandwidth                  = dev->lora_bw,
            .coding_rate                = dev->lora_cr,
            .low_datarate_optimization  = dev->low_datarate_optimization
          };

          sx126x_set_modulation_params_lora(dev, &modparams);

          /* Packet params */

          struct sx126x_packetparams_lora_s pktparams = {
            .crc_enable          = dev->lora_crc,
            .fixed_length_header = dev->lora_fixed_header,
            .payload_length      = dev->payload_len,
            .invert_iq           = dev->invert_iq,
            .preambles           = dev->preambles
          };

          sx126x_set_packet_params_lora(dev, &pktparams);
          break;
        }

      default:
      break;
    }

  /* Sync word */

  sx126x_set_syncword(dev, dev->syncword, sizeof(dev->syncword));

  /* IRQ MASK */

  sx126x_set_dio_irq_params(dev, dev->irq_mask,
    dev->lower->masks.dio1_mask,
    dev->lower->masks.dio2_mask,
    dev->lower->masks.dio3_mask);

  /* DIO 2 */

  sx126x_set_dio2_as_rf_switch(dev, dev->lower->use_dio2_as_rf_sw);

  /* DIO 3 */

  sx126x_set_dio3_as_tcxo(dev, dev->lower->dio3_voltage,
                          dev->lower->dio3_delay);
  return 0;
}

/* Interrupt handling *******************************************************/

static int sx126x_irq0handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  DEBUGASSERT(work_available(&dev->irq0_work));

  return work_queue(HPWORK, &dev->irq0_work, sx126x_isr0_process, arg, 0);
}

static inline int sx126x_attachirq0(FAR struct sx126x_dev_s *dev, xcpt_t isr,
  FAR void *arg)
{
  DEBUGASSERT(dev->lower->irq0attach != NULL);

  return dev->lower->irq0attach(isr, arg);
}

static void sx126x_isr0_process(FAR void *arg)
{
  DEBUGASSERT(arg);

  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;

  wlinfo("SX126x ISR0 process triggered");

  /* Get and clear IRQ bits */

  sx126x_spi_lock(dev);
  sx126x_get_irq_status(dev, &dev->irqbits);
  sx126x_spi_unlock(dev);

  wlinfo("IRQ status 0x%X", dev->irqbits);

  /* On TX done */

  if (dev->irqbits & SX126X_IRQ_TXDONE_MASK)
    {
      wlinfo("TX done");

      /* Release writing threads */

      nxsem_post(&dev->tx_sem);
    }

  /* On RX done */

  if (dev->irqbits & SX126X_IRQ_RXDONE_MASK)
    {
      wlinfo("RX done");

      nxsem_post(&dev->rx_sem);
    }

  /* On CAD done */

  if (dev->irqbits & SX126X_IRQ_CADDONE_MASK)
    {
      wlinfo("CAD done");
    }

  /* On CAD detect */

  if (dev->irqbits & SX126X_IRQ_CADDETECTED_MASK)
    {
      wlinfo("CAD detect");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sx126x_register(FAR struct spi_dev_s *spi,
                     FAR const struct sx126x_lower_s *lower,
                     const char *path)
{
  /* Register the dev using an unique dev_number,
   * so multiple radios can be registered at once
   */

  if (lower->dev_number >= SX126X_MAX_DEVICES)
    {
      wlerr("SX126x dev_number %d is greater than \
            allowed amount of SX126x devices",
            lower->dev_number);
      return;
    }

  struct sx126x_dev_s *dev;
  dev = &g_sx126x_devices[lower->dev_number];
  dev->lower   = lower;
  dev->spi     = spi;

  /* Initialize locks and semaphores */

  nxmutex_init(&dev->lock);
  nxsem_init(&dev->rx_sem, 0, 0);
  nxsem_init(&dev->tx_sem, 0, 0);

  sx126x_attachirq0(dev, sx126x_irq0handler, dev);

  (void)register_driver(path, &sx126x_ops, 0666,
                        dev);
}
