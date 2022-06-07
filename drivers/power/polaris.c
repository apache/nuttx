/****************************************************************************
 * drivers/power/polaris.c
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
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef __POLARIS_C
#define __POLARIS_C

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/wqueue.h>

#include "polaris.h"
#include "polaris_nvm.c"
#include "polaris_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PAGE_SIZE 256
#define ON        (bool)0
#define OFF       (bool)1
#define NO_DEBUG

/****************************************************************************
 * Private
 ****************************************************************************/

struct stwlc38_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;   /* Battery charger device */

  /* Data fields specific to the lower half STWLC38 driver follow */

  FAR struct stwlc38_lower_s *lower;
  FAR struct i2c_master_s *i2c;             /* I2C interface */
  FAR struct ioexpander_dev_s *rpmsg_dev;   /* Ioexpander device */
  FAR struct ioexpander_dev_s *io_dev;      /* Ioexpander device */
  struct work_s work;                       /* Interrupt handler worker */
  struct work_s detect_work;                /* charger detect work */
  bool charging;                            /* Mark charge_manager is not running */
  int batt_state_flag;
  int detect_work_exit;
  bool tx_is_pp;                            /* XM tx or not */
  bool rx_vout_off_flag;
  bool rx_vout_update_flag;
  int rx_vout_off_count;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stwlc38_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status);
static int stwlc38_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health);
static int stwlc38_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status);
static int stwlc38_voltage(FAR struct battery_charger_dev_s *dev,
                            int value);
static int stwlc38_current(FAR struct battery_charger_dev_s *dev,
                            int value);
static int stwlc38_input_current(FAR struct battery_charger_dev_s *dev,
                                  int value);
static int stwlc38_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param);
static int stwlc38_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value);
static int stwlc38_get_det_state(FAR struct stwlc38_dev_s *priv,
                          FAR int *status);
static int stwlc38_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value);
static int stwlc38_voltage_info(FAR struct battery_charger_dev_s *dev,
                                int *value);
static int stwlc38_get_protocol(FAR struct battery_charger_dev_s *dev,
                                int *value);
static int stwlc38_get_standard_type(FAR struct battery_charger_dev_s *dev,
                                      bool *status);

/* Charger rx interrupt functions */

static int stwlc38_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                ioe_pinset_t pinset, FAR void *arg);
static int stwlc38_det_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void stwlc38_det_worker(FAR void *arg);
static void stwlc38_worker(FAR void *arg);
static void detect_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_stwlc38ops =
{
  stwlc38_state,
  stwlc38_health,
  stwlc38_online,
  stwlc38_voltage,
  stwlc38_current,
  stwlc38_input_current,
  stwlc38_operate,
  stwlc38_chipid,
  stwlc38_get_voltage,
  stwlc38_voltage_info,
  stwlc38_get_protocol,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wlc_i2c_read(FAR struct stwlc38_dev_s *priv, uint8_t *cmd,
                        int cmd_length, uint8_t *read_data, int read_count)
{
  struct i2c_msg_s msg[2];
  int err;
  int retries = 0;
  FAR struct i2c_master_s *dev = priv->i2c;

#ifdef DEBUG
  int i = 0;
#endif

  msg[0].addr = priv->lower->addr;
  msg[0].buffer = cmd;
  msg[0].length = cmd_length;
  msg[0].flags = I2C_M_NOSTOP;

  msg[1].addr = priv->lower->addr;
  msg[1].buffer = read_data;
  msg[1].length = read_count;
  msg[1].flags = I2C_M_READ;

  for (retries = 0; retries < ST_IIC_RETRY_NUM; retries++)
    {
      err = I2C_TRANSFER(dev, msg, 2);
      if (err >= 0)
        {
          break;
        }
      else
        {
          nxsig_usleep(1);
          baterr("ERROR: i2c transfer failed! err: %d retries:%d\n",
          err, retries);
        }
    }

#ifdef DEBUG
  batinfo("[WLC] WR-W: ");
  for (i = 0; i < cmd_length; i++)
    printk(KERN_CONT "%02X ", cmd[i]);

  batinfo("[WLC] WR-R: ");
  for (i = 0; i < read_count; i++)
    printk(KERN_CONT "%02X ", read_data[i]);
#endif

  return OK;
}

static int wlc_i2c_write(FAR struct stwlc38_dev_s *priv,
                         uint8_t *cmd, int cmd_length)
{
  struct i2c_msg_s msg[1];
  int err;
  FAR struct i2c_master_s *dev = priv->i2c;

#ifdef DEBUG
  int i = 0;
#endif

  msg[0].addr = priv->lower->addr;
  msg[0].buffer = cmd;
  msg[0].length = cmd_length;
  msg[0].flags = 0;

#ifdef DEBUG
  batinfo("[WLC] W: ");
  for (i = 0; i < cmd_length; i++)
    printk(KERN_CONT "%02X ", cmd[i]);
#endif

  if ((err = I2C_TRANSFER(dev, msg, 1)) < OK)
    {
      baterr("[WLC] i2c transfer failed! err: %d\n", err);
      return err;
    }

  return OK;
}

static int hw_i2c_write(FAR struct stwlc38_dev_s *priv, uint32_t addr,
                        FAR uint8_t *data, uint32_t data_length)
{
  uint8_t cmd[5 + data_length];

  cmd[0] = OPCODE_WRITE;
  cmd[1] = (uint8_t)((addr >> 24) & 0xff);
  cmd[2] = (uint8_t)((addr >> 16) & 0xff);
  cmd[3] = (uint8_t)((addr >> 8) & 0xff);
  cmd[4] = (uint8_t)((addr >> 0) & 0xff);
  memcpy(&cmd[5], data, data_length);

  if ((wlc_i2c_write(priv, cmd, sizeof(cmd))) < OK)
    {
      baterr("[WLC] Error in writing Hardware I2c!\n");
      return E_BUS_W;
    }

  return OK;
}

static int fw_i2c_write(FAR struct stwlc38_dev_s *priv, uint16_t addr,
                        FAR uint8_t *data, uint32_t data_length)
{
  uint8_t cmd[2 + data_length];

  cmd[0] = (uint8_t)((addr >>  8) & 0xff);
  cmd[1] = (uint8_t)((addr >>  0) & 0xff);
  memcpy(&cmd[2], data, data_length);
  if ((wlc_i2c_write(priv, cmd, sizeof(cmd))) < OK)
    {
      baterr("[WLC] ERROR: in writing Hardware I2c!\n");
      return E_BUS_W;
    }

  return OK;
}

static int hw_i2c_read(FAR struct stwlc38_dev_s *priv, uint32_t addr,
                       uint8_t *read_buff, int read_count)
{
  uint8_t cmd[5];
  cmd[0] = OPCODE_WRITE;
  cmd[1] = (uint8_t)((addr >> 24) & 0xff);
  cmd[2] = (uint8_t)((addr >> 16) & 0xff);
  cmd[3] = (uint8_t)((addr >>  8) & 0xff);
  cmd[4] = (uint8_t)((addr >>  0) & 0xff);
  if ((wlc_i2c_read(priv, cmd, 5 , read_buff, read_count)) < OK)
    {
      baterr("[WLC] Error in writing Hardware I2c!\n");
      return E_BUS_WR;
    }

  return OK;
}

static int fw_i2c_read(FAR struct stwlc38_dev_s *priv, uint16_t addr,
                       uint8_t *read_buff, int read_count)
{
  uint8_t cmd[2];
  cmd[0] = (uint8_t)((addr >>  8) & 0xff);
  cmd[1] = (uint8_t)((addr >>  0) & 0xff);
  if ((wlc_i2c_read(priv, cmd, 2, read_buff, read_count)) < OK)
    {
      baterr("[WLC] Error in writing Hardware I2c!\n");
      return E_BUS_WR;
    }

  return OK;
}

static int get_polaris_chip_info(FAR struct stwlc38_dev_s *priv,
                                 struct polaris_chip_info *info)
{
  uint8_t read_buff[14];

  memset(read_buff, 0, 14);
  if (fw_i2c_read(priv, WLC_CHIPID_LOW_REG, read_buff, 14) < OK)
    {
      baterr("[WLC] Error while getting polaris_chip_info\n");
      return E_BUS_R;
    }

  info->chip_id = (uint16_t)(read_buff[0] + (read_buff[1] << 8));
  info->chip_revision = read_buff[2];
  info->customer_id = read_buff[3];
  info->project_id = (uint16_t)(read_buff[4] + (read_buff[5] << 8));
  info->nvm_patch_id = (uint16_t)(read_buff[6] + (read_buff[7] << 8));
  info->ram_patch_id = (uint16_t)(read_buff[8] + (read_buff[9] << 8));
  info->config_id = (uint16_t)(read_buff[10] + (read_buff[11] << 8));
  info->pe_id = (uint16_t)(read_buff[12] + (read_buff[13] << 8));

  if (hw_i2c_read(priv, HWREG_HW_VER_ADDR, read_buff, 1) < OK)
    {
      baterr("[WLC] Error while getting polaris_chip_info\n");
      return E_BUS_R;
    }

  info->cut_id = read_buff[0];
  batinfo("[WLC] ChipID: %04X Chip Revision: %02X CustomerID: %02X \
          RomID: %04X NVMPatchID: %04X RAMPatchID: %04X CFG: %04X \
          PE: %04X\n", info->chip_id, info->chip_revision, \
          info->customer_id, info->project_id, info->nvm_patch_id, \
          info->ram_patch_id, info->config_id, info->pe_id);
  return OK;
}

static int polaris_nvm_write_sector(FAR struct stwlc38_dev_s *priv,
                                    const uint8_t *data,
                                    int data_length,
                                    int sector_index)
{
  int err = 0;
  int i = 0;
  int timeout = 1;
  uint8_t reg_value = (uint8_t)sector_index;
  uint8_t write_buff[NVM_SECTOR_SIZE_BYTES];

  batinfo("[WLC] writing sector %02X\n", sector_index);
  if (data_length > NVM_SECTOR_SIZE_BYTES)
    {
      batinfo("[WLC] sector data bigger than 256 bytes\n");
      return E_INVALID_INPUT;
    }

  memset(write_buff, 0, NVM_SECTOR_SIZE_BYTES);
  memcpy(write_buff, data, data_length);

  err = fw_i2c_write(priv, FWREG_NVM_SECTOR_INDEX_ADDR, &reg_value, 1);
  if (err != OK) return err;

  reg_value = 0x10;
  err = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);
  if (err != OK) return err;

  err = fw_i2c_write(priv, FWREG_AUX_DATA_00, write_buff, data_length);
  if (err != OK) return err;

  reg_value = 0x04;
  err = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);
  if (err != OK) return err;

  for (i = 0; i < 20; i++)
    {
      usleep(1);
      err = fw_i2c_read(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);
      if (err != OK) return err;
      if ((reg_value & 0x04) == 0)
        {
          timeout = 0;
          break;
        }
    }

  reg_value = 0x20;
  if (fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1) != OK)
  baterr("[WLC] Error power down the NVM\n");

  return timeout == 0 ? OK : E_TIMEOUT;
}

static int polaris_nvm_write_bulk(FAR struct stwlc38_dev_s *priv,
                                  const uint8_t *data,
                                  int data_length,
                                  uint8_t sector_index)
{
  int err = 0;
  int remaining = data_length;
  int to_write_now = 0;
  int written_already = 0;
  while (remaining > 0)
    {
      if (remaining > NVM_SECTOR_SIZE_BYTES)
        to_write_now = NVM_SECTOR_SIZE_BYTES;
      else
        to_write_now = remaining;

      err = polaris_nvm_write_sector(priv, data + written_already,
                                     to_write_now, sector_index);
      if (err != OK) return err;
      remaining -= to_write_now;
      written_already += to_write_now;
      sector_index++;
    }

  return OK;
}

static int polaris_nvm_write(FAR struct stwlc38_dev_s *priv)
{
  int err = 0;
  uint8_t reg_value = 0;
  uint8_t count = 5;

  /* check if OP MODE = DC POWER */

  err = fw_i2c_read(priv, FWREG_OP_MODE_ADDR, &reg_value, 1);
  if (err != OK) return err;
  batinfo("[WLC] OP MODE %02X\n", reg_value);

  if (reg_value != FW_OP_MODE_SA)
    {
      batinfo("[WLC] no DC power detected, nvm programming aborted\n");
      return E_UNEXPECTED_OP_MODE;
    }

  /* FW system reset */

  while (count)
    {
      batinfo("[WLC] the left %d time to try i2c0 status.. \n", count);
      count--;
      reg_value = 0x40;
      err = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);
      if (err != OK)
        {
          if (count == 0)
            {
              return err;
            }
          else
            {
              usleep(AFTER_SYS_RESET_SLEEP_MS);
              continue;
            }
        }

      usleep(AFTER_SYS_RESET_SLEEP_MS * 1000);
      reg_value = 0xc5;
      err = fw_i2c_write(priv, FWREG_NVM_PWD_ADDR, &reg_value, 1);
      if (err != OK)
        {
          if (count == 0)
            {
              return err;
            }
          else
            {
              usleep(AFTER_SYS_RESET_SLEEP_MS);
              continue;
            }
        }
      else
        {
          batinfo("[WLC] i2c0 status is ok \n");
          break;
        }
    }

  batinfo("[WLC] RRAM Programming.. \n");

  batinfo("[WLC] RRAM Programming to write FW_data.. \n");
  err = polaris_nvm_write_bulk(priv, patch_data, NVM_PATCH_SIZE,
                               NVM_PATCH_START_SECTOR_INDEX);
  if (err != OK) return err;

  batinfo("[WLC] RRAM Programming to write cfg_data.. \n");
  err = polaris_nvm_write_bulk(priv, cfg_data, NVM_CFG_SIZE,
                               NVM_CFG_START_SECTOR_INDEX);
  if (err != OK) return err;

  /* system reset */

  reg_value = 0x01;

  /**************************************************************************
   * the follow includes a workaround
   * it should return error when meeting i2c tramsfer failed, but if so, the
   * procedure logic will meet a difficult.
   * therefore, print err info and return ok. the workaround is only to hint
   * the I2C transfer failed, which does not effect normal work behind.
   **************************************************************************/

  if ((err = hw_i2c_write(priv, HWREG_HW_SYS_RST_ADDR, &reg_value, 1)) != OK)
    batinfo("[WLC] return err");

  /* the reset need 500ms or so, for this workaround, setup 1000ms */

  usleep(AFTER_SYS_RESET_SLEEP_MS * 1000);

  return OK;
}

static ssize_t nvm_program_show(FAR struct stwlc38_dev_s *priv)
{
  int err = OK;
  int config_id_mismatch = 0;
  int patch_id_mismatch = 0;
  struct polaris_chip_info chip_info;
  batinfo("[WLC] NVM Programming started\n");

  if (get_polaris_chip_info(priv, &chip_info) < OK)
    {
      baterr("[WLC] Error in reading polaris_chip_info\n");
      err = E_BUS_R;
      goto exit_0;
    }

  /* determine what has to be programmed depending on version ids */

  batinfo("[WLC] Cut Id: %02X\n", chip_info.cut_id);
  if (chip_info.cut_id != NVM_TARGET_CUT_ID)
    {
      batinfo("[WLC] HW cut id mismatch with Target cut id, \
              NVM programming aborted\n");
      err = E_UNEXPECTED_HW_REV;
      goto exit_0;
    }

  if (chip_info.config_id != NVM_CFG_VERSION_ID)
    {
      batinfo("[WLC] Config ID mismatch - running|header: [%04X|%04X]\n", \
              chip_info.config_id, NVM_CFG_VERSION_ID);
      config_id_mismatch = 1;
    }

  if (chip_info.nvm_patch_id != NVM_PATCH_VERSION_ID)
    {
      batinfo("[WLC] Patch ID mismatch - running|header: [%04X|%04X]\n", \
              chip_info.nvm_patch_id, NVM_PATCH_VERSION_ID);
      patch_id_mismatch = 1;
    }

  if (config_id_mismatch == 0 && patch_id_mismatch == 0)
    {
      batinfo("[WLC] NVM programming is not required, \
              both cfg and patch are up to date\n");
      err = OK;
      goto exit_0;
    }

  /**************************************************************************
   * program both cfg and patch
   * in case one of the two needs to be programmed
   **************************************************************************/

  if ((err = polaris_nvm_write(priv)) != OK)
    {
      err = E_NVM_WRITE;
      baterr("[WLC] NVM programming failed\n");
      goto exit_0;
    }

  batinfo("[WLC] NVM programming completed, \
          now checking patch and cfg id\n");

  if (get_polaris_chip_info(priv, &chip_info) < OK)
    {
      baterr("[WLC] Error in reading lyra_chip_info\n");
      err = E_BUS_R;
      goto exit_0;
    }

  if ((chip_info.config_id == NVM_CFG_VERSION_ID)
      && (chip_info.nvm_patch_id == NVM_PATCH_VERSION_ID))
    {
      batinfo("[WLC] NVM patch and cfg id is OK\n");
      batinfo("[WLC] NVM Programming is successful\n");
    }
  else
    {
      err = E_NVM_DATA_MISMATCH;
      if (chip_info.config_id != NVM_CFG_VERSION_ID)
          batinfo("[WLC] Config Id mismatch after NVM programming\n");
      if (chip_info.nvm_patch_id != NVM_PATCH_VERSION_ID)
          batinfo("[WLC] Patch Id mismatch after NVM programming\n");
      baterr("[WLC] NVM Programming failed\n");
    }

exit_0:
  batinfo("[WLC] NVM programming exited\n");
  return err;
}

static int stwlc38_onoff_ldo_output(FAR struct stwlc38_dev_s *priv,
                                     bool onoff)
{
  int ret;

  /* Turn on vout ldo output when gpio2 input low */

  ret = IOEXP_SETDIRECTION(priv->rpmsg_dev, priv->lower->sleep_pin,
                   IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      baterr("Failed to set sleep_pin as output: %d\n", ret);
      return ret;
    }

  ret = IOEXP_WRITEPIN(priv->rpmsg_dev, priv->lower->sleep_pin, onoff);
  if (ret < 0)
    {
      baterr("Failed to write sleep_pin as %s, error: %d\n",
             onoff ? "OFF" : "ON" , ret);
      return ret;
    }

  return OK;
}

static int stwlc38_onoff_vaa(FAR struct stwlc38_dev_s *priv,
                                     bool onoff)
{
  int ret;

  /* Turn on vout ldo output when gpio2 input low */

  ret = IOEXP_SETDIRECTION(priv->rpmsg_dev, priv->lower->vaa_pin,
                   IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      baterr("Failed to set vaa_pin as output: %d\n", ret);
      return ret;
    }

  ret = IOEXP_WRITEPIN(priv->rpmsg_dev, priv->lower->vaa_pin, onoff ? 0 : 1);
  if (ret < 0)
    {
      baterr("Failed to set vaa_pin as %s, error: %d\n",
             onoff ? "Enable" : "Disable" , ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: stwlc38_interrupt_handler
 *
 * Description:
 *   Handle the rx interrupt.
 *
 * Input Parameters:
 *   dev     - ioexpander device.
 *   pinset  - Interrupt pin.
 *   arg     - Device struct.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stwlc38_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the stwlc38 new
   * data interrupt pin since it signals that new data has been measured.
   */

  FAR struct stwlc38_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Task the worker with retrieving the latest sensor data. We should not
   * do this in a interrupt since it might take too long. Also we cannot lock
   * the I2C bus from within an interrupt.
   */

  work_queue(LPWORK, &priv->work, stwlc38_worker, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: detect_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void detect_worker(FAR void *arg)
{
  FAR struct stwlc38_dev_s *priv = arg;
  int charger_is_exit;
  int ret;

  ret = stwlc38_state(&priv->dev, &charger_is_exit);
  if (ret == OK)
    {
      if (priv->batt_state_flag != charger_is_exit)
        {
          syslog(LOG_INFO, "rx wireless detect pin:%d\n", charger_is_exit);
          priv->batt_state_flag = charger_is_exit;
          battery_charger_changed(&priv->dev, BATTERY_STATE_CHANGED);
        }

      if (!charger_is_exit)
        {
          if (!priv->charging)
            {
              syslog(LOG_INFO, "charge_manager exit work cancel:%d\n",
                 charger_is_exit);
              priv->detect_work_exit = DETECT_WORK_NO_EXIST;
              work_cancel(LPWORK, &priv->detect_work);
            }
          else
            {
              work_queue(LPWORK, &priv->detect_work, detect_worker, priv,
                     RX_DETECT_WORK_TIME / USEC_PER_TICK);
            }
        }

      else if (priv->rx_vout_off_flag)
        {
          work_queue(LPWORK, &priv->detect_work, detect_worker, priv,
                     RX_DETECT_WORK_TIME / USEC_PER_TICK);
        }
      else
        {
          work_cancel(LPWORK, &priv->detect_work);
        }
    }
  else
    {
      baterr("[WLC] get stwlc38_state fail\n");
    }
}

/****************************************************************************
 * Name: stwlc38_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the I2C bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int stwlc38_check_intr(FAR struct stwlc38_dev_s *priv,
                          FAR struct rx_int_state_s *rx_int_state)
{
  int ret;
  uint32_t reg_value;

  ret = fw_i2c_read(priv, WLC_RX_INTR_LATCH_REG, (uint8_t *)&reg_value, 4);
  if (ret != OK)
    {
      baterr("[WLC] failed to read INTR states !!!\n");
      return ret;
    }

  batinfo("[WLC] read WLC_RX_INTR_LATCH_REG is %08"PRIx32"\n", reg_value);

  rx_int_state->wlc_rx_int_otp       =
      ((reg_value & WLC_RX_OTP_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_ocp       =
      ((reg_value & WLC_RX_OCP_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_ovp       =
      ((reg_value & WLC_RX_OVP_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_scp       =
      ((reg_value & WLC_RX_SCP_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_ss_tx     =
      ((reg_value & WLC_RX_SS_TX_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_output_on =
      ((reg_value & WLC_RX_OUTPUT_ON_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_int_uvp       =
      ((reg_value & WLC_RX_UVP_INT_MASK) == false) ? false : true;
  rx_int_state->wlc_rx_pp_done       =
      ((reg_value & WLC_RX_PP_DONE_INT_MASK) == false) ? false : true;

  if (rx_int_state->wlc_rx_pp_done == true)  priv->tx_is_pp = true;

  /* CLR int register */

  batinfo("[WLC] start to CLR INTR states !!!\n");
  reg_value = 0xffffffff;
  ret = fw_i2c_write(priv, WLC_RX_INTR_CLR_REG, (uint8_t *)&reg_value, 4);
  batinfo("[WLC] read WLC_RX_INTR_CLR_REG is %08"PRIx32" \n", reg_value);
  if (ret != OK)
    {
      baterr("[WLC] Failed to CLR INTR states !!!\n");
      return ret;
    }

  return ret;
}

static void stwlc38_worker(FAR void *arg)
{
  FAR struct stwlc38_dev_s *priv = arg;
  struct rx_int_state_s rx_int_state;

  DEBUGASSERT(priv != NULL);

  /* Read out the latest rx data */

  memset(&rx_int_state, 0, sizeof(struct rx_int_state_s));
  if (stwlc38_check_intr(priv, &rx_int_state) == 0)
    {
      /* push data to upper half driver */

      batinfo("SUCCESS: stwlc38_check_intr\n");
      batinfo("  rx_int_state.wlc_rx_int_otp       = %s\n",
               rx_int_state.wlc_rx_int_otp ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_ocp       = %s\n",
               rx_int_state.wlc_rx_int_ocp ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_ovp       = %s\n",
               rx_int_state.wlc_rx_int_ovp ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_scp       = %s\n",
               rx_int_state.wlc_rx_int_scp ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_ss_tx     = %s\n",
               rx_int_state.wlc_rx_int_ss_tx ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_output_on = %s\n",
               rx_int_state.wlc_rx_int_output_on ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_output_off = %s\n",
               rx_int_state.wlc_rx_int_output_off ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_int_uvp       = %s\n",
               rx_int_state.wlc_rx_int_uvp ? "true" : "false");
      batinfo("  rx_int_state.wlc_rx_pp_done       = %s\n",
               rx_int_state.wlc_rx_pp_done ? "true" : "false");
    }

  /**************************************************************************
   *  if recieved ss intr, start charge_manger app
   *  if removed tx, the charge_manager app will return
   **************************************************************************/

  if (rx_int_state.wlc_rx_int_output_on &&
              priv->detect_work_exit == DETECT_WORK_NO_EXIST)
    {
      syslog(LOG_INFO, "rx wireless charger inster\n");
      priv->charging = true; /* Mark charge manager will be running */
      priv->detect_work_exit = DETECT_WORK_EXIST;
      work_queue(LPWORK, &priv->detect_work, detect_worker, priv, 0);
    }

  return;
}

static void stwlc38_det_worker(FAR void *arg)
{
  FAR struct stwlc38_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);
  if (priv->batt_state_flag == BATT_CHARGING_STAT_ENTER)
    {
      if (priv->rx_vout_off_flag)
        {
          if (priv->rx_vout_update_flag)
            {
              work_queue(LPWORK, &priv->detect_work, detect_worker, priv, 0);
              priv->rx_vout_update_flag = false;
            }

          priv->rx_vout_off_count = 0;
        }
      else
        {
          work_queue(LPWORK, &priv->detect_work, detect_worker, priv,
                     RX_DETECT_WORK_TIME / USEC_PER_TICK);
        }
    }
}

static int stwlc38_get_det_state(FAR struct stwlc38_dev_s *priv,
                          FAR int *status)
{
  bool wpc_det = 0;
  int  ret;

  /* Check WPC_DET, output High when SS package sent */

  ret = IOEXP_SETDIRECTION(priv->rpmsg_dev, priv->lower->detect_pin,
                           IOEXPANDER_DIRECTION_IN_PULLDOWN);
  if (ret < 0)
    {
      baterr("Failed to set direction (wpc_det): %d\n", ret);
    }

  ret = IOEXP_READPIN(priv->rpmsg_dev, priv->lower->detect_pin, &wpc_det);
  if (ret < 0)
    {
      baterr("Failed to read pin (wpc_det): %d\n", ret);
    }

  *status = (int)wpc_det;

  if (!*status) priv->tx_is_pp = false;

  return OK;
}

static int stwlc38_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;

  if (priv->rx_vout_off_flag)
    {
      *status = true;
      priv->rx_vout_off_count++;
      if (priv->rx_vout_off_count == 3)
        {
          *status = false;
          priv->rx_vout_off_flag = false;
          priv->rx_vout_off_count = 0;
        }
    }
  else
    {
      stwlc38_get_det_state(priv, status);
    }

  return OK;
}

static int stwlc38_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int err;
  uint16_t reg_value = 0;
  uint16_t temp;

  err = fw_i2c_read(priv, WLC_RX_CHIP_TEMP_REG, (uint8_t *)&reg_value, 2);
  if (err != OK)
    {
      *health = -1;
      return err;
    }

  temp = reg_value / 10;

  if (temp < 0)
    *health = WLC_HEALTH_UNKNOWN;
  else if (temp > WLC_HEALTH_TEMP_MAX)
    *health = WLC_HEALTH_OVERHEAT;
  else if (temp < WLC_HEALTH_TEMP_MIN)
    *health = WLC_HEALTH_OVERCOLD;
  else
    *health = WLC_HEALTH_GOOD;

  return OK;
}

static int stwlc38_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status)
{
  int ret;

  ret = stwlc38_get_standard_type(dev, status);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

static int stwlc38_voltage(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int ret;
  uint16_t reg_value;

  reg_value = (uint16_t)(((value - WLC_RX_VOL_BASE) << 6) / 25);

  ret = fw_i2c_write(priv, WLC_RX_VOUT_SET_REG, (uint8_t *)&reg_value, 2);
  if (ret != OK)
    {
      baterr("[WLC] fw i2c wirte faild \n");
      return ret;
    }

  return ret;
}

static int stwlc38_current(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int ret;
  uint8_t reg_value;

  reg_value = (uint8_t)(value / 100);

  ret = fw_i2c_write(priv, WLC_RX_ILIM_SET_REG, &reg_value, 1);

  return ret;
}

static int stwlc38_input_current(FAR struct battery_charger_dev_s *dev,
                                  FAR int value)
{
  batinfo("Unsupported setup input current limit!");
  return OK;
}

static int stwlc38_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *) param;
  int op;
  int ret = OK;
  uint8_t reg_value = 0;

  op = msg->operate_type;
  switch (op)
    {
      case BATIO_OPRTN_RESET:

        /* FW system reset */

        reg_value = 0x40;
        ret = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);
        if (ret < 0)
          {
            baterr("Failed to FW system reset, Error: %d\n", ret);
            ret = -EINVAL;
          }

  /**************************************************************************
   * system reset:
   * the follow includes a workaround
   * it should return error when meeting i2c tramsfer failed, but if so, the
   * procedure logic will meet a difficult.
   * therefore, print err info and return ok. the workaround is only to hint
   * the I2C transfer failed, which does not effect normal work behind.
   **************************************************************************/

        reg_value = 0x01;
        ret = hw_i2c_write(priv, HWREG_HW_SYS_RST_ADDR, &reg_value, 1);
        usleep(AFTER_SYS_RESET_SLEEP_MS);
        break;

      case BATIO_OPRTN_SYSON:

        /* Turn on vout ldo output when gpio2 input low */

        ret = stwlc38_onoff_ldo_output(priv, ON);
        if (ret < 0)
          {
            baterr("Failed to trun ON wpc ldo output, Error: %d\n", ret);
            ret = -EINVAL;
          }
          break;

      case BATIO_OPRTN_SYSOFF:

        /* Turn off vout ldo output when gpio2 input high */

        ret = stwlc38_onoff_ldo_output(priv, OFF);
        if (ret < 0)
          {
            baterr("Failed to trun OFF wpc ldo output, Error: %d\n", ret);
            ret = -EINVAL;
          }
          break;

      case BATIO_OPRTN_CHARGE:

        /* if informed, mark charge_manager has been destroyed */

        priv->charging = false;
        break;

      case BATIO_OPRTN_EN_TERM:
        priv->rx_vout_off_flag = msg->u8[0];
        if (priv->rx_vout_off_flag)
          {
            priv->rx_vout_update_flag = true;
          }

        break;

      default:
        batinfo("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: stwlc38_chipid
 *
 * Description:
 *       Get chip id
 *
 ****************************************************************************/

static int stwlc38_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  struct polaris_chip_info chip_info;

  if (get_polaris_chip_info(priv, &chip_info) < OK)
    {
      baterr("[WLC] Error in reading polaris_chip_info\n");
      return E_BUS_R;
    }

  *value = chip_info.chip_id;

  batinfo("the chipid of stwlc38 is: %d\n", *value);
  return OK;
}

/****************************************************************************
 * Name: stwlc38_get_voltage
 *
 * Description:
 *   Get the actual output voltage from rx
 *
 ****************************************************************************/

static int stwlc38_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  uint16_t reg_value;

  if (fw_i2c_read(priv, WLC_RX_VOUT_SET_REG,
                 (FAR uint8_t *)&reg_value, 2) < OK)
    {
      baterr("[WLC] Error in reading WLC_RX_VOUT_SET_REG\n");
      return E_BUS_R;
    }

  *value = ((reg_value * 25) >> 6) + WLC_RX_VOL_BASE;

  batinfo("The the actual output voltage of stwlc38 is %d mv \n", *value);
  return OK;
}

static int stwlc38_voltage_info(FAR struct battery_charger_dev_s *dev,
                                int *value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int16_t reg_ce = 0xff;
  uint16_t reg_value = 0;
  if (fw_i2c_read(priv, WLC_LAST_CE,
                 (FAR uint8_t *)&reg_ce, 2) < OK)
    {
      baterr("[WLC] Error in reading WLC_LAST_CE\n");
      return E_BUS_R;
    }

  if (reg_ce < -1 || reg_ce > 1)
    {
      baterr("[WLC] rx out not ready\n");
      return E_BUS_R;
    }

  if (fw_i2c_read(priv, WLC_RX_VOUT_REG,
                 (FAR uint8_t *)&reg_value, 2) < OK)
    {
      baterr("[WLC] Error in reading WLC_RX_VOUT_REG\n");
      return E_BUS_R;
    }

  *value = reg_value;
  batinfo("The rx output voltage of stwlc38 is %d mv \n", *value);
  return OK;
}

static int stwlc38_get_protocol(FAR struct battery_charger_dev_s *dev,
                           int *value)
{
  int ret = 0;
  uint8_t buffer[3];
  int protocol_type = 0;

  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;

  ret = fw_i2c_read(priv, WLC_XM_PP_STATUS, buffer, 3);
  if (ret < 0)
    {
      baterr("Error: stwlc38 hw i2c read faild\n");
      return ret;
    }

  if (buffer[1] == WLC_SUPPORT_QC_3_0)
    {
      protocol_type |= BATTERY_PROTOCOL_QC3P0;
    }

  if (buffer[2] == WLC_GEN_TX)
    {
      protocol_type |= BATTERY_PROTOCOL_TX_XIAOMI;
    }

  *value = protocol_type;
  return ret;
}

static int stwlc38_init_interrupt(FAR struct stwlc38_dev_s *priv)
{
  int ret;
  void *ioepattach;

  ret = IOEXP_SETDIRECTION(priv->io_dev, priv->lower->int_pin,
                           IOEXPANDER_DIRECTION_IN_PULLUP);
  if (ret < 0)
    {
      baterr("Failed to set direction: %d\n", ret);
    }

  ioepattach = IOEP_ATTACH(priv->io_dev, priv->lower->int_pin,
                          stwlc38_interrupt_handler, priv);
  if (ioepattach == NULL)
    {
      baterr("Failed to attach stwlc38_interrupt_handler");
      ret = -EIO;
    }

  ret = IOEXP_SETOPTION(priv->io_dev, priv->lower->int_pin,
              IOEXPANDER_OPTION_INTCFG, (FAR void *)IOEXPANDER_VAL_FALLING);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->io_dev, stwlc38_interrupt_handler);
    }

  return ret;
}

static int stwlc38_get_standard_type(FAR struct battery_charger_dev_s *dev,
                                      bool *status)
{
  int ret = 0;
  uint8_t buffer[3];

  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;

  ret = fw_i2c_read(priv, WLC_XM_PP_STATUS, buffer, 3);
  if (ret < 0)
    {
      baterr("Error: stwlc38 hw i2c read faild\n");
      return ret;
    }

  if ((buffer[1] == WLC_SUPPORT_QC_3_0) && (buffer[2] == WLC_GEN_TX))
    {
      *status = true;
    }
  else
    {
      *status = false;
    }

  return ret;
}

static int stwlc38_det_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct stwlc38_dev_s *priv = arg;
  DEBUGASSERT(priv != NULL);

  work_queue(LPWORK, &priv->work, stwlc38_det_worker, priv, 0);

  return OK;
}

static int stwlc38_det_init_interrupt(FAR struct stwlc38_dev_s *priv)
{
  int ret;
  void *ioepattach;

  ret = IOEXP_SETDIRECTION(priv->rpmsg_dev, priv->lower->detect_pin,
                           IOEXPANDER_DIRECTION_IN);
  if (ret < 0)
    {
      baterr("Failed to set direction: %d\n", ret);
    }

  ioepattach = IOEP_ATTACH(priv->rpmsg_dev, priv->lower->detect_pin,
                          stwlc38_det_interrupt_handler, priv);
  if (ioepattach == NULL)
    {
      baterr("Failed to attach stwlc38_interrupt_handler");
      ret = -EIO;
    }

  ret = IOEXP_SETOPTION(priv->rpmsg_dev, priv->lower->detect_pin,
              IOEXPANDER_OPTION_INTCFG, (void *)IOEXPANDER_VAL_FALLING);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->rpmsg_dev, stwlc38_det_interrupt_handler);
    }

  return ret;
}

FAR struct battery_charger_dev_s *
  stwlc38_initialize(FAR struct i2c_master_s *i2c,
                     FAR struct stwlc38_lower_s *lower,
                     FAR struct ioexpander_dev_s *rpmsg_dev,
                     FAR struct ioexpander_dev_s *io_dev)
{
  FAR struct stwlc38_dev_s *priv;
  struct polaris_chip_info chip_info;
  int ret;

  /* Initialize the STWLC38 device structure */

  priv = kmm_zalloc(sizeof(struct stwlc38_dev_s));
  if (priv)
    {
      /* Initialize the STWLC38 device structure */

      priv->dev.ops   = &g_stwlc38ops;
      priv->i2c       = i2c;
      priv->lower     = lower;
      priv->rpmsg_dev = rpmsg_dev;
      priv->io_dev    = io_dev;
      priv->charging  = false;
      priv->batt_state_flag = BATT_CHARGING_STAT_INIT;
      priv->detect_work_exit = DETECT_WORK_EXIST;
      priv->tx_is_pp = false;
      priv->rx_vout_off_flag = false;
      priv->rx_vout_off_count = 0;
      priv->rx_vout_update_flag = true;
    }
  else
    {
      return NULL;
    }

  if (get_polaris_chip_info(priv, &chip_info) < OK)
    {
      baterr("[WLC] Error in reading polaris_chip_info\n");
      return NULL;
    }

  ret = nvm_program_show(priv);
  if (ret != OK)
    baterr("Failed to [WLC] NVM programming exited, Error: %d\n", ret);

  ret = stwlc38_init_interrupt(priv);
  if (ret < 0)
    {
      baterr("Failed to init_interrupt: %d\n", ret);
    }

  ret = stwlc38_det_init_interrupt(priv);
  if (ret < 0)
    {
      baterr("Failed to det init interrupt: %d\n", ret);
    }

  /* Turn on WPC_VAA_2V5 */

  /* It bring interrupt abnormal to turn on vaa, so turn off it */

  ret = stwlc38_onoff_vaa(priv, OFF);
  if (ret < 0)
    {
      baterr("Failed to set vaa_pin as Enable: %d\n", ret);
    }

  /* Turn on vout ldo output when gpio2 input low */

  ret = stwlc38_onoff_ldo_output(priv, ON);
  if (ret < 0)
    {
      baterr("Failed to trun ON wpc ldo output: %d\n", ret);
    }

  work_queue(LPWORK, &priv->detect_work, detect_worker, priv, 0);

  return (FAR struct battery_charger_dev_s *)priv;
}
#endif /* __POLARIS_C */
