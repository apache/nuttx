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

#define PAGE_SIZE 256
#define NO_DEBUG

/****************************************************************************
 * Private Functions
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

/* Charger rx interrupt functions */

static int stwlc38_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void stwlc38_worker(FAR void *arg);

struct stwlc38_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;   /* Battery charger device */

  /* Data fields specific to the lower half STWLC38 driver follow */

  uint8_t addr;                         /* I2C address */
  uint32_t frequency;                   /* I2C frequency */
  uint32_t pin;                         /* Interrupt pin */
  uint32_t current;                     /* rx current */
  FAR struct i2c_master_s *i2c;         /* I2C interface */
  FAR struct ioexpander_dev_s *ioedev;  /* Ioexpander device */
  struct work_s work;                   /* Interrupt handler worker */
};

static uint8_t type_of_command[CMD_STR_LEN] =
{
  0
};

static int number_parameters;

static int wlc_i2c_read(FAR struct stwlc38_dev_s *priv, uint8_t *cmd,
                        int cmd_length, uint8_t *read_data, int read_count)
{
  struct i2c_msg_s msg[2];
  int err;
  FAR struct i2c_master_s *dev = priv->i2c;

#ifdef DEBUG
  int i = 0;
#endif

  msg[0].addr = priv->addr;
  msg[0].buffer = cmd;
  msg[0].length = cmd_length;
  msg[0].flags = 0;

  msg[1].addr = priv->addr;
  msg[1].buffer = read_data;
  msg[1].length = read_count;
  msg[1].flags = I2C_M_READ;

  if ((err = I2C_TRANSFER(dev, msg, 2)) < OK)
    {
      baterr("[WLC] i2c transfer failed! err: %d\n", err);
      return err;
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

  msg[0].addr = priv->addr;
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
                        uint8_t *data, uint32_t data_length)
{
  uint8_t *cmd;
  cmd = kmm_zalloc((5 + data_length) * sizeof(uint8_t));

  cmd[0] = OPCODE_WRITE;
  cmd[1] = (uint8_t)((addr >> 24) & 0xff);
  cmd[2] = (uint8_t)((addr >> 16) & 0xff);
  cmd[3] = (uint8_t)((addr >> 8) & 0xff);
  cmd[4] = (uint8_t)((addr >> 0) & 0xff);
  memcpy(&cmd[5], data, data_length);

  if ((wlc_i2c_write(priv, cmd, (5 + data_length))) < OK)
    {
      baterr("[WLC] Error in writing Hardware I2c!\n");
      kmm_free(cmd);
      return E_BUS_W;
    }

  kmm_free(cmd);
  return OK;
}

static int fw_i2c_write(FAR struct stwlc38_dev_s *priv, uint16_t addr,
                        uint8_t *data, uint32_t data_length)
{
  uint8_t *cmd;
  cmd = kmm_zalloc((2 + data_length) * sizeof(uint8_t));

  cmd[0] = (uint8_t)((addr >>  8) & 0xff);
  cmd[1] = (uint8_t)((addr >>  0) & 0xff);
  memcpy(&cmd[2], data, data_length);
  if ((wlc_i2c_write(priv, cmd, (2 + data_length))) < OK)
    {
      baterr("[WLC] ERROR: in writing Hardware I2c!\n");
      kmm_free(cmd);
      return E_BUS_W;
    }

  kmm_free(cmd);
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

char *print_hex(char *label, uint8_t *buff, int count, uint8_t *result)
{
  int i;
  int offset;
  offset = strlen(label) + 1;
  strlcpy(result, label, offset); /* +1 for terminator char */
  for (i = 0; i < count; i++)
    {
      snprintf(&result[offset], 4, "%02X ", buff[i]);

      /* this append automatically a null terminator char */

      offset += 3;
    }

  return result;
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

  reg_value = 0x40;
  if ((err = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1)) != OK)
      return err;
  usleep(AFTER_SYS_RESET_SLEEP_MS);
  reg_value = 0xc5;
  if ((err = fw_i2c_write(priv, FWREG_NVM_PWD_ADDR, &reg_value, 1)) != OK)
      return err;
  batinfo("[WLC] RRAM Programming.. \n");

  err = polaris_nvm_write_bulk(priv, patch_data, NVM_PATCH_SIZE,
                               NVM_PATCH_START_SECTOR_INDEX);
  if (err != OK) return err;
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

  usleep(AFTER_SYS_RESET_SLEEP_MS);

  return OK;
}

static ssize_t chip_info_show(FAR struct stwlc38_dev_s *priv, char *buf)
{
  int error;
  char temp[100];
  uint8_t read_buff[4];
  uint8_t cmd[2];
  FAR struct i2c_master_s *dev = priv->i2c;

  memset(read_buff, 0, 4);
  memset(cmd, 0, 2);

  cmd[0] = (WLC_CHIPID_LOW_REG & 0xff00) >> 8;
  cmd[1] = (WLC_CHIPID_LOW_REG & 0xff);
  batinfo("[WLC] Chip Id Command: %02X %02X\n", cmd[0], cmd[1]);

  if (wlc_i2c_read(dev, cmd, 2, read_buff, 4) < OK)
    {
      batinfo("[WLC] ERROR: could not read the register\n");
      error = snprintf(buf, PAGE_SIZE,
                       "CHIP INFO READ ERROR {%02X}\n", E_BUS_WR);
      return error;
    }

  batinfo("[WLC] Chip Id : 0x%04X\n", (read_buff[0] | (read_buff[1] << 8)));
  batinfo("[WLC] Chip Revision : 0x%02X\n", read_buff[2]);
  error = snprintf(buf, PAGE_SIZE, "%s\n", print_hex("Chip Info: ",
                   read_buff, 3, temp));

  return error;
}

static ssize_t nvm_program_show(FAR struct stwlc38_dev_s *priv, char *buf)
{
  int err = 0;
  FAR struct i2c_master_s *dev = priv->i2c;
  int count = 0;
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

  chip_info.cut_id = chip_info.chip_revision; /* TEST: */

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
  count = snprintf(buf, PAGE_SIZE, "{ %08X }\n", err);

  return count;
}

static ssize_t st_polaris_i2c_bridge_show(FAR struct stwlc38_dev_s *priv,
                                          char *buf)
{
  int err = 0;
  FAR struct i2c_master_s *dev = priv->i2c;
  uint8_t *read_buff = NULL;
  uint8_t *all_strbuff = NULL;
  int read_count = 0;
  int i = 0;
  int index = 0;
  int size = (6 * 2) + 1;
  if (number_parameters != 0)
    {
      switch (type_of_command[0])
        {
          case WRITE_READ_OPERATION:
            {
              if (number_parameters >= MIN_WR_BYTE_LENGTH)
                {
                  read_count = ((type_of_command[number_parameters - 2] << 8)
                               | (type_of_command[number_parameters - 1]));
                  batinfo("[WLC] read Count is %d\n", read_count);
                  read_buff = kmm_zalloc(read_count * sizeof(uint8_t));
                  if (read_buff == NULL)
                    {
                      err = E_MEMORY_ALLOC;
                      goto cleanup;
                    }

                  if (wlc_i2c_read(priv, type_of_command + 1,
                                   (number_parameters - 3),
                                   read_buff, read_count) < OK)
                    {
                      err = E_BUS_WR;
                    }
                  else
                    {
                      size += (read_count * sizeof(uint8_t)) * 2;
                      all_strbuff = kmm_zalloc(size);
                      if (all_strbuff == NULL)
                        {
                          err = E_MEMORY_ALLOC;
                          goto cleanup;
                        }

                      err = OK;
                      snprintf(&all_strbuff[index], 11, "{ %08X", err);
                      index += 10;
                      for (i = 0; i < read_count; i++)
                        {
                          snprintf(&all_strbuff[index], 3, "%02X",
                                   read_buff[i]);
                          index += 2;
                        }
                      snprintf(&all_strbuff[index], 3, " }");
                      index += 2;
                      err = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);
                      number_parameters = 0;
                      kmm_free(read_buff);
                      kmm_free(all_strbuff);
                      return err;
                    }
                }
              else
                {
                  batinfo("[WLC] Invalid input for Write read operation\n");
                  err = E_INVALID_INPUT;
                }

              goto cleanup;
            }

          case WRITE_OPERATION:
            {
              if (number_parameters >= MIN_W_BYTE_LENGTH)
                {
                  if (wlc_i2c_write(priv, type_of_command + 1,
                                   (number_parameters - 1)) < OK)
                    {
                      err = E_BUS_W;
                    }
                  else
                    {
                      err = OK;
                    }
                }
              else
                {
                  batinfo("[WLC] Invalid input for Write operation\n");
                  err = E_INVALID_INPUT;
                }

              goto cleanup;
            }

          default:
            {
              batinfo("[WLC] Invalid input for i2c bridge\n");
              err = E_INVALID_INPUT;
              goto cleanup;
            }
        }
    }
  else
    {
       err = E_INVALID_INPUT;
       batinfo("[WLC] Invalid input for write/write_read operation\n");
       goto cleanup;
    }

cleanup:
  number_parameters = 0;
  if (read_buff != NULL) kmm_free(read_buff);
  err = snprintf(buf, PAGE_SIZE, "{ %08X }\n", err);
  return err;
}

static ssize_t st_polaris_i2c_bridge_store(FAR struct stwlc38_dev_s *priv,
                                           const char *buf, size_t count)
{
  int n;
  int temp;
  char *p = (char *)buf;
  number_parameters = 0;

  for (n = 0; n < (count + 1) / 3; n++)
    {
      if (sscanf(p, "%02X ", &temp) == 1)
        {
          p += 3;
          type_of_command[n] = (uint8_t)temp;
          batinfo("[WLC] type_of_command[%d] = %02X\n",
                  n, type_of_command[n]);
          number_parameters++;
        }
    }

  batinfo("[WLC] Number of Parameters = %d\n", number_parameters);
  return count;
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
  IOEXP_SETOPTION(priv->ioedev, priv->pin,
                      IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return OK;
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

static int stwlc38_readrx(priv)
{
  /* to-do */

  return OK;
}

static void stwlc38_worker(FAR void *arg)
{
  FAR struct stwlc38_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  IOEXP_SETOPTION(priv->ioedev, priv->pin,
                      IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_FALLING);

  /* Read out the latest rx data */

  if (stwlc38_readrx(priv) == 0)
    {
      /* push data to upper half driver */

      return OK;
    }
}

static const struct battery_charger_operations_s g_stwlc38ops =
{
  stwlc38_state,
  stwlc38_health,
  stwlc38_online,
  stwlc38_voltage,
  stwlc38_current,
  stwlc38_input_current,
  stwlc38_operate,
};

static int stwlc38_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int err;
  uint8_t reg_value = 0;

  /* return device operate mode */

  err = fw_i2c_read(priv, FWREG_OP_MODE_ADDR, &reg_value, 1);
  if (err != OK)
    {
      *status = -1;
      return err;
    }

  *status = (int)reg_value;

  return OK;
}

static int stwlc38_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int err;
  uint16_t reg_value = 0;
  uint16_t temp;

  err = fw_i2c_read(priv, WLC_RX_CHIP_TEMP_REG, &reg_value, 2);
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
  *status = true;
  return OK;
}

static int stwlc38_voltage(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int err;
  uint16_t reg_value;

  reg_value = (uint16_t)(value / 100 - 5);

  err = fw_i2c_write(priv, WLC_RX_VOUT_SET_REG, &reg_value, 2);
  if (err != OK) return err;

  return OK;
}

static int stwlc38_current(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  int err;
  uint8_t reg_value;

  reg_value = (uint8_t)(value / 100);

  err = fw_i2c_write(priv, WLC_RX_ILIM_SET_REG, &reg_value, 1);
  if (err != OK) return err;

  return OK;
}

static int stwlc38_input_current(FAR struct battery_charger_dev_s *dev,
                                  FAR int value)
{
  return OK;
}

static int stwlc38_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param)
{
  FAR struct stwlc38_dev_s *priv = (FAR struct stwlc38_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *) param;
  int op;
  int value;
  int ret = OK;
  uint8_t reg_value = 0;

  op = msg->operate_type;
  value = (int)msg->u32;
  switch (op)
    {
      case BATIO_OPRTN_RESET:

        /* FW system reset */

        reg_value = 0x40;
        ret = fw_i2c_write(priv, FWREG_SYS_CMD_ADDR, &reg_value, 1);

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

      default:
        batinfo("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

FAR struct battery_charger_dev_s *
  stwlc38_initialize(FAR struct i2c_master_s *i2c,
                     uint32_t pin,
                     uint8_t addr,
                     uint32_t frequency,
                     uint32_t current,
                     FAR struct ioexpander_dev_s *dev)
{
  FAR struct stwlc38_dev_s *priv;
  char *buf;
  uint8_t count;
  int ret;

  /* Initialize the STWLC38 device structure */

  priv = kmm_zalloc(sizeof(struct stwlc38_dev_s));
  if (priv)
    {
      /* Initialize the STWLC38 device structure */

      priv->dev.ops   = &g_stwlc38ops;
      priv->i2c       = i2c;
      priv->pin       = pin;
      priv->addr      = addr;
      priv->current   = current;
      priv->frequency = frequency;
      priv->ioedev    = dev;
    }

  /* Interrupt register */

  ret = IOEXP_SETDIRECTION(priv->ioedev, priv->pin,
                           IOEXPANDER_DIRECTION_IN_PULLUP);
  if (ret < 0)
    {
      baterr("Failed to set direction: %d\n", ret);
    }

  ret = IOEP_ATTACH(priv->ioedev, priv->pin,
                          stwlc38_interrupt_handler, priv);
  if (ret == NULL)
    {
      baterr("Failed to attach: %d\n", ret);
      ret = -EIO;
    }

  ret = IOEXP_SETOPTION(priv->ioedev, priv->pin,
                        IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->ioedev, stwlc38_interrupt_handler);
    }

  count = nvm_program_show(priv, buf);

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* __POLARIS_C */
