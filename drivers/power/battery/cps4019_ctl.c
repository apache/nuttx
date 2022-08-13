/****************************************************************************
 * drivers/power/battery/cps4019_ctl.c
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
#ifndef __CPS4019_C
#define __CPS4019_C

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

#include "cps4019_nvm.c"
#include "cps4019_reg.h"

#define PAGE_SIZE 256
#define ON        (bool)0
#define OFF       (bool)1
#define NO_DEBUG

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cps4019_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status);
static int cps4019_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health);
static int cps4019_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status);
static int cps4019_voltage(FAR struct battery_charger_dev_s *dev,
                            int value);
static int cps4019_current(FAR struct battery_charger_dev_s *dev,
                            int value);
static int cps4019_input_current(FAR struct battery_charger_dev_s *dev,
                                  int value);
static int cps4019_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param);
static int cps4019_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value);
static int cps4019_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value);

/* Charger rx interrupt functions */

static int cps4019_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg);
static void cps4019_worker(FAR void *arg);
static void detect_worker(FAR void *arg);

struct cps4019_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_charger_dev_s dev;   /* Battery charger device */

  /* Data fields specific to the lower half cps4019 driver follow */

  FAR struct cps4019_lower_s *lower;
  FAR struct i2c_master_s *i2c;             /* I2C interface */
  FAR struct ioexpander_dev_s *rpmsg_dev;   /* Ioexpander device */
  FAR struct ioexpander_dev_s *io_dev;      /* Ioexpander device */
  struct work_s work;                       /* Interrupt handler worker */
  struct work_s detect_work;                /* charger detect work */
  bool charging;                            /* Mark charge_manager is not running */
  int batt_state_flag;
  int detect_work_exit;
};

static int cps_i2c_read(FAR struct cps4019_dev_s *priv, uint8_t *cmd,
                        int cmd_length, uint8_t *read_data, int read_count)
{
  struct i2c_msg_s msg[2];
  int err;
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

  if ((err = I2C_TRANSFER(dev, msg, 2)) < OK)
    {
      baterr("[CPS] i2c transfer failed! err: %d\n", err);
      return err;
    }

#ifdef DEBUG
  batinfo("[CPS] WR-W: ");
  for (i = 0; i < cmd_length; i++)
    printk(KERN_CONT "%02X ", cmd[i]);

  batinfo("[CPS] WR-R: ");
  for (i = 0; i < read_count; i++)
    printk(KERN_CONT "%02X ", read_data[i]);
#endif

  return OK;
}

static int cps_i2c_write(FAR struct cps4019_dev_s *priv,
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
  batinfo("[CPS] W: ");
  for (i = 0; i < cmd_length; i++)
    printk(KERN_CONT "%02X ", cmd[i]);
#endif

  if ((err = I2C_TRANSFER(dev, msg, 1)) < OK)
    {
      baterr("[CPS] i2c transfer failed! err: %d\n", err);
      return err;
    }

  return OK;
}

static int hw_i2c_write(FAR struct cps4019_dev_s *priv, uint32_t addr,
                        uint8_t *data, uint32_t data_length)
{
  uint8_t *cmd;
  cmd = kmm_zalloc((4 + data_length) * sizeof(uint8_t));

  cmd[0] = (uint8_t)((addr >> 24) & 0xff);
  cmd[1] = (uint8_t)((addr >> 16) & 0xff);
  cmd[2] = (uint8_t)((addr >> 8) & 0xff);
  cmd[3] = (uint8_t)((addr >> 0) & 0xff);
  memcpy(&cmd[4], data, data_length);

  if ((cps_i2c_write(priv, cmd, (4 + data_length))) < OK)
    {
      baterr("[CPS] Error in writing Hardware I2c!\n");
      kmm_free(cmd);
      return E_BUS_W;
    }

  kmm_free(cmd);
  return OK;
}

static int fw_i2c_write(FAR struct cps4019_dev_s *priv, uint16_t addr,
                        uint8_t *data, uint32_t data_length)
{
  uint8_t *cmd;
  cmd = kmm_zalloc((2 + data_length) * sizeof(uint8_t));

  cmd[0] = (uint8_t)((addr >>  8) & 0xff);
  cmd[1] = (uint8_t)((addr >>  0) & 0xff);
  memcpy(&cmd[2], data, data_length);
  if ((cps_i2c_write(priv, cmd, (2 + data_length))) < OK)
    {
      baterr("[CPS] ERROR: in writing Hardware I2c!\n");
      kmm_free(cmd);
      return E_BUS_W;
    }

  kmm_free(cmd);
  return OK;
}

static int hw_i2c_read(FAR struct cps4019_dev_s *priv, uint32_t addr,
                       uint8_t *read_buff, int read_count)
{
  uint8_t cmd[4];

  cmd[0] = (uint8_t)((addr >> 24) & 0xff);
  cmd[1] = (uint8_t)((addr >> 16) & 0xff);
  cmd[2] = (uint8_t)((addr >>  8) & 0xff);
  cmd[3] = (uint8_t)((addr >>  0) & 0xff);
  if ((cps_i2c_read(priv, cmd, 4 , read_buff, read_count)) < OK)
    {
      baterr("[CPS] Error in writing Hardware I2c!\n");
      return E_BUS_WR;
    }

  return OK;
}

static int fw_i2c_read(FAR struct cps4019_dev_s *priv, uint16_t addr,
                       uint8_t *read_buff, int read_count)
{
  uint8_t cmd[2];
  cmd[0] = (uint8_t)((addr >>  8) & 0xff);
  cmd[1] = (uint8_t)((addr >>  0) & 0xff);
  if ((cps_i2c_read(priv, cmd, 2, read_buff, read_count)) < OK)
    {
      baterr("[CPS] Error in read Hardware I2c!\n");
      return E_BUS_WR;
    }

  return OK;
}

static int get_cps4019_chip_info(FAR struct cps4019_dev_s *priv,
                                 struct cps4019_chip_info *info)
{
  uint8_t read_buff[6];

  memset(read_buff, 0, 6);
  if (fw_i2c_read(priv, CPS_CHIP_ID_L_REG, read_buff, 6) < OK)
    {
      baterr("[CPS] Error while getting cps4019_chip_info\n");
      return E_BUS_R;
    }

  info->chip_id = (uint16_t)(read_buff[0] + (read_buff[1] << 8));
  info->fw_revision_l = (uint16_t)(read_buff[2] + (read_buff[3] << 8));
  info->fw_revision_h = (uint16_t)(read_buff[4] + (read_buff[5] << 8));

  batinfo("[CPS] ChipID: %04X Chip Revision: %02X CustomerID: %02X\n",
          info->chip_id, info->fw_revision_l, info->fw_revision_h);

  return OK;
}

uint32_t big_little_endian_convert(uint32_t dat)
{
  char  *p;
  char  tmp[4];

  p = (char *)(&dat);
  tmp[0] = p[3];
  tmp[1] = p[2];
  tmp[2] = p[1];
  tmp[3] = p[0];

  return *(int *)(tmp);
}

static int cps4019_wait_cmd_done(FAR struct cps4019_dev_s *priv)
{
  int err = 0;
  int wait_time_out = 1000; /* ms */
  uint32_t read_buff;

  while (1)
    {
      err = hw_i2c_read(priv, HWREG_HW_BL_CHEK_RESULT_ADDR,
                        (uint8_t *)&read_buff, 4);
      if (err != OK)  return err;
      wait_time_out--;
      usleep(1000);

      read_buff = big_little_endian_convert(read_buff);
      if ((read_buff & 0xff) == 0x55)
        {
          break;
        }

      if ((read_buff & 0xff) == 0xaa)
        {
          batinfo("result check 0xAA\n");
          return -1;
        }

      if (wait_time_out < 0)
        {
          batinfo("bl check timeout\n");
          return -1;
        }
    }

  return err;
}

static int cps4019_mvn_load_bootloader(FAR struct cps4019_dev_s *priv,
                                       uint8_t *data,
                                       int data_length)
{
  int err = 0;
  batinfo("start load bootloader\n");

  batinfo("[CPS] start bootloader Programming.. \n");

  uint32_t reg_value = 0x0e000000;
  reg_value = big_little_endian_convert(0x0e000000);
  err = hw_i2c_write(priv, HWREG_HW_ENABLE_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_ENABLE_ADDR err\n");

  reg_value = big_little_endian_convert(0x0000a061);
  err = hw_i2c_write(priv, HWREG_HW_PASSWORD_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_PASSWORD_ADDR err\n");

  reg_value = big_little_endian_convert(0x00000008);
  err = hw_i2c_write(priv, HWREG_HW_HALT_MCU_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_HALT_MCU_ADDR err\n");

  err = hw_i2c_write(priv, HWREG_HW_SRAM_ADDR, data, data_length);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_SRAM_ADDR err\n");

  reg_value = big_little_endian_convert(0x00000001);
  err = hw_i2c_write(priv, HWREG_HW_DISABLE_TRIM_ADDR,
                     (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_DISABLE_TRIM_ADDR err\n");

  reg_value = big_little_endian_convert(0x00000066);
  err = hw_i2c_write(priv, HWREG_HW_EN_REGMAP_ADDR,
                     (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_EN_REGMAP_ADDR err\n");

  if (err != OK) return err;
  batinfo("[CPS] systerm restart\n");

  /* check bootloader */

  usleep(10 * 1000);
  reg_value = big_little_endian_convert(0x0e000000);
  err = hw_i2c_write(priv, HWREG_HW_ENABLE_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return HWREG_HW_ENABLE_ADDR err\n");

  reg_value = big_little_endian_convert(0x000000b0);
  err = hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    {
      batinfo("[CPS] return HWREG_HW_CTL_ADDR err\n");
      return err;
    }

  if ((err = cps4019_wait_cmd_done(priv)) != OK)
    {
      batinfo("[CPS] return bootlloader check fail err\n");
      return err;
    }

  batinfo("[CPS] load bootlloader success\n");

  return OK;
}

static int cps4019_mvn_load_fireware(FAR struct cps4019_dev_s *priv,
                                     const uint8_t *data,
                                     int data_length)
{
  batinfo("[CPS] start fireware Programming.. \n");
  int err = 0;
  int buff0_flag = 0;
  int buff1_flag = 0;
  int write_cout = 0;
  int k = 0;
  uint32_t reg_value = 0;
  int *p_convert = NULL;
  uint8_t *p;

  uint32_t program_size;
  program_size = big_little_endian_convert(HWREG_HW_PROGRAM_BUFF_SIZE);
  write_cout = 0;
  err = hw_i2c_write(priv, HWREG_HW_FW_SIZE_ADDR,
                     (uint8_t *)&program_size, 4);
  if (err != OK)
    batinfo("[CPS] return err");

  p_convert = (int *)data;
  for (k = 0; k < 16 * 1024 / 4; k++)
    {
      p_convert[k] = big_little_endian_convert(p_convert[k]);
    }

start_write_fw_code:
  p = (uint8_t *)p_convert;
  reg_value = big_little_endian_convert(0x00000060);
  err = hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
  if (err != OK)
    batinfo("[CPS] return err");

  usleep(500 * 1000);
  if ((err = cps4019_wait_cmd_done(priv)) != OK)
    {
      batinfo("[CPS] erase mtp result check fail err\n");
      goto update_fail;
    }

  write_cout++;

  for (k = 0; k < (16 * 1024 / 4) / HWREG_HW_PROGRAM_BUFF_SIZE; k++)
    {
      if (buff0_flag == 0)
        {
          /* write buf0 */

          hw_i2c_write(priv, HWREG_HW_BUFFER0_ADDR,
                       p, HWREG_HW_PROGRAM_BUFF_SIZE *4);
          p = p + HWREG_HW_PROGRAM_BUFF_SIZE *4;
          if (buff1_flag == 1)
            {
              /* wait finish */

              if ((err = cps4019_wait_cmd_done(priv)) != OK)
                {
                  batinfo("[CPS] write buff0 check fail err\n");
                  goto update_fail;
                }

               buff1_flag = 0;
            }

          /* write buff 0 CMD */

          reg_value = big_little_endian_convert(0x00000010);
          hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
          buff0_flag = 1;
          continue;
        }

      if (buff1_flag == 0)
        {
          /* write buf1 */

          hw_i2c_write(priv, HWREG_HW_BUFFER1_ADDR,
                       p, HWREG_HW_PROGRAM_BUFF_SIZE *4);
          p = p + HWREG_HW_PROGRAM_BUFF_SIZE * 4;
          if (buff0_flag == 1)
            {
              /* wait finish */

              if ((err = cps4019_wait_cmd_done(priv)) != OK)
                {
                  batinfo("[CPS] wirte buff1 check fail err\n");
                  goto update_fail;
                }

              buff0_flag = 0;
            }

          /* write buff 0 CMD */

          reg_value = big_little_endian_convert(0x00000020);
          hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
          buff1_flag = 1;
          continue;
        }
    }

  if (buff0_flag == 1)
    {
      /* wait finish */

      if ((err = cps4019_wait_cmd_done(priv)) != OK)
        {
          batinfo("[CPS]  buff0 check fail err\n");
          goto update_fail;
        }

      buff0_flag = 0;
    }

  if (buff1_flag == 1)
    {
      /* wait finish */

      if ((err = cps4019_wait_cmd_done(priv)) != OK)
        {
          batinfo("[CPS] buff1 check fail err\n");
          goto update_fail;
        }

      buff1_flag = 0;
    }

  /* check crc */

  reg_value = big_little_endian_convert(0x00000090);
  hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
  if ((err = cps4019_wait_cmd_done(priv)) != OK)
    {
      batinfo("[CPS] return cmd 0x00000090 fail err\n");
      if (write_cout < 5) goto start_write_fw_code;
      else goto update_fail;
    }

  /* write mcu start flag */

  reg_value = big_little_endian_convert(0x00000080);
  hw_i2c_write(priv, HWREG_HW_CTL_ADDR, (uint8_t *)&reg_value, 4);
  if ((err = cps4019_wait_cmd_done(priv)) != OK)
    {
      batinfo("[CPS] return cmd 0x00000080 fail err\n");
      goto update_fail;
    }

  reg_value = big_little_endian_convert(0x00000001);
  hw_i2c_write(priv, HWREG_HW_RESET_ADDR, (uint8_t *)&reg_value, 4);
  batinfo("[CPS] fw load SUCCESS\n");
  return 0;

update_fail:
  batinfo("[CPS] fw load fail\n");
  return -1;
}

static int cps4019_nvm_write(FAR struct cps4019_dev_s *priv)
{
  int err = 0;
  uint8_t reg_value = 0;

  /* check if OP MODE = RX POWER */

  err = fw_i2c_read(priv, CPS_SYS_MODE_REG, &reg_value, 1);
  if (err != OK) return err;
  batinfo("[CPS] OP MODE %02X\n", reg_value);

  if (reg_value == SYS_MODE_RX)
    {
      batinfo("[CPS] is rx mode, nvm programming aborted\n");
      return OK;
    }

  /* load bootloader */

  err = cps4019_mvn_load_bootloader(priv, (uint8_t *)CPS4019_BL,
                                    CPS4019_BL_SIZE);

  /* load fireware */

  err = cps4019_mvn_load_fireware(priv, (uint8_t *)CPS4019_FW,
                                  CPS4019_FW_SIZE);

  if (err != OK) return err;

  return OK;
}

static ssize_t nvm_program_show(FAR struct cps4019_dev_s *priv)
{
  int err = OK;
  uint32_t chip_version;
  struct cps4019_chip_info chip_info;
  batinfo("[CPS] NVM Programming started\n");

  if (get_cps4019_chip_info(priv, &chip_info) < OK)
    {
      baterr("[CPS] Error in reading cps4019_chip_info\n");
      err = E_BUS_R;
      goto exit_0;
    }

  /* determine what has to be programmed depending on version ids */

  chip_version = chip_info.fw_revision_l << 2 | chip_info.fw_revision_h;
  batinfo("[CPS] chip_id: 0x%x\n", chip_info.chip_id);
  batinfo("[CPS] chip_version: 0x%"PRIx32"\n", chip_version);

  /**************************************************************************
   * program both cfg and patch
   * in case one of the two needs to be programmed
   **************************************************************************/

  if (chip_version == CPS4019_FW_ID)
    {
      batinfo("fw is the same exit program fw\n");
      return OK;
    }

  if ((err = cps4019_nvm_write(priv)) != OK)
    {
      err = E_NVM_WRITE;
      baterr("[CPS] NVM programming failed\n");
      goto exit_0;
    }

#if 0
  if (get_cps4019_chip_info(priv, &chip_info) < OK)
    {
      baterr("[CPS] Error in reading chip_info\n");
      err = E_BUS_R;
      goto exit_0;
    }

  batinfo("[CPS] chip_id: %02X\n", chip_info.chip_id);
  batinfo("[CPS] fw_revision_l: %02X\n", chip_info.fw_revision_l);
  batinfo("[CPS] fw_revision_h: %02X\n", chip_info.fw_revision_h);
#endif

  return 0;

exit_0:
  batinfo("[CPS] NVM programming exited\n");
  return err;
}

static int cps4019_onoff_ldo_output(FAR struct cps4019_dev_s *priv,
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

static int cps4019_onoff_vaa(FAR struct cps4019_dev_s *priv,
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

static int cps4019_handle_tx_data(FAR struct cps4019_dev_s *priv)
{
  int ret;
  uint8_t reg_value[9];

  ret = fw_i2c_read(priv, CPS_RX_RCVD_DATA_REG,
                    (uint8_t *)&reg_value[0], 10);
  if (ret != OK)
    {
      batinfo("read tx data fail\n");
      return ret;
    }

  batinfo("header=0x%x, cmd=0x%x, data[0]=0x%x, data[1]=0x%x\n",
           reg_value[0], reg_value[1], reg_value[2], reg_value[3]);

  switch (reg_value[0])
    {
      /* get handshake result */

      case 0x38:
        if (reg_value[1] == 0x3b &&
            reg_value[2] == 0x02 &&
            reg_value[3] == 0xbc)
          {
            batinfo("cps4019 rx handshanke success\n");
          }

        break;
      case 0x18:
        if (reg_value[1] == 0x1a)
          {
            batinfo("adater support qc3.0\n");
          }
        else if(reg_value[1] == 0x1b)
          {
            batinfo("adater not support qc3.0\n");
          }
        else if(reg_value[1] == 0x11)
          {
            batinfo("tx is first xiaomi charge product\n");
          }

        break;

      default:
        break;
  }

  return OK;
}

/****************************************************************************
 * Name: cps4019_interrupt_handler
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

static int cps4019_interrupt_handler(FAR struct ioexpander_dev_s *dev,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  /* This function should be called upon a rising edge on the cps4019 new
   * data interrupt pin since it signals that new data has been measured.
   */

  FAR struct cps4019_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  work_queue(LPWORK, &priv->work, cps4019_worker, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: cps4019_worker
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

static int cps4019_check_intr(FAR struct cps4019_dev_s *priv,
                          FAR struct rx_int_state_s *rx_int_state)
{
  int ret;
  uint16_t reg_value;

  ret = fw_i2c_read(priv, CPS_INT_REG, (uint8_t *)&reg_value, 2);
  if (ret != OK)
    {
      baterr("[CPS] failed to read INTR states !!!\n");
      return ret;
    }

  batinfo("[CPS] read CPS_RX_INTR_LATCH_REG is %08X \n", reg_value);

  rx_int_state->cps_rx_int_otp       =
      ((reg_value & CPS_RX_OTP_INT_MASK) == false) ? false : true;
  rx_int_state->cps_rx_int_ocp       =
      ((reg_value & CPS_RX_OCP_INT_MASK) == false) ? false : true;
  rx_int_state->cps_rx_int_ovp       =
      ((reg_value & CPS_RX_OVP_INT_MASK) == false) ? false : true;
  rx_int_state->cps_rx_int_ss_tx     =
      ((reg_value & CPS_RX_ID_CFG_FINISH_INT_MASK) == false) ? false : true;
  rx_int_state->cps_rx_int_output_on =
      ((reg_value & CPS_RX_VOUT_STATE_INT_MASK) == false) ? false : true;
  rx_int_state->cps_rx_int_uvp       =
      ((reg_value & CPS_RX_UVP_INT_MASK) == false) ? false : true;

  if (reg_value & CPS_RX_R_TX_INT_MASK)
    {
      cps4019_handle_tx_data(priv);
    }

  ret = fw_i2c_read(priv, CPS_INT_ENB_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] read CPS_RX_INTR_EN_REG is %08X \n", reg_value);
  ret = fw_i2c_read(priv, CPS_INT_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] read CPS_RX_INTR_LATCH_REG is %08X \n", reg_value);

  /* CLR int register */

  batinfo("[CPS] start to CLR INTR states !!!\n");
  reg_value = 0xffff;
  ret = fw_i2c_write(priv, CPS_INT_CLR_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] read CPS_RX_INTR_CLR_REG is %08X \n", reg_value);
  if (ret != OK)
    {
      baterr("[CPS] Failed to CLR INTR states !!!\n");
      return ret;
    }

  return ret;
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
  FAR struct cps4019_dev_s *priv = arg;
  int charger_is_exit;
  int ret;

  ret = cps4019_state(&priv->dev, &charger_is_exit);
  if (ret == OK)
    {
      if (priv->batt_state_flag != charger_is_exit)
        {
          priv->batt_state_flag = charger_is_exit;
          battery_charger_changed(&priv->dev, BATTERY_STATE_CHANGED);
        }

      if (!priv->charging && !charger_is_exit)
        {
          priv->detect_work_exit = DETECT_WORK_NO_EXIST;
          work_cancel(LPWORK, &priv->detect_work);
        }
      else
        {
          work_queue(LPWORK, &priv->detect_work, detect_worker, priv,
                     CHARGER_DETECT_WORK_TIME);
        }
    }
  else
    {
      baterr("[CPS] get cps4019_state fail\n");
    }
}

static void cps4019_worker(FAR void *arg)
{
  FAR struct cps4019_dev_s *priv = arg;
  struct rx_int_state_s rx_int_state;

  DEBUGASSERT(priv != NULL);

  /* Read out the latest rx data */

  memset(&rx_int_state, 0, sizeof(struct rx_int_state_s));
  if (cps4019_check_intr(priv, &rx_int_state) == 0)
    {
      /* push data to upper half driver */

      batinfo("SUCCESS: cps4019_check_intr\n");
      batinfo("  rx_int_state.cps_rx_int_otp       = %s\n",
               rx_int_state.cps_rx_int_otp ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_ocp       = %s\n",
               rx_int_state.cps_rx_int_ocp ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_ovp       = %s\n",
               rx_int_state.cps_rx_int_ovp ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_ss_tx     = %s\n",
               rx_int_state.cps_rx_int_ss_tx ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_output_on = %s\n",
               rx_int_state.cps_rx_int_output_on ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_output_off = %s\n",
               rx_int_state.cps_rx_int_output_off ? "true" : "false");
      batinfo("  rx_int_state.cps_rx_int_uvp       = %s\n",
               rx_int_state.cps_rx_int_uvp ? "true" : "false");
    }

  /**************************************************************************
   *  if recieved ss intr, start charge_manger app
   *  if removed tx, the charge_manager app will return
   **************************************************************************/

  if (rx_int_state.cps_rx_int_output_on &&
              priv->detect_work_exit == DETECT_WORK_NO_EXIST)
    {
      priv->charging = true; /* Mark charge manager will be running */
      priv->detect_work_exit = DETECT_WORK_EXIST;
      work_queue(LPWORK, &priv->detect_work, detect_worker, priv, 0);
    }

  return;
}

static const struct battery_charger_operations_s g_cps4019ops =
{
  cps4019_state,
  cps4019_health,
  cps4019_online,
  cps4019_voltage,
  cps4019_current,
  cps4019_input_current,
  cps4019_operate,
  cps4019_chipid,
  cps4019_get_voltage,
};

static int cps4019_state(FAR struct battery_charger_dev_s *dev,
                          FAR int *status)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  bool wpc_det = 0;
  int  ret;
  uint8_t read_buff[2];

  memset(read_buff, 0, 2);
  if (fw_i2c_read(priv, CPS_ADC_VOUT_REG, read_buff, 2) < OK)
    {
      baterr("[CPS] Error in reading CPS_ADC_VOUT_REG\n");
      *status = 0;
      return OK;
    }

  /* Check WPC_DET, output High when SS package sent */

  ret = IOEXP_SETDIRECTION(priv->rpmsg_dev, priv->lower->detect_pin,
                           IOEXPANDER_DIRECTION_IN);
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

  if ((read_buff[1] << 8 | read_buff[0]) > 4900)
    {
       *status = 1;
    }

  return OK;
}

static int cps4019_health(FAR struct battery_charger_dev_s *dev,
                           FAR int *health)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  int err;
  uint16_t reg_value = 0;
  uint16_t temp;

  err = fw_i2c_read(priv, CPS_ADC_DIE_TEMP_REG, (uint8_t *)&reg_value, 2);
  if (err != OK)
    {
      *health = -1;
      return err;
    }

  temp = reg_value / 10;

  if (temp < 0)
    *health = CPS_HEALTH_UNKNOWN;
  else if (temp > CPS_HEALTH_TEMP_MAX)
    *health = CPS_HEALTH_OVERHEAT;
  else if (temp < CPS_HEALTH_TEMP_MIN)
    *health = CPS_HEALTH_OVERCOLD;
  else
    *health = CPS_HEALTH_GOOD;

  return OK;
}

static int cps4019_online(FAR struct battery_charger_dev_s *dev,
                           FAR bool *status)
{
  int ret;
  unsigned int *chipid = NULL;

  ret = cps4019_chipid(dev, chipid);

  *status = (ret == OK) ? true : false;

  return ret;
}

static int cps4019_voltage(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  int ret;
  uint16_t reg_value;
  uint8_t count = 5;

  /* check */

  reg_value = (uint16_t)value;

  while (count)
    {
      ret = fw_i2c_write(priv, CPS_VOUT_SET_REG,
                         (uint8_t *)&reg_value, 2);
      if (ret != OK)
        {
          if (count == 0)
            {
              return ret;
            }
          else
            {
              count--;
              batinfo("[CPS] left %d time to try i2c0 status.. \n", count);
              usleep(AFTER_SYS_RESET_SLEEP_MS);
              continue;
            }
        }
      else
        {
          break;
        }
    }

  batinfo("adjust vout = %d\n", value);
  ret = fw_i2c_read(priv, CPS_RX_OP_FREQ_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] rx operation frequency is %dkHz\n", reg_value);
  ret = fw_i2c_read(priv, CPS_ADC_VOUT_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] rx output voltage is %dmV\n", reg_value);
  ret = fw_i2c_read(priv, CPS_ADC_IOUT_REG, (uint8_t *)&reg_value, 2);
  batinfo("[CPS] rx output curerent is %dmA\n", reg_value);

  return ret;
}

static int cps4019_current(FAR struct battery_charger_dev_s *dev,
                            FAR int value)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  int ret;
  uint8_t reg_value;

  reg_value = (uint8_t)(value / 50);

  ret = fw_i2c_write(priv, CPS_ILIMIT_SET_REG, &reg_value, 1);

  return ret;
}

static int cps4019_input_current(FAR struct battery_charger_dev_s *dev,
                                  FAR int value)
{
  batinfo("Unsupported setup input current limit!");
  return OK;
}

static int cps4019_operate(FAR struct battery_charger_dev_s *dev,
                            uintptr_t param)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
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

        reg_value = CPS_RX_CMD_MCU_RESET;
        ret = fw_i2c_write(priv, CPS_CMD_REG, &reg_value, 2);
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

  /**************************************************************************
   * CPS4019 do not support hw reset aommand,
   * It need a reboot to check new fw version.
   **************************************************************************/

        break;

      case BATIO_OPRTN_SYSON:

        /* Turn on vout ldo output when gpio2 input low */

        ret = cps4019_onoff_ldo_output(priv, ON);
        if (ret < 0)
          {
            baterr("Failed to trun ON wpc ldo output, Error: %d\n", ret);
            ret = -EINVAL;
          }
          break;

      case BATIO_OPRTN_SYSOFF:

        /* Turn off vout ldo output when gpio2 input high */

        ret = cps4019_onoff_ldo_output(priv, OFF);
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

      default:
        batinfo("Unsupported opt: 0x%X\n", op);
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cps4019_chipid
 *
 * Description:
 *       Get chip id
 *
 ****************************************************************************/

static int cps4019_chipid(FAR struct battery_charger_dev_s *dev,
                           unsigned int *value)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  struct cps4019_chip_info chip_info;

  if (get_cps4019_chip_info(priv, &chip_info) < OK)
    {
      baterr("[CPS] Error in reading cps4019_chip_info\n");
      return E_BUS_R;
    }

  *value = chip_info.chip_id;

  batinfo("the chipid of cps4019 is: %d\n", *value);
  return OK;
}

/****************************************************************************
 * Name: cps4019_get_voltage
 *
 * Description:
 *   Get the actual output voltage from rx
 *
 ****************************************************************************/

static int cps4019_get_voltage(FAR struct battery_charger_dev_s *dev,
                                int *value)
{
  FAR struct cps4019_dev_s *priv = (FAR struct cps4019_dev_s *)dev;
  uint8_t read_buff[2];

  memset(read_buff, 0, 2);
  if (fw_i2c_read(priv, CPS_ADC_VOUT_REG, read_buff, 2) < OK)
    {
      baterr("[CPS] Error in reading CPS_ADC_VOUT_REG\n");
      return E_BUS_R;
    }

  *value = read_buff[1] << 8 | read_buff[0];

  batinfo("The the actual output voltage of cps4019 is %d mv", *value);
  return OK;
}

static int cps4019_init_interrupt(FAR struct cps4019_dev_s *priv)
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
                          cps4019_interrupt_handler, priv);
  if (ioepattach == NULL)
    {
      baterr("Failed to attach cps4019_interrupt_handler");
      ret = -EIO;
    }

  ret = IOEXP_SETOPTION(priv->io_dev, priv->lower->int_pin,
              IOEXPANDER_OPTION_INTCFG,  (FAR void *)IOEXPANDER_VAL_FALLING);
  if (ret < 0)
    {
      baterr("Failed to set option: %d\n", ret);
      IOEP_DETACH(priv->io_dev, cps4019_interrupt_handler);
    }

  return ret;
}

FAR struct battery_charger_dev_s *
  cps4019_initialize(FAR struct i2c_master_s *i2c,
                     FAR struct cps4019_lower_s *lower,
                     FAR struct ioexpander_dev_s *rpmsg_dev,
                     FAR struct ioexpander_dev_s *io_dev)
{
  FAR struct cps4019_dev_s *priv;
  int ret;

  /* Initialize the cps4019 device structure */

  priv = kmm_zalloc(sizeof(struct cps4019_dev_s));
  if (priv)
    {
      /* Initialize the cps4019 device structure */

      priv->dev.ops   = &g_cps4019ops;
      priv->i2c       = i2c;
      priv->lower     = lower;
      priv->rpmsg_dev = rpmsg_dev;
      priv->io_dev    = io_dev;
      priv->charging  = false;
      priv->batt_state_flag = BATT_CHARGING_STAT_INIT;
      priv->detect_work_exit = DETECT_WORK_EXIST;
    }
  else
    {
      return NULL;
    }

  ret = nvm_program_show(priv);
  if (ret != OK)
    baterr("Failed to [CPS] NVM programming exited, Error: %d\n", ret);

  ret = cps4019_init_interrupt(priv);
  if (ret < 0)
    {
      baterr("Failed to init_interrupt: %d\n", ret);
    }

  /* Turn on WPC_VAA_2V5 */

  /* It bring interrupt abnormal to turn on vaa, so turn off it */

  ret = cps4019_onoff_vaa(priv, OFF);
  if (ret < 0)
    {
      baterr("Failed to set vaa_pin as Enable: %d\n", ret);
    }

  /* Turn on vout ldo output when gpio2 input low */

  ret = cps4019_onoff_ldo_output(priv, ON);
  if (ret < 0)
    {
      baterr("Failed to trun ON wpc ldo output: %d\n", ret);
    }

  work_queue(LPWORK, &priv->detect_work, detect_worker, priv,
             DETECT_WORK_INIT_TIME);

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* __CPS4019_C */

