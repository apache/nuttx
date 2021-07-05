/***************************************************************************
 * drivers/input/nt38350.c
 *
 * Copyright (C) 2021 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/ioexpander/ioexpander.h>

#include <nuttx/signal.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/nt38350.h>

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Configuration ***********************************************************/

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_NT38350_NPOLLWAITERS
#  define CONFIG_NT38350_NPOLLWAITERS 2
#endif

#define NVT_OFFLINE_LOG

#define NVT_DEV_FORMAT               "/dev/input%d"
#define NVT_DEV_NAMELEN              16
#define NVT_IIC_RETRY_NUM            2
#define NVT_POINT_DATA_LEN           17
#define NVT_I2C_BLDR_ADDRESS         0x01
#define NVT_I2C_FW_ADDRESS           0x01
#define NVT_I2C_HW_ADDRESS           0x62
#define NVT_TOUCH_DEFAULT_MAX_WIDTH  430
#define NVT_TOUCH_DEFAULT_MAX_HEIGHT 372
#define NVT_TOUCH_MAX_FINGER_NUM     2
#define NVT_TOUCH_KEY_NUM            0
#define NVT_TOUCH_FORCE_NUM          1000
#define NVT_CHIP_VER_TRIM_ADDR       0x3f004
#define NVT_CHIP_VER_TRIM_OLD_ADDR   0x1f64e
#define NVT_ID_BYTE_MAX              6

#define NVT_SIZE_4KB                 4096
#define NVT_FLASH_SECTOR_SIZE        NVT_SIZE_4KB
#define NVT_SIZE_64KB                65536
#define NVT_BLOCK_64KB_NUM           4
#define NVT_FLASH_END_FLAG_LEN       3

#define NVT_BUS_TRANSFER_LENGTH      64

#define NVT_DELAY_100US              100
#define NVT_DELAY_1MS                1000
#define NVT_DELAY_5MS                5000
#define NVT_DELAY_10MS               10000
#define NVT_DELAY_15MS               15000
#define NVT_DELAY_20MS               20000
#define NVT_DELAY_80MS               80000

#define NVT_NORMAL_MODE              0x00
#define NVT_TEST_MODE_2              0x22
#define NVT_HANDSHAKING_HOST_READY   0xbb
#define NVT_XDATA_SECTOR_SIZE        256

#ifdef  NVT_OFFLINE_LOG
#define NVT_POINT_DATA_EXT_LEN       4   /* Event buffer offset 0x11~0x14 */
#define NVT_S2D_DATA_LEN             59  /* Event buffer offset 0x15~0x4F
                                          * (Byte  1~59 of 2Ddata)
                                          */
#define NVT_FW_CMD_HANDLE_LEN        18  /* Event buffer offset 0x50~0x61 */
#define NVT_S2D_DATA_EXT_LEN         13  /* Event buffer offset 0x62~0x6E
                                          * (Byte 60~72 of 2Ddata)
                                          */
#define NVT_FW_FRAME_CNT_LEN         2   /* Event buffer offset 0x6F~0x70 */
#define NVT_POINT_DATA_EXBUF_LEN     113
#endif

#ifndef max
# define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
# define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

typedef enum
{
  RESET_STATE_INIT = 0xa0, /* IC reset */
  RESET_STATE_REK,         /* ReK baseline */
  RESET_STATE_REK_FINISH,  /* baseline is ready */
  RESET_STATE_NORMAL_RUN,  /* normal run */
  RESET_STATE_MAX  = 0xaf
} rst_complete_state_t;

typedef enum
{
  EVENT_MAP_HOST_CMD                      = 0x50,
  EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE   = 0x51,
  EVENT_MAP_RESET_COMPLETE                = 0x60,
  EVENT_MAP_FWINFO                        = 0x78,
  EVENT_MAP_PROJECTID                     = 0x9a,
} i2c_event_map_t;

enum nt38350_contact_e
{
  CONTACT_NONE = 0,
  CONTACT_DOWN,
  CONTACT_MOVE,
  CONTACT_UP
};

enum nvt_finger_contact_e
{
  FINGER_DOWN = 0x01,
  FINGER_MOVE = 0x02,
  FINGER_UP   = 0xff
};

struct nvt_ts_mem_map_s
{
  uint32_t event_buf_addr;
  uint32_t raw_pipe0_addr;
  uint32_t raw_pipe1_addr;
  uint32_t baseline_addr;
  uint32_t baseline_btn_addr;
  uint32_t diff_pipe0_addr;
  uint32_t diff_pipe1_addr;
  uint32_t raw_btn_pipe0_addr;
  uint32_t raw_btn_pipe1_addr;
  uint32_t diff_btn_pipe0_addr;
  uint32_t diff_btn_pipe1_addr;
  uint32_t read_flash_checksum_addr;
  uint32_t rw_flash_data_addr;
};

struct nvt_ts_hw_info_s
{
  uint8_t carrier_system;
  uint8_t hw_crc;
};

struct nvt_ts_trim_id_table_s
{
  uint8_t      id[NVT_ID_BYTE_MAX];
  uint8_t      mask[NVT_ID_BYTE_MAX];
  const struct nvt_ts_mem_map_s *mmap;
  const struct nvt_ts_hw_info_s *hwinfo;
};

struct ts_nt38350_sample_s
{
  uint8_t    id;
  uint8_t    contact;
  bool       valid;
  uint16_t   x;
  uint16_t   y;
  uint16_t   pressure;
  uint16_t   width;
};

struct nvt_diff_s
{
  uint8_t x_num;
  uint8_t y_num;
  int16_t data[30]
};

/* This structure describes the state of one nt38350 driver instance */

struct nt38350_dev_s
{
#ifdef CONFIG_NT38350_REFCNT
  uint8_t                       crefs;             /* Number of times the device has been opened */
#endif
  uint8_t                       nwaiters;          /* Number of threads waiting for nt38350 data */
  uint8_t                       id;                /* Current touch point ID */
  volatile bool                 valid;             /* An True:  New, valid touch data in touch_info */
  sem_t                         devsem;            /* Manages exclusive access to this structure */
  sem_t                         waitsem;           /* Used to wait for the availability of data */
  uint32_t                      frequency;

  FAR struct nt38350_config_s   *config;           /* Board configuration data */
  FAR struct                    i2c_master_s *i2c; /* Saved I2C driver instance */
  struct work_s                 work;              /* Supports the interrupt handling "bottom half" */
  struct ts_nt38350_sample_s    sample;            /* Last sampled touch point data */

  /* Touch info */

  char                          *fw_path;
  uint8_t                       fw_ver;
  uint8_t                       x_num;
  uint8_t                       y_num;
  uint8_t                       max_touch_num;
  uint8_t                       max_button_num;
  uint8_t                       carrier_system;
  uint16_t                      abs_x_max;
  uint16_t                      abs_y_max;
  uint16_t                      nvt_pid;
  uint32_t                      fw_size;
  const struct nvt_ts_mem_map_s *mmap;
  size_t                        fw_need_write_size;
#ifdef NVT_OFFLINE_LOG
  struct work_s                 nvt_log_wq;
  uint8_t point_xdata_temp[NVT_POINT_DATA_EXBUF_LEN];
#endif

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd                 *fds[CONFIG_NT38350_NPOLLWAITERS];
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static void nt38350_notify(FAR struct nt38350_dev_s *priv);
static int nt38350_sample(FAR struct nt38350_dev_s *priv,
                          FAR struct ts_nt38350_sample_s *sample);

static int nt38350_i2c_read(FAR struct i2c_master_s *dev,
                            FAR const struct i2c_config_s *config,
                            uint16_t address,
                            uint8_t *rdbuffer, int length);
static int nt38350_i2c_write(FAR struct i2c_master_s *dev,
                             FAR const struct i2c_config_s *config,
                             uint16_t address,
                             uint8_t *buffer, int length);

static int nt38350_read_reg(FAR struct nt38350_dev_s *priv,
                            uint16_t address,
                            FAR uint8_t *buffer,
                            uint16_t length);
static int nt38350_write_reg(FAR struct nt38350_dev_s *priv,
                             uint16_t address,
                             FAR uint8_t *buffer,
                             uint16_t length);

/* Character driver methods */

static int nt38350_open(FAR struct file *filep);
static int nt38350_close(FAR struct file *filep);
static ssize_t nt38350_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int nt38350_ioctl(FAR struct file *filep,
                         int cmd, unsigned long arg);
static int nt38350_poll(FAR struct file *filep,
                        struct pollfd *fds,
                        bool setup);

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_nt38350_fops =
{
  nt38350_open,
  nt38350_close,
  nt38350_read,
  0,
  0,
  nt38350_ioctl,
  nt38350_poll
};

static const struct nvt_ts_mem_map_s g_nt38350_memory_map =
{
  .event_buf_addr           = 0x20000,
  .raw_pipe0_addr           = 0x20160,
  .raw_pipe1_addr           = 0x20660,
  .baseline_addr            = 0x20268,
  .baseline_btn_addr        = 0x20768,
  .diff_pipe0_addr          = 0x201fc,
  .diff_pipe1_addr          = 0x206fc,
  .raw_btn_pipe0_addr       = 0x20260,
  .raw_btn_pipe1_addr       = 0x20260,
  .diff_btn_pipe0_addr      = 0x20760,
  .diff_btn_pipe1_addr      = 0x20760,
  .read_flash_checksum_addr = 0x21000,
  .rw_flash_data_addr       = 0x21002,
};

static const struct nvt_ts_hw_info_s g_nt38350_hw_info =
{
  .carrier_system = 0,
  .hw_crc         = 1,
};

static const struct nvt_ts_trim_id_table_s g_trim_id_table[] =
{
  {
    .id =
      {
        0xff, 0xff, 0xff, 0x50, 0x83, 0x03
      },
    .mask =
      {
        0, 0, 0, 1, 1, 1
      },
    .mmap   = &g_nt38350_memory_map,
    .hwinfo = &g_nt38350_hw_info
  },
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static int nt38350_i2c_read(FAR struct i2c_master_s *dev,
                            FAR const struct i2c_config_s *config,
                            uint16_t address,
                            uint8_t *rdbuffer, int length)
{
  /* Setup for the transfer */

  struct i2c_msg_s msg[2];
  int ret = -1;
  int retries = 0;

  msg[0].frequency = config->frequency;
  msg[0].addr      = address;
  msg[0].flags     = 0;
  msg[0].buffer    = &rdbuffer[0];
  msg[0].length    = 1;

  msg[1].frequency = config->frequency;
  msg[1].addr      = address;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &rdbuffer[1];
  msg[1].length    = length - 1;

  /* Then perform the transfer dev addr and reg addr, read reg value */

  while (retries < NVT_IIC_RETRY_NUM)
    {
      ret = I2C_TRANSFER(dev, msg, 2);
      if (ret == 0)
          break;
      retries++;
    }

  if (retries >= NVT_IIC_RETRY_NUM)
    {
      /* Failed read */

      syslog(LOG_ERR, "Failed to I2C read\n");
      return -1;
    }

  return (ret >= 0) ? OK : ret;
}

static int nt38350_i2c_write(FAR struct i2c_master_s *dev,
                             FAR const struct i2c_config_s *config,
                             uint16_t address,
                             uint8_t *buffer, int length)
{
  struct i2c_msg_s msg[1];
  int ret;

  /* Setup for the transfer */

  msg[0].frequency = config->frequency;
  msg[0].addr      = address;
  msg[0].flags     = 0;
  msg[0].buffer    = (uint8_t *)buffer;
  msg[0].length    = length;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev, msg, 1);
  return (ret >= 0) ? OK : ret;
}

static int nt38350_read_reg(FAR struct nt38350_dev_s *priv,
                            uint16_t address,
                            FAR uint8_t *buffer,
                            uint16_t length)
{
  struct i2c_config_s config;
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->config->frequency;
  config.address   = address;
  config.addrlen   = 7;
  ret = nt38350_i2c_read(priv->config->i2c, &config,
                         address, buffer, length);
  if (ret != OK)
    {
      /* Read error */

      ierr("ERROR: %s Failed to read reg: %d\n", __func__, ret);
      return ret;
    }
}

static int nt38350_write_reg(FAR struct nt38350_dev_s *priv,
                             uint16_t address,
                             FAR uint8_t *buffer,
                             uint16_t length)
{
  struct i2c_config_s config;
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->config->frequency;
  config.address   = address;
  config.addrlen   = 7;
  ret = nt38350_i2c_write(priv->config->i2c, &config,
                          address, buffer, length);
  if (ret != OK)
    {
      ierr("ERROR: Failed to write reg %s : %d\n", __func__, __LINE__);

      /* syserr("ERROR: i2c_write returned error code %d\n", ret); */

      return ret;
    }

  return OK;
}

static void u8toi16(const uint8_t *stream, int32_t *data, uint32_t len)
{
  int i;

  for (i = 0; i < len / 2; i++)
    {
      data[i] = ((int32_t)stream[i * 2] + (((int32_t)stream[i * 2 + 1]) << 8));
    }
}

/***************************************************************************
 *Description:
 *    Novatek touchscreen set index/page/addr address.
 *
 * return :
 *    Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_set_page(FAR struct nt38350_dev_s *priv, uint16_t i2c_addr,
            uint32_t addr)
{
  uint8_t buf[4];
  buf[0] = 0xff;
  buf[1] = (addr >> 16) & 0xff;
  buf[2] = (addr >> 8) & 0xff;

  return nt38350_write_reg(priv, i2c_addr, buf, 3);
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen reset MCU then into idle mode
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_sw_reset_idle(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[4];
  int ret;

  buf[0] = 0x00;
  buf[1] = 0xa5;

  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);

  usleep(NVT_DELAY_15MS);

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen reset MCU (boot) function.
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_bootloader_reset(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[4];
  int ret;

  buf[0] = 0x00;
  buf[1] = 0x69;

  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret != OK)
    {
      ierr("ERROR: Failed to reset bootloader\n");
    }

  usleep(NVT_DELAY_15MS);

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen clear FW status function.
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_clear_fw_status(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[4];
  int32_t i;
  int ret;
  const int32_t retry = 20;

  for (i = 0; i < retry; i++)
    {
      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
        EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE);

      buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
      buf[1] = 0x00;
      ret = nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
      buf[1] = 0xff;
      ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      if (buf[1] == 0x00)
        {
          break;
        }

      usleep(NVT_DELAY_10MS);
    }

  if (i >= retry)
    {
      ierr("ERROR: Failed to clear FW status\n");
      return -1;
    }
  else
    {
      return OK;
    }
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen check FW status function.
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_check_fw_status(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[4];
  int32_t i;
  int ret;
  const int32_t retry = 50;

  for (i = 0; i < retry; i++)
    {
      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
        EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE);

      buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      if ((buf[1] & 0xf0) == 0xa0)
         break;
      usleep(NVT_DELAY_10MS);
    }

  if (i >= retry)
    {
      ierr("ERROR: Failed to clear FW status\n");
      return -1;
    }
  else
    {
      return OK;
    }
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen check FW reset state function.
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_check_fw_reset_state(FAR struct nt38350_dev_s *priv,
                            rst_complete_state_t check_reset_state)
{
  uint8_t buf[8];
  int32_t ret;
  int32_t retry = 0;

  while (1)
    {
      usleep(NVT_DELAY_10MS);
      buf[0] = EVENT_MAP_RESET_COMPLETE;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX))
        {
          ret = OK;
          break;
        }

      retry++;
      if (retry > 100)
        {
          ierr("ERROR: Failed to check FW state\n");
          ret = -1;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen get novatek project id information
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_read_pid(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[3];
  uint16_t nvt_pid;
  int ret;

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
      EVENT_MAP_PROJECTID);
  buf[0] = EVENT_MAP_PROJECTID;
  buf[1] = 0x00;
  buf[2] = 0x00;
  ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 3);

  nvt_pid = (buf[2] << 8) + buf[1];
  priv->nvt_pid = nvt_pid;

#if NVT_DEBUG
  iinfo("PID = %04x\n", nvt_pid);
#endif

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen get firmware related information
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_get_fw_info(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[64];
  uint32_t retry_count = 0;
  int32_t ret;

info_retry:
  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
               EVENT_MAP_FWINFO);
  buf[0] = EVENT_MAP_FWINFO;
  ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 17);
  priv->fw_ver = buf[1];
  priv->x_num = buf[3];
  priv->y_num = buf[4];
  priv->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
  priv->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
  priv->max_button_num = buf[11];

  if ((buf[1] + buf[2]) != 0xff)
    {
      priv->fw_ver = 0;
      priv->x_num = 6;
      priv->y_num = 6;
      priv->abs_x_max = NVT_TOUCH_DEFAULT_MAX_WIDTH;
      priv->abs_y_max = NVT_TOUCH_DEFAULT_MAX_HEIGHT;
      priv->max_button_num = NVT_TOUCH_KEY_NUM;

      if (retry_count < 3)
        {
          retry_count++;
          ierr("retry_count = %d\n", retry_count);
          goto info_retry;
        }
      else
        {
           ret = -1;
        }
    }

  nvt_read_pid(priv);

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen check and stop crc reboot loop
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_stop_crc_reboot(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[8];
  int32_t retry;
  int ret;

  /* ---change I2C index to prevent geting 0xFF, but not 0xFC--- */

  nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x1f64e);

  /* ---read to check if buf is 0xFC which means IC is in CRC reboot */

  buf[0] = 0x4e;
  ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 4);

  if ((buf[1] == 0xfc) ||
             ((buf[1] == 0xff) && (buf[2] == 0xff) && (buf[3] == 0xff)))
    {
      for (retry = 5; retry > 0; retry--)
        {
          /* ---write i2c cmds to reset idle : 1st--- */

          buf[0] = 0x00;
          buf[1] = 0xa5;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);

          /* ---write i2c cmds to reset idle : 2ed--- */

          buf[0] = 0x00;
          buf[1] = 0xa5;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          usleep(NVT_DELAY_1MS);

          /* ---clear CRC_ERR_FLAG--- */

          nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x3f135);

          buf[0] = 0x35;
          buf[1] = 0xa5;
          ret = nt38350_write_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 2);

          /* ---check CRC_ERR_FLAG--- */

          nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x3f135);

          buf[0] = 0x35;
          buf[1] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 2);
          if (buf[1] == 0xa5)
               break;
        }

      if (retry == 0)
          ierr("ERROR: CRC auto reboot is not able "
               "to be stopped buf[1]=0x%02x\n", buf[1]);
    }

  return ret;
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen check chip version trim function.
 *
 *return:
 *   Executive outcomes. 0---succeed. other---access fail.
 ***************************************************************************/

static int nvt_ts_check_chip_ver_trim(FAR struct nt38350_dev_s *priv,
             uint32_t chip_ver_trim_addr)
{
  int32_t found_nvt_chip = 0;
  uint8_t buf[8];
  int32_t retry;
  int32_t list;
  int32_t ret = -1;
  int32_t i;

  nvt_bootloader_reset(priv);

  for (retry = 5; retry > 0; retry--)
    {
      nvt_sw_reset_idle(priv);
      buf[0] = 0x00;
      buf[1] = 0x35;
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      usleep(NVT_DELAY_10MS);

      nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, chip_ver_trim_addr);

      buf[0] = chip_ver_trim_addr & 0xff;
      buf[1] = 0x00;
      buf[2] = 0x00;
      buf[3] = 0x00;
      buf[4] = 0x00;
      buf[5] = 0x00;
      buf[6] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 7);

      /* Get Touch IC ID */

#if NVT_DEBUG
      iinfo("buf[1]=0x%02x, buf[2]=0x%02x, buf[3]=0x%02x,"
            "buf[4]=0x%02x, buf[5]=0x%02x,buf[6]=0x%02x\n",
            buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
#endif

      /* ---Stop CRC check to prevent IC auto reboot--- */

      if ((buf[1] == 0xfc) ||
          ((buf[1] == 0xff) && (buf[2] == 0xff) && (buf[3] == 0xff)))
        {
          nvt_stop_crc_reboot(priv);
          continue;
        }

      /* compare read chip id on supported list */

      for (list = 0; list < (sizeof(g_trim_id_table) /
                     sizeof(struct nvt_ts_trim_id_table_s)); list++)
        {
          found_nvt_chip = 0;

          /* compare each byte */

          for (i = 0; i < NVT_ID_BYTE_MAX; i++)
            {
              if (g_trim_id_table[list].mask[i])
                {
                  if (buf[i + 1] != g_trim_id_table[list].id[i])
                       break;
                }
            }

          if (i == NVT_ID_BYTE_MAX)
            {
              found_nvt_chip = 1;
            }

          if (found_nvt_chip)
            {
#if NVT_DEBUG
              ierr("This NVT touch IC\n");
#endif
              priv->mmap = g_trim_id_table[list].mmap;
              priv->carrier_system =
                    g_trim_id_table[list].hwinfo->carrier_system;
              ret = 0;
              goto out;
            }
          else
            {
              priv->mmap = NULL;
              ret = -1;
            }
        }

      usleep(NVT_DELAY_10MS);
    }

out:
  return ret;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen get firmware size
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int get_nvt_fw_size(FAR struct nt38350_dev_s *priv)
{
  uint32_t fw_size;
  char *fw_path = priv->fw_path;
  FILE *fp = NULL;

  fp = fopen(fw_path, "rb");
  if (fp != NULL)
    {
      if (fseek(fp, 0, SEEK_END))
        {
          fclose(fp);
          return -1;
        }

      fw_size = ftell(fp);
      fclose(fp);
      priv->fw_size = fw_size;
      return 0;
    }

  return -1;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen get firmware content
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int get_nvt_fw_content(FAR struct nt38350_dev_s *priv,
                              FAR uint8_t *fw_data)
{
  int ret;
  char *fw_path = priv->fw_path;
  FILE *fp = NULL;
  uint32_t fw_size = priv->fw_size;

  fp = fopen(fw_path, "rb");
  if (fp != NULL)
    {
      fseek(fp, 0, SEEK_SET);

      ret = fread(fw_data, fw_size, 1, fp);
      if (ret == 0)
        {
          ierr("ERROR: Failed get firmware content\n");
          return -1;
        }
      else
        {
          return 0;
        }
    }

  return -1;
}

static int nvt_get_fw_need_write_size(FAR struct nt38350_dev_s *priv,
                                      FAR uint8_t *data)
{
  int ret;
  int i;
  uint32_t total_sectors_to_check;

  total_sectors_to_check = priv->fw_size / NVT_FLASH_SECTOR_SIZE;
#if NVT_DEBUG
  iinfo("total_sectors_to_check %d\n", total_sectors_to_check);
#endif

  for (i = total_sectors_to_check; i > 0; i--)
    {
      /* check if there is end flag "NVT" at the end of this sector */

      if (strncmp(&data[i * NVT_FLASH_SECTOR_SIZE -
          NVT_FLASH_END_FLAG_LEN], "NVT", NVT_FLASH_END_FLAG_LEN) == 0)
        {
          priv->fw_need_write_size = i * NVT_FLASH_SECTOR_SIZE;
#if NVT_DEBUG
          iinfo("fw_need_write_size = %zu(0x%zx)\n", priv->fw_need_write_size,
                 priv->fw_need_write_size);
#endif
          return 0;
        }
    }

  return -1;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen initial bootloader and flash
 *     block function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_init_bootloader(FAR struct nt38350_dev_s *priv)
{
  int32_t ret;
  int32_t retry;
  uint8_t buf[8] =
  {
    0
  };

  /* SW Reset & Idle */

  nvt_sw_reset_idle(priv);

  /* Initiate Flash Block */

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = NVT_I2C_FW_ADDRESS;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
  if (ret < 0)
    {
      ierr("ERROR: Inittial Flash Block error!!(%d)\n", ret);
      return ret;
    }

  /* Check 0xAA (Initiate Flash Block) */

  retry = 0;
  while (1)
    {
      usleep(NVT_DELAY_1MS);
      buf[0] = 0x00;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Check 0xAA (Inittial Flash Block) "
               "error.(%d)\n", ret);
          return ret;
        }

      if (buf[1] == 0xaa)
        {
          break;
        }

      retry++;

      if (retry > 20)
        {
          ierr("ERROR: Check 0xAA (Inittial Flash Block) error."
               "status=0x%02X\n", buf[1]);
          return -1;
        }
    }

  usleep(NVT_DELAY_20MS);

  return 0;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen resume from deep power down function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_resume_pd(FAR struct nt38350_dev_s *priv)
{
  int32_t ret;
  int32_t retry;
  uint8_t buf[4] =
  {
    0
  };

  /* Resume Command */

  buf[0] = 0x00;
  buf[1] = 0xab;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: Write Enable error!!(%d)\n", ret);
      return ret;
    }

  /* Check 0xAA (Resume Command) */

  retry = 0;
  while (1)
    {
      usleep(NVT_DELAY_1MS);
      buf[0] = 0x00;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Check 0xAA (Resume Command) error!!(%d)\n", ret);
          return ret;
        }

      if (buf[1] == 0xaa)
        {
          break;
        }

      retry++;

      if (retry > 20)
        {
          ierr("ERROR: Check 0xAA (Resume Command) error!!"
               "status=0x%02X\n", buf[1]);
          return -1;
        }
    }

  usleep(NVT_DELAY_10MS);

  return 0;
}

/***************************************************************************
 *Description:
 *      Novatek touchscreen erase flash sectors function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_erase_flash(FAR struct nt38350_dev_s *priv)
{
  int32_t ret;
  int32_t count;
  int32_t i;
  int32_t flash_address;
  int32_t retry;
  uint8_t buf[12] =
  {
    0
  };

  /* Write Enable */

  buf[0] = 0x00;
  buf[1] = 0x06;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: Write Enable (for Write Status Register)"
           " error!!(%d)\n", ret);
      return ret;
    }

  /* Check 0xAA (Write Enable) */

  retry = 0;

  while (1)
    {
      usleep(NVT_DELAY_1MS);
      buf[0] = 0x00;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Check 0xAA (Write Enable for Write Status Register)"
               " error!!(%d)\n", ret);
          return ret;
        }

      if (buf[1] == 0xaa)
        {
          break;
        }

      retry++;
      if (retry > 20)
        {
          ierr("ERROR: Check 0xAA (Write Enable for Write "
               "Status Register)error!! status=0x%02X\n", buf[1]);
          return -1;
        }
    }

  /* Write Status Register */

  buf[0] = 0x00;
  buf[1] = 0x01;
  buf[2] = 0x00;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
  if (ret < 0)
    {
      ierr("ERROR: Write Status Register error!!(%d)\n", ret);
      return ret;
    }

  /* Check 0xAA (Write Status Register) */

  retry = 0;
  while (1)
    {
      usleep(NVT_DELAY_1MS);
      buf[0] = 0x00;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Check 0xAA (Write Status Register)"
               " error!!(%d)\n", ret);
          return ret;
        }

      if (buf[1] == 0xaa)
        {
          break;
        }

      retry++;
      if (retry > 20)
        {
          ierr("ERROR: Check 0xAA (Write Status Register) "
               "error!! status=0x%02X\n", buf[1]);
          return -1;
        }
    }

  /* Read Status */

  retry = 0;
  while (1)
    {
      usleep(NVT_DELAY_5MS);
      buf[0] = 0x00;
      buf[1] = 0x05;
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Read Status (for Write Status Register)"
               "error!!(%d)\n", ret);
          return ret;
        }

      /* Check 0xAA (Read Status) */

      buf[0] = 0x00;
      buf[1] = 0x00;
      buf[2] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
      if (ret < 0)
        {
          ierr("ERROR: Check 0xAA (Read Status for"
               "Write Status Register) error!!(%d)\n", ret);
          return ret;
        }

      if ((buf[1] == 0xaa) && (buf[2] == 0x00))
        {
          break;
        }

      retry++;
      if (retry > 100)
        {
          ierr("ERROR: Check 0xAA (Read Status for Write Status"
               " Register) failed,buf[1]=0x%02X, buf[2]=0x%02X,"
               "retry=%d\n", buf[1], buf[2], retry);
          return -1;
        }
    }

  if (priv->fw_need_write_size % NVT_FLASH_SECTOR_SIZE)
    count = priv->fw_need_write_size / NVT_FLASH_SECTOR_SIZE + 1;
  else
    count = priv->fw_need_write_size / NVT_FLASH_SECTOR_SIZE;

  for (i = 0; i < count; i++)
    {
      /* Write Enable */

      buf[0] = 0x00;
      buf[1] = 0x06;
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Write Enable error!!(%d,%d)\n", ret, i);
          return ret;
        }

      /* Check 0xAA (Write Enable) */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_1MS);
          buf[0] = 0x00;
          buf[1] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Write Enable) error!!(%d,%d)\n",
                   ret, i);
              return ret;
            }

          if (buf[1] == 0xaa)
            {
              break;
            }

          retry++;
          if (retry > 20)
            {
              ierr("ERROR: Check 0xAA (Write Enable) error!!"
                   "status=0x%02X\n", buf[1]);
              return -1;
            }
        }

      flash_address = i * NVT_FLASH_SECTOR_SIZE;

      /* Sector Erase */

      buf[0] = 0x00;

      /* Command : Sector Erase */

      buf[1] = 0x20;
      buf[2] = ((flash_address >> 16) & 0xff);
      buf[3] = ((flash_address >> 8) & 0xff);
      buf[4] = (flash_address & 0xff);
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 5);
      if (ret < 0)
        {
          ierr("ERROR: Sector Erase error!!(%d,%d)\n", ret, i);
          return ret;
        }

      /* Check 0xAA (Sector Erase) */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_1MS);
          buf[0] = 0x00;
          buf[1] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Sector Erase) error!!(%d,%d)\n",
                   ret, i);
              return ret;
            }

          if (buf[1] == 0xaa)
            {
              break;
            }

          retry++;

          if (retry > 20)
            {
              ierr("ERROR: Check 0xAA (Sector Erase) failed,"
                   "buf[1]=0x%02X,retry=%d\n", buf[1], retry);
              return -1;
            }
        }

      /* Read Status */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_5MS);
          buf[0] = 0x00;
          buf[1] = 0x05;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Read Status error!!(%d,%d)\n", ret, i);
              return ret;
            }

          /* Check 0xAA (Read Status) */

          buf[0] = 0x00;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Read Status) error!!(%d,%d)\n",
                   ret, i);
              return ret;
            }

          if ((buf[1] == 0xaa) && (buf[2] == 0x00))
            {
              break;
            }

          retry++;

          if (retry > 100)
            {
              ierr("ERROR: Check 0xAA (Read Status) failed,"
                   "buf[1]=0x%02X, buf[2]=0x%02X,retry=%d\n",
                    buf[1], buf[2], retry);
              return -1;
            }
        }
    }

  return 0;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen write flash sectors function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_write_flash(FAR struct nt38350_dev_s *priv, uint8_t *data)
{
  int32_t i;
  int32_t j;
  int32_t k;
  int32_t ret;
  int32_t retry;
  int32_t count;
  uint8_t tmpvalue;
  int32_t percent;
  uint32_t flash_address;
  int32_t previous_percent = -1;
  uint32_t xdata_addr = priv->mmap->rw_flash_data_addr;
  uint8_t buf[64] =
  {
     0
  };

  /* change I2C buffer index */

  ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, xdata_addr);
  if (ret < 0)
    {
      ierr("ERROR: change I2C buffer index error!!(%d)\n", ret);
      return ret;
    }

  if (priv->fw_need_write_size % 256)
    {
      count = priv->fw_need_write_size / 256 + 1;
    }
  else
    {
      count = priv->fw_need_write_size / 256;
    }

  for (i = 0; i < count; i++)
    {
      flash_address = i * 256;

      /* Write Enable */

      buf[0] = 0x00;
      buf[1] = 0x06;
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      if (ret < 0)
        {
          ierr("ERROR: Write Enable error!!(%d)\n", ret);
          return ret;
        }

      /* Check 0xAA (Write Enable) */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_100US);
          buf[0] = 0x00;
          buf[1] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Write Enable) error!!(%d,%d)\n",
                   ret, i);
              return ret;
            }

          if (buf[1] == 0xaa)
            {
              break;
            }

          retry++;

          if (retry > 20)
            {
              ierr("ERROR: Check 0xAA (Write Enable) error!!"
                   "status=0x%02X\n", buf[1]);
              return -1;
            }
        }

      /* Write Page : 256 bytes */

      const size_t min_size = min(priv->fw_need_write_size - i * 256,
                                 (size_t)256);

      for (j = 0; j < min_size; j += 32)
        {
          buf[0] = (xdata_addr + j) & 0xff;
          for (k = 0; k < 32; k++)
            {
              buf[1 + k] = data[flash_address + j + k];
            }

          ret = nt38350_write_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 33);
          if (ret < 0)
            {
              ierr("ERROR: Write Page error!!(%d), j=%d\n", ret, j);
              return ret;
            }
        }

      if (priv->fw_need_write_size - flash_address >= 256)
        tmpvalue = (flash_address >> 16) + ((flash_address >> 8) & 0xff) +
                   (flash_address & 0xff) + 0x00 + (255);
      else
        tmpvalue = (flash_address >> 16) + ((flash_address >> 8) & 0xff) +
                   (flash_address & 0xff) + 0x00 +
                   (priv->fw_need_write_size - flash_address - 1);

      const size_t min_size_tmp = min(priv->fw_need_write_size -
                                      flash_address, (size_t)256);

      for (k = 0; k < min_size_tmp; k++)
        tmpvalue += data[flash_address + k];

      tmpvalue = 255 - tmpvalue + 1;

      /* Page Program */

      buf[0] = 0x00;
      buf[1] = 0x02;
      buf[2] = ((flash_address >> 16) & 0xff);
      buf[3] = ((flash_address >> 8) & 0xff);
      buf[4] = (flash_address & 0xff);
      buf[5] = 0x00;
      buf[6] = min(priv->fw_need_write_size - flash_address,
                   (size_t)256) - 1;
      buf[7] = tmpvalue;
      ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 8);
      if (ret < 0)
        {
          ierr("ERROR: Page Program error!!(%d), i=%d\n", ret, i);
          return ret;
        }

      /* Check 0xAA (Page Program) */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_1MS);
          buf[0] = 0x00;
          buf[1] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Page Program error!!(%d)\n", ret);
              return ret;
            }

          if (buf[1] == 0xaa || buf[1] == 0xea)
            {
              break;
            }

          retry++;
          if (retry > 20)
            {
              ierr("ERROR: Check 0xAA (Page Program) failed,"
                   "buf[1]=0x%02X,retry=%d\n", buf[1], retry);
              return -1;
            }
        }

      if (buf[1] == 0xea)
        {
          ierr("ERROR: Page Program error!! i=%d  %d\n", i, __LINE__);
          return -3;
        }

      /* Read Status */

      retry = 0;
      while (1)
        {
          usleep(NVT_DELAY_5MS);
          buf[0] = 0x00;
          buf[1] = 0x05;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          if (ret < 0)
            {
              ierr("ERROR: Read Status error!!(%d)\n", ret);
              return ret;
            }

          /* Check 0xAA (Read Status) */

          buf[0] = 0x00;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Read Status) error!!(%d)\n", ret);
              return ret;
            }

          if (((buf[1] == 0xaa) && (buf[2] == 0x00)) || (buf[1] == 0xea))
            {
              break;
            }

          retry++;
          if (retry > 100)
            {
              ierr("ERROR: Check 0xAA (Read Status) failed,"
                   "buf[1]=0x%02X, buf[2]=0x%02X,retry=%d\n",
                    buf[1], buf[2], retry);
              return -1;
            }
        }

      if (buf[1] == 0xea)
        {
          ierr("Page Program error!! i=%d %d\n", i, __LINE__);
          return -4;
        }

      percent = ((i + 1) * 100) / count;
      if (((percent % 10) == 0) && (percent != previous_percent))
        {
          iinfo("Programming...%2d%%\n", percent);
          previous_percent = percent;
        }
    }

  return 0;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen verify checksum of written
 *     flash function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_verify_flash(FAR struct nt38350_dev_s *priv, uint8_t *data)
{
  int32_t i;
  int32_t k;
  int32_t ret;
  int32_t retry;
  size_t len_in_blk;
  uint32_t xdata_addr = priv->mmap->read_flash_checksum_addr;
  uint8_t buf[64] =
  {
    0
  };

  uint16_t wr_filechksum[NVT_BLOCK_64KB_NUM] =
  {
    0
  };

  uint16_t rd_filechksum[NVT_BLOCK_64KB_NUM] =
  {
    0
  };

  for (i = 0; i < NVT_BLOCK_64KB_NUM; i++)
    {
      if (priv->fw_need_write_size > (i * NVT_SIZE_64KB))
        {
          /* Calculate wr_filechksum of each 64KB block */

          len_in_blk = min(priv->fw_need_write_size -
                       i * NVT_SIZE_64KB, (size_t)NVT_SIZE_64KB);
          wr_filechksum[i] = i + 0x00 + 0x00 +
              (((len_in_blk - 1) >> 8) & 0xff) + ((len_in_blk - 1) & 0xff);
          for (k = 0; k < len_in_blk; k++)
            {
              wr_filechksum[i] += data[k + i * NVT_SIZE_64KB];
            }

          wr_filechksum[i] = 65535 - wr_filechksum[i] + 1;

          /* Fast Read Command */

          buf[0] = 0x00;
          buf[1] = 0x07;
          buf[2] = i;
          buf[3] = 0x00;
          buf[4] = 0x00;
          buf[5] = ((len_in_blk - 1) >> 8) & 0xff;
          buf[6] = (len_in_blk - 1) & 0xff;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 7);
          if (ret < 0)
            {
              ierr("ERROR: Fast Read Command error!!(%d)\n", ret);
              return ret;
            }

          /* Check 0xAA (Fast Read Command) */

          retry = 0;
          while (1)
            {
              usleep(NVT_DELAY_80MS);
              buf[0] = 0x00;
              buf[1] = 0x00;
              ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
              if (ret < 0)
                {
                  ierr("ERROR: Check 0xAA (Fast Read Command)"
                       "error!!(%d)\n", ret);
                  return ret;
                }

              if (buf[1] == 0xaa)
                {
                  break;
                }

              retry++;
              if (retry > 5)
                {
                  ierr("ERROR: Check 0xAA (Fast Read Command)failed,"
                        "buf[1]=0x%02X,retry=%d\n", buf[1], retry);
                  return -1;
                }
            }

          /* Read Checksum (write addr high byte & middle byte) */

          ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, xdata_addr);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum (write addr high byte &"
                   "middle byte)error!(%d)\n", ret);
              return ret;
            }

          /* Read Checksum */

          buf[0] = (xdata_addr) & 0xff;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum error!!(%d)\n", ret);
              return ret;
            }

          rd_filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
          if (wr_filechksum[i] != rd_filechksum[i])
            {
              ierr("ERROR: Verify Fail%d!!\n", i);
              ierr("ERROR: rd_filechksum[%d]=0x%04X,"
                   "wr_filechksum[%d]=0x%04X\n",
                   i, rd_filechksum[i], i, wr_filechksum[i]);
              return -1;
            }
        }
    }

  return 0;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen check firmware checksum function.
 *
 *return:
 *     Executive outcomes. 0---checksum not match.
 *     1---checksum match. -1--- checksum read failed.
 ***************************************************************************/

static int nvt_check_checksum(FAR struct nt38350_dev_s *priv, uint8_t *data)
{
  uint32_t xdata_addr = priv->mmap->read_flash_checksum_addr;
  int32_t ret;
  int32_t i;
  int32_t k;
  size_t len_in_blk;
  int32_t retry;
  uint8_t buf[64] =
  {
    0
  };

  uint16_t wr_filechksum[NVT_BLOCK_64KB_NUM] =
  {
    0
  };

  uint16_t rd_filechksum[NVT_BLOCK_64KB_NUM] =
  {
    0
  };

  if (nvt_resume_pd(priv))
    {
      ierr("ERROR: Resume PD error!!\n");
      return -1;
    }

  for (i = 0; i < NVT_BLOCK_64KB_NUM; i++)
    {
      if (priv->fw_need_write_size > (i * NVT_SIZE_64KB))
        {
          /* Calculate wr_filechksum of each 64KB block */

          len_in_blk = min(priv->fw_need_write_size -
             i * NVT_SIZE_64KB, (size_t)NVT_SIZE_64KB);
          wr_filechksum[i] = i + 0x00 + 0x00 +
             (((len_in_blk - 1) >> 8) & 0xff) + ((len_in_blk - 1) & 0xff);
          for (k = 0; k < len_in_blk; k++)
            {
              wr_filechksum[i] += data[k + i * NVT_SIZE_64KB];
            }

          wr_filechksum[i] = 65535 - wr_filechksum[i] + 1;

          /* Fast Read Command */

          buf[0] = 0x00;
          buf[1] = 0x07;
          buf[2] = i;
          buf[3] = 0x00;
          buf[4] = 0x00;
          buf[5] = ((len_in_blk - 1) >> 8) & 0xff;
          buf[6] = (len_in_blk - 1) & 0xff;
          ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 7);
          if (ret < 0)
            {
              ierr("ERROR: Fast Read Command error!!(%d)\n", ret);
              return ret;
            }

          /* Check 0xAA (Fast Read Command) */

          retry = 0;
          while (1)
            {
              usleep(NVT_DELAY_80MS);
              buf[0] = 0x00;
              buf[1] = 0x00;
              ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
              if (ret < 0)
                {
                  ierr("ERROR: Check 0xAA (Fast Read Command) "
                       "error!!(%d)\n", ret);
                  return ret;
                }

              if (buf[1] == 0xaa)
                {
                  break;
                }

              retry++;

              if (retry > 5)
                {
                  ierr("ERROR: Check 0xAA (Fast Read Command)failed,"
                       "buf[1]=0x%02X,retry=%d\n", buf[1], retry);
                  return -1;
                }
            }

          /* Read Checksum (write addr high byte & middle byte) */

          ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, xdata_addr);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum (write addr high byte &"
                "middle byte)error!(%d)\n", ret);
              return ret;
            }

          /* Read Checksum */

          buf[0] = (xdata_addr) & 0xff;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum error!!(%d)\n", ret);
              return ret;
            }

          rd_filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
          if (wr_filechksum[i] != rd_filechksum[i])
            {
              ierr("ERROR: rd_filechksum[%d]=0x%04X,"
                   "wr_filechksum[%d]=0x%04X\n",
                   i, rd_filechksum[i], i, wr_filechksum[i]);
              ierr("ERROR: firmware checksum not match!!\n");
              return 0;
            }
        }
    }

  return 1;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen request update firmware function.
 *
 *return:
 *     Executive outcomes. 0---succeed. -1,-22---failed.
 ***************************************************************************/

static int nvt_update_firmware_request(FAR struct nt38350_dev_s *priv,
          FAR uint8_t *data)
{
  size_t nvt_fw_bin_ver_offset;
  size_t nvt_fw_bin_ver_bar_offset;
  /* check FW need to write size */

  if (nvt_get_fw_need_write_size(priv, data))
    {
      ierr("ERROR: get fw need to write size fail!\n");
      return -1;
    }

  nvt_fw_bin_ver_offset = priv->fw_need_write_size - NVT_SIZE_4KB;
  nvt_fw_bin_ver_bar_offset = nvt_fw_bin_ver_offset + 1;

  /* check if FW version add FW version bar equals 0xFF */

  if ((data[nvt_fw_bin_ver_offset]) +
       (data[nvt_fw_bin_ver_bar_offset]) != 0xff)
    {
      ierr("ERROR: bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
      ierr("ERROR: FW_VER=0x%02X, FW_VER_BAR=0x%02X\n",
           data[nvt_fw_bin_ver_offset],
           data[nvt_fw_bin_ver_bar_offset]);
      return -1;
    }

  return 0;
}

/***************************************************************************
 *Description:
 *    Novatek touchscreen update firmware function.
 *
 *return:
 *     Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_update_firmware(FAR struct nt38350_dev_s *priv,
                               uint8_t *data)
{
  int32_t ret;

  /* ---Stop CRC check to prevent IC auto reboot--- */

  nvt_stop_crc_reboot(priv);

  /* Step 1 : initial bootloader */

  ret = nvt_init_bootloader(priv);
  if (ret)
    {
      return ret;
    }

  /* Step 2 : Resume PD */

  ret = nvt_resume_pd(priv);
  if (ret)
    {
      return ret;
    }

  /* Step 3 : Erase */

  ret = nvt_erase_flash(priv);
  if (ret)
    {
      return ret;
    }

  /* Step 4 : Program */

  ret = nvt_write_flash(priv, data);
  if (ret)
    {
      return ret;
    }

  /* Step 5 : Verify */

  ret = nvt_verify_flash(priv, data);
  if (ret)
    {
      return ret;
    }

  /* Step 6 : Bootloader Reset */

  nvt_bootloader_reset(priv);
  nvt_check_fw_reset_state(priv, RESET_STATE_INIT);
  nvt_get_fw_info(priv);

  return ret;
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen check firmware version function.
 *
 *return:
 *     Executive outcomes. 0---need update. 1---need not
 *     update.
 ***************************************************************************/

static int nvt_check_fw_ver(FAR struct nt38350_dev_s *priv,
           uint8_t *data)
{
  int32_t ret;
  size_t nvt_fw_bin_ver_offset = priv->fw_need_write_size - NVT_SIZE_4KB;
  size_t nvt_fw_bin_ver_bar_offset = nvt_fw_bin_ver_offset + 1;
  uint8_t buf[16] =
  {
    0
  };

  /* write i2c index to EVENT BUF ADDR */

  ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS,
                     priv->mmap->event_buf_addr | EVENT_MAP_FWINFO);
  if (ret < 0)
    {
      ierr("ERROR: i2c write error!(%d)\n", ret);
      return ret;
    }

  /* read Firmware Version */

  buf[0] = EVENT_MAP_FWINFO;
  buf[1] = 0x00;
  buf[2] = 0x00;
  ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
  if (ret < 0)
    {
      ierr("ERROR: i2c read error!(%d)\n", ret);
      return ret;
    }

#if NVT_DEBUG
  iinfo("IC FW Ver = 0x%02X, FW Ver Bar = 0x%02X\n", buf[1], buf[2]);
  iinfo("Bin FW Ver = 0x%02X, FW ver Bar = 0x%02X\n",
        data[nvt_fw_bin_ver_offset],
        data[nvt_fw_bin_ver_bar_offset]);
#endif

  /* check IC FW_VER + FW_VER_BAR equals 0xFF or not,
   * need to update if not
   */

  if ((buf[1] + buf[2]) != 0xff)
    {
      ierr("ERROR: IC FW_VER + FW_VER_BAR not equals to 0xFF!\n");
      return 0;
    }

  /* compare IC and binary FW version */

  if (buf[1] > data[nvt_fw_bin_ver_offset])
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/***************************************************************************
 *Description:
 *     Novatek touchscreen check flash end flag function.
 *
 *return:
 *     Executive outcomes. 0---succeed. 1,negative---failed.
 ***************************************************************************/

static int nvt_check_flash_end_flag(FAR struct nt38350_dev_s *priv)
{
  int32_t ret;
  size_t  nvt_flash_end_flag_addr = priv->fw_need_write_size -
                                    NVT_FLASH_END_FLAG_LEN;
  uint8_t buf[8] =
  {
    0
  };

  uint8_t nvt_end_flag[NVT_FLASH_END_FLAG_LEN + 1] =
  {
    0
  };

  /* Step 1 : initial bootloader */

  ret = nvt_init_bootloader(priv);
  if (ret)
    {
      return ret;
    }

  /* Step 2 : Resume PD */

  ret = nvt_resume_pd(priv);
  if (ret)
    {
      return ret;
    }

  /* Step 3 : unlock */

  buf[0] = 0x00;
  buf[1] = 0x35;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: write unlock error!!(%d)\n", ret);
      return ret;
    }

  usleep(NVT_DELAY_10MS);

  /* Step 4 : Flash Read Command */

  buf[0] = 0x00;
  buf[1] = 0x03;
  buf[2] = (nvt_flash_end_flag_addr >> 16) & 0xff; /* Addr_H */
  buf[3] = (nvt_flash_end_flag_addr >> 8) & 0xff;  /* Addr_M */
  buf[4] = nvt_flash_end_flag_addr & 0xff;         /* Addr_L */
  buf[5] = (NVT_FLASH_END_FLAG_LEN >> 8) & 0xff;   /* Len_H  */
  buf[6] = NVT_FLASH_END_FLAG_LEN & 0xff;          /* Len_L */
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 7);
  if (ret < 0)
    {
      ierr("ERROR: write Read Command error!!(%d)\n", ret);
      return ret;
    }

  usleep(NVT_DELAY_10MS);

  /* Check 0xAA (Read Command) */

  buf[0] = 0x00;
  buf[1] = 0x00;
  ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: Check 0xaa (Read Command) error!!(%d)\n", ret);
      return ret;
    }

  if (buf[1] != 0xaa)
    {
      ierr("ERROR: Check 0xaa (Read Command) error!! status=0x%02X\n",
           buf[1]);
      return -1;
    }

  usleep(NVT_DELAY_10MS);

  /* Step 5 : Read Flash Data */

  ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS,
               priv->mmap->read_flash_checksum_addr);
  if (ret < 0)
    {
      ierr("ERROR: change index error!! (%d)\n", ret);
      return ret;
    }

  usleep(NVT_DELAY_10MS);

  /*  Read Back */

  buf[0] = priv->mmap->read_flash_checksum_addr & 0xff;
  ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 6);
  if (ret < 0)
    {
      ierr("ERROR: Read Back error!! (%d)\n", ret);
      return ret;
    }

  /* buf[3:5] => NVT End Flag */

  strncpy(nvt_end_flag, &buf[3], NVT_FLASH_END_FLAG_LEN);
#if NVT_DEBUG
  iinfo("nvt_end_flag=%s (%02X %02X %02X)\n",
        nvt_end_flag, buf[3], buf[4], buf[5]);
#endif

  if (strncmp(nvt_end_flag, "NVT", NVT_FLASH_END_FLAG_LEN) == 0)
    {
      return 0;
    }
  else
    {
      ierr("ERROR: \"NVT\" end flag not found!\n");
      return 1;
    }
}

/***************************************************************************
 *Description:
 *       Novatek touchscreen update firmware when booting
 *       function.
 *
 *return:
 *       n.a.
 ***************************************************************************/

static int nvt_boot_update_firmware(FAR struct nt38350_dev_s *priv)
{
  int32_t ret;
  irqstate_t flags;
  uint8_t *fw_data = NULL;

  ret = get_nvt_fw_size(priv);
  if (ret)
    {
      ierr("ERROR: Get nvt firmware size failed\n");
      goto err;
    }

  fw_data = (uint8_t *)kmm_zalloc(priv->fw_size);
  if (fw_data == NULL)
    {
      ierr("ERROR: Failed to malloc fw_data\n");
      goto err;
    }

  ret = get_nvt_fw_content(priv, fw_data);
  if (ret)
    {
      ierr("ERROR: Failed to get fimware content\n");
      goto err;
    }

  ret = nvt_update_firmware_request(priv, fw_data);
  if (ret)
    {
      ierr("ERROR: nvt_update_firmware_request failed. (%d)\n", ret);
      goto err;
    }

  flags = enter_critical_section();

  nvt_sw_reset_idle(priv);

  ret = nvt_check_checksum(priv, fw_data);

  /* read firmware checksum failed */

  if (ret < 0)
    {
      iinfo("read firmware checksum failed\n");
      nvt_update_firmware(priv, fw_data);
    }
  else if ((ret == 0) && (nvt_check_fw_ver(priv, fw_data) == 0))
    {
  /* (fw checksum not match) && (bin fw version >= ic fw version) */

      iinfo("firmware version not match\n");
      ret = nvt_update_firmware(priv, fw_data);
    }
  else if (nvt_check_flash_end_flag(priv))
    {
      iinfo("check flash end flag failed\n");
      ret = nvt_update_firmware(priv, fw_data);
    }
  else
    {
      nvt_bootloader_reset(priv);
      ret = nvt_check_fw_reset_state(priv, RESET_STATE_INIT);
      if (ret)
        {
          ierr("ERROR: check fw reset state failed\n");
          ret = nvt_update_firmware(priv, fw_data);
        }
    }

  leave_critical_section(flags);

  kmm_free(fw_data);
  fw_data = NULL;
  return ret;

err:
  ierr("ERROR: Failed to update firmware\n");
  kmm_free(fw_data);
  fw_data = NULL;
  return ret;
}

/***************************************************************************
 *Description:
 *       Novatek touchscreen get firmware pipe function.
 *
 *return:
 *        Executive outcomes. 0---pipe 0. 1---pipe 1.
 ***************************************************************************/

static uint8_t nvt_get_fw_pipe(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[2] =
  {
    0
  };

  /* ---set xdata index to EVENT BUF ADDR--- */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS,
                priv->mmap->event_buf_addr |
                EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE);

  /* ---read fw status--- */

  buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
  buf[1] = 0x00;
  nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

  return (buf[1] & 0x01);
}

/***************************************************************************
 *Description:
 *       Novatek touchscreen change mode function.
 *
 *return:
 *       n.a.
 ***************************************************************************/

static void nvt_change_mode(FAR struct nt38350_dev_s *priv, uint8_t mode)
{
  uint8_t buf[8] =
  {
    0
  };

  /* ---set xdata index to EVENT BUF ADDR--- */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS,
               priv->mmap->event_buf_addr | EVENT_MAP_HOST_CMD);

  /* ---set mode--- */

  buf[0] = EVENT_MAP_HOST_CMD;
  buf[1] = mode;
  nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

  if (mode == NVT_NORMAL_MODE)
    {
      usleep(NVT_DELAY_20MS);
      buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
      buf[1] = NVT_HANDSHAKING_HOST_READY;
      nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);
      usleep(NVT_DELAY_20MS);
    }
}

/***************************************************************************
 *Description:
 *      Novatek touchscreen read meta data function.
 *
 *return:
 *       n.a.
 ***************************************************************************/

static void nvt_read_mdata(FAR struct nt38350_dev_s *priv,
                           uint32_t xdata_addr,
                           uint32_t xdata_btn_addr,
                           FAR int32_t *data)
{
  int32_t i;
  int32_t data_len;
  int32_t loop_cnt;
  int32_t residual_len;
  uint32_t xdata_addr_tmp = xdata_addr;
  uint8_t buf[NVT_BUS_TRANSFER_LENGTH + 1] =
  {
    0
  };

  /* ---set read data length--- */

  data_len = priv->x_num * priv->y_num * 2;
  loop_cnt = data_len / NVT_BUS_TRANSFER_LENGTH;
  residual_len = data_len % NVT_BUS_TRANSFER_LENGTH;

#if NVT_DEBUG
  iinfo("data_len=%d, loop_cnt=%d, residual_len=%d\n",
        data_len, loop_cnt, residual_len);
#endif

  /* read xdata : step 1 */

  for (i = 0; i < loop_cnt; i++)
    {
      /* ---change xdata index--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS,
                  (xdata_addr + i * NVT_BUS_TRANSFER_LENGTH));

      /* ---read data--- */

      buf[0] = ((xdata_addr + i * NVT_BUS_TRANSFER_LENGTH) & 0xff);
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf,
             NVT_BUS_TRANSFER_LENGTH + 1);
      u8toi16((buf + 1 + i * NVT_BUS_TRANSFER_LENGTH),
               data, NVT_BUS_TRANSFER_LENGTH);
    }

  /* read residual xdata : step2 */

  if (residual_len != 0)
    {
      /* ---change xdata index--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, (xdata_addr_tmp +
                   loop_cnt * NVT_BUS_TRANSFER_LENGTH));

      /* ---read data--- */

      buf[0] = ((xdata_addr_tmp +
                 loop_cnt * NVT_BUS_TRANSFER_LENGTH) & 0xff);
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf,
                       residual_len + 1);
      u8toi16((buf + 1 + loop_cnt * NVT_BUS_TRANSFER_LENGTH),
               data, data_len);
    }

#if NVT_TOUCH_KEY_NUM > 0

  /* read button xdata : step3
   * ---change xdata index---
   */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, xdata_btn_addr);

  /* ---read data--- */

  buf[0] = (xdata_btn_addr & 0xff);
  nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf,
                   (NVT_TOUCH_KEY_NUM * 2 + 1));

  /* ---2bytes-to-1data--- */

  for (i = 0; i < NVT_TOUCH_KEY_NUM; i++)
    {
      data[priv->x_num * priv->y_num + i] =
          (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
    }
#endif

  /* ---set xdata index to EVENT BUF ADDR--- */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS,
             priv->mmap->event_buf_addr);
}

/***************************************************************************
 *Description:
 *      Novatek touchscreen nvt diff get function.
 *
 *return:
 *      Executive outcomes. 0---succeed. negative---failed.
 ***************************************************************************/

static int nvt_diff_get(FAR struct nt38350_dev_s *priv, int32_t *data)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

#if NVT_TOUCH_ESD_PROTECT
  nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

  if (nvt_clear_fw_status(priv))
    {
      ret = -EACCES;
      goto err;
    }

  nvt_change_mode(priv, NVT_TEST_MODE_2);

  if (nvt_check_fw_status(priv))
    {
      ret = -EACCES;
      goto err;
    }

  if (nvt_get_fw_info(priv))
    {
      ret = -EACCES;
      goto err;
    }

  if (nvt_get_fw_pipe(priv) == 0)
    {
      nvt_read_mdata(priv, priv->mmap->raw_pipe0_addr,
           priv->mmap->diff_btn_pipe0_addr, data);
    }
  else
    {
      nvt_read_mdata(priv, priv->mmap->raw_pipe1_addr,
           priv->mmap->diff_btn_pipe1_addr, data);
    }

  nvt_change_mode(priv, NVT_NORMAL_MODE);

  leave_critical_section(flags);
  return 0;

err:
  leave_critical_section(flags);
  return ret;
}

static int nvt_diff_show(FAR struct nt38350_dev_s *priv,
                         FAR struct nvt_diff_s *diff)
{
  int ret;
  int data_len = priv->x_num * priv->y_num;
  int32_t read_bufx[30] =
  {
    0
  };

  diff->x_num = priv->x_num;
  diff->y_num = priv->y_num;

  ret = nvt_diff_get(priv, read_bufx);
  if (ret == 0)
    {
      memcpy(diff->data, read_bufx, data_len);
    }
  else
    {
      ierr("ERROR: nvt diff show failed\n");
      return ret;
    }

  return 0;
}

#ifdef NVT_OFFLINE_LOG
static void nvt_log_data_to_csv(FAR void *arg)
{
  int      ret;
  int32_t  x = 0;
  int32_t  y = 0;
  int32_t  write_ret;
  char     date_buf[64];
  char     *fbufp = NULL;
  struct   file *fp = NULL;
  int32_t  iarrayindex  = 0;
  int32_t  fw_frame_cnt = 0;
  uint32_t output_len   = 0;
  uint32_t input_x1     = 0;
  uint32_t input_x2     = 0;
  uint32_t input_y1     = 0;
  uint32_t input_y2     = 0;
  char     *csv_file_path = "/mnt/data/NVTLogData.csv";
  FAR struct nt38350_dev_s    *priv = (FAR struct nt38350_dev_s *)arg;
  struct tm tm =
  {
  };

  struct timespec ts =
  {
  };

  fbufp = (char *)kmm_zalloc(8192);
  if (!fbufp)
    {
      ierr("ERROR: kmm_zalloc for fbufp failed!\n");
      return;
    }

  /* open csv file */

  fp = open(csv_file_path, O_RDWR | O_CREAT);
  if (fp == NULL || fp < 0)
    {
      ierr("ERROR: open %s failed\n", csv_file_path);
      if (fbufp)
        {
          kmm_free(fbufp);
          fbufp = NULL;
        }
    }

  /* saved header info to csv file */

  ret = read(fp, fbufp, 1);
  if (fbufp[0] != 'L')
    {
      sprintf(fbufp, "LogVer:''2.0'',TLVer:''Driver'',ChipID:''38350'',"
              "FWVer:''%2X'',Xch:''%2d'',Ych:''%2d'',Xresolution:''%4d'',"
              "Yresolution:''%4d'',FingerNum:''2'',ButtonNum:''0'',"
              "PIDVer:''%4X'',Frame_Info:''0''\r\n",
              priv->fw_ver, priv->x_num, priv->y_num,
              priv->abs_x_max, priv->abs_y_max, priv->nvt_pid);
      write_ret = write(fp, fbufp, 192);
    }

  /* Formatted data such as below:
   * Time(hh:mm:ss:ms), fw_frame_cnt, finger1 Coord_X, finger1 Coord_Y,
   * finger2_Coord_X, finger2_CoordY 2D data
   */

  clock_gettime(CLOCK_REALTIME, &ts);

  localtime_r(&ts.tv_sec, &tm);

  ret = strftime(date_buf, 64, "%e/%m/%Y %H:%M:%S", &tm);

  input_x1 = (uint32_t)((priv->point_xdata_temp[1]) << 4) +
                       (uint32_t) ((priv->point_xdata_temp[3]) >> 4);
  input_y1 = (uint32_t)((priv->point_xdata_temp[2]) << 4) +
                       (uint32_t) ((priv->point_xdata_temp[3]) & 0x0f);
  input_x2 = (uint32_t)((priv->point_xdata_temp[7]) << 4) +
                       (uint32_t) ((priv->point_xdata_temp[9]) >> 4);
  input_y2 = (uint32_t)((priv->point_xdata_temp[8]) << 4) +
                       (uint32_t) ((priv->point_xdata_temp[9]) & 0x0f);
  fw_frame_cnt = (int32_t)((priv->point_xdata_temp[89]) +
                       256 * priv->point_xdata_temp[90]);

  sprintf(fbufp, "timestamp: %s,Frame index: %5d ,"
            "RawData: ,%2X,%4X,%5d,%5d,%5d,%5d\r\n",
            date_buf, fw_frame_cnt, priv->fw_ver,
            priv->nvt_pid, input_x1, input_y1, input_x2, input_y2);
  for (y = 0; y < priv->y_num; y++)
    {
      for (x = 0; x < priv->x_num; x++)
        {
          iarrayindex = y * priv->x_num + x;

          /* 2 bytes to 1 byte */

          sprintf(fbufp + 93 + iarrayindex * 7 + y * 2, "%5d, ",
            (int16_t)(priv->point_xdata_temp[NVT_POINT_DATA_LEN +
             iarrayindex * 2] + 256 *
             priv->point_xdata_temp[NVT_POINT_DATA_LEN +
             iarrayindex * 2 + 1]));
        }

      sprintf(fbufp + 93 + (iarrayindex + 1) * 7 + y * 2, "\r\n");
    }

  sprintf(fbufp  + 93 + (iarrayindex + 1) * 7 + (y * 2), "\r\n");

#if NVT_DEBUG
  iinfo("%s", fbufp);
#endif

  /* Saved data to csv file */

  output_len = 93 + priv->y_num * priv->x_num * 7 +
                    priv->y_num * 2 + 2;

  lseek(fp, 0, SEEK_END);

  write_ret = write(fp, fbufp, output_len);

  if (write_ret <= 0)
    {
      ierr("write %s failed\n", csv_file_path);
      if (fp)
        {
          close(fp);
          fp = NULL;
        }

      if (fbufp)
        {
          kmm_free(fbufp);
          fbufp = NULL;
        }

      return;
    }

  if (fp)
    {
      close(fp);
      fp = NULL;
    }

  if (fbufp)
    {
      kmm_free(fbufp);
      fbufp = NULL;
    }
}
#endif

/***************************************************************************
 * Name: nt38350_notify
 ***************************************************************************/

static void nt38350_notify(FAR struct nt38350_dev_s *priv)
{
  int i;

  /* If there are threads waiting on poll() for nt38350 data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_NT38350_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
#if NVT_DEBUG
          iinfo("Report events: %02x\n", fds->revents);
#endif
          nxsem_post(fds->sem);
        }
    }

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the NT38350
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/***************************************************************************
 * Name: nt38350_sample
 ***************************************************************************/

static int nt38350_sample(FAR struct nt38350_dev_s *priv,
                          FAR struct ts_nt38350_sample_s *sample)
{
  irqstate_t flags;
  int ret = - EAGAIN;

  flags = enter_critical_section();

  if (priv->valid)
    {
       memcpy(sample, &priv->sample, sizeof(struct ts_nt38350_sample_s));

      if (sample->contact == CONTACT_UP)
        {
          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
        {
          priv->sample.contact = CONTACT_MOVE;
        }

       priv->valid = false;
       ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/***************************************************************************
 * Name: nt38350_waitsample
 ***************************************************************************/

static int nt38350_waitsample(FAR struct nt38350_dev_s *priv,
                              struct ts_nt38350_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  sched_lock();
  flags = enter_critical_section();

  nxsem_post(&priv->devsem);

  while (nt38350_sample(priv, sample) < 0)
    {
      /* Wait for a change in the nt38350 state */

      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait: %d\n", ret);
          goto errout;
        }
    }

  ret = nxsem_wait(&priv->devsem);

errout:
  leave_critical_section(flags);
  sched_unlock();
  return ret;
}

static void nt38350_data_worker(FAR void *arg)
{
  FAR struct nt38350_dev_s    *priv = (FAR struct nt38350_dev_s *)arg;
  FAR struct nt38350_config_s *config;
  int      i;
  int      ret;
#ifdef NVT_OFFLINE_LOG
  uint8_t  point_data[NVT_POINT_DATA_EXBUF_LEN + 1] =
  {
    0
  };
#else
  uint8_t  point_data[NVT_POINT_DATA_LEN + 1] =
  {
    0
  };
#endif

  uint32_t position = 0;
  uint32_t input_x = 0;
  uint32_t input_y = 0;
  uint32_t input_w = 0;
  uint32_t input_p = 0;
  uint8_t  input_id = 0;

  DEBUGASSERT(priv != NULL);

  config = priv->config;
  DEBUGASSERT(config != NULL);

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->devsem);

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);

#ifdef NVT_OFFLINE_LOG
  ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, point_data,
                  NVT_POINT_DATA_EXBUF_LEN + 1);
#else
  ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, point_data,
                  NVT_POINT_DATA_LEN + 1);
#endif
  if (ret < 0)
    {
      ierr("Point data read failed!\n");
    }

#ifdef NVT_OFFLINE_LOG
  memcpy(priv->point_xdata_temp, (point_data + 1), NVT_POINT_DATA_LEN);
  memcpy((priv->point_xdata_temp + NVT_POINT_DATA_LEN),
         (point_data + 1 + NVT_POINT_DATA_LEN + NVT_POINT_DATA_EXT_LEN),
         NVT_S2D_DATA_LEN);
  memcpy((priv->point_xdata_temp + NVT_POINT_DATA_LEN + NVT_S2D_DATA_LEN),
        (point_data + 1 + NVT_POINT_DATA_LEN + NVT_POINT_DATA_EXT_LEN +
        NVT_S2D_DATA_LEN + NVT_FW_CMD_HANDLE_LEN),
        (NVT_S2D_DATA_EXT_LEN + NVT_FW_FRAME_CNT_LEN));
  work_queue(LPWORK, &priv->nvt_log_wq, nvt_log_data_to_csv, priv, 100);
#endif

  for (i = 0; i < priv->max_touch_num; i++)
    {
      position = 1 + 6 * i;
      input_id = (uint8_t)(point_data[position + 0] >> 3);
      if ((input_id == 0) || (input_id > priv->max_touch_num))
        continue;

      if (((point_data[position] & 0x07) == FINGER_DOWN) ||
          ((point_data[position] & 0x07) == FINGER_MOVE))
        {
          if ((point_data[position] & 0x07) == FINGER_DOWN)
            priv->sample.contact = CONTACT_DOWN;
          if ((point_data[position] & 0x07) == FINGER_MOVE)
            priv->sample.contact = CONTACT_MOVE;
          input_x = (uint32_t)(point_data[position + 1] << 4) +
                    (uint32_t) (point_data[position + 3] >> 4);
          input_y = (uint32_t)(point_data[position + 2] << 4) +
                (uint32_t) (point_data[position + 3] & 0x0f);
#ifdef CONFIG_ROTATION
          priv->sample.x = NVT_TOUCH_DEFAULT_MAX_WIDTH - input_x;
          priv->sample.y = NVT_TOUCH_DEFAULT_MAX_HEIGHT - input_y;
#else
          priv->sample.x = input_x;
          priv->sample.y = input_y;
#endif
          input_w = (uint32_t)(point_data[position + 4]);
          if (input_w == 0)
            input_w = 1;
          priv->sample.width = input_w;
          if (i < 2)
            {
              input_p = (uint32_t)(point_data[position + 5]) +
                        (uint32_t)(point_data[i + 15] << 8);
              if (input_p > NVT_TOUCH_FORCE_NUM)
                  input_p = NVT_TOUCH_FORCE_NUM;
            }
        else
        {
           input_p = (uint32_t)(point_data[position + 5]);
        }

          if (input_p == 0)
            input_p = 1;
          priv->sample.pressure = input_p;
        }
      else if (point_data[position] == FINGER_UP)
        {
          priv->sample.contact = CONTACT_UP;
#ifdef CONFIG_ROTATION
          priv->sample.x = NVT_TOUCH_DEFAULT_MAX_WIDTH - input_x;
          priv->sample.y = NVT_TOUCH_DEFAULT_MAX_HEIGHT - input_y;
#else
          priv->sample.x = input_x;
          priv->sample.y = input_y;
#endif
          priv->sample.width = input_w;
          priv->sample.pressure = input_p;
        }
      else
        {
          priv->sample.contact = CONTACT_NONE;
          input_x = 0;
          input_y = 0;
          input_w = 0;
          input_p = 0;
        }
    }

  priv->sample.valid = true;
  priv->sample.id = priv->id;
  priv->valid = true;

  nt38350_notify(priv);

  config->enable(config, true);
  nxsem_post(&priv->devsem);
}

/***************************************************************************
 * Name: nt38350_data_interrupt
 ***************************************************************************/

static int nt38350_data_interrupt(FAR struct ioexpander_dev_s *dev,
                                  ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct nt38350_dev_s    *priv = arg;
  FAR struct nt38350_config_s *config;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Which nt38350 device caused the interrupt? */

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Transfer processing to the worker thread.  Since nt38350 interrupts
   * are disabled while the work is pending, no special action should be
   * required to protected the work queue.
   */

  ret = work_queue(HPWORK, &priv->work, nt38350_data_worker, priv, 0);
  if (ret != OK)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

/***************************************************************************
 * Name: nt38350_open
 ***************************************************************************/

static int nt38350_open(FAR struct file *filep)
{
#ifdef CONFIG_NT38350_REFCNT
  FAR struct inode         *inode;
  FAR struct nt38350_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct nt38350_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  nxsem_post(&priv->devsem);
  return ret;
#else
  return OK;
#endif
}

/***************************************************************************
 * Name: nt38350_close
 ***************************************************************************/

static int nt38350_close(FAR struct file *filep)
{
#ifdef CONFIG_NT38350_REFCNT
  FAR struct inode         *inode;
  FAR struct nt38350_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct nt38350_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  nxsem_post(&priv->devsem);
#endif
  return OK;
}

/***************************************************************************
 * Name: nt38350_read
 ***************************************************************************/

static ssize_t nt38350_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode           *inode;
  FAR struct nt38350_dev_s   *priv;
  FAR struct touch_sample_s  *report;
  struct ts_nt38350_sample_s sample;
  int                        ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct nt38350_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      ierr("ERROR: Unsupported read size: %d\n", len);
      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait: %d\n", ret);
      return ret;
    }

  /* Try to read sample data. */

  ret = nt38350_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = nt38350_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: nt38350_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled nt38350 data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = sample.id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;
  report->point[0].w         = sample.width;
  report->point[0].pressure  = sample.pressure;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up.  Is the positional data valid?  This is important to
       * know because the release will be sent to the window based on its
       * last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }

#if NVT_DEBUG
  iinfo("  id:      %d\n",   report->point[0].id);
  iinfo("  flags:   %02x\n", report->point[0].flags);
  iinfo("  x:       %d\n",   report->point[0].x);
  iinfo("  y:       %d\n",   report->point[0].y);
  iinfo("  w:       %d\n",   report->point[0].w);
  iinfo("  p:       %d\n",   report->point[0].pressure);
#endif
  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/***************************************************************************
 * Name: nt38350_ioctl
 ***************************************************************************/

static int nt38350_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct nt38350_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct nt38350_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETCALIB:  /* arg: Pointer to int calibration value */
        {
          priv->config->rxplate = (uint16_t)arg;
        }
        break;

      case TSIOC_GETCALIB:  /* arg: Pointer to int calibration value */
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          *ptr = priv->config->rxplate;
        }
        break;

      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          priv->config->frequency = *ptr;
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          *ptr = priv->config->frequency;
        }
        break;

      case TSIOC_GETFWVERSION:  /* arg: Pointer to uint32_t fw version value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          *ptr = (uint32_t)priv->fw_ver;
        }
        break;

      case TSIOC_GETNVTDIFF:  /* arg: Pointer to struct nvt_diff_s */
        {
          FAR struct nvt_diff_s *ptr = (FAR struct nvt_diff_s *)arg;
          nvt_diff_show(priv, ptr);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/***************************************************************************
 * Name: nt38350_poll
 ***************************************************************************/

static int nt38350_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct nt38350_dev_s *priv;
  FAR struct inode *inode;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct nt38350_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_NT38350_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_NT38350_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->valid)
        {
          nt38350_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int nt38350_register(FAR struct nt38350_config_s *config,
                     const char *devname)
{
  FAR struct nt38350_dev_s *priv;
  int ret;

#if NVT_DEBUG
  iinfo("dev: %p devname: %s \n", dev, devname);
#endif

  DEBUGASSERT(config != NULL && devname != NULL);
  DEBUGASSERT(config->attach != NULL && config->enable  != NULL &&
                config->clear  != NULL);

  priv = kmm_zalloc(sizeof(struct nt38350_dev_s));
  if (priv == NULL)
    {
      ierr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize the nt38350 device driver instance */

  priv->config = config;            /* Save the board configuration */

  priv->fw_path = CONFIG_NT38350_FW_PATH; /* Set up firmware path */

#if NVT_DEBUG
  iinfo("RST %d IRQ %d %d\n", priv->config->gpio_rst_pin,
        priv->config->gpio_irq_pin, __LINE__);
#endif

  nxsem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  nxsem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* The event wait semaphore is used for signaling and, hence, should not
   * have priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Hardware Initial */

  usleep(NVT_DELAY_10MS);

  ret = nvt_ts_check_chip_ver_trim(priv, NVT_CHIP_VER_TRIM_ADDR);
  if (ret)
    {
      ret = nvt_ts_check_chip_ver_trim(priv, NVT_CHIP_VER_TRIM_OLD_ADDR);
      if (ret)
        {
          ierr("ERROR: Chip is not identified\n");
          ret = -ENODEV;
          goto errout_with_priv;
        }
    }

  nvt_bootloader_reset(priv);
  nvt_check_fw_reset_state(priv, RESET_STATE_INIT);
  nvt_get_fw_info(priv);

  priv->max_touch_num = NVT_TOUCH_MAX_FINGER_NUM;

#ifdef CONFIG_NT38350_NEED_UPGRADE_FW
  ret = nvt_boot_update_firmware(priv);
  if (ret)
    {
      ierr("ERROR: Failed to nvt_boot_update_firmware\n");
      ret = -ENODEV;
      goto errout_with_priv;
    }
#endif

  ret = config->attach(config, nt38350_data_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      ret = -ENODEV;
      goto errout_with_priv;
    }

  config->enable(config, true);

  ret = register_driver(devname, &g_nt38350_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      config->detach(config, nt38350_data_interrupt);
      ret = -ENODEV;
      goto errout_with_priv;
    }

  /* And return success (?) */

  return OK;
errout_with_priv:
  kmm_free(priv);
  nxsem_destroy(&priv->devsem);
  return ret;
}
