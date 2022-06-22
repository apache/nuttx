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
#include <sys/stat.h>
#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/power/pm.h>

#include <nuttx/signal.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/nt38350.h>
#ifdef CONFIG_PM
#include <arch/board/pm_domain.h>
#endif
/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Configuration ***********************************************************/

#ifndef CONFIG_NT38350_TOUCHSCREEN_BUFF_NUMS
#  define CONFIG_NT38350_TOUCHSCREEN_BUFF_NUMS 2
#endif

#ifndef CONFIG_NVT_TOUCH_DEFAULT_WIDTH
# define CONFIG_NVT_TOUCH_DEFAULT_WIDTH 480
#endif

#ifndef CONFIG_NVT_TOUCH_DEFAULT_HEIGHT
# define CONFIG_NVT_TOUCH_DEFAULT_HEIGHT 480
#endif

#define NVT_DEV_FORMAT               "/dev/input%d"
#define NVT_DEV_NAMELEN              16
#define NVT_IIC_RETRY_NUM            2
#define NVT_POINT_DATA_LEN           17
#define NVT_I2C_BLDR_ADDRESS         0x01
#define NVT_I2C_FW_ADDRESS           0x01
#define NVT_I2C_HW_ADDRESS           0x62
#define NVT_TOUCH_DEFAULT_MAX_WIDTH  CONFIG_NVT_TOUCH_DEFAULT_WIDTH
#define NVT_TOUCH_DEFAULT_MAX_HEIGHT CONFIG_NVT_TOUCH_DEFAULT_HEIGHT
#define NVT_TOUCH_MAX_FINGER_NUM     1
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
#define NVT_DELAY_35MS               35000
#define NVT_DELAY_80MS               80000
#define NVT_DELAY_83MS               83000
#define NVT_DELAY_100MS              100000

#define NVT_NORMAL_MODE              0x00
#define NVT_TEST_MODE_2              0x22
#define NVT_HANDSHAKING_HOST_READY   0xbb
#define NVT_XDATA_SECTOR_SIZE        256

#define NVT_RESULT_FOLDER            "data/tp"
#define NVT_OFFLOG_FOLDER            "data/log/tp"
#define NVT_INIT_RETRY_TIMES         5

#ifdef  CONFIG_NVT_OFFLINE_LOG
#define NVT_DEBUGLOG_TYPE            0x02
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

#ifdef CONFIG_WAKEUP_GESTURE
#define GESTURE_WORD_C               12
#define GESTURE_WORD_W               13
#define GESTURE_WORD_V               14
#define GESTURE_DOUBLE_CLICK         15
#define GESTURE_WORD_Z               16
#define GESTURE_WORD_M               17
#define GESTURE_WORD_O               18
#define GESTURE_WORD_E               19
#define GESTURE_WORD_S               20
#define GESTURE_SLIDE_UP             21
#define GESTURE_SLIDE_DOWN           22
#define GESTURE_SLIDE_LEFT           23
#define GESTURE_SLIDE_RIGHT          24
#define GESTURE_PALM                 25

#define DATA_PROTOCOL                30 /* customized gesture id */

/* function page definition */
#define FUNCPAGE_GESTURE             1
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
  CONTACT_PALM,
  CONTACT_UP
};

enum nvt_finger_contact_e
{
  FINGER_DOWN = 0x01,
  FINGER_MOVE = 0x02,
  FINGER_UP   = 0xff
};

enum nvt_pm_e
{
  NVT_PM_DPSTDBY = 0x11,  /* Deep Standby mode */
  NVT_PM_PD      = 0x12,  /* Power down mode, not recommend */
  NVT_PM_FDM     = 0x13   /* Finger detect mode */
};

enum nvt_icpower_state_e
{
  NVT_POWER_ON  = 0x01,
  NVT_POWER_OFF = 0x02
};

struct nvt_power_resume_s
{
  uint8_t times;
  uint8_t icpower_state;
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

/* This structure describes the state of one nt38350 driver instance */

struct nt38350_dev_s
{
  uint8_t                       id;                /* Current touch point ID */
  uint32_t                      frequency;

  FAR struct nt38350_config_s   *config;           /* Board configuration data */
  FAR struct                    i2c_master_s *i2c; /* Saved I2C driver instance */
  struct work_s                 work;              /* Supports the interrupt handling "bottom half" */
  struct ts_nt38350_sample_s    sample;            /* Last sampled touch point data */
  struct ts_nt38350_sample_s    old_sample;        /* Old sampled touch point data */
  struct touch_lowerhalf_s      lower;             /* touchscreen device lowerhalf instance */
#ifdef CONFIG_PM
  struct pm_callback_s          pm;
  enum pm_state_e               current_state;
  int8_t                        enable_fdm;
  struct nvt_power_resume_s     nvt_pwr_resume;     /* The IC globle power state, the IC power controled by display */
#endif

  /* Touch info */

  char                          *fw_path;
  uint8_t                       fw_ver;
  uint8_t                       fw_type;
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
  uint8_t                       touch_awake;
  bool                          idle_mode;
#ifdef CONFIG_NVT_OFFLINE_LOG
  uint8_t point_xdata_temp[NVT_POINT_DATA_EXBUF_LEN];
#endif
#ifdef CONFIG_NT38350_NEED_UPGRADE_FW
  struct work_s                 fwwork;
#endif
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

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

static int nt38350_control(FAR struct touch_lowerhalf_s *lower,
                           int cmd, unsigned long arg);

/***************************************************************************
 * Private Data
 ***************************************************************************/

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

static const uint8_t lighting_palm[3] =
{
  0xf0, 0x04, 0x01
};

#ifdef CONFIG_NVT_TOUCH_MP
static const uint8_t AIN_X[10] =
{
  0, 1, 2, 3, 4, 5
};

static const uint8_t AIN_Y[10] =
{
  0, 1, 2, 3, 4, 5
};

static const int32_t ps_config_lmt_short_rawdata_p[10 * 10] =
{
  65535,  14008,  14008,  14008,  14008,  65535,
  14008,  14008,  14008,  14008,  14008,  14008,
  14008,  14008,  14008,  14008,  14008,  14008,
  14008,  14008,  14008,  14008,  14008,  14008,
  14008,  14008,  14008,  14008,  14008,  14008,
  65535,  14008,  14008,  14008,  14008,  65535
};

static const int32_t ps_config_lmt_short_rawdata_n[10 * 10] =
{
  -65535,  10100,  10100,  10100,  10100,  -65535,
  10100,   10100,  10100,  10100,  10100,  10100,
  10100,   10100,  10100,  10100,  10100,  10100,
  10100,   10100,  10100,  10100,  10100,  10100,
  10100,   10100,  10100,  10100,  10100,  10100,
  -65535,  10100,  10100,  10100,  10100,  -65535
};

static const int32_t ps_config_lmt_open_rawdata_p[10 * 10] =
{
  65535,  1572,  2114,  2116,  1576,  65535,
  1439,   2052,  2209,  2202,  2044,  1456,
  1976,   2066,  2171,  2172,  2066,  1992,
  1964,   2044,  2156,  2137,  2065,  1961,
  1390,   2000,  2105,  2097,  2003,  1365,
  65535,  1451,  1975,  1976,  1465,  65535
};

static const int32_t ps_config_lmt_open_rawdata_n[10 * 10] =
{
  -65535,  673,  906,  907,  675,  -65535,
  616,     879,  946,  943,  876,  624,
  847,     885,  930,  931,  885,  853,
  841,     876,  924,  916,  885,  840,
  595,     857,  902,  898,  858,  585,
  -65535,  622,  846,  847,  628,  -65535
};

static const int32_t ps_config_lmt_fw_rawdata_p[10 * 10] =
{
  65535,  3276,  3351,  3458,  3354,  65535,
  3344,   3288,  3339,  3434,  3332,  3354,
  3376,   3334,  3360,  3392,  3367,  3397,
  3361,   3343,  3358,  3256,  3357,  3364,
  3397,   3305,  3304,  3320,  3340,  3308,
  65535,  3375,  3259,  3379,  3378,  65535
};

static const int32_t ps_config_lmt_fw_rawdata_n[10 * 10] =
{
  -65535,  1404,  1436,  1482,  1437,  -65535,
  1433,    1409,  1431,  1471,  1428,  1437,
  1447,    1429,  1440,  1453,  1443,  1456,
  1440,    1432,  1439,  1395,  1438,  1441,
  1456,    1416,  1416,  1423,  1431,  1417,
  -65535,  1446,  1396,  1448,  1447,  -65535
};

static const int32_t ps_config_lmt_fw_cc_p[10 * 10] =
{
  65535,  125,  166,  166,  125,  65535,
  114,    164,  175,  175,  162,  115,
  157,    162,  172,  172,  162,  157,
  155,    162,  169,  169,  162,  154,
  109,    158,  165,  166,  158,  109,
  65535,  115,  154,  154,  114,  65535
};

static const int32_t ps_config_lmt_fw_cc_n[10 * 10] =
{
  -65535,  56,  75,  75,  56,  -65535,
  51,      73,  78,  78,  73,  52,
  70,      73,  77,  77,  73,  70,
  70,      73,  76,  76,  73,  69,
  48,      71,  74,  75,  71,  48,
  -65535,  52,  69,  69,  51,  -65535
};

static const int32_t ps_config_lmt_fw_diff_p[10 * 10] =
{
  65535,  65,  65,  65,  65,  65535,
  65,     65,  65,  65,  65,  65,
  65,     65,  65,  65,  65,  65,
  65,     65,  65,  65,  65,  65,
  65,     65,  65,  65,  65,  65,
  65535,  65,  65,  65,  65,  65535
};

static const int32_t ps_config_lmt_fw_diff_n[10 *10] =
{
  -65535,  -65,  -65,  -65,  -65,  -65535,
  -65,     -65,  -65,  -65,  -65,  -65,
  -65,     -65,  -65,  -65,  -65,  -65,
  -65,     -65,  -65,  -65,  -65,  -65,
  -65,     -65,  -65,  -65,  -65,  -65,
  -65535,  -65,  -65,  -65,  -65,  -65535
};
#endif

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

  for (retries = 0; retries < NVT_IIC_RETRY_NUM; retries++)
    {
      ret = I2C_TRANSFER(dev, msg, 2);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == NVT_IIC_RETRY_NUM -1)
            {
              break;
            }

          I2C_RESET(dev);
          iwarn("reset I2C, retries %d \n", retries);
#endif
        }
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
  int retries;

  /* Setup for the transfer */

  msg[0].frequency = config->frequency;
  msg[0].addr      = address;
  msg[0].flags     = 0;
  msg[0].buffer    = (uint8_t *)buffer;
  msg[0].length    = length;

  /* Then perform the transfer. */

  for (retries = 0; retries < NVT_IIC_RETRY_NUM; retries++)
    {
      ret = I2C_TRANSFER(dev, msg, 1);
      if (ret >= 0)
        {
          break;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == NVT_IIC_RETRY_NUM -1)
            {
              break;
            }

          I2C_RESET(dev);
          iwarn("reset I2C , retries %d\n",  retries);
#endif
        }
    }

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
  if (ret < 0)
    {
      ierr("ERROR: Failed to read reg: %d\n", ret);
      return ret;
    }

  return ret;
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
  if (ret < 0)
    {
      ierr("ERROR: Failed to write reg: %d\n", ret);
      return ret;
    }

  return OK;
}

static void u8toi16(const uint8_t *stream, int32_t *data, uint32_t len)
{
  int i;

  for (i = 0; i < len / 2; i++)
    {
      data[i] = (int16_t)(stream[i * 2] +
                 256 * stream[i * 2 + 1]);
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
  if (ret < 0)
    {
      ierr("ERROR: Failed to reset bootloader\n");
    }

  usleep(NVT_DELAY_35MS);

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
      if (ret < 0)
        {
          ierr("ERROR: Failed to read register\n");
        }

      if (buf[1] == 0x00)
        {
          break;
        }

      usleep(NVT_DELAY_10MS);
    }

  if (i >= retry)
    {
      ierr("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
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
      if (ret < 0)
        {
          ierr("ERROR: Failed to read register\n");
        }

      if ((buf[1] & 0xf0) == 0xa0)
         break;
      usleep(NVT_DELAY_10MS);
    }

  if (i >= retry)
    {
      ierr("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
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
  int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 12 : 50;

  while (1)
    {
      buf[0] = EVENT_MAP_RESET_COMPLETE;
      buf[1] = 0x00;
      ret = nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 6);

      if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX))
        {
          ret = OK;
          break;
        }

      retry++;
      if (retry > retry_max)
        {
          ierr("buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
               buf[1], buf[2], buf[3], buf[4], buf[5]);
          ret = -1;
          break;
        }

      usleep(NVT_DELAY_10MS);
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

#ifdef CONFIG_NVT_DEBUG
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
  priv->fw_type = buf[14];

  iwarn("fw_ver: 0x%02x, fw_type: 0x%02x\n", priv->fw_ver, priv->fw_type);

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
 *   none
 ***************************************************************************/

static void nvt_stop_crc_reboot(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[8];
  int32_t retry;

  /* ---change I2C index to prevent geting 0xFF, but not 0xFC--- */

  nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x1f64e);

  /* ---read to check if buf is 0xFC which means IC is in CRC reboot */

  buf[0] = 0x4e;
  nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 4);

  if ((buf[1] == 0xfc) ||
             ((buf[1] == 0xff) && (buf[2] == 0xff) && (buf[3] == 0xff)))
    {
      /* IC is in CRC fail reboot loop, need to be stopped */

      for (retry = 5; retry > 0; retry--)
        {
          /* ---write i2c cmds to reset idle : 1st--- */

          buf[0] = 0x00;
          buf[1] = 0xa5;
          nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);

          /* ---write i2c cmds to reset idle : 2ed--- */

          buf[0] = 0x00;
          buf[1] = 0xa5;
          nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
          usleep(NVT_DELAY_1MS);

          /* ---clear CRC_ERR_FLAG--- */

          nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x3f135);

          buf[0] = 0x35;
          buf[1] = 0xa5;
          nt38350_write_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 2);

          /* ---check CRC_ERR_FLAG--- */

          nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, 0x3f135);

          buf[0] = 0x35;
          buf[1] = 0x00;
          nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 2);
          if (buf[1] == 0xa5)
              break;
        }

      if (retry == 0)
          ierr("ERROR: CRC auto reboot is not able "
               "to be stopped buf[1]=0x%02x\n", buf[1]);
    }
}

/***************************************************************************
 *Description:
 *   Novatek touchscreen check chip version trim function.
 *
 *return:
 *   Executive outcomes. 0---NVT IC. other---access fail
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

  /* if bootloader reset fail, i2c bus maybe is broken, return fail */

  ret = nvt_bootloader_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  for (retry = 5; retry > 0; retry--)
    {
      nvt_sw_reset_idle(priv);
      buf[0] = 0x00;
      buf[1] = 0x35;
      nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
      usleep(NVT_DELAY_10MS);

      nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, chip_ver_trim_addr);

      buf[0] = chip_ver_trim_addr & 0xff;
      buf[1] = 0x00;
      buf[2] = 0x00;
      buf[3] = 0x00;
      buf[4] = 0x00;
      buf[5] = 0x00;
      buf[6] = 0x00;
      nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 7);

      /* Get Touch IC ID */

      iinfo("buf[1]=0x%02x, buf[2]=0x%02x, buf[3]=0x%02x,"
            "buf[4]=0x%02x, buf[5]=0x%02x,buf[6]=0x%02x\n",
            buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

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
              iinfo("This NVT touch IC\n");
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
  struct file file;
  int ret;

  ret = file_open(&file, fw_path, O_RDONLY | O_BINARY);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_seek(&file, 0, SEEK_END);
  if (ret < 0)
    {
      goto out;
    }

  fw_size = file_seek(&file, 0, SEEK_CUR);
  priv->fw_size = fw_size;
  ret = 0;
out:
  file_close(&file);
  return ret;
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
  struct file file;
  uint32_t fw_size = priv->fw_size;

  ret = file_open(&file, fw_path, O_RDONLY | O_BINARY);
  if (ret < 0)
    {
      return ret;
    }

  file_seek(&file, 0, SEEK_SET);
  ret = file_read(&file, fw_data, fw_size);
  if (ret != fw_size)
    {
      ierr("ERROR: Failed get firmware content\n");
      ret = ret < 0 ? ret : -ENODATA;
      goto out;
    }

  ret = 0;
out:
  file_close(&file);
  return ret;
}

static int nvt_get_fw_need_write_size(FAR struct nt38350_dev_s *priv,
                                      FAR uint8_t *data)
{
  int i;
  uint32_t total_sectors_to_check;

  total_sectors_to_check = priv->fw_size / NVT_FLASH_SECTOR_SIZE;
#ifdef CONFIG_NVT_DEBUG
  iinfo("total_sectors_to_check %d\n", total_sectors_to_check);
#endif

  for (i = total_sectors_to_check; i > 0; i--)
    {
      /* check if there is end flag "NVT" at the end of this sector */

      if (strncmp((char *)&data[i * NVT_FLASH_SECTOR_SIZE -
          NVT_FLASH_END_FLAG_LEN], "NVT", NVT_FLASH_END_FLAG_LEN) == 0)
        {
          priv->fw_need_write_size = i * NVT_FLASH_SECTOR_SIZE;
#ifdef CONFIG_NVT_DEBUG
          iinfo("fw_need_write_size = %zu(0x%zx)\n",
                priv->fw_need_write_size,
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
      ierr("ERROR: Inittial Flash Block error!!(%ld)\n", ret);
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
               "error.(%ld)\n", ret);
          return ret;
        }

      usleep(NVT_DELAY_10MS);

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
      ierr("ERROR: Write Enable error!!(%ld)\n", ret);
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
          ierr("ERROR: Check 0xAA (Resume Command) error!!(%ld)\n", ret);
          return ret;
        }

      usleep(NVT_DELAY_10MS);

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
           " error!!(%ld)\n", ret);
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
               " error!!(%ld)\n", ret);
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
      ierr("ERROR: Write Status Register error!!(%ld)\n", ret);
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
               " error!!(%ld)\n", ret);
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
               "error!!(%ld)\n", ret);
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
               "Write Status Register) error!!(%ld)\n", ret);
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
               "retry=%ld\n", buf[1], buf[2], retry);
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
          ierr("ERROR: Write Enable error!!(%ld,%ld)\n", ret, i);
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
              ierr("ERROR: Check 0xAA (Write Enable) error!!(%ld,%ld)\n",
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
          ierr("ERROR: Sector Erase error!!(%ld,%ld)\n", ret, i);
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
              ierr("ERROR: Check 0xAA (Sector Erase) error!!(%ld,%ld)\n",
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
                   "buf[1]=0x%02X,retry=%ld\n", buf[1], retry);
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
              ierr("ERROR: Read Status error!!(%ld,%ld)\n", ret, i);
              return ret;
            }

          /* Check 0xAA (Read Status) */

          buf[0] = 0x00;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Read Status) error!!(%ld,%ld)\n",
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
                   "buf[1]=0x%02X, buf[2]=0x%02X,retry=%ld\n",
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
      ierr("ERROR: change I2C buffer index error!!(%ld)\n", ret);
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
          ierr("ERROR: Write Enable error!!(%ld)\n", ret);
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
              ierr("ERROR: Check 0xAA (Write Enable) error!!(%ld,%ld)\n",
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
              ierr("ERROR: Write Page error!!(%ld), j=%ld\n", ret, j);
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
          ierr("ERROR: Page Program error!!(%ld), i=%ld\n", ret, i);
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
              ierr("ERROR: Page Program error!!(%ld)\n", ret);
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
                   "buf[1]=0x%02X,retry=%ld\n", buf[1], retry);
              return -1;
            }
        }

      if (buf[1] == 0xea)
        {
          ierr("ERROR: Page Program error!! i=%ld  %d\n", i, __LINE__);
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
              ierr("ERROR: Read Status error!!(%ld)\n", ret);
              return ret;
            }

          /* Check 0xAA (Read Status) */

          buf[0] = 0x00;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Check 0xAA (Read Status) error!!(%ld)\n", ret);
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
                   "buf[1]=0x%02X, buf[2]=0x%02X,retry=%ld\n",
                    buf[1], buf[2], retry);
              return -1;
            }
        }

      if (buf[1] == 0xea)
        {
          ierr("Page Program error!! i=%ld %d\n", i, __LINE__);
          return -4;
        }

      percent = ((i + 1) * 100) / count;
      if (((percent % 10) == 0) && (percent != previous_percent))
        {
          iwarn("Programming...%2ld%%\n", percent);
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
              ierr("ERROR: Fast Read Command error!!(%ld)\n", ret);
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
                       "error!!(%ld)\n", ret);
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
                        "buf[1]=0x%02X,retry=%ld\n", buf[1], retry);
                  return -1;
                }
            }

          /* Read Checksum (write addr high byte & middle byte) */

          ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, xdata_addr);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum (write addr high byte &"
                   "middle byte)error!(%ld)\n", ret);
              return ret;
            }

          /* Read Checksum */

          buf[0] = (xdata_addr) & 0xff;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum error!!(%ld)\n", ret);
              return ret;
            }

          rd_filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
          if (wr_filechksum[i] != rd_filechksum[i])
            {
              ierr("ERROR: Verify Fail%ld!!\n", i);
              ierr("ERROR: rd_filechksum[%ld]=0x%04X,"
                   "wr_filechksum[%ld]=0x%04X\n",
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
              ierr("ERROR: Fast Read Command error!!(%ld)\n", ret);
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
                       "error!!(%ld)\n", ret);
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
                       "buf[1]=0x%02X,retry=%ld\n", buf[1], retry);
                  return -1;
                }
            }

          /* Read Checksum (write addr high byte & middle byte) */

          ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS, xdata_addr);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum (write addr high byte &"
                "middle byte)error!(%ld)\n", ret);
              return ret;
            }

          /* Read Checksum */

          buf[0] = (xdata_addr) & 0xff;
          buf[1] = 0x00;
          buf[2] = 0x00;
          ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
          if (ret < 0)
            {
              ierr("ERROR: Read Checksum error!!(%ld)\n", ret);
              return ret;
            }

          rd_filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
          if (wr_filechksum[i] != rd_filechksum[i])
            {
              ierr("ERROR: rd_filechksum[%ld]=0x%04X,"
                   "wr_filechksum[%ld]=0x%04X\n",
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
  if (ret < 0)
    {
      return ret;
    }

  /* Step 2 : Resume PD */

  ret = nvt_resume_pd(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 3 : Erase */

  ret = nvt_erase_flash(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 4 : Program */

  ret = nvt_write_flash(priv, data);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 5 : Verify */

  ret = nvt_verify_flash(priv, data);
  if (ret < 0)
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
#ifdef CONFIG_NVT_DEBUG
  size_t nvt_fw_bin_ver_bar_offset = nvt_fw_bin_ver_offset + 1;
#endif
  uint8_t buf[16] =
  {
    0
  };

  /* write i2c index to EVENT BUF ADDR */

  ret = nvt_set_page(priv, NVT_I2C_BLDR_ADDRESS,
                     priv->mmap->event_buf_addr | EVENT_MAP_FWINFO);
  if (ret < 0)
    {
      ierr("ERROR: i2c write error!(%ld)\n", ret);
      return ret;
    }

  /* read Firmware Version */

  buf[0] = EVENT_MAP_FWINFO;
  buf[1] = 0x00;
  buf[2] = 0x00;
  ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 3);
  if (ret < 0)
    {
      ierr("ERROR: i2c read error!(%ld)\n", ret);
      return ret;
    }

#ifdef CONFIG_NVT_DEBUG
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
  if (ret < 0)
    {
      return ret;
    }

  /* Step 2 : Resume PD */

  ret = nvt_resume_pd(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 3 : unlock */

  buf[0] = 0x00;
  buf[1] = 0x35;
  ret = nt38350_write_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: write unlock error!!(%ld)\n", ret);
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
      ierr("ERROR: write Read Command error!!(%ld)\n", ret);
      return ret;
    }

  usleep(NVT_DELAY_10MS);

  /* Check 0xAA (Read Command) */

  buf[0] = 0x00;
  buf[1] = 0x00;
  ret = nt38350_read_reg(priv, NVT_I2C_HW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("ERROR: Check 0xaa (Read Command) error!!(%ld)\n", ret);
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
      ierr("ERROR: change index error!! (%ld)\n", ret);
      return ret;
    }

  usleep(NVT_DELAY_10MS);

  /*  Read Back */

  buf[0] = priv->mmap->read_flash_checksum_addr & 0xff;
  ret = nt38350_read_reg(priv, NVT_I2C_BLDR_ADDRESS, buf, 6);
  if (ret < 0)
    {
      ierr("ERROR: Read Back error!! (%ld)\n", ret);
      return ret;
    }

  /* buf[3:5] => NVT End Flag */

  memcpy(nvt_end_flag, &buf[3], NVT_FLASH_END_FLAG_LEN);
#ifdef CONFIG_NVT_DEBUG
  iinfo("nvt_end_flag=%s (%02X %02X %02X)\n",
        nvt_end_flag, buf[3], buf[4], buf[5]);
#endif

  if (strncmp((char *)nvt_end_flag, "NVT", NVT_FLASH_END_FLAG_LEN) == 0)
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
  if (ret < 0)
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
  if (ret < 0)
    {
      ierr("ERROR: Failed to get fimware content\n");
      goto err;
    }

  ret = nvt_update_firmware_request(priv, fw_data);
  if (ret < 0)
    {
      ierr("ERROR: nvt_update_firmware_request failed. (%ld)\n", ret);
      goto err;
    }

  flags = enter_critical_section();

  nvt_sw_reset_idle(priv);

  ret = nvt_check_checksum(priv, fw_data);

  /* read firmware checksum failed */

  if (ret < 0)
    {
      iwarn("read firmware checksum failed\n");
      nvt_update_firmware(priv, fw_data);
    }
  else if ((ret == 0) && (nvt_check_fw_ver(priv, fw_data) == 0))
    {
  /* (fw checksum not match) && (bin fw version >= ic fw version) */

      iwarn("firmware version not match\n");
      ret = nvt_update_firmware(priv, fw_data);
    }
  else if (nvt_check_flash_end_flag(priv))
    {
      iwarn("check flash end flag failed\n");
      ret = nvt_update_firmware(priv, fw_data);
    }
  else
    {
      nvt_bootloader_reset(priv);
      ret = nvt_check_fw_reset_state(priv, RESET_STATE_INIT);
      if (ret < 0)
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

#ifdef CONFIG_NVT_DEBUG
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
      u8toi16(buf + 1,
              data + i * NVT_BUS_TRANSFER_LENGTH / 2,
              NVT_BUS_TRANSFER_LENGTH);
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
      u8toi16(buf + 1,
              data + loop_cnt * NVT_BUS_TRANSFER_LENGTH / 2, residual_len);
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
      nvt_read_mdata(priv, priv->mmap->diff_pipe0_addr,
           priv->mmap->diff_btn_pipe0_addr, data);
    }
  else
    {
      nvt_read_mdata(priv, priv->mmap->diff_pipe1_addr,
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
  int32_t read_bufx[NVT_RESULT_BUFSIZE] =
  {
    0
  };

  diff->x_num = priv->x_num;
  diff->y_num = priv->y_num;

  ret = nvt_diff_get(priv, read_bufx);
  if (ret == 0)
    {
      memcpy(diff->data, read_bufx, data_len * sizeof(int32_t));
    }
  else
    {
      ierr("ERROR: nvt diff show failed\n");
      return ret;
    }

  return 0;
}

#ifdef CONFIG_NVT_OFFLINE_LOG
static void nvt_log_data_to_csv(FAR void *arg)
{
  int      ret;
  int32_t  x = 0;
  int32_t  y = 0;
  int32_t  write_ret;
  char     *fbufp = NULL;
  int      fp = -1;
  int32_t  iarrayindex  = 0;
  int32_t  fw_frame_cnt = 0;
  uint32_t output_len   = 0;
  uint32_t input_x1     = 0;
  uint32_t input_x2     = 0;
  uint32_t input_y1     = 0;
  uint32_t input_y2     = 0;
  char     *csv_file_path = CONFIG_NVTLOG_PATH;
  FAR struct nt38350_dev_s    *priv = (FAR struct nt38350_dev_s *)arg;
  struct tm tmz =
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

  /* if file folder not exist, first create the folder */

  if (access(NVT_OFFLOG_FOLDER, F_OK) < 0)
    {
      if (mkdir(NVT_OFFLOG_FOLDER, 0777) != 0)
        {
          ierr("create file folder fail\n");
          return;
        }
    }

  /* open csv file */

  fp = open(csv_file_path, O_RDWR | O_CREAT);
  if (fp < 0)
    {
      if (fbufp)
        {
          kmm_free(fbufp);
          fbufp = NULL;
        }

      return;
    }

  /* saved header info to csv file */

  ret = read(fp, fbufp, 1);
  if (ret <= 0)
    {
      iwarn("read header fail, maybe it is first time create.\n");
    }

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
  localtime_r(&ts.tv_sec, &tmz);

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

  sprintf(fbufp, "timestamp:''%2d:%2d:%2d:000'',Frame index:''%5ld'',"
            "RawData:,,%2X,%4X,%5ld,%5ld,%5ld,%5ld,\r\n",
            tmz.tm_hour, tmz.tm_min, tmz.tm_sec, fw_frame_cnt, priv->fw_ver,
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

#ifdef CONFIG_NVT_DEBUG
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
    }

  if (fbufp)
    {
      kmm_free(fbufp);
      fbufp = NULL;
    }
}
#endif

#ifdef CONFIG_NVT_TOUCH_MP
static void nvt_print_data_log_in_one_line(int32_t *data,
                                           int32_t data_num)
{
  char *tmp_log = NULL;
  int32_t i = 0;

  tmp_log = (char *)kmm_zalloc(data_num * 7 + 1);
  if (!tmp_log)
    {
      ierr("ERROR: kzalloc for tmp_log failed!\n ");
      return;
    }

  for (i = 0; i < data_num; i++)
    {
      sprintf(tmp_log + i * 7, "%5ld, ", data[i]);
    }

  tmp_log[data_num * 7] = '\0';

  iinfo("%s \n", tmp_log);

  if (tmp_log)
    {
      kmm_free(tmp_log);
      tmp_log = NULL;
    }

  return;
}

static int8_t nvt_switch_freqhopendis(FAR struct nt38350_dev_s *priv,
                                      uint8_t freqhopendis)
{
  uint8_t retry = 0;
  int8_t  ret = 0;
  uint8_t buf[8] =
  {
    0
  };

  for (retry = 0; retry < 20; retry++)
    {
      /* ---set xdata index to EVENT BUF ADDR--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
                   EVENT_MAP_HOST_CMD);

      /* ---switch FreqHopEnDis--- */

      buf[0] = EVENT_MAP_HOST_CMD;
      buf[1] = freqhopendis;
      nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      usleep(NVT_DELAY_35MS);

      buf[0] = EVENT_MAP_HOST_CMD;
      buf[1] = 0xff;
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      if (buf[1] == 0x00)
        {
          break;
        }
    }

  if (retry == 20)
    {
      ierr("ERROR: switch FreqHopEnDis 0x%02X failed, buf[1]=0x%02X\n",
              freqhopendis, buf[1]);
      ret = -1;
    }

  return ret;
}

static int32_t nvt_save_rawdata_to_csv(int32_t *rawdata,
                                       uint8_t x_ch,
                                       uint8_t y_ch,
                                       const char *file_path,
                                       uint32_t offset)
{
  char     *fbufp = NULL;
  int32_t  x = 0;
  int32_t  y = 0;
  int32_t  iarrayindex = 0;
  int32_t  write_ret = 0;
  uint32_t output_len = 0;
  int fp = -1;

  fbufp = (char *)kmm_zalloc(1024);
  if (!fbufp)
    {
      ierr("ERROR: kzalloc for fbufp failed!\n");
      return -ENOMEM;
    }

  for (y = 0; y < y_ch; y++)
    {
      for (x = 0; x < x_ch; x++)
        {
          iarrayindex = y * x_ch + x;
          sprintf(fbufp + iarrayindex * 7 + y * 2, "%5ld, ",
                  rawdata[iarrayindex]);
        }

      nvt_print_data_log_in_one_line(rawdata + y * x_ch, x_ch);
      sprintf(fbufp + (iarrayindex + 1) * 7 + y * 2, "\r\n");
    }

  if (offset == 0)
    {
      fp = open(file_path, O_RDWR | O_CREAT | O_TRUNC);
    }
  else
    {
      fp = open(file_path, O_RDWR | O_CREAT);
    }

  if (fp < 0)
    {
      ierr("ERROR: open %s failed\n", file_path);
      if (fbufp)
        {
          kmm_free(fbufp);
          fbufp = NULL;
        }

      return -1;
    }

  output_len = y_ch * x_ch * 7 + y_ch * 2;

  if (offset > 0)
     lseek(fp, 0, SEEK_END);
  else
     lseek(fp, 0, SEEK_SET);

  write_ret = write(fp, fbufp, output_len);
  if (write_ret <= 0)
    {
      ierr("ERROR: write %s failed\n", file_path);
      if (fp)
        {
          close(fp);
        }

      if (fbufp)
        {
          kmm_free(fbufp);
          fbufp = NULL;
        }

      return -1;
    }

  if (fp)
    {
      close(fp);
    }

  if (fbufp)
    {
      kmm_free(fbufp);
      fbufp = NULL;
    }

  return 0;
}

static void nvt_enable_short_test(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[8] =
  {
    0
  };

  /* ---set xdata index to EVENT BUF ADDR--- */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
               EVENT_MAP_HOST_CMD);

  /* ---enable short test--- */

  buf[0] = EVENT_MAP_HOST_CMD;
  buf[1] = 0x43;
  buf[2] = 0xaa;
  buf[3] = 0x02;
  buf[4] = 0x00;
  nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 5);
}

static int32_t nvt_polling_hand_shake_status(FAR struct nt38350_dev_s *priv)
{
  int32_t i = 0;
  const int32_t retry = 250;
  uint8_t buf[8] =
  {
    0
  };

  usleep(NVT_DELAY_20MS);

  for (i = 0; i < retry; i++)
    {
      /* ---set xdata index to EVENT BUF ADDR--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
                   EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE);

      /* ---read fw status--- */

      buf[0] = EVENT_MAP_HANDSHAKING_OR_SUB_CMD_BYTE;
      buf[1] = 0x00;
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 2);

      if ((buf[1] == 0xa0) || (buf[1] == 0xa1))
        {
          break;
        }

      usleep(NVT_DELAY_10MS);
    }

  if (i >= retry)
    {
      ierr("ERROR: polling hand shake status failed, "
           "buf[1]=0x%02X\n", buf[1]);

      /* Read back 5 bytes from offset EVENT_MAP_HOST_CMD for debug check */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
                   EVENT_MAP_HOST_CMD);

      buf[0] = EVENT_MAP_HOST_CMD;
      buf[1] = 0x00;
      buf[2] = 0x00;
      buf[3] = 0x00;
      buf[4] = 0x00;
      buf[5] = 0x00;
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf, 6);
      ierr("ERROR: Read back 5 bytes from offset EVENT_MAP_HOST_CMD: "
           "0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
           buf[1], buf[2], buf[3], buf[4], buf[5]);

      return -1;
    }
  else
    {
      return 0;
    }
}

static int32_t nvt_read_fw_short(FAR struct nt38350_dev_s *priv,
                                 FAR struct nvt_mp_test_s *mp)
{
  uint32_t raw_pipe_addr = 0;
  uint8_t  *rawdata_buf = NULL;
  uint32_t x = 0;
  uint32_t y = 0;
  int32_t  iarrayindex = 0;
  uint8_t  buf[128] =
  {
    0
  };

  /* ---Enter Test Mode--- */

  if (nvt_clear_fw_status(priv))
    {
      return -EAGAIN;
    }

  nvt_enable_short_test(priv);

  if (nvt_polling_hand_shake_status(priv))
    {
      return -EAGAIN;
    }

  rawdata_buf = (uint8_t *)kmm_zalloc(NVT_X_CHANNEL * NVT_Y_CHANNEL * 2);
  if (!rawdata_buf)
    {
      ierr("ERROR: kzalloc for rawdata_buf failed!\n");
      return -ENOMEM;
    }

  if (nvt_get_fw_pipe(priv) == 0)
    {
      raw_pipe_addr = priv->mmap->raw_pipe0_addr;
    }
  else
    {
      raw_pipe_addr = priv->mmap->raw_pipe1_addr;
    }

  for (y = 0; y < NVT_Y_CHANNEL; y++)
    {
      /* ---change xdata index--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS,
                   raw_pipe_addr + y * NVT_X_CHANNEL * 2);
      buf[0] = (uint8_t)((raw_pipe_addr + y * NVT_X_CHANNEL * 2) & 0xff);
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf,
                       NVT_X_CHANNEL * 2 + 1);
      memcpy(rawdata_buf + y * NVT_X_CHANNEL * 2,
             buf + 1, NVT_X_CHANNEL * 2);
    }

  for (y = 0; y < NVT_Y_CHANNEL; y++)
    {
      for (x = 0; x < NVT_X_CHANNEL; x++)
        {
          iarrayindex = y * NVT_X_CHANNEL + x;
          mp->raw_data[iarrayindex] = (int16_t)(rawdata_buf[iarrayindex * 2]
                                 + 256 * rawdata_buf[iarrayindex * 2 + 1]);
        }
    }

  if (rawdata_buf)
    {
      kmm_free(rawdata_buf);
      rawdata_buf = NULL;
    }

  /* ---Leave Test Mode--- */

  nvt_change_mode(priv, NVT_NORMAL_MODE);

  /* Save Rawdata to CSV file */

  if (nvt_save_rawdata_to_csv(mp->raw_data, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                              mp->test_item, 0) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  return 0;
}

static int32_t rawdatatest_singlepoint_sub(FAR int32_t raw_data[],
                                   uint8_t record_result[],
                                   uint8_t x_ch, uint8_t y_ch,
                                   const int32_t rawdata_limit_postive[],
                                   const int32_t rawdata_limit_negative[])
{
  int32_t i = 0;
  int32_t j = 0;
  int32_t iarrayindex = 0;
  bool    ispass = true;

  for (j = 0; j < y_ch; j++)
    {
      for (i = 0; i < x_ch; i++)
        {
          iarrayindex = j * x_ch + i;

          record_result[iarrayindex] = 0x00; /* default value for pass */

          if (raw_data[iarrayindex] > rawdata_limit_postive[iarrayindex])
            {
              record_result[iarrayindex] |= 0x01;
            }

          if (raw_data[iarrayindex] < rawdata_limit_negative[iarrayindex])
            {
              record_result[iarrayindex] |= 0x02;
            }
        }
    }

  /* ---Check RecordResult--- */

  for (j = 0; j < y_ch; j++)
    {
      for (i = 0; i < x_ch; i++)
        {
          if (record_result[j * x_ch + i] != 0)
            {
              ispass = false;
              break;
            }
        }
    }

  if (ispass == false)
    {
      return -1; /* FAIL */
    }
  else
    {
      return 0; /* PASS */
    }
}

static int32_t nvt_read_baseline(FAR struct nt38350_dev_s *priv,
                                 FAR struct nvt_mp_test_s *mp)
{
  nvt_read_mdata(priv, priv->mmap->baseline_addr,
                 priv->mmap->baseline_btn_addr, mp->raw_data);

  /* Save Rawdata to CSV file */

  if (nvt_save_rawdata_to_csv(mp->raw_data, NVT_X_CHANNEL,
                              NVT_Y_CHANNEL, mp->test_item, 0) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  return 0;
}

static int32_t nvt_read_cc(FAR struct nt38350_dev_s *priv,
                           FAR struct nvt_mp_test_s *mp)
{
  if (nvt_get_fw_pipe(priv) == 0)
    {
      nvt_read_mdata(priv, priv->mmap->diff_pipe1_addr,
                     priv->mmap->diff_btn_pipe1_addr, mp->raw_data);
    }
  else
    {
      nvt_read_mdata(priv, priv->mmap->diff_pipe0_addr,
                     priv->mmap->diff_btn_pipe0_addr, mp->raw_data);
    }

  /* Save Rawdata to CSV file */

  if (nvt_save_rawdata_to_csv(mp->raw_data, NVT_X_CHANNEL,
                              NVT_Y_CHANNEL, mp->test_item, 0) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  return 0;
}

static void nvt_enable_noise_collect(FAR struct nt38350_dev_s *priv,
                                     int32_t frame_num)
{
  uint8_t buf[8] =
  {
    0
  };

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
               EVENT_MAP_HOST_CMD);

  buf[0] = EVENT_MAP_HOST_CMD;
  buf[1] = 0x47;
  buf[2] = 0xaa;
  buf[3] = frame_num;
  buf[4] = 0x00;
  nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 5);
}

static int32_t nvt_read_fw_noise(FAR struct nt38350_dev_s *priv,
                                 FAR struct nvt_mp_test_s *mp,
                                 FAR int *diffmax_flag,
                                 FAR int *diffmin_flag)
{
  uint32_t x = 0;
  uint32_t y = 0;
  int32_t iarrayindex = 0;
  int32_t frame_num = 0;
  uint32_t rawdata_diff_min_offset = 0;
  int32_t *rawdata_diff_max;
  int32_t *rawdata_diff_min;

  /* ---Enter Test Mode--- */

  rawdata_diff_max = (int32_t *)kmm_zalloc(NVT_RAWDATA_BUFSIZE);
  if (!rawdata_diff_max)
    {
      ierr("ERROR: kzalloc for rawdata_diff_max failed!\n");
      return -ENOMEM;
    }

  rawdata_diff_min = (int32_t *)kmm_zalloc(NVT_RAWDATA_BUFSIZE);
  if (!rawdata_diff_min)
    {
      ierr("ERROR: kzalloc for rawdata_diff_min failed!\n");
      return -ENOMEM;
    }

  if (nvt_clear_fw_status(priv))
    {
      return -EAGAIN;
    }

  frame_num = PS_CONFIG_DIFF_TEST_FRAME / 10;

  nvt_enable_noise_collect(priv, frame_num);

  /* need wait PS_Config_Diff_Test_Frame * 8.3ms */

  usleep(frame_num * NVT_DELAY_83MS);

  if (nvt_polling_hand_shake_status(priv))
    {
      return -EAGAIN;
    }

  if (nvt_get_fw_pipe(priv) == 0)
    {
      nvt_read_mdata(priv, priv->mmap->diff_pipe0_addr,
                     priv->mmap->diff_btn_pipe0_addr, mp->raw_data);
    }
  else
    {
      nvt_read_mdata(priv, priv->mmap->diff_pipe1_addr,
                     priv->mmap->diff_btn_pipe1_addr, mp->raw_data);
    }

  for (y = 0; y < priv->y_num; y++)
    {
      for (x = 0; x < priv->x_num; x++)
        {
          iarrayindex = y * priv->x_num + x;
          rawdata_diff_max[iarrayindex] =
                          (int8_t)((mp->raw_data[iarrayindex] >> 8) & 0xff);
          rawdata_diff_min[iarrayindex] =
                          (int8_t)(mp->raw_data[iarrayindex] & 0xff);
        }
    }

  /* ---Leave Test Mode--- */

  nvt_change_mode(priv, NVT_NORMAL_MODE);

  /* Save Rawdata to CSV file */

  if (nvt_save_rawdata_to_csv(rawdata_diff_max, priv->x_num, priv->y_num,
      mp->test_item, 0) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  rawdata_diff_min_offset = priv->y_num * priv->x_num * 7 +
                            priv->y_num * 2;

  /* Save Rawdata to CSV file */

  if (nvt_save_rawdata_to_csv(rawdata_diff_min, priv->x_num, priv->y_num,
      mp->test_item, rawdata_diff_min_offset) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  *diffmax_flag = rawdatatest_singlepoint_sub(rawdata_diff_max,
                mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                ps_config_lmt_fw_diff_p, ps_config_lmt_fw_diff_n);

  *diffmin_flag = rawdatatest_singlepoint_sub(rawdata_diff_min,
                mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                ps_config_lmt_fw_diff_p, ps_config_lmt_fw_diff_n);
  if (rawdata_diff_max)
    kmm_free(rawdata_diff_max);

  if (rawdata_diff_min)
    kmm_free(rawdata_diff_min);

  return 0;
}

static void nvt_enable_open_test(FAR struct nt38350_dev_s *priv)
{
  uint8_t buf[5] =
  {
    0
  };

  /* ---set xdata index to EVENT BUF ADDR--- */

  nvt_set_page(priv, NVT_I2C_FW_ADDRESS, priv->mmap->event_buf_addr |
               EVENT_MAP_HOST_CMD);

  /* ---enable open test--- */

  buf[0] = EVENT_MAP_HOST_CMD;
  buf[1] = 0x45;
  buf[2] = 0xaa;
  buf[3] = 0x02;
  buf[4] = 0x00;
  nt38350_write_reg(priv, NVT_I2C_FW_ADDRESS, buf, 5);
}

static int32_t nvt_read_fw_open(FAR struct nt38350_dev_s *priv,
                                FAR struct nvt_mp_test_s *mp)
{
  uint32_t raw_pipe_addr = 0;
  uint8_t *rawdata_buf = NULL;
  uint32_t x = 0;
  uint32_t y = 0;
  uint8_t buf[128] =
  {
    0
  };

  /* ---Enter Test Mode--- */

  if (nvt_clear_fw_status(priv))
    {
      return -EAGAIN;
    }

  nvt_enable_open_test(priv);

  if (nvt_polling_hand_shake_status(priv))
    {
      return -EAGAIN;
    }

  rawdata_buf = (uint8_t *)kmm_zalloc(NVT_IC_X_CFG_SIZE *
                 NVT_IC_Y_CFG_SIZE * 2);
  if (!rawdata_buf)
    {
      ierr("ERROR: kzalloc for rawdata_buf failed!\n");
      return -ENOMEM;
    }

  if (nvt_get_fw_pipe(priv) == 0)
    {
      raw_pipe_addr = priv->mmap->raw_pipe0_addr;
    }
  else
    {
      raw_pipe_addr = priv->mmap->raw_pipe1_addr;
    }

  for (y = 0; y < NVT_IC_Y_CFG_SIZE; y++)
    {
      /* ---change xdata index--- */

      nvt_set_page(priv, NVT_I2C_FW_ADDRESS, raw_pipe_addr +
                   y * NVT_IC_X_CFG_SIZE * 2);
      buf[0] = (uint8_t)((raw_pipe_addr +
                         y * NVT_IC_X_CFG_SIZE * 2) & 0xff);
      nt38350_read_reg(priv, NVT_I2C_FW_ADDRESS, buf,
                       NVT_IC_X_CFG_SIZE * 2 + 1);
      memcpy(rawdata_buf + y * NVT_IC_X_CFG_SIZE * 2,
             buf + 1, NVT_IC_X_CFG_SIZE * 2);
    }

  for (y = 0; y < NVT_IC_Y_CFG_SIZE; y++)
    {
      for (x = 0; x < NVT_IC_X_CFG_SIZE; x++)
        {
          if ((AIN_Y[y] != 0xff) && (AIN_X[x] != 0xff))
            {
              mp->raw_data[AIN_Y[y] * NVT_X_CHANNEL + AIN_X[x]] =
                   (int16_t)((rawdata_buf[(y * NVT_IC_X_CFG_SIZE + x) * 2] +
                   256 * rawdata_buf[(y * NVT_IC_X_CFG_SIZE + x) * 2 + 1]));
            }
        }
    }

  if (rawdata_buf)
    {
      kmm_free(rawdata_buf);
      rawdata_buf = NULL;
    }

  /* ---Leave Test Mode--- */

  nvt_change_mode(priv, NVT_NORMAL_MODE);

  /* Save RawData to CSV file */

  if (nvt_save_rawdata_to_csv(mp->raw_data, NVT_X_CHANNEL,
                              NVT_Y_CHANNEL, mp->test_item, 0) < 0)
    {
      ierr("ERROR: save rawdata to CSV file failed\n");
      return -EAGAIN;
    }

  return 0;
}

static void nvt_selftest(FAR struct nt38350_dev_s *priv,
                         FAR struct nvt_mp_test_s *mp)
{
  /* if file folder not exist, first create the folder */

  if (access(NVT_RESULT_FOLDER, F_OK) < 0)
    {
      if (mkdir(NVT_RESULT_FOLDER, 0777) != 0)
        {
          ierr("create file folder fail\n");
          return;
        }
    }

  if (nvt_check_fw_reset_state(priv, RESET_STATE_REK))
    {
      ierr("check fw reset state failed!\n");
    }

  if (nvt_switch_freqhopendis(priv, NVT_FREQ_HOP_DISABLE))
    {
      ierr("switch frequency hopping disable failed!\n");
    }

  if (nvt_check_fw_reset_state(priv, RESET_STATE_NORMAL_RUN))
    {
      ierr("check fw reset state failed!\n");
    }

  usleep(NVT_DELAY_100MS);

  if (nvt_clear_fw_status(priv))
    {
      ierr("clear fw status failed!\n");
    }

  nvt_change_mode(priv, NVT_MP_MODE_CC);

  if (nvt_check_fw_status(priv))
    {
      ierr("check fw status failed!\n");
    }

  switch (mp->cmd)
    {
      case NVT_FW_RAW:
        mp->test_item = CONFIG_NVT_FW_RAWDATA_CSV_FILE;
        mp->result_flag = 0;
        if (nvt_read_baseline(priv, mp) != 0)
          {
            mp->result_flag = 1;
          }
        else
          {
            mp->result_flag = rawdatatest_singlepoint_sub(mp->raw_data,
                      mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                      ps_config_lmt_fw_rawdata_p,
                      ps_config_lmt_fw_rawdata_n);
          }

        break;
      case NVT_CC:
        mp->test_item = CONFIG_NVT_FW_CC_CSV_FILE;
        mp->result_flag = 0;
        if (nvt_read_cc(priv, mp) != 0)
          {
            mp->result_flag = 1;
          }
        else
          {
            mp->result_flag = rawdatatest_singlepoint_sub(mp->raw_data,
                        mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                        ps_config_lmt_fw_cc_p, ps_config_lmt_fw_cc_n);
          }

        break;
      case NVT_FW_NOISE:
      case NVT_FW_SHORT:
      case NVT_FW_OPEN:
        nvt_change_mode(priv, NVT_NORMAL_MODE);
        if (mp->cmd == NVT_FW_NOISE)
          {
            int testresult_fw_diffmax;
            int testresult_fw_diffmin;
            mp->test_item = CONFIG_NVT_NOISE_TEST_CSV_FILE;
            mp->result_flag = 0;
            if (nvt_read_fw_noise(priv, mp, &testresult_fw_diffmax,
                                  &testresult_fw_diffmin) != 0)
              {
                mp->result_flag = 1;
              }
            else
              {
                if ((testresult_fw_diffmax == -1) ||
                    (testresult_fw_diffmin == -1))
                  mp->result_flag = -1;
                else
                  mp->result_flag = 0;
              }
          }
        else if (mp->cmd == NVT_FW_SHORT)
          {
            mp->test_item = CONFIG_NVT_SHORT_TEST_CSV_FILE;
            mp->result_flag = 0;
            if (nvt_read_fw_short(priv, mp) != 0)
              {
                mp->result_flag = 1;
              }
            else
              {
                mp->result_flag = rawdatatest_singlepoint_sub(mp->raw_data,
                           mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                           ps_config_lmt_short_rawdata_p,
                           ps_config_lmt_short_rawdata_n);
              }
          }
        else if (mp->cmd == NVT_FW_OPEN)
          {
            mp->test_item = CONFIG_NVT_OPEN_TEST_CSV_FILE;
            mp->result_flag = 0;
            if (nvt_read_fw_open(priv, mp) != 0)
              {
                mp->result_flag = 1;
              }
            else
              {
                mp->result_flag = rawdatatest_singlepoint_sub(mp->raw_data,
                           mp->record_result, NVT_X_CHANNEL, NVT_Y_CHANNEL,
                           ps_config_lmt_open_rawdata_p,
                           ps_config_lmt_open_rawdata_n);
              }
          }
        else
          {
            ;
          }

        break;
      default:
        ierr("cmd out of mp_test_e range! cmd: %d\n", mp->cmd);
        break;
    }

  nvt_bootloader_reset(priv);
}
#endif

#ifdef CONFIG_WAKEUP_GESTURE
void nvt_ts_wakeup_gesture_report(FAR struct nt38350_dev_s *priv,
                                  uint8_t gesture_id, uint8_t *data)
{
  uint8_t func_type = data[2];
  uint8_t func_id = data[3];
  struct touch_sample_s    sample;
  bool valid_gesture;

  /* support fw specifal data protocol */

  memset(&sample, 0, sizeof(struct touch_sample_s));

  if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE))
    {
      gesture_id = func_id;
    }
  else if (gesture_id > DATA_PROTOCOL)
    {
      iinfo("gesture_id %d is invalid, func_type=%d, func_id=%d\n",
             gesture_id, func_type, func_id);
      return;
    }

#ifdef CONFIG_NVT_DEBUG
  iinfo("gesture_id = %d\n", gesture_id);
#endif

  sample.npoints = 1;
  sample.point[0].flags = TOUCH_GESTURE_VALID;

  /* only support double click gesture wakeup */

  switch (gesture_id)
    {
      case GESTURE_DOUBLE_CLICK:
        iwarn("double click detect!\n");
        sample.point[0].gesture = TOUCH_DOUBLE_CLICK;
        valid_gesture = true;
        break;
      case GESTURE_SLIDE_UP:
        sample.point[0].gesture = TOUCH_SLIDE_UP;
        valid_gesture = false;
        break;
      case GESTURE_SLIDE_DOWN:
        sample.point[0].gesture = TOUCH_SLIDE_DOWN;
        valid_gesture = false;
        break;
      case GESTURE_SLIDE_LEFT:
        sample.point[0].gesture = TOUCH_SLIDE_LEFT;
        valid_gesture = false;
        break;
      case GESTURE_SLIDE_RIGHT:
        sample.point[0].gesture = TOUCH_SLIDE_RIGHT;
        valid_gesture = false;
        break;
      case GESTURE_PALM:
        sample.point[0].gesture = TOUCH_PALM;
        valid_gesture = false;
        break;
      default:
        valid_gesture = false;
        break;
    }

  if (valid_gesture)
    {
      touch_event(priv->lower.priv, &sample);
    }
}
#endif

static void nt38350_data_worker(FAR void *arg)
{
  FAR struct nt38350_dev_s    *priv = (FAR struct nt38350_dev_s *)arg;
  FAR struct nt38350_config_s *config;
  struct touch_sample_s  sample =
  {
    0
  };

  int      ret;
#ifdef CONFIG_NVT_OFFLINE_LOG
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

  memset(&priv->sample, 0, sizeof(struct ts_nt38350_sample_s));

#ifdef CONFIG_NVT_OFFLINE_LOG
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

#ifdef CONFIG_NVT_DEBUG
  iinfo("0x%02x, 0x%02x, 0x%02x, 0x%02x 0x%02x, 0x%02x\n",
        point_data[1], point_data[2],
        point_data[3], point_data[4],
        point_data[5], point_data[6]);
#endif

#ifdef CONFIG_NVT_OFFLINE_LOG
  if (priv->fw_type == NVT_DEBUGLOG_TYPE)
    {
      memcpy(priv->point_xdata_temp, (point_data + 1), NVT_POINT_DATA_LEN);
      memcpy((priv->point_xdata_temp + NVT_POINT_DATA_LEN),
             (point_data + 1 + NVT_POINT_DATA_LEN + NVT_POINT_DATA_EXT_LEN),
             NVT_S2D_DATA_LEN);
      memcpy((priv->point_xdata_temp + NVT_POINT_DATA_LEN +
             NVT_S2D_DATA_LEN),
             (point_data + 1 + NVT_POINT_DATA_LEN + NVT_POINT_DATA_EXT_LEN +
             NVT_S2D_DATA_LEN + NVT_FW_CMD_HANDLE_LEN),
             (NVT_S2D_DATA_EXT_LEN + NVT_FW_FRAME_CNT_LEN));
      nvt_log_data_to_csv(priv);
    }
#endif

#ifdef CONFIG_WAKEUP_GESTURE
  if ((priv->touch_awake == 0) || (priv->idle_mode))
    {
      input_id = (uint8_t)(point_data[1] >> 3);
      nvt_ts_wakeup_gesture_report(priv, input_id, point_data);
      config->enable(config, true);
      return;
    }
#endif

  /* only support single point touch */

  position = 1;
  if (((point_data[position] & 0x07) == FINGER_DOWN) ||
      ((point_data[position] & 0x07) == FINGER_MOVE))
    {
      if ((point_data[position] & 0x07) == FINGER_DOWN)
        priv->sample.contact = CONTACT_DOWN;
      else if ((point_data[position] & 0x07) == FINGER_MOVE)
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

      /* nt38350 without force function, pressure set const 1 */

      input_p = 1;
      priv->sample.pressure = input_p;

      priv->old_sample = priv->sample;
    }
  else if ((point_data[position] == lighting_palm[0]) &&
           (point_data[position + 1] == lighting_palm[1]) &&
           (point_data[position + 2] == lighting_palm[2]))
    {
      priv->sample.contact = CONTACT_PALM;
    }
  else if (point_data[position] == FINGER_UP)
    {
      priv->sample.contact = CONTACT_UP;
      priv->sample.x = priv->old_sample.x;
      priv->sample.y = priv->old_sample.y;
      priv->sample.width = priv->old_sample.width;
      priv->sample.pressure = priv->old_sample.pressure;
    }
  else
    {
      priv->sample.contact = CONTACT_NONE;
      input_x = 0;
      input_y = 0;
      input_w = 0;
      input_p = 0;
    }

  sample.npoints            = 1;
  sample.point[0].id        = 0;
  sample.point[0].gesture   = 0xff;
  sample.point[0].x         = priv->sample.x;
  sample.point[0].y         = priv->sample.y;
  sample.point[0].w         = priv->sample.width;
  sample.point[0].pressure  = priv->sample.pressure;
  sample.point[0].timestamp = touch_get_time();

#ifdef CONFIG_NVT_DEBUG
  iinfo("x %d, y %d, width %d pressure %d\n",
        sample.point[0].x, sample.point[0].y,
        sample.point[0].w, sample.point[0].pressure);
#endif

  if (priv->sample.contact == CONTACT_UP)
    {
      sample.point[0].flags = TOUCH_UP | TOUCH_ID_VALID |
                              TOUCH_POS_VALID;
    }
  else if (priv->sample.contact == CONTACT_DOWN)
    {
      iwarn("press: x %d, y %d\n", sample.point[0].x, sample.point[0].y);
      sample.point[0].flags = TOUCH_DOWN | TOUCH_ID_VALID |
                              TOUCH_POS_VALID;
    }
  else if (priv->sample.contact == CONTACT_MOVE)
    {
      sample.point[0].flags = TOUCH_MOVE | TOUCH_ID_VALID |
                              TOUCH_POS_VALID;
    }
  else if (priv->sample.contact == CONTACT_PALM)
    {
      iwarn("palm detect!\n");
      sample.point[0].flags   = TOUCH_GESTURE_VALID | TOUCH_ID_VALID;
      sample.point[0].gesture = TOUCH_PALM;
    }
  else
    {
      sample.point[0].flags = TOUCH_UP | TOUCH_ID_VALID;
    }

  touch_event(priv->lower.priv, &sample);

  config->enable(config, true);
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
  if (ret < 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

#ifdef CONFIG_PM
static void nt38350_poweroff_cb(void *arg)
{
  FAR struct nvt_power_resume_s *ptr =
            (FAR struct nvt_power_resume_s *)(arg);

  ptr->times++;
  ptr->icpower_state = NVT_POWER_OFF;
  iinfo("NT38350 ICPowerState %d\n", ptr->icpower_state);
}

static int nt38350_hardware_reinit(FAR struct nt38350_dev_s *priv)
{
  int ret;

  ret = nvt_bootloader_reset(priv);
  priv->nvt_pwr_resume.icpower_state = NVT_POWER_ON;

  return ret;
}

static int nt38350_ts_resume(FAR struct nt38350_dev_s *dev)
{
  int ret;
  FAR struct nt38350_config_s *config;

  config = dev->config;
  DEBUGASSERT(config != NULL);

  ret = nvt_check_fw_reset_state(dev, RESET_STATE_REK);
  if (ret < 0)
    {
      iwarn("FW is not ready, try to reset bootloader.\n");
      nvt_bootloader_reset(dev);
      ret = nvt_check_fw_reset_state(dev, RESET_STATE_REK);
      if (ret < 0)
        {
          ierr("FW is not ready!\n");
          return ret;
        }
    }

  dev->touch_awake = 1;
  dev->idle_mode   = false;
  dev->nvt_pwr_resume.times = 0;

  return ret;
}

static int nt38350_ts_suspend(FAR struct nt38350_dev_s *dev, uint8_t cmd)
{
  int ret;
  FAR struct nt38350_config_s *config;
  uint8_t buf[2];
  config = dev->config;
  DEBUGASSERT(config != NULL);

  buf[0] = EVENT_MAP_HOST_CMD;
  buf[1] = cmd;
  ret = nt38350_write_reg(dev, NVT_I2C_FW_ADDRESS, buf, 2);
  if (ret < 0)
    {
      ierr("Failed to resume\n");
      return ret;
    }

  dev->touch_awake = 0;

  return ret;
}

static int nt38350_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  return OK;
}

static void nt38350_pm_notify(FAR struct pm_callback_s *cb,
                              int domain, enum pm_state_e pmstate)
{
  FAR struct nt38350_dev_s *dev = container_of(cb,
                                               struct nt38350_dev_s, pm);
  int enable_state;

  enable_state = dev->config->get_icpower_state();
  if (!enable_state)
    {
      return;
    }

  if (domain == PM_DOMAIN_FACTEST || domain == PM_DOMAIN_OLED_TP)
    {
      switch (pmstate)
        {
        case PM_RESTORE:
        case PM_NORMAL:
          if (dev->current_state == PM_NORMAL)
            return;
          if (dev->nvt_pwr_resume.times >= NVT_INIT_RETRY_TIMES)
            {
              ierr("Retry resume NT38350 touch failed! times = %d\n",
                   dev->nvt_pwr_resume.times);
              dev->nvt_pwr_resume.times = 0;
              return;
            }

          if (dev->nvt_pwr_resume.icpower_state == NVT_POWER_OFF)
            {
              /* if power off, reset i2c to make sure it can work normal */

#ifdef CONFIG_I2C_RESET
              I2C_RESET(dev->config->i2c);
#endif
              nt38350_hardware_reinit(dev);
            }

          nt38350_ts_resume(dev);
          break;
        case PM_STANDBY:
        case PM_SLEEP:
          if (dev->current_state == PM_STANDBY ||
              dev->current_state == PM_SLEEP)
            return;
          if (dev->enable_fdm)
            {
              nt38350_ts_suspend(dev, NVT_PM_FDM);
            }
          else
            {
              dev->config->prepare_poweroff();
              nt38350_ts_suspend(dev, NVT_PM_DPSTDBY);
            }
          break;
        case PM_IDLE:
          dev->idle_mode = true;
          break;
        default:
          break;
        }

      dev->current_state = pmstate;
    }
}

static int nt38350_recovery_cb(void *arg)
{
  int ret;
  FAR struct nt38350_dev_s *dev = (struct nt38350_dev_s *)arg;

  ret = nt38350_hardware_reinit(dev);
  if (ret < 0)
    {
      ierr("Failed to reinit hardware\n");
      return ret;
    }

  ret = nt38350_ts_resume(dev);
  if (ret < 0)
    {
      ierr("Failed to resume\n");
      return ret;
    }

  dev->current_state = PM_NORMAL;
  iinfo("Change to pm normal state\n");

  return ret;
}
#endif

/***************************************************************************
 * Name: nt38350_control
 ***************************************************************************/

static int nt38350_control(FAR struct touch_lowerhalf_s *lower,
                           int cmd, unsigned long arg)
{
  FAR struct nt38350_dev_s *priv = container_of(lower,
                                                struct nt38350_dev_s,
                                                lower);
  int ret = 0;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(priv != NULL);

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
#ifdef CONFIG_NVT_TOUCH_MP
      case TSIOC_SELFTEST:  /* arg: Pointer to struct nvt_mp_test_s */
        {
          FAR struct nvt_mp_test_s *ptr = (FAR struct nvt_mp_test_s *)arg;
          nvt_selftest(priv, ptr);
        }
        break;
#endif
      case TSIOC_UPGRADEFW:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          *ptr = nvt_boot_update_firmware(priv);
        }
        break;
#ifdef CONFIG_PM
      case TSIOC_ENABLEGESTURE:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);
          priv->enable_fdm = *ptr;
        }
        break;
#endif
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifdef CONFIG_NT38350_NEED_UPGRADE_FW

/***************************************************************************
 * Name: nvt_update_firmware_work
 ***************************************************************************/

static void nvt_update_firmware_work(FAR void *arg)
{
  FAR struct nt38350_dev_s *priv = (FAR struct nt38350_dev_s *)arg;

  /* let touch screen stay in normal work mode, not power off */

#ifdef CONFIG_PM
  pm_stay(PM_DOMAIN_OLED_TP, PM_NORMAL);
#endif

  nvt_boot_update_firmware(priv);

  /* after update process, relax pm state */

#ifdef CONFIG_PM
  pm_relax(PM_DOMAIN_OLED_TP, PM_NORMAL);
#endif
}

#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int nt38350_register(FAR struct nt38350_config_s *config,
                     const char *devname)
{
  FAR struct nt38350_dev_s *priv;
  int ret = 0;

#ifdef CONFIG_NVT_DEBUG
  iinfo("devname: %s \n", devname);
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

  priv->config = config;                  /* Save the board configuration */
  priv->fw_path = CONFIG_NT38350_FW_PATH; /* Set up firmware path */
  priv->lower.maxpoint = NVT_TOUCH_MAX_FINGER_NUM;
  priv->lower.control  = nt38350_control;

#ifdef CONFIG_NVT_DEBUG
  iinfo("RST %d IRQ %d %d\n", priv->config->gpio_rst_pin,
        priv->config->gpio_irq_pin, __LINE__);
#endif

  /* Hardware Initial */

  usleep(NVT_DELAY_10MS);

  ret = nvt_ts_check_chip_ver_trim(priv, NVT_CHIP_VER_TRIM_ADDR);
  if (ret < 0)
    {
      ret = nvt_ts_check_chip_ver_trim(priv, NVT_CHIP_VER_TRIM_OLD_ADDR);
      if (ret < 0)
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
  ret = work_queue(HPWORK, &priv->fwwork, nvt_update_firmware_work,
                   priv, 0);
  if (ret < 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
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
  priv->touch_awake = 1;
  priv->idle_mode   = false;

#ifdef CONFIG_PM
  extern int lcdc_pmcb_early_register(struct pm_callback_s *cb);
  priv->current_state = PM_NORMAL;
  priv->pm.prepare = nt38350_pm_prepare;
  priv->pm.notify  = nt38350_pm_notify;
  lcdc_pmcb_early_register(&priv->pm);
  priv->enable_fdm = 0;

  priv->nvt_pwr_resume.icpower_state = NVT_POWER_ON;
  priv->config->powerdev_register_cb(NULL,
                nt38350_poweroff_cb, &(priv->nvt_pwr_resume));
  priv->config->panel_recovery_register(nt38350_recovery_cb, priv);
#endif

  ret = touch_register(&(priv->lower), devname,
                       CONFIG_NT38350_TOUCHSCREEN_BUFF_NUMS);
  if (ret < 0)
    {
      ierr("ERROR: touch_register() failed: %d\n", ret);
      ret = -ENODEV;
      goto errout_with_irq;
    }

  return ret;

errout_with_irq:
  config->detach(config, nt38350_data_interrupt);

errout_with_priv:
  kmm_free(priv);
  return ret;
}
