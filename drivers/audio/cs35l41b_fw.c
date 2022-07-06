/****************************************************************************
 * drivers/audio/cs35l41b_fw.c
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/cs35l41b.h>

#include "cs35l41b.h"
#include "cs35l41b_fw.h"
#include "cs35l41_fw_img.h"
#include "cs35l41_tune_fw_img.h"
#include "cs35l41_cal_fw_img.h"

#include <fcntl.h>
#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHECK_DSP_PROCESS_TIMES                         5

#define FW_IMG_BLOCK_SIZE                               512
#define FW_IMG_MODVAL                                   ((1 << 16) - 1)

#define FW_PREHEADER_SIZE                               8
#define FW_PREHEADER_OFFSET                             0
#define FW_HEADER_SIZE                                  32
#define FW_HEADER_OFFSET                                8

#define FW_SYM_TABLE_OFFSET                             40
#define FW_SYM_TABLE_SINGLE_OFFSET                      8

#define FW_DATA_HEADER_SIZE                             8
#define FW_DATA_HEADER_BLOCK_SIZE_OFFSET                0
#define FW_DATA_HEADER_BLOCK_ADDRESS_OFFSET             4

#define FW_IMG_MODVAL                                   ((1 << 16) - 1)

#define FW_PREHEADER_TYPE                               0
#define FW_HEADER_TYPE                                  1
#define FW_SYM_TABLE_TYPE                               2
#define FW_ALGORITHM_ID_LIST_TYPE                       3
#define FW_DATA_HEADER_TYPE                             4
#define FW_DATA_TYPE                                    5
#define FW_MAGIC2_TYPE                                  6
#define FW_CHECKSUM_TYPE                                7

#define CS35L41_DSP_STATUS_WORDS_TOTAL                  10

/* FIRMWARE_HALO_CSPL */

#define CS35L41_SYM_FIRMWARE_HALO_CSPL_HALO_STATE       (0x1)
#define CS35L41_SYM_FIRMWARE_HALO_CSPL_HALO_HEARTBEAT   (0x2)

/* CSPL */

#define CS35L41_SYM_CSPL_CSPL_STATE                     (0x3)
#define CS35L41_SYM_CSPL_CSPL_TEMPERATURE               (0x4)
#define CS35L41_SYM_CSPL_CAL_R                          (0x5)
#define CS35L41_SYM_CSPL_CAL_AMBIENT                    (0x6)
#define CS35L41_SYM_CSPL_CAL_STATUS                     (0x7)
#define CS35L41_SYM_CSPL_CAL_CHECKSUM                   (0x8)
#define CS35L41_SYM_CSPL_CAL_R_SELECTED                 (0x9)
#define CS35L41_SYM_CSPL_CAL_SET_STATUS                 (0xa)

#define CS35L41_CAL_STATUS_CALIB_SUCCESS                (0x1)

#define CS35L41B_CALIBERATION_INFO_STR                  "ro.factory.audio_par"

#define SUPPORT_DEFAULT_SAMPLERATE                      (44100)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef struct
{
  uint32_t img_magic_number_1;  /* image magic number1 */
  uint32_t img_format_rev;      /* image format rev */
} fw_img_preheader_t;

typedef struct
{
  uint32_t img_size;            /* image size */
  uint32_t sym_table_size;      /* symbol linking table size */
  uint32_t alg_id_list_size;    /* algorithm id list */
  uint32_t fw_id;               /* fw id */
  uint32_t fw_version;          /* fw version */
  uint32_t data_blocks;         /* fw data blocks */
  uint32_t max_block_size;      /* max block size of fw */
  uint32_t fw_img_release;      /* fw release number */
} fw_img_v2_header_t;

typedef struct
{
  uint32_t id;                  /* symbol linking id */
  uint32_t address;             /* symbol linking address of fw */
} fw_img_sym_table_t;

typedef struct
{
  uint32_t block_size;          /* data block size */
  uint32_t block_address;       /* data block address */
  uint32_t block_read_size;     /* read block size */
  uint32_t block_write_size;    /* write block size */
  FAR uint8_t  *block_read;     /* read block buffer */
  FAR uint8_t  *block_write;    /* write block buffer */
} fw_data_t;

typedef struct
{
  uint32_t                offset;             /* read fw offset */
  uint32_t                cal_value0;         /* checksum cal_value0 */
  uint32_t                cal_value1;         /* checksum cal_value1 */
  FAR uint32_t            *alg_id_list;       /* algorithm id list pointer */
  uint32_t                magic_number2;      /* image magic number 2 */
  uint32_t                checksum;           /* checksum value */
  FAR uint8_t             *imager_buffer;     /* pointer imager buffer */
  fw_img_preheader_t      preheader;          /* image preheader */
  fw_img_v2_header_t      header;             /* image header */
  FAR fw_img_sym_table_t  *sym_table;         /* symbol linking table pointer */
  fw_data_t               data;               /* fw data */
  bool                    is_checksum;        /* is need checksum? */
  bool                    is_read_from_file;  /* is read data from file? */
} fw_info_t;

typedef struct
{
  union
    {
      uint32_t words[CS35L41_DSP_STATUS_WORDS_TOTAL];
      struct
        {
          uint32_t halo_state;
          uint32_t halo_heartbeat;
          uint32_t cspl_state;
          uint32_t cspl_temperature;
          uint32_t cal_r;
          uint32_t cal_ambient;
          uint32_t cal_status;
          uint32_t cal_checksum;
          uint32_t cal_r_selected;
          uint32_t cal_set_status;
        };
    } data;

  bool is_hb_inc;              /* (True) The HALO HEARTBEAT is incrementing */
  bool is_calibration_applied; /* (True) Calibration values are applied */
  bool is_temp_changed;        /* (True) Monitored temperature is varying. */
  bool is_cal_vaild;
} dsp_status_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static fw_info_t g_fw_info;
static dsp_status_t g_dsp_status;

static const uint32_t g_cs35l41_post_boot_config[] =
{
  CS35L41_MIXER_DSP1RX5_INPUT_REG, CS35L41_INPUT_SRC_VPMON,
  CS35L41_MIXER_DSP1RX6_INPUT_REG, CS35L41_INPUT_SRC_CLASSH,
  CS35L41_MIXER_DSP1RX7_INPUT_REG, CS35L41_INPUT_SRC_TEMPMON,
  CS35L41_MIXER_DSP1RX8_INPUT_REG, CS35L41_INPUT_SRC_RSVD
};

static const uint32_t
g_cs35l41_dsp_status_controls[CS35L41_DSP_STATUS_WORDS_TOTAL] =
{
  CS35L41_SYM_FIRMWARE_HALO_CSPL_HALO_STATE,
  CS35L41_SYM_FIRMWARE_HALO_CSPL_HALO_HEARTBEAT,
  CS35L41_SYM_CSPL_CSPL_STATE,
  CS35L41_SYM_CSPL_CAL_SET_STATUS,
  CS35L41_SYM_CSPL_CAL_R_SELECTED,
  CS35L41_SYM_CSPL_CAL_R,
  CS35L41_SYM_CSPL_CAL_STATUS,
  CS35L41_SYM_CSPL_CAL_CHECKSUM,
  CS35L41_SYM_CSPL_CSPL_TEMPERATURE,
};

#if (SUPPORT_DEFAULT_SAMPLERATE == 48000)
static const uint32_t g_cs35l41_fs_syscfg[] =
{
  0x00002c04, 0x00000430,
  0x00002c0c, 0x00000003,
  0x00004804, 0x00000021,
};
#elif (SUPPORT_DEFAULT_SAMPLERATE == 44100)
static const uint32_t g_cs35l41_fs_syscfg[] =
{
  0x00002c04, 0x00000330,
  0x00002c0c, 0x0000000b,
  0x00004804, 0x00000019,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_cal_checksum
 *
 * Description:
 *   cs35l41b calculate checksum value
 *
 ****************************************************************************/

static void cs35l41b_cal_checksum(FAR fw_info_t *fw_info,
                                  FAR uint8_t *address,
                                  uint32_t size,
                                  bool is_reset)
{
  FAR uint8_t *temp_addr;
  uint32_t temp_size;
  uint32_t i;
  uint16_t temp_value;

  if (is_reset)
    {
      fw_info->cal_value0 = 0;
      fw_info->cal_value1 = 0;
      return;
    }

  if (!address || !fw_info)
    {
      auderr("paramters are invalid\n");
      return;
    }

  temp_addr = address;
  temp_size = size / sizeof(uint16_t);    /* get uint16_t type value */

  for (i = 0; i < temp_size; i++)
    {
      temp_value = (uint16_t)(*temp_addr) +
                   ((uint16_t)(*(temp_addr + 1)) << 8);
      fw_info->cal_value0 =
      (fw_info->cal_value0 + temp_value) % FW_IMG_MODVAL;

      fw_info->cal_value1 = (fw_info->cal_value1 + fw_info->cal_value0) %
                            FW_IMG_MODVAL;
      temp_addr += sizeof(uint16_t);
    }
}

/****************************************************************************
 * Name: read_fw_data
 *
 * Description:
 *   read fw data
 *
 ****************************************************************************/

static void read_fw_data(FAR fw_info_t *fw_info,
                         FAR uint8_t *buffer,
                         uint32_t size,
                         uint32_t offset)
{
  FAR uint8_t *address;

  if (!fw_info->is_read_from_file)
    {
      address = fw_info->imager_buffer + offset;
      memcpy(buffer, address, size);
    }
  else
    {
      address = fw_info->imager_buffer + offset;
    }

  if (fw_info->is_checksum)
    {
      cs35l41b_cal_checksum(fw_info, address, size, false);
    }
}

/****************************************************************************
 * Name: cs35l41b_read_fw_preheader
 *
 * Description:
 *   cs35l41b read fw preheader
 *
 ****************************************************************************/

static void cs35l41b_read_fw_preheader(FAR fw_info_t *fw_info)
{
  read_fw_data(fw_info, (uint8_t *)&(fw_info->preheader),
               sizeof(fw_img_preheader_t), fw_info->offset);

  fw_info->offset += FW_PREHEADER_SIZE;
}

/****************************************************************************
 * Name: cs35l41b_read_fw_header
 *
 * Description:
 *   cs35l41b read fw header
 *
 ****************************************************************************/

static void cs35l41b_read_fw_header(FAR fw_info_t *fw_info)
{
  read_fw_data(fw_info, (uint8_t *)&(fw_info->header),
               sizeof(fw_img_v2_header_t), fw_info->offset);

  fw_info->offset += FW_HEADER_SIZE;
}

/****************************************************************************
 * Name: cs35l41b_read_fw_symbol_linking_table
 *
 * Description:
 *   cs35l41b read fw symbol link table data
 *
 ****************************************************************************/

static int cs35l41b_read_fw_symbol_linking_table(FAR fw_info_t *fw_info)
{
  uint32_t i;
  FAR fw_img_sym_table_t *sym_table;

  if (fw_info->header.sym_table_size == 0 || !fw_info->sym_table)
    {
      return ERROR;
    }

  sym_table = fw_info->sym_table;

  for (i = 0; i < fw_info->header.sym_table_size; i++)
    {
      read_fw_data(fw_info, (uint8_t *)sym_table,
                   sizeof(fw_img_sym_table_t), fw_info->offset);
      fw_info->offset += sizeof(fw_img_sym_table_t);
      sym_table++;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_read_fw_algorithm_id_list
 *
 * Description:
 *   cs35l41b read fw algorithm ID list data
 *
 ****************************************************************************/

static int cs35l41b_read_fw_algorithm_id_list(FAR fw_info_t *fw_info)
{
  uint32_t i;
  FAR uint32_t *algorithm_addr;

  if (fw_info->header.alg_id_list_size == 0 || !fw_info->alg_id_list)
    {
      return ERROR;
    }

  algorithm_addr = fw_info->alg_id_list;

  for (i = 0; i < fw_info->header.alg_id_list_size; i++)
    {
      read_fw_data(fw_info, (uint8_t *)algorithm_addr,
                   sizeof(uint32_t), fw_info->offset);
      fw_info->offset += sizeof(uint32_t);
      algorithm_addr++;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_read_fw_data_header
 *
 * Description:
 *   cs35l41b read fw header of data
 *
 ****************************************************************************/

static void cs35l41b_read_fw_data_header(FAR fw_info_t *fw_info)
{
  read_fw_data(fw_info, (uint8_t *)&(fw_info->data.block_size),
               sizeof(uint32_t), fw_info->offset);
  fw_info->offset += sizeof(uint32_t);

  read_fw_data(fw_info, (uint8_t *)&(fw_info->data.block_address),
               sizeof(uint32_t), fw_info->offset);
  fw_info->offset += sizeof(uint32_t);
}

/****************************************************************************
 * Name: cs35l41b_read_fw_data
 *
 * Description:
 *   cs35l41b read fw data
 *
 ****************************************************************************/

static int cs35l41b_read_fw_data(FAR fw_info_t *fw_info, uint32_t size)
{
  if (!fw_info->data.block_read)
    {
      return ERROR;
    }

  fw_info->data.block_read_size = size;

  read_fw_data(fw_info, (uint8_t *)(fw_info->data.block_read),
               sizeof(uint8_t) * fw_info->data.block_read_size,
               fw_info->offset);
  fw_info->offset += size;

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_write_fw_data
 *
 * Description:
 *   cs35l41b write fw data
 *
 ****************************************************************************/

static int cs35l41b_write_fw_data(FAR struct cs35l41b_dev_s *priv,
                                     uint32_t block_addr, uint8_t *data,
                                     uint32_t size)
{
  int ret;

  ret = cs35l41b_write_block(priv, block_addr, data, size);
  if (ret == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_update_fw_data
 *
 * Description:
 *   cs35l41b update fw data
 *
 ****************************************************************************/

static int cs35l41b_update_fw_data(FAR struct cs35l41b_dev_s *priv,
                                       FAR fw_info_t *fw_info)
{
  uint32_t block_count;
  uint32_t block_size;
  uint32_t register_address;
  uint32_t i;
  FAR uint8_t  *data;
  uint32_t data_size;

  block_count   = fw_info->header.data_blocks;

  for (i = 0; i < block_count; i++)
    {
      cs35l41b_read_fw_data_header(fw_info);
      block_size        = fw_info->data.block_size;
      register_address  = fw_info->data.block_address;

      cs35l41b_read_fw_data(fw_info, block_size);
      data              = fw_info->data.block_read;
      data_size         = fw_info->data.block_read_size;

      if (cs35l41b_write_fw_data(priv, register_address,
                                 data, data_size) != OK)
        {
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41_read_magic_number2
 *
 * Description:
 *   cs35l41b read magic number 2
 *
 ****************************************************************************/

static void cs35l41_read_magic_number2(FAR fw_info_t *fw_info)
{
  read_fw_data(fw_info, (uint8_t *)&(fw_info->magic_number2),
               sizeof(uint32_t), fw_info->offset);

  fw_info->offset += sizeof(uint32_t);
}

/****************************************************************************
 * Name: cs35l41_read_checksum
 *
 * Description:
 *   cs35l41b read checksum value
 *
 ****************************************************************************/

static void cs35l41_read_checksum(FAR fw_info_t *fw_info)
{
  read_fw_data(fw_info, (uint8_t *)&(fw_info->checksum),
               sizeof(uint32_t), fw_info->offset);

  fw_info->offset += sizeof(uint32_t);
  audinfo("checksum:0x%08lx\n", fw_info->checksum);
}

/****************************************************************************
 * Name: load_fw_process
 *
 * Description:
 *   cs35l41b load fw process
 *
 ****************************************************************************/

static int load_fw_process(FAR struct cs35l41b_dev_s *priv,
                               FAR fw_info_t *fw_info)
{
  /* check if need calculate checksum */

  fw_info->is_checksum                   = true;
  cs35l41b_cal_checksum(fw_info, NULL, 0, true);

  /* step1:read fw img preheader */

  cs35l41b_read_fw_preheader(fw_info);

  /* step2:read fw img header */

  cs35l41b_read_fw_header(fw_info);

  /* malloc memory for max block data size */

  fw_info->data.block_read =
  kmm_zalloc(fw_info->header.max_block_size * sizeof(uint8_t));
  if (!fw_info->data.block_read)
    {
      auderr("kmm_zalloc memory failed\n");
      return ERROR;
    }

  if (fw_info->header.sym_table_size)
    {
      fw_info->sym_table = kmm_zalloc(sizeof(fw_img_sym_table_t) *
                           fw_info->header.sym_table_size);
      if (!fw_info->sym_table)
        {
          auderr("kmm_zalloc memory failed\n");
          return ERROR;
        }

      /* step3:read symbols */

      cs35l41b_read_fw_symbol_linking_table(fw_info);
    }

  if (fw_info->header.alg_id_list_size)
    {
      fw_info->alg_id_list = kmm_zalloc(sizeof(uint32_t) *
                             fw_info->header.alg_id_list_size);
      if (!fw_info->alg_id_list)
        {
          auderr("kmm_zalloc memory failed\n");
          return ERROR;
        }

      /* step4:read alg_id_list */

      cs35l41b_read_fw_algorithm_id_list(fw_info);
    }

  /* step5:update fw data */

  if (cs35l41b_update_fw_data(priv, fw_info) != OK)
    {
      return ERROR;
    }

  /* step6:read magic number2 */

  cs35l41_read_magic_number2(fw_info);

  /* step7:read checksum */

  fw_info->is_checksum = false;  /* disable calculate checksum value */
  cs35l41_read_checksum(fw_info);

  audinfo("cal_value0:  0x%08lx\n", fw_info->cal_value0);
  audinfo("cal_value1:  0x%08lx\n", fw_info->cal_value1);
  audinfo("cal_checksum:0x%08lx\n", fw_info->cal_value0 +
                                   (fw_info->cal_value1 << 16));

  if (fw_info->checksum != (fw_info->cal_value0 +
                           (fw_info->cal_value1 << 16)))
    {
      auderr("fw checksum is not match!\n");
      return ERROR;
    }

  /* step6:free data */

  if (fw_info->header.sym_table_size)
    {
      kmm_free(fw_info->sym_table);
    }

  if (fw_info->header.alg_id_list_size)
    {
      kmm_free(fw_info->alg_id_list);
    }

  kmm_free(fw_info->data.block_read);

  memset(fw_info, 0, sizeof(fw_info_t));

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_load_main_fw_process
 *
 * Description:
 *   cs35l41b load main fw
 *
 ****************************************************************************/

static int cs35l41b_load_main_fw_process(FAR struct cs35l41b_dev_s *priv)
{
  g_fw_info.imager_buffer = (uint8_t *)g_cs35l41_fw_img;
  if (load_fw_process(priv, &g_fw_info) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_load_tune_process
 *
 * Description:
 *   cs35l41b load tune fw
 *
 ****************************************************************************/

static int cs35l41b_load_tune_process(FAR struct cs35l41b_dev_s *priv)
{
  if (priv->scenario_mode == CS35L41B_SCENARIO_SCO)
    {
      g_fw_info.imager_buffer = (uint8_t *)g_cs35l41_tune_sco_fw_img;
    }
  else
    {
      g_fw_info.imager_buffer = (uint8_t *)g_cs35l41_tune_fw_img;
    }

  if (load_fw_process(priv, &g_fw_info) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_load_cal_process
 *
 * Description:
 *   cs35l41b load calibration fw
 *
 ****************************************************************************/

static int cs35l41b_load_cal_process(FAR struct cs35l41b_dev_s *priv)
{
  g_fw_info.imager_buffer = (uint8_t *)g_cs35l41_cal_fw_img;
  if (load_fw_process(priv, &g_fw_info) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: get_symbol_link_address
 *
 * Description:
 *   get symbol link table address of id
 *
 ****************************************************************************/

static uint32_t get_symbol_link_address(int id)
{
  uint8_t *temp_buffer = (uint8_t *)g_cs35l41_fw_img;
  uint32_t addr;

  /* skip preheader and header */

  temp_buffer += FW_SYM_TABLE_OFFSET;

  temp_buffer += (id - 1) * FW_SYM_TABLE_SINGLE_OFFSET;
  temp_buffer += 4;

  addr = temp_buffer[0] + ((uint32_t)temp_buffer[1] << 8) +
         ((uint32_t)temp_buffer[2] << 16) + ((uint32_t)temp_buffer[3] << 24);

  return addr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_set_boot_configuration
 *
 * Description:
 *   cs35l41b set boot configuration
 *
 ****************************************************************************/

int cs35l41b_set_boot_configuration(FAR struct cs35l41b_dev_s *priv)
{
  uint8_t i;
  int ret;

  for (i = 0; i < 4; i++)
    {
      ret = cs35l41b_write_register(priv, g_cs35l41_post_boot_config[2 * i],
                                    g_cs35l41_post_boot_config[2 * i + 1]);
      if (ret == ERROR)
        {
          return ERROR;
        }
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_UNLOCK_1);
  if (ret == ERROR)
    {
      return ERROR;
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_UNLOCK_2);
  if (ret == ERROR)
    {
      return ERROR;
    }

  for (i = 0; i < 3; i++)
    {
      ret = cs35l41b_write_register(priv, g_cs35l41_fs_syscfg[2 * i],
                                    g_cs35l41_fs_syscfg[2 * i + 1]);
      if (ret == ERROR)
        {
          return ERROR;
        }
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_LOCK_1);
  if (ret == ERROR)
    {
      return ERROR;
    }

  ret = cs35l41b_write_register(priv, CS35L41_CTRL_KEYS_TEST_KEY_CTRL_REG,
                                CS35L41_TEST_KEY_CTRL_LOCK_2);
  if (ret == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_dsp_process
 *
 * Description:
 *   cs35l41b get dsp process status
 *
 ****************************************************************************/

int cs35l41b_is_dsp_processing(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t  i;
  uint32_t  regval;
  uint32_t  times;
  int ret = OK;

  for (i = 0; i < CS35L41_DSP_STATUS_WORDS_TOTAL; i++)
    {
      ret = cs35l41b_read_register(priv, &g_dsp_status.data.words[i],
            get_symbol_link_address(g_cs35l41_dsp_status_controls[i]));

      if (ret < 0)
        {
          auderr("cs35l41b read register failed\n");
          return ERROR;
        }
    }

  up_udelay(1000 * 10);

  times = CHECK_DSP_PROCESS_TIMES;

  while (times--)
    {
      for (i = 0; i < CS35L41_DSP_STATUS_WORDS_TOTAL; i++)
        {
          ret = cs35l41b_read_register(priv, &regval,
                get_symbol_link_address(g_cs35l41_dsp_status_controls[i]));
          if (ret < 0)
            {
              auderr("cs35l41b read register failed\n");
              return ERROR;
            }

          /* If the current field is HALO_HEARTBEAT,
           * and there is a change in subsequent values
           */

          if ((i == 1) && (regval != g_dsp_status.data.words[i]))
            {
              g_dsp_status.is_hb_inc = true;
              auderr("times:%d\n", times);
              goto done;
            }

          /* If the current field is CSPL_TEMPERATURE,
           * and there is a change in subsequent values
           */

          if ((i == 8) && (regval != g_dsp_status.data.words[i]))
            {
              g_dsp_status.is_temp_changed = true;
            }

          g_dsp_status.data.words[i] = regval;
        }
    }

done:
  if (g_dsp_status.is_hb_inc)
    {
      return OK;
    }

  return ERROR;
}

/****************************************************************************
 * Name: cs35l41b_write_caliberate_ambient
 *
 * Description:
 *   cs35l41b write cliberate ambient degree
 *
 ****************************************************************************/

int cs35l41b_write_caliberate_ambient(FAR struct cs35l41b_dev_s *priv,
                                      uint32_t ambient_temp_deg_c)
{
  uint32_t address;
  int ret;

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_AMBIENT);
  ret = cs35l41b_write_register(priv, address, ambient_temp_deg_c);
  if (ret == ERROR)
    {
      auderr("Error write 0x%08x!\n", address);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_calibrate
 *
 * Description:
 *   cs35l41b cliberate process
 *
 ****************************************************************************/

int cs35l41b_calibrate(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t address;
  uint32_t val;
  int ret = OK;

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_R);
  ret = cs35l41b_read_register(priv, &val, address);
  if (ret < 0)
    {
      auderr("Error read register!\n");
      goto error;
    }

  g_dsp_status.data.cal_r = val;

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_STATUS);
  ret = cs35l41b_read_register(priv, &val, address);
  if (ret < 0)
    {
      auderr("Error read register!\n");
      goto error;
    }

  if ((val != CS35L41_CAL_STATUS_CALIB_SUCCESS) && (val != 3))
    {
      auderr("Error calibrate cs35l41b!\n");
      ret = ERROR;
      goto error;
    }

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_CHECKSUM);
  ret = cs35l41b_read_register(priv, &val, address);
  if (ret < 0)
    {
      auderr("Error read register!\n");
      goto error;
    }

  if (val != (g_dsp_status.data.cal_r + CS35L41_CAL_STATUS_CALIB_SUCCESS))
    {
      auderr("Error verify calibrate cs35l41b!\n");
      ret = ERROR;
      goto error;
    }

error:
  if (ret == ERROR)
    {
      g_dsp_status.is_cal_vaild = false;
      return ERROR;
    }

  g_dsp_status.is_cal_vaild = true;

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_get_calibration_result
 *
 * Description:
 *   cs35l41b get cliberate value
 *
 ****************************************************************************/

uint32_t cs35l41b_get_calibration_result(void)
{
  if (!g_dsp_status.is_cal_vaild)
    {
      g_dsp_status.data.cal_r = 0x1fffffff;
    }

  return g_dsp_status.data.cal_r;
}

/****************************************************************************
 * Name: cs35l41b_load_calibration_value
 *
 * Description:
 *   cs35l41b load caliberate value
 *
 ****************************************************************************/

int cs35l41b_load_calibration_value(FAR struct cs35l41b_dev_s *priv)
{
  uint32_t address;
  int ret;
  uint32_t value;

  if (priv->is_calibrate_value_loaded)
    {
      return ERROR;
    }

  if (!priv->lower->get_caliberate_result)
    {
      audwarn("lower get calibration result is invalid!\n");
      value = 0x1fb1;
    }
  else
    {
      if (priv->lower->get_caliberate_result(&value) != OK)
        {
          value = 0x1fb1;
        }
    }

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_R);

  audwarn("caliberate value:0x%08lx | address:0x%08lx\n", value, address);

  ret = cs35l41b_write_register(priv, address, value);
  if (ret == ERROR)
    {
      return ERROR;
    }

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_STATUS);
  ret = cs35l41b_write_register(priv, address,
                                CS35L41_CAL_STATUS_CALIB_SUCCESS);
  if (ret == ERROR)
    {
      return ERROR;
    }

  address = get_symbol_link_address(CS35L41_SYM_CSPL_CAL_CHECKSUM);
  ret = cs35l41b_write_register(priv, address,
        (value + CS35L41_CAL_STATUS_CALIB_SUCCESS));
  if (ret == ERROR)
    {
      return ERROR;
    }

  priv->is_calibrate_value_loaded = true;

  return OK;
}

/****************************************************************************
 * Name: cs35l41_dsp_boot
 *
 * Description:
 *   cs35l41b internal dsp boot
 *
 ****************************************************************************/

int cs35l41_dsp_boot(FAR struct cs35l41b_dev_s *priv, int mode)
{
  if (mode == CS35L41_ASP_MODE)
    {
      priv->mode  = CS35L41_ASP_MODE;
      priv->state = CS35L41_STATE_STANDBY;
      return OK;
    }

  if (cs35l41b_load_main_fw_process(priv) == ERROR)
    {
      return ERROR;
    }

  if (mode == CS35L41_DSP_TUNE_MODE)
    {
      priv->mode  = CS35L41_DSP_TUNE_MODE;

      if (cs35l41b_load_tune_process(priv) == ERROR)
        {
          return ERROR;
        }
    }
  else if (mode == CS35L41_DSP_CAL_MODE)
    {
      priv->mode  = CS35L41_DSP_CAL_MODE;

      if (cs35l41b_load_cal_process(priv) == ERROR)
        {
          return ERROR;
        }
    }

  if (priv->mode == CS35L41_DSP_TUNE_MODE)
    {
      if (cs35l41b_load_calibration_value(priv) == ERROR)
        {
          return ERROR;
        }
    }

  if (cs35l41b_set_boot_configuration(priv) == ERROR)
    {
      return ERROR;
    }

  priv->state = CS35L41_STATE_DSP_STANDBY;

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_tune_change_params
 *
 * Description:
 *   cs35l41b change tune image
 *
 ****************************************************************************/

int cs35l41b_tune_change_params(FAR struct cs35l41b_dev_s *priv)
{
  static int scenario_mode = CS35L41B_SCENARIO_SPEAKER;

  /* do not need change current scenario mode */

  if (priv->scenario_mode == scenario_mode)
    {
      return OK;
    }

  scenario_mode = priv->scenario_mode;

  if (cs35l41b_start_tuning_switch(priv) == ERROR)
    {
      return ERROR;
    }

  if (cs35l41b_set_boot_configuration(priv) == ERROR)
    {
      return ERROR;
    }

  if (cs35l41b_load_tune_process(priv) == ERROR)
    {
      return ERROR;
    }

  if (cs35l41b_finish_tuning_switch(priv) == ERROR)
    {
      return ERROR;
    }

  return OK;
}
