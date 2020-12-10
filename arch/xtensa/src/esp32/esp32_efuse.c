/****************************************************************************
 * arch/xtensa/src/esp32/esp32_efuse.c
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

#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/kmalloc.h>
#include <nuttx/efuse/esp_efuse.h>

#include "hardware/esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Character driver methods */

static int     efuse_open(FAR struct file *filep);
static int     efuse_close(FAR struct file *filep);
static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     efuse_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/* This structure is used for access control and batch writing mode */

struct efuse_access_s
{
  sem_t   exclsem;            /* Supports mutual exclusion */
  bool    batch_writing_mode; /* Controls batch writing */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_efusefops =
{
  efuse_open,
  efuse_close,
  efuse_read,
  efuse_write,
  NULL,
  efuse_ioctl,
  NULL
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp_efuse_init(void)
{
  FAR struct efuse_access_s *priv;
  int ret;

  /* Allocate a new efuse access instance */

  priv = (FAR struct efuse_access_s *)
    kmm_zalloc(sizeof(struct efuse_access_s));

  if (!priv)
    {
      merr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Initially batch mode is deactivated */

  priv->batch_writing_mode = false;

  /* Register the character driver */

  ret = register_driver("/dev/efuse", &g_efusefops, 0666, priv);
  if (ret < 0)
    {
      merr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: efuse_open
 *
 * Description:
 *   This function is called whenever the LM-75 device is opened.
 *
 ****************************************************************************/

static int efuse_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: efuse_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int efuse_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: efuse_read
 ****************************************************************************/

static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: efuse_write
 ****************************************************************************/

static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: efuse_ioctl
 ****************************************************************************/

static int efuse_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct efuse_access_s *priv = inode->i_private;
  int ret   = OK;

  switch (cmd)
    {
      /* Read a blob */

      case EFUSEIOC_READ_FIELD_BLOB:
        {
          FAR struct esp_efuse_par_id *param =
                     (FAR struct esp_efuse_par_id *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          if (param->field == NULL || param->size == 0)
            {
              merr("Error: This field is null or its size is 0\n");
              return -EINVAL;
            }

          esp_efuse_read_field_blob(param->field, param->data, param->size);
        }
        break;

      /* Read a bit */

      case EFUSEIOC_READ_FIELD_BIT:
        {
          FAR struct esp_efuse_par_id *param =
                     (FAR struct esp_efuse_par_id *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          /* Read the field bit */

          esp_efuse_read_field_blob(param->field, param->data, 1);
        }
        break;

      /* Write a blob */

      case EFUSEIOC_WRITE_FIELD_BLOB:
        {
          FAR struct esp_efuse_par_id *param =
                     (FAR struct esp_efuse_par_id *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          if (param->field == NULL || param->size == 0)
            {
              merr("Error: This field is null or its size is 0\n");
              return -EINVAL;
            }

          /* Get exclusive access */

          nxsem_wait_uninterruptible(&priv->exclsem);

          /* Write the blob data to the field */

          esp_efuse_write_field_blob(param->field, param->data, param->size);

          /* Burn the EFUSEs */

          esp_efuse_burn_efuses();

          nxsem_post(&priv->exclsem);
        }
        break;

      /* Write a bit */

      case EFUSEIOC_WRITE_FIELD_BIT:
        {
          FAR struct esp_efuse_par_id *param =
                     (FAR struct esp_efuse_par_id *)((uintptr_t)arg);
          uint8_t existing = 0;
          const uint8_t one = 1;

          DEBUGASSERT(param != NULL);

          if (param->field == NULL || param->field[0]->bit_count != 1)
            {
              merr("Error: This field is not 1 bit long\n");
              return -EINVAL;
            }

          /* Read existing bit, if it is already 1 */

          esp_efuse_read_field_blob(param->field, &existing, 1);

          if (existing)
            {
              merr("Error: The fuse is already burned\n");
              return -EINVAL;
            }

          /* Get exclusive access */

          nxsem_wait_uninterruptible(&priv->exclsem);

          /* Write the register field */

          esp_efuse_write_field_blob(param->field, &one, 1);

          /* Burn the EFUSEs */

          esp_efuse_burn_efuses();

          nxsem_post(&priv->exclsem);
        }
        break;

      /* Read a register from efuse */

      case EFUSEIOC_READ_REG:
        {
          FAR struct esp_efuse_par *param =
                     (FAR struct esp_efuse_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          *param->data = esp_efuse_read_reg(param->block,
                                           param->reg);
        }
        break;

      /* Write a register to efuse */

      case EFUSEIOC_WRITE_REG:
        {
          FAR struct esp_efuse_par *param =
                     (FAR struct esp_efuse_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          /* Get exclusive access */

          nxsem_wait_uninterruptible(&priv->exclsem);

          /* Write the efuse register */

          esp_efuse_write_reg(param->block, param->reg, *param->data);

          /* Burn the EFUSEs */

          esp_efuse_burn_efuses();

          nxsem_post(&priv->exclsem);
        }
        break;

      /* Read a block */

      case EFUSEIOC_READ_BLOCK:
        {
          FAR struct esp_efuse_par *param =
                     (FAR struct esp_efuse_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          nxsem_wait_uninterruptible(&priv->exclsem);

          /* Write the key data at the block */

          esp_efuse_read_block(param->block, (void *) param->data,
                                param->bit_offset, param->bit_size);

          nxsem_post(&priv->exclsem);
        }
        break;

      /* Write a block */

      case EFUSEIOC_WRITE_BLOCK:
        {
          FAR struct esp_efuse_par *param =
                     (FAR struct esp_efuse_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          /* Get exclusive access */

          nxsem_wait_uninterruptible(&priv->exclsem);

          /* Write the key data at the block */

          esp_efuse_write_block(param->block, param->data,
                                param->bit_offset, param->bit_size);

          /* Burn the EFUSEs */

          esp_efuse_burn_efuses();

          nxsem_post(&priv->exclsem);
        }
        break;

      default:
        {
          minfo("Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

