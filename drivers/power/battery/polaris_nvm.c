/****************************************************************************
 * drivers/power/battery/polaris_nvm.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include "polaris.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define FW_PATH                 "/vendor/charge/WPC_FW.txt"
#define HEAD_INFO_LINS          5
#define MAX_BYTES_PER_LINE      64
/****************************************************************************
 * Private Functions
 ****************************************************************************/

int get_fw_head_info(struct polaris_chip_info *head_info)
{
  FILE *f_fw = NULL;
  char buf[MAX_BYTES_PER_LINE];

  /* open source file */

  if (NULL == (f_fw = fopen(FW_PATH, "r")))
    {
      set_errno(-ENOENT);
      goto exit_err;
    }

  /* get head info */

  memset(buf, 0, sizeof(buf));
  if (NULL == fgets(buf, sizeof(buf), f_fw))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  sscanf(buf, "NVM_TARGET_CUT_ID:%d", (int *)&(head_info->cut_id));

  memset(buf, 0, sizeof(buf));
  if (NULL == fgets(buf, sizeof(buf), f_fw))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  sscanf(buf, "NVM_CFG_SIZE:%d", (int *)&(head_info->config_size));

  memset(buf, 0, sizeof(buf));
  if (NULL == fgets(buf, sizeof(buf), f_fw))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  sscanf(buf, "NVM_CFG_VERSION_ID:%x", (int *)&(head_info->config_id));

  memset(buf, 0, sizeof(buf));
  if (NULL == fgets(buf, sizeof(buf), f_fw))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  sscanf(buf, "NVM_PATCH_SIZE:%d", (int *)&(head_info->patch_size));

  memset(buf, 0, sizeof(buf));
  if (NULL == fgets(buf, sizeof(buf), f_fw))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  sscanf(buf, "NVM_PATCH_VERSION_ID:%x", (int *)&(head_info->nvm_patch_id));

  fclose(f_fw);
  return OK;

exit_err:
  if (NULL != f_fw)
    {
      fclose(f_fw);
    }

  baterr("NVM head err:%d\n", get_errno());
  return(get_errno());
}

int get_fw_data(uint8_t *cfg_data, uint8_t *patch_data)
{
  FILE *f_fw = NULL;
  char *token = NULL;
  char *ptr;
  char buf[MAX_BYTES_PER_LINE];
  uint16_t data_bytes = 0;
  uint8_t i;

  /* input parameters check */

  if ((NULL == cfg_data) || (NULL == patch_data))
    {
      set_errno(-EINVAL);
      goto exit_err;
    }

  /* open source file */

  if (NULL == (f_fw = fopen(FW_PATH, "r")))
    {
      set_errno(-ENOENT);
      goto exit_err;
    }

  /* jump to data section */

  for (i = 0; i < HEAD_INFO_LINS; i++)
    {
      if (NULL == fgets(buf, sizeof(buf), f_fw))
        {
          set_errno(-ENODATA);
          goto exit_err;
        }
    }

  /* get cfg data */

  memset(buf, 0, sizeof(buf));
  if ((NULL == fgets(buf, sizeof(buf), f_fw)) || strcmp(buf, "cfg_data:\n"))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  memset(buf, 0, sizeof(buf));
  while (fgets(buf, sizeof(buf), f_fw))
    {
      if (strcmp(buf, "#\n"))
        {
          token = strtok(buf, ",");
          while (token != NULL)
            {
              if (strcmp(token, "\n"))
                {
                  cfg_data[data_bytes] = (uint8_t)strtol(token, &ptr, 16);
                  data_bytes++;
                }

              token = strtok(NULL, ",");
            }

          memset(buf, 0, sizeof(buf));
        }
      else
        {
          memset(buf, 0, sizeof(buf));
          break;
        }
    }

  /* get patch data */

  data_bytes = 0;
  memset(buf, 0, sizeof(buf));
  if ((NULL == fgets(buf, sizeof(buf), f_fw)) \
  || strcmp(buf, "patch_data:\n"))
    {
      set_errno(-ENODATA);
      goto exit_err;
    }

  memset(buf, 0, sizeof(buf));
  while (fgets(buf, sizeof(buf), f_fw))
    {
      if (strcmp(buf, "#\n"))
        {
          token = strtok(buf, ",");
          while (token != NULL)
            {
              if (strcmp(token, "\n"))
                {
                  patch_data[data_bytes] = (uint8_t)strtol(token, &ptr, 16);
                  data_bytes++;
                }

              token = strtok(NULL, ",");
            }

          memset(buf, 0, sizeof(buf));
        }
      else
        {
          memset(buf, 0, sizeof(buf));
          break;
        }
    }

  fclose(f_fw);
  return OK;

exit_err:
  if (NULL != f_fw)
    {
      fclose(f_fw);
    }

  baterr("NVM data err:%d\n", get_errno());
  return(get_errno());
}
