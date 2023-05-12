/****************************************************************************
 * drivers/crypto/pnt/pnt_se05x_api.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pnt_se05x_api.h"

#include "../se05x_internal.h"
#include "pnt_util.h"
#include "scp03_keys.h"
#include <nuttx/kmalloc.h>
#include <phNxpEse_Internal.h>
#include <se05x_APDU_apis.h>
#include <smCom.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCP03_KEY_SIZE 16
#define DATA_CHUNK_SIZE 100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pnt_handle
{
  Se05xSession_t session;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const SE05x_ECSignatureAlgo_t
    signature_algorithm_mapping[SE05X_ALGORITHM_SIZE] =
{
        kSE05x_ECSignatureAlgo_NA,      kSE05x_ECSignatureAlgo_PLAIN,
        kSE05x_ECSignatureAlgo_SHA,     kSE05x_ECSignatureAlgo_SHA_224,
        kSE05x_ECSignatureAlgo_SHA_256, kSE05x_ECSignatureAlgo_SHA_384,
        kSE05x_ECSignatureAlgo_SHA_512
};

static const uint8_t scp03_enc_key[SCP03_KEY_SIZE] = SCP03_ENC_KEY;
static const uint8_t scp03_mac_key[SCP03_KEY_SIZE] = SCP03_MAC_KEY;
static const uint8_t scp03_dek_key[SCP03_KEY_SIZE] = SCP03_DEK_KEY;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool set_enable_pin(FAR struct se05x_dev_s *se05x, bool state)
{
  if (se05x->config->set_enable_pin == NULL)
    {
      return FALSE;
    }

  return se05x->config->set_enable_pin(state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pnt_se05x_open(FAR struct se05x_dev_s *se05x)
{
  se05x->pnt = kmm_malloc(sizeof(struct pnt_handle));
  int ret = se05x->pnt != NULL ? 0 : -EIO;

  if (ret == 0)
    {
      memset(&(se05x->pnt->session), 0, sizeof(Se05xSession_t));

      se05x->pnt->session.pScp03_enc_key = (FAR uint8_t *)scp03_enc_key;
      se05x->pnt->session.pScp03_mac_key = (FAR uint8_t *)scp03_mac_key;
      se05x->pnt->session.pScp03_dek_key = (FAR uint8_t *)scp03_dek_key;
      se05x->pnt->session.scp03_enc_key_len = SCP03_KEY_SIZE;
      se05x->pnt->session.scp03_mac_key_len = SCP03_KEY_SIZE;
      se05x->pnt->session.scp03_dek_key_len = SCP03_KEY_SIZE;
      ret = set_enable_pin(se05x, true) ? 0 : -EIO;
    }

  if (ret == 0)
    {
      se05x->pnt->session.skip_applet_select = 0;
      se05x->pnt->session.session_resume = 0;
      smStatus_t status =
          Se05x_API_SessionOpen(&(se05x->pnt->session), se05x);
      ret = status == SM_OK ? 0 : -EIO;
    }

  /* if error */

  if (ret < 0)
    {
      if (se05x->pnt->session.conn_context != NULL)
        {
          kmm_free(se05x->pnt->session.conn_context);
        }

      if (se05x->pnt != NULL)
        {
          kmm_free(se05x->pnt);
        }
    }

  return ret;
}

void pnt_se05x_close(FAR struct se05x_dev_s *se05x)
{
  Se05x_API_SessionClose(&(se05x->pnt->session));
  (void)set_enable_pin(se05x, FALSE);
  kmm_free(se05x->pnt);
}

int pnt_se05x_get_info(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_info_s *se05x_info)
{
  bool result = select_card_manager(&(se05x->pnt->session));
  identify_rsp_t identify_response;
  if (result)
    {
      result = se05x_identify(&(se05x->pnt->session), &identify_response);
    }

  if (result)
    {
      se05x_info->oef_id = (identify_response.configuration_id[2] << 8) +
                           identify_response.configuration_id[3];
    }

  return result ? 0 : -EIO;
}

int pnt_se05x_get_uid(FAR struct se05x_dev_s *se05x,
                      FAR struct se05x_uid_s *uid)
{
  SE05x_Result_t dummy = kSE05x_Result_NA;
  size_t uid_size = SE050_MODULE_UNIQUE_ID_LEN;

  smStatus_t status = Se05x_API_CheckObjectExists(
      &(se05x->pnt->session), KSE05X_APPLETRESID_UNIQUE_ID, &dummy);
  int result = status == SM_OK ? 0 : -ENODATA;
  if (result == 0)
    {
      status = Se05x_API_ReadObject(&(se05x->pnt->session),
                                    KSE05X_APPLETRESID_UNIQUE_ID, 0,
                                    (uint16_t)uid_size, uid->uid, &uid_size);
      result = status == SM_OK ? 0 : -EIO;
    }

  return result;
}

int pnt_se05x_generate_keypair(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_generate_keypair_s *generate_keypair_args)
{
  smStatus_t status = Se05x_API_WriteECKey(
      &(se05x->pnt->session), NULL, 0, generate_keypair_args->id,
      kSE05x_ECCurve_NIST_P256, NULL, 0, NULL, 0, kSE05x_INS_NA,
      kSE05x_KeyPart_Pair);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_set_public_key(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args)
{
  smStatus_t status = Se05x_API_WriteECKey(
      &(se05x->pnt->session), NULL, 0, set_publickey_args->entry.id,
      kSE05x_ECCurve_NIST_P256, NULL, 0, set_publickey_args->content.buffer,
      set_publickey_args->content.buffer_size, kSE05x_INS_NA,
      kSE05x_KeyPart_Public);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_set_data(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args)
{
  size_t remainder = set_publickey_args->content.buffer_size;
  smStatus_t status = SM_OK;
  uint16_t offset = 0;
  bool first_cycle = TRUE;

  while ((remainder > 0) && (status == SM_OK))
    {
      size_t chunk_size =
          remainder > DATA_CHUNK_SIZE ? DATA_CHUNK_SIZE : remainder;
      status = Se05x_API_WriteBinary(
          &(se05x->pnt->session), NULL, set_publickey_args->entry.id, offset,
          first_cycle ? set_publickey_args->content.buffer_size : 0,
          set_publickey_args->content.buffer + offset, chunk_size);
      remainder -= chunk_size;
      offset += chunk_size;
      first_cycle = FALSE;
    }

  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_get_key(FAR struct se05x_dev_s *se05x,
                      FAR struct se05x_key_transmission_s *get_key_args)
{
  get_key_args->content.buffer_content_size =
      get_key_args->content.buffer_size;
  smStatus_t status =
      Se05x_API_ReadObject(&(se05x->pnt->session), get_key_args->entry.id, 0,
                           0, get_key_args->content.buffer,
                           &get_key_args->content.buffer_content_size);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_get_data(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_key_transmission_s *get_key_args)
{
  uint16_t remainder;
  smStatus_t status = Se05x_API_ReadSize(&(se05x->pnt->session),
                                         get_key_args->entry.id, &remainder);

  if (remainder > get_key_args->content.buffer_size)
    {
      status = SM_NOT_OK;
    }

  uint16_t offset = 0;

  while ((remainder > 0) && (status == SM_OK))
    {
      size_t chunk_size =
          remainder > DATA_CHUNK_SIZE ? DATA_CHUNK_SIZE : remainder;
      status = Se05x_API_ReadObject(
          &(se05x->pnt->session), get_key_args->entry.id, offset, chunk_size,
          get_key_args->content.buffer + offset, &chunk_size);
      remainder -= chunk_size;
      offset += chunk_size;
    }

  get_key_args->content.buffer_content_size = offset;
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_delete_key(FAR struct se05x_dev_s *se05x, uint32_t key_id)
{
  smStatus_t status =
      Se05x_API_DeleteSecureObject(&(se05x->pnt->session), key_id);
  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_derive_key(FAR struct se05x_dev_s *se05x,
                         FAR struct se05x_derive_key_s *derive_key_args)
{
  uint8_t public_key[65];
  size_t public_key_size = sizeof(public_key);
  smStatus_t status = Se05x_API_ReadObject(&(se05x->pnt->session),
                                           derive_key_args->public_key_id, 0,
                                           0, public_key, &public_key_size);

  if (status == SM_OK)
    {
      derive_key_args->content.buffer_content_size =
          derive_key_args->content.buffer_size;
      status = Se05x_API_ECDHGenerateSharedSecret(
          &(se05x->pnt->session), derive_key_args->private_key_id,
          public_key, public_key_size, derive_key_args->content.buffer,
          &derive_key_args->content.buffer_content_size);
    }

  return status == SM_OK ? 0 : -EIO;
}

int pnt_se05x_create_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *create_signature_args)
{
  create_signature_args->signature.buffer_content_size =
      create_signature_args->signature.buffer_size;
  int result =
      Se05x_API_ECDSASign(
          &(se05x->pnt->session), create_signature_args->key_id,
          signature_algorithm_mapping[create_signature_args->algorithm],
          create_signature_args->tbs.buffer,
          create_signature_args->tbs.buffer_content_size,
          create_signature_args->signature.buffer,
          &create_signature_args->signature.buffer_content_size) == SM_OK
          ? 0
          : -EIO;

  return result;
}

int pnt_se05x_verify_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *verify_signature_args)
{
  SE05x_Result_t se05x_result;
  int result =
      Se05x_API_ECDSAVerify(
          &(se05x->pnt->session), verify_signature_args->key_id,
          signature_algorithm_mapping[verify_signature_args->algorithm],
          verify_signature_args->tbs.buffer,
          verify_signature_args->tbs.buffer_content_size,
          verify_signature_args->signature.buffer,
          verify_signature_args->signature.buffer_content_size,
          &se05x_result) == SM_OK
          ? 0
          : -EACCES;

  if ((result == 0) && (se05x_result != kSE05x_Result_SUCCESS))
    {
      result = -EIO;
    }

  return result;
}
