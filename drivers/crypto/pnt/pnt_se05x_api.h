/****************************************************************************
 * drivers/crypto/pnt/pnt_se05x_api.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_PNT_PNT_API_H_
#define __INCLUDE_NUTTX_CRYPTO_PNT_PNT_API_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/crypto/se05x.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct se05x_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int pnt_se05x_open(FAR struct se05x_dev_s *se05x);
void pnt_se05x_close(FAR struct se05x_dev_s *se05x);

/****************************************************************************
 * Name: pnt_se05x_get_info
 *
 * Description:
 *   Get information on the variant of the SE05x
 *
 * Input Parameters:
 *   se05x        - Ptr to se05x device struct
 *   se05x_info   - Ptr to storage of retrieved SE050 info.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_get_info(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_info_s *se05x_info);

/****************************************************************************
 * Name: pnt_se05x_get_uid
 *
 * Description:
 *   Get the unique id of the SE05x
 *
 * Input Parameters:
 *   se05x        - Ptr to se05x device struct
 *   se05x_uid    - Ptr to storage for the unique id
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_get_uid(FAR struct se05x_dev_s *se05x,
                      FAR struct se05x_uid_s *uid);

/****************************************************************************
 * Name: pnt_se05x_generate_keypair
 *
 * Description:
 *   Generate a private/public keypair.
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   generate_keypair_args - Ptr to arguments needed to generate keypair
 *   (input)
 *     ->id               id where to store key
 *     ->cipher           cipher type
 *                        (defaults to se05x_asym_cipher_EC_NIST_P_256)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_generate_keypair(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_generate_keypair_s *generate_keypair_args);

/****************************************************************************
 * Name: pnt_se05x_set_public_key
 *
 * Description:
 *   Store key into keystore. Key must be in raw format
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   set_publickey_args    - Ptr to arguments needed to set public key
 *   (input)
 *     ->entry.id         id where to store key
 *     ->entry.cipher     cipher type
 *                        (defaults to se05x_asym_cipher_EC_NIST_P_256)
 *     ->content          assign with public key
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_set_public_key(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args);

/****************************************************************************
 * Name: pnt_se05x_set_data
 *
 * Description:
 *   Store data into keystore.
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   set_publickey_args    - Ptr to arguments needed to set data
 *   (input)
 *     ->entry.id         id where to store key
 *     ->entry.cipher     (not used)
 *     ->content          assign with data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_set_data(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *set_publickey_args);

/****************************************************************************
 * Name: pnt_se05x_get_key
 *
 * Description:
 *   Get the key from keystore. Key is returned in raw format
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   get_publickey_args    - Ptr to arguments needed to get public key
 *   (input)
 *     ->entry.id        id where to get key
 *   (output)
 *     ->entry.type      key type
 *     ->content         public key will be copied into
 *                       buffer. Must be allocated!
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_get_key(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_key_transmission_s *get_publickey_args);

/****************************************************************************
 * Name: pnt_se05x_get_data
 *
 * Description:
 *   Get data from keystore. The returned data is raw data
 *
 * Input Parameters:
 *   se05x               - Ptr to se05x device struct
 *   get_data_args       - Ptr to arguments needed to get public key
 *   (input)
 *     ->entry.id        id where to get key
 *   (output)
 *     ->entry.type      key type
 *     ->content         data will be copied into
 *                       buffer. Must be allocated!
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_get_data(FAR struct se05x_dev_s *se05x,
                       FAR struct se05x_key_transmission_s *get_data_args);

/****************************************************************************
 * Name: pnt_se05x_delete_key
 *
 * Description:
 *   Delete key from keystore
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   key_id                - key ID
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_delete_key(FAR struct se05x_dev_s *se05x, uint32_t key_id);

/****************************************************************************
 * Name: pnt_se05x_derive_key
 *
 * Description:
 *   Derive a symmetric key using a private key of an owned keypair and
 *   a public key that is received from the peer.
 *   Currently only keys derived from ecdsa keys are supported
 *
 * Input Parameters:
 *   se05x              - Ptr to se05x device struct
 *   derive_key_args    - Ptr to arguments needed to derive symmetric key
 *   (input)
 *     ->private_key_id  id of entry in keystore to get private key
 *                       This entry may be a keypair
 *     ->public_key_id   id of entry in keystore to get public key
 *   (output)
 *     ->content         symmetric key will be copied into
 *                       buffer. Must be allocated! (>= 32 bytes)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_derive_key(FAR struct se05x_dev_s *se05x,
                         FAR struct se05x_derive_key_s *derive_key_args);

/****************************************************************************
 * Name: pnt_se05x_create_signature
 *
 * Description:
 *   Create a signature, using a hash value and a private key.
 *   To create the signature the hash value is encrypted using the private
 *key.
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   create_signature_args - Ptr to arguments needed to create signature
 *   (input)
 *     ->key_id          id where to get private key
 *     ->algorithm       hash algorithm
 *     ->tbs             digest
 *   (output)
 *     ->signature       the generated signature. Must be allocated!
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_create_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *create_signature_args);

/****************************************************************************
 * Name: pnt_se05x_verify_signature
 *
 * Description:
 *   Verify a signature, using a hash value, a signature and a public key.
 *   To verify the signature, the provided signature is decrypted using the
 *   public key. The results in the original hash value which should be the
 *   same as the provided hash value.
 *
 * Input Parameters:
 *   se05x                 - Ptr to se05x device struct
 *   verify_signature_args - Ptr to arguments needed to create signature
 *   (input)
 *     ->key_id          id where to get public key
 *     ->algorithm       hash algorithm
 *     ->tbs             digest
 *     ->signature       the signature
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pnt_se05x_verify_signature(
    FAR struct se05x_dev_s *se05x,
    FAR struct se05x_signature_s *verify_signature_args);

#endif /* __INCLUDE_NUTTX_CRYPTO_PNT_PNT_API_H_ */
