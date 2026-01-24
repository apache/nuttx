/****************************************************************************
 * crypto/cryptosoft.c
 *
 * SPDX-License-Identifier: OAR
 * SPDX-FileCopyrightText: Copyright (c) 2000, 2001 Angelos D. Keromytis
 * SPDX-FileContributor: Angelos D. Keromytis (angelos@cis.upenn.edu)
 *
 * Permission to use, copy, and modify this software with or without fee
 * is hereby granted, provided that this entire notice is included in
 * all source code copies of any software which is or includes a copy or
 * modification of this software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTY. IN PARTICULAR, NONE OF THE AUTHORS MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE
 * MERCHANTABILITY OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR
 * PURPOSE.
 *
 * This code was written by Angelos D. Keromytis in Athens, Greece, in
 * February 2000. Network Security Technologies Inc. (NSTI) kindly
 * supported the development of this code.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <endian.h>
#include <fcntl.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/configdata.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
#include <crypto/bn.h>
#include <crypto/cryptodev.h>
#include <crypto/cryptosoft.h>
#include <crypto/curve25519.h>
#include <crypto/ecc.h>
#include <crypto/xform.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT

#define SWKEY_MAGIC_STRING  "SWKEYMGMT"
#define SWKEY_FILL_NAME(name, keyid, type)       \
  do                                             \
    {                                            \
      snprintf(name, sizeof(name), "%s.%lu.%s",  \
               SWKEY_MAGIC_STRING, keyid, type); \
    }                                            \
  while (0)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct swkey_data_s
{
  uint32_t id;
  uint32_t size;
  uint32_t flags;
  uint8_t buf[CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE];
  TAILQ_ENTRY(swkey_data_s) next;
};

struct swkey_context_s
{
  struct file file;
  TAILQ_HEAD(swkey_list, swkey_data_s) head;
};

#endif /* CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO

FAR struct swcr_data **swcr_sessions = NULL;
uint32_t swcr_sesnum = 0;
int swcr_id = -1;

#endif /* CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT

/* key data operations in flash */

/****************************************************************************
 * Name: swkey_write
 *
 * Description:
 *   Storing key data into flash and mapping it to the keyid
 *
 ****************************************************************************/

static int swkey_write(FAR struct file *filep, uint32_t keyid,
                       FAR const void *data, uint32_t len, int flags)
{
  struct config_data_s config;
  int ret;

  if (keyid == 0 || data == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Write the data and flags to the Flash */

  memset(&config, 0, sizeof(config));
  SWKEY_FILL_NAME(config.name, keyid, "data");
  config.len = len;
  config.configdata = (uint8_t *)data;
  ret = file_ioctl(filep, CFGDIOC_SETCONFIG, &config);
  if (ret < 0)
    {
      return ret;
    }

  SWKEY_FILL_NAME(config.name, keyid, "flags");
  config.len = sizeof(uint32_t);
  config.configdata = (uint8_t *)&flags;
  return file_ioctl(filep, CFGDIOC_SETCONFIG, &config);
}

/****************************************************************************
 * Name: swkey_remove
 *
 * Description:
 *   Removing key data from flash
 *
 ****************************************************************************/

static int swkey_remove(FAR struct file *filep, uint32_t keyid)
{
  struct config_data_s config;
  int ret;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  /* Remove the flags and data */

  memset(&config, 0, sizeof(config));
  SWKEY_FILL_NAME(config.name, keyid, "flags");
  ret = file_ioctl(filep, CFGDIOC_DELCONFIG, &config);
  if (ret < 0)
    {
      return ret;
    }

  memset(config.name, 0, sizeof(config.name));
  SWKEY_FILL_NAME(config.name, keyid, "data");
  return file_ioctl(filep, CFGDIOC_DELCONFIG, &config);
}

/****************************************************************************
 * Name: swkey_get_flags
 *
 * Description:
 *   Getting key flags from flash
 *
 ****************************************************************************/

static int swkey_get_flags(FAR struct file *filep, uint32_t keyid,
                           FAR uint32_t *flags)
{
  struct config_data_s config;

  if (keyid == 0 || flags == NULL)
    {
      return -EINVAL;
    }

  memset(&config, 0, sizeof(config));
  SWKEY_FILL_NAME(config.name, keyid, "flags");
  config.len = sizeof(uint32_t);
  config.configdata = (uint8_t *)flags;
  return file_ioctl(filep, CFGDIOC_GETCONFIG, &config);
}

/****************************************************************************
 * Name: swkey_read
 *
 * Description:
 *   Getting key data from flash
 *
 ****************************************************************************/

static int swkey_read(FAR struct file *filep, uint32_t keyid,
                      FAR void *buf, uint32_t buflen)
{
  struct config_data_s config;
  int ret;

  if (keyid == 0 || buf == NULL)
    {
      return -EINVAL;
    }

  memset(&config, 0, sizeof(config));
  SWKEY_FILL_NAME(config.name, keyid, "data");
  config.len = buflen;
  config.configdata = buf;
  ret = file_ioctl(filep, CFGDIOC_GETCONFIG, &config);
  if (ret < 0)
    {
      return ret;
    }

  return config.len;
}

/* key data operations in cache */

/****************************************************************************
 * Name: swkey_get_context
 *
 * Description:
 *   Access key cache list entries
 *
 ****************************************************************************/

static FAR struct swkey_context_s *swkey_get_context(void)
{
  FAR struct swkey_context_s *ctx;
  int swkey_id;

  swkey_id = crypto_find_driverid(CRYPTOCAP_F_KEY_MGMT);
  if (swkey_id < 0)
    {
      return NULL;
    }

  ctx = (FAR struct swkey_context_s *)crypto_driver_get_priv(swkey_id);
  if (ctx == NULL)
    {
      return NULL;
    }

  if (ctx->file.f_inode == NULL)
    {
      if (file_open(&ctx->file,
                    CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_DEVICE,
                    O_RDWR | O_CLOEXEC) < 0)
        {
          return NULL;
        }
    }

  return ctx;
}

/****************************************************************************
 * Name: swkey_get_cache_data
 *
 * Description:
 *   Acquire an available key slot in cache. If the key exists in the cache,
 * utilize that slot immediately; otherwise, locate the last slot.
 *
 ****************************************************************************/

static FAR struct swkey_data_s *
swkey_get_cache_data(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  FAR struct swkey_data_s *data;

  TAILQ_FOREACH(data, &ctx->head, next)
    {
      if (data->id == keyid)
        {
          break;
        }
    }

  if (data == NULL)
    {
      data = TAILQ_LAST(&ctx->head, swkey_list);
      if (data->id)
        {
          swkey_write(&ctx->file, data->id, data->buf,
                      data->size, data->flags);
        }
    }

  return data;
}

/****************************************************************************
 * Name: swkey_promote_cache_data
 *
 * Description:
 *   Update the key cache linked list.
 *   Move the accessed key cache to the head position to ensure
 *   the most frequently used keys remain cached.
 *
 ****************************************************************************/

static void swkey_promote_cache_data(FAR struct swkey_context_s *ctx,
                                     FAR struct swkey_data_s *data)
{
  TAILQ_REMOVE(&ctx->head, data, next);
  TAILQ_INSERT_HEAD(&ctx->head, data, next);
}

/* key management operations */

/****************************************************************************
 * Name: swkey_clean_cache_data
 *
 * Description:
 *   Clean the cache slot
 *
 ****************************************************************************/

static void swkey_clean_cache_data(FAR struct swkey_data_s *data)
{
  explicit_bzero(data->buf, sizeof(data->buf));
  data->id = 0;
  data->size = 0;
  data->flags = 0;
}

/****************************************************************************
 * Name: swkey_is_valid
 *
 * Description:
 *   Check whether the given keyid is available in the driver
 *
 ****************************************************************************/

static int swkey_is_valid(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  uint32_t flags;
  int ret;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  ret = swkey_get_flags(&ctx->file, keyid, &flags);
  if (ret == -ENOENT)
    {
      /* No such file means keyid unused and available
       * and occupied this keyid with MAGIC-STRING
       */

      return swkey_write(&ctx->file, keyid, SWKEY_MAGIC_STRING,
                                     sizeof(SWKEY_MAGIC_STRING), 0);
    }
  else if (ret == 0)
    {
      /* success means keyid used and unavailable */

      return -EEXIST;
    }

  return ret;
}

/****************************************************************************
 * Name: swkey_alloc
 *
 * Description:
 *   Acquire an available key ID from the driver
 *
 ****************************************************************************/

static int swkey_alloc(FAR struct swkey_context_s *ctx,
                       FAR uint32_t *keyid)
{
  int i;

  for (i = 1; i <= CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_NKEYS; i++)
    {
      if (swkey_is_valid(ctx, i) == 0)
        {
          *keyid = i;
          return OK;
        }
    }

  return -ENOMEM;
}

/****************************************************************************
 * Name: swkey_import
 *
 * Description:
 *   Import key data into cache key slot and bind to the keyid
 *
 ****************************************************************************/

static int swkey_import(FAR struct swkey_context_s *ctx,
                        uint32_t keyid, FAR void *buf,
                        uint32_t buflen, uint32_t flags)
{
  FAR struct swkey_data_s *data;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  if (buflen > CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE)
    {
      return swkey_write(&ctx->file, keyid, buf, buflen, flags);
    }

  data = swkey_get_cache_data(ctx, keyid);
  data->id = keyid;
  data->size = buflen;
  memcpy(data->buf, buf, data->size);
  if (flags & CRYPTO_F_NOT_EXPORTABLE)
    {
      data->flags |= CRYPTO_F_NOT_EXPORTABLE;
    }
  else
    {
      data->flags &= ~CRYPTO_F_NOT_EXPORTABLE;
    }

  swkey_promote_cache_data(ctx, data);
  return OK;
}

/****************************************************************************
 * Name: swkey_delete
 *
 * Description:
 *   Remove a specific key by keyid
 *
 ****************************************************************************/

static int swkey_delete(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  FAR struct swkey_data_s *data;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  data = swkey_get_cache_data(ctx, keyid);
  if (data->id == keyid)
    {
      swkey_clean_cache_data(data);
    }

  return swkey_remove(&ctx->file, keyid);
}

/****************************************************************************
 * Name: swkey_export
 *
 * Description:
 *   Export key data by keyid
 *
 ****************************************************************************/

static int swkey_export(FAR struct swkey_context_s *ctx,
                        uint32_t keyid, FAR void *buf,
                        uint32_t buflen)
{
  FAR struct swkey_data_s *data;
  uint32_t flags;
  int ret;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  data = swkey_get_cache_data(ctx, keyid);
  if (data->id == keyid)
    {
      /* Key in cache, export data and update cache */

      if (data->flags & CRYPTO_F_NOT_EXPORTABLE)
        {
          return -EACCES;
        }

      if (buflen < data->size)
        {
          return -ENOBUFS;
        }

      memcpy(buf, data->buf, data->size);
      swkey_promote_cache_data(ctx, data);
      return data->size;
    }

  /* Key not in cache, get key from flash */

  ret = swkey_get_flags(&ctx->file, keyid, &flags);
  if (ret < 0)
    {
      return ret;
    }

  if (flags & CRYPTO_F_NOT_EXPORTABLE)
    {
      return -EACCES;
    }

  ret = swkey_read(&ctx->file, keyid, buf, buflen);
  if (ret < 0)
    {
      return ret;
    }
  else if (ret > buflen)
    {
      return -ENOBUFS;
    }
  else if (memcmp(buf, SWKEY_MAGIC_STRING, ret) == 0)
    {
      return -ENOENT;
    }

  if (ret < CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE)
    {
      data->id = keyid;
      data->size = ret;
      data->flags = flags;
      memcpy(data->buf, buf, ret);
      swkey_promote_cache_data(ctx, data);
    }

  return ret;
}

/****************************************************************************
 * Name: swkey_gen_secp256r1_key
 *
 * Description:
 *   Generate SECP256R1 keypair and bound with keyid
 *
 ****************************************************************************/

static int swkey_gen_secp256r1_key(FAR struct swkey_context_s *ctx,
                                   uint32_t priv_keyid,
                                   uint32_t pub_keyid)
{
  FAR struct swkey_data_s *data;
  uint8_t priv[secp256r1];
  uint8_t pub[secp256r1 * 2];
  int ret = -EINVAL;

  if (priv_keyid == 0 || pub_keyid == 0)
    {
      return ret;
    }

  if (ecc_make_key_uncomp(pub, pub + secp256r1, priv) == 0)
    {
      return ret;
    }

  /* Private keys cannot be exported */

  ret = swkey_write(&ctx->file, priv_keyid, priv, secp256r1,
                    CRYPTO_F_NOT_EXPORTABLE);
  if (ret < 0)
    {
      return ret;
    }

  ret = swkey_write(&ctx->file, pub_keyid, pub, secp256r1 * 2, 0);
  if (ret < 0)
    {
      swkey_delete(ctx, priv_keyid);
      return ret;
    }

  if (CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE >= secp256r1)
    {
      data = swkey_get_cache_data(ctx, priv_keyid);
      data->id = priv_keyid;
      data->size = secp256r1;
      data->flags = CRYPTO_F_NOT_EXPORTABLE;
      memcpy(data->buf, priv, secp256r1);
      swkey_promote_cache_data(ctx, data);
    }

  if (CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE >= secp256r1 * 2)
    {
      data = swkey_get_cache_data(ctx, pub_keyid);
      data->id = pub_keyid;
      data->size = secp256r1 * 2;
      data->flags = 0;
      memcpy(data->buf, pub, secp256r1 * 2);
      swkey_promote_cache_data(ctx, data);
    }

  return ret;
}

/****************************************************************************
 * Name: swkey_gen_aes_key
 *
 * Description:
 *   Generate AES key and bound with keyid
 *
 ****************************************************************************/

static int swkey_gen_aes_key(FAR struct swkey_context_s *ctx, uint32_t keyid,
                             uint32_t keylen)
{
  FAR struct swkey_data_s *data;
  int ret = -EINVAL;
  char buf[32];

  if (keyid == 0)
    {
      return ret;
    }

  /* Generate a key sufficient for AES-128/192/256 */

  arc4random_buf(buf, keylen);
  ret = swkey_write(&ctx->file, keyid, buf, keylen, 0);
  if (ret < 0)
    {
      return ret;
    }

  if (keylen <= CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE)
    {
      data = swkey_get_cache_data(ctx, keyid);
      data->id = keyid;
      data->size = keylen;
      data->flags = 0;
      memcpy(data->buf, buf, keylen);
      swkey_promote_cache_data(ctx, data);
    }

  return ret;
}

/****************************************************************************
 * Name: swkey_save
 *
 * Description:
 *   Write key from cache to Flash
 *
 ****************************************************************************/

static int swkey_save(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  FAR struct swkey_data_s *data;
  int ret = -EINVAL;

  if (keyid == 0)
    {
      return ret;
    }

  data = swkey_get_cache_data(ctx, keyid);
  if (data->id == keyid)
    {
      ret = swkey_write(&ctx->file, keyid, data->buf,
                        data->size, data->flags);
      if (ret < 0)
        {
          return ret;
        }

      swkey_promote_cache_data(ctx, data);
    }

  return ret;
}

/****************************************************************************
 * Name: crypto_load_key
 *
 * Description:
 *   Load key data from Flash to cache
 *
 ****************************************************************************/

static int swkey_load(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  FAR struct swkey_data_s *data;
  char buf[CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE];
  int readlen;

  if (keyid == 0)
    {
      return -EINVAL;
    }

  readlen = swkey_read(&ctx->file, keyid, buf, sizeof(buf));
  if (readlen < 0)
    {
      return readlen;
    }
  else if (readlen > CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_BUFSIZE)
    {
      return -EFBIG;
    }

  data = swkey_get_cache_data(ctx, keyid);
  data->id = keyid;
  data->size = readlen;
  swkey_get_flags(&ctx->file, keyid, &data->flags);
  memcpy(data->buf, buf, data->size);
  swkey_promote_cache_data(ctx, data);
  return OK;
}

/****************************************************************************
 * Name: swkey_unload
 *
 * Description:
 *   Unload key data from cache
 *
 ****************************************************************************/

static int swkey_unload(FAR struct swkey_context_s *ctx, uint32_t keyid)
{
  FAR struct swkey_data_s *data;
  int ret = -EINVAL;

  if (keyid == 0)
    {
      return ret;
    }

  data = swkey_get_cache_data(ctx, keyid);
  if (data->id == keyid)
    {
      ret = swkey_write(&ctx->file, data->id, data->buf,
                        data->size, data->flags);
      if (ret < 0)
        {
          return ret;
        }

      swkey_clean_cache_data(data);
    }

  return ret;
}

/****************************************************************************
 * Name: swkey_kprocess
 *
 * Description:
 *   Key management process function in crypto driver
 *
 ****************************************************************************/

static int swkey_kprocess(FAR struct cryptkop *krp)
{
  FAR struct swkey_context_s *ctx;
  uint32_t priv_keyid;
  uint32_t pub_keyid;
  uint32_t keylen;
  uint32_t keyid;
  int ret;

  /* Sanity check */

  if (krp == NULL)
    {
      return -EINVAL;
    }

  ctx = swkey_get_context();
  if (ctx == NULL)
    {
      return -EINVAL;
    }

  if (krp->krp_param[0].crp_nbits != sizeof(uint32_t) * 8)
    {
      return -EINVAL;
    }

  keyid = *(uint32_t *)krp->krp_param[0].crp_p;

  /* Go through crypto descriptors, processing as we go */

  switch (krp->krp_op)
    {
      case CRK_ALLOCATE_KEY:
        krp->krp_status = swkey_alloc(ctx, &keyid);
        if (krp->krp_status == 0)
          {
            memcpy(krp->krp_param[0].crp_p, &keyid, sizeof(uint32_t));
          }

        break;
      case CRK_VALIDATE_KEYID:
        krp->krp_status = swkey_is_valid(ctx, keyid);
        break;
      case CRK_IMPORT_KEY:
        krp->krp_status = swkey_import(ctx, keyid,
                                       krp->krp_param[1].crp_p,
                                       krp->krp_param[1].crp_nbits / 8,
                                       krp->krp_flags);
        break;
      case CRK_DELETE_KEY:
        krp->krp_status = swkey_delete(ctx, keyid);
        break;
      case CRK_EXPORT_KEY:
        ret = swkey_export(ctx, keyid,
                           krp->krp_param[1].crp_p,
                           krp->krp_param[1].crp_nbits / 8);
        if (ret < 0)
          {
            krp->krp_status = ret;
          }
        else
          {
            krp->krp_param[1].crp_nbits = ret * 8;
          }

        break;
      case CRK_GENERATE_AES_KEY:
        if (krp->krp_param[1].crp_nbits != sizeof(uint32_t) * 8)
          {
            return -EINVAL;
          }

        keylen = *(uint32_t *)krp->krp_param[1].crp_p;
        if (keylen != 16 && keylen != 24 && keylen != 32)
          {
            return -EINVAL;
          }

        krp->krp_status = swkey_gen_aes_key(ctx, keyid, keylen);
        break;
      case CRK_GENERATE_SECP256R1_KEY:
        priv_keyid = keyid;
        if (krp->krp_param[1].crp_nbits != sizeof(uint32_t) * 8)
          {
            return -EINVAL;
          }

        pub_keyid = *(uint32_t *)krp->krp_param[1].crp_p;
        krp->krp_status = swkey_gen_secp256r1_key(ctx, priv_keyid,
                                                       pub_keyid);
        break;
      case CRK_SAVE_KEY:
        krp->krp_status = swkey_save(ctx, keyid);
        break;
      case CRK_LOAD_KEY:
        krp->krp_status = swkey_load(ctx, keyid);
        break;
      case CRK_UNLOAD_KEY:
        krp->krp_status = swkey_unload(ctx, keyid);
        break;
      default:

        /* Unknown/unsupported operation */

        krp->krp_status = -EINVAL;
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: swkey_context_init
 *
 * Description:
 *   Init software key ctx
 *
 ****************************************************************************/

static int swkey_context_init(FAR struct swkey_context_s *ctx)
{
  FAR struct swkey_data_s *data;
  int i;

  TAILQ_INIT(&ctx->head);
  for (i = 0; i < CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT_NSLOTS; i++)
    {
      data = (FAR struct swkey_data_s *)kmm_zalloc(sizeof(*data));
      if (data == NULL)
        {
          return -ENOMEM;
        }

      TAILQ_INSERT_HEAD(&ctx->head, data, next);
    }

  return OK;
}

/****************************************************************************
 * Name: swkey_context_cleanup
 *
 * Description:
 *   Cleanup software key ctx
 *
 ****************************************************************************/

static void swkey_context_cleanup(FAR struct swkey_context_s *ctx)
{
  FAR struct swkey_data_s *data;

  TAILQ_FOREACH(data, &ctx->head, next)
    {
      memset(data, 0, sizeof(struct swkey_data_s));
      kmm_free(data);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* key management operations */

/****************************************************************************
 * Name: swkey_init
 *
 * Description:
 *   Register software key management driver
 *
 ****************************************************************************/

void swkey_init(void)
{
  int swkey_id = crypto_get_driverid(CRYPTOCAP_F_KEY_MGMT);
  FAR struct swkey_context_s *ctx;
  int kalgs[CRK_ALGORITHM_MAX + 1];

  ctx = (FAR struct swkey_context_s *)kmm_zalloc(sizeof(*ctx));
  if (ctx == NULL)
    {
      return;
    }

  if (swkey_context_init(ctx))
    {
      swkey_context_cleanup(ctx);
      kmm_free(ctx);
      return;
    }

  crypto_driver_set_priv(swkey_id, ctx);

  kalgs[CRK_ALLOCATE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_VALIDATE_KEYID] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_IMPORT_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_DELETE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_EXPORT_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_GENERATE_AES_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_GENERATE_SECP256R1_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_SAVE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_LOAD_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_UNLOAD_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_kregister(swkey_id, kalgs, swkey_kprocess);
}

#endif /* CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT */

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO

/* Apply a symmetric encryption/decryption algorithm. */

int swcr_encdec(FAR struct cryptop *crp, FAR struct cryptodesc *crd,
                FAR struct swcr_data *sw, caddr_t buf)
{
  FAR char *output;
  unsigned char blk[EALG_MAX_BLOCK_LEN];
  FAR unsigned char *iv;
  FAR unsigned char *ivp;
  FAR unsigned char *nivp;
  unsigned char iv2[EALG_MAX_BLOCK_LEN];
  FAR const struct enc_xform *exf;
  int i;
  int j;
  int blks;
  int ivlen;

  exf = sw->sw_exf;
  blks = exf->blocksize;
  ivlen = exf->ivsize;

  /* Initialize the IV */

  if (crd->crd_flags & CRD_F_ENCRYPT)
    {
      /* Do we need to write the IV */

      if (!(crd->crd_flags & CRD_F_IV_PRESENT))
        {
          arc4random_buf(crd->crd_iv, ivlen);
          bcopy(crd->crd_iv, buf + crd->crd_inject, ivlen);
        }
    }
  else
    {
      /* Decryption */

      /* IV explicitly provided ? */

      if (!(crd->crd_flags & CRD_F_IV_EXPLICIT))
        {
          /* Get IV off buf */

          bcopy(buf + crd->crd_inject, crd->crd_iv, ivlen);
        }
    }

  iv = crd->crd_iv;
  ivp = iv;

  /* xforms that provide a reinit method perform all IV
   * handling themselves.
   */

  if (exf->reinit)
    {
      exf->reinit((caddr_t)sw->sw_kschedule, iv);
    }

  i = crd->crd_len;

  buf = buf + crd->crd_skip;
  output = crp->crp_dst;
  while (i > 0)
    {
      bcopy(buf, blk, exf->blocksize);
      buf += exf->blocksize;
      if (exf->reinit)
        {
          if (crd->crd_flags & CRD_F_ENCRYPT)
            {
              exf->encrypt((caddr_t)sw->sw_kschedule,
                  blk);
            }
          else
            {
              exf->decrypt((caddr_t)sw->sw_kschedule,
                  blk);
            }
        }
      else if (crd->crd_flags & CRD_F_ENCRYPT)
        {
          /* XOR with previous block */

          for (j = 0; j < blks; j++)
            blk[j] ^= ivp[j];

          exf->encrypt((caddr_t)sw->sw_kschedule, blk);

          /* Keep encrypted block for XOR'ng
           * with next block
           */

          bcopy(blk, iv, blks);
          ivp = iv;
        }
      else
        {
          /* decrypt */

          /* Keep encrypted block for XOR'ing
           * with next block
           */

          nivp = (ivp == iv) ? iv2 : iv;
          bcopy(blk, nivp, blks);

          exf->decrypt((caddr_t)sw->sw_kschedule, blk);

          /* XOR with previous block */

          for (j = 0; j < blks; j++)
            {
              blk[j] ^= ivp[j];
            }

          ivp = nivp;
        }

      bcopy(blk, output, exf->blocksize);
      output += exf->blocksize;

      i -= blks;

      /* Could be done... */

      if (i == 0)
        {
          break;
        }
    }

  bcopy(ivp, crp->crp_iv, ivlen);

  return 0; /* Done with encryption/decryption */
}

/* Compute keyed-hash authenticator. */

int swcr_authcompute(FAR struct cryptop *crp,
                     FAR struct cryptodesc *crd,
                     FAR struct swcr_data *sw,
                     caddr_t buf)
{
  unsigned char aalg[AALG_MAX_RESULT_LEN];
  FAR const struct auth_hash *axf = sw->sw_axf;
  int err;

  if (sw->sw_ictx == 0)
    {
      return -EINVAL;
    }

  err = axf->update(&sw->sw_ctx, (FAR uint8_t *)buf + crd->crd_skip,
                    crd->crd_len);

  if (err)
    {
      return err;
    }

  if (crd->crd_flags & CRD_F_ESN)
    {
      axf->update(&sw->sw_ctx, crd->crd_esn, 4);
    }

  switch (sw->sw_alg)
    {
      case CRYPTO_MD5_HMAC:
      case CRYPTO_SHA1_HMAC:
      case CRYPTO_RIPEMD160_HMAC:
      case CRYPTO_SHA2_256_HMAC:
      case CRYPTO_SHA2_384_HMAC:
      case CRYPTO_SHA2_512_HMAC:
        if (sw->sw_octx == NULL)
          {
            return -EINVAL;
          }

        if (crd->crd_flags & CRD_F_UPDATE)
          {
            break;
          }

        axf->final(aalg, &sw->sw_ctx);
        bcopy(sw->sw_octx, &sw->sw_ctx, axf->ctxsize);
        axf->update(&sw->sw_ctx, aalg, axf->hashsize);
        axf->final((FAR uint8_t *)crp->crp_mac, &sw->sw_ctx);
        bcopy(sw->sw_ictx, &sw->sw_ctx, axf->ctxsize);
        break;
    }

  return 0;
}

int swcr_hash(FAR struct cryptop *crp,
              FAR struct cryptodesc *crd,
              FAR struct swcr_data *sw,
              caddr_t buf)
{
  FAR const struct auth_hash *axf = sw->sw_axf;

  if (crd->crd_flags & CRD_F_UPDATE)
    {
      return axf->update(&sw->sw_ctx, (FAR uint8_t *)buf + crd->crd_skip,
                         crd->crd_len);
    }
  else
    {
      axf->final((FAR uint8_t *)crp->crp_mac, &sw->sw_ctx);
    }

  return 0;
}

/* Apply a combined encryption-authentication transformation */

int swcr_authenc(FAR struct cryptop *crp)
{
  uint32_t blkbuf[div_round_up(EALG_MAX_BLOCK_LEN, sizeof(uint32_t))];
  FAR u_char *blk = (u_char *)blkbuf;
  u_char aalg[AALG_MAX_RESULT_LEN];
  u_char iv[EALG_MAX_BLOCK_LEN];
  union authctx ctx;
  FAR struct cryptodesc *crd;
  FAR struct cryptodesc *crda = NULL;
  FAR struct cryptodesc *crde = NULL;
  FAR struct swcr_data *sw;
  FAR struct swcr_data *swa;
  FAR struct swcr_data *swe = NULL;
  FAR const struct auth_hash *axf = NULL;
  FAR const struct enc_xform *exf = NULL;
  caddr_t buf = (caddr_t)crp->crp_buf;
  caddr_t aad = (caddr_t)crp->crp_aad;
  FAR uint32_t *blkp;
  int aadlen = 0;
  int blksz = 0;
  int ivlen = 0;
  int iskip = 0;
  int oskip = 0;
  int len;
  int i;

  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      for (sw = swcr_sessions[crp->crp_sid & 0xffffffff];
           sw && sw->sw_alg != crd->crd_alg;
           sw = sw->sw_next);

      if (sw == NULL)
        {
          return -EINVAL;
        }

      switch (sw->sw_alg)
        {
          case CRYPTO_AES_GCM_16:
          case CRYPTO_AES_GMAC:
          case CRYPTO_AES_CMAC:
          case CRYPTO_CHACHA20_POLY1305:
          swe = sw;
          crde = crd;
          exf = swe->sw_exf;
          ivlen = exf->ivsize;
          break;
          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:
          case CRYPTO_AES_128_CMAC:
          case CRYPTO_CHACHA20_POLY1305_MAC:
          swa = sw;
          crda = crd;
          axf = swa->sw_axf;
          if (swa->sw_ictx == 0)
            {
              return -EINVAL;
            }

          bcopy(swa->sw_ictx, &ctx, axf->ctxsize);
          blksz = axf->blocksize;
          break;
          default:
            return -EINVAL;
        }
    }

  if (crde == NULL || crda == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the IV */

  if (crde->crd_flags & CRD_F_ENCRYPT)
    {
      /* IV explicitly provided ? */

      if (crde->crd_flags & CRD_F_IV_EXPLICIT)
        {
          bcopy(crde->crd_iv, iv, ivlen);
        }
      else
        {
          arc4random_buf(iv, ivlen);
        }

      if (!((crde->crd_flags) & CRD_F_IV_PRESENT))
        {
          bcopy(iv, buf + crde->crd_inject, ivlen);
        }
    }
  else
    {
        /* Decryption */

        /* IV explicitly provided ? */

      if (crde->crd_flags & CRD_F_IV_EXPLICIT)
        {
          bcopy(crde->crd_iv, iv, ivlen);
        }
      else
        {
          /* Get IV off buf */

          bcopy(iv, buf + crde->crd_inject, ivlen);
        }
    }

  /* Supply MAC with IV */

  if (axf->reinit)
    {
      axf->reinit(&ctx, iv, ivlen);
    }

  /* Supply MAC with AAD */

  if (aad)
    {
      aadlen = crda->crd_len;
      /* Section 5 of RFC 4106 specifies that AAD construction consists of
      * {SPI, ESN, SN} whereas the real packet contains only {SPI, SN}.
      * Unfortunately it doesn't follow a good example set in the Section
      * 3.3.2.1 of RFC 4303 where upper part of the ESN, located in the
      * external (to the packet) memory buffer, is processed by the hash
      * function in the end thus allowing to retain simple programming
      * interfaces and avoid kludges like the one below.
      */

      if (crda->crd_flags & CRD_F_ESN)
        {
          aadlen += 4;

          /* SPI */

          bcopy(buf + crda->crd_skip, blk, 4);
          iskip = 4; /* loop below will start with an offset of 4 */

          /* ESN */

          bcopy(crda->crd_esn, blk + 4, 4);
          oskip = iskip + 4; /* offset output buffer blk by 8 */
        }

      for (i = iskip; i < crda->crd_len; i += axf->hashsize)
        {
          len = MIN(crda->crd_len - i, axf->hashsize - oskip);
          bcopy(buf + crda->crd_skip + i, blk + oskip, len);
          bzero(blk + len + oskip, axf->hashsize - len - oskip);
          axf->update(&ctx, blk, axf->hashsize);
          oskip = 0; /* reset initial output offset */
        }
    }

  if (exf->reinit)
    {
      exf->reinit((caddr_t)swe->sw_kschedule, iv);
    }

  /* Do encryption/decryption with MAC */

  if (buf)
    {
      for (i = 0; i < crde->crd_len; i += blksz)
        {
          len = MIN(crde->crd_len - i, blksz);
          if (len < blksz)
            {
              bzero(blk, blksz);
            }

          bcopy(buf + i, blk, len);
          if (crde->crd_flags & CRD_F_ENCRYPT)
            {
              exf->encrypt((caddr_t)swe->sw_kschedule, blk);
              axf->update(&ctx, blk, len);
            }
          else
            {
              axf->update(&ctx, blk, len);
              exf->decrypt((caddr_t)swe->sw_kschedule, blk);
            }

          if (crp->crp_dst)
            {
              bcopy(blk, crp->crp_dst + i, len);
            }
        }
    }

  /* Do any required special finalization */

  if (crp->crp_mac)
    {
      switch (crda->crd_alg)
        {
          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:

            /* length block */

            bzero(blk, axf->hashsize);
            blkp = (uint32_t *)blk + 1;
            *blkp = htobe32(aadlen * 8);
            blkp = (uint32_t *)blk + 3;
            *blkp = htobe32(crde->crd_len * 8);
            axf->update(&ctx, blk, axf->hashsize);
            break;

          case CRYPTO_CHACHA20_POLY1305_MAC:

            /* length block */

            bzero(blk, axf->hashsize);
            blkp = (uint32_t *)blk;
            *blkp = htole32(aadlen);
            blkp = (uint32_t *)blk + 2;
            *blkp = htole32(crde->crd_len);
            axf->update(&ctx, blk, axf->hashsize);
            break;
        }

      /* Finalize MAC */

      axf->final(aalg, &ctx);

      /* Inject the authentication data */

      bcopy(aalg, crp->crp_mac, axf->authsize);
    }

  return 0;
}

/* Apply a compression/decompression algorithm */

int swcr_compdec(FAR struct cryptodesc *crd, FAR struct swcr_data *sw,
                 caddr_t buf, int outtype)
{
  FAR uint8_t *data;
  FAR uint8_t *out;
  FAR const struct comp_algo *cxf;
  uint32_t result;

  cxf = sw->sw_cxf;

  /* We must handle the whole buffer of data in one time
   * then if there is not all the data in the mbuf, we must
   * copy in a buffer.
   */

  data = kmm_malloc(crd->crd_len);
  if (data == NULL)
    {
      return -EINVAL;
    }

  bcopy(buf + crd->crd_skip, data, crd->crd_len);

  if (crd->crd_flags & CRD_F_COMP)
    {
      result = cxf->compress(data, crd->crd_len, &out);
    }
  else
    {
      result = cxf->decompress(data, crd->crd_len, &out);
    }

  kmm_free(data);
  if (result == 0)
    {
      return -EINVAL;
    }

  sw->sw_size = result;

  /* Check the compressed size when doing compression */

  if (crd->crd_flags & CRD_F_COMP)
    {
      if (result > crd->crd_len)
        {
          /* Compression was useless, we lost time */

          kmm_free(out);
          return 0;
        }
    }

  bcopy(out, buf + crd->crd_skip, result);
  kmm_free(out);
  return 0;
}

/* Generate a new software session. */

int swcr_newsession(FAR uint32_t *sid, FAR struct cryptoini *cri)
{
  FAR struct swcr_data **swd;
  FAR const struct auth_hash *axf;
  FAR const struct enc_xform *txf;
  uint32_t i;
  int k;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  if (swcr_sessions)
    {
      for (i = 1; i < swcr_sesnum; i++)
        {
          if (swcr_sessions[i] == NULL)
            {
              break;
            }
        }
    }

  if (swcr_sessions == NULL || i == swcr_sesnum)
    {
      if (swcr_sessions == NULL)
        {
          i = 1; /* We leave swcr_sessions[0] empty */
          swcr_sesnum = CRYPTO_SW_SESSIONS;
        }
      else
        {
          swcr_sesnum *= 2;
        }

      swd = kmm_calloc(swcr_sesnum, sizeof(struct swcr_data *));
      if (swd == NULL)
        {
          /* Reset session number */

          if (swcr_sesnum == CRYPTO_SW_SESSIONS)
            {
              swcr_sesnum = 0;
            }
          else
            {
              swcr_sesnum /= 2;
            }

          return -ENOBUFS;
        }

      /* Copy existing sessions */

      if (swcr_sessions)
        {
          bcopy(swcr_sessions, swd,
              (swcr_sesnum / 2) * sizeof(struct swcr_data *));
          kmm_free(swcr_sessions);
        }

      swcr_sessions = swd;
    }

  swd = &swcr_sessions[i];
  *sid = i;

  while (cri)
    {
      *swd = kmm_zalloc(sizeof(struct swcr_data));
      if (*swd == NULL)
        {
          swcr_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_3DES_CBC:
            txf = &enc_xform_3des;
            goto enccommon;
          case CRYPTO_BLF_CBC:
            txf = &enc_xform_blf;
            goto enccommon;
          case CRYPTO_CAST_CBC:
            txf = &enc_xform_cast5;
            goto enccommon;
          case CRYPTO_AES_CBC:
            txf = &enc_xform_aes;
            goto enccommon;
          case CRYPTO_AES_CTR:
            txf = &enc_xform_aes_ctr;
            goto enccommon;
          case CRYPTO_AES_XTS:
            txf = &enc_xform_aes_xts;
            goto enccommon;
          case CRYPTO_AES_GCM_16:
            txf = &enc_xform_aes_gcm;
            goto enccommon;
          case CRYPTO_AES_GMAC:
            txf = &enc_xform_aes_gmac;
            (*swd)->sw_exf = txf;
            break;
          case CRYPTO_AES_CMAC:
            txf = &enc_xform_aes_cmac;
            (*swd)->sw_exf = txf;
            break;
          case CRYPTO_AES_OFB:
            txf = &enc_xform_aes_ofb;
            goto enccommon;
          case CRYPTO_AES_CFB_8:
            txf = &enc_xform_aes_cfb_8;
            goto enccommon;
          case CRYPTO_AES_CFB_128:
            txf = &enc_xform_aes_cfb_128;
            goto enccommon;
          case CRYPTO_CHACHA20_POLY1305:
            txf = &enc_xform_chacha20_poly1305;
            goto enccommon;
          case CRYPTO_NULL:
            txf = &enc_xform_null;
            goto enccommon;
          enccommon:
            if (txf->ctxsize > 0)
              {
                (*swd)->sw_kschedule = kmm_zalloc(txf->ctxsize);
                if ((*swd)->sw_kschedule == NULL)
                  {
                    swcr_freesession(i);
                    return -EINVAL;
                  }
              }

            if (cri->cri_klen / 8 > txf->maxkey ||
                cri->cri_klen / 8 < txf->minkey)
              {
                swcr_freesession(i);
                return -EINVAL;
              }

            if (txf->setkey((*swd)->sw_kschedule,
                (FAR uint8_t *)cri->cri_key,
                cri->cri_klen / 8) < 0)
              {
                swcr_freesession(i);
                return -EINVAL;
              }

            (*swd)->sw_exf = txf;
            break;

          case CRYPTO_MD5_HMAC:
            axf = &auth_hash_hmac_md5_96;
            goto authcommon;
          case CRYPTO_SHA1_HMAC:
            axf = &auth_hash_hmac_sha1_96;
            goto authcommon;
          case CRYPTO_RIPEMD160_HMAC:
            axf = &auth_hash_hmac_ripemd_160_96;
            goto authcommon;
          case CRYPTO_SHA2_256_HMAC:
            axf = &auth_hash_hmac_sha2_256_128;
            goto authcommon;
          case CRYPTO_SHA2_384_HMAC:
            axf = &auth_hash_hmac_sha2_384_192;
            goto authcommon;
          case CRYPTO_SHA2_512_HMAC:
            axf = &auth_hash_hmac_sha2_512_256;
          authcommon:
            (*swd)->sw_ictx = kmm_malloc(axf->ctxsize);
            if ((*swd)->sw_ictx == NULL)
              {
                swcr_freesession(i);
                return -ENOBUFS;
              }

            (*swd)->sw_octx = kmm_malloc(axf->ctxsize);
            if ((*swd)->sw_octx == NULL)
              {
                swcr_freesession(i);
                return -ENOBUFS;
              }

            /* If the key is too long, hash it first using ictx */

            if (cri->cri_klen / 8 > axf->keysize)
              {
                axf->init((*swd)->sw_ictx);
                axf->update((*swd)->sw_ictx,
                            (FAR uint8_t *)cri->cri_key,
                            cri->cri_klen / 8);
                axf->final((unsigned char *)cri->cri_key,
                           (*swd)->sw_ictx);
                cri->cri_klen = axf->hashsize * 8;
              }

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= HMAC_IPAD_VAL;
              }

            axf->init((*swd)->sw_ictx);
            axf->update((*swd)->sw_ictx, (FAR uint8_t *)cri->cri_key,
                        cri->cri_klen / 8);
            axf->update((*swd)->sw_ictx, hmac_ipad_buffer,
                        axf->blocksize - (cri->cri_klen / 8));

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= (HMAC_IPAD_VAL ^ HMAC_OPAD_VAL);
              }

            axf->init((*swd)->sw_octx);
            axf->update((*swd)->sw_octx, (FAR uint8_t *)cri->cri_key,
                        cri->cri_klen / 8);
            axf->update((*swd)->sw_octx, hmac_opad_buffer,
                        axf->blocksize - (cri->cri_klen / 8));

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= HMAC_OPAD_VAL;
              }

            (*swd)->sw_axf = axf;
            bcopy((*swd)->sw_ictx, &(*swd)->sw_ctx, axf->ctxsize);
            break;

          case CRYPTO_MD5:
            axf = &auth_hash_md5;
            goto auth3common;
          case CRYPTO_RIPEMD160:
            axf = &auth_hash_ripemd_160;
            goto auth3common;
          case CRYPTO_SHA1:
            axf = &auth_hash_sha1;
            goto auth3common;
          case CRYPTO_SHA2_224:
            axf = &auth_hash_sha2_224;
            goto auth3common;
          case CRYPTO_SHA2_256:
            axf = &auth_hash_sha2_256;
            goto auth3common;
          case CRYPTO_SHA2_384:
            axf = &auth_hash_sha2_384;
            goto auth3common;
          case CRYPTO_SHA2_512:
            axf = &auth_hash_sha2_512;

          auth3common:
            (*swd)->sw_ictx = kmm_zalloc(axf->ctxsize);
            if ((*swd)->sw_ictx == NULL)
              {
                swcr_freesession(i);
                return -ENOBUFS;
              }

            axf->init((*swd)->sw_ictx);
            (*swd)->sw_axf = axf;
            bcopy((*swd)->sw_ictx, &(*swd)->sw_ctx, axf->ctxsize);

            if (cri->cri_sid != -1)
              {
                if (swcr_sessions[cri->cri_sid] == NULL)
                  {
                    swcr_freesession(i);
                    return -EINVAL;
                  }

                bcopy(&swcr_sessions[cri->cri_sid]->sw_ctx, &(*swd)->sw_ctx,
                      axf->ctxsize);
              }
            break;

          case CRYPTO_AES_128_GMAC:
            axf = &auth_hash_gmac_aes_128;
            goto auth4common;

          case CRYPTO_AES_192_GMAC:
            axf = &auth_hash_gmac_aes_192;
            goto auth4common;

          case CRYPTO_AES_256_GMAC:
            axf = &auth_hash_gmac_aes_256;
            goto auth4common;

          case CRYPTO_AES_128_CMAC:
            axf = &auth_hash_cmac_aes_128;
            goto auth4common;

          case CRYPTO_POLY1305:
            axf = &auth_hash_poly1305;
            goto auth4common;

          case CRYPTO_CRC32:
            axf = &auth_hash_crc32;
            goto auth4common;

          case CRYPTO_CHACHA20_POLY1305_MAC:
            axf = &auth_hash_chacha20_poly1305;

          auth4common:
            (*swd)->sw_ictx = kmm_malloc(axf->ctxsize);
            if ((*swd)->sw_ictx == NULL)
              {
                swcr_freesession(i);
                return -ENOBUFS;
              }

            axf->init((*swd)->sw_ictx);
            axf->setkey((*swd)->sw_ictx, (FAR uint8_t *)cri->cri_key,
                        cri->cri_klen / 8);
            bcopy((*swd)->sw_ictx, &(*swd)->sw_ctx, axf->ctxsize);
            (*swd)->sw_axf = axf;
            break;

          case CRYPTO_ESN:

            /* nothing to do */

            break;
          default:
            swcr_freesession(i);
            return -EINVAL;
        }

      (*swd)->sw_alg = cri->cri_alg;
      cri = cri->cri_next;
      swd = &((*swd)->sw_next);
    }

  return 0;
}

/* Free a session. */

int swcr_freesession(uint64_t tid)
{
  FAR struct swcr_data *swd;
  FAR const struct enc_xform *txf;
  FAR const struct auth_hash *axf;
  uint32_t sid = ((uint32_t) tid) & 0xffffffff;

  if (sid > swcr_sesnum || swcr_sessions == NULL ||
      swcr_sessions[sid] == NULL)
    {
      return -EINVAL;
    }

  /* Silently accept and return */

  if (sid == 0)
    {
      return 0;
    }

  while ((swd = swcr_sessions[sid]) != NULL)
    {
      swcr_sessions[sid] = swd->sw_next;

      switch (swd->sw_alg)
        {
          case CRYPTO_3DES_CBC:
          case CRYPTO_BLF_CBC:
          case CRYPTO_CAST_CBC:
          case CRYPTO_RIJNDAEL128_CBC:
          case CRYPTO_AES_CTR:
          case CRYPTO_AES_XTS:
          case CRYPTO_AES_GCM_16:
          case CRYPTO_AES_GMAC:
          case CRYPTO_AES_CMAC:
          case CRYPTO_AES_OFB:
          case CRYPTO_AES_CFB_8:
          case CRYPTO_AES_CFB_128:
          case CRYPTO_CHACHA20_POLY1305:
          case CRYPTO_NULL:
            txf = swd->sw_exf;

            if (swd->sw_kschedule)
            {
              explicit_bzero(swd->sw_kschedule, txf->ctxsize);
              kmm_free(swd->sw_kschedule);
            }

            break;

          case CRYPTO_MD5_HMAC:
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_RIPEMD160_HMAC:
          case CRYPTO_SHA2_256_HMAC:
          case CRYPTO_SHA2_384_HMAC:
          case CRYPTO_SHA2_512_HMAC:
            axf = swd->sw_axf;

            if (swd->sw_ictx)
              {
                explicit_bzero(swd->sw_ictx, axf->ctxsize);
                kmm_free(swd->sw_ictx);
              }

            if (swd->sw_octx)
              {
                explicit_bzero(swd->sw_octx, axf->ctxsize);
                kmm_free(swd->sw_octx);
              }

            break;

          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:
          case CRYPTO_AES_128_CMAC:
          case CRYPTO_CHACHA20_POLY1305_MAC:
          case CRYPTO_MD5:
          case CRYPTO_POLY1305:
          case CRYPTO_RIPEMD160:
          case CRYPTO_SHA1:
          case CRYPTO_SHA2_224:
          case CRYPTO_SHA2_256:
          case CRYPTO_SHA2_384:
          case CRYPTO_SHA2_512:
          case CRYPTO_CRC32:
            axf = swd->sw_axf;

            if (swd->sw_ictx)
              {
                explicit_bzero(swd->sw_ictx, axf->ctxsize);
                kmm_free(swd->sw_ictx);
              }

            break;
          }

      kmm_free(swd);
    }

  return 0;
}

/* Process a software request. */

int swcr_process(struct cryptop *crp)
{
  FAR const struct enc_xform *txf;
  FAR struct cryptodesc *crd;
  FAR struct swcr_data *sw;
  uint32_t lid;

  /* Sanity check */

  if (crp == NULL)
    {
      return -EINVAL;
    }

  if (crp->crp_desc == NULL || crp->crp_buf == NULL)
    {
      crp->crp_etype = -EINVAL;
      goto done;
    }

  lid = crp->crp_sid & 0xffffffff;
  if (lid >= swcr_sesnum || lid == 0 || swcr_sessions[lid] == NULL)
    {
      crp->crp_etype = -ENOENT;
      goto done;
    }

  /* Go through crypto descriptors, processing as we go */

  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      /* Find the crypto context.
       * XXX Note that the logic here prevents us from having
       * XXX the same algorithm multiple times in a session
       * XXX (or rather, we can but it won't give us the right
       * XXX results). To do that, we'd need some way of differentiating
       * XXX between the various instances of an algorithm (so we can
       * XXX locate the correct crypto context).
       */

      for (sw = swcr_sessions[lid];
           sw && sw->sw_alg != crd->crd_alg;
           sw = sw->sw_next);

      /* No such context ? */

      if (sw == NULL)
        {
          crp->crp_etype = -EINVAL;
          goto done;
        }

      switch (sw->sw_alg)
        {
          case CRYPTO_NULL:
            {
              break;
            }

          case CRYPTO_3DES_CBC:
          case CRYPTO_BLF_CBC:
          case CRYPTO_CAST_CBC:
          case CRYPTO_RIJNDAEL128_CBC:
          case CRYPTO_AES_CTR:
          case CRYPTO_AES_XTS:
          case CRYPTO_AES_OFB:
          case CRYPTO_AES_CFB_8:
          case CRYPTO_AES_CFB_128:
            txf = sw->sw_exf;

            if (crp->crp_iv)
              {
                if (!(crd->crd_flags & CRD_F_IV_EXPLICIT))
                  {
                    bcopy(crp->crp_iv, crd->crd_iv, txf->ivsize);
                    crd->crd_flags |= CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT;
                    crd->crd_skip = 0;
                  }
              }
            else
              {
                crd->crd_flags |= CRD_F_IV_PRESENT;
                crd->crd_skip = txf->blocksize;
                crd->crd_len -= txf->blocksize;
              }

            if ((crp->crp_etype = swcr_encdec(crp, crd, sw,
                crp->crp_buf)) != 0)
              {
                goto done;
              }

            break;
          case CRYPTO_MD5_HMAC:
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_RIPEMD160_HMAC:
          case CRYPTO_SHA2_256_HMAC:
          case CRYPTO_SHA2_384_HMAC:
          case CRYPTO_SHA2_512_HMAC:
            if ((crp->crp_etype = swcr_authcompute(crp, crd, sw,
                crp->crp_buf)) != 0)
              {
                goto done;
              }

            break;

          case CRYPTO_MD5:
          case CRYPTO_POLY1305:
          case CRYPTO_RIPEMD160:
          case CRYPTO_SHA1:
          case CRYPTO_SHA2_224:
          case CRYPTO_SHA2_256:
          case CRYPTO_SHA2_384:
          case CRYPTO_SHA2_512:
          case CRYPTO_CRC32:
            if ((crp->crp_etype = swcr_hash(crp, crd, sw,
                crp->crp_buf)) != 0)
              {
                goto done;
              }

            break;

          case CRYPTO_AES_GCM_16:
          case CRYPTO_AES_GMAC:
          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:
          case CRYPTO_AES_128_CMAC:
          case CRYPTO_CHACHA20_POLY1305:
          case CRYPTO_CHACHA20_POLY1305_MAC:
            crp->crp_etype = swcr_authenc(crp);
            goto done;
            break;

          default:

            /* Unknown/unsupported algorithm */

            crp->crp_etype = -EINVAL;
            goto done;
        }
    }

done:
  return 0;
}

int swcr_mod_exp(struct cryptkop *krp)
{
  uint8_t *input = (uint8_t *)krp->krp_param[0].crp_p;
  uint8_t *exp = (uint8_t *)krp->krp_param[1].crp_p;
  uint8_t *modulus = (uint8_t *)krp->krp_param[2].crp_p;
  uint8_t *output = (uint8_t *)krp->krp_param[3].crp_p;
  int input_len = krp->krp_param[0].crp_nbits / 8;
  int exp_len = krp->krp_param[1].crp_nbits / 8;
  int modulus_len = krp->krp_param[2].crp_nbits / 8;
  int output_len = krp->krp_param[3].crp_nbits / 8;
  struct bn a;
  struct bn e;
  struct bn n;
  struct bn r;

  bignum_init(&a);
  bignum_init(&e);
  bignum_init(&n);
  bignum_init(&r);
  memcpy(e.array, exp, exp_len);
  memcpy(n.array, modulus, modulus_len);
  memcpy(a.array, input, input_len);
  pow_mod_faster(&a, &e, &n, &r);
  memcpy(output, r.array, output_len);
  return 0;
}

static int swcr_dh_make_public(FAR struct cryptkop *krp)
{
  /* Curve25519 is used for testing. In fact,
   * the four parameters of this interface are p, g, x, gx；
   * p: used to determine the conic curve;
   * g: the base point of the curve;
   * x: the private key produced by random;
   * gx: the public key generated by the private key,
   *  which could be calculated by gx = g ^ x mod p;
   * In curve25519, p and g are fixed.
   */

  uint8_t *secret = (uint8_t *)krp->krp_param[2].crp_p;
  uint8_t *public = (uint8_t *)krp->krp_param[3].crp_p;

  curve25519_generate_secret(secret);
  return curve25519_generate_public(public, secret);
}

static int swcr_dh_make_common(FAR struct cryptkop *krp)
{
  /* Curve25519 is used for testing. In fact,
   * the four parameters of this interface are:
   * public key / private key / p (the conic curve) / shared key
   */

  uint8_t *public = (uint8_t *)krp->krp_param[0].crp_p;
  uint8_t *secret = (uint8_t *)krp->krp_param[1].crp_p;
  uint8_t *shared = (uint8_t *)krp->krp_param[3].crp_p;
  return curve25519(shared, secret, public);
}

int swcr_rsa_verify(struct cryptkop *krp)
{
  uint8_t *exp = (uint8_t *)krp->krp_param[0].crp_p;
  uint8_t *modulus = (uint8_t *)krp->krp_param[1].crp_p;
  uint8_t *sig = (uint8_t *)krp->krp_param[2].crp_p;
  uint8_t *hash = (uint8_t *)krp->krp_param[3].crp_p;
  uint8_t *padding = (uint8_t *)krp->krp_param[4].crp_p;
  int exp_len = krp->krp_param[0].crp_nbits / 8;
  int modulus_len = krp->krp_param[1].crp_nbits / 8;
  int sig_len = krp->krp_param[2].crp_nbits / 8;
  int hash_len = krp->krp_param[3].crp_nbits / 8;
  int padding_len = krp->krp_param[4].crp_nbits / 8;
  struct bn a;
  struct bn e;
  struct bn n;
  struct bn r;

  bignum_init(&a);
  bignum_init(&e);
  bignum_init(&n);
  bignum_init(&r);
  memcpy(e.array, exp, exp_len);
  memcpy(n.array, modulus, modulus_len);
  memcpy(a.array, sig, sig_len);
  pow_mod_faster(&a, &e, &n, &r);
  return !!memcmp(r.array, hash, hash_len) +
         !!memcmp(r.array + hash_len, padding, padding_len);
}

static int swcr_ecc256_genkey(FAR struct cryptkop *krp)
{
  uint8_t d[secp256r1];
  uint8_t x[secp256r1];
  uint8_t y[secp256r1];

  if (ecc_make_key_uncomp(x, y, d) == 0)
    {
      return -EINVAL;
    }

  memcpy(krp->krp_param[0].crp_p, d, secp256r1);
  memcpy(krp->krp_param[1].crp_p, x, secp256r1);
  memcpy(krp->krp_param[2].crp_p, y, secp256r1);
  return OK;
}

static int swcr_ecc256_sign(struct cryptkop *krp)
{
  uint8_t *d = (uint8_t *)krp->krp_param[0].crp_p;
  uint8_t *hash = (uint8_t *)krp->krp_param[1].crp_p;
  uint8_t sig[secp256r1 * 2];

  if (ecdsa_sign(d, hash, sig) == 0)
    {
      return -EINVAL;
    }

  memcpy(krp->krp_param[2].crp_p, sig, secp256r1);
  memcpy(krp->krp_param[3].crp_p, sig + secp256r1, secp256r1);
  return OK;
}

static int swcr_ecc256_verify(struct cryptkop *krp)
{
  uint8_t *x = (uint8_t *)krp->krp_param[0].crp_p;
  uint8_t *y = (uint8_t *)krp->krp_param[1].crp_p;
  uint8_t *r = (uint8_t *)krp->krp_param[3].crp_p;
  uint8_t *s = (uint8_t *)krp->krp_param[4].crp_p;
  uint8_t *hash = (uint8_t *)krp->krp_param[5].crp_p;
  uint8_t publickey[secp256r1 + 1];
  uint8_t signature[secp256r1 * 2];

  memcpy(publickey + 1, x, secp256r1);
  publickey[0] = 2 + (y[secp256r1 - 1] & 0x01);
  memcpy(signature, r, secp256r1);
  memcpy(signature + secp256r1, s, secp256r1);
  return ecdsa_verify(publickey, hash, signature) == 0;
}

int swcr_kprocess(struct cryptkop *krp)
{
  /* Sanity check */

  if (krp == NULL)
    {
      return -EINVAL;
    }

  /* Go through crypto descriptors, processing as we go */

  switch (krp->krp_op)
    {
      case CRK_MOD_EXP:
        if ((krp->krp_status = swcr_mod_exp(krp)) != 0)
          {
            goto done;
          }

        break;
      case CRK_DH_MAKE_PUBLIC:
        if ((krp->krp_status = swcr_dh_make_public(krp) != 0))
          {
            goto done;
          }

        break;
      case CRK_DH_COMPUTE_KEY:
        if ((krp->krp_status = swcr_dh_make_common(krp)) != 0)
          {
            goto done;
          }

        break;
      case CRK_RSA_PKCS15_VERIFY:
        if ((krp->krp_status = swcr_rsa_verify(krp)) != 0)
          {
            goto done;
          }

        break;
      case CRK_ECDSA_SECP256R1_SIGN:
        if ((krp->krp_status = swcr_ecc256_sign(krp)) != 0)
          {
            goto done;
          }

        break;
      case CRK_ECDSA_SECP256R1_VERIFY:
        if ((krp->krp_status = swcr_ecc256_verify(krp)) != 0)
          {
            goto done;
          }

        break;
      case CRK_ECDSA_SECP256R1_GENKEY:
        if ((krp->krp_status = swcr_ecc256_genkey(krp)) != 0)
          {
            goto done;
          }

        break;
      default:

        /* Unknown/unsupported algorithm */

        krp->krp_status = -EINVAL;
        goto done;
    }

done:
  return 0;
}

/* Initialize the driver, called from the kernel main(). */

void swcr_init(void)
{
  int algs[CRYPTO_ALGORITHM_MAX + 1];
  int kalgs[CRK_ALGORITHM_MAX + 1];
  int flags = CRYPTOCAP_F_SOFTWARE | CRYPTOCAP_F_ENCRYPT_MAC |
              CRYPTOCAP_F_MAC_ENCRYPT;

  swcr_id = crypto_get_driverid(flags);
  if (swcr_id < 0)
    {
      /* This should never happen */

      PANIC();
    }

  algs[CRYPTO_3DES_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_BLF_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_CAST_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_MD5_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA1_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_RIPEMD160_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_RIJNDAEL128_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CTR] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_XTS] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_GCM_16] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_GMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_NULL] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_384_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_512_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_128_GMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_192_GMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_256_GMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_OFB] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CFB_8] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CFB_128] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_CHACHA20_POLY1305] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_CHACHA20_POLY1305_MAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_MD5] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_POLY1305] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_RIPEMD160] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA1] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_224] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_384] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_512] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_CRC32] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_128_CMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_ESN] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(swcr_id, algs, swcr_newsession,
                  swcr_freesession, swcr_process);

  kalgs[CRK_MOD_EXP] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_DH_MAKE_PUBLIC] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_DH_COMPUTE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_RSA_PKCS15_VERIFY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_SIGN] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_VERIFY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_GENKEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  crypto_kregister(swcr_id, kalgs, swcr_kprocess);
}

#endif /* CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO */
