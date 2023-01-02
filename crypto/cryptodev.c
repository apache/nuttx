/****************************************************************************
 * crypto/cryptodev.c
 * $OpenBSD: cryptodev.c,v 1.82 2014/08/18 05:11:03 dlg Exp $
 * Copyright (c) 2001 Theo de Raadt
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Effort sponsored in part by the Defense Advanced Research Projects
 * Agency (DARPA) and Air Force Research Laboratory, Air Force
 * Materiel Command, USAF, under agreement number F30602-01-2-0537.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/queue.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/drivers/drivers.h>

#include <crypto/xform.h>
#include <crypto/cryptodev.h>
#include <crypto/cryptosoft.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR struct cryptocap *crypto_drivers;
extern int crypto_drivers_num;
int usercrypto = 1;         /* userland may do crypto requests */
int userasymcrypto = 1;     /* userland may do asymmetric crypto reqs */
int cryptodevallowsoft = 1; /* 0 is only use hardware crypto
                             * 1 is use hardware & software crypto
                             */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct csession
{
  TAILQ_ENTRY(csession) next;
  uint64_t sid;
  uint32_t ses;

  uint32_t cipher;
  FAR const struct enc_xform *txform;
  uint32_t mac;
  FAR const struct auth_hash *thash;

  caddr_t key;
  int keylen;
  u_char tmp_iv[EALG_MAX_BLOCK_LEN];

  caddr_t mackey;
  int mackeylen;
  int error;
};

struct fcrypt
{
  TAILQ_HEAD(csessionlist, csession) csessions;
  int sesn;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t cryptof_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t cryptof_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
static int cryptof_ioctl(FAR struct file *filep,
                         int cmd, unsigned long arg);
static int cryptof_poll(FAR struct file *filep,
                        struct pollfd *fds, bool setup);
static int cryptof_close(FAR struct file *filep);

static int cryptoopen(FAR struct file *filep);
static int cryptoclose(FAR struct file *filep);
static int cryptoioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cryptofops =
{
  NULL,                /* open   */
  cryptof_close,       /* close  */
  cryptof_read,        /* read   */
  cryptof_write,       /* write  */
  NULL,                /* seek   */
  cryptof_ioctl,       /* ioctl  */
  NULL,                /* mmap   */
  NULL,                /* truncate */
  cryptof_poll         /* poll   */
};

static const struct file_operations g_cryptoops =
{
  cryptoopen,          /* open   */
  cryptoclose,         /* close  */
  NULL,                /* read   */
  NULL,                /* write  */
  NULL,                /* seek   */
  cryptoioctl,         /* ioctl  */
  NULL,                /* mmap   */
  NULL,                /* truncate */
  NULL                 /* poll   */
};

static struct inode g_cryptoinode =
{
  .i_crefs = 1,
  .u.i_ops = &g_cryptofops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

FAR struct csession *csefind(FAR struct fcrypt *, u_int);
int csedelete(FAR struct fcrypt *, FAR struct csession *);
FAR struct csession *cseadd(FAR struct fcrypt *, FAR struct csession *);
FAR struct csession *csecreate(FAR struct fcrypt *, uint64_t,
                               caddr_t, uint64_t,
                               caddr_t, uint64_t, uint32_t,
                               uint32_t, FAR const struct enc_xform *,
                               FAR const struct auth_hash *);
int csefree(FAR struct csession *);

int cryptodev_op(FAR struct csession *,
                 FAR struct crypt_op *);
int cryptodev_key(FAR struct crypt_kop *);
int cryptodev_dokey(FAR struct crypt_kop *kop, FAR struct crparam *kvp);

int cryptodev_cb(FAR struct cryptop *);
int cryptodevkey_cb(FAR struct cryptkop *);

/* ARGSUSED */

static ssize_t cryptof_read(FAR struct file *filep,
                            FAR char *buffer, size_t len)
{
  return -EIO;
}

/* ARGSUSED */

static ssize_t cryptof_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len)
{
  return -EIO;
}

/* ARGSUSED */

static int cryptof_ioctl(FAR struct file *filep,
                         int cmd, unsigned long arg)
{
  struct cryptoini cria;
  struct cryptoini crie;
  FAR struct fcrypt *fcr = filep->f_priv;
  FAR struct csession *cse;
  FAR struct session_op *sop;
  FAR struct crypt_op *cop;
  FAR const struct enc_xform *txform = NULL;
  FAR const struct auth_hash *thash = NULL;
  uint64_t sid;
  uint32_t ses;
  int error = 0;

  switch (cmd)
    {
      case CIOCGSESSION:
        sop = (FAR struct session_op *)arg;
        switch (sop->cipher)
          {
            case 0:
              break;
            case CRYPTO_3DES_CBC:
              txform = &enc_xform_3des;
              break;
            case CRYPTO_BLF_CBC:
              txform = &enc_xform_blf;
              break;
            case CRYPTO_CAST_CBC:
              txform = &enc_xform_cast5;
              break;
            case CRYPTO_AES_CBC:
              txform = &enc_xform_aes;
              break;
            case CRYPTO_AES_CTR:
              txform = &enc_xform_aes_ctr;
              break;
            case CRYPTO_AES_XTS:
              txform = &enc_xform_aes_xts;
              break;
            case CRYPTO_NULL:
              txform = &enc_xform_null;
              break;
            default:
              return -EINVAL;
          }

        switch (sop->mac)
          {
            case 0:
              break;
            case CRYPTO_MD5_HMAC:
              thash = &auth_hash_hmac_md5_96;
              break;
            case CRYPTO_SHA1_HMAC:
              thash = &auth_hash_hmac_sha1_96;
              break;
            case CRYPTO_RIPEMD160_HMAC:
              thash = &auth_hash_hmac_ripemd_160_96;
              break;
            case CRYPTO_SHA2_256_HMAC:
              thash = &auth_hash_hmac_sha2_256_128;
              break;
            case CRYPTO_SHA2_384_HMAC:
              thash = &auth_hash_hmac_sha2_384_192;
              break;
            case CRYPTO_SHA2_512_HMAC:
              thash = &auth_hash_hmac_sha2_512_256;
              break;
            case CRYPTO_AES_128_GMAC:
              thash = &auth_hash_gmac_aes_128;
              break;
            default:
              return -EINVAL;
          }

        bzero(&crie, sizeof(crie));
        bzero(&cria, sizeof(cria));

        if (txform)
          {
            crie.cri_alg = txform->type;
            crie.cri_klen = sop->keylen * 8;
            if (sop->keylen > txform->maxkey ||
                sop->keylen < txform->minkey)
              {
                error = -EINVAL;
                goto bail;
              }

            crie.cri_key = kmm_malloc(crie.cri_klen / 8);
            if (crie.cri_key == NULL)
              {
                error = -ENOMEM;
                goto bail;
              }

            memcpy(crie.cri_key, sop->key, crie.cri_klen / 8);
            if (thash)
              {
                crie.cri_next = &cria;
              }
          }

        if (thash)
          {
            cria.cri_alg = thash->type;
            cria.cri_klen = sop->mackeylen * 8;
            if (sop->mackeylen > thash->keysize)
              {
                error = -EINVAL;
                goto bail;
              }

            if (cria.cri_klen)
              {
                cria.cri_key = kmm_malloc(cria.cri_klen / 8);
                if (cria.cri_key == NULL)
                  {
                    error = -ENOMEM;
                    goto bail;
                  }

                memcpy(cria.cri_key, sop->mackey, cria.cri_klen / 8);
              }
          }

        error = crypto_newsession(&sid, txform ? &crie : &cria,
                                  !cryptodevallowsoft);

        if (error)
          {
            goto bail;
          }

        cse = csecreate(fcr, sid, crie.cri_key, crie.cri_klen,
              cria.cri_key, cria.cri_klen, sop->cipher, sop->mac, txform,
              thash);

        if (cse == NULL)
          {
            crypto_freesession(sid);
            error = -EINVAL;
            goto bail;
          }

        sop->ses = cse->ses;

bail:
        if (error)
          {
            if (crie.cri_key)
              {
                explicit_bzero(crie.cri_key, crie.cri_klen / 8);
                kmm_free(crie.cri_key);
              }

            if (cria.cri_key)
              {
                explicit_bzero(cria.cri_key, cria.cri_klen / 8);
                kmm_free(cria.cri_key);
              }
          }

        break;
      case CIOCFSESSION:
        ses = *(FAR uint32_t *)arg;
        cse = csefind(fcr, ses);
        if (cse == NULL)
          {
            return -EINVAL;
          }

        csedelete(fcr, cse);
        error = csefree(cse);
        break;
      case CIOCCRYPT:
        cop = (FAR struct crypt_op *)arg;
        cse = csefind(fcr, cop->ses);
        if (cse == NULL)
          {
            return -EINVAL;
          }

        error = cryptodev_op(cse, cop);
        break;
      case CIOCKEY:
        error = cryptodev_key((FAR struct crypt_kop *)arg);
        break;
      case CIOCASYMFEAT:
        error = crypto_getfeat((FAR int *)arg);
        break;
      default:
        error = -EINVAL;
    }

  return error;
}

int cryptodev_op(FAR struct csession *cse,
                 FAR struct crypt_op *cop)
{
  FAR struct cryptop *crp = NULL;
  FAR struct cryptodesc *crde = NULL;
  FAR struct cryptodesc *crda = NULL;
  int error;
  uint32_t hid;

  if (cop->len > 64 * 1024 - 4)
    {
      return -E2BIG;
    }

  if (cse->txform)
    {
      if (cop->len == 0 || (cop->len % cse->txform->blocksize) != 0)
        {
          return -EINVAL;
        }
    }

  /* number of requests, not logical and */

  crp = crypto_getreq((cse->txform != NULL) + (cse->thash != NULL));
  if (crp == NULL)
    {
      error = -ENOMEM;
      goto bail;
    }

  if (cse->thash)
    {
      crda = crp->crp_desc;
      if (cse->txform)
        crde = crda->crd_next;
    }
  else
    {
      if (cse->txform)
        {
          crde = crp->crp_desc;
        }
      else
        {
          error = -EINVAL;
          goto bail;
        }
    }

  if (crda)
    {
      crda->crd_skip = 0;
      crda->crd_len = cop->len;
      crda->crd_inject = 0;

      crda->crd_alg = cse->mac;
      crda->crd_key = cse->mackey;
      crda->crd_klen = cse->mackeylen * 8;
      if (cop->flags & COP_FLAG_UPDATE)
        {
          crda->crd_flags |= CRD_F_UPDATE;
        }
      else
        {
          crda->crd_flags &= ~CRD_F_UPDATE;
        }
    }

  if (crde)
    {
      if (cop->op == COP_ENCRYPT)
        {
          crde->crd_flags |= CRD_F_ENCRYPT;
        }
      else
        {
          crde->crd_flags &= ~CRD_F_ENCRYPT;
        }

      crde->crd_len = cop->len;
      crde->crd_inject = 0;
      crde->crd_alg = cse->cipher;
      crde->crd_key = cse->key;
      crde->crd_klen = cse->keylen * 8;
    }

  crp->crp_ilen = cop->len;
  crp->crp_buf = cop->src;
  crp->crp_sid = cse->sid;
  crp->crp_opaque = cse;

  if (cop->iv)
    {
      if (crde == NULL)
        {
          error = -EINVAL;
          goto bail;
        }

      if (!(crde->crd_flags & CRD_F_IV_EXPLICIT))
        {
          memcpy(cse->tmp_iv, cop->iv, cse->txform->blocksize);
          bcopy(cse->tmp_iv, crde->crd_iv, cse->txform->blocksize);
          crde->crd_flags |= CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT;
          crde->crd_skip = 0;
        }
    }
  else if (crde)
    {
      crde->crd_flags |= CRD_F_IV_PRESENT;
      crde->crd_skip = cse->txform->blocksize;
      crde->crd_len -= cse->txform->blocksize;
    }

  if (cop->dst)
    {
      if (crde == NULL)
        {
          error = -EINVAL;
          goto bail;
        }

      crp->crp_dst = cop->dst;
    }

  if (cop->mac)
    {
      if (crda == NULL)
        {
          error = -EINVAL;
          goto bail;
        }

      crp->crp_mac = cop->mac;
    }

  /* try the fast path first */

  crp->crp_flags = CRYPTO_F_IOV | CRYPTO_F_NOQUEUE;
  hid = (crp->crp_sid >> 32) & 0xffffffff;
  if (hid >= crypto_drivers_num)
    {
      goto dispatch;
    }

  if (crypto_drivers[hid].cc_flags & CRYPTOCAP_F_SOFTWARE)
    {
      goto dispatch;
    }

  if (crypto_drivers[hid].cc_process == NULL)
    {
      goto dispatch;
    }

  error = crypto_drivers[hid].cc_process(crp);
  if (error)
    {
      /* clear error */

      crp->crp_etype = 0;
      goto dispatch;
    }

  goto processed;
dispatch:
  crp->crp_flags = CRYPTO_F_IOV;
  crypto_invoke(crp);
processed:

  if (cop->flags & COP_FLAG_UPDATE == 0)
    {
      crde->crd_flags &= ~CRD_F_IV_EXPLICIT;
    }

  if (cse->error)
    {
      error = cse->error;
      goto bail;
    }

  if (crp->crp_etype != 0)
    {
      error = crp->crp_etype;
      goto bail;
    }

bail:
  if (crp)
    {
      crypto_freereq(crp);
    }

  return error;
}

int cryptodev_key(FAR struct crypt_kop *kop)
{
  FAR struct cryptkop *krp = NULL;
  int error = -EINVAL;
  int in;
  int out;
  int size;
  int i;

  if (kop->crk_iparams + kop->crk_oparams > CRK_MAXPARAM)
    {
      return -EFBIG;
    }

  in = kop->crk_iparams;
  out = kop->crk_oparams;
  switch (kop->crk_op)
    {
      case CRK_MOD_EXP:
        if (in == 3 && out == 1)
          break;
        return -EINVAL;
      case CRK_MOD_EXP_CRT:
        if (in == 6 && out == 1)
          break;
        return -EINVAL;
      case CRK_DSA_SIGN:
        if (in == 5 && out == 2)
          break;
        return -EINVAL;
      case CRK_DSA_VERIFY:
        if (in == 7 && out == 0)
          break;
        return -EINVAL;
      case CRK_DH_COMPUTE_KEY:
        if (in == 3 && out == 1)
          break;
        return -EINVAL;
      default:
        return -EINVAL;
    }

  krp = kmm_malloc(sizeof *krp);
  krp->krp_op = kop->crk_op;
  krp->krp_status = kop->crk_status;
  krp->krp_iparams = kop->crk_iparams;
  krp->krp_oparams = kop->crk_oparams;
  krp->krp_status = 0;

  for (i = 0; i < CRK_MAXPARAM; i++)
    {
      krp->krp_param[i].crp_nbits = kop->crk_param[i].crp_nbits;
      if (kop->crk_param[i].crp_nbits > 65536)
        {
          /* XXX how big do we need to support? */

          goto fail;
        }
    }

  for (i = 0; i < krp->krp_iparams + krp->krp_oparams; i++)
    {
      size = (krp->krp_param[i].crp_nbits + 7) / 8;
      if (size == 0)
        {
          continue;
        }

      krp->krp_param[i].crp_p = kmm_malloc(size);
      if (i >= krp->krp_iparams)
        {
          continue;
        }

      memcpy(krp->krp_param[i].crp_p, kop->crk_param[i].crp_p, size);
      if (error)
        {
          goto fail;
        }
    }

  error = crypto_kinvoke(krp);
  if (error)
    {
      goto fail;
    }

  if (krp->krp_status != 0)
    {
      error = krp->krp_status;
      goto fail;
    }

  for (i = krp->krp_iparams; i < krp->krp_iparams + krp->krp_oparams; i++)
    {
      size = (krp->krp_param[i].crp_nbits + 7) / 8;
      if (size == 0)
        {
          continue;
        }

      memcpy(kop->crk_param[i].crp_p,
             krp->krp_param[i].crp_p, size);
    }

fail:
  if (krp)
    {
      kop->crk_status = krp->krp_status;
      for (i = 0; i < CRK_MAXPARAM; i++)
        {
          if (krp->krp_param[i].crp_p)
            {
              explicit_bzero(krp->krp_param[i].crp_p,
                  (krp->krp_param[i].crp_nbits + 7) / 8);
              kmm_free(krp->krp_param[i].crp_p);
            }
        }

      kmm_free(krp);
    }

  return error;
}

/* ARGSUSED */

static int cryptof_poll(FAR struct file *filep,
                        struct pollfd *fds, bool setup)
{
  return 0;
}

/* ARGSUSED */

static int cryptof_close(FAR struct file *filep)
{
  FAR struct fcrypt *fcr = filep->f_priv;
  FAR struct csession *cse;

  while ((cse = TAILQ_FIRST(&fcr->csessions)))
    {
      TAILQ_REMOVE(&fcr->csessions, cse, next);
      (void)csefree(cse);
    }

    kmm_free(fcr);
    filep->f_priv = NULL;

  return 0;
}

static int cryptoopen(FAR struct file *filep)
{
  if (usercrypto == 0)
    {
      return -ENXIO;
    }

  return 0;
}

static int cryptoclose(FAR struct file *filep)
{
  return 0;
}

static int cryptoioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct fcrypt *fcr;
  int fd;
  int error = 0;

  switch (cmd)
    {
      case CRIOGET:
        fcr = kmm_malloc(sizeof(struct fcrypt));
        TAILQ_INIT(&fcr->csessions);

        fd = file_allocate(&g_cryptoinode, 0,
                           0, fcr, 0, true);
        if (fd < 0)
          {
            kmm_free(fcr);
            return fd;
          }

        fcr->sesn = 0;
        *(FAR uint32_t *)arg = fd;
        break;
      default:
        error = -EINVAL;
        break;
    }

  return error;
}

FAR struct csession *csefind(FAR struct fcrypt *fcr, u_int ses)
{
  FAR struct csession *cse;

  TAILQ_FOREACH(cse, &fcr->csessions, next)
  if (cse->ses == ses)
    {
      return cse;
    }

  return NULL;
}

int csedelete(FAR struct fcrypt *fcr, FAR struct csession *cse_del)
{
  FAR struct csession *cse;

  TAILQ_FOREACH(cse, &fcr->csessions, next)
    {
      if (cse == cse_del)
        {
          TAILQ_REMOVE(&fcr->csessions, cse, next);
          return 1;
        }
    }

  return 0;
}

FAR struct csession *cseadd(FAR struct fcrypt *fcr,
                            FAR struct csession *cse)
{
  TAILQ_INSERT_TAIL(&fcr->csessions, cse, next);
  cse->ses = fcr->sesn++;
  return cse;
}

FAR struct csession *csecreate(FAR struct fcrypt *fcr, uint64_t sid,
                               caddr_t key, uint64_t keylen,
                               caddr_t mackey, uint64_t mackeylen,
                               uint32_t cipher, uint32_t mac,
                               FAR const struct enc_xform *txform,
                               FAR const struct auth_hash *thash)
{
  FAR struct csession *cse;

  cse = kmm_malloc(sizeof(struct csession));
  if (cse == NULL)
    {
      return NULL;
    }

  cse->key = key;
  cse->keylen = keylen / 8;
  cse->mackey = mackey;
  cse->mackeylen = mackeylen / 8;
  cse->sid = sid;
  cse->cipher = cipher;
  cse->mac = mac;
  cse->txform = txform;
  cse->thash = thash;
  cse->error = 0;
  cseadd(fcr, cse);
  return cse;
}

int csefree(FAR struct csession *cse)
{
  int error;

  error = crypto_freesession(cse->sid);
  if (cse->key)
    {
      kmm_free(cse->key);
    }

  if (cse->mackey)
    {
      kmm_free(cse->mackey);
    }

  kmm_free(cse);
  return error;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void devcrypto_register(void)
{
  register_driver("/dev/crypto", &g_cryptoops, 0666, NULL);
  swcr_init();

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE
  hwcr_init();
#endif
}
