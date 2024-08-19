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
#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE
int cryptodevallowsoft = 1; /* 0 is only use hardware crypto
                             * 1 is use hardware & software crypto
                             */
#else
int cryptodevallowsoft = 0;
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct csession
{
  TAILQ_ENTRY(csession) next;
  uint64_t sid;
  uint32_t ses;

  uint32_t cipher;
  uint32_t mac;
  bool txform;
  bool thash;

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
  TAILQ_HEAD(cryptkoplist, cryptkop) crpk_ret;
  int sesn;
  FAR struct pollfd *fds;
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
                        FAR struct pollfd *fds, bool setup);
static int cryptof_open(FAR struct file *filep);
static int cryptof_close(FAR struct file *filep);

static int cryptoopen(FAR struct file *filep);
static ssize_t cryptoread(FAR struct file *filep,
                          FAR char *buffer, size_t len);
static ssize_t cryptowrite(FAR struct file *filep,
                           FAR const char *buffer, size_t len);
static int cryptoclose(FAR struct file *filep);
static int cryptoioctl(FAR struct file *filep, int cmd, unsigned long arg);

static FAR struct csession *csefind(FAR struct fcrypt *, u_int);
static int csedelete(FAR struct fcrypt *, FAR struct csession *);
static FAR struct csession *cseadd(FAR struct fcrypt *,
                                   FAR struct csession *);
static FAR struct csession *csecreate(FAR struct fcrypt *, uint64_t,
                                      caddr_t, uint64_t,
                                      caddr_t, uint64_t, uint32_t,
                                      uint32_t, bool, bool);
static int csefree(FAR struct csession *);

static int cryptodev_op(FAR struct csession *,
                        FAR struct crypt_op *);
static int cryptodev_key(FAR struct fcrypt *, FAR struct crypt_kop *);
static int cryptodevkey_cb(FAR struct cryptkop *);
static int cryptodev_getkeystatus(FAR struct fcrypt *,
                                  FAR struct crypt_kop *);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cryptofops =
{
  cryptof_open,        /* open   */
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
  cryptoread,          /* read   */
  cryptowrite,         /* write  */
  NULL,                /* seek   */
  cryptoioctl,         /* ioctl  */
  NULL,                /* mmap   */
  NULL,                /* truncate */
  NULL                 /* poll   */
};

static struct inode g_cryptoinode =
{
  .i_flags = FSNODEFLAG_TYPE_DRIVER,
  .i_crefs = 1,
  .u.i_ops = &g_cryptofops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  bool txform = false;
  bool thash = false;
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
            case CRYPTO_BLF_CBC:
            case CRYPTO_CAST_CBC:
            case CRYPTO_AES_CBC:
            case CRYPTO_AES_CMAC:
            case CRYPTO_AES_CTR:
            case CRYPTO_AES_XTS:
            case CRYPTO_AES_OFB:
            case CRYPTO_AES_CFB_8:
            case CRYPTO_AES_CFB_128:
            case CRYPTO_NULL:
              txform = true;
              break;
            default:
              return -EINVAL;
          }

        switch (sop->mac)
          {
            case 0:
              break;
            case CRYPTO_MD5_HMAC:
            case CRYPTO_SHA1_HMAC:
            case CRYPTO_RIPEMD160_HMAC:
            case CRYPTO_SHA2_256_HMAC:
            case CRYPTO_SHA2_384_HMAC:
            case CRYPTO_SHA2_512_HMAC:
            case CRYPTO_AES_128_GMAC:
            case CRYPTO_AES_128_CMAC:
            case CRYPTO_MD5:
            case CRYPTO_POLY1305:
            case CRYPTO_RIPEMD160:
            case CRYPTO_SHA1:
            case CRYPTO_SHA2_224:
            case CRYPTO_SHA2_256:
            case CRYPTO_SHA2_384:
            case CRYPTO_SHA2_512:
            case CRYPTO_CRC32:
              thash = true;
              break;
            default:
              return -EINVAL;
          }

        bzero(&crie, sizeof(crie));
        bzero(&cria, sizeof(cria));

        if (txform)
          {
            crie.cri_alg = sop->cipher;
            crie.cri_klen = sop->keylen * 8;

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
            cria.cri_alg = sop->mac;
            cria.cri_sid = -1;
            cria.cri_klen = sop->mackeylen * 8;

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
        error = cryptodev_key(fcr, (FAR struct crypt_kop *)arg);
        break;
      case CIOCKEYRET:
        error = cryptodev_getkeystatus(fcr, (FAR struct crypt_kop *)arg);
        break;
      case CIOCASYMFEAT:
        error = crypto_getfeat((FAR int *)arg);
        break;
      default:
        error = -ENOTTY;
    }

  return error;
}

static int cryptodev_op(FAR struct csession *cse,
                        FAR struct crypt_op *cop)
{
  FAR struct cryptop *crp = NULL;
  FAR struct cryptodesc *crde = NULL;
  FAR struct cryptodesc *crda = NULL;
  int error = OK;
  uint32_t hid;

  /* number of requests, not logical and */

  crp = crypto_getreq(cse->txform + cse->thash);
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

      crp->crp_iv = cop->iv;
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

  if (crde && (cop->flags & COP_FLAG_UPDATE) == 0)
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

static int cryptodev_key(FAR struct fcrypt *fcr, FAR struct crypt_kop *kop)
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
          {
            break;
          }

        return -EINVAL;
      case CRK_MOD_EXP_CRT:
        if (in == 6 && out == 1)
          {
            break;
          }

        return -EINVAL;
      case CRK_DSA_SIGN:
        if (in == 5 && out == 2)
          {
            break;
          }

        return -EINVAL;
      case CRK_DSA_VERIFY:
        if (in == 7 && out == 0)
          {
            break;
          }

        return -EINVAL;
      case CRK_DH_COMPUTE_KEY:
        if (in == 3 && out == 1)
          {
            break;
          }

        return -EINVAL;
      case CRK_DH_MAKE_PUBLIC:
        if (in == 2 && out == 2)
          {
            break;
          }

        return -EINVAL;
      case CRK_RSA_PKCS15_VERIFY:
        if (in == 5 && out == 0)
          {
            break;
          }

        return -EINVAL;
      case CRK_ECDSA_SECP256R1_SIGN:
        if (in == 2 && out == 2)
          {
            break;
          }

        return -EINVAL;
      case CRK_ECDSA_SECP256R1_VERIFY:
        if (in == 6 && out == 0)
          {
            break;
          }

        return -EINVAL;
      case CRK_ECDSA_SECP256R1_GENKEY:
        if (in == 0 && out == 4)
          {
            break;
          }

        return -EINVAL;
      default:
        return -EINVAL;
    }

  krp = kmm_zalloc(sizeof *krp);
  if (krp == NULL)
    {
      return -ENOMEM;
    }

  krp->krp_op = kop->crk_op;
  krp->krp_iparams = kop->crk_iparams;
  krp->krp_oparams = kop->crk_oparams;
  krp->krp_status = 0;
  krp->krp_flags = kop->crk_flags;
  krp->krp_reqid = kop->crk_reqid;
  krp->krp_fcr = fcr;
  krp->krp_callback = cryptodevkey_cb;

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

      krp->krp_param[i].crp_p = kmm_zalloc(size);
      if (i >= krp->krp_iparams)
        {
          continue;
        }

      memcpy(krp->krp_param[i].crp_p, kop->crk_param[i].crp_p, size);
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

static int cryptodevkey_cb(FAR struct cryptkop *krp)
{
  TAILQ_INSERT_TAIL(&krp->krp_fcr->crpk_ret, krp, krp_next);
  if (krp->krp_fcr->fds != NULL)
    {
      poll_notify(&krp->krp_fcr->fds, 1, POLLIN);
    }

  return OK;
}

static int cryptodev_getkeystatus(struct fcrypt *fcr, struct crypt_kop *ret)
{
  FAR struct cryptkop *krp = NULL;
  int i;
  int size;

  if (TAILQ_EMPTY(&fcr->crpk_ret))
    {
      return -EAGAIN;
    }

  /* return the result in task list to the upper layer  */

  krp = TAILQ_FIRST(&fcr->crpk_ret);
  TAILQ_REMOVE(&fcr->crpk_ret, krp, krp_next);

  ret->crk_op = krp->krp_op;
  ret->crk_status = krp->krp_status;
  ret->crk_iparams = krp->krp_iparams;
  ret->crk_oparams = krp->krp_oparams;
  for (i = 0; i < krp->krp_iparams + krp->krp_oparams; i++)
    {
      size = (krp->krp_param[i].crp_nbits + 7) / 8;

      /* copy result into oparams */

      if (i < ret->crk_iparams || size == 0)
        {
          continue;
        }

      memcpy(ret->crk_param[i].crp_p, krp->krp_param[i].crp_p, size);
    }

  /* free asynchronous result */

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
  return OK;
}

/* ARGSUSED */

static int cryptof_poll(FAR struct file *filep,
                        FAR struct pollfd *fds, bool setup)
{
  FAR struct fcrypt *fcr = filep->f_priv;

  if (fcr == NULL || fds == NULL)
    {
      return -EINVAL;
    }

  if (setup)
    {
      if (!TAILQ_EMPTY(&fcr->crpk_ret))
        {
          poll_notify(&fds, 1, POLLIN);
          return OK;
        }

      if (fcr->fds)
        {
          return -EBUSY;
        }

      fcr->fds = fds;
    }
  else
    {
      fcr->fds = NULL;
    }

  return OK;
}

/* ARGSUSED */

static int cryptof_close(FAR struct file *filep)
{
  FAR struct fcrypt *fcr = filep->f_priv;
  FAR struct csession *cse;
  FAR struct cryptkop *krp;
  int i;

  while ((cse = TAILQ_FIRST(&fcr->csessions)))
    {
      TAILQ_REMOVE(&fcr->csessions, cse, next);
      (void)csefree(cse);
    }

  while ((krp = TAILQ_FIRST(&fcr->crpk_ret)))
    {
      TAILQ_REMOVE(&fcr->crpk_ret, krp, krp_next);
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

  kmm_free(fcr);
  filep->f_priv = NULL;
  return 0;
}

/* Clone csessions into a new fd */

static int cryptof_open(FAR struct file *filep)
{
  FAR struct fcrypt *fcr = filep->f_priv;
  FAR struct fcrypt *fcrd = NULL;
  FAR struct csession *cse;
  FAR struct csession *csed;
  struct cryptoini cria;
  struct cryptoini crie;
  uint64_t sid;
  int ret = 0;

  if (fcr == NULL)
    {
      return -EINVAL;
    }

  /* A 'struct fcrypt' is bound to the fd of the cryptodev,
   * which stores a list of sessions. Each encryption operation
   * would create a context and get a session id, which can be
   * used to find the specific encryption operation. Therefore,
   * in order to complete the copy operation, it is necessary to
   * create same session based on the copy fd and obtain the newly
   * generated session id.
   */

  fcrd = kmm_zalloc(sizeof(struct fcrypt));
  if (fcrd == NULL)
    {
      return -ENOMEM;
    }

  TAILQ_INIT(&fcrd->csessions);
  TAILQ_FOREACH(cse, &fcr->csessions, next)
    {
      bzero(&crie, sizeof(crie));
      bzero(&cria, sizeof(cria));
      if (cse->txform)
        {
          crie.cri_alg = cse->cipher;
          crie.cri_klen = cse->keylen * 8;

          crie.cri_key = kmm_malloc(cse->keylen);
          if (crie.cri_key == NULL)
            {
              ret = -ENOMEM;
              goto bail;
            }

          memcpy(crie.cri_key, cse->key, cse->keylen);
          if (cse->thash)
            {
              crie.cri_next = &cria;
            }
        }

      if (cse->thash)
        {
          cria.cri_alg = cse->mac;
          cria.cri_sid = cse->sid;
          cria.cri_klen = cse->mackeylen * 8;

          if (cria.cri_klen)
            {
              cria.cri_key = kmm_malloc(cse->mackeylen);
              if (cria.cri_key == NULL)
                {
                  ret = -ENOMEM;
                  goto bail;
                }

              memcpy(cria.cri_key, cse->mackey, cse->mackeylen);
            }
        }

      ret = crypto_newsession(&sid, cse->txform ? &crie : &cria,
                              !cryptodevallowsoft);
      if (ret < 0)
        {
          goto bail;
        }

      csed = csecreate(fcrd, sid, crie.cri_key, crie.cri_klen,
                        cria.cri_key, cria.cri_klen,
                        cse->cipher, cse->mac, cse->txform,
                        cse->thash);
      if (csed == NULL)
        {
          crypto_freesession(sid);
          ret = -EINVAL;
          goto bail;
        }

      csed->ses = cse->ses;
    }

  filep->f_priv = fcrd;
  return 0;

bail:
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

  return ret;
}

static int cryptoopen(FAR struct file *filep)
{
  if (usercrypto == 0)
    {
      return -ENXIO;
    }

  return 0;
}

static ssize_t cryptoread(FAR struct file *filep,
                          FAR char *buffer, size_t len)
{
  return 0;
}

static ssize_t cryptowrite(FAR struct file *filep,
                           FAR const char *buffer, size_t len)
{
  return len;
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
        fcr = kmm_zalloc(sizeof(struct fcrypt));
        if (fcr == NULL)
          {
            return -ENOMEM;
          }

        TAILQ_INIT(&fcr->csessions);
        TAILQ_INIT(&fcr->crpk_ret);

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

static FAR struct csession *csefind(FAR struct fcrypt *fcr, u_int ses)
{
  FAR struct csession *cse;

  TAILQ_FOREACH(cse, &fcr->csessions, next)
  if (cse->ses == ses)
    {
      return cse;
    }

  return NULL;
}

static int csedelete(FAR struct fcrypt *fcr, FAR struct csession *cse_del)
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

static FAR struct csession *cseadd(FAR struct fcrypt *fcr,
                                   FAR struct csession *cse)
{
  TAILQ_INSERT_TAIL(&fcr->csessions, cse, next);
  cse->ses = fcr->sesn++;
  return cse;
}

static FAR struct csession *csecreate(FAR struct fcrypt *fcr, uint64_t sid,
                                      caddr_t key, uint64_t keylen,
                                      caddr_t mackey, uint64_t mackeylen,
                                      uint32_t cipher, uint32_t mac,
                                      bool txform, bool thash)
{
  FAR struct csession *cse;

  cse = kmm_malloc(sizeof(struct csession));
  if (cse != NULL)
    {
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
    }

  return cse;
}

static int csefree(FAR struct csession *cse)
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

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE
  swcr_init();
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE
  hwcr_init();
#endif
}
