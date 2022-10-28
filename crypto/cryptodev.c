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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/pool.h>
#include <sys/mbuf.h>
#include <sys/proc.h>
#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/errno.h>
#include <dev/rndvar.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/rmd160.h>
#include <crypto/cast.h>
#include <crypto/blf.h>
#include <crypto/cryptodev.h>
#include <crypto/xform.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR struct cryptocap *crypto_drivers;
extern int crypto_drivers_num;
int usercrypto = 0;         /* userland may do crypto requests */
int userasymcrypto = 0;     /* userland may do asymmetric crypto reqs */
int cryptodevallowsoft = 0; /* only use hardware crypto */

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
  struct iovec iovec[IOV_MAX];
  struct uio uio;
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

void cryptoattach(int);

int cryptof_read(FAR struct file *, FAR off_t *,
                 FAR struct uio *, FAR struct ucred *);
int cryptof_write(FAR struct file *, FAR off_t *,
                  FAR struct uio *, FAR struct ucred *);
int cryptof_ioctl(FAR struct file *, u_long, caddr_t, FAR struct proc *p);
int cryptof_poll(FAR struct file *, int, FAR struct proc *);
int cryptof_kqfilter(FAR struct file *, FAR struct knote *);
int cryptof_stat(FAR struct file *, FAR struct stat *, FAR struct proc *);
int cryptof_close(FAR struct file *, FAR struct proc *);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fileops g_cryptofops =
{
  cryptof_read,
  cryptof_write,
  cryptof_ioctl,
  cryptof_poll,
  cryptof_kqfilter,
  cryptof_stat,
  cryptof_close
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
                 FAR struct crypt_op *, FAR struct proc *);
int cryptodev_key(FAR struct crypt_kop *);
int cryptodev_dokey(FAR struct crypt_kop *kop, FAR struct crparam *kvp);

int cryptodev_cb(FAR struct cryptop *);
int cryptodevkey_cb(FAR struct cryptkop *);

/* ARGSUSED */

int cryptof_read(FAR struct file *fp, FAR off_t *poff,
                 FAR struct uio *uio, FAR struct ucred *cred)
{
  return EIO;
}

/* ARGSUSED */

int cryptof_write(FAR struct file *fp, FAR off_t *poff,
                  FAR struct uio *uio, FAR struct ucred *cred)
{
  return EIO;
}

/* ARGSUSED */

int cryptof_ioctl(FAR struct file *fp, u_long cmd,
                  caddr_t data, FAR struct proc *p)
{
  struct cryptoini cria;
  struct cryptoini crie;
  FAR struct fcrypt *fcr = fp->f_data;
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
        sop = (FAR struct session_op *)data;
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
              return EINVAL;
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
              return EINVAL;
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
                error = EINVAL;
                goto bail;
              }

            crie.cri_key = malloc(crie.cri_klen / 8, M_XDATA,
                M_WAITOK);
            if ((error = copyin(sop->key, crie.cri_key,
                crie.cri_klen / 8)))
              {
                goto bail;
              }

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
                error = EINVAL;
                goto bail;
              }

            if (cria.cri_klen)
              {
                cria.cri_key = malloc(cria.cri_klen / 8,
                    M_XDATA, M_WAITOK);
                if ((error = copyin(sop->mackey, cria.cri_key,
                    cria.cri_klen / 8)))
                  {
                    goto bail;
                  }
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
            error = EINVAL;
            goto bail;
          }

        sop->ses = cse->ses;

bail:
        if (error)
          {
            if (crie.cri_key)
              {
                explicit_bzero(crie.cri_key, crie.cri_klen / 8);
                free(crie.cri_key, M_XDATA, 0);
              }

            if (cria.cri_key)
              {
                explicit_bzero(cria.cri_key, cria.cri_klen / 8);
                free(cria.cri_key, M_XDATA, 0);
              }
          }

        break;
      case CIOCFSESSION:
        ses = *(FAR uint32_t *)data;
        cse = csefind(fcr, ses);
        if (cse == NULL)
          {
            return EINVAL;
          }

        csedelete(fcr, cse);
        error = csefree(cse);
        break;
      case CIOCCRYPT:
        cop = (FAR struct crypt_op *)data;
        cse = csefind(fcr, cop->ses);
        if (cse == NULL)
          {
            return EINVAL;
          }

        error = cryptodev_op(cse, cop, p);
        break;
      case CIOCKEY:
        error = cryptodev_key((FAR struct crypt_kop *)data);
        break;
      case CIOCASYMFEAT:
        error = crypto_getfeat((FAR int *)data);
        break;
      default:
        error = EINVAL;
    }

  return error;
}

int cryptodev_op(FAR struct csession *cse,
                 FAR struct crypt_op *cop,
                 FAR struct proc *p)
{
  FAR struct cryptop *crp = NULL;
  FAR struct cryptodesc *crde = NULL;
  FAR struct cryptodesc *crda = NULL;
  int s;
  int error;
  uint32_t hid;

  if (cop->len > 64 * 1024 - 4)
    {
      return E2BIG;
    }

  if (cse->txform)
    {
      if (cop->len == 0 || (cop->len % cse->txform->blocksize) != 0)
        {
          return EINVAL;
        }
    }

  bzero(&cse->uio, sizeof(cse->uio));
  cse->uio.uio_iovcnt = 1;
  cse->uio.uio_segflg = UIO_SYSSPACE;
  cse->uio.uio_rw = UIO_WRITE;
  cse->uio.uio_procp = p;
  cse->uio.uio_iov = cse->iovec;
  bzero(&cse->iovec, sizeof(cse->iovec));
  cse->uio.uio_iov[0].iov_len = cop->len;
  cse->uio.uio_iov[0].iov_base = dma_alloc(cop->len, M_WAITOK);
  cse->uio.uio_resid = cse->uio.uio_iov[0].iov_len;

  /* number of requests, not logical and */

  crp = crypto_getreq((cse->txform != NULL) + (cse->thash != NULL));
  if (crp == NULL)
    {
      error = ENOMEM;
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
          error = EINVAL;
          goto bail;
        }
    }

  if ((error = copyin(cop->src, cse->uio.uio_iov[0].iov_base, cop->len)))
    {
      goto bail;
    }

  if (crda)
    {
      crda->crd_skip = 0;
      crda->crd_len = cop->len;
      crda->crd_inject = 0;

      crda->crd_alg = cse->mac;
      crda->crd_key = cse->mackey;
      crda->crd_klen = cse->mackeylen * 8;
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
  crp->crp_buf = (caddr_t)&cse->uio;
  crp->crp_callback = cryptodev_cb;
  crp->crp_sid = cse->sid;
  crp->crp_opaque = cse;

  if (cop->iv)
    {
      if (crde == NULL)
        {
          error = EINVAL;
          goto bail;
        }

      if ((error = copyin(cop->iv, cse->tmp_iv, cse->txform->blocksize)))
        {
          goto bail;
        }

      bcopy(cse->tmp_iv, crde->crd_iv, cse->txform->blocksize);
      crde->crd_flags |= CRD_F_IV_EXPLICIT | CRD_F_IV_PRESENT;
      crde->crd_skip = 0;
    }
  else if (crde)
    {
      crde->crd_flags |= CRD_F_IV_PRESENT;
      crde->crd_skip = cse->txform->blocksize;
      crde->crd_len -= cse->txform->blocksize;
    }

  if (cop->mac)
    {
      if (crda == NULL)
        {
          error = EINVAL;
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
  crypto_dispatch(crp);
processed:
  s = splnet();
  while (!(crp->crp_flags & CRYPTO_F_DONE))
    {
      error = tsleep(cse, PSOCK, "crydev", 0);
    }

  splx(s);
  if (error)
    {
      /* XXX can this happen?  if so, how do we recover? */

      goto bail;
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

  if (cop->dst &&
      (error = copyout(cse->uio.uio_iov[0].iov_base, cop->dst, cop->len)))
    {
      goto bail;
    }

  if (cop->mac &&
      (error = copyout(crp->crp_mac, cop->mac, cse->thash->hashsize)))
    {
      goto bail;
    }

bail:
  if (crp)
    {
      crypto_freereq(crp);
    }

  if (cse->uio.uio_iov[0].iov_base)
    {
      dma_free(cse->uio.uio_iov[0].iov_base, cop->len);
    }

  return error;
}

int cryptodev_cb(FAR struct cryptop *crp)
{
  FAR struct csession *cse = crp->crp_opaque;

  cse->error = crp->crp_etype;
  if (crp->crp_etype == EAGAIN)
    {
      crp->crp_flags = CRYPTO_F_IOV;
      return crypto_dispatch(crp);
    }

  wakeup(cse);
  return (0);
}

int cryptodevkey_cb(FAR struct cryptkop *krp)
{
  wakeup(krp);
  return (0);
}

int cryptodev_key(FAR struct crypt_kop *kop)
{
  FAR struct cryptkop *krp = NULL;
  int error = EINVAL;
  int in;
  int out;
  int size;
  int i;

  if (kop->crk_iparams + kop->crk_oparams > CRK_MAXPARAM)
    {
      return EFBIG;
    }

  in = kop->crk_iparams;
  out = kop->crk_oparams;
  switch (kop->crk_op)
    {
      case CRK_MOD_EXP:
        if (in == 3 && out == 1)
          break;
        return EINVAL;
      case CRK_MOD_EXP_CRT:
        if (in == 6 && out == 1)
          break;
        return EINVAL;
      case CRK_DSA_SIGN:
        if (in == 5 && out == 2)
          break;
        return EINVAL;
      case CRK_DSA_VERIFY:
        if (in == 7 && out == 0)
          break;
        return EINVAL;
      case CRK_DH_COMPUTE_KEY:
        if (in == 3 && out == 1)
          break;
        return EINVAL;
      default:
        return EINVAL;
    }

  krp = malloc(sizeof *krp, M_XDATA, M_WAITOK | M_ZERO);
  krp->krp_op = kop->crk_op;
  krp->krp_status = kop->crk_status;
  krp->krp_iparams = kop->crk_iparams;
  krp->krp_oparams = kop->crk_oparams;
  krp->krp_status = 0;
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

      krp->krp_param[i].crp_p = malloc(size, M_XDATA, M_WAITOK);
      if (i >= krp->krp_iparams)
        {
          continue;
        }

      error = copyin(kop->crk_param[i].crp_p,
                      krp->krp_param[i].crp_p, size);
      if (error)
        {
          goto fail;
        }
    }

  error = crypto_kdispatch(krp);
  if (error)
    {
      goto fail;
    }

  error = tsleep(krp, PSOCK, "crydev", 0);
  if (error)
    {
      /* XXX can this happen?  if so, how do we recover? */

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

      error = copyout(krp->krp_param[i].crp_p,
                      kop->crk_param[i].crp_p, size);
      if (error)
        {
          goto fail;
        }
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
              free(krp->krp_param[i].crp_p, M_XDATA, 0);
            }
        }

      free(krp, M_XDATA, 0);
    }

  return error;
}

/* ARGSUSED */

int cryptof_poll(FAR struct file *fp, int events, FAR struct proc *p)
{
  return 0;
}

/* ARGSUSED */

int cryptof_kqfilter(FAR struct file *fp, FAR struct knote *kn)
{
  return 0;
}

/* ARGSUSED */

int cryptof_stat(FAR struct file *fp,
                 FAR struct stat *sb,
                 FAR struct proc *p)
{
  return EOPNOTSUPP;
}

/* ARGSUSED */

int cryptof_close(FAR struct file *fp, FAR struct proc *p)
{
  FAR struct fcrypt *fcr = fp->f_data;
  FAR struct csession *cse;

  while ((cse = TAILQ_FIRST(&fcr->csessions)))
    {
      TAILQ_REMOVE(&fcr->csessions, cse, next);
      (void)csefree(cse);
    }

  free(fcr, M_XDATA, 0);
  fp->f_data = NULL;
  return 0;
}

void cryptoattach(int n)
{
}

int cryptoopen(dev_t dev, int flag, int mode, FAR struct proc *p)
{
  if (usercrypto == 0)
    {
      return ENXIO;
    }

#ifdef CRYPTO
  return 0;
#else
  return ENXIO;
#endif
}

int cryptoclose(dev_t dev, int flag, int mode, FAR struct proc *p)
{
  return (0);
}

int cryptoioctl(dev_t dev, u_long cmd,
                caddr_t data, int flag, FAR struct proc *p)
{
  FAR struct file *f;
  FAR struct fcrypt *fcr;
  int fd;
  int error = 0;

  switch (cmd)
    {
      case CRIOGET:
        fcr = malloc(sizeof(struct fcrypt), M_XDATA, M_WAITOK);
        TAILQ_INIT(&fcr->csessions);
        fcr->sesn = 0;

        fdplock(p->p_fd);
        error = falloc(p, &f, &fd);
        fdpunlock(p->p_fd);
        if (error)
          {
            free(fcr, M_XDATA, 0);
            return error;
          }

        f->f_flag = FREAD | FWRITE;
        f->f_type = DTYPE_CRYPTO;
        f->f_ops = &cryptofops;
        f->f_data = fcr;
        *(FAR uint32_t *)data = fd;
        FILE_SET_MATURE(f, p);
        break;
      default:
        error = EINVAL;
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

  cse = malloc(sizeof(struct csession), M_XDATA, M_NOWAIT);
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
      free(cse->key, M_XDATA, 0);
    }

  if (cse->mackey)
    {
      free(cse->mackey, M_XDATA, 0);
    }

  free(cse, M_XDATA, 0);
  return error;
}
