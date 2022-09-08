/****************************************************************************
 * crypto/cryptosoft.c
 * $OpenBSD: cryptosoft.c,v 1.71 2014/07/13 23:24:47 deraadt Exp $
 * The author of this code is Angelos D. Keromytis (angelos@cis.upenn.edu)
 *
 * This code was written by Angelos D. Keromytis in Athens, Greece, in
 * February 2000. Network Security Technologies Inc. (NSTI) kindly
 * supported the development of this code.
 *
 * Copyright (c) 2000, 2001 Angelos D. Keromytis
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <endian.h>
#include <nuttx/kmalloc.h>
#include <crypto/cryptodev.h>
#include <crypto/cryptosoft.h>
#include <crypto/xform.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef howmany
#  define howmany(x, y)  (((x) + ((y) - 1)) / (y))
#endif

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct swcr_data **swcr_sessions = NULL;
uint32_t swcr_sesnum = 0;
int swcr_id = -1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Apply a symmetric encryption/decryption algorithm. */

int swcr_encdec(FAR struct cryptop *crp, FAR struct cryptodesc *crd,
                FAR struct swcr_data *sw, caddr_t buf)
{
  unsigned char iv[EALG_MAX_BLOCK_LEN];
  unsigned char blk[EALG_MAX_BLOCK_LEN];
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

  /* Check for non-padded data */

  if (crd->crd_len % blks)
    {
      return -EINVAL;
    }

  /* Initialize the IV */

  if (crd->crd_flags & CRD_F_ENCRYPT)
    {
      /* IV explicitly provided ? */

      if (crd->crd_flags & CRD_F_IV_EXPLICIT)
        {
          bcopy(crd->crd_iv, iv, ivlen);
        }
      else
        {
          arc4random_buf(iv, ivlen);
        }

      /* Do we need to write the IV */

      if (!(crd->crd_flags & CRD_F_IV_PRESENT))
        {
          bcopy(iv, buf + crd->crd_inject, ivlen);
        }
    }
  else
    {
      /* Decryption */

      /* IV explicitly provided ? */

      if (crd->crd_flags & CRD_F_IV_EXPLICIT)
        {
          bcopy(crd->crd_iv, iv, ivlen);
        }
      else
        {
          /* Get IV off buf */

          bcopy(iv, buf + crd->crd_inject, ivlen);
        }
    }

  ivp = iv;

  /* xforms that provide a reinit method perform all IV
   * handling themselves.
   */

  if (exf->reinit)
    {
      exf->reinit((caddr_t)sw->sw_kschedule, iv);
    }

  i = crd->crd_len;

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

      bcopy(blk, crp->crp_dst, exf->blocksize);
      crp->crp_dst += exf->blocksize;

      i -= blks;

      /* Could be done... */

      if (i == 0)
        {
          break;
        }
    }

  return 0; /* Done with encryption/decryption */
}

/* Compute keyed-hash authenticator. */

int swcr_authcompute(FAR struct cryptop *crp,
                     FAR struct cryptodesc *crd,
                     FAR struct swcr_data *sw,
                     caddr_t buf)
{
  unsigned char aalg[AALG_MAX_RESULT_LEN];
  FAR const struct auth_hash *axf;
  union authctx ctx;
  int err;

  if (sw->sw_ictx == 0)
    {
      return -EINVAL;
    }

  axf = sw->sw_axf;

  bcopy(sw->sw_ictx, &ctx, axf->ctxsize);
  err = axf->update(&ctx, (FAR uint8_t *)buf, crd->crd_len);
  if (err)
    {
      return err;
    }

  if (crd->crd_flags & CRD_F_ESN)
    {
      axf->update(&ctx, crd->crd_esn, 4);
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

        axf->final(aalg, &ctx);
        bcopy(sw->sw_octx, &ctx, axf->ctxsize);
        axf->update(&ctx, aalg, axf->hashsize);
        axf->final(aalg, &ctx);
        break;
    }

  /* Inject the authentication data */

  bcopy(aalg, crp->crp_mac, axf->hashsize);
  return 0;
}

/* Apply a combined encryption-authentication transformation */

int swcr_authenc(FAR struct cryptop *crp)
{
  uint32_t blkbuf[howmany(EALG_MAX_BLOCK_LEN, sizeof(uint32_t))];
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
  FAR uint32_t *blkp;
  int aadlen;
  int blksz;
  int i;
  int ivlen;
  int len;
  int iskip;
  int oskip;

  ivlen = blksz = iskip = oskip = 0;

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
          case CRYPTO_CHACHA20_POLY1305:
          swe = sw;
          crde = crd;
          exf = swe->sw_exf;
          ivlen = exf->ivsize;
          break;
          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:
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

  if (exf->reinit)
    {
      exf->reinit((caddr_t)swe->sw_kschedule, iv);
    }

  /* Do encryption/decryption with MAC */

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

      bcopy(blk, crp->crp_dst + i, len);
    }

  /* Do any required special finalization */

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
          case CRYPTO_CHACHA20_POLY1305_MAC:
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

          case CRYPTO_AES_GCM_16:
          case CRYPTO_AES_GMAC:
          case CRYPTO_AES_128_GMAC:
          case CRYPTO_AES_192_GMAC:
          case CRYPTO_AES_256_GMAC:
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

/* Initialize the driver, called from the kernel main(). */

void swcr_init(void)
{
  int algs[CRYPTO_ALGORITHM_MAX + 1];
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
  algs[CRYPTO_CHACHA20_POLY1305] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_CHACHA20_POLY1305_MAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_ESN] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(swcr_id, algs, swcr_newsession,
                  swcr_freesession, swcr_process);
}
