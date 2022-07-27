/****************************************************************************
 * include/crypto/cryptosoft.h
 * $OpenBSD: cryptosoft.h,v 1.14 2012/12/07 17:03:22 mikeb Exp $
 *
 * The author of this code is Angelos D. Keromytis (angelos@cis.upenn.edu)
 *
 * This code was written by Angelos D. Keromytis in Athens, Greece, in
 * February 2000. Network Security Technologies Inc. (NSTI) kindly
 * supported the development of this code.
 *
 * Copyright (c) 2000 Angelos D. Keromytis
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
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_CRYPTOSOFT_H
#define __INCLUDE_CRYPTO_CRYPTOSOFT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Software session entry */

struct swcr_data
{
  int sw_alg; /* Algorithm */
  union
    {
      struct
      {
        FAR uint8_t *ictx;
        FAR uint8_t *octx;
        uint32_t klen;
        FAR const struct auth_hash *axf;
      } SWCR_AUTH;

      struct
      {
        FAR uint8_t *kschedule;
        FAR const struct enc_xform *exf;
      } SWCR_ENC;

      struct
      {
        uint32_t size;
        FAR const struct comp_algo *cxf;
      } SWCR_COMP;
    } SWCR_UN;

#define sw_ictx   SWCR_UN.SWCR_AUTH.ictx
#define sw_octx   SWCR_UN.SWCR_AUTH.octx
#define sw_klen   SWCR_UN.SWCR_AUTH.klen
#define sw_axf    SWCR_UN.SWCR_AUTH.axf
#define sw_kschedule SWCR_UN.SWCR_ENC.kschedule
#define sw_exf    SWCR_UN.SWCR_ENC.exf
#define sw_size   SWCR_UN.SWCR_COMP.size
#define sw_cxf    SWCR_UN.SWCR_COMP.cxf

  struct swcr_data *sw_next;
};

#ifdef _KERNEL
extern const uint8_t hmac_ipad_buffer[HMAC_MAX_BLOCK_LEN];
extern const uint8_t hmac_opad_buffer[HMAC_MAX_BLOCK_LEN];

int swcr_encdec(FAR struct cryptodesc *,
                FAR struct swcr_data *, caddr_t, int);
int swcr_authcompute(FAR struct cryptop *, FAR struct cryptodesc *,
                     FAR struct swcr_data *, caddr_t, int);
int swcr_authenc(FAR struct cryptop *);
int swcr_compdec(FAR struct cryptodesc *, FAR struct swcr_data *,
                 caddr_t, int);
int swcr_process(FAR struct cryptop *);
int swcr_newsession(FAR uint32_t *, FAR struct cryptoini *);
int swcr_freesession(uint64_t);
void swcr_init(void);
#endif /* _KERNEL */

#endif /* __INCLUDE_CRYPTO_CRYPTOSOFT_H */
