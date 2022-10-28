/*	$OpenBSD: cryptodev.h,v 1.58 2013/10/31 10:32:38 mikeb Exp $	*/

/*
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
 * Copyright (c) 2001 Theo de Raadt
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 *
 */

#ifndef _CRYPTO_CRYPTO_H_
#define _CRYPTO_CRYPTO_H_

#include <sys/ioccom.h>
#include <sys/task.h>

/* Some initial values */
#define CRYPTO_DRIVERS_INITIAL	4
#define CRYPTO_DRIVERS_MAX	128
#define CRYPTO_SW_SESSIONS	32

/* HMAC values */
#define HMAC_MD5_BLOCK_LEN	64
#define HMAC_SHA1_BLOCK_LEN	64
#define HMAC_RIPEMD160_BLOCK_LEN 64
#define HMAC_SHA2_256_BLOCK_LEN	64
#define HMAC_SHA2_384_BLOCK_LEN	128
#define HMAC_SHA2_512_BLOCK_LEN	128
#define HMAC_MAX_BLOCK_LEN	HMAC_SHA2_512_BLOCK_LEN	/* keep in sync */
#define HMAC_IPAD_VAL		0x36
#define HMAC_OPAD_VAL		0x5C

/* Encryption algorithm block sizes */
#define DES_BLOCK_LEN		8
#define DES3_BLOCK_LEN		8
#define BLOWFISH_BLOCK_LEN	8
#define CAST128_BLOCK_LEN	8
#define RIJNDAEL128_BLOCK_LEN	16
#define EALG_MAX_BLOCK_LEN	16 /* Keep this updated */

/* Maximum hash algorithm result length */
#define AALG_MAX_RESULT_LEN	64 /* Keep this updated */

#define CRYPTO_DES_CBC		1
#define CRYPTO_3DES_CBC		2
#define CRYPTO_BLF_CBC		3
#define CRYPTO_CAST_CBC		4
#define CRYPTO_MD5_HMAC		6
#define CRYPTO_SHA1_HMAC	7
#define CRYPTO_RIPEMD160_HMAC	8
#define CRYPTO_MD5_KPDK		9
#define CRYPTO_SHA1_KPDK	10
#define CRYPTO_RIJNDAEL128_CBC	11 /* 128 bit blocksize */
#define CRYPTO_AES_CBC		11 /* 128 bit blocksize -- the same as above */
#define CRYPTO_ARC4		12
#define CRYPTO_MD5		13
#define CRYPTO_SHA1		14
#define CRYPTO_DEFLATE_COMP	15 /* Deflate compression algorithm */
#define CRYPTO_NULL		16
#define CRYPTO_LZS_COMP		17 /* LZS compression algorithm */
#define CRYPTO_SHA2_256_HMAC	18
#define CRYPTO_SHA2_384_HMAC	19
#define CRYPTO_SHA2_512_HMAC	20
#define CRYPTO_AES_CTR		21
#define CRYPTO_AES_XTS		22
#define CRYPTO_AES_GCM_16	23
#define CRYPTO_AES_128_GMAC	24
#define CRYPTO_AES_192_GMAC	25
#define CRYPTO_AES_256_GMAC	26
#define CRYPTO_AES_GMAC		27
#define CRYPTO_ESN		28 /* Support for Extended Sequence Numbers */
#define CRYPTO_ALGORITHM_MAX	28 /* Keep updated */

/* Algorithm flags */
#define	CRYPTO_ALG_FLAG_SUPPORTED	0x01 /* Algorithm is supported */
#define	CRYPTO_ALG_FLAG_RNG_ENABLE	0x02 /* Has HW RNG for DH/DSA */
#define	CRYPTO_ALG_FLAG_DSA_SHA		0x04 /* Can do SHA on msg */

/* Standard initialization structure beginning */
struct cryptoini {
	int		cri_alg;	/* Algorithm to use */
	int		cri_klen;	/* Key length, in bits */
	int		cri_rnd;	/* Algorithm rounds, where relevant */
	caddr_t		cri_key;	/* key to use */
	union {
		u_int8_t	iv[EALG_MAX_BLOCK_LEN];	/* IV to use */
		u_int8_t	esn[4];			/* high-order ESN */
	} u;
#define cri_iv		u.iv
#define cri_esn		u.esn
	struct cryptoini *cri_next;
};

/* Describe boundaries of a single crypto operation */
struct cryptodesc {
	int		crd_skip;	/* How many bytes to ignore from start */
	int		crd_len;	/* How many bytes to process */
	int		crd_inject;	/* Where to inject results, if applicable */
	int		crd_flags;

#define	CRD_F_ENCRYPT		0x01	/* Set when doing encryption */
#define	CRD_F_IV_PRESENT	0x02	/* When encrypting, IV is already in
					   place, so don't copy. */
#define	CRD_F_IV_EXPLICIT	0x04	/* IV explicitly provided */
#define	CRD_F_DSA_SHA_NEEDED	0x08	/* Compute SHA-1 of buffer for DSA */
#define CRD_F_COMP		0x10    /* Set when doing compression */
#define CRD_F_ESN		0x20	/* Set when ESN field is provided */

	struct cryptoini	CRD_INI; /* Initialization/context data */
#define crd_esn		CRD_INI.cri_esn
#define crd_iv		CRD_INI.cri_iv
#define crd_key		CRD_INI.cri_key
#define crd_rnd		CRD_INI.cri_rnd
#define crd_alg		CRD_INI.cri_alg
#define crd_klen	CRD_INI.cri_klen

	struct cryptodesc *crd_next;
};

/* Structure describing complete operation */
struct cryptop {
	struct task	crp_task;

	u_int64_t	crp_sid;	/* Session ID */
	int		crp_ilen;	/* Input data total length */
	int		crp_olen;	/* Result total length */
	int		crp_alloctype;	/* Type of buf to allocate if needed */

	int		crp_etype;	/*
					 * Error type (zero means no error).
					 * All error codes except EAGAIN
					 * indicate possible data corruption (as in,
					 * the data have been touched). On all
					 * errors, the crp_sid may have changed
					 * (reset to a new one), so the caller
					 * should always check and use the new
					 * value on future requests.
					 */
	int		crp_flags;

#define CRYPTO_F_IMBUF	0x0001	/* Input/output are mbuf chains, otherwise contig */
#define CRYPTO_F_IOV	0x0002	/* Input/output are uio */
#define CRYPTO_F_REL	0x0004	/* Must return data in same place */
#define CRYPTO_F_NOQUEUE	0x0008	/* Don't use crypto queue/thread */
#define CRYPTO_F_DONE	0x0010	/* request completed */

	void 		*crp_buf;	/* Data to be processed */
	void 		*crp_opaque;	/* Opaque pointer, passed along */
	struct cryptodesc *crp_desc;	/* Linked list of processing descriptors */

	int (*crp_callback)(struct cryptop *); /* Callback function */

	caddr_t		crp_mac;
};

#define CRYPTO_BUF_IOV		0x1
#define CRYPTO_BUF_MBUF		0x2

#define CRYPTO_OP_DECRYPT	0x0
#define CRYPTO_OP_ENCRYPT	0x1

/* bignum parameter, in packed bytes, ... */
struct crparam {
	caddr_t		crp_p;
	u_int		crp_nbits;
};

#define CRK_MAXPARAM	8

struct crypt_kop {
	u_int		crk_op;		/* ie. CRK_MOD_EXP or other */
	u_int		crk_status;	/* return status */
	u_short		crk_iparams;	/* # of input parameters */
	u_short		crk_oparams;	/* # of output parameters */
	u_int		crk_pad1;
	struct crparam	crk_param[CRK_MAXPARAM];
};
#define CRK_MOD_EXP		0
#define CRK_MOD_EXP_CRT		1
#define CRK_DSA_SIGN		2
#define CRK_DSA_VERIFY		3
#define CRK_DH_COMPUTE_KEY	4
#define CRK_ALGORITHM_MAX	4 /* Keep updated */

#define CRF_MOD_EXP		(1 << CRK_MOD_EXP)
#define CRF_MOD_EXP_CRT		(1 << CRK_MOD_EXP_CRT)
#define CRF_DSA_SIGN		(1 << CRK_DSA_SIGN)
#define CRF_DSA_VERIFY		(1 << CRK_DSA_VERIFY)
#define CRF_DH_COMPUTE_KEY	(1 << CRK_DH_COMPUTE_KEY)

struct cryptkop {
	struct task	krp_task;

	u_int		krp_op;		/* ie. CRK_MOD_EXP or other */
	u_int		krp_status;	/* return status */
	u_short		krp_iparams;	/* # of input parameters */
	u_short		krp_oparams;	/* # of output parameters */
	u_int32_t	krp_hid;
	struct crparam	krp_param[CRK_MAXPARAM];	/* kvm */
	int		(*krp_callback)(struct cryptkop *);
};

/* Crypto capabilities structure */
struct cryptocap {
	u_int64_t	cc_operations;	/* Counter of how many ops done */
	u_int64_t	cc_bytes;	/* Counter of how many bytes done */
	u_int64_t	cc_koperations;	/* How many PK ops done */

	u_int32_t	cc_sessions;	/* How many sessions allocated */

	/* Symmetric/hash algorithms supported */
	int		cc_alg[CRYPTO_ALGORITHM_MAX + 1];

	/* Asymmetric algorithms supported */
	int		cc_kalg[CRK_ALGORITHM_MAX + 1];

	int		cc_queued;	/* Operations queued */

	u_int8_t	cc_flags;
#define CRYPTOCAP_F_CLEANUP     0x01
#define CRYPTOCAP_F_SOFTWARE    0x02
#define CRYPTOCAP_F_ENCRYPT_MAC 0x04 /* Can do encrypt-then-MAC (IPsec) */
#define CRYPTOCAP_F_MAC_ENCRYPT 0x08 /* Can do MAC-then-encrypt (TLS) */

	int		(*cc_newsession) (u_int32_t *, struct cryptoini *);
	int		(*cc_process) (struct cryptop *);
	int		(*cc_freesession) (u_int64_t);
	int		(*cc_kprocess) (struct cryptkop *);
};

/*
 * ioctl parameter to request creation of a session.
 */
struct session_op {
	u_int32_t	cipher;		/* ie. CRYPTO_DES_CBC */
	u_int32_t	mac;		/* ie. CRYPTO_MD5_HMAC */

	u_int32_t	keylen;		/* cipher key */
	caddr_t		key;
	int		mackeylen;	/* mac key */
	caddr_t		mackey;

	u_int32_t	ses;		/* returns: session # */
};

/*
 * ioctl parameter to request a crypt/decrypt operation against a session.
 */
struct crypt_op {
	u_int32_t	ses;
	u_int16_t	op;		/* ie. COP_ENCRYPT */
#define COP_ENCRYPT	1
#define COP_DECRYPT	2
	u_int16_t	flags;		/* always 0 */

	u_int		len;
	caddr_t		src, dst;	/* become iov[] inside kernel */
	caddr_t		mac;		/* must be big enough for chosen MAC */
	caddr_t		iv;
};

#define CRYPTO_MAX_MAC_LEN	20

/*
 * done against open of /dev/crypto, to get a cloned descriptor.
 * Please use F_SETFD against the cloned descriptor.
 */
#define	CRIOGET		_IOWR('c', 100, u_int32_t)

/* the following are done against the cloned descriptor */
#define	CIOCGSESSION	_IOWR('c', 101, struct session_op)
#define	CIOCFSESSION	_IOW('c', 102, u_int32_t)
#define CIOCCRYPT	_IOWR('c', 103, struct crypt_op)
#define CIOCKEY		_IOWR('c', 104, struct crypt_kop)

#define CIOCASYMFEAT	_IOR('c', 105, u_int32_t)

#ifdef _KERNEL
int	crypto_newsession(u_int64_t *, struct cryptoini *, int);
int	crypto_freesession(u_int64_t);
int	crypto_dispatch(struct cryptop *);
int	crypto_kdispatch(struct cryptkop *);
int	crypto_register(u_int32_t, int *,
	    int (*)(u_int32_t *, struct cryptoini *), int (*)(u_int64_t),
	    int (*)(struct cryptop *));
int	crypto_kregister(u_int32_t, int *, int (*)(struct cryptkop *));
int	crypto_unregister(u_int32_t, int);
int32_t	crypto_get_driverid(u_int8_t);
int	crypto_invoke(struct cryptop *);
int	crypto_kinvoke(struct cryptkop *);
void	crypto_done(struct cryptop *);
void	crypto_kdone(struct cryptkop *);
int	crypto_getfeat(int *);

void	cuio_copydata(struct uio *, int, int, caddr_t);
void	cuio_copyback(struct uio *, int, int, const void *);
int	cuio_getptr(struct uio *, int, int *);
int	cuio_apply(struct uio *, int, int,
	    int (*f)(caddr_t, caddr_t, unsigned int), caddr_t);

struct	cryptop *crypto_getreq(int);
void	crypto_freereq(struct cryptop *);
#endif /* _KERNEL */
#endif /* _CRYPTO_CRYPTO_H_ */
