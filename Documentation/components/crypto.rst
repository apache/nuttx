====================
Crypto API Subsystem
====================

Overview
========

The NuttX Crypto API subsystem provides a unified interface for cryptographic operations, supporting various encryption, decryption, hashing, and authentication algorithms. The subsystem abstracts hardware and software crypto implementations through a common interface.

Supported Algorithms
====================

Symmetric Encryption Algorithms
--------------------------------

**AES (Advanced Encryption Standard)**

- AES-CBC Mode:
  - CRYPTO_AES_CBC (128-bit key size)
  - CRYPTO_AES_192_CBC (192-bit key size)
  - CRYPTO_AES_256_CBC (256-bit key size)

- AES-CTR Mode (Counter mode):
  - CRYPTO_AES_CTR

- AES-XTS Mode (XEX-based Tweaked CodeBook):
  - CRYPTO_AES_XTS

- AES-GCM Mode (Galois/Counter Mode):
  - CRYPTO_AES_GCM_16

- AES-OFB Mode (Output Feedback):
  - CRYPTO_AES_OFB

- AES-CFB Mode (Cipher Feedback):
  - CRYPTO_AES_CFB_8 (8-bit)
  - CRYPTO_AES_CFB_128 (128-bit)

**Other Block Cipher Modes**

- Blowfish (BLF):
  - CRYPTO_BLF_CBC

- CAST (CAST-128):
  - CRYPTO_CAST_CBC

- Rijndael (128-bit):
  - CRYPTO_RIJNDAEL128_CBC

- Null (No encryption):
  - CRYPTO_NULL

Authentication and Hashing Algorithms
--------------------------------------

**HMAC (Hash-based Message Authentication Code)**

- MD5-HMAC:
  - CRYPTO_MD5_HMAC

- SHA-1 HMAC:
  - CRYPTO_SHA1_HMAC

- SHA-2 HMAC:
  - CRYPTO_SHA2_256_HMAC (256-bit)
  - CRYPTO_SHA2_384_HMAC (384-bit)
  - CRYPTO_SHA2_512_HMAC (512-bit)

**Hash Functions**

- MD5:
  - CRYPTO_MD5

- SHA-1:
  - CRYPTO_SHA1

- SHA-2:
  - CRYPTO_SHA2_224 (224-bit)
  - CRYPTO_SHA2_256 (256-bit)
  - CRYPTO_SHA2_384 (384-bit)
  - CRYPTO_SHA2_512 (512-bit)

- RIPEMD-160:
  - CRYPTO_RIPEMD160 (as hash function)
  - CRYPTO_RIPEMD160_HMAC

**Message Authentication Codes**

- AES-GMAC (Galois Message Authentication Code):
  - CRYPTO_AES_128_GMAC (128-bit key)
  - CRYPTO_AES_192_GMAC (192-bit key)
  - CRYPTO_AES_256_GMAC (256-bit key)
  - CRYPTO_AES_GMAC (generic)

- AES-CMAC (Cipher-based Message Authentication Code):
  - CRYPTO_AES_CMAC
  - CRYPTO_AES_128_CMAC (128-bit)

- Poly1305:
  - CRYPTO_POLY1305
  - CRYPTO_CHACHA20_POLY1305
  - CRYPTO_CHACHA20_POLY1305_MAC

**Stream Ciphers**

- ChaCha20:
  - CRYPTO_CHACHA20_POLY1305 (with Poly1305 MAC)

Integrity and Checksums
------------------------

- CRC-32:
  - CRYPTO_CRC32

- Extended Sequence Numbers (ESN):
  - CRYPTO_ESN

Compression
-----------

- Deflate Compression:
  - CRYPTO_DEFLATE_COMP

Usage
=====

The Crypto API is accessed through the cryptodev interface, which provides ioctl commands for initializing cryptographic sessions and performing operations.

Basic Usage Pattern
-------------------

1. Open the cryptodev device (/dev/crypto)
2. Initialize a cryptographic session with desired algorithm
3. Submit crypto operations (encrypt/decrypt/hash)
4. Close the session when done

For more details, refer to the cryptodev.h header file and specific driver documentation.
