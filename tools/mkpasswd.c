/****************************************************************************
 * tools/mkpasswd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Description:
 *   Host build tool that generates a NuttX /etc/passwd entry with a
 *   PBKDF2-HMAC-SHA256 password hash.  This is a pure C replacement for the
 *   former tools/mkpasswd.py, removing the Python dependency from the build.
 *
 *   The hash format is identical to that used at runtime by:
 *     apps/fsutils/passwd/passwd_encrypt.c
 *     apps/fsutils/passwd/passwd_verify.c
 *
 * Usage:
 *   mkpasswd --user <name> --password <pass> [options] [-o <output>]
 *
 * Options:
 *   --user       <str>  Username (required)
 *   --password   <str>  Plaintext password (required, not stored in output)
 *   --uid        <int>  User ID          (default: 0)
 *   --gid        <int>  Group ID         (default: 0)
 *   --home       <str>  Home directory   (default: /)
 *   --iterations <int>  PBKDF2 iterations (default: 10000)
 *   -o           <path> Output file      (default: stdout)
 *
 * Output format (matches NuttX passwd file format):
 *   username:$pbkdf2-sha256$<iter>$<salt>$<hash>:uid:gid:home
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef _POSIX_C_SOURCE
#  define _POSIX_C_SOURCE 200809L
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#ifndef CONFIG_WINDOWS_NATIVE
#  include <fcntl.h>
#  include <sys/random.h>
#  include <sys/stat.h>
#  include <unistd.h>
#else
#  include <direct.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MKPASSWD_NL              "\n\n"
#define PASSWD_MCF_PREFIX        "$pbkdf2-sha256$"
#define PASSWD_SALT_BYTES        16
#define PASSWD_HASH_BYTES        32
#define MAX_ENCRYPTED            96
#define MAX_PASSWORD             256
#define MIN_PASSWORD             8
#define DEFAULT_ITERATIONS       10000
#define MIN_ITERATIONS           1000
#define MAX_ITERATIONS           200000

static const char g_base64url[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

/****************************************************************************
 * Private Types (minimal SHA-256)
 ****************************************************************************/

struct sha256_ctx
{
  uint32_t state[8];
  uint64_t bitlen;
  uint8_t  data[64];
  uint32_t datalen;
};

/****************************************************************************
 * Private Functions (SHA-256 + HMAC-SHA256 + PBKDF2)
 ****************************************************************************/

static uint32_t rotr32(uint32_t x, uint32_t n)
{
  return (x >> n) | (x << (32 - n));
}

static void sha256_transform(struct sha256_ctx *ctx,
                             const uint8_t data[64])
{
  static const uint32_t k[64] =
  {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
  };

  uint32_t m[64];
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t f;
  uint32_t g;
  uint32_t h;
  uint32_t t1;
  uint32_t t2;
  uint32_t s0;
  uint32_t s1;
  int i;

  for (i = 0; i < 16; i++)
    {
      m[i] = ((uint32_t)data[i * 4] << 24) |
             ((uint32_t)data[i * 4 + 1] << 16) |
             ((uint32_t)data[i * 4 + 2] << 8) |
             ((uint32_t)data[i * 4 + 3]);
    }

  for (i = 16; i < 64; i++)
    {
      s0 = rotr32(m[i - 15], 7) ^ rotr32(m[i - 15], 18) ^
           (m[i - 15] >> 3);
      s1 = rotr32(m[i - 2], 17) ^ rotr32(m[i - 2], 19) ^
           (m[i - 2] >> 10);

      m[i] = m[i - 16] + s0 + m[i - 7] + s1;
    }

  a = ctx->state[0];
  b = ctx->state[1];
  c = ctx->state[2];
  d = ctx->state[3];
  e = ctx->state[4];
  f = ctx->state[5];
  g = ctx->state[6];
  h = ctx->state[7];

  for (i = 0; i < 64; i++)
    {
      t1 = h + (rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25)) +
           ((e & f) ^ ((~e) & g)) + k[i] + m[i];
      t2 = (rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22)) +
           ((a & b) ^ (a & c) ^ (b & c));
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

static void sha256_init(struct sha256_ctx *ctx)
{
  ctx->datalen = 0;
  ctx->bitlen  = 0;
  ctx->state[0] = 0x6a09e667;
  ctx->state[1] = 0xbb67ae85;
  ctx->state[2] = 0x3c6ef372;
  ctx->state[3] = 0xa54ff53a;
  ctx->state[4] = 0x510e527f;
  ctx->state[5] = 0x9b05688c;
  ctx->state[6] = 0x1f83d9ab;
  ctx->state[7] = 0x5be0cd19;
}

static void sha256_update(struct sha256_ctx *ctx,
                          const uint8_t *data, size_t len)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      ctx->data[ctx->datalen] = data[i];
      ctx->datalen++;
      if (ctx->datalen == 64)
        {
          sha256_transform(ctx, ctx->data);
          ctx->bitlen += 512;
          ctx->datalen = 0;
        }
    }
}

static void sha256_final(struct sha256_ctx *ctx, uint8_t hash[32])
{
  uint32_t i;
  uint32_t j;

  i = ctx->datalen;

  if (ctx->datalen < 56)
    {
      ctx->data[i++] = 0x80;
      while (i < 56)
        {
          ctx->data[i++] = 0x00;
        }
    }
  else
    {
      ctx->data[i++] = 0x80;
      while (i < 64)
        {
          ctx->data[i++] = 0x00;
        }

      sha256_transform(ctx, ctx->data);
      memset(ctx->data, 0, 56);
    }

  ctx->bitlen += (uint64_t)ctx->datalen * 8;
  ctx->data[63] = (uint8_t)(ctx->bitlen);
  ctx->data[62] = (uint8_t)(ctx->bitlen >> 8);
  ctx->data[61] = (uint8_t)(ctx->bitlen >> 16);
  ctx->data[60] = (uint8_t)(ctx->bitlen >> 24);
  ctx->data[59] = (uint8_t)(ctx->bitlen >> 32);
  ctx->data[58] = (uint8_t)(ctx->bitlen >> 40);
  ctx->data[57] = (uint8_t)(ctx->bitlen >> 48);
  ctx->data[56] = (uint8_t)(ctx->bitlen >> 56);
  sha256_transform(ctx, ctx->data);

  for (i = 0; i < 4; i++)
    {
      for (j = 0; j < 8; j++)
        {
          hash[i + (j * 4)] = (uint8_t)((ctx->state[j] >>
                                         (24 - i * 8)) & 0xff);
        }
    }
}

static void hmac_sha256(const uint8_t *key, size_t keylen,
                        const uint8_t *data, size_t datalen,
                        uint8_t mac[32])
{
  struct sha256_ctx ctx;
  uint8_t k_ipad[64];
  uint8_t k_opad[64];
  uint8_t tk[32];
  size_t i;

  if (keylen > 64)
    {
      sha256_init(&ctx);
      sha256_update(&ctx, key, keylen);
      sha256_final(&ctx, tk);
      key    = tk;
      keylen = 32;
    }

  memset(k_ipad, 0, sizeof(k_ipad));
  memset(k_opad, 0, sizeof(k_opad));
  memcpy(k_ipad, key, keylen);
  memcpy(k_opad, key, keylen);

  for (i = 0; i < 64; i++)
    {
      k_ipad[i] ^= 0x36;
      k_opad[i] ^= 0x5c;
    }

  sha256_init(&ctx);
  sha256_update(&ctx, k_ipad, 64);
  sha256_update(&ctx, data, datalen);
  sha256_final(&ctx, mac);

  sha256_init(&ctx);
  sha256_update(&ctx, k_opad, 64);
  sha256_update(&ctx, mac, 32);
  sha256_final(&ctx, mac);
}

static int pbkdf2_hmac_sha256(const uint8_t *pass, size_t passlen,
                              const uint8_t *salt, size_t saltlen,
                              uint32_t iterations,
                              uint8_t *out, size_t outlen)
{
  uint8_t u[32];
  uint8_t t[32];
  uint8_t saltblk[64];
  size_t generated = 0;
  uint32_t block;
  uint32_t i;
  uint32_t j;

  if (iterations == 0 || outlen == 0 || saltlen + 4 > sizeof(saltblk))
    {
      return -1;
    }

  for (block = 1; generated < outlen; block++)
    {
      memcpy(saltblk, salt, saltlen);
      saltblk[saltlen + 0] = (uint8_t)((block >> 24) & 0xff);
      saltblk[saltlen + 1] = (uint8_t)((block >> 16) & 0xff);
      saltblk[saltlen + 2] = (uint8_t)((block >> 8) & 0xff);
      saltblk[saltlen + 3] = (uint8_t)(block & 0xff);

      hmac_sha256(pass, passlen, saltblk, saltlen + 4, u);
      memcpy(t, u, sizeof(t));

      for (i = 1; i < iterations; i++)
        {
          hmac_sha256(pass, passlen, u, sizeof(u), u);
          for (j = 0; j < 32; j++)
            {
              t[j] ^= u[j];
            }
        }

      if (outlen - generated >= 32)
        {
          memcpy(out + generated, t, 32);
          generated += 32;
        }
      else
        {
          memcpy(out + generated, t, outlen - generated);
          generated = outlen;
        }
    }

  return 0;
}

static int fill_random(uint8_t *buf, size_t len)
{
#ifndef CONFIG_WINDOWS_NATIVE
  ssize_t nread;
  int fd;

#  ifdef SYS_getrandom
  nread = getrandom(buf, len, 0);
  if (nread == (ssize_t)len)
    {
      return 0;
    }
#  endif

  fd = open("/dev/urandom", O_RDONLY);
  if (fd < 0)
    {
      return -1;
    }

  nread = read(fd, buf, len);
  close(fd);

  return nread == (ssize_t)len ? 0 : -1;
#else
  (void)buf;
  (void)len;
  return -1;
#endif
}

static int base64url_encode(const uint8_t *in, size_t inlen,
                            char *out, size_t outlen)
{
  uint32_t acc = 0;
  size_t i;
  size_t o = 0;
  int bits = 0;

  for (i = 0; i < inlen; i++)
    {
      acc = (acc << 8) | in[i];
      bits += 8;

      while (bits >= 6)
        {
          if (o + 1 >= outlen)
            {
              return -1;
            }

          bits -= 6;
          out[o++] = g_base64url[(acc >> bits) & 0x3f];
        }
    }

  if (bits > 0)
    {
      if (o + 1 >= outlen)
        {
          return -1;
        }

      out[o++] = g_base64url[(acc << (6 - bits)) & 0x3f];
    }

  if (o >= outlen)
    {
      return -1;
    }

  out[o] = '\0';
  return 0;
}

static int validate_password_complexity(const char *password)
{
  const char *specials = "!@#$%^&*()_+-=[]{}|;:,.<>?";
  const char *p;
  int has_upper   = 0;
  int has_lower   = 0;
  int has_digit   = 0;
  int has_special = 0;

  if (strlen(password) < MIN_PASSWORD)
    {
      fprintf(stderr, "\nError: password must be at least 8 characters\n\n");
      return -1;
    }

  if (strlen(password) > MAX_PASSWORD)
    {
      fprintf(stderr, "\nError: password must be at most %d characters\n\n",
              MAX_PASSWORD);
      return -1;
    }

  for (p = password; *p; p++)
    {
      if (isupper((unsigned char)*p))
        {
          has_upper = 1;
        }
      else if (islower((unsigned char)*p))
        {
          has_lower = 1;
        }
      else if (isdigit((unsigned char)*p))
        {
          has_digit = 1;
        }
      else if (strchr(specials, *p))
        {
          has_special = 1;
        }
    }

  if (!has_upper)
    {
      fprintf(stderr,
              "\nError: password must contain at least one uppercase "
              "letter (A-Z)\n\n");
      return -1;
    }

  if (!has_lower)
    {
      fprintf(stderr,
              "\nError: password must contain at least one lowercase "
              "letter (a-z)\n\n");
      return -1;
    }

  if (!has_digit)
    {
      fprintf(stderr,
              "\nError: password must contain at least one digit "
              "(0-9)\n\n");
      return -1;
    }

  if (!has_special)
    {
      fprintf(stderr,
              "\nError: password must contain at least one special "
              "character (!@#$%%^&*()_+-=[]{}|;:,.<>?)\n\n");
      return -1;
    }

  return 0;
}

static int passwd_hash(const char *password,
                       uint32_t iterations,
                       char encrypted[MAX_ENCRYPTED + 1])
{
  uint8_t salt[PASSWD_SALT_BYTES];
  uint8_t hash[PASSWD_HASH_BYTES];
  char salt_b64[32];
  char hash_b64[48];
  size_t passlen;
  int ret;

  passlen = strlen(password);

  if (fill_random(salt, sizeof(salt)) < 0)
    {
      fputs(MKPASSWD_NL, stderr);
      fprintf(stderr, "mkpasswd: cannot obtain random salt\n");
      return -1;
    }

  if (pbkdf2_hmac_sha256((const uint8_t *)password, passlen,
                         salt, sizeof(salt), iterations,
                         hash, sizeof(hash)) < 0)
    {
      return -1;
    }

  if (base64url_encode(salt, sizeof(salt), salt_b64,
                       sizeof(salt_b64)) < 0 ||
      base64url_encode(hash, sizeof(hash), hash_b64,
                       sizeof(hash_b64)) < 0)
    {
      return -1;
    }

  ret = snprintf(encrypted, MAX_ENCRYPTED + 1,
                 PASSWD_MCF_PREFIX "%u$%s$%s",
                 iterations, salt_b64, hash_b64);
  if (ret < 0 || (size_t)ret > MAX_ENCRYPTED)
    {
      return -1;
    }

  return 0;
}

static int mkdir_p(const char *path)
{
  char  *tmp;
  char  *p;
  size_t len;

  tmp = strdup(path);
  if (tmp == NULL)
    {
      return -1;
    }

  len = strlen(tmp);

  if (len > 0 && tmp[len - 1] == '/')
    {
      tmp[len - 1] = '\0';
    }

  for (p = tmp + 1; *p != '\0'; p++)
    {
      if (*p == '/')
        {
          *p = '\0';
#ifndef CONFIG_WINDOWS_NATIVE
          mkdir(tmp, 0755);
#else
          mkdir(tmp);
#endif
          *p = '/';
        }
    }

#ifndef CONFIG_WINDOWS_NATIVE
  mkdir(tmp, 0755);
#else
  mkdir(tmp);
#endif
  free(tmp);
  return 0;
}

static void show_usage(const char *progname)
{
  fprintf(stderr,
          "Usage: %s --user <name> --password <pass> [options] [-o <file>]\n"
          "\n"
          "Options:\n"
          "  --user       <str>  Username (required)\n"
          "  --password   <str>  Plaintext password (required)\n"
          "  --uid        <int>  User ID          (default: 0)\n"
          "  --gid        <int>  Group ID         (default: 0)\n"
          "  --home       <str>  Home directory   (default: /)\n"
          "  --iterations <int>  PBKDF2 iterations (default: %d)\n"
          "  -o           <path> Output file      (default: stdout)\n"
          "\n"
          "Output format:  username:$pbkdf2-sha256$...:uid:gid:home\n",
          progname, DEFAULT_ITERATIONS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  const char *user;
  const char *password;
  const char *home;
  const char *outpath;
  FILE *out;
  char encrypted[MAX_ENCRYPTED + 1];
  int uid;
  int gid;
  uint32_t iterations;
  int i;
  int ret;

  user       = NULL;
  password   = NULL;
  home       = "/";
  outpath    = NULL;
  uid        = 0;
  gid        = 0;
  iterations = DEFAULT_ITERATIONS;

  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "--user") == 0 && i + 1 < argc)
        {
          user = argv[++i];
        }
      else if (strcmp(argv[i], "--password") == 0 && i + 1 < argc)
        {
          password = argv[++i];
        }
      else if (strcmp(argv[i], "--uid") == 0 && i + 1 < argc)
        {
          uid = atoi(argv[++i]);
        }
      else if (strcmp(argv[i], "--gid") == 0 && i + 1 < argc)
        {
          gid = atoi(argv[++i]);
        }
      else if (strcmp(argv[i], "--home") == 0 && i + 1 < argc)
        {
          home = argv[++i];
        }
      else if (strcmp(argv[i], "--iterations") == 0 && i + 1 < argc)
        {
          iterations = (uint32_t)strtoul(argv[++i], NULL, 10);
        }
      else if ((strcmp(argv[i], "-o") == 0 ||
                strcmp(argv[i], "--output") == 0) && i + 1 < argc)
        {
          outpath = argv[++i];
        }
      else if (strcmp(argv[i], "--help") == 0 ||
               strcmp(argv[i], "-h") == 0)
        {
          show_usage(argv[0]);
          return 0;
        }
      else
        {
          fputs(MKPASSWD_NL, stderr);
          fprintf(stderr, "mkpasswd: unknown option: %s\n", argv[i]);
          show_usage(argv[0]);
          return 1;
        }
    }

  if (user == NULL)
    {
      fputs(MKPASSWD_NL, stderr);
      fprintf(stderr, "mkpasswd: --user is required\n");
      show_usage(argv[0]);
      return 1;
    }

  if (password == NULL)
    {
      fputs(MKPASSWD_NL, stderr);
      fprintf(stderr, "mkpasswd: --password is required\n");
      show_usage(argv[0]);
      return 1;
    }

  if (validate_password_complexity(password) < 0)
    {
      return 1;
    }

  if (iterations < MIN_ITERATIONS || iterations > MAX_ITERATIONS)
    {
      fputs(MKPASSWD_NL, stderr);
      fprintf(stderr,
              "mkpasswd: --iterations must be between %d and %d\n",
              MIN_ITERATIONS, MAX_ITERATIONS);
      return 1;
    }

  ret = passwd_hash(password, iterations, encrypted);
  if (ret < 0)
    {
      return 1;
    }

  if (outpath != NULL)
    {
      char *dir;
      char *last;

      dir  = strdup(outpath);
      last = strrchr(dir, '/');

      if (last != NULL && last != dir)
        {
          *last = '\0';
          mkdir_p(dir);
        }

      free(dir);

      out = fopen(outpath, "w");
      if (out == NULL)
        {
          fputs(MKPASSWD_NL, stderr);
          fprintf(stderr, "mkpasswd: cannot open output file '%s': %s\n",
                  outpath, strerror(errno));
          return 1;
        }
    }
  else
    {
      out = stdout;
    }

  fprintf(out, "%s:%s:%d:%d:%s\n", user, encrypted, uid, gid, home);

  if (outpath != NULL)
    {
      fclose(out);
    }

  return 0;
}
