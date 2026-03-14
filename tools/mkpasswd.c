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
 *   TEA-encrypted password hash.
 *
 *   The encryption algorithm and base64 encoding are identical to those
 *   used at runtime by:
 *     libs/libc/misc/lib_tea_encrypt.c
 *     apps/fsutils/passwd/passwd_encrypt.c
 *
 * Usage:
 *   mkpasswd --user <name> --password <pass> [options] [-o <output>]
 *
 * Options:
 *   --user     <str>  Username (required)
 *   --password <str>  Plaintext password (required, not stored in output)
 *   --uid      <int>  User ID          (default: 0)
 *   --gid      <int>  Group ID         (default: 0)
 *   --home     <str>  Home directory   (default: /)
 *   --key1     <hex>  TEA key word 1   (default: 0x12345678)
 *   --key2     <hex>  TEA key word 2   (default: 0x9abcdef0)
 *   --key3     <hex>  TEA key word 3   (default: 0x12345678)
 *   --key4     <hex>  TEA key word 4   (default: 0x9abcdef0)
 *   -o         <path> Output file      (default: stdout)
 *
 * Output format (matches NuttX passwd file format):
 *   username:encrypted_hash:uid:gid:home
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Expose strdup(), mkdir() and other POSIX.1-2008 extensions when
 * compiling with strict C99 mode (-std=c99). Has no effect on C11/GNU
 * builds or MSVC.
 */

#ifndef _POSIX_C_SOURCE
#  define _POSIX_C_SOURCE 200809L
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <sys/stat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TEA key schedule constant (derived from the golden ratio) */

#define TEA_KEY_SCHEDULE_CONSTANT  0x9e3779b9u

/* Password size limits – must match apps/fsutils/passwd/passwd.h.
 * MAX_ENCRYPTED is the max length of the encrypted hash (ASCII chars).
 * MAX_PASSWORD is the max length of the plaintext password.
 */

#define MAX_ENCRYPTED  48
#define MAX_PASSWORD   (3 * MAX_ENCRYPTED / 4)

/* Default TEA key values - must match CONFIG_FSUTILS_PASSWD_KEY1-4 defaults
 * in apps/fsutils/passwd/Kconfig so that the generated hash verifies
 * correctly at runtime when the user has not changed the key config.
 */

#define DEFAULT_KEY1   0x12345678u
#define DEFAULT_KEY2   0x9abcdef0u
#define DEFAULT_KEY3   0x12345678u
#define DEFAULT_KEY4   0x9abcdef0u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* 8-byte block interpreted as bytes, 16-bit halves, or 32-bit words */

union block_u
{
  char     b[8];
  uint16_t h[4];
  uint32_t l[2];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tea_encrypt
 *
 * Description:
 *   Encrypt two 32-bit words in-place using the Tiny Encryption Algorithm.
 *   This is an exact copy of the algorithm in
 *   libs/libc/misc/lib_tea_encrypt.c (public-domain TEA by Wheeler &
 *   Needham), inlined here so that the host tool has no NuttX dependencies.
 *
 * Input Parameters:
 *   value - Two-element array [v0, v1] to encrypt (modified in-place)
 *   key   - Four-element 128-bit key array
 *
 ****************************************************************************/

static void tea_encrypt(uint32_t *value, const uint32_t *key)
{
  uint32_t v0  = value[0];
  uint32_t v1  = value[1];
  uint32_t sum = 0;
  int i;

  for (i = 0; i < 32; i++)
    {
      sum += TEA_KEY_SCHEDULE_CONSTANT;
      v0  += ((v1 << 4) + key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + key[1]);
      v1  += ((v0 << 4) + key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + key[3]);
    }

  value[0] = v0;
  value[1] = v1;
}

/****************************************************************************
 * Name: passwd_base64
 *
 * Description:
 *   Encode the low 6 bits of a byte as a custom base64 character.
 *   Alphabet: A-Z (0-25), a-z (26-51), 0-9 (52-61), + (62), / (63).
 *   The colon ':' is deliberately absent so it never collides with the
 *   passwd field separator.
 *
 *   This matches passwd_base64() in apps/fsutils/passwd/passwd_encrypt.c.
 *
 ****************************************************************************/

static char passwd_base64(uint8_t binary)
{
  binary &= 63;

  if (binary < 26)
    {
      return (char)('A' + binary);
    }

  binary -= 26;
  if (binary < 26)
    {
      return (char)('a' + binary);
    }

  binary -= 26;
  if (binary < 10)
    {
      return (char)('0' + binary);
    }

  binary -= 10;
  if (binary == 0)
    {
      return '+';
    }

  return '/';
}

/****************************************************************************
 * Name: passwd_encrypt
 *
 * Description:
 *   Encrypt a plaintext password string and store the result as a
 *   NUL-terminated base64 string in `encrypted`.
 *
 *   Algorithm (identical to apps/fsutils/passwd/passwd_encrypt.c):
 *     1. Process the password in 8-byte gulps, padding short gulps with
 *        ASCII spaces.
 *     2. TEA-encrypt each 8-byte gulp as two uint32_t words.
 *     3. Interpret the result as four uint16_t half-words.
 *     4. Stream-encode those half-words 6 bits at a time using the custom
 *        base64 alphabet above.
 *
 * Input Parameters:
 *   password  - NUL-terminated plaintext password
 *   key       - Four-element 128-bit TEA key
 *   encrypted - Output buffer (at least MAX_ENCRYPTED + 1 bytes)
 *
 * Returned Value:
 *   0 on success, -1 on error (password too long).
 *
 ****************************************************************************/

static int passwd_encrypt(const char *password,
                          const uint32_t *key,
                          char encrypted[MAX_ENCRYPTED + 1])
{
  union block_u value;
  const char   *src;
  char         *dest;
  uint32_t      tmp;
  uint8_t       remainder;
  int           remaining;
  int           gulpsize;
  int           nbits;
  int           i;

  remaining = (int)strlen(password);
  if (remaining > MAX_PASSWORD)
    {
      fprintf(stderr, "mkpasswd: password too long (max %d characters)\n",
              MAX_PASSWORD);
      return -1;
    }

  src       = password;
  dest      = encrypted;
  *dest     = '\0';
  remainder = 0;
  nbits     = 0;

  for (; remaining > 0; remaining -= gulpsize)
    {
      /* Copy up to 8 bytes into the block, padding the rest with spaces */

      gulpsize = 8;
      if (gulpsize > remaining)
        {
          gulpsize = remaining;
        }

      for (i = 0; i < gulpsize; i++)
        {
          value.b[i] = *src++;
        }

      for (; i < 8; i++)
        {
          value.b[i] = ' ';
        }

      /* TEA-encrypt the block in-place */

      tea_encrypt(value.l, key);

      /* Stream-encode the four 16-bit half-words into base64 */

      tmp = remainder;

      for (i = 0; i < 4; i++)
        {
          tmp    = ((uint32_t)value.h[i] << nbits) | tmp;
          nbits += 16;

          while (nbits >= 6)
            {
              *dest++ = passwd_base64((uint8_t)(tmp & 0x3f));
              tmp   >>= 6;
              nbits  -= 6;
            }
        }

      remainder = (uint8_t)tmp;
      *dest     = '\0';
    }

  /* Flush any remaining bits */

  if (nbits > 0)
    {
      *dest++ = passwd_base64(remainder);
      *dest   = '\0';
    }

  return 0;
}

/****************************************************************************
 * Name: parse_uint32_hex
 *
 * Description:
 *   Parse a hex string (with or without leading "0x"/"0X") into a uint32_t.
 *   Returns 0 on success, -1 on parse error.
 *
 ****************************************************************************/

static int parse_uint32_hex(const char *str, uint32_t *out)
{
  char *endptr;
  unsigned long val;

  if (str == NULL || *str == '\0')
    {
      return -1;
    }

  errno = 0;
  val   = strtoul(str, &endptr, 0);  /* base 0: auto-detect 0x prefix */
  if (errno != 0 || *endptr != '\0')
    {
      return -1;
    }

  *out = (uint32_t)val;
  return 0;
}

/****************************************************************************
 * Name: mkdir_p
 *
 * Description:
 *   Create all directory components in `path`, like "mkdir -p".
 *   Returns 0 on success, -1 on error.
 *
 ****************************************************************************/

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

  /* Strip trailing slash */

  if (len > 0 && tmp[len - 1] == '/')
    {
      tmp[len - 1] = '\0';
    }

  for (p = tmp + 1; *p != '\0'; p++)
    {
      if (*p == '/')
        {
          *p = '\0';
#ifdef CONFIG_WINDOWS_NATIVE
          mkdir(tmp);
#else
          mkdir(tmp, 0755);
#endif
          *p = '/';
        }
    }

#ifdef CONFIG_WINDOWS_NATIVE
  mkdir(tmp);
#else
  mkdir(tmp, 0755);
#endif
  free(tmp);
  return 0;
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr,
          "Usage: %s --user <name> --password <pass> [options] [-o <file>]\n"
          "\n"
          "Options:\n"
          "  --user     <str>  Username (required)\n"
          "  --password <str>  Plaintext password (required)\n"
          "  --uid      <int>  User ID          (default: 0)\n"
          "  --gid      <int>  Group ID         (default: 0)\n"
          "  --home     <str>  Home directory   (default: /)\n"
          "  --key1     <hex>  TEA key word 1   (default: 0x%08x)\n"
          "  --key2     <hex>  TEA key word 2   (default: 0x%08x)\n"
          "  --key3     <hex>  TEA key word 3   (default: 0x%08x)\n"
          "  --key4     <hex>  TEA key word 4   (default: 0x%08x)\n"
          "  -o         <path> Output file      (default: stdout)\n"
          "\n"
          "Output format:  username:encrypted_hash:uid:gid:home\n",
          progname,
          DEFAULT_KEY1, DEFAULT_KEY2, DEFAULT_KEY3, DEFAULT_KEY4);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  const char *user     = NULL;
  const char *password = NULL;
  const char *home     = "/";
  const char *outpath  = NULL;
  int         uid      = 0;
  int         gid      = 0;
  uint32_t    key[4]   =
    {
      DEFAULT_KEY1, DEFAULT_KEY2, DEFAULT_KEY3, DEFAULT_KEY4
    };

  char encrypted[MAX_ENCRYPTED + 1];
  FILE *out;
  int   opt;
  int   ret;

  static const struct option g_long_options[] =
    {
      { "user",     required_argument, NULL, 'u' },
      { "password", required_argument, NULL, 'p' },
      { "uid",      required_argument, NULL, 'i' },
      { "gid",      required_argument, NULL, 'g' },
      { "home",     required_argument, NULL, 'd' },
      { "key1",     required_argument, NULL, '1' },
      { "key2",     required_argument, NULL, '2' },
      { "key3",     required_argument, NULL, '3' },
      { "key4",     required_argument, NULL, '4' },
      { "output",   required_argument, NULL, 'o' },
      { "help",     no_argument,       NULL, 'h' },
      { NULL,       0,                 NULL,  0  },
    };

  while ((opt = getopt_long(argc, argv, "u:p:i:g:d:o:h",
                            g_long_options, NULL)) != -1)
    {
      switch (opt)
        {
          case 'u':
            user = optarg;
            break;

          case 'p':
            password = optarg;
            break;

          case 'i':
            uid = atoi(optarg);
            break;

          case 'g':
            gid = atoi(optarg);
            break;

          case 'd':
            home = optarg;
            break;

          case '1':
            if (parse_uint32_hex(optarg, &key[0]) < 0)
              {
                fprintf(stderr,
                        "mkpasswd: invalid --key1 value: %s\n",
                        optarg);
                return 1;
              }

            break;

          case '2':
            if (parse_uint32_hex(optarg, &key[1]) < 0)
              {
                fprintf(stderr,
                        "mkpasswd: invalid --key2 value: %s\n",
                        optarg);
                return 1;
              }

            break;

          case '3':
            if (parse_uint32_hex(optarg, &key[2]) < 0)
              {
                fprintf(stderr,
                        "mkpasswd: invalid --key3 value: %s\n",
                        optarg);
                return 1;
              }

            break;

          case '4':
            if (parse_uint32_hex(optarg, &key[3]) < 0)
              {
                fprintf(stderr,
                        "mkpasswd: invalid --key4 value: %s\n",
                        optarg);
                return 1;
              }

            break;

          case 'o':
            outpath = optarg;
            break;

          case 'h':
            show_usage(argv[0]);
            return 0;

          default:
            show_usage(argv[0]);
            return 1;
        }
    }

  /* Validate required arguments */

  if (user == NULL)
    {
      fprintf(stderr, "mkpasswd: --user is required\n");
      show_usage(argv[0]);
      return 1;
    }

  if (password == NULL)
    {
      fprintf(stderr, "mkpasswd: --password is required\n");
      show_usage(argv[0]);
      return 1;
    }

  if (password[0] == '\0')
    {
      fprintf(stderr, "mkpasswd: --password must not be empty\n");
      return 1;
    }

  /* Encrypt the password using TEA + custom base64.
   * Only the hash is written to the output file; the plaintext is never
   * stored in firmware.
   */

  ret = passwd_encrypt(password, key, encrypted);
  if (ret < 0)
    {
      return 1;
    }

  /* Open the output stream */

  if (outpath != NULL)
    {
      /* Create parent directory if it does not exist */

      char *dir  = strdup(outpath);
      char *last = strrchr(dir, '/');

      if (last != NULL && last != dir)
        {
          *last = '\0';
          mkdir_p(dir);
        }

      free(dir);

      out = fopen(outpath, "w");
      if (out == NULL)
        {
          fprintf(stderr, "mkpasswd: cannot open output file '%s': %s\n",
                  outpath, strerror(errno));
          return 1;
        }
    }
  else
    {
      out = stdout;
    }

  /* Write the passwd entry.
   * Format: username:encrypted_hash:uid:gid:home
   * This matches the format expected by apps/fsutils/passwd/passwd_find.c
   * and the existing NuttX /etc/passwd files.
   */

  fprintf(out, "%s:%s:%d:%d:%s\n", user, encrypted, uid, gid, home);

  if (outpath != NULL)
    {
      fclose(out);
    }

  return 0;
}
