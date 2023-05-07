/****************************************************************************
 * libs/libc/misc/lib_fdsan.c
 * Copyright (C) 2018 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <android/fdsan.h>

#include <debug.h>
#include <pthread.h>
#include <stdio.h>
#include <setjmp.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint64_t android_fdsan_get_tag_value(uint64_t tag)
{
  /* Lop off the most significant byte and sign extend. */

  return (uint64_t)((int64_t)(tag << 8) >> 8);
}

const char *android_fdsan_get_tag_type(uint64_t tag)
{
  uint64_t high_bits;
  uint64_t type = tag >> 56;
  switch (type)
    {
      case ANDROID_FDSAN_OWNER_TYPE_FILE:
        return "FILE*";
      case ANDROID_FDSAN_OWNER_TYPE_DIR:
        return "DIR*";
      case ANDROID_FDSAN_OWNER_TYPE_UNIQUE_FD:
        return "unique_fd";
      case ANDROID_FDSAN_OWNER_TYPE_FILEINPUTSTREAM:
        return "FileInputStream";
      case ANDROID_FDSAN_OWNER_TYPE_FILEOUTPUTSTREAM:
        return "FileOutputStream";
      case ANDROID_FDSAN_OWNER_TYPE_RANDOMACCESSFILE:
        return "RandomAccessFile";
      case ANDROID_FDSAN_OWNER_TYPE_PARCELFILEDESCRIPTOR:
        return "ParcelFileDescriptor";
      case ANDROID_FDSAN_OWNER_TYPE_SQLITE:
        return "sqlite";
      case ANDROID_FDSAN_OWNER_TYPE_ART_FDFILE:
        return "ART FdFile";
      case ANDROID_FDSAN_OWNER_TYPE_DATAGRAMSOCKETIMPL:
        return "DatagramSocketImpl";
      case ANDROID_FDSAN_OWNER_TYPE_SOCKETIMPL:
        return "SocketImpl";
      case ANDROID_FDSAN_OWNER_TYPE_ZIPARCHIVE:
        return "ZipArchive";

      case ANDROID_FDSAN_OWNER_TYPE_GENERIC_00:
      default:
        return "native object of unknown type";

      case ANDROID_FDSAN_OWNER_TYPE_GENERIC_FF:

        /********************************************************************
         * If bits 48 to 56 are set,
         * this is a sign-extended generic native pointer
         ********************************************************************/

        high_bits = tag >> 48;
        if (high_bits == (1 << 16) - 1)
          {
            return "native object of unknown type";
          }

        return "Java object of unknown type";
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint64_t android_fdsan_create_owner_tag(android_fdsan_owner_type_t type,
                                        uint64_t tag)
{
  uint64_t result;
  uint64_t mask;

  if (tag == 0)
    {
      return 0;
    }

  DEBUGASSERT((type & 0xff) == type);

  result = (uint64_t)(type) << 56;
  mask = ((uint64_t)1 << 56) - 1;
  result |= tag & mask;
  return result;
}

int android_fdsan_close_with_tag(int fd, uint64_t expected_tag)
{
  int ret;

  android_fdsan_exchange_owner_tag(fd, expected_tag, 0);
  ret = close(fd);

  /**************************************************************************
   * If we were expecting to close with a tag, abort on EBADF.
   **************************************************************************/

  if (expected_tag && ret == -1 && errno == EBADF)
    {
      ferr("double-close of file descriptor %d detected\n", fd);
      PANIC();
    }

  return ret;
}

void android_fdsan_exchange_owner_tag(int fd, uint64_t expected_tag,
                                      uint64_t new_tag)
{
  uint64_t tag;
  int ret;

  ret = ioctl(fd, FIOC_GETTAG, &tag);
  if (ret < 0)
    {
      return;
    }

  if (tag == expected_tag)
    {
      ret = ioctl(fd, FIOC_SETTAG, &new_tag);
      DEBUGASSERT(ret == 0);
    }
  else
    {
      if (expected_tag && tag)
        {
          ferr("failed to exchange ownership of file descriptor: fd %d is "
               "owned by %s 0x%" PRIx64 ", was expected"
               "to be owned by %s 0x%" PRIx64 "\n",
               fd, android_fdsan_get_tag_type(tag),
               android_fdsan_get_tag_value(tag),
               android_fdsan_get_tag_type(expected_tag),
               android_fdsan_get_tag_value(expected_tag));
          PANIC();
        }
      else if (expected_tag && !tag)
        {
          ferr("failed to exchange ownership of file descriptor: fd %d is "
               "unowned, was expected to be owned by %s 0x%" PRIx64 "\n",
               fd, android_fdsan_get_tag_type(expected_tag),
               android_fdsan_get_tag_value(expected_tag));
          PANIC();
        }
      else if (!expected_tag && tag)
        {
          ferr("failed to exchange ownership of file descriptor: fd %d is "
               "owned by %s 0x%" PRIx64 ", was expected to be unowned\n",
               fd, android_fdsan_get_tag_type(tag),
               android_fdsan_get_tag_value(tag));
          PANIC();
        }
      else if (!expected_tag && !tag)
        {
          /******************************************************************
           * This should never happen: our CAS failed,
           * but expected == actual?
           ******************************************************************/

          ferr("fdsan atomic_compare_exchange_strong failed unexpectedly "
               "while exchanging owner tag\n");
          PANIC();
        }
    }
}
