/****************************************************************************
 * syscall/syscall_uaccess.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdarg.h>
#include <string.h>
#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/syslog/syslog.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool uaccess_arg(uintptr_t arg)
{
  return up_addrenv_user_vaddr(arg) ||
         up_addrenv_va_to_pa((FAR void *)arg) == 0;
}

static FAR struct iovec *uaccess_iov(FAR const struct iovec *iov,
                                     size_t iovcnt)
{
  FAR struct iovec *copy;
  FAR void *base;
  size_t size;
  size_t i;

  if (iovcnt > SIZE_MAX / sizeof(struct iovec))
    {
      return NULL;
    }

  size = iovcnt * sizeof(struct iovec);
  uaccess_check(iov, size);
  copy = kmm_malloc(size);
  if (copy == NULL)
    {
      return NULL;
    }

  memcpy(copy, iov, size);
  for (i = 0; i < iovcnt; i++)
    {
      base = copy[i].iov_base;
      if (copy[i].iov_len > 0 && !uaccess_ok(base, copy[i].iov_len))
        {
          kmm_free(copy);
          uaccess_fault(base);
        }
    }

  return copy;
}

static ssize_t uaccess_rw(ssize_t (*rw)(int, FAR const struct iovec *, int),
                          int fildes, FAR const struct iovec *iov,
                          int iovcnt)
{
  FAR struct iovec *copy;
  ssize_t ret;

  if (iovcnt <= 0)
    {
      return rw(fildes, iov, iovcnt);
    }

  copy = uaccess_iov(iov, iovcnt);
  if (copy == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  ret = rw(fildes, copy, iovcnt);
  kmm_free(copy);
  return ret;
}

static int uaccess_syslog(int priority, FAR const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start(ap, fmt);
  ret = nx_vsyslog(priority, fmt, &ap);
  va_end(ap);
  return ret;
}

#ifdef CONFIG_NET
static ssize_t uaccess_msg(int sockfd, FAR struct msghdr *msg, int flags,
                           bool recv)
{
  struct msghdr copy;
  ssize_t ret;

  if (msg == NULL)
    {
      return recv ? recvmsg(sockfd, NULL, flags) :
                    sendmsg(sockfd, NULL, flags);
    }

  uaccess_check(msg, sizeof(*msg));
  memcpy(&copy, msg, sizeof(copy));

  if (copy.msg_name != NULL)
    {
      uaccess_check(copy.msg_name, copy.msg_namelen);
    }

  if (copy.msg_control != NULL)
    {
      uaccess_check(copy.msg_control, copy.msg_controllen);
    }

  if (copy.msg_iovlen > 0)
    {
      copy.msg_iov = uaccess_iov(copy.msg_iov, copy.msg_iovlen);
      if (copy.msg_iov == NULL)
        {
          set_errno(ENOMEM);
          return ERROR;
        }
    }

  if (recv)
    {
      ret = recvmsg(sockfd, &copy, flags);
      msg->msg_namelen    = copy.msg_namelen;
      msg->msg_controllen = copy.msg_controllen;
      msg->msg_flags      = copy.msg_flags;
    }
  else
    {
      ret = sendmsg(sockfd, &copy, flags);
    }

  if (copy.msg_iovlen > 0)
    {
      kmm_free(copy.msg_iov);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL
int uaccess_boardctl(unsigned int cmd, uintptr_t arg)
{
  if (!uaccess_arg(arg))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  switch (cmd)
    {
#ifdef CONFIG_BOARDCTL_ROMDISK
      case BOARDIOC_ROMDISK:
        set_errno(EPERM);
        return ERROR;
#endif

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
      case BOARDIOC_USBDEV_CONTROL:
        {
          struct boardioc_usbdev_ctrl_s ctrl;

          uaccess_check((FAR const void *)arg, sizeof(ctrl));
          memcpy(&ctrl, (FAR const void *)arg, sizeof(ctrl));
          if (ctrl.handle != NULL)
            {
              uaccess_check(ctrl.handle, sizeof(*ctrl.handle));
            }

          if (ctrl.action == BOARDIOC_USBDEV_DISCONNECT &&
              ctrl.usbdev != BOARDIOC_USBDEV_CDCACM)
            {
              set_errno(EPERM);
              return ERROR;
            }

          return boardctl(cmd, (uintptr_t)&ctrl);
        }
#endif

      default:
        return boardctl(cmd, arg);
    }
}
#endif

int uaccess_fcntl(int fd, int cmd, ...)
{
  uintptr_t arg;
  va_list ap;

  va_start(ap, cmd);
  arg = va_arg(ap, uintptr_t);
  va_end(ap);

  switch (cmd)
    {
      case F_GETLK:
      case F_SETLK:
      case F_SETLKW:
        uaccess_check((FAR const void *)arg, sizeof(struct flock));
        break;

      case F_GETPATH:
        uaccess_check((FAR const void *)arg, PATH_MAX);
        break;

      default:
        if (!uaccess_arg(arg))
          {
            set_errno(EFAULT);
            return ERROR;
          }
        break;
    }

  return fcntl(fd, cmd, arg);
}

int uaccess_ioctl(int fd, int req, ...)
{
  unsigned long arg;
  va_list ap;

  va_start(ap, req);
  arg = va_arg(ap, unsigned long);
  va_end(ap);

  switch (req)
    {
      case BIOC_XIPBASE:
      case DIOC_GETPRIV:
#ifdef CONFIG_CDCACM
      case CAIOC_REGISTERCB:
#endif
        set_errno(EPERM);
        return ERROR;

      default:
        break;
    }

  if (!uaccess_arg(arg))
    {
      set_errno(EFAULT);
      return ERROR;
    }

  return ioctl(fd, req, arg);
}

#ifndef CONFIG_DISABLE_PTHREAD
int uaccess_nx_pthread_create(pthread_trampoline_t trampoline,
                              FAR pthread_t *thread,
                              FAR const pthread_attr_t *attr,
                              pthread_startroutine_t entry,
                              pthread_addr_t arg)
{
  pthread_attr_t copy;

  if (attr == NULL)
    {
      return nx_pthread_create(trampoline, thread, NULL, entry, arg);
    }

  memcpy(&copy, attr, sizeof(copy));
  if (copy.stackaddr != NULL)
    {
      uaccess_check(copy.stackaddr, copy.stacksize);
    }

  return nx_pthread_create(trampoline, thread, &copy, entry, arg);
}
#endif

int uaccess_nx_vsyslog(int priority, FAR const IPTR char *src,
                       FAR va_list *ap)
{
  return uaccess_syslog(priority, "%s", src);
}

int uaccess_prctl(int option, ...)
{
  uintptr_t arg1;
  uintptr_t arg2;
  va_list ap;

  va_start(ap, option);
  arg1 = va_arg(ap, uintptr_t);
  arg2 = va_arg(ap, uintptr_t);
  va_end(ap);

  switch (option)
    {
      case PR_SET_NAME:
      case PR_GET_NAME:
      case PR_SET_NAME_EXT:
      case PR_GET_NAME_EXT:
        uaccess_check((FAR const void *)arg1, CONFIG_TASK_NAME_SIZE + 1);
        break;

      default:
        break;
    }

  return prctl(option, arg1, arg2);
}

ssize_t uaccess_readv(int fildes, FAR const struct iovec *iov, int iovcnt)
{
  return uaccess_rw(readv, fildes, iov, iovcnt);
}

ssize_t uaccess_writev(int fildes, FAR const struct iovec *iov, int iovcnt)
{
  return uaccess_rw(writev, fildes, iov, iovcnt);
}

#ifdef CONFIG_NET
ssize_t uaccess_recvmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  return uaccess_msg(sockfd, msg, flags, true);
}

ssize_t uaccess_sendmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  return uaccess_msg(sockfd, msg, flags, false);
}
#endif

#endif /* CONFIG_BUILD_KERNEL */
