/****************************************************************************
 * net/vsock/vsock_addr.c
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

#include <errno.h>
#include <string.h>

#include "vsock/vsock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int vsock_addr_is_valid(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR const struct sockaddr_vm *vmaddr =
    (FAR const struct sockaddr_vm *)addr;

  if (addrlen < sizeof(struct sockaddr_vm))
    {
      return -EINVAL;
    }

  if (vmaddr == NULL)
    {
      return -EFAULT;
    }

  if (vmaddr->svm_family != AF_VSOCK)
    {
      return -EAFNOSUPPORT;
    }

  if (vmaddr->svm_flags & ~VMADDR_FLAG_TO_HOST)
    {
      return -EINVAL;
    }

  return 0;
}

int vsock_addr_set(FAR struct sockaddr_vm *vmaddr,
                   FAR const struct sockaddr *addr,
                   socklen_t addrlen)
{
  int ret;

  ret = vsock_addr_is_valid(addr, addrlen);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(vmaddr, addr, sizeof(struct sockaddr_vm));
  return 0;
}

int vsock_addr_get(FAR const struct sockaddr_vm *vmaddr,
                   FAR struct sockaddr *addr,
                   FAR socklen_t *addrlen)
{
  if (addr == NULL || addrlen == NULL ||
      *addrlen < sizeof(struct sockaddr_vm))
    {
      return -EINVAL;
    }

  memcpy(addr, vmaddr, sizeof(struct sockaddr_vm));
  *addrlen = sizeof(struct sockaddr_vm);
  return 0;
}
