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

void vsock_addr_init(FAR struct sockaddr_vm *addr, uint64_t cid,
                     uint32_t port)
{
  memset(addr, 0, sizeof(*addr));
  addr->svm_family = AF_VSOCK;
  addr->svm_cid    = cid;
  addr->svm_port   = port;
}

bool vsock_addr_equal(FAR const struct sockaddr_vm *addr1,
                      FAR const struct sockaddr_vm *addr2)
{
  return addr1->svm_cid == addr2->svm_cid &&
         addr1->svm_port == addr2->svm_port;
}
