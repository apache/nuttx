/****************************************************************************
 * drivers/usbhost/usbhost_devaddr.c
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

/* Manage USB device addresses */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem and usbhost_givesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static int usbhost_takesem(FAR struct usbhost_devaddr_s *devgen)
{
  return nxsem_wait_uninterruptible(&devgen->exclsem);
}

#define usbhost_givesem(devgen) nxsem_post(&devgen->exclsem)

/****************************************************************************
 * Name: usbhost_devaddr_allocate
 *
 * Description:
 *   Allocate a new unique device address.
 *
 * Assumptions:
 *   Caller hold the exclsem
 *
 ****************************************************************************/

static int usbhost_devaddr_allocate(FAR struct usbhost_devaddr_s *devgen)
{
  uint8_t startaddr = devgen->next;
  uint8_t devaddr;
  int index;
  int bitno;

  /* Loop until we find a valid device address */

  for (; ; )
    {
      /* Try the next device address */

      devaddr = devgen->next;
      if (devgen->next >= 0x7f)
        {
          devgen->next = 1;
        }
      else
        {
          devgen->next++;
        }

      /* Is this address already allocated? */

      index = devaddr >> 5;
      bitno = devaddr & 0x1f;
      if ((devgen->alloctab[index] & (1 << bitno)) == 0)
        {
          /* No... allocate it now */

          devgen->alloctab[index] |= (1 << bitno);
          return (int)devaddr;
        }

      /* This address has already been allocated.  The following logic will
       * prevent (unexpected) infinite loops.
       */

      if (startaddr == devaddr)
        {
          /* We are back where we started... the are no free device address */

          return -ENOMEM;
        }
    }
}

/****************************************************************************
 * Name: usbhost_devaddr_free
 *
 * Description:
 *   De-allocate a device address.
 *
 * Assumptions:
 *   Caller hold the exclsem
 *
 ****************************************************************************/

static void usbhost_devaddr_free(FAR struct usbhost_devaddr_s *devgen,
                                 uint8_t devaddr)
{
  int index;
  int bitno;

  /* Free the address by clearing the associated bit in the alloctab[]; */

  index = devaddr >> 5;
  bitno = devaddr & 0x1f;

  DEBUGASSERT((devgen->alloctab[index] |= (1 << bitno)) != 0);
  devgen->alloctab[index] &= ~(1 << bitno);

  /* Reset the next pointer if the one just released has a lower value */

  if (devaddr < devgen->next)
    {
      devgen->next = devaddr;
    }
}

/****************************************************************************
 * Name: usbhost_roothubport
 *
 * Description:
 *   Find and return a reference the root hub port.
 *
 ****************************************************************************/

static inline FAR struct usbhost_roothubport_s *
usbhost_roothubport(FAR struct usbhost_hubport_s *hport)
{
#ifdef CONFIG_USBHOST_HUB
  /* If this is a root hub port then the parent port pointer will be NULL.
   * Otherwise, we need to traverse the parent pointer list until we find the
   * root hub port.
   */

  while (hport->parent != NULL)
    {
      /* This is not a root hub port.  It is a port on a hub.  Try the port
       * of the parent hub that supports this port.
       */

      hport = hport->parent;
    }
#endif

  return (FAR struct usbhost_roothubport_s *)hport;
}

/****************************************************************************
 * Name: usbhost_devaddr_gen
 *
 * Description:
 *   Find root hub port and return a reference to the device function address
 *   data set.
 *
 ****************************************************************************/

static FAR struct usbhost_devaddr_s *
usbhost_devaddr_gen(FAR struct usbhost_hubport_s *hport)
{
  FAR struct usbhost_roothubport_s *rhport;

  rhport = usbhost_roothubport(hport);
  if (rhport != NULL)
    {
      return &rhport->devgen;
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_devaddr_initialize
 *
 * Description:
 *   Initialize the caller provided struct usbhost_devaddr_s instance in
 *   preparation for the management of device addresses on behalf of an root
 *   hub port.
 *
 * Input Parameters:
 *   rhport - A reference to a roothubport structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbhost_devaddr_initialize(FAR struct usbhost_roothubport_s *rhport)
{
  FAR struct usbhost_devaddr_s *devgen;

  DEBUGASSERT(rhport);
  devgen = &rhport->devgen;

  memset(devgen, 0, sizeof(struct usbhost_devaddr_s));
  nxsem_init(&devgen->exclsem, 0, 1);
  devgen->next = 1;
}

/****************************************************************************
 * Name: usbhost_devaddr_create
 *
 * Description:
 *   Create a new unique device address for this hub port.
 *
 * Input Parameters:
 *   hport - A reference to a hub port structure to which a device has been
 *     newly connected and so is in need of a function address.
 *
 * Returned Value:
 *   On success, a new device function address in the range 0x01 to 0x7f
 *   is returned.  On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int usbhost_devaddr_create(FAR struct usbhost_hubport_s *hport)
{
  FAR struct usbhost_devaddr_s *devgen;
  int devaddr;
  int ret;

  /* Get the address generation data from the root hub port */

  DEBUGASSERT(hport);
  devgen = usbhost_devaddr_gen(hport);
  DEBUGASSERT(devgen);

  /* Get exclusive access to the root hub port device address data */

  ret = usbhost_takesem(devgen);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a device address */

  devaddr = usbhost_devaddr_allocate(devgen);
  usbhost_givesem(devgen);

  if (devaddr < 0)
    {
      uerr("ERROR: Failed to allocate a device address\n");
    }

  return devaddr;
}

/****************************************************************************
 * Name: usbhost_devaddr_destroy
 *
 * Description:
 *   Release a device address previously assigned by
 *   usbhost_devaddr_create().
 *
 * Input Parameters:
 *   hport - A reference to a hub port structure from which a device has been
 *     disconnected and so no longer needs the function address.
 *   devaddr - The address to be released.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbhost_devaddr_destroy(FAR struct usbhost_hubport_s *hport,
                             uint8_t devaddr)
{
  FAR struct usbhost_devaddr_s *devgen;
  int ret;

  /* Ignore bad device address */

  if (devaddr > 0 && devaddr < 0x7f)
    {
      /* Get the address generation data from the root hub port */

      DEBUGASSERT(hport);
      devgen = usbhost_devaddr_gen(hport);
      DEBUGASSERT(devgen);

      /* Get exclusive access to the root hub port device address data */

      do
        {
          ret = usbhost_takesem(devgen);

          /* The only expected error would -ECANCELED meaning that the parent
           * thread has been canceled.  We have to continue and free the
           * device address in this case.
           */

          DEBUGASSERT(ret == OK || ret == -ECANCELED);
        }
      while (ret < 0);

      /* Free the device address */

      usbhost_devaddr_free(devgen, devaddr);
      usbhost_givesem(devgen);
    }
}
