/*******************************************************************************
 * drivers/usbhost/usbhost_devaddr.c
 * Manage USB device addresses
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/usb/usbhost_devaddr.h>

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem and usbhost_givesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void usbhost_takesem(FAR struct usbhost_devaddr_s *hcd)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&hcd->exclsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define usbhost_givesem(hcd) sem_post(&hcd->exclsem)

/*******************************************************************************
 * Name: usbhost_devaddr_hash
 *
 * Description:
 *   Create a hash value from a device address.
 *
 *******************************************************************************/

static inline uint8_t usbhost_devaddr_hash(uint8_t devaddr)
{
  uint8_t ret = devaddr;

  ret ^= (devaddr >> 2);
  ret ^= (devaddr >> 3);
  return ret & USBHOST_DEVADDR_HASHMASK;
}

/*******************************************************************************
 * Name: usbhost_devaddr_allocate
 *
 * Description:
 *   Allocate a new unique device address for this HCD.
 *
 * Assumptions:
 *   Caller hold the exclsem
 *
 *******************************************************************************/

static int usbhost_devaddr_allocate(FAR struct usbhost_devaddr_s *hcd)
{
  uint8_t startaddr = hcd->next;
  uint8_t devaddr;
  int index;
  int bitno;

  /* Loop until we find a valid device address */

  for (;;)
    {
      /* Try the next device address */

      devaddr = hcd->next;
      if (hcd->next >= 0x7f)
        {
          hcd->next = 1;
        }
      else
        {
          hcd->next++;
        }

      /* Is this address already allocated? */

      index = devaddr >> 5;
      bitno = devaddr & 0x1f;
      if ((hcd->alloctab[index] & (1 << bitno)) == 0)
        {
          /* No... allocate it now */

          hcd->alloctab[index] |= (1 << bitno);
          return (int)devaddr;
        }

      /* This address has already been allocated.  The followign logic will
       * prevent (unexpected) infinite loops.
       */

      if (startaddr == devaddr)
        {
          /* We are back where we started... the are no free device address */

          return -ENOMEM;
        }
    }
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: usbhost_devaddr_initialize
 *
 * Description:
 *   Initialize the caller provided struct usbhost_devaddr_s instance in
 *   preparation for the management of device addresses on behalf of an HCD.
 *
 *******************************************************************************/

void usbhost_devaddr_initialize(FAR struct usbhost_devaddr_s *hcd)
{
  DEBUGASSERT(hcd);

  memset(hcd, 0, sizeof(struct usbhost_devaddr_s));
  sem_init(&hcd->exclsem, 0, 1);
  hcd->next = 1;
}

/*******************************************************************************
 * Name: usbhost_devaddr_create
 *
 * Description:
 *   Create a new unique device address for this HCD.  Bind the void* arg to the
 *   the device address and return the newly allocated device address.
 *
 *******************************************************************************/

int usbhost_devaddr_create(FAR struct usbhost_devaddr_s *hcd,
                           FAR void *associate)
{
  FAR struct usbhost_devhash_s *hentry;
  uint8_t hvalue;
  int devaddr;

  /* Allocate a hash table entry */

  hentry = (FAR struct usbhost_devhash_s *)kmalloc(sizeof(struct usbhost_devhash_s));
  if (!hentry)
    {
      udbg("ERROR: Failed to allocate a hash table entry\n");
      return -ENOMEM;
    }

  /* Get exclusive access to the HCD device address data */

  usbhost_takesem(hcd);

  /* Allocate a device address */

  devaddr = usbhost_devaddr_allocate(hcd);
  if (devaddr < 0)
    {
      udbg("ERROR: Failed to allocate a device address\n");
      free(hentry);
    }
  else
    {
      /* Initialize the hash table entry */

      hentry->devaddr = devaddr;
      hentry->payload = associate;

      /* Add the new device address to the hash table */

      hvalue = usbhost_devaddr_hash(devaddr);
      hentry->flink = hcd->hashtab[hvalue];
      hcd->hashtab[hvalue] = hentry;

      /* Try to re-use the lowest numbered device addresses */

      if (hcd->next > devaddr)
        {
          hcd->next = devaddr;
        }
    }

  usbhost_givesem(hcd);
  return devaddr;
}

/*******************************************************************************
 * Name: usbhost_devaddr_find
 *
 * Description:
 *   Given a device address, find the void* value that was bound to the device
 *   address by usbhost_devaddr_create() when the device address was allocated.
 *
 *******************************************************************************/

FAR void *usbhost_devaddr_find(FAR struct usbhost_devaddr_s *hcd,
                               uint8_t devaddr)
{
  FAR struct usbhost_devhash_s *hentry;
  uint8_t hvalue;

  /* Get exclusive access to the HCD device address data */

  hvalue = usbhost_devaddr_hash(devaddr);
  usbhost_takesem(hcd);

  /* Check each entry in the hash table */

  for (hentry = hcd->hashtab[hvalue]; hentry; hentry = hentry->flink)
    {
      /* Is this the address we are looking for? */

      if (hentry->devaddr == devaddr)
        {
          /* Yes.. return the payload from the hash table entry */

          usbhost_givesem(hcd);
          return hentry->payload;
        }
    }

  /* Didn't find the device address */

  usbhost_givesem(hcd);
  return NULL;
}

/*******************************************************************************
 * Name: usbhost_devaddr_destroy
 *
 * Description:
 *   Release a device address previously allocated by usbhost_devaddr_destroy()
 *   and destroy the association with the void* data.
 *
 *******************************************************************************/

void usbhost_devaddr_destroy(FAR struct usbhost_devaddr_s *hcd, uint8_t devaddr)
{
  FAR struct usbhost_devhash_s *hentry;
  FAR struct usbhost_devhash_s *prev;
  uint8_t hvalue;

  /* Get exclusive access to the HCD device address data */

  hvalue = usbhost_devaddr_hash(devaddr);
  usbhost_takesem(hcd);

  /* Search the hast table for the matching entry */

  for (hentry = hcd->hashtab[hvalue], prev = NULL;
       hentry;
       prev = hentry, hentry = hentry->flink)
    {
      /* Is this the address we are looking for? */

      if (hentry->devaddr == devaddr)
        {
          /* Yes.. remove the entry from the hash list */

          if (prev)
            {
              prev->flink = hentry->flink;
            }
          else
            {
              hcd->hashtab[hvalue] = hentry->flink;
            }

          /* And release the entry */

          kfree(hentry);
          break;
        }
    }

  usbhost_givesem(hcd);
}
