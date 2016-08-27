/****************************************************************************
 * drivers/usbhost/usbhost_composite.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbhost.h>

#include "usbhost_composite.h"

#ifdef CONFIG_USBHOST_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one component class of the composite */

struct usbhost_component_s
{
  /* This the the classobject returned by each contained class */

  FAR struct usbhost_class_s **usbclass

  /* This is the information that we need to do the registry lookup for this
   * class member.
   */

  struct usbhost_id_s id,
};

/* This structure contains the internal, private state of the USB host
 * CDC/ACM class.
 */

struct usbhsot_composite_s
{
  /* This is the externally visible portion of the state.  The usbclass must
   * the first element of the structure.  It is then cast compatible with
   * struct usbhsot_composite_s.
   */

  struct usbhost_class_s usbclass;

  /* Class specific data follows */

  uint16_t nclasses;   /* Number of component classes in the composite */

  /* The following points to an allocated array of type struct
   * usbhost_component_s.  Element element of the array corresponds to one
   * component class in the composite.
   */

  FAR struct usbhost_component_s *members;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* struct usbhost_class_s methods */

static int  usbhost_connect(FAR struct usbhost_class_s *usbclass,
              FAR const uint8_t *configdesc, int desclen);
static int  usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_disconnect_all
 *
 * Description:
 *   Disconnect all contained class instances.
 *
 * Input Parameters:
 *   priv - Reference to private, composite container state stucture.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_disconnect_all(FAR struct usbhsot_composite_s *priv)
{
  FAR struct usbhost_component_s *member;

  /* Loop, processing each class that has been included into the composite */

  for (i = 0; i < nclasses; i++)
    {
      member = &priv->members[i];

      /* Has this member been included to the composite? */

      if (member->usbclass != NULL)
        {
          /* Yes.. disconnect it, freeing all of the class resources */

          CLASS_DISCONNECTED(member->usbclass);
          member->usbclass = NULL;
        }
    }
}

/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhsot_composite_s *priv = (FAR struct usbhsot_composite_s *)usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Forward the connection information to each contained class in the
   * composite.
   * REVIST:  Is that right?  Or should it be forwarded only to the class
   * matching the configdesc?  I am not sure that is going on here.
   */
#warning Missing logic

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *usbclass)
{
  FAR struct usbhsot_composite_s *priv = (FAR struct usbhsot_composite_s *)usbclass;

  DEBUGASSERT(priv != NULL);

  /* Forward the disconnect event to each contained class in the composite. */

  usbhost_disconnect_all(priv);

  /* Free the allocate array of composite members */

  if (priv->members != NULL)
    {
      kmm_free(priv->members);
    }

  /* The destroy the composite container itself */

  kmm_free(priv);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_composite
 *
 * Description:
 *   As the final steps in the device enumeration sequence this function
 *   will be called in order to determine (1) determine if the device is
 *   a composite device, and if so, (2) create the composite class which
 *   contains all of the individual class instances making up the composite.
 *
 * Input Parameters:
 *   hport      - The downstream port to which the (potential) composite
 *                device has been connected.
 *   configdesc - The full configuration descriptor
 *   desclen    - The length of the configuration descriptor
 *   id         - Lookup information extracted from the device descriptor.
 *                for the case of the composite devices, we need only the
 *                vid and pid.
 *   usbclass   - If the class driver for the device is successful located
 *                and bound to the hub port, the allocated class instance
 *                is returned into this caller-provided memory location.
 *
 * Returned Value:
 *   Zero (OK) is returned if (1) the device was determined to be a
 *   composite device and (2) the composite class wrapper was sucessfully
 *   created and bound to the HCD.  A negated errno value is returned on
 *   any failure.  The value -ENOENT, in particular means that the attached
 *   device is not a composite device.  Other values would indicate other
 *   various, unexpected failures.
 *
 ****************************************************************************/

int usbhost_composite(FAR struct usbhost_hubport_s *hport,
                      FAR const uint8_t *configdesc, int desclen,
                      FAR struct usbhost_id_s *id,
                      FAR struct usbhost_class_s **usbclass)
{
  FAR struct usbhsot_composite_s *priv;
  FAR struct usbhost_component_s *member;
  FAR const struct usbhost_registry_s *reg;
  FAR struct usb_desc_s *desc;
  uint32_t mergeset;
  uint16_t nintfs;
  uint16_t nmerged;
  uint16_t nclasses;
  int offset;
  int i;

  /* Determine if this a composite device has been connected to the
   * downstream port.
   *
   * First look at there device descriptor information.  A composite
   * device is only possible if:
   *
   * 1. Manufacturers of composite devices typically assign a value of zero
   *    to the device class (bDeviceClass), subclass (bDeviceSubClass), and
   *    protocol (bDeviceProtocol) fields in the device descriptor, as
   *    specified by the Universal Serial Bus Specification. This allows
   *    the manufacturer to associate each individual interface with a
   *    different device class and protocol.
   *
   * 2. The USB-IF core team has devised a special class and protocol code
   *    set that notifies the operating system that one or more IADs are
   *    present in device firmware. A device's device descriptor must have
   *    the values that appear in the following table:
   *
   *   bDeviceClass    0xEF
   *   bDeviceSubClass 0x02
   *   bDeviceProtocol 0x01
   */

   if (id->base != USB_CLASS_PER_INTERFACE && id->base != USB_CLASS_MISC)
     {
       return -ENOENT;
     }

  /* First, count the number of interface descriptors (nintfs) and the
   * number of interfaces that are assocated to one device via IAD
   * descriptor (nmerged).
   */

  mergeset = 0
  nintfs   = 0;
  nmerged  = 0;

  for (offset = 0; offset < desclen - sizeof(struct usb_desc_s); )
    {
      desc = (FAR struct usb_desc_s *)&configdesc[offset];
      int len = desc->len;

      if (offset + len < desclen)
        {
          /* Is this an interface descriptor? */

          if (desc->type == USB_DESC_TYPE_INTERFACE)
            {
              FAR struct usb_ifdesc_s *ifdesc =
                (FAR struct usb_ifdesc_s *)desc;

              DEBUGASSERT(ifdesc->iif < 32);
              nintfs++;
            }

          /* Check for IAD descriptors that will be used when it is
           * necessary to associate multiple interfaces with a single
           * device.
           */

         else if (desc->type == USB_DESC_TYPE_INTERFACEASSOCIATION)
           {
              FAR struct usb_iaddesc_s *iad = (FAR struct usb_iaddesc_s *)desc;
              uint32_t mask;

              /* Keep count of the number of merged interfaces */

              nmerged += (iad->nifs - 1);

              /* Keep track of which interfaces have been merged */

              DEBUGASSERT(iad->firstif + iad->nifs < 32);
              mask     = (1 << iad->nifs) - 1;
              mergset |= mask << iad->firstif;
           }
        }

      offset += len;
    }

  if (nintfs < 2)
    {
      /* Only one interface.  Can't be a composite device */

      return -ENOENT;
    }

#if 0 /* I think not needed, the device descriptor classid check should handle this */
  /* Special case:  Some NON-composite device have more than on interface:  CDC/ACM
   * and MSC both may have two interfaces.
   */

  if (nintfs < 3 && nmerged == 0)
    {
      /* Do the special case checks */
#warning Missing logic
    }
#endif

  /* The total number of classes is then the number of interfaces minus the
   * number of interfaces merged via the IAD descriptor.
   */

  if (nintfs <= nmerged )
    {
      /* Should not happen.  Means a bug. */

      return -EINVAL;
    }

  nclasses = nintfs - nmerged;

  /* Allocate the composite class container */

  priv = (FAR struct usbhsot_composite_s *)
    kmm_zalloc(sizeof(struct usbhsot_composite_s));

  if (priv == NULL)
    {
      uerr("ERROR: Failed to allocate class container\n")
      return -ENOMEM;
    }

  priv->members = (FAR struct usbhost_component_s *)
    kmm_zalloc(nclasses * sizeof(struct usbhost_component_s));

  if (priv->members == NULL)
    {
      uerr("ERROR: Failed to allocate class members\n")
      ret = -ENOMEM;
      goto errout_with_container;
    }

  /* Initialize the non-zero elements of the class container */

  priv->usbclass.hport        = hport;
  priv->usbclass.connect      = usbhost_connect;
  priv->usbclass.disconnected = usbhost_disconnected;
  priv->nclasses              = nclasses;

  /* Re-parse the configuration descriptor and save the CLASS ID information
   * in the member structure:  If the interface is defined by an interface
   * descriptor, then we have to use the info in the interface descriptor;
   * If the interface has a IAD, we have to use info in the IAD.
   */

  for (i = 0, offset = 0; offset < desclen - sizeof(struct usb_desc_s); )
    {
      desc = (FAR struct usb_desc_s *)&configdesc[offset];
      int len = desc->len;

      if (offset + len < desclen)
        {
          /* Is this an interface descriptor? */

          if (desc->type == USB_DESC_TYPE_INTERFACE)
            {
              FAR struct usb_ifdesc_s *ifdesc =
                (FAR struct usb_ifdesc_s *)desc;

              /* Was the interface merged via an IAD descriptor? */

              DEBUGASSERT(ifdesc->iif < 32);
              if ((mergset & (1 << ifdesc->iif)) == 0)
                {
                  FAR struct usbhost_id_s *member =
                    (FAR struct usbhost_id_s *)&priv->members[i];

                  /* No, this interface was not merged.  Save the registry
                   * lookup information from the interface descriptor.
                   */

                   member->base     = ifdesc->classid;
                   member->subclass = ifdesc->subclass;
                   member->proto    = ifdesc->protocol;
                   member->vid      = id->vid;
                   member->pid      = id->pid;

                   /* Increment the member index */

                   i++;
                }
            }

          /* Check for IAD descriptors that will be used when it is
           * necessary to associate multiple interfaces with a single
           * device.
           */

          else if (desc->type == USB_DESC_TYPE_INTERFACEASSOCIATION)
            {
              FAR struct usb_iaddesc_s *iad = (FAR struct usb_iaddesc_s *)desc;

                  /* Yes.. Save the registry lookup information from the IAD. */

                   member->base     = iad->classid;
                   member->subclass = iad->subclass;
                   member->proto    = iad->protocol;
                   member->vid      = id->vid;
                   member->pid      = id->pid;

                   /* Increment the member index */

                   i++;
            }
        }

      offset += len;
    }

  /* If everything worked, the final index must be the same as the pre-
   * calculated number of member classes.
   */

  DEBUGASSERT(i == nclasses);

  /* Now loop, performing the registry lookup on each class in the
   * composite.
   */

  for (i = 0; i < nclasses; i++)
    {
       member = &priv->members[i];

      /* Is there is a class implementation registered to support this
       * device.
       * REVISIT: This should have been saved in member structure when the
       * number of member classes was counted.
       */

      reg = usbhost_findclass(&priv->id);
      if (reg == NULL)
        {
          uinfo("usbhost_findclass failed\n");
          ret = -EINVAL;
          goto errour_with_members;
        }

      /* Yes.. there is a class for this device.  Get an instance of its
       * interface.
       */

      member->usbclass = CLASS_CREATE(reg, hport, id);
      if (member->usbclass == NULL)
        {
          uinfo("CLASS_CREATE failed\n");
          ret = -ENOMEM;
          goto errour_with_members;
        }
    }

  /* All classes have been found, instantiated and bound to the composite class
   * container.  Now bind the composite class continer to the HCD.
   *
   * REVISIT: I dont' think this is right.
   */

  ret = CLASS_CONNECT(&priv->usbclass, configdesc, desclen);
  if (ret < 0)
    {
      /* On failure, call the class disconnect method of each contained
       * class which should then free the allocated usbclass instance.
       */

      uerr("ERROR: CLASS_CONNECT failed: %d\n", ret);
      goto errout_with_members;
    }

  /* Return our USB class structure */

  *usbclass = &priv->usbclass;
  return OK;

errout_with_members:
  /* On an failure, call the class disconnect method of each contained
   * class which should then free the allocated usbclass instance.
   */

  usbhost_disconnect_all(priv);

  /* Free the allocate array of composite members */

  if (priv->members != NULL)
    {
      kmm_free(priv->members);
    }

errout_with_container:
  /* Then free the composite container itself */

  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_USBHOST_COMPOSITE */
