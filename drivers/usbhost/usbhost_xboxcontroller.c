/****************************************************************************
 * drivers/usbhost/usbhost_xboxcontroller.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/input/xbox-controller.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_XBOXCONTROLLER_DEFPRIO
#  define CONFIG_XBOXCONTROLLER_DEFPRIO 50
#endif

#ifndef CONFIG_XBOXCONTROLLER_STACKSIZE
#  define CONFIG_XBOXCONTROLLER_STACKSIZE 1024
#endif

#ifndef CONFIG_XBOXCONTROLLER_NPOLLWAITERS
#  define CONFIG_XBOXCONTROLLER_NPOLLWAITERS 2
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/xbox[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/xbox%c"
#define DEV_NAMELEN         11

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_EPOUTFOUND  0x04 /* Required interrupt OUT EP descriptor found */
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_CREFS   0x7fff

/* Received message types */

#define USBHOST_WAITING_CONNECTION  0x02
#define USBHOST_GUIDE_BUTTON_STATUS 0x07
#define USBHOST_BUTTON_DATA         0x20

/* Button definitions */

#define XBOX_BUTTON_GUIDE_INDEX        4
#define XBOX_BUTTON_SYNC_INDEX         4
#define XBOX_BUTTON_SYNC_MASK          (1 << 0)
#define XBOX_BUTTON_START_INDEX        4
#define XBOX_BUTTON_START_MASK         (1 << 2)
#define XBOX_BUTTON_BACK_INDEX         4
#define XBOX_BUTTON_BACK_MASK          (1 << 3)
#define XBOX_BUTTON_A_INDEX            4
#define XBOX_BUTTON_A_MASK             (1 << 4)
#define XBOX_BUTTON_B_INDEX            4
#define XBOX_BUTTON_B_MASK             (1 << 5)
#define XBOX_BUTTON_X_INDEX            4
#define XBOX_BUTTON_X_MASK             (1 << 6)
#define XBOX_BUTTON_Y_INDEX            4
#define XBOX_BUTTON_Y_MASK             (1 << 7)
#define XBOX_BUTTON_DPAD_UP_INDEX      5
#define XBOX_BUTTON_DPAD_UP_MASK       (1 << 0)
#define XBOX_BUTTON_DPAD_DOWN_INDEX    5
#define XBOX_BUTTON_DPAD_DOWN_MASK     (1 << 1)
#define XBOX_BUTTON_DPAD_LEFT_INDEX    5
#define XBOX_BUTTON_DPAD_LEFT_MASK     (1 << 2)
#define XBOX_BUTTON_DPAD_RIGHT_INDEX   5
#define XBOX_BUTTON_DPAD_RIGHT_MASK    (1 << 3)
#define XBOX_BUTTON_BUMPER_LEFT_INDEX  5
#define XBOX_BUTTON_BUMPER_LEFT_MASK   (1 << 4)
#define XBOX_BUTTON_BUMPER_RIGHT_INDEX 5
#define XBOX_BUTTON_BUMPER_RIGHT_MASK  (1 << 5)
#define XBOX_BUTTON_STICK_LEFT_INDEX   5
#define XBOX_BUTTON_STICK_LEFT_MASK    (1 << 6)
#define XBOX_BUTTON_STICK_RIGHT_INDEX  5
#define XBOX_BUTTON_STICK_RIGHT_MASK   (1 << 7)
#define XBOX_BUTTON_TRIGGER_LEFT       3
#define XBOX_BUTTON_TRIGGER_RIGHT      4
#define XBOX_BUTTON_STICK_LEFT_X       5
#define XBOX_BUTTON_STICK_LEFT_Y       6
#define XBOX_BUTTON_STICK_RIGHT_X      7
#define XBOX_BUTTON_STICK_RIGHT_Y      8
#define XBOX_BUTTON_SET(buffer, index, mask) \
  ((((buffer)[(index)] & (mask)) != 0) ? true : false);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;

  /* The remainder of the fields are provide to the class driver */

  char                                 devchar;      /* Character identifying the /dev/xbox[n] device */
  volatile bool                        disconnected; /* TRUE: Device has been disconnected */
  volatile bool                        polling;      /* TRUE: Poll thread is running */
  volatile bool                        open;         /* TRUE: The controller device is open */
  volatile bool                        valid;        /* TRUE: New sample data is available */
  volatile bool                        initialized;  /* TRUE: The initialization packet has been sent */
  uint8_t                              ifno;         /* Interface number */
  uint8_t                              nwaiters;     /* Number of threads waiting for controller data */
  sem_t                                waitsem;      /* Used to wait for controller data */
  int16_t                              crefs;        /* Reference count on the driver instance */
  sem_t                                exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s                        work;         /* For interacting with the worker thread */
  FAR uint8_t                         *tbuffer;      /* The allocated transfer buffer */
  FAR uint8_t                          obuffer[20];  /* The fixed output transfer buffer */
  size_t                               tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t                         epin;         /* IN endpoint */
  usbhost_ep_t                         epout;        /* OUT endpoint */
  pid_t                                pollpid;      /* PID of the poll task */
  size_t                               out_seq_num;  /* The sequence number for outgoing packets */
  struct xbox_controller_buttonstate_s rpt;          /* The latest report out of the controller. */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_XBOXCONTROLLER_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static int usbhost_takesem(FAR sem_t *sem);
static void usbhost_forcetake(FAR sem_t *sem);
#define usbhost_givesem(s) nxsem_post(s);

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Polling support */

static void usbhost_pollnotify(FAR struct usbhost_state_s *dev);
static int usbhost_xboxcontroller_poll(int argc, char *argv[]);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc,
                                  int desclen);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val);
#endif

/* Transfer descriptor memory management */

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *
  usbhost_create(FAR struct usbhost_hubport_s *hport,
                 FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Driver methods.  We export the controller as a standard character driver */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
static int usbhost_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB class driver to a connected USB device.
 */

static const struct usbhost_id_s g_xboxcontroller_id[] =
{
  /* XBox One classic controller */

  {
    USB_CLASS_VENDOR_SPEC,  /* base -- Must be one of the USB_CLASS_* definitions in usb.h */
    0x0047,                 /* subclass -- depends on the device */
    0x00d0,                 /* proto -- depends on the device */
    0x045e,                 /* vid */
    0x02dd                  /* pid */
  },

  /* XBox One S controller */

  {
    USB_CLASS_VENDOR_SPEC,  /* base -- Must be one of the USB_CLASS_* definitions in usb.h */
    0x0047,                 /* subclass -- depends on the device */
    0x00d0,                 /* proto -- depends on the device */
    0x045e,                 /* vid */
    0x02ea                  /* pid */
  }
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_xboxcontroller =
{
  NULL,                     /* flink */
  usbhost_create,           /* create */
  2,                        /* nids */
  g_xboxcontroller_id       /* id[] */
};

/* The configuration information for the block file device. */

static const struct file_operations g_xboxcontroller_fops =
{
  usbhost_open,             /* open */
  usbhost_close,            /* close */
  usbhost_read,             /* read */
  usbhost_write,            /* write */
  NULL,                     /* seek */
  usbhost_ioctl,            /* ioctl */
  usbhost_poll              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                    /* unlink */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/xboxa-z. */

static uint32_t g_devinuse;

/* The following are used to managed the class creation operation */

static sem_t                   g_exclsem; /* For mutually exclusive thread creation */
static sem_t                   g_syncsem; /* Thread data passing interlock */
static struct usbhost_state_s *g_priv;    /* Data passed to thread */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static int usbhost_takesem(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: usbhost_forcetake
 *
 * Description:
 *   This is just another wrapper but this one continues even if the thread
 *   is canceled.  This must be done in certain conditions where were must
 *   continue in order to clean-up resources.
 *
 ****************************************************************************/

static void usbhost_forcetake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_wait_uninterruptible(sem);

      /* The only expected error would -ECANCELED meaning that the
       * parent thread has been canceled.  We have to continue and
       * terminate the poll in this case.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());

  priv = (FAR struct usbhost_state_s *)
    kmm_malloc(sizeof(struct usbhost_state_s));

  uinfo("Allocated: %p\n", priv);
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   usbclass - A reference to the class instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance (perhaps calling sched_kmm_free() in case we are
   * executing from an interrupt handler.
   */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->devchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = enter_critical_section();
      g_devinuse &= ~(1 << devno);
      leave_critical_section(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  uinfo("crefs: %d\n", priv->crefs);

  hport = priv->usbclass.hport;

  DEBUGASSERT(hport->drvr);

  uinfo("crefs: %d\n", priv->crefs);

  /* Unregister the driver */

  uinfo("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(hport->drvr, priv->epin);
    }

  /* Free any transfer buffers */

  usbhost_tfree(priv);

  /* Destroy the semaphores */

  nxsem_destroy(&priv->exclsem);
  nxsem_destroy(&priv->waitsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(hport->drvr, hport);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* And free the class instance. */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_pollnotify
 *
 * Description:
 *   Wake any threads waiting for controller data
 *
 * Input Parameters:
 *   priv - A reference to the controller state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_pollnotify(FAR struct usbhost_state_s *priv)
{
  int i;

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      nxsem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for controller data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_XBOXCONTROLLER_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          iinfo("Report events: %08" PRIx32 "\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }
}

/****************************************************************************
 * Name: usbhost_xboxcontroller_poll
 *
 * Description:
 *   Periodically check for new controller data.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int usbhost_xboxcontroller_poll(int argc, char *argv[])
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhost_hubport_s *hport;
  irqstate_t flags;
#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
  unsigned int npolls = 0;
#endif
  unsigned int nerrors = 0;
  ssize_t nbytes;
  int ret = OK;

  /* Synchronize with the start-up logic.  Get the private instance, re-start
   * the start-up logic, and wait a bit to make sure that all of the class
   * creation logic has a chance to run to completion.
   *
   * NOTE: that the reference count is *not* incremented here.  When the
   * driver structure was created, it was created with a reference count of
   * one.  This thread is responsible for that count.  The count will be
   * decrement when this thread exits.
   */

  priv = g_priv;
  DEBUGASSERT(priv != NULL  && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  priv->polling = true;
  usbhost_givesem(&g_syncsem);
  nxsig_sleep(1);

  /* Loop here until the device is disconnected */

  uinfo("Entering poll loop\n");

  while (!priv->disconnected)
    {
      /* Read the next ccontroller report.  We will stall here until the
       * controller sends data.
       */

      nbytes = DRVR_TRANSFER(hport->drvr, priv->epin,
                             priv->tbuffer, priv->tbuflen);

      /* Check for errors -- Bail if an excessive number of consecutive
       * errors are encountered.
       */

      if (nbytes < 0)
        {
          /* If DRVR_TRANSFER() returns EAGAIN, that simply means that
           * the devices was not ready and has NAK'ed the transfer.  That
           * should not be treated as an error (unless it persists for a
           * long time).
           */

          if (nbytes != -EAGAIN)
            {
              uerr("ERROR: DRVR_TRANSFER returned: %d/%u\n",
              (int)nbytes, nerrors);

              if (++nerrors > 200)
                {
                  uerr("  Too many errors... aborting: %d\n", nerrors);
                  ret = (int)nbytes;
                  break;
                }
            }
        }

      /* The report was received correctly. */

      else
        {
          /* Success, reset the error counter */

          nerrors = 0;

          /* The type of message is in the first byte */

          switch (priv->tbuffer[0])
            {
            case USBHOST_WAITING_CONNECTION:
              /* Send the initialization message when we received the
               * the first waiting connection message.
               */

              if (!priv->initialized)
                {
                  /* Get exclusive access to the controller state data */

                  ret = usbhost_takesem(&priv->exclsem);
                  if (ret < 0)
                    {
                      goto exitloop;
                    }

                  priv->tbuffer[0] = 0x05;
                  priv->tbuffer[1] = 0x20;
                  priv->tbuffer[2] = priv->out_seq_num++;
                  priv->tbuffer[3] = 0x01;
                  priv->tbuffer[4] = 0x00;
                  nbytes = DRVR_TRANSFER(hport->drvr, priv->epout,
                                         priv->tbuffer, 5);
                  priv->initialized = true;

                  /* Release our lock on the state structure */

                  usbhost_givesem(&priv->exclsem);
                }

              break;

            case USBHOST_GUIDE_BUTTON_STATUS:

              /* Get exclusive access to the controller state data */

              ret = usbhost_takesem(&priv->exclsem);
              if (ret < 0)
                {
                  goto exitloop;
                }

              /* Read the data out of the controller report. */

              priv->rpt.guide =
                (priv->tbuffer[XBOX_BUTTON_GUIDE_INDEX] != 0) ? true : false;
              priv->valid = true;

              /* The One X controller requires an ACK of the guide button
               * status message.
               */

              if (priv->tbuffer[1] == 0x30)
                {
                  static const uint8_t guide_button_report_ack[] =
                  {
                    0x01, 0x20, 0x00, 0x09, 0x00, 0x07, 0x20, 0x02,
                    0x00, 0x00, 0x00, 0x00, 0x00
                  };

                  /* Remember the input packet sequence number. */

                  uint8_t seq_num = priv->tbuffer[2];

                  /* Copy the ACK packet into the transfer buffer. */

                  memcpy(priv->tbuffer, guide_button_report_ack,
                         sizeof(guide_button_report_ack));

                  /* Ensure the sequence number is the same as the input
                   * packet.
                   */

                  priv->tbuffer[2] = seq_num;

                  /* Perform the transfer. */

                  nbytes =
                    DRVR_TRANSFER(hport->drvr, priv->epout, priv->tbuffer,
                                  sizeof(guide_button_report_ack));
                }

              /* Notify any waiters that new controller data is available */

              usbhost_pollnotify(priv);

              /* Release our lock on the state structure */

              usbhost_givesem(&priv->exclsem);

              break;

            case USBHOST_BUTTON_DATA:

              /* Ignore the controller data if no task has opened the
               * driver.
               */

              if (priv->open)
                {
                  /* Get exclusive access to the controller state data */

                  ret = usbhost_takesem(&priv->exclsem);
                  if (ret < 0)
                    {
                      goto exitloop;
                    }

                  /* Read the data out of the controller report. */

                  priv->rpt.sync =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_SYNC_INDEX,
                                    XBOX_BUTTON_SYNC_MASK);
                  priv->rpt.start =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_START_INDEX,
                                    XBOX_BUTTON_START_MASK);
                  priv->rpt.back =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_BACK_INDEX,
                                    XBOX_BUTTON_BACK_MASK);
                  priv->rpt.a =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_A_INDEX,
                                    XBOX_BUTTON_A_MASK);
                  priv->rpt.b =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_B_INDEX,
                                    XBOX_BUTTON_B_MASK);
                  priv->rpt.x =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_X_INDEX,
                                    XBOX_BUTTON_X_MASK);
                  priv->rpt.y =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_Y_INDEX,
                                    XBOX_BUTTON_Y_MASK);
                  priv->rpt.dpad_up =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_DPAD_UP_INDEX,
                                    XBOX_BUTTON_DPAD_UP_MASK);
                  priv->rpt.dpad_down =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_DPAD_DOWN_INDEX,
                                    XBOX_BUTTON_DPAD_DOWN_MASK);
                  priv->rpt.dpad_left =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_DPAD_LEFT_INDEX,
                                    XBOX_BUTTON_DPAD_LEFT_MASK);
                  priv->rpt.dpad_right =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_DPAD_RIGHT_INDEX,
                                    XBOX_BUTTON_DPAD_RIGHT_MASK);
                  priv->rpt.bumper_left =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_BUMPER_LEFT_INDEX,
                                    XBOX_BUTTON_BUMPER_LEFT_MASK);
                  priv->rpt.bumper_right =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_BUMPER_RIGHT_INDEX,
                                    XBOX_BUTTON_BUMPER_RIGHT_MASK);
                  priv->rpt.stick_click_left =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_STICK_LEFT_INDEX,
                                    XBOX_BUTTON_STICK_LEFT_MASK);
                  priv->rpt.stick_click_right =
                    XBOX_BUTTON_SET(priv->tbuffer,
                                    XBOX_BUTTON_STICK_RIGHT_INDEX,
                                    XBOX_BUTTON_STICK_RIGHT_MASK);

                  priv->rpt.trigger_left =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_TRIGGER_LEFT];
                  priv->rpt.trigger_right =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_TRIGGER_RIGHT];
                  priv->rpt.stick_left_x =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_STICK_LEFT_X];
                  priv->rpt.stick_left_y =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_STICK_LEFT_Y];
                  priv->rpt.stick_right_x =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_STICK_RIGHT_X];
                  priv->rpt.stick_right_y =
                    ((int16_t *)(priv->tbuffer))[XBOX_BUTTON_STICK_RIGHT_Y];

                  priv->valid = true;

                  /* Notify any waiters that new controller data is
                   * available.
                   */

                  usbhost_pollnotify(priv);

                  /* Release our lock on the state structure */

                  usbhost_givesem(&priv->exclsem);
                }

              break;

            default:
              uinfo("Received message type: %x\n", priv->tbuffer[0]);
            }
        }

      /* If USB debug is on, then provide some periodic indication that
       * polling is still happening.
       */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
      npolls++;
      if ((npolls & 31) == 0)
        {
          uinfo("Still polling: %d\n", npolls);
        }
#endif
    }

exitloop:
  /* We get here when the driver is removed, when too many errors have
   * been encountered, or when the thread is canceled.
   *
   * Make sure that we have exclusive access to the private data structure.
   * There may now be other tasks with the character driver open and actively
   * trying to interact with the class driver.
   */

  usbhost_forcetake(&priv->exclsem);

  /* Indicate that we are no longer running and decrement the reference
   * count held by this thread.  If there are no other users of the class,
   * we can destroy it now.  Otherwise, we have to wait until the all
   * of the file descriptors are closed.
   */

  uinfo("Controller removed, polling halted\n");

  flags = enter_critical_section();
  priv->polling = false;

  /* Decrement the reference count held by this thread. */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* There are two possibilities:
   * 1) The reference count is greater than zero.  This means that there
   *    are still open references to the controller driver.  In this case
   *    we need to wait until usbhost_close() is called and all of the
   *    open driver references are decremented.  Then usbhost_destroy() can
   *    be called from usbhost_close().
   * 2) The reference count is now zero.  This means that there are no
   *    further open references and we can call usbhost_destroy() now.
   */

  if (priv->crefs < 1)
    {
      /* Unregister the driver and destroy the instance (while we hold
       * the semaphore!)
       */

      usbhost_destroy(priv);
    }
  else
    {
      /* No, we will destroy the driver instance when it is final open
       * reference is closed
       */

      usbhost_givesem(&priv->exclsem);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: usbhost_sample
 *
 * Description:
 *   Check if new controller data is available
 *
 * Input Parameters:
 *    priv   - controller state instance
 *    sample - The location to return the sample data
 *
 ****************************************************************************/

static int usbhost_sample(FAR struct usbhost_state_s *priv,
                          FAR struct xbox_controller_buttonstate_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Is there new mouse data available? */

  if (priv->valid)
    {
      /* Return a copy of the sampled data. */

      memcpy(sample, &priv->rpt,
             sizeof(struct xbox_controller_buttonstate_s));

      /* The sample has been reported and is no longer valid */

      priv->valid = false;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: usbhost_waitsample
 *
 * Description:
 *   Wait for the next valid controller sample
 *
 * Input Parameters:
 *    priv   - controller state instance
 *    sample - The location to return the sample data
 *
 ****************************************************************************/

static int usbhost_waitsample(FAR struct usbhost_state_s *priv,
                  FAR struct xbox_controller_buttonstate_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = enter_critical_section();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxsem_post(&priv->exclsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (usbhost_sample(priv, sample) < 0)
    {
      /* Wait for a change in the HIDMOUSE state */

      iinfo("Waiting..\n");
      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait: %d\n", ret);
          goto errout;
        }

      /* Did the controller become disconnected while we were waiting */

      if (priv->disconnected)
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  iinfo("Sampled\n");

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our
   * sample.  Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxsem_wait(&priv->exclsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the HIDMOUSE for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s epindesc;
  FAR struct usbhost_epdesc_s epoutdesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&epindesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&epoutdesc, 0, sizeof(struct usbhost_epdesc_s));

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
   */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s) && !done)
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed from a preceding
             * interface?
             */

            if ((found & USBHOST_ALLFOUND) == USBHOST_ALLFOUND)
              {
                /* Yes.. then break out of the loop and use the preceding
                 * interface.
                 */

                done = true;
              }
            else
              {
                /* Otherwise, discard any endpoints previously found */

                found = USBHOST_IFFOUND;
              }
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            uinfo("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT interrupt endpoint.  There should be only
                     * one interrupt OUT endpoint.
                     */

                    if ((found & USBHOST_EPOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    epoutdesc.hport        = hport;
                    epoutdesc.addr         = epdesc->addr &
                                             USB_EP_ADDR_NUMBER_MASK;
                    epoutdesc.in           = false;
                    epoutdesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epoutdesc.interval     = epdesc->interval;
                    epoutdesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    uerr("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
                          epoutdesc.addr, epoutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN interrupt endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_EPINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPINFOUND;

                    /* Save the bulk IN endpoint information */

                    epindesc.hport        = hport;
                    epindesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = true;
                    epindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epindesc.interval     = epdesc->interval;
                    epindesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    uerr("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          done = true;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if (found != USBHOST_ALLFOUND)
    {
      uerr("ERROR: Found IF:%s BIN:%s EPOUT:%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO",
           (found & USBHOST_EPOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(hport->drvr, &epoutdesc, &priv->epout);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Interrupt OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(hport->drvr, &epindesc, &priv->epin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate Interrupt IN endpoint\n");
      DRVR_EPFREE(hport->drvr, priv->epout);
      return ret;
    }

  uinfo("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  char devname[DEV_NAMELEN];
  int ret = OK;

  /* Set aside a transfer buffer for exclusive use by the class driver */

  ret = usbhost_talloc(priv);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Start a worker task to poll the USB device.  It would be nice to use
   * the NuttX worker thread to do this, but this task needs to wait for
   * events and activities on the worker thread should not involve
   * significant waiting.  Having a dedicated thread is more efficient in
   * this sense, but requires more memory resources, primarily for the
   * dedicated stack (CONFIG_XBOXCONTROLLER_STACKSIZE).
   */

  /* The inputs to a task started by kthread_create() are very awkward for
   * this purpose.  They are really designed for command line tasks
   * (argc/argv). So the following is kludge pass binary data when the
   * controller poll task is started.
   *
   * First, make sure we have exclusive access to g_priv (what is the
   * likelihood of this being used?  About zero, but we protect it anyway).
   */

  ret = usbhost_takesem(&g_exclsem);
  if (ret < 0)
    {
      usbhost_tfree(priv);
      goto errout;
    }

  g_priv = priv;

  uinfo("Starting thread\n");
  ret = kthread_create("xbox", CONFIG_XBOXCONTROLLER_DEFPRIO,
                       CONFIG_XBOXCONTROLLER_STACKSIZE,
                       (main_t)usbhost_xboxcontroller_poll,
                       (FAR char * const *)NULL);
  if (ret < 0)
    {
      /* Failed to started the poll thread... probably due to memory
       * resources.
       */

      usbhost_givesem(&g_exclsem);
      goto errout;
    }

  priv->pollpid = (pid_t)ret;

  /* Now wait for the poll task to get properly initialized */

  usbhost_forcetake(&g_syncsem);
  usbhost_givesem(&g_exclsem);

  /* Configure the device */

  /* Register the driver */

  uinfo("Register block driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_xboxcontroller_fops, 0666, priv);

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the block
   * driver has been registered.
   */

errout:
  usbhost_forcetake(&priv->exclsem);
  priv->crefs--;
  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 |
         (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest + 2, (uint16_t)(val >> 16));
}
#endif

/****************************************************************************
 * Name: usbhost_talloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->tbuffer == NULL);
  hport = priv->usbclass.hport;

  return DRVR_ALLOC(hport->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tfree(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  int result = OK;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);

  if (priv->tbuffer)
    {
      hport          = priv->usbclass.hport;
      result         = DRVR_FREE(hport->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }

  return result;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.  The create() method is a callback into the class
 *   implementation.  It is used to (1) create a new instance of the USB
 *   host class state and to (2) bind a USB host driver "session" to the
 *   class instance.  Use of this create() method will support environments
 *   where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub hat manages the new class instance.
 *   id    - In the case where the device supports multiple base classes,
 *           subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hport input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_create(FAR struct usbhost_hubport_s *hport,
                 FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
          /* Initialize class method function pointers */

          priv->usbclass.hport        = hport;
          priv->usbclass.connect      = usbhost_connect;
          priv->usbclass.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by the
           * driver.
           */

          priv->crefs = 1;

          /* Initialize semaphores (this works okay in the interrupt
           * context).
           */

          nxsem_init(&priv->exclsem, 0, 1);
          nxsem_init(&priv->waitsem, 0, 0);

          /* The waitsem semaphore is used for signaling and, hence, should
           * not have priority inheritance enabled.
           */

          nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

          /* Return the instance of the USB class driver */

          return &priv->usbclass;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/

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
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.
 *   It is the responsibility of the higher level enumeration logic to call
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
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret < 0)
        {
          uerr("ERROR: usbhost_devinit() failed: %d\n", ret);
        }
    }

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
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  priv->disconnected = true;
  uinfo("Disconnected\n");

  /* Are there a thread(s) waiting for controller data that will never
   * come?
   */

  for (i = 0; i < priv->nwaiters; i++)
    {
      /* Yes.. wake them up */

      usbhost_givesem(&priv->waitsem);
    }

  /* Possibilities:
   *
   * - Failure occurred before the controller poll task was started
   *   successfully.  In this case, the disconnection will have to be
   *   handled on the worker task.
   * - Failure occurred after the controller poll task was started
   *   successfully.  In this case, the disconnection can be performed on
   *   the mouse poll thread.
   */

  if (priv->polling)
    {
      /* The polling task is still alive. Signal the mouse polling task.
       * When that task wakes up, it will decrement the reference count and,
       * perhaps, destroy the class instance.  Then it will exit.
       */

      nxsig_kill(priv->pollpid, SIGALRM);
    }
  else
    {
      /* In the case where the failure occurs before the polling task was
       * started.  Now what?  We are probably executing from an interrupt
       * handler here.  We will use the worker thread.  This is kind of
       * wasteful and begs for a re-design.
       */

      DEBUGASSERT(priv->work.worker == NULL);
      work_queue(HPWORK, &priv->work, usbhost_destroy, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Character driver methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int usbhost_open(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the controller device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * disconnect events.
   */

  flags = enter_critical_section();
  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Was the driver previously open?  We need to perform special
       * initialization on the first time that the driver is opened.
       */

      if (!priv->open)
        {
          /* Set the thresholding values so that the first button press
           * will be reported.
           */

#ifdef NEVER
          priv->xlast = INVALID_POSITION_B16;
          priv->ylast = INVALID_POSITION_B16;
#ifdef CONFIG_INPUT_MOUSE_WHEEL
          priv->wlast = INVALID_POSITION_B16;
#endif
          /* Set the reported position to the center of the range */

          priv->xaccum = (HIDMOUSE_XMAX_B16 >> 1);
          priv->yaccum = (HIDMOUSE_YMAX_B16 >> 1);
#endif
        }

      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      priv->open = true;
      ret        = OK;
    }

  leave_critical_section(flags);
  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int usbhost_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs >= 1);
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* We need to disable interrupts momentarily to assure that there are no
   * asynchronous poll or disconnect events.
   */

  flags = enter_critical_section();
  priv->crefs--;

  /* Check if the USB controller device is still connected.  If the device is
   * no longer connected, then unregister the driver and free the driver
   * class instance.
   */

  if (priv->disconnected)
    {
      /* If the reference count is one or less then there are two
       * possibilities:
       *
       * 1) It might be zero meaning that the polling thread has already
       *    exited and decremented its count.
       * 2) If might be one meaning either that (a) the polling thread is
       *    still running and still holds a count, or (b) the polling thread
       *    has exited, but there is still an outstanding open reference.
       */

      if (priv->crefs == 0 || (priv->crefs == 1 && priv->polling))
        {
          /* Yes.. In either case, then the driver is no longer open */

          priv->open = false;

          /* Check if the USB keyboard device is still connected. */

          if (priv->crefs == 0)
            {
              /* The polling thread is no longer running */

              DEBUGASSERT(!priv->polling);

              /* If the device is no longer connected, unregister the driver
               * and free the driver class instance.
               */

              usbhost_destroy(priv);

              /* Skip giving the semaphore... it is no longer valid */

              leave_critical_section(flags);
              return OK;
            }
          else /* if (priv->crefs == 1) */
            {
              /* The polling thread is still running.  Signal it so that it
               * will wake up and call usbhost_destroy().  The particular
               * signal that we use does not matter in this case.
               */

              nxsig_kill(priv->pollpid, SIGALRM);
            }
        }
    }

  usbhost_givesem(&priv->exclsem);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode                          *inode;
  FAR struct usbhost_state_s                *priv;
  FAR struct xbox_controller_buttonstate_s  sample;
  int                                       ret;

  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the controller is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB controller is no longer connected.  Refuse any further
       * attempts to access the driver.
       */

      ret = -ENODEV;
      goto errout;
    }

  /* Try to read sample data. */

  ret = usbhost_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* Yes.. then return a failure */

          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = usbhost_waitsample(priv, &sample);
      ret = 0;
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: usbhost_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* We now have sampled controller data that we can report to the caller.  */

  memcpy(buffer, &sample, sizeof(struct xbox_controller_buttonstate_s));

  ret = sizeof(struct xbox_controller_buttonstate_s);

errout:
  usbhost_givesem(&priv->exclsem);
  iinfo("Returning: %d\n", ret);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  /* Not implemented. */

  return -ENOSYS;
}

/****************************************************************************
 * Name: usbhost_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int usbhost_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode            *inode;
  FAR struct usbhost_state_s  *priv;
  int                          ret = 0;
  int                          nbytes;
  FAR struct usbhost_hubport_s *hport;
  static uint8_t rumble_cmd[] =
  {
    0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0xff, 0x00, 0xff
  };

  uinfo("Entered\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;
  hport = priv->usbclass.hport;

  /* Check if the controller is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB controller is no longer connected.  Refuse any further
       * attempts to access the driver.
       */

      ret = -ENODEV;
      goto errout;
    }

  /* Determine which IOCTL command to execute. */

  switch (cmd)
    {
    case XBOX_CONTROLLER_IOCTL_RUMBLE:

      /* The least significant byte is the weak actuator strength.
       * The second byte is the strong actuator strength.
       */

      memcpy(priv->obuffer, rumble_cmd, sizeof(rumble_cmd));
      priv->obuffer[2] = priv->out_seq_num++;
      priv->obuffer[8] = (arg >> 1) & 0xff; /* Strong (left actuator) */
      priv->obuffer[9] = arg & 0xff;        /* Weak (right actuator)  */

      /* Perform the transfer. */

      nbytes = DRVR_TRANSFER(hport->drvr, priv->epout,
                 priv->obuffer, sizeof(rumble_cmd));

      /* Did we encounter an error? */

      if (nbytes < 0)
        {
          ret = nbytes;
        }

      break;

    default:

      ret = -ENOTTY;
      goto errout;
    }

errout:
  iinfo("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: usbhost_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  int                         ret;
  int                         i;

  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the controller is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB controller is no longer connected.  Refuse any further
       * attempts to access the driver.
       */

      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_XBOXCONTROLLER_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_XBOXCONTROLLER_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered controller data.
       */

      if (priv->valid)
        {
          usbhost_pollnotify(priv);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_xboxcontroller_init
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_xboxcontroller_init(void)
{
  /* Perform any one-time initialization of the class implementation */

  nxsem_init(&g_exclsem, 0, 1);
  nxsem_init(&g_syncsem, 0, 0);

  /* The g_syncsem semaphore is used for signaling and, hence, should not
   * have priority inheritance enabled.
   */

  nxsem_set_protocol(&g_syncsem, SEM_PRIO_NONE);

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_xboxcontroller);
}
