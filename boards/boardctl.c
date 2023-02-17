/****************************************************************************
 * boards/boardctl.c
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

#include <sys/types.h>
#include <sys/boardctl.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/board.h>
#include <nuttx/lib/modlib.h>
#include <nuttx/binfmt/symtab.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/reboot_notifier.h>

#ifdef CONFIG_NX
#  include <nuttx/nx/nxmu.h>
#endif

#ifdef CONFIG_VNCSERVER
#  include <nuttx/video/vnc.h>
#endif

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
#  include <nuttx/usb/cdcacm.h>
#  include <nuttx/usb/pl2303.h>
#  include <nuttx/usb/usbmsc.h>
#  include <nuttx/usb/composite.h>
#endif

#ifdef CONFIG_BOARDCTL_TESTSET
#  include <nuttx/spinlock.h>
#endif

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_BUILTIN)
#  include <nuttx/lib/builtin.h>
#endif

#ifdef CONFIG_BOARDCTL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: boardctl_usbdevctrl
 *
 * Description:
 *   Handler the USB device control command.
 *
 * Input Parameters:
 *   ctrl - Described the USB device control command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
static inline int
  boardctl_usbdevctrl(FAR struct boardioc_usbdev_ctrl_s *ctrl)
{
  int ret = OK;

  switch (ctrl->usbdev)
    {
#ifdef CONFIG_CDCACM
      case BOARDIOC_USBDEV_CDCACM:           /* CDC/ACM, not in a composite */
        switch (ctrl->action)
          {
            case BOARDIOC_USBDEV_INITIALIZE: /* Initialize CDC/ACM device */
              break;                         /* There is no CDC/ACM initialization */

            case BOARDIOC_USBDEV_CONNECT:    /* Connect the CDC/ACM device */
#ifndef CONFIG_CDCACM_COMPOSITE
              {
                DEBUGASSERT(ctrl->handle != NULL);
                ret = cdcacm_initialize(ctrl->instance, ctrl->handle);
              }
#endif
              break;

            case BOARDIOC_USBDEV_DISCONNECT: /* Disconnect the CDC/ACM device */
              {
                DEBUGASSERT(ctrl->handle != NULL && *ctrl->handle != NULL);
                cdcacm_uninitialize(*ctrl->handle);
              }
              break;

            default:
              ret = -EINVAL;
              break;
          }
        break;
#endif

#ifdef CONFIG_PL2303
      case BOARDIOC_USBDEV_PL2303:           /* PL2303 serial, not in a composite */
        switch (ctrl->action)
          {
            case BOARDIOC_USBDEV_INITIALIZE: /* Initialize PL2303 serial device */
              break;                         /* There is no PL2303 serial initialization */

            case BOARDIOC_USBDEV_CONNECT:    /* Connect the CDC/ACM device */
              ret = usbdev_serialinitialize(ctrl->instance);
              break;

            case BOARDIOC_USBDEV_DISCONNECT: /* There is no PL2303 serial disconnect */
              ret = -ENOSYS;
              break;

            default:
              ret = -EINVAL;
              break;
          }
        break;
#endif

#ifdef CONFIG_USBMSC
      case BOARDIOC_USBDEV_MSC:              /* Mass storage class */
        switch (ctrl->action)
          {
            case BOARDIOC_USBDEV_INITIALIZE: /* Initialize USB MSC device */
              {
                ret = board_usbmsc_initialize(ctrl->instance);
              }
              break;

            case BOARDIOC_USBDEV_CONNECT:    /* Connect the USB MSC device */
              {
                DEBUGASSERT(ctrl->handle != NULL);
                ret = -ENOSYS;
              }
              break;

            case BOARDIOC_USBDEV_DISCONNECT: /* Disconnect the USB MSC device */
              {
                DEBUGASSERT(ctrl->handle != NULL && *ctrl->handle != NULL);
                usbmsc_uninitialize(*ctrl->handle);
              }
              break;

            default:
              ret = -EINVAL;
              break;
          }
        break;
#endif

#ifdef CONFIG_USBDEV_COMPOSITE
      case BOARDIOC_USBDEV_COMPOSITE:        /* Composite device */
        switch (ctrl->action)
          {
            case BOARDIOC_USBDEV_INITIALIZE: /* Initialize Composite device */
              {
                ret = board_composite_initialize(ctrl->instance);
              }
              break;

            case BOARDIOC_USBDEV_CONNECT:    /* Connect the Composite device */
              {
                DEBUGASSERT(ctrl->handle != NULL);

                *ctrl->handle =
                  board_composite_connect(ctrl->instance, ctrl->config);

                if (*ctrl->handle == NULL)
                  {
                    ret = -EIO;
                  }
              }
              break;

            case BOARDIOC_USBDEV_DISCONNECT: /* Disconnect the Composite
                                              * device */
              {
                DEBUGASSERT(ctrl->handle != NULL && *ctrl->handle != NULL);
                composite_uninitialize(*ctrl->handle);
              }
              break;

            default:
              ret = -EINVAL;
              break;
          }
        break;
#endif
      default:
        ret = -EINVAL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: boardctl_pmctrl
 *
 * Description:
 *   Handle power state transition and query command.
 *
 * Input Parameters:
 *   ctrl - Described the power state transition and query command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static inline int boardctl_pmctrl(FAR struct boardioc_pm_ctrl_s *ctrl)
{
  int ret = OK;

  switch (ctrl->action)
    {
      case BOARDIOC_PM_ACTIVITY:
        pm_activity(ctrl->domain, ctrl->priority);
        break;

      case BOARDIOC_PM_STAY:
        pm_stay(ctrl->domain, ctrl->state);
        break;

      case BOARDIOC_PM_RELAX:
        pm_relax(ctrl->domain, ctrl->state);
        break;

      case BOARDIOC_PM_STAYCOUNT:
        ctrl->count = pm_staycount(ctrl->domain, ctrl->state);
        break;

      case BOARDIOC_PM_QUERYSTATE:
        ctrl->state = pm_querystate(ctrl->domain);
        break;

      case BOARDIOC_PM_CHANGESTATE:
        ret = pm_changestate(ctrl->domain, ctrl->state);
        break;

      case BOARDIOC_PM_CHECKSTATE:
        ctrl->state = pm_checkstate(ctrl->domain);
        break;

      default:
        ret = -EINVAL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: boardctl
 *
 * Description:
 *   In a small embedded system, there will typically be a much greater
 *   interaction between application and low-level board features.  The
 *   canonically correct to implement such interactions is by implementing a
 *   character driver and performing the interactions via low level ioctl
 *   calls.  This, however, may not be practical in many cases and will lead
 *   to "correct" but awkward implementations.
 *
 *   boardctl() is non-standard OS interface to alleviate the problem.  It
 *   basically circumvents the normal device driver ioctl interlace and
 *   allows the application to perform direction IOCTL-like calls to the
 *   board-specific logic.  In it is especially useful for setting up board
 *   operational and test configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to indicate the nature of the failure.
 *
 ****************************************************************************/

int boardctl(unsigned int cmd, uintptr_t arg)
{
  int ret;

  switch (cmd)
    {
      /* CMD:           BOARDIOC_INIT
       * DESCRIPTION:   Perform one-time application initialization.
       * ARG:           The boardctl() argument is passed to the
       *                board_app_initialize() implementation without
       *                modification.  The argument has no meaning to NuttX;
       *                the meaning of the argument is a contract between
       *                the board-specific initialization logic and the
       *                matching application logic.  The value cold be such
       *                things as a mode enumeration value, a set of DIP
       *                switch switch settings, a pointer to configuration
       *                data read from a file or serial FLASH, or whatever
       *                you would like to do with it.  Every implementation
       *                should accept zero/NULL as a default configuration.
       * CONFIGURATION: CONFIG_BOARDCTL
       * DEPENDENCIES:  Board logic must provide board_app_initialization
       */

      case BOARDIOC_INIT:
        {
          ret = board_app_initialize(arg);
        }
        break;

#ifdef CONFIG_BOARDCTL_FINALINIT
      /* CMD:           BOARDIOC_FINALINIT
       * DESCRIPTION:   Perform one-time application initialization after
       *                start-up script.
       * ARG:           The argument has no meaning
       * CONFIGURATION: CONFIG_BOARDCTL_FINALINIT
       * DEPENDENCIES:  Board logic must provide board_app_finalinitialize
       */

      case BOARDIOC_FINALINIT:
        {
          ret = board_app_finalinitialize(arg);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_POWEROFF
      /* CMD:           BOARDIOC_POWEROFF
       * DESCRIPTION:   Power off the board
       * ARG:           Integer value providing power off status information
       * CONFIGURATION: CONFIG_BOARDCTL_POWEROFF
       * DEPENDENCIES:  Board logic must provide board_power_off
       */

      case BOARDIOC_POWEROFF:
        {
          reboot_notifier_call_chain(SYS_POWER_OFF, (FAR void *)arg);
          ret = board_power_off((int)arg);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_RESET
      /* CMD:           BOARDIOC_RESET
       * DESCRIPTION:   Reset the board
       * ARG:           Integer value providing power off status information
       * CONFIGURATION: CONFIG_BOARDCTL_RESET
       * DEPENDENCIES:  Board logic must provide board_reset
       */

      case BOARDIOC_RESET:
        {
          reboot_notifier_call_chain(SYS_RESTART, (FAR void *)arg);
          ret = board_reset((int)arg);
        }
        break;
#endif

#ifdef CONFIG_PM
      /* CMD:           BOARDIOC_PM_CONTROL
       * DESCRIPTION:   manage power state transition and query
       * ARG:           A pointer to an instance of struct boardioc_pm_ctrl_s
       * CONFIGURATION: CONFIG_PM
       * DEPENDENCIES:  None
       */

      case BOARDIOC_PM_CONTROL:
        {
          FAR struct boardioc_pm_ctrl_s *ctrl =
            (FAR struct boardioc_pm_ctrl_s *)arg;

          DEBUGASSERT(ctrl != NULL);
          ret = boardctl_pmctrl(ctrl);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_UNIQUEID
      /* CMD:           BOARDIOC_UNIQUEID
       * DESCRIPTION:   Return a unique ID associated with the board (such
       *                as a serial number or a MAC address).
       * ARG:           A writable array of size
       *                CONFIG_BOARDCTL_UNIQUEID_SIZE in which to receive
       *                the board unique ID.
       * DEPENDENCIES:  Board logic must provide the board_uniqueid()
       *                interface.
       */

      case BOARDIOC_UNIQUEID:
        {
          ret = board_uniqueid((FAR uint8_t *)arg);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_UNIQUEKEY
      /* CMD:           BOARDIOC_UNIQUEKEY
       * DESCRIPTION:   Return a unique KEY associated with the board (such
       *                as a trusted key or a private identity).
       * ARG:           A writable array of size
       *                CONFIG_BOARDCTL_UNIQUEKEY_SIZE in which to receive
       *                the board unique KEY.
       * DEPENDENCIES:  Board logic must provide the board_uniquekey()
       *                interface.
       */

      case BOARDIOC_UNIQUEKEY:
        {
          ret = board_uniquekey((FAR uint8_t *)arg);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_SWITCH_BOOT
      /* CMD:           BOARDIOC_SWITCH_BOOT
       * DESCRIPTION:   Used to change the system boot behavior. Switch to
       *                the updated or specified boot system.
       * ARG:           Boot system updated or specified
       * DEPENDENCIES:  Board logic must provide the board_switch_boot()
       *                interface.
       */

      case BOARDIOC_SWITCH_BOOT:
        {
          ret = board_switch_boot((FAR const char *)arg);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE
      /* CMD:           BOARDIOC_BOOT_IMAGE
       * DESCRIPTION:   Boot a new application firmware image.
       *                Execute the required actions for booting a new
       *                application firmware image (e.g. deinitialize
       *                peripherals, load the Program Counter register with
       *                the application firmware image entry point address).
       * ARG:           Pointer to a read-only instance of struct
       *                boardioc_boot_info_s.
       * DEPENDENCIES:  Board logic must provide the board_boot_image()
       *                interface.
       */

      case BOARDIOC_BOOT_IMAGE:
        {
          FAR const struct boardioc_boot_info_s *info =
            (FAR const struct boardioc_boot_info_s *)arg;

          DEBUGASSERT(info != NULL);

          ret = board_boot_image(info->path, info->header_size);
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_MKRD
      /* CMD:           BOARDIOC_MKRD
       * DESCRIPTION:   Create a RAM disk
       * ARG:           Pointer to read-only instance of struct
       *                boardioc_mkrd_s.
       * CONFIGURATION: CONFIG_BOARDCTL_MKRD
       * DEPENDENCIES:  None
       */

      case BOARDIOC_MKRD:
        {
          FAR const struct boardioc_mkrd_s *desc =
            (FAR const struct boardioc_mkrd_s *)arg;

          if (desc == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = mkrd((int)desc->minor, desc->nsectors, desc->sectsize,
                         desc->rdflags);
            }
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_ROMDISK
      /* CMD:           BOARDIOC_ROMDISK
       * DESCRIPTION:   Register
       * ARG:           Pointer to read-only instance of struct
       *                boardioc_romdisk_s.
       * CONFIGURATION: CONFIG_BOARDCTL_ROMDISK
       * DEPENDENCIES:  None
       */

      case BOARDIOC_ROMDISK:
        {
          FAR const struct boardioc_romdisk_s *desc =
            (FAR const struct boardioc_romdisk_s *)arg;

          if (desc == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = romdisk_register((int)desc->minor, desc->image,
                                     desc->nsectors, desc->sectsize);
            }
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_APP_SYMTAB
      /* CMD:           BOARDIOC_APP_SYMTAB
       * DESCRIPTION:   Select the application symbol table.  This symbol
       *                table provides the symbol definitions exported to
       *                application code from application space.
       * ARG:           A pointer to an instance of struct boardioc_symtab_s
       * CONFIGURATION: CONFIG_BOARDCTL_APP_SYMTAB
       * DEPENDENCIES:  None
       */

      case BOARDIOC_APP_SYMTAB:
        {
          FAR const struct boardioc_symtab_s *symdesc =
            (FAR const struct boardioc_symtab_s *)arg;

         DEBUGASSERT(symdesc != NULL);
         exec_setsymtab(symdesc->symtab, symdesc->nsymbols);
         ret = OK;
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_OS_SYMTAB
      /* CMD:           BOARDIOC_OS_SYMTAB
       * DESCRIPTION:   Select the OS symbol table.  This symbol table
       *                provides the symbol definitions exported by the OS to
       *                kernel modules.
       * ARG:           A pointer to an instance of struct boardioc_symtab_s
       * CONFIGURATION: CONFIG_BOARDCTL_OS_SYMTAB
       * DEPENDENCIES:  None
       */

      case BOARDIOC_OS_SYMTAB:
        {
          FAR const struct boardioc_symtab_s *symdesc =
            (FAR const struct boardioc_symtab_s *)arg;

         DEBUGASSERT(symdesc != NULL);
         modlib_setsymtab(symdesc->symtab, symdesc->nsymbols);
         ret = OK;
        }
        break;
#endif

#ifdef CONFIG_BUILTIN
      /* CMD:           BOARDIOC_BUILTINS
       * DESCRIPTION:   Provide the user-space list of built-in applications
       *                for use by BINFS in protected mode.  Normally this
       *                is small set of globals provided by user-space
       *                logic.  It provides name-value pairs for associating
       *                built-in application names with user-space entry
       *                point addresses.  These globals are only needed for
       *                use by BINFS which executes built-in applications
       *                from kernel-space in PROTECTED mode. In the FLAT
       *                build, the user space globals are readily
       *                available.  (BINFS is not supportable in KERNEL mode
       *                since user-space address have no general meaning
       *                that configuration).
       * ARG:           A pointer to an instance of struct boardioc_builtin_s
       * CONFIGURATION: This BOARDIOC command is always available when
       *                CONFIG_BUILTIN is enabled, but does nothing unless
       *                CONFIG_BUILD_KERNEL is selected.
       * DEPENDENCIES:  None
       */

      case BOARDIOC_BUILTINS:
        {
#if defined(CONFIG_BUILD_PROTECTED)
          FAR const struct boardioc_builtin_s *builtin =
            (FAR const struct boardioc_builtin_s *)arg;

         DEBUGASSERT(builtin != NULL);
         builtin_setlist(builtin->builtins, builtin->count);
#endif
         ret = OK;
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_USBDEVCTRL
      /* CMD:           BOARDIOC_USBDEV_CONTROL
       * DESCRIPTION:   Manage USB device classes
       * ARG:           A pointer to an instance of struct
       *                boardioc_usbdev_ctrl_s
       * CONFIGURATION: CONFIG_BOARDCTL && CONFIG_BOARDCTL_USBDEVCTRL
       * DEPENDENCIES:  Board logic must provide board_<usbdev>_initialize()
       */

      case BOARDIOC_USBDEV_CONTROL:
        {
          FAR struct boardioc_usbdev_ctrl_s *ctrl =
            (FAR struct boardioc_usbdev_ctrl_s *)arg;

          DEBUGASSERT(ctrl != NULL);
          ret = boardctl_usbdevctrl(ctrl);
        }
        break;
#endif

#ifdef CONFIG_NX
      /* CMD:           BOARDIOC_NX_START
       * DESCRIPTION:   Start the NX server
       * ARG:           Integer display number to be served by this NXMU
       *                instance.
       * CONFIGURATION: CONFIG_NX
       * DEPENDENCIES:  Base graphics logic provides nxmu_start()
       */

      case BOARDIOC_NX_START:
        {
          /* REVISIT:  Plane number is forced to zero.  On multiplanar
           * displays there may be multiple planes.  Only one is supported
           * here.
           */

          ret = nxmu_start((int)arg, 0);
        }
        break;
#endif

#ifdef CONFIG_VNCSERVER
      /* CMD:           BOARDIOC_VNC_START
       * DESCRIPTION:   Start the NX server and framebuffer driver.
       * ARG:           A reference readable instance of struct
       *                boardioc_vncstart_s
       * CONFIGURATION: CONFIG_VNCSERVER
       * DEPENDENCIES:  VNC server provides nx_vnc_fbinitialize()
       */

      case BOARDIOC_VNC_START:
        {
          FAR struct boardioc_vncstart_s *vnc =
            (FAR struct boardioc_vncstart_s *)arg;

          if (vnc == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              /* Setup the VNC server to support keyboard/mouse inputs */

              ret = nx_vnc_fbinitialize(vnc->display, vnc->handle);
            }
        }
        break;
#endif

#ifdef CONFIG_NXTERM
      /* CMD:           BOARDIOC_NXTERM
       * DESCRIPTION:   Create an NX terminal device
       * ARG:           A reference readable/writable instance of struct
       *                boardioc_nxterm_create_s
       * CONFIGURATION: CONFIG_NXTERM
       * DEPENDENCIES:  Base NX terminal logic provides nx_register() and
       *                nxtk_register()
       */

      case BOARDIOC_NXTERM:
        {
          FAR struct boardioc_nxterm_create_s *nxterm =
            (FAR struct boardioc_nxterm_create_s *)arg;

          if (nxterm == NULL)
            {
              ret = -EINVAL;
            }
          else if (nxterm->type == BOARDIOC_XTERM_RAW)
            {
              nxterm->nxterm = nx_register((NXWINDOW)nxterm->hwnd,
                                           &nxterm->wndo,
                                           (int)nxterm->minor);

              ret = nxterm->nxterm == NULL ? -ENODEV : OK;
            }
          else if (nxterm->type == BOARDIOC_XTERM_FRAMED)
            {
              nxterm->nxterm = nxtk_register((NXTKWINDOW)nxterm->hwnd,
                                             &nxterm->wndo,
                                             (int)nxterm->minor);

              ret = nxterm->nxterm == NULL ? -ENODEV : OK;
            }
          else if (nxterm->type == BOARDIOC_XTERM_TOOLBAR)
            {
              nxterm->nxterm = nxtool_register((NXTKWINDOW)nxterm->hwnd,
                                               &nxterm->wndo,
                                               (int)nxterm->minor);

              ret = nxterm->nxterm == NULL ? -ENODEV : OK;
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* CMD:           BOARDIOC_NXTERM_IOCTL
       * DESCRIPTION:   Create an NX terminal IOCTL command.  Normal
       *                IOCTLs cannot be be performed in most graphics
       *                contexts since the depend on the task holding an
       *                open file descriptor
       * ARG:           A reference readable/writable instance of struct
       *                boardioc_nxterm_ioctl_s
       * CONFIGURATION: CONFIG_NXTERM
       * DEPENDENCIES:  Base NX terminal logic provides nxterm_ioctl_tap()
       */

      case BOARDIOC_NXTERM_IOCTL:
        {
          FAR struct boardioc_nxterm_ioctl_s *nxterm =
            (FAR struct boardioc_nxterm_ioctl_s *)arg;

          ret = nxterm_ioctl_tap(nxterm->cmd, nxterm->arg);
        }
        break;

#endif /* CONFIG_NXTERM */

#ifdef CONFIG_BOARDCTL_TESTSET
      /* CMD:           BOARDIOC_TESTSET
       * DESCRIPTION:   Access architecture-specific up_testset() operation
       * ARG:           A pointer to a write-able spinlock object.  On
       *                success the  preceding spinlock state is returned:
       *                0=unlocked, 1=locked.
       * CONFIGURATION: CONFIG_BOARDCTL_TESTSET
       * DEPENDENCIES:  Architecture-specific logic provides up_testset()
       */

      case BOARDIOC_TESTSET:
        {
          volatile FAR spinlock_t *lock = (volatile FAR spinlock_t *)arg;

          if (lock == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = up_testset(lock) == SP_LOCKED ? 1 : 0;
            }
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
      /* CMD:           BOARDIOC_RESET_CAUSE
       * DESCRIPTION:   Get the cause of last-time board reset
       * ARG:           A pointer to an instance of struct
       *                boardioc_reset_cause_s
       * CONFIGURATION: CONFIG_BOARDCTL_RESET_CAUSE
       * DEPENDENCIES:  Board logic must provide the
       *                board_reset_cause() interface.
       */

      case BOARDIOC_RESET_CAUSE:
        {
          FAR struct boardioc_reset_cause_s *cause =
            (FAR struct boardioc_reset_cause_s *)arg;

          DEBUGASSERT(cause != NULL);
          ret = board_reset_cause(cause);
        }
        break;
#endif

       default:
         {
#ifdef CONFIG_BOARDCTL_IOCTL
           /* Boards may also select CONFIG_BOARDCTL_IOCTL=y to enable
            * board-specific commands.  In this case, all commands not
            * recognized by boardctl() will be forwarded to the board-
            * provided board_ioctl() function.
            */

           ret = board_ioctl(cmd, arg);
#else
           ret = -ENOTTY;
#endif
         }
         break;
    }

  /* Set the errno value on any errors */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}

#endif /* CONFIG_BOARDCTL */
