/****************************************************************************
 * examples/thttpd/main.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/ioctl.h>
#include <sys/mount.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/uip/thttpd.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-lib.h>
#include <netinet/ether.h>

#include <nuttx/ramdisk.h>
#include <nuttx/binfmt.h>
#include <nuttx/nxflat.h>

#include "content/romfs.h"
#include "content/symtab.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Check configuration.  This is not all of the configuration settings that
 * are required -- only the more obvious.
 */

#if CONFIG_NFILE_DESCRIPTORS < 1
#  error "You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file"
#endif

#ifndef CONFIG_NXFLAT
#  error "You must select CONFIG_NXFLAT in your configuration file"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "You must select CONFIG_FS_ROMFS in your configuration file"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "You must not disable mountpoints via CONFIG_DISABLE_MOUNTPOINT in your configuration file"
#endif

#ifdef CONFIG_BINFMT_DISABLE
#  error "You must not disable loadable modules via CONFIG_BINFMT_DISABLE in your configuration file"
#endif

/* Describe the ROMFS file system */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b)+SECTORSIZE-1)/SECTORSIZE)
#define ROMFSDEV     "/dev/ram0"
#define MOUNTPT      CONFIG_THTTPD_PATH

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush()   fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message      lib_lowprintf
#    define msgflush()
#  else
#    define message      printf
#    define msgflush()   fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These values must be provided by the user before the THTTPD task daemon
 * is started:
 *
 * g_thttpdsymtab:  A symbol table describing all of the symbols exported
 *   from the base system.  These symbols are used to bind address references
 *   in CGI programs to NuttX.
 * g_nsymbols:  The number of symbols in g_thttpdsymtab[].
 */

FAR const struct symtab_s *g_thttpdsymtab;
int                         g_thttpdnsymbols;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_initialize
 ****************************************************************************/

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  /* Stub that must be provided only if the toolchain does not support weak
   * functions.
   */
}
#endif

/****************************************************************************
 * user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  struct in_addr addr;
#ifdef CONFIG_EXAMPLE_THTTPD_NOMAC
  uint8_t mac[IFHWADDRLEN];
#endif
  char *thttpd_argv = "thttpd";
  int ret;

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLE_THTTPD_NOMAC
  message("Assigning MAC\n");

  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xb0;
  mac[3] = 0x0b;
  mac[4] = 0xba;
  mac[5] = 0xbe;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  message("Setup network addresses\n");
  addr.s_addr = HTONL(CONFIG_THTTPD_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_THTTPD_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_THTTPD_NETMASK);
  uip_setnetmask("eth0", &addr);

  /* Initialize the NXFLAT binary loader */

  message("Initializing the NXFLAT binary loader\n");
  ret = nxflat_initialize();
  if (ret < 0)
    {
      message("ERROR: Initialization of the NXFLAT loader failed: %d\n", ret);
      exit(1);
    }

  /* Create a ROM disk for the ROMFS filesystem */

  message("Registering romdisk\n");
  ret = romdisk_register(0, (uint8_t*)romfs_img, NSECTORS(romfs_img_len), SECTORSIZE);
  if (ret < 0)
    {
      message("ERROR: romdisk_register failed: %d\n", ret);
      nxflat_uninitialize();
      exit(1);
    }

  /* Mount the file system */

  message("Mounting ROMFS filesystem at target=%s with source=%s\n",
         MOUNTPT, ROMFSDEV);

  ret = mount(ROMFSDEV, MOUNTPT, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      message("ERROR: mount(%s,%s,romfs) failed: %s\n",
              ROMFSDEV, MOUNTPT, errno);
      nxflat_uninitialize();
    }

  /* Start THTTPD.  At present, symbol table info is passed via global variables */

  g_thttpdsymtab   = exports;
  g_thttpdnsymbols = NEXPORTS;

  message("Starting THTTPD\n");
  msgflush();
  thttpd_main(1, &thttpd_argv);
  message("THTTPD terminated\n");
  msgflush();
  return 0;
}
