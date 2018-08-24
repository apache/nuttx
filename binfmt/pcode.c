/****************************************************************************
 * binfmt/pcode.c
 *
 *   Copyright (C) 2014-2017 Gregory Nutt. All rights reserved.
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

#include <sys/mount.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/poff.h>
#include <nuttx/fd/fs.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/pcode.h>

#include "pexec.h"
#include "pedefs.h"

#ifdef CONFIG_BINFMT_PCODE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check configuration.  This is not all of the configuration settings that
 * are required -- only the more obvious.
 */

#if CONFIG_NFILE_DESCRIPTORS < 1
#  error You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file
#endif

#ifdef CONFIG_BINFMT_DISABLE
#  error The binary loader is disabled (CONFIG_BINFMT_DISABLE)!
#endif

#ifndef CONFIG_SCHED_ONEXIT
#  error CONFIG_SCHED_ONEXIT is required
#endif

#ifndef CONFIG_BINFMT_PCODE_VARSTACKSIZE
# define CONFIG_BINFMT_PCODE_VARSTACKSIZE 1024
#endif

#ifndef CONFIG_BINFMT_PCODE_STRSTACKSIZE
# define CONFIG_BINFMT_PCODE_STRSTACKSIZE 128
#endif

#ifdef CONFIG_BINFMT_PCODE_TEST_FS
#  ifndef CONFIG_FS_ROMFS
#    error You must select CONFIG_FS_ROMFS in your configuration file
#  endif

#  ifdef CONFIG_DISABLE_MOUNTPOINT
#    error You must not disable mountpoints via CONFIG_DISABLE_MOUNTPOINT in your configuration file
#  endif

#  ifndef CONFIG_BINFMT_PCODE_TEST_DEVMINOR
#    define CONFIG_BINFMT_PCODE_TEST_DEVMINOR 0
#  endif

#  ifndef CONFIG_BINFMT_PCODE_TEST_DEVPATH
#    define CONFIG_BINFMT_PCODE_TEST_DEVPATH "/dev/ram0"
#  endif

#  ifndef CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT
#    define CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT "/bin"
#  endif
#endif

/* Describe the ROMFS file system */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b)+SECTORSIZE-1)/SECTORSIZE)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

struct binfmt_handoff_s
{
  sem_t exclsem;              /* Supports mutually exclusive access */
  FAR struct binary_s *binp;  /* Binary format being handed off */
  FAR char *fullpath;         /* Full path to the P-Code file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pcode_run(FAR char *exepath, size_t varsize, size_t strsize);
#ifdef CONFIG_BINFMT_PCODE_TEST_FS
static int pcode_mount_testfs(void);
#endif
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static void pcode_onexit(int exitcode, FAR void *arg);
#endif
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static int pcode_proxy(int argc, char **argv);
#endif
static int pcode_load(FAR struct binary_s *binp);
static int pcode_unload(FAR struct binary_s *binp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_pcode_binfmt =
{
  NULL,          /* next */
  pcode_load,    /* load */
  pcode_unload,  /* unload */
};

struct binfmt_handoff_s g_pcode_handoff;

#ifdef CONFIG_BINFMT_PCODE_TEST_FS
#  include "romfs.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcode_run
 *
 * Description:
 *   Execute/interpret a P-Code file.  This function does not return until
 *   the P-code program terminates or until a fatal error occurs.
 *
 * Input Parameters:
 *   exepath - The full path to the P-Code binary.
 *   varsize - Size of the P-Code variable stack
 *   strsize - the size of the P-Code string stack.
 *
 * Returned Value:
 *   OK if the P-Code program successfully terminated; A negated errno value
 *   is returned on the event of any failure.
 *
 ****************************************************************************/

static int pcode_run(FAR char *exepath, size_t varsize, size_t strsize)
{
  FAR struct pexec_s *st;
  int errcode;
  int ret = OK;

  /* Load the POFF file into memory */

  st = pload(exepath, varsize, varsize);
  if (!st)
    {
      berr("ERROR: Could not load %s\n", exepath);
      return -ENOEXEC;
    }

  binfo("Loaded %s\n", exepath);

  /* Execute the P-Code program until a stopping condition occurs */

  for (; ; )
    {
      /* Execute the instruction; Check for exceptional conditions */

      errcode = pexec(st);
      if (errcode != eNOERROR)
        {
          break;
        }
    }

  if (errcode != eEXIT)
    {
      /* REVISIT: Select a more appropriated return errocode */

      berr("ERROR: Runtime error 0x%02x -- Execution Stopped\n", errcode);
      ret = -ENOEXEC;
    }

  /* Clean up resources used by the interpreter */

  binfo("Execution terminated\n");
  pexec_release(st);
  return ret;
}

/****************************************************************************
 * Name: pcode_mount_testfs
 *
 * Description:
 *   If so configured, then mount the P-Code test file system
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_PCODE_TEST_FS
static int pcode_mount_testfs(void)
{
  int ret;

  /* Create a ROM disk for the ROMFS filesystem */

  binfo("Registering romdisk at /dev/ram%d\n", CONFIG_BINFMT_PCODE_TEST_DEVMINOR);
  ret = romdisk_register(CONFIG_BINFMT_PCODE_TEST_DEVMINOR, (FAR uint8_t *)romfs_img,
                         NSECTORS(ROMFS_IMG_LEN), SECTORSIZE);
  if (ret < 0)
    {
      berr("ERROR: romdisk_register failed: %d\n", ret);
      return ret;
    }

  /* Mount the test file system */

  binfo("Mounting ROMFS filesystem at target=%s with source=%s\n",
         CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT,
         CONFIG_BINFMT_PCODE_TEST_DEVPATH);

  ret = mount(CONFIG_BINFMT_PCODE_TEST_DEVPATH,
              CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT,
              "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      int errval = get_errno();
      DEBUGASSERT(errval > 0);

      berr("ERROR: mount(%s,%s,romfs) failed: %d\n",
           CONFIG_BINFMT_PCODE_TEST_DEVPATH,
           CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT, errval);

      return -errval;
    }

  /* Does the system support the PATH variable?  Has the PATH variable
   * already been set?  If YES and NO, then set the PATH variable to
   * the ROMFS mountpoint.
   */

#if defined(CONFIG_BINFMT_EXEPATH) && !defined(CONFIG_PATH_INITIAL)
  (void)setenv("PATH", CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT, 1);
#endif

  return OK;
}
#else
#  define pcode_mount_testfs() (OK)
#endif

/****************************************************************************
 * Name: pcode_onexit
 *
 * Description:
 *   This is the proxy program that runs and starts the P-Code interpreter.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static void pcode_onexit(int exitcode, FAR void *arg)
{
  FAR struct binary_s *binp = (FAR struct binary_s *)arg;
  DEBUGASSERT(binp);

  /* And unload the module */

  (void)unload_module(binp);
}
#endif

/****************************************************************************
 * Name: pcode_proxy
 *
 * Description:
 *   This is the proxy program that runs and starts the P-Code interpreter.
 *
 * REVISIT:  There are issues here when CONFIG_BUILD_PROTECTED or
 * CONFIG_BUILD_KERNEL are selected.  Also this implementation is too highly
 * couple to logic in the apps/ directory.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static int pcode_proxy(int argc, char **argv)
{
  FAR struct binary_s *binp;
  FAR char *fullpath;
  int ret;

  /* Get the struct binary_s instance from the handoff structure */

  binp                     = g_pcode_handoff.binp;
  g_pcode_handoff.binp     = NULL;
  fullpath                 = g_pcode_handoff.fullpath;
  g_pcode_handoff.fullpath = NULL;

  nxsem_post(&g_pcode_handoff.exclsem);
  DEBUGASSERT(binp && fullpath);

  binfo("Executing %s\n", fullpath);

  /* Set-up the on-exit handler that will unload the module on exit */

  ret = on_exit(pcode_onexit, binp);
  if (ret < 0)
    {
      berr("ERROR: on_exit failed: %d\n", get_errno());
      kmm_free(fullpath);
      return EXIT_FAILURE;
    }

  /* Load the P-code file and execute it */

  ret = pcode_run(fullpath, CONFIG_BINFMT_PCODE_VARSTACKSIZE,
                  CONFIG_BINFMT_PCODE_STRSTACKSIZE);

  /* We no longer need the fullpath */

  kmm_free(fullpath);

  /* Check the result of the interpretation */

  if (ret < 0)
    {
      berr("ERROR: Execution failed\n");
      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
#else
#  error Missing logic for the case of CONFIG_BUILD_PROTECTED/KERNEL
#endif

/****************************************************************************
 * Name: pcode_load
 *
 * Description:
 *   Verify that the file is a pcode binary.
 *
 ****************************************************************************/

static int pcode_load(struct binary_s *binp)
{
  FAR struct poff_fileheader_s hdr;
  FAR uint8_t *ptr;
  size_t remaining;
  ssize_t nread;
  int fd;
  int ret;

  binfo("Loading file: %s\n", binp->filename);

  /* Open the binary file for reading (only) */

  fd = open(binp->filename, O_RDONLY);
  if (fd < 0)
    {
      int errval = get_errno();
      berr("ERROR: Failed to open binary %s: %d\n", binp->filename, errval);
      return -errval;
    }

  /* Read the POFF file header */

  for (remaining = sizeof(struct poff_fileheader_s),
       ptr = (FAR uint8_t *)&hdr; remaining > 0; )
    {
      /* Read the next GULP */

      nread = nx_read(fd, ptr, remaining);
      if (nread < 0)
        {
          /* If the failure is EINTR, then this is not an error; the
           * nx_read() was simply interrupted by a signal.
           */

          if (nread != -EINTR)
            {
              berr("ERROR: read failed: %d\n", (int)nread);
              ret = nread;
              goto errout_with_fd;
            }

          berr("Interrupted by a signal\n");
        }
      else
        {
          /* Set up for the next gulp */

          DEBUGASSERT(nread > 0 && nread <= remaining);
          remaining -= nread;
          ptr += nread;
        }
    }

#ifdef CONFIG_BINFMT_PCODE_DUMPBUFFER
  lib_dumpbuffer("POFF File Header", &hdr, sizeof(poff_fileheader_s));
#endif

  /* Verify that the file is a P-Code executable */

  if (memcmp(&hdr.fh_ident, FHI_POFF_MAG, 4) != 0 || hdr.fh_type != FHT_EXEC)
    {
      _err("ERROR: File is not a P-code executable: %d\n");
      ret = -ENOEXEC;
      goto errout_with_fd;
    }

  /* Return the load information.
   * REVISIT:  There are issues here when CONFIG_BUILD_PROTECTED or
   * CONFIG_BUILD_KERNEL are selected.
   */

  binp->entrypt   = pcode_proxy;
  binp->stacksize = CONFIG_BINFMT_PCODE_STACKSIZE;
  binp->priority  = CONFIG_BINFMT_PCODE_PRIORITY;

  /* Get exclusive access to the p-code handoff structure */

  do
    {
      ret = nxsem_wait(&g_pcode_handoff.exclsem);
      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  /* Save the data that we need to handoff to the child thread */

  DEBUGASSERT(g_pcode_handoff.binp == NULL &&
              g_pcode_handoff.fullpath == NULL);

  /* Duplicate the full path to the binary */

  g_pcode_handoff.fullpath = strdup(binp->filename);
  if (!g_pcode_handoff.fullpath)
    {
      berr("ERROR: Failed to duplicate the full path: %d\n",
           binp->filename);

      nxsem_post(&g_pcode_handoff.exclsem);
      ret = -ENOMEM;
      goto errout_with_fd;
    }

  g_pcode_handoff.binp = binp;

  /* Successfully identified (but not really loaded) a p-code binary */

  ret = OK;

errout_with_fd:
  close(fd);
  return ret;
}

/****************************************************************************
 * Name: pcode_unload
 *
 * Description:
 *   Called when the pcode binary is unloaded.  This is necessary primarily
 *   to handler error conditions where unload_module is called after
 *   pcode_load without having executed the P-Code module.
 *
 ****************************************************************************/

static int pcode_unload(struct binary_s *binp)
{
  /* Increment the semaphore count back to one if appropriate */

  if (g_pcode_handoff.binp)
    {
      g_pcode_handoff.binp = NULL;
      nxsem_post(&g_pcode_handoff.exclsem);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcode_initialize
 *
 * Description:
 *   In order to use the P-code binary format, this function must be called
 *   during system initialization to register the P-Code binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int pcode_initialize(void)
{
  int ret;

  /* Initialize globals */

  nxsem_init(&g_pcode_handoff.exclsem, 0, 1);

  /* Mount the test file system */

  ret = pcode_mount_testfs();
  if (ret < 0)
    {
      berr("ERROR: Failed to mount test file system: %d\n", ret);
      return ret;
    }

  /* Register ourselves as a binfmt loader */

  binfo("Registering P-Code Loader\n");

  ret = register_binfmt(&g_pcode_binfmt);
  if (ret != 0)
    {
      berr("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: pcode_uninitialize
 *
 * Description:
 *   Unregister the pcode binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pcode_uninitialize(void)
{
  int ret;

  /* Unregister the binary format */

  ret = unregister_binfmt(&g_pcode_binfmt);
  if (ret < 0)
    {
      berr("ERROR: unregister_binfmt() failed: %d\n", ret);
    }

#ifdef CONFIG_BINFMT_PCODE_TEST_FS
  ret = umount(CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT);
  if (ret < 0)
    {
      int errval = get_errno();
      DEBUGASSERT(errval > 0);

      berr("ERROR: umount(%s) failed: %d\n",
           CONFIG_BINFMT_PCODE_TEST_MOUNTPOINT, errval);
      UNUSED(errval);
    }
#endif

  /* Uninitialize globals */

  nxsem_destroy(&g_pcode_handoff.exclsem);
}

#endif /* CONFIG_BINFMT_PCODE */
