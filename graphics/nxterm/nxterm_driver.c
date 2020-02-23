/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_driver.c
 *
 *   Copyright (C) 2012, 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "nxterm.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nxterm_open(FAR struct file *filep);
static int     nxterm_close(FAR struct file *filep);
static ssize_t nxterm_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     nxterm_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     nxterm_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the common NX driver file operations */

#ifdef CONFIG_NXTERM_NXKBDIN

const struct file_operations g_nxterm_drvrops =
{
  nxterm_open,  /* open */
  nxterm_close, /* close */
  nxterm_read,  /* read */
  nxterm_write, /* write */
  NULL,         /* seek */
  nxterm_ioctl, /* ioctl */
  nxterm_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ,
  nxterm_unlink /* unlink */
#endif
};

#else /* CONFIG_NXTERM_NXKBDIN */

const struct file_operations g_nxterm_drvrops =
{
  nxterm_open,  /* open */
  nxterm_close, /* close */
  NULL,         /* read */
  nxterm_write, /* write */
  NULL,         /* seek */
  nxterm_ioctl, /* ioctl */
  NULL          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ,
  nxterm_unlink /* unlink */
#endif
};

#endif /* CONFIG_NXTERM_NXKBDIN */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_open
 ****************************************************************************/

static int nxterm_open(FAR struct file *filep)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct nxterm_state_s *priv  = inode->i_private;

  DEBUGASSERT(filep && filep->f_inode);

  /* Get the driver structure from the inode */

  inode = filep->f_inode;
  priv  = (FAR struct nxterm_state_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Verify that the driver is opened for write-only access */

#ifndef CONFIG_NXTERM_NXKBDIN
  if ((filep->f_oflags & O_RDOK) != 0)
    {
      gerr("ERROR: Attempted open with read access\n");
      return -EACCES;
    }
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* Increment the count of open file reference */

  DEBUGASSERT(priv->orefs != UINT8_MAX);
  priv->orefs++;
#endif

  /* Assign the driver structure to the file */

  filep->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: nxterm_close
 ****************************************************************************/

static int nxterm_close(FAR struct file *filep)
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  FAR struct nxterm_state_s *priv;
  int ret;

  /* Recover our private state structure */

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL);
  priv = (FAR struct nxterm_state_s *)filep->f_priv;

  /* Get exclusive access */

  ret = nxterm_semwait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Has the driver been unlinked?  Was this the last open references to the
   * terminal driver?
   */

  DEBUGASSERT(priv->orefs > 0);
  if (priv->unlinked && priv->orefs <= 1)
    {
      /* Yes.. Unregister the terminal device */

      nxterm_unregister(priv);
    }
  else
    {
      /* No.. Just decrement the count of open file references */

      priv->orefs--;
    }

  nxterm_sempost(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: nxterm_write
 ****************************************************************************/

static ssize_t nxterm_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct nxterm_state_s *priv;
  enum nxterm_vt100state_e state;
  ssize_t remaining;
  char ch;
  int ret;

  /* Recover our private state structure */

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL);
  priv = (FAR struct nxterm_state_s *)filep->f_priv;

  /* Get exclusive access */

  ret = nxterm_semwait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Hide the cursor while we update the display */

  nxterm_hidecursor(priv);

  /* Loop writing each character to the display */

  for (remaining = (ssize_t)buflen; remaining > 0; remaining--)
    {
      /* Get the next character from the user buffer */

      ch = *buffer++;

      /* Check if this character is part of a VT100 escape sequence */

      do
        {
          /* Is the character part of a VT100 escape sequnce? */

          state = nxterm_vt100(priv, ch);
          switch (state)
            {
              /* Character is not part of a VT100 escape sequence (and no
               * characters are buffer.
               */

              default:
              case VT100_NOT_CONSUMED:
                {
                  /* We can output the character to the window */

                  nxterm_putc(priv, (uint8_t)ch);
                }
              break;

            /* The full VT100 escape sequence was processed (and the new
             * character was consumed)
             */

            case VT100_PROCESSED:

            /* Character was consumed as part of the VT100 escape processing
             * (but the escape sequence is still incomplete.
             */

            case VT100_CONSUMED:
              {
                /* Do nothing... the VT100 logic owns the character */
              }
              break;

            /* Invalid/unsupported character in escape sequence */

            case VT100_ABORT:
              {
                int i;

                /* Add the first unhandled character to the window */

                nxterm_putc(priv, (uint8_t)priv->seq[0]);

                /* Move all buffer characters down one */

                for (i = 1; i < priv->nseq; i++)
                  {
                    priv->seq[i-1] = priv->seq[i];
                  }

                priv->nseq--;

                /* Then loop again and check if what remains is part of a
                 * VT100 escape sequence.  We could speed this up by
                 * checking if priv->seq[0] == ASCII_ESC.
                 */
              }
              break;
            }
        }
      while (state == VT100_ABORT);
    }

  /* Show the cursor at its new position */

  nxterm_showcursor(priv);
  nxterm_sempost(priv);
  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: nxterm_ioctl
 ****************************************************************************/

static int nxterm_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  /* NOTE:  We don't need driver context here because the NXTERM handle
   * provided within each of the NXTERM IOCTL command data.  Mutual
   * exclusion is similar managed by the IOCTL cmmand handler.
   *
   * This permits the IOCTL to be called in abnormal context (such as
   * from boardctl())
   */

  return nxterm_ioctl_tap(cmd, arg);
}

/****************************************************************************
 * Name: nxterm_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int nxterm_unlink(FAR struct inode *inode)
{
  FAR struct nxterm_state_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = inode->i_private;

  /* Get exclusive access */

  ret = nxterm_semwait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the terminal driver? */

  if (priv->orefs > 0)
    {
      /* Yes.. Just mark the driver unlinked.  Resources will be cleaned up
       * when the final reference is close.
       */

      priv->unlinked = true;
    }
  else
    {
      /* No.. Unregister the terminal device now */

      nxterm_unregister(priv);
    }

  nxterm_sempost(priv);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_ioctl_tap
 *
 * Description:
 *   Execute an NXTERM IOCTL command from an external caller.
 *
 * NOTE:  We don't need driver context here because the NXTERM handle
 * provided within each of the NXTERM IOCTL command data.  Mutual
 * exclusion is similar managed by the IOCTL cmmand handler.
 *
 * This permits the IOCTL to be called in abnormal context (such as
 * from boardctl())
 *
 ****************************************************************************/

int nxterm_ioctl_tap(int cmd, uintptr_t arg)
{
  int ret;

  switch (cmd)
    {
      /* CMD:           NXTERMIOC_NXTERM_REDRAW
       * DESCRIPTION:   Re-draw a portion of the NX console.  This function
       *                should be called from the appropriate window callback
       *                logic.
       * ARG:           A reference readable instance of struct
       *                nxtermioc_redraw_s
       * CONFIGURATION: CONFIG_NXTERM
       */

       case NXTERMIOC_NXTERM_REDRAW:
         {
           FAR struct nxtermioc_redraw_s *redraw =
             (FAR struct nxtermioc_redraw_s *)((uintptr_t)arg);

           nxterm_redraw(redraw->handle, &redraw->rect, redraw->more);
           ret = OK;
         }
         break;

      /* CMD:           NXTERMIOC_NXTERM_KBDIN
       * DESCRIPTION:   Provide NxTerm keyboard input to NX.
       * ARG:           A reference readable instance of struct
       *                nxtermioc_kbdin_s
       * CONFIGURATION: CONFIG_NXTERM_NXKBDIN
       */

       case NXTERMIOC_NXTERM_KBDIN:
         {
#ifdef CONFIG_NXTERM_NXKBDIN
           FAR struct nxtermioc_kbdin_s *kbdin =
             (FAR struct nxtermioc_kbdin_s *)((uintptr_t)arg);

           nxterm_kbdin(kbdin->handle, kbdin->buffer, kbdin->buflen);
           ret = OK;
#else
           ret = -ENOSYS;
#endif
         }
         break;

      /* CMD:           NXTERMIOC_NXTERM_RESIZE
       * DESCRIPTION:   Inform NxTerm keyboard the the size of the window has
       *                changed
       * ARG:           A reference readable instance of struct nxtermioc_resize_s
       * CONFIGURATION: CONFIG_NXTERM
       */

       case NXTERMIOC_NXTERM_RESIZE:
         {
           FAR struct nxtermioc_resize_s *resize =
             (FAR struct nxtermioc_resize_s *)((uintptr_t)arg);

           ret = nxterm_resize(resize->handle, &resize->size);
         }
         break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
