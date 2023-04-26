/****************************************************************************
 * drivers/video/fb.c
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

/* Framebuffer character driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/video/fb.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines one framebuffer device.  Note that which is
 * everything in this structure is constant data set up and initialization
 * time.  Therefore, no there is requirement for serialized access to this
 * structure.
 */

struct fb_chardev_s
{
  FAR struct fb_vtable_s *vtable; /* Framebuffer interface */
  FAR void *fbmem;                /* Start of frame buffer memory */
  FAR struct pollfd *fds;         /* Polling structure of waiting thread */
  size_t fblen;                   /* Size of the framebuffer */
  uint8_t plane;                  /* Video plan number */
  uint8_t bpp;                    /* Bits per pixel */
  volatile bool pollready;        /* Poll ready flag */
  clock_t vsyncoffset;            /* VSync offset ticks */
  struct wdog_s wdog;             /* VSync offset timer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t fb_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
static ssize_t fb_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static off_t   fb_seek(FAR struct file *filep, off_t offset, int whence);
static int     fb_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int     fb_mmap(FAR struct file *filep,
                       FAR struct mm_map_entry_s *map);
static int     fb_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fb_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  fb_read,       /* read */
  fb_write,      /* write */
  fb_seek,       /* seek */
  fb_ioctl,      /* ioctl */
  fb_mmap,       /* mmap */
  NULL,          /* truncate */
  fb_poll        /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fb_read
 ****************************************************************************/

static ssize_t fb_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;
  size_t start;
  size_t end;
  size_t size;

  ginfo("len: %u\n", (unsigned int)len);

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  /* Get the start and size of the transfer */

  start = filep->f_pos;
  if (start >= fb->fblen)
    {
      return 0;  /* Return end-of-file */
    }

  end = start + len;
  if (end >= fb->fblen)
    {
      end = fb->fblen;
    }

  size = end - start;

  /* And transfer the data from the frame buffer */

  memcpy(buffer, fb->fbmem + start, size);
  filep->f_pos += size;
  return size;
}

/****************************************************************************
 * Name: fb_write
 ****************************************************************************/

static ssize_t fb_write(FAR struct file *filep, FAR const char *buffer,
                        size_t len)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;
  size_t start;
  size_t end;
  size_t size;

  ginfo("len: %u\n", (unsigned int)len);

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  fb->pollready = false;

  /* Get the start and size of the transfer */

  start = filep->f_pos;
  if (start >= fb->fblen)
    {
      return -EFBIG;  /* Cannot extend the framebuffer */
    }

  end = start + len;
  if (end >= fb->fblen)
    {
      end = fb->fblen;
    }

  size = end - start;

  /* And transfer the data into the frame buffer */

  memcpy(fb->fbmem + start, buffer, size);
  filep->f_pos += size;
  return size;
}

/****************************************************************************
 * Name: fb_seek
 *
 * Description:
 *   Seek the logical file pointer to the specified position.  The offset
 *   is in units of pixels, with offset zero being the beginning of the
 *   framebuffer.
 *
 ****************************************************************************/

static off_t fb_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;
  off_t newpos;
  int ret;

  ginfo("offset: %u whence: %d\n", (unsigned int)offset, whence);

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = fb->fblen + offset;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *   end of the existing data in the file. If data is later written at this
   *   point, subsequent reads of data in the gap shall return bytes with the
   *   value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second.  Return EINVAL if
   *  "...the resulting file offset would be negative for a regular file,
   *   block special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: fb_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int fb_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;
  int ret;

  ginfo("cmd: %d arg: %ld\n", cmd, arg);

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  /* Process the IOCTL command */

  switch (cmd)
    {
      case FBIOGET_VIDEOINFO:  /* Get color plane info */
        {
          FAR struct fb_videoinfo_s *vinfo =
            (FAR struct fb_videoinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(vinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->getvideoinfo != NULL);
          ret = fb->vtable->getvideoinfo(fb->vtable, vinfo);
        }
        break;

      case FBIOGET_PLANEINFO:  /* Get video plane info */
        {
          FAR struct fb_planeinfo_s *pinfo =
            (FAR struct fb_planeinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(pinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->getplaneinfo != NULL);
          ret = fb->vtable->getplaneinfo(fb->vtable, fb->plane, pinfo);
        }
        break;

#ifdef CONFIG_FB_CMAP
      case FBIOGET_CMAP:       /* Get RGB color mapping */
        {
          FAR struct fb_cmap_s *cmap =
            (FAR struct fb_cmap_s *)((uintptr_t)arg);

          DEBUGASSERT(cmap != 0 && fb->vtable != NULL &&
                      fb->vtable->getcmap != NULL);
          ret = fb->vtable->getcmap(fb->vtable, cmap);
        }
        break;

      case FBIOPUT_CMAP:       /* Put RGB color mapping */
        {
          FAR const struct fb_cmap_s *cmap =
            (FAR struct fb_cmap_s *)((uintptr_t)arg);

          DEBUGASSERT(cmap != 0 && fb->vtable != NULL &&
                      fb->vtable->putcmap != NULL);
          ret = fb->vtable->putcmap(fb->vtable, cmap);
        }
        break;
#endif
#ifdef CONFIG_FB_HWCURSOR
      case FBIOGET_CURSOR:     /* Get cursor attributes */
        {
          FAR struct fb_cursorattrib_s *attrib =
            (FAR struct fb_cursorattrib_s *)((uintptr_t)arg);

          DEBUGASSERT(attrib != 0 && fb->vtable != NULL &&
                      fb->vtable->getcursor != NULL);
          ret = fb->vtable->getcursor(fb->vtable, attrib);
        }
        break;

      case FBIOPUT_CURSOR:     /* Set cursor attributes */
        {
          FAR struct fb_setcursor_s *cursor =
            (FAR struct fb_setcursor_s *)((uintptr_t)arg);

          DEBUGASSERT(cursor != 0 && fb->vtable != NULL &&
                      fb->vtable->setcursor != NULL);
          ret = fb->vtable->setcursor(fb->vtable, cursor);
        }
        break;
#endif

#ifdef CONFIG_FB_UPDATE
      case FBIO_UPDATE:  /* Update the modified framebuffer data  */
        {
          struct fb_area_s *area = (FAR struct fb_area_s *)((uintptr_t)arg);

          DEBUGASSERT(fb->vtable != NULL && fb->vtable->updatearea != NULL);
          ret = fb->vtable->updatearea(fb->vtable, area);
        }
        break;
#endif

#ifdef CONFIG_FB_SYNC
      case FBIO_WAITFORVSYNC:  /* Wait upon vertical sync */
        {
          ret = fb->vtable->waitforvsync(fb->vtable);
        }
        break;
#endif

#ifdef CONFIG_FB_OVERLAY
      case FBIO_SELECT_OVERLAY:  /* Select video overlay */
        {
          struct fb_overlayinfo_s oinfo;

          DEBUGASSERT(fb->vtable != NULL &&
                      fb->vtable->getoverlayinfo != NULL);
          memset(&oinfo, 0, sizeof(oinfo));
          ret = fb->vtable->getoverlayinfo(fb->vtable, arg, &oinfo);
          if (ret == OK)
            {
              fb->fbmem = oinfo.fbmem;
              fb->fblen = oinfo.fblen;
              fb->bpp   = oinfo.bpp;
            }
        }
        break;

      case FBIOGET_OVERLAYINFO:  /* Get video overlay info */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->getoverlayinfo != NULL);
          ret = fb->vtable->getoverlayinfo(fb->vtable,
                                           oinfo->overlay, oinfo);
        }
        break;

      case FBIOSET_TRANSP:  /* Set video overlay transparency */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->settransp != NULL);
          ret = fb->vtable->settransp(fb->vtable, oinfo);
        }
        break;

      case FBIOSET_CHROMAKEY:  /* Set video overlay chroma key */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->setchromakey != NULL);
          ret = fb->vtable->setchromakey(fb->vtable, oinfo);
        }
        break;

      case FBIOSET_COLOR:  /* Set video overlay color */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->setcolor != NULL);
          ret = fb->vtable->setcolor(fb->vtable, oinfo);
        }
        break;

      case FBIOSET_BLANK:  /* Blank or unblank video overlay */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->setblank != NULL);
          ret = fb->vtable->setblank(fb->vtable, oinfo);
        }
        break;

      case FBIOSET_AREA:  /* Set active video overlay area */
        {
          FAR struct fb_overlayinfo_s *oinfo =
            (FAR struct fb_overlayinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(oinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->setarea != NULL);
          ret = fb->vtable->setarea(fb->vtable, oinfo);
        }
        break;

#ifdef CONFIG_FB_OVERLAY_BLIT
      case FBIOSET_BLIT:  /* Blit operation between video overlays */
        {
          FAR struct fb_overlayblit_s *blit =
            (FAR struct fb_overlayblit_s *)((uintptr_t)arg);

          DEBUGASSERT(blit != 0 && fb->vtable != NULL &&
                      fb->vtable->blit != NULL);
          ret = fb->vtable->blit(fb->vtable, blit);
        }
        break;

      case FBIOSET_BLEND:  /* Blend operation between video overlays */
        {
          FAR struct fb_overlayblend_s *blend =
            (FAR struct fb_overlayblend_s *)((uintptr_t)arg);

          DEBUGASSERT(blend != 0 && fb->vtable != NULL &&
                      fb->vtable->blend != NULL);
          ret = fb->vtable->blend(fb->vtable, blend);
        }
        break;
#endif
#endif /* CONFIG_FB_OVERLAY */

      case FBIOSET_POWER:
        {
          DEBUGASSERT(fb->vtable != NULL &&
                      fb->vtable->setpower != NULL);
          ret = fb->vtable->setpower(fb->vtable, (int)arg);
        }
        break;

      case FBIOGET_POWER:
        {
          FAR int *power = (FAR int *)((uintptr_t)arg);

          DEBUGASSERT(power != NULL && fb->vtable != NULL &&
                      fb->vtable->getpower != NULL);
          *(power) = fb->vtable->getpower(fb->vtable);
          ret = OK;
        }
        break;

      case FBIOGET_FRAMERATE:
        {
          FAR int *rate = (FAR int *)((uintptr_t)arg);

          DEBUGASSERT(rate != NULL && fb->vtable != NULL &&
                      fb->vtable->getframerate != NULL);
          *(rate) = fb->vtable->getframerate(fb->vtable);
          ret = OK;
        }
        break;

      case FBIOSET_FRAMERATE:
        {
          DEBUGASSERT(fb->vtable != NULL &&
                      fb->vtable->setframerate != NULL);
          ret = fb->vtable->setframerate(fb->vtable, (int)arg);
        }
        break;

      case FBIOPAN_DISPLAY:
        {
          FAR struct fb_planeinfo_s *pinfo =
            (FAR struct fb_planeinfo_s *)((uintptr_t)arg);

          DEBUGASSERT(pinfo != NULL && fb->vtable != NULL &&
                      fb->vtable->pandisplay != NULL);
          ret = fb->vtable->pandisplay(fb->vtable, pinfo);
          fb->pollready = false;
        }
        break;

      case FBIO_CLEARNOTIFY:
        {
          fb->pollready = false;
          ret = OK;
        }
        break;

      case FBIOSET_VSYNCOFFSET:
        {
          fb->vsyncoffset = USEC2TICK(arg);
          ret = OK;
        }
        break;

      case FBIOGET_VSCREENINFO:
        {
          struct fb_videoinfo_s vinfo;
          struct fb_planeinfo_s pinfo;
          FAR struct fb_var_screeninfo *varinfo =
            (FAR struct fb_var_screeninfo *)((uintptr_t)arg);

          DEBUGASSERT(varinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->getvideoinfo != NULL &&
                      fb->vtable->getplaneinfo != NULL);
          ret = fb->vtable->getvideoinfo(fb->vtable, &vinfo);
          if (ret < 0)
            {
              break;
            }

          memset(&pinfo, 0, sizeof(pinfo));
          ret = fb->vtable->getplaneinfo(fb->vtable, fb->plane, &pinfo);
          if (ret < 0)
            {
              break;
            }

          memset(varinfo, 0, sizeof(struct fb_var_screeninfo));
          varinfo->xres           = vinfo.xres;
          varinfo->yres           = vinfo.yres;
          varinfo->xres_virtual   = pinfo.xres_virtual;
          varinfo->yres_virtual   = pinfo.yres_virtual;
          varinfo->xoffset        = pinfo.xoffset;
          varinfo->yoffset        = pinfo.yoffset;
          varinfo->bits_per_pixel = pinfo.bpp;
          varinfo->grayscale      = FB_ISMONO(vinfo.fmt);
          switch (vinfo.fmt)
            {
              case FB_FMT_Y1:
                varinfo->red.offset    = 0;
                varinfo->green.offset  = 0;
                varinfo->blue.offset   = 0;
                varinfo->red.length    = 1;
                varinfo->green.length  = 1;
                varinfo->blue.length   = 1;
                break;

              case FB_FMT_Y8:
                varinfo->red.offset    = 0;
                varinfo->green.offset  = 0;
                varinfo->blue.offset   = 0;
                varinfo->red.length    = 8;
                varinfo->green.length  = 8;
                varinfo->blue.length   = 8;
                break;

              case FB_FMT_RGB16_555:
                varinfo->red.offset    = 10;
                varinfo->green.offset  = 5;
                varinfo->blue.offset   = 0;
                varinfo->red.length    = 5;
                varinfo->green.length  = 5;
                varinfo->blue.length   = 5;
                break;

              case FB_FMT_RGB16_565:
                varinfo->red.offset    = 11;
                varinfo->green.offset  = 5;
                varinfo->blue.offset   = 0;
                varinfo->red.length    = 5;
                varinfo->green.length  = 6;
                varinfo->blue.length   = 5;
                break;

              case FB_FMT_RGB24:
              case FB_FMT_RGB32:
                varinfo->red.offset    = 16;
                varinfo->green.offset  = 8;
                varinfo->blue.offset   = 0;
                varinfo->red.length    = 8;
                varinfo->green.length  = 8;
                varinfo->blue.length   = 8;
                break;

              case FB_FMT_RGBA32:
                varinfo->red.offset    = 16;
                varinfo->green.offset  = 8;
                varinfo->blue.offset   = 0;
                varinfo->transp.offset = 24;
                varinfo->red.length    = 8;
                varinfo->green.length  = 8;
                varinfo->blue.length   = 8;
                varinfo->transp.length = 8;
                break;
          }
        }
        break;

      case FBIOGET_FSCREENINFO:
        {
          struct fb_videoinfo_s vinfo;
          struct fb_planeinfo_s pinfo;
          FAR struct fb_fix_screeninfo *fixinfo =
            (FAR struct fb_fix_screeninfo *)((uintptr_t)arg);

          DEBUGASSERT(fixinfo != 0 && fb->vtable != NULL &&
                      fb->vtable->getvideoinfo != NULL &&
                      fb->vtable->getplaneinfo != NULL);
          ret = fb->vtable->getvideoinfo(fb->vtable, &vinfo);
          if (ret < 0)
            {
              break;
            }

          memset(&pinfo, 0, sizeof(pinfo));
          ret = fb->vtable->getplaneinfo(fb->vtable, fb->plane, &pinfo);
          if (ret < 0)
            {
              break;
            }

          memset(fixinfo, 0, sizeof(struct fb_fix_screeninfo));
#ifdef CONFIG_FB_MODULEINFO
          strlcpy(fixinfo->id, (FAR const char *)vinfo.moduleinfo,
                  sizeof(fixinfo->id));
#endif
          fixinfo->smem_start  = (unsigned long)pinfo.fbmem;
          fixinfo->smem_len    = pinfo.fblen;
          fixinfo->type        = FB_ISYUVPLANAR(vinfo.fmt) ?
                                 FB_TYPE_INTERLEAVED_PLANES :
                                 FB_TYPE_PACKED_PIXELS;
          fixinfo->visual      = FB_ISMONO(vinfo.fmt) ?
                                 FB_VISUAL_MONO10 : FB_VISUAL_TRUECOLOR;
          fixinfo->line_length = pinfo.stride;
        }
        break;

      default:
        if (fb->vtable->ioctl != NULL)
          {
            ret = fb->vtable->ioctl(fb->vtable, cmd, arg);
          }
        else
          {
            gerr("ERROR: Unsupported IOCTL command: %d\n", cmd);
            ret = -ENOTTY;
          }
        break;
    }

  return ret;
}

static int fb_mmap(FAR struct file *filep, FAR struct mm_map_entry_s *map)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;
  int ret = -EINVAL;

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  /* Return the address corresponding to the start of frame buffer. */

  if (map->offset >= 0 && map->offset < fb->fblen &&
      map->length && map->offset + map->length <= fb->fblen)
    {
      map->vaddr = (FAR char *)fb->fbmem + map->offset;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: fb_poll
 *
 * Description:
 *   Wait for framebuffer to be writable.
 *
 ****************************************************************************/

static int fb_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  FAR struct fb_chardev_s *fb;

  /* Get the framebuffer instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  fb    = (FAR struct fb_chardev_s *)inode->i_private;

  if (setup)
    {
      if (fb->fds == NULL)
        {
          fb->fds = fds;
          fds->priv = &fb->fds;
        }
      else
        {
          return -EBUSY;
        }

      if (fb->pollready)
        {
          poll_notify(&fb->fds, 1, POLLOUT);
        }
    }
  else if (fds->priv)
    {
      fb->fds = NULL;
      fds->priv = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: fb_do_pollnotify
 ****************************************************************************/

static void fb_do_pollnotify(wdparm_t arg)
{
  FAR struct fb_chardev_s *fb = (FAR struct fb_chardev_s *)arg;

  fb->pollready = true;

  /* Notify framebuffer is writable. */

  poll_notify(&fb->fds, 1, POLLOUT);
}

/****************************************************************************
 * Name: fb_pollnotify
 *
 * Description:
 *   Notify the waiting thread that the framebuffer can be written.
 *
 * Input Parameters:
 *   vtable - Pointer to framebuffer's virtual table.
 *
 ****************************************************************************/

void fb_pollnotify(FAR struct fb_vtable_s *vtable)
{
  FAR struct fb_chardev_s *fb;

  DEBUGASSERT(vtable != NULL);

  fb = vtable->priv;

  /* Prevent calling before getting the vtable. */

  if (fb == NULL)
    {
      return;
    }

  if (fb->vsyncoffset > 0)
    {
      wd_start(&fb->wdog, fb->vsyncoffset, fb_do_pollnotify, (wdparm_t)fb);
    }
  else
    {
      fb_do_pollnotify((wdparm_t)fb);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fb_register
 *
 * Description:
 *   Register the framebuffer character device at /dev/fbN where N is the
 *   display number if the devices supports only a single plane.  If the
 *   hardware supports multiple color planes, then the device will be
 *   registered at /dev/fbN.M where N is the again display number but M
 *   is the display plane.
 *
 * Input Parameters:
 *   display - The display number for the case of boards supporting multiple
 *             displays or for hardware that supports multiple
 *             layers (each layer is consider a display).  Typically zero.
 *   plane   - Identifies the color plane on hardware that supports separate
 *             framebuffer "planes" for each color component.
 *
 * Returned Value:
 *   Zero (OK) is returned success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int fb_register(int display, int plane)
{
  FAR struct fb_chardev_s *fb;
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;
#ifdef CONFIG_FB_OVERLAY
  struct fb_overlayinfo_s oinfo;
#endif
  char devname[16];
  int nplanes;
  int ret;

  /* Allocate a framebuffer state instance */

  fb = (FAR struct fb_chardev_s *)kmm_zalloc(sizeof(struct fb_chardev_s));
  if (fb == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the frame buffer device. */

  ret = up_fbinitialize(display);
  if (ret < 0)
    {
      gerr("ERROR: up_fbinitialize() failed for display %d: %d\n",
           display, ret);
      goto errout_with_fb;
    }

  DEBUGASSERT((unsigned)plane <= UINT8_MAX);
  fb->plane  = plane;

  fb->vtable = up_fbgetvplane(display, plane);
  if (fb->vtable == NULL)
    {
      gerr("ERROR: up_fbgetvplane() failed, vplane=%d\n", plane);
      goto errout_with_fb;
    }

  fb->vtable->priv = fb;

  /* Initialize the frame buffer instance. */

  DEBUGASSERT(fb->vtable->getvideoinfo != NULL);
  ret = fb->vtable->getvideoinfo(fb->vtable, &vinfo);
  if (ret < 0)
    {
      gerr("ERROR: getvideoinfo() failed: %d\n", ret);
      goto errout_with_fb;
    }

  nplanes = vinfo.nplanes;
  DEBUGASSERT(vinfo.nplanes > 0 && (unsigned)plane < vinfo.nplanes);

  DEBUGASSERT(fb->vtable->getplaneinfo != NULL);
  memset(&pinfo, 0, sizeof(pinfo));
  ret = fb->vtable->getplaneinfo(fb->vtable, plane, &pinfo);
  if (ret < 0)
    {
      gerr("ERROR: getplaneinfo() failed: %d\n", ret);
      goto errout_with_fb;
    }

  fb->fbmem  = pinfo.fbmem;
  fb->fblen  = pinfo.fblen;
  fb->bpp    = pinfo.bpp;

  /* Clear the framebuffer memory */

  memset(pinfo.fbmem, 0, pinfo.fblen);

#ifdef CONFIG_FB_OVERLAY
  /* Initialize first overlay but do not select */

  DEBUGASSERT(fb->vtable->getoverlayinfo != NULL);
  memset(&oinfo, 0, sizeof(oinfo));
  ret = fb->vtable->getoverlayinfo(fb->vtable, 0, &oinfo);
  if (ret < 0)
    {
      gerr("ERROR: getoverlayinfo() failed: %d\n", ret);
      goto errout_with_fb;
    }

  /* Clear the overlay memory. Necessary when plane 0 and overlay 0
   * different.
   */

  memset(oinfo.fbmem, 0, oinfo.fblen);
#endif

  /* Register the framebuffer device */

  if (nplanes < 2)
    {
      snprintf(devname, 16, "/dev/fb%d", display);
    }
  else
    {
      snprintf(devname, 16, "/dev/fb%d.%d", display, plane);
    }

  ret = register_driver(devname, &g_fb_fops, 0666, (FAR void *)fb);
  if (ret < 0)
    {
      gerr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_fb;
    }

  return OK;

errout_with_fb:
  kmm_free(fb);
  return ret;
}
