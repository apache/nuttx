/****************************************************************************
 * examples/romfs/romfs_main.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

/* Mount the ROMFS image, Verifty that it contains the
 * following:
 *
 * testdir
 *  |---------- [drwxr-xr-x      4096]  adir
 *  |   |------ [-rw-r--r--        21]  anotherfile.txt
 *  |   |------ [drwxr-xr-x      4096]  subdir
 *  |   |   `-- [-rw-r--r--        21]  subdirfile.txt
 *  |   `------ [-rw-r--r--        25]  yafile.txt
 *  |---------- [-rw-r--r--        15]  afile.txt
 *  |---------- [-rw-r--r--        21]  hfile
 *  `---------- [lrwxrwxrwx        11]  ldir -> adir/subdir
 *
 * testdir/ldir is a soft-link and should not be detectable.
 * hfile is a hardlink to subdirfile and should be identical
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>

#include <nuttx/ramdisk.h>

#include "romfs_testdir.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration settings */

#ifndef CONFIG_EXAMPLES_ROMFS_RAMDEVNO
#  define CONFIG_EXAMPLES_ROMFS_RAMDEVNO 1
#endif

#ifndef CONFIG_EXAMPLES_ROMFS_SECTORSIZE
#  define CONFIG_EXAMPLES_ROMFS_SECTORSIZE 64
#endif

#ifndef CONFIG_EXAMPLES_ROMFS_MOUNTPOINT
#  define CONFIG_EXAMPLES_ROMFS_MOUNTPOINT "/usr/local/share"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "Mountpoint support is disabled"
#endif

#if CONFIG_NFILE_DESCRIPTORS < 4
#  error "Not enough file descriptors"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "ROMFS support not enabled"
#endif

#define NSECTORS(b)        (((b)+CONFIG_EXAMPLES_ROMFS_SECTORSIZE-1)/CONFIG_EXAMPLES_ROMFS_SECTORSIZE)
#define STR_RAMDEVNO(m)    #m
#define MKMOUNT_DEVNAME(m) "/dev/ram" STR_RAMDEVNO(m)
#define MOUNT_DEVNAME      MKMOUNT_DEVNAME(CONFIG_EXAMPLES_ROMFS_RAMDEVNO)

/* Test directory stuff */

#define WRITABLE_MODE      (S_IWOTH|S_IWGRP|S_IWUSR)
#define READABLE_MODE      (S_IROTH|S_IRGRP|S_IRUSR)
#define EXECUTABLE_MODE    (S_IXOTH|S_IXGRP|S_IXUSR)

#define DIRECTORY_MODE     (S_IFDIR|READABLE_MODE|EXECUTABLE_MODE)
#define FILE_MODE          (S_IFREG|READABLE_MODE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct node_s
{
  struct node_s *peer;      /* Next node in this directory */
  boolean        directory; /* True: directory */
  boolean        found;     /* True: found and verified */
  const char    *name;      /* Node name */
  mode_t         mode;      /* Expected permissions */
  size_t         size;      /* Expected size */
  union
    {
      const char    *filecontent; /* Context of text file */
      struct node_s *child;       /* Subdirectory start */
    } u;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_afilecontent[]       = "This is a file";
static const char g_anotherfilecontent[] = "This is another file";
static const char g_yafilecontent[]      = "This is yet another file";
static const char g_subdirfilecontent[]  = "File in subdirectory";

#define g_hfilecontent g_subdirfilecontent

static struct node_s g_adir;
static struct node_s g_afile;
static struct node_s g_hfile;

static struct node_s g_anotherfile;
static struct node_s g_subdir;
static struct node_s g_yafile;

static struct node_s g_subdirfile;

static int g_nerrors = 0;

static char g_pathbuffer[1024];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 ****************************************************************************/

static void connectem(void)
{
  g_adir.peer                  = &g_afile;
  g_adir.directory             = TRUE;
  g_adir.found                 = FALSE;
  g_adir.name                  = "adir";
  g_adir.mode                  = DIRECTORY_MODE;
  g_adir.size                  = 0;
  g_adir.u.child               = &g_anotherfile;

  g_afile.peer                 = &g_hfile;
  g_afile.directory            = FALSE;
  g_afile.found                = FALSE;
  g_afile.name                 = "afile.txt";
  g_afile.mode                 = FILE_MODE;
  g_afile.size                 = strlen(g_afilecontent)+1;
  g_afile.u.filecontent        = g_afilecontent;

  g_hfile.peer                 = NULL;
  g_hfile.directory            = FALSE; /* Actually a hard link */
  g_hfile.found                = FALSE;
  g_hfile.name                 = "hfile";
  g_hfile.mode                 = FILE_MODE;
  g_hfile.size                 = strlen(g_hfilecontent)+1;
  g_hfile.u.filecontent        = g_hfilecontent;

  g_anotherfile.peer           = &g_yafile;
  g_anotherfile.directory      = FALSE;
  g_anotherfile.found          = FALSE;
  g_anotherfile.name           = "anotherfile.txt";
  g_anotherfile.mode           = FILE_MODE;
  g_anotherfile.size           = strlen(g_anotherfilecontent)+1;
  g_anotherfile.u.filecontent  = g_anotherfilecontent;

  g_yafile.peer                = &g_subdir;
  g_yafile.directory           = FALSE;
  g_yafile.found               = FALSE;
  g_yafile.name                = "yafile.txt";
  g_yafile.mode                = FILE_MODE;
  g_yafile.size                = strlen(g_yafilecontent)+1;
  g_yafile.u.filecontent       = g_yafilecontent;

  g_subdir.peer                = NULL;
  g_subdir.directory           = TRUE;
  g_subdir.found               = FALSE;
  g_subdir.name                = "subdir";
  g_subdir.mode                = DIRECTORY_MODE;
  g_subdir.size                = 0;
  g_subdir.u.child             = &g_subdirfile;

  g_subdirfile.peer            = NULL;
  g_subdirfile.directory       = FALSE;
  g_subdirfile.found           = FALSE;
  g_subdirfile.name            = "subdirfile.txt";
  g_subdirfile.mode            = FILE_MODE;
  g_subdirfile.size            = strlen(g_subdirfilecontent)+1;
  g_subdirfile.u.filecontent   = g_subdirfilecontent;
}

/****************************************************************************
 * Name: findindirectory
 ****************************************************************************/

static struct node_s *findindirectory(struct node_s *entry, const char *name)
{
  for (; entry; entry = entry->peer)
    {
      if (!entry->found && strcmp(entry->name, name) == 0)
        {
          entry->found = TRUE;
          return entry;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: checkattributes
 ****************************************************************************/

static void checkattributes(const char *path, mode_t mode, size_t size)
{
  struct stat buf;
  int ret;

  ret = stat(path, &buf);
  if (ret != 0)
    {
      printf("  -- ERROR: Failed to stat %s: %d\n", path, errno);
      g_nerrors++;
      return;
    }

  if (mode != buf.st_mode)
    {
      printf("  -- ERROR: Expected mode %08x, got %08x\n", mode, buf.st_mode);
      g_nerrors++;
    }

  if (size != buf.st_size)
    {
      printf("  -- ERROR: Expected size %d, got %d\n", mode, buf.st_size);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: readdirectories
 ****************************************************************************/

static void readdirectories(const char *path, struct node_s *entry)
{
  DIR *dirp;
  struct node_s *node;
  struct dirent *direntry;
  char *fullpath;

  printf("Traversing directory: %s\n", path);
  dirp = opendir(path);
  if (!dirp)
    {
      printf("  ERROR opendir(\"%s\") failed: %d\n", path, errno);
      g_nerrors++;
      return;
    }

  for (direntry = readdir(dirp); direntry; direntry = readdir(dirp))
    {
      if (strcmp(direntry->d_name, ".") == 0 || strcmp(direntry->d_name, "..") == 0)
        {
           printf("  Skipping %s\n", direntry->d_name);
           continue;
        }

      node = findindirectory(entry, direntry->d_name);
      if (!node)
        {
          printf("  ERROR: No node found for %s\n", direntry->d_name);
          g_nerrors++;
          continue;
        }

      /* Get the full path to the entry */

      sprintf(g_pathbuffer, "%s/%s", path, direntry->d_name);
      fullpath = strdup(g_pathbuffer);

      if (DIRENT_ISDIRECTORY(direntry->d_type))
        {
          printf("  DIRECTORY: %s/\n", fullpath);
          if (!node->directory)
            {
              printf("  -- ERROR: Expected type directory\n");
              g_nerrors++;
            }
          else
            {
              checkattributes(fullpath, node->mode, 0);
              readdirectories(fullpath, node->u.child);
              printf("Continuing directory: %s\n", path);
            }
        }
      else
        {
          printf("  FILE: %s/\n", fullpath);
          if (node->directory)
            {
              printf("  -- ERROR: Expected type file\n");
              g_nerrors++;
            }
          else
            {
              checkattributes(fullpath, node->mode, node->size);
            }
        }
      free(fullpath);
    }

  closedir(dirp);
}

/****************************************************************************
 * Name: checkdirectories
 ****************************************************************************/

static void checkdirectories(struct node_s *entry)
{
  for (; entry; entry = entry->peer)
    {
      if (!entry->found )
        {
          printf("ERROR: %s never found\n", entry->name);
          g_nerrors++;
        }

      if (entry->directory)
        {
          checkdirectories(entry->u.child);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_initialize
 ****************************************************************************/

void user_initialize(void)
{
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
   int  ret;

  /* Create a RAM disk for the test */

  ret = romdisk_register(CONFIG_EXAMPLES_ROMFS_RAMDEVNO, testdir_img, 
                         NSECTORS(testdir_img_len), CONFIG_EXAMPLES_ROMFS_SECTORSIZE);
  if (ret < 0)
    {
      printf("ERROR: Failed to create RAM disk\n");
      return 1;
    }

  /* Mount the test file system */

  printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
         CONFIG_EXAMPLES_ROMFS_MOUNTPOINT, MOUNT_DEVNAME);

  ret = mount(MOUNT_DEVNAME, CONFIG_EXAMPLES_ROMFS_MOUNTPOINT, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      printf("ERROR: Mount failed: %d\n", errno);
      return 1;
    }

  /* Perform the test */

  connectem();
  readdirectories(CONFIG_EXAMPLES_ROMFS_MOUNTPOINT, &g_adir);
  checkdirectories(&g_adir);

  if (g_nerrors)
    {
      printf("Finished with  %d errors\n", g_nerrors);
      return g_nerrors;
    }

  printf("PASSED\n");
  return 0;
}
