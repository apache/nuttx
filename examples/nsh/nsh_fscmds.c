/****************************************************************************
 * nsh_fscmds.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
#include <sys/types.h>

#if CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/stat.h>
# include <fcntl.h>
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/mount.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <dirent.h>
#include <limits.h>
#include <libgen.h>
#include <errno.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LSFLAGS_SIZE          1
#define LSFLAGS_LONG          2
#define LSFLAGS_RECURSIVE     4

/* The size of the I/O buffer may be specified in the
 * configs/<board-name>defconfig file -- provided that it is at least as
 * large as PATH_MAX.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0
#  ifdef CONFIG_NSH_IOBUFFERSIZE
#    if CONFIG_NSH_IOBUFFERSIZE > (PATH_MAX + 1)
#      define IOBUFFERSIZE CONFIG_NSH_IOBUFFERSIZE
#    else
#      define IOBUFFERSIZE (PATH_MAX + 1)
#    endif
#  else
#    define IOBUFFERSIZE      1024
#  endif
# else
#    define IOBUFFERSIZE (PATH_MAX + 1)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*direntry_handler_t)(FAR void *, const char *, struct dirent *, void *);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_iobuffer[IOBUFFERSIZE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trim_dir
 ****************************************************************************/

static void trim_dir(char *arg)
{
 /* Skip any trailing '/' characters (unless it is also the leading '/') */

 int len = strlen(arg) - 1;
 while (len > 0 && arg[len] == '/')
   {
      arg[len] = '\0';
      len--;
   }
}

/****************************************************************************
 * Name: getdirpath
 ****************************************************************************/

static char *getdirpath(const char *path, const char *file)
{
  /* Handle the case where all that is left is '/' */

  if (strcmp(path, "/") == 0)
    {
      sprintf(g_iobuffer, "/%s", file);
    }
  else
    {
      sprintf(g_iobuffer, "%s/%s", path, file);
    }

  g_iobuffer[PATH_MAX] = '\0';
  return strdup(g_iobuffer);
}

/****************************************************************************
 * Name: foreach_direntry
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int foreach_direntry(FAR void *handle, const char *cmd, const char *dirpath,
                            direntry_handler_t handler, void *pvarg)
{
  DIR *dirp;
  int ret = OK;

  /* Trim trailing '/' from directory names */

#ifdef CONFIG_FULL_PATH
  trim_dir(arg);
#endif

  /* Open the directory */

  dirp = opendir(dirpath);

  if (!dirp)
    {
      /* Failed to open the directory */

      nsh_output(handle, g_fmtnosuch, cmd, "directory", dirpath);
      return ERROR;
    }

  /* Read each directory entry */

  for (;;)
    {
      struct dirent *entryp = readdir(dirp);
      if (!entryp)
        {
          /* Finished with this directory */

          break;
        }

      /* Call the handler with this directory entry */

      if (handler(handle, dirpath, entryp, pvarg) <  0)
        {
          /* The handler reported a problem */

          ret = ERROR;
          break;
        }
    }

  closedir(dirp);
  return ret;
}
#endif

/****************************************************************************
 * Name: ls_handler
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_handler(FAR void *handle, const char *dirpath, struct dirent *entryp, void *pvarg)
{
  unsigned int lsflags = (unsigned int)pvarg;
  int ret;

  /* Check if any options will require that we stat the file */

  if ((lsflags & (LSFLAGS_SIZE|LSFLAGS_LONG)) != 0)
    {
      struct stat buf;
      char *fullpath = getdirpath(dirpath, entryp->d_name);

      /* Yes, stat the file */

      ret = stat(fullpath, &buf);
      free(fullpath);
      if (ret != 0)
        {
          nsh_output(handle, g_fmtcmdfailed, "ls", "stat", strerror(errno));
          return OK;
        }

      if ((lsflags & LSFLAGS_LONG) != 0)
        {
          char details[] = "----------";
          if (S_ISDIR(buf.st_mode))
            {
              details[0]='d';
            }
          else if (S_ISCHR(buf.st_mode))
            {
              details[0]='c';
            }
          else if (S_ISBLK(buf.st_mode))
            {
              details[0]='b';
            }

          if ((buf.st_mode & S_IRUSR) != 0)
            {
              details[1]='r';
            }

          if ((buf.st_mode & S_IWUSR) != 0)
            {
              details[2]='w';
            }

          if ((buf.st_mode & S_IXUSR) != 0)
            {
              details[3]='x';
            }

          if ((buf.st_mode & S_IRGRP) != 0)
            {
              details[4]='r';
            }

          if ((buf.st_mode & S_IWGRP) != 0)
            {
              details[5]='w';
            }

          if ((buf.st_mode & S_IXGRP) != 0)
            {
              details[6]='x';
            }

          if ((buf.st_mode & S_IROTH) != 0)
            {
              details[7]='r';
            }

          if ((buf.st_mode & S_IWOTH) != 0)
            {
              details[8]='w';
            }

          if ((buf.st_mode & S_IXOTH) != 0)
            {
              details[9]='x';
            }

          nsh_output(handle, " %s", details);
        }

      if ((lsflags & LSFLAGS_SIZE) != 0)
        {
          nsh_output(handle, "%8d", buf.st_size);
        }
    }

  /* then provide the filename that is common to normal and verbose output */

#ifdef CONFIG_FULL_PATH
  nsh_output(handle, " %s/%s", arg, entryp->d_name);
#else
  nsh_output(handle, " %s", entryp->d_name);
#endif

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      nsh_output(handle, "/\n");
    }
  else
    {
      nsh_output(handle, "\n");
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: ls_recursive
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_recursive(FAR void *handle, const char *dirpath, struct dirent *entryp, void *pvarg)
{
  /* Is this entry a directory? */

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      /* Yes.. */

      char *newpath;
      newpath = getdirpath(dirpath, entryp->d_name);

      /* List the directory contents */

      nsh_output(handle, "%s:\n", newpath);
      foreach_direntry(handle, "ls", newpath, ls_handler, pvarg);

      /* Then recurse to list each directory within the directory */

      foreach_direntry(handle, "ls", newpath, ls_recursive, pvarg);
      free(newpath);
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_cat
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
void cmd_cat(FAR void *handle, int argc, char **argv)
{
  char buffer[1024];

  /* Open the file for reading */

  int fd = open(argv[1], O_RDONLY);
  if (fd < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "open", strerror(errno));
      return;
    }

  /* And just dump it byte for byte into stdout */

  for (;;)
    {
      int nbytesread = read(fd, buffer, 1024);

      /* Check for read errors */

      if (nbytesread < 0)
        {
          /* EINTR is not an error */

          if (errno != EINTR)
            {
              nsh_output(handle, g_fmtcmdfailed, argv[0], "read", strerror(errno));
              break;
            }
        }

      /* Check for data successfully read */

      else if (nbytesread > 0)
        {
          int nbyteswritten = 0;

          while (nbyteswritten < nbytesread)
            {
              int n = write(1, buffer, nbytesread);
              if (n < 0)
                {
                  /* EINTR is not an error */

                  if (errno != EINTR)
                    {
                      nsh_output(handle, g_fmtcmdfailed, argv[0], "write", strerror(errno));
                      break;
                    }
                }
              else
                {
                  nbyteswritten += n;
                }
            }
        }

      /* Otherwise, it is the end of file */

      else
        {
          break;
        }
    }

  (void)close(fd);
}
#endif

/****************************************************************************
 * Name: cmd_cp
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
void cmd_cp(FAR void *handle, int argc, char **argv)
{
  struct stat buf;
  char *fullpath = NULL;
  const char *wrpath = argv[2];
  int oflags = O_WRONLY|O_CREAT|O_TRUNC;
  int rdfd;
  int wrfd;
  int ret;

  /* Open the source file for reading */

  rdfd = open(argv[1], O_RDONLY);
  if (rdfd < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "open", strerror(errno));
      return;
    }

  /* Check if the destination is a directory */

  ret = stat(wrpath, &buf);
  if (ret == 0)
    {
      /* Something exists here... is it a directory? */

      if (S_ISDIR(buf.st_mode))
        {
          /* Yes, it is a directory. Remove any trailing '/' characters from the path */

          trim_dir(argv[2]);

          /* Construct the full path to the new file */

          fullpath = getdirpath(argv[2], basename(argv[1]) );
          if (!fullpath)
            {
              nsh_output(handle, g_fmtcmdoutofmemory, argv[0]);
              goto out_with_rdfd;
            }

          /* Open then fullpath for writing */
          wrpath = fullpath;
        }
      else if (!S_ISREG(buf.st_mode))
        {
          /* Maybe it is a driver? */

          oflags = O_WRONLY;
        }
    }

  /* Now open the destination */

  wrfd = open(wrpath, oflags, 0666);
  if (wrfd < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "open", strerror(errno));
      goto out_with_fullpath;
    }

  /* Now copy the file */

  for (;;)
    {
      int nbytesread;
      int nbyteswritten;

      do
        {
          nbytesread = read(rdfd, g_iobuffer, IOBUFFERSIZE);
          if (nbytesread == 0)
            {
              /* End of file */

              goto out_with_wrfd;
            }
          else if (nbytesread < 0 && errno != EINTR)
            {
              /* Read error */

              nsh_output(handle, g_fmtcmdfailed, argv[0], "read", strerror(errno));
              goto out_with_wrfd;
            }
        }
      while (nbytesread <= 0);

      do
        {
          nbyteswritten = write(wrfd, g_iobuffer, nbytesread);
          if (nbyteswritten >= 0)
            {
              nbytesread -= nbyteswritten;
            }
          else if (errno != EINTR)
            {
              /* Read error */

              nsh_output(handle, g_fmtcmdfailed, argv[0], "write", strerror(errno));
              goto out_with_wrfd;
            }
        }
      while (nbytesread > 0);
    }

out_with_wrfd:
  close(wrfd);

out_with_fullpath:
  if (fullpath)
    {
      free(fullpath);
    }

out_with_rdfd:
  close(rdfd);
}
#endif

/****************************************************************************
 * Name: cmd_ls
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
void cmd_ls(FAR void *handle, int argc, char **argv)
{
  unsigned int lsflags = 0;
  int ret;

  /* Get the ls options */

  int option;
  while ((option = getopt(argc, argv, "lRs")) != ERROR)
    {
      switch (option)
        {
          case 'l':
            lsflags |= (LSFLAGS_SIZE|LSFLAGS_LONG);
            break;

          case 'R':
            lsflags |= LSFLAGS_RECURSIVE;
            break;

          case 's':
            lsflags |= LSFLAGS_SIZE;
            break;

          case '?':
          default:
            nsh_output(handle, g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are one required arguments after the options */

  if (optind + 1 <  argc)
    {
      nsh_output(handle, g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 1 >  argc)
    {
      nsh_output(handle, g_fmtargrequired, argv[0]);
      return;
    }

  /* List the directory contents */

  nsh_output(handle, "%s:\n", argv[optind]);
  ret = foreach_direntry(handle, "ls", argv[optind], ls_handler, (void*)lsflags);
  if (ret == OK && (lsflags & LSFLAGS_RECURSIVE) != 0)
    {
      /* Then recurse to list each directory within the directory */

      ret = foreach_direntry(handle, "ls", argv[optind], ls_recursive, (void*)lsflags);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mkdir
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_mkdir(FAR void *handle, int argc, char **argv)
{
  int result = mkdir(argv[1], 0777);
  if ( result < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "mkdir", strerror(errno));
    }
}
#endif

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_mount(FAR void *handle, int argc, char **argv)
{
  char *filesystem = 0;
  int result;

  /* Get the mount options */

  int option;
  while ((option = getopt(argc, argv, ":t:")) != ERROR)
    {
      switch (option)
        {
          case 't':
            filesystem = optarg;
            break;

          case ':':
            nsh_output(handle, g_fmtargrequired, argv[0]);
            return;

          case '?':
          default:
            nsh_output(handle, g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are two required arguments after the options */

  if (optind + 2 <  argc)
    {
      nsh_output(handle, g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 2 >  argc)
    {
      nsh_output(handle, g_fmtargrequired, argv[0]);
      return;
    }

  /* Perform the mount */
  result = mount(argv[optind], argv[optind+1], filesystem, 0, NULL);
  if ( result < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "mount", strerror(errno));
    }
}
#endif

/****************************************************************************
 * Name: cmd_rm
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_rm(FAR void *handle, int argc, char **argv)
{
  if (unlink(argv[1]) < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "unlink", strerror(errno));
    }
}
#endif

/****************************************************************************
 * Name: cmd_rm
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_rmdir(FAR void *handle, int argc, char **argv)
{
  if (rmdir(argv[1]) < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "rmdir", strerror(errno));
    }
}
#endif

/****************************************************************************
 * Name: cmd_umount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_umount(FAR void *handle, int argc, char **argv)
{
  /* Perform the umount */
  int result = umount(argv[1]);
  if ( result < 0)
    {
      nsh_output(handle, g_fmtcmdfailed, argv[0], "umount", strerror(errno));
    }
}
#endif
