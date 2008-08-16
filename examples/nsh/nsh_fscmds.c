/****************************************************************************
 * examples/nsh/nsh_fscmds.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#if CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/stat.h>
# include <fcntl.h>
# if !defined(CONFIG_DISABLE_MOUNTPOINT)
#   ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
#     include <sys/mount.h>
#     include <nuttx/mkfatfs.h>
#   endif
#endif
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
#  ifdef CONFIG_EXAMPLES_NSH_FILEIOSIZE
#    if CONFIG_EXAMPLES_NSH_FILEIOSIZE > (PATH_MAX + 1)
#      define IOBUFFERSIZE CONFIG_EXAMPLES_NSH_FILEIOSIZE
#    else
#      define IOBUFFERSIZE (PATH_MAX + 1)
#    endif
#  else
#    define IOBUFFERSIZE 1024
#  endif
# else
#    define IOBUFFERSIZE (PATH_MAX + 1)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*direntry_handler_t)(FAR struct nsh_vtbl_s *, const char *, struct dirent *, void *);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common buffer for file I/O.  Note the use of this common buffer precludes
 * multiple copies of NSH running concurrently.  It should be allocated per
 * NSH instance and retained in the "vtbl" as is done for the telnet
 * connection.
 */

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
static int foreach_direntry(FAR struct nsh_vtbl_s *vtbl, const char *cmd, const char *dirpath,
                            direntry_handler_t handler, void *pvarg)
{
  DIR *dirp;
  int ret = OK;

  /* Trim trailing '/' from directory names */

#ifdef CONFIG_EXAMPLES_NSH_FULLPATH
  trim_dir(arg);
#endif

  /* Open the directory */

  dirp = opendir(dirpath);

  if (!dirp)
    {
      /* Failed to open the directory */

      nsh_output(vtbl, g_fmtnosuch, cmd, "directory", dirpath);
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

      if (handler(vtbl, dirpath, entryp, pvarg) <  0)
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
static int ls_handler(FAR struct nsh_vtbl_s *vtbl, const char *dirpath, struct dirent *entryp, void *pvarg)
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
          nsh_output(vtbl, g_fmtcmdfailed, "ls", "stat", NSH_ERRNO);
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

          nsh_output(vtbl, " %s", details);
        }

      if ((lsflags & LSFLAGS_SIZE) != 0)
        {
          nsh_output(vtbl, "%8d", buf.st_size);
        }
    }

  /* then provide the filename that is common to normal and verbose output */

#ifdef CONFIG_EXAMPLES_NSH_FULLPATH
  nsh_output(vtbl, " %s/%s", arg, entryp->d_name);
#else
  nsh_output(vtbl, " %s", entryp->d_name);
#endif

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      nsh_output(vtbl, "/\n");
    }
  else
    {
      nsh_output(vtbl, "\n");
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: ls_recursive
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_recursive(FAR struct nsh_vtbl_s *vtbl, const char *dirpath, struct dirent *entryp, void *pvarg)
{
  /* Is this entry a directory? */

  if (DIRENT_ISDIRECTORY(entryp->d_type))
    {
      /* Yes.. */

      char *newpath;
      newpath = getdirpath(dirpath, entryp->d_name);

      /* List the directory contents */

      nsh_output(vtbl, "%s:\n", newpath);
      foreach_direntry(vtbl, "ls", newpath, ls_handler, pvarg);

      /* Then recurse to list each directory within the directory */

      foreach_direntry(vtbl, "ls", newpath, ls_recursive, pvarg);
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
void cmd_cat(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char buffer[IOBUFFERSIZE];

  /* Open the file for reading */

  int fd = open(argv[1], O_RDONLY);
  if (fd < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
      return;
    }

  /* And just dump it byte for byte into stdout */

  for (;;)
    {
      int nbytesread = read(fd, buffer, IOBUFFERSIZE);

      /* Check for read errors */

      if (nbytesread < 0)
        {
          /* EINTR is not an error */

          if (errno != EINTR)
            {
              nsh_output(vtbl, g_fmtcmdfailed, argv[0], "read", NSH_ERRNO);
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
                      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "write", NSH_ERRNO);
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
void cmd_cp(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
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
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
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
              nsh_output(vtbl, g_fmtcmdoutofmemory, argv[0]);
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
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
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

              nsh_output(vtbl, g_fmtcmdfailed, argv[0], "read", NSH_ERRNO);
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

              nsh_output(vtbl, g_fmtcmdfailed, argv[0], "write", NSH_ERRNO);
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
void cmd_ls(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
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
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are one required arguments after the options */

  if (optind + 1 <  argc)
    {
      nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 1 >  argc)
    {
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return;
    }

  /* List the directory contents */

  nsh_output(vtbl, "%s:\n", argv[optind]);
  ret = foreach_direntry(vtbl, "ls", argv[optind], ls_handler, (void*)lsflags);
  if (ret == OK && (lsflags & LSFLAGS_RECURSIVE) != 0)
    {
      /* Then recurse to list each directory within the directory */

      ret = foreach_direntry(vtbl, "ls", argv[optind], ls_recursive, (void*)lsflags);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mkdir
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_mkdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int result = mkdir(argv[1], 0777);
  if ( result < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkdir", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mkfatfs
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_FAT)
void cmd_mkfatfs(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
  int result = mkfatfs(argv[1], &fmt);
  if ( result < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkfatfs", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mkfifo
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_mkfifo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int result = mkfifo(argv[1], 0777);
  if ( result < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkfifo", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
void cmd_mount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
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
            nsh_output(vtbl, g_fmtargrequired, argv[0]);
            return;

          case '?':
          default:
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            return;
        }
    }

  /* There are two required arguments after the options */

  if (optind + 2 <  argc)
    {
      nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
      return;
    }
  else if (optind + 2 >  argc)
    {
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return;
    }

  /* Perform the mount */
  result = mount(argv[optind], argv[optind+1], filesystem, 0, NULL);
  if ( result < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mount", NSH_ERRNO);
    }
}
#endif
#endif

/****************************************************************************
 * Name: cmd_rm
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_rm(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  if (unlink(argv[1]) < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "unlink", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_rmdir
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
void cmd_rmdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  if (rmdir(argv[1]) < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "rmdir", NSH_ERRNO);
    }
}
#endif

/****************************************************************************
 * Name: cmd_sh
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0
void cmd_sh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FILE *stream;
  char *buffer;
  char *pret;

  /* Get a reference to the common input buffer */

  buffer = nsh_linebuffer(vtbl);
  if (buffer)
    {
      stream = fopen(argv[1], "r");
      if (!stream)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "fopen", NSH_ERRNO);
          return;
        }

      do
        {
          /* Get the next line of input from the file*/

          fflush(stdout);
          pret = fgets(buffer, CONFIG_EXAMPLES_NSH_LINELEN, stream);
          if (pret)
            {
              /* Parse process the command.  NOTE:  this is recursive...
               * we got to cmd_sh via a call to nsh_parse.  So some
               * considerable amount of stack may be used.
               */

              (void)nsh_parse(vtbl, buffer);
            }
        }
      while(pret);
      fclose(stream);
    }
}
#endif

/****************************************************************************
 * Name: cmd_umount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
#ifdef CONFIG_FS_FAT /* Need at least one filesytem in configuration */
void cmd_umount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  /* Perform the umount */
  int result = umount(argv[1]);
  if ( result < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "umount", NSH_ERRNO);
    }
}
#endif
#endif
