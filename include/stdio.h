/************************************************************
 * stdio.h
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
 ************************************************************/

#ifndef __STDIO_H
#define __STDIO_H

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdarg.h>
#include <sched.h>
#include <semaphore.h>
#include <time.h>

#include <nuttx/fs.h>

/************************************************************
 * Definitions
 ************************************************************/

/* File System Definitions **********************************/

/* File system error values *********************************/

#define EOF        (-1)

/* File I/O constants ***************************************/

#define MAXNAMLEN  100
#define NAME_MAX   100
#define PATH_MAX   101
#define S_IFMT     0170000
#define S_IFIFO    0010000
#define S_IFCHR    0020000
#define S_IFDIR    0040000
#define S_IFBLK    0060000
#define S_IFREG    0100000
#define S_IFLNK    0120000
#define S_IFSOCK   0140000
#define SEEK_SET   0
#define SEEK_CUR   1
#define SEEK_END   2

/* Wind-River IOCTL constants */

#define FIOFLUSH         2
#define	FIONBIO         16
#define	FIOSELECT       28
#define FIOUNSELECT     29
#define FIOTRUNC        42
#define FIOHANDLETONAME 45
#define FIOLABELGET     33
#define SET_HIDDEN      0x1004
#define PHYS_BLK_IO     255

#define SECTOR_ALIGN_BYTES   512
#define FILE_BUF_SIZE        (4 * SECTOR_ALIGN_BYTES)
#define FILE_BUF_ALIGN_BYTES 16

/* File type code for the EntryType field in dirent struct */

#define FS_FILE_TYPE      0
#define FS_DIRECTORY_TYPE 1

/* The first three _iob entries are reserved for standard I/O */

#define stdin  (__stdfile(0))
#define stdout (__stdfile(1))
#define stderr (__stdfile(2))

/* These APIs are not implemented and/or can be synthesized from
 * supported APIs.
 */

#define putc(c,s)  fputc((c),(s))
#define putchar(c) fputc(c, stdout)
#define getc(s)    fgetc(s)
#define getchar()  fgetc(stdin)
#define ftell(s)   fseek((s),0,SEEK_CUR)
#define rewind(s)  ((void)fseek((s),0,SEEK_SET))
#define fsync(f)

/************************************************************
 * Public Type Definitions
 ************************************************************/

/* The POSIX specification requires that the caller of readdir_r
 * provide storage "large enough for a dirent with the d_name
 * member and an array of char containing at least {NAME_MAX}
 * plus one elements.  The legacy dirent structure does not
 * contain such an array.  The legacy dirent structure is
 * renamed _dirent below.
 */

struct _dirent
{
  char *d_name;               /* name of directory entry */
};
struct dirent
{
  char *d_name;               /* A pointer to szName */
  char szName[NAME_MAX+1];    /* name of the directory entry */
};

typedef struct
{
  unsigned char EntryType;    /* FS_FILE_TYPE or FS_DIRECTORY_TYPE */
  char szName[NAME_MAX];      /* name of the directory entry */
} fsdirent;

typedef struct
{
  unsigned long inode;
  int           generation;
  char         *fileName;
} HANDLE_TO_NAME_IOCTL;

struct stat
{
  dev_t          st_dev;     /* ID of device containing a */
                             /* directory entry for this file */
  ino_t          st_ino;     /* Inode number */
  unsigned short st_mode;    /* File type, attributes, and */
                             /* access control summary */
  unsigned short st_nlink;   /* Number of links */
  uid_t          st_uid;     /* User ID of file owner */
  gid_t          st_gid;     /* Group ID of file group */
  dev_t          st_rdev;    /* Device ID; this entry defined */
                             /* only for char or blk spec files */
  off_t          st_size;    /* File size (bytes) */
  time_t         st_atime;   /* Time of last access */
  time_t         st_mtime;   /* Last modification time */
  time_t         st_ctime;   /* Last file status change time */
                             /* Measured in secs since */
                             /* 00:00:00 GMT, Jan 1, 1970 */
  long          st_blksize;  /* Non-standard, Wind-River field */
  unsigned long st_blocks;   /* Non-standard, Wind-River field */
  long          st_gen;      /* file generation value: Non-standard, Wind-River field */
};

struct statfs
{
  long          f_bavail;    /* free blocks available to non-superuser */
  long          f_bfree;     /* free blocks */
  long          f_blocks;    /* total blocks in file system */
  long          f_bsize;     /* fundamental file system block (bytes) */
  long          f_ffree;     /* free file nodes in file system */
  long          f_files;     /* total file nodes in file system */
  long          f_type;      /* type of info, zero for now */
};

/* Streams */

typedef struct file_struct FILE;

typedef void DIR;

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Inline Functions
 ************************************************************/

/* Used to reference stdin, stdout, and stderr */

static inline FILE *__stdfile(int fd)
{
  if ((unsigned int)fd < CONFIG_NFILE_DESCRIPTORS)
    {
      struct streamlist *streams = sched_getstreams();
      if (streams)
        {
          return &streams->sl_streams[fd];
        }
    }
  return NULL;
}

/************************************************************
 * Public Function Prototypes
 ************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* ANSI-like File System Interfaces */

EXTERN int    fclose(FILE *stream);
EXTERN int    fflush(FILE *stream);
EXTERN int    feof(FILE *stream);
EXTERN int    ferror(FILE *stream);
EXTERN int    fgetc(FILE *stream);
EXTERN char  *fgets(char *s, int n, FILE *stream);
EXTERN FILE  *fopen(const char *path, const char *type);
EXTERN int    fprintf(FILE *stream, const char *format, ...);
EXTERN int    fputc(int c, FILE *stream);
EXTERN int    fputs(const char *s, FILE *stream);
EXTERN size_t fread(void *ptr, size_t size, size_t n_items,
		    FILE *stream);
EXTERN int    fseek(FILE *stream, long int offset, int whence);
EXTERN size_t fwrite(const void *ptr, size_t size,
		     size_t n_items, FILE *stream);
EXTERN int    printf(const char *format, ...);
EXTERN int    puts(const char *s);
EXTERN int    rename(const char *source, const char *target);
EXTERN int    sprintf(char *dest, const char *format, ...);
EXTERN int    ungetc(int c, FILE *stream);
EXTERN int    vprintf(const char *s, va_list ap);
EXTERN int    vfprintf(FILE *stream, const char *s, va_list ap);
EXTERN int    vsprintf(char *buf, const char *s, va_list ap);

/* POSIX-like File System Interfaces */

EXTERN int    chdir(const char *path);
EXTERN int    close(int fd);
EXTERN int    closedir(DIR *dirp);
EXTERN int    creat(const char *path, mode_t mode);
EXTERN FILE  *fdopen(int fd, const char *type);
EXTERN int    fstat(int fd, struct stat *buf);
EXTERN char  *getcwd(char *buf, size_t size);
EXTERN int    ioctl(int fd, int req, unsigned long arg);
EXTERN off_t  lseek(int fd, off_t offset, int whence);
EXTERN int    mkdir(const char *path, mode_t mode);
EXTERN int    open( const char *path, int oflag, ... );
EXTERN DIR   *opendir(const char *path);
EXTERN int    read(int fd, void *buf, unsigned int nbytes);
EXTERN struct _dirent *readdir(DIR *dirp);
EXTERN int    readdir_r(DIR *dirp, struct dirent *entry, struct dirent **result);
EXTERN void   rewinddir(DIR *dirp);
EXTERN int    rmdir(const char *path);
EXTERN void   seekdir(DIR *dirp, int loc);
EXTERN int    stat(const char *path, struct stat *buf);
EXTERN int    statfs(const char *path, struct statfs *buf);
EXTERN int    telldir(DIR *dirp);
EXTERN int    unlink(const char *path);
EXTERN int    write(int fd, const void *buf, unsigned int nbytes);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __STDIO_H */
