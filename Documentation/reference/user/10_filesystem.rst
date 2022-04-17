======================
File System Interfaces
======================

.. _file_system_overview:

NuttX File System Overview
==========================

**Overview**. NuttX includes an optional, scalable file system. This
file-system may be omitted altogether; NuttX does not depend on the
presence of any file system.

**Pseudo Root File System**. A simple *in-memory*, *pseudo* file system
can be enabled by default. This is an *in-memory* file system because it
does not require any storage medium or block driver support. Rather,
file system contents are generated on-the-fly as referenced via standard
file system operations (open, close, read, write, etc.). In this sense,
the file system is *pseudo* file system (in the same sense that the
Linux ``/proc`` file system is also referred to as a pseudo file
system).

Any user supplied data or logic can be accessed via the pseudo-file
system. Built in support is provided for character and block
:ref:`driver <drivers-porting>` *nodes* in the any
pseudo file system directory. (By convention, however, all driver nodes
should be in the ``/dev`` pseudo file system directory).

**Mounted File Systems** The simple in-memory file system can be
extended my mounting block devices that provide access to true file
systems backed up via some mass storage device. NuttX supports the
standard ``mount()`` command that allows a block driver to be bound to a
mount-point within the pseudo file system and to a a file system. At
present, NuttX supports only the VFAT file system.

**Comparison to Linux** From a programming perspective, the NuttX file
system appears very similar to a Linux file system. However, there is a
fundamental difference: The NuttX root file system is a pseudo file
system and true file systems may be mounted in the pseudo file system.
In the typical Linux installation by comparison, the Linux root file
system is a true file system and pseudo file systems may be mounted in
the true, root file system. The approach selected by NuttX is intended
to support greater scalability from the very tiny platform to the
moderate platform.

**File System Interfaces**. The NuttX file system simply supports a set
of standard, file system APIs (``open()``, ``close()``, ``read()``,
``write``, etc.) and a registration mechanism that allows devices
drivers to a associated with *nodes* in a file-system-like name space.

Driver Operations
=================

``fcntl.h``
-----------

.. c:function:: int creat(FAR const char *path, mode_t mode);

.. c:function:: int open(FAR const char *path, int oflag, ...);

.. c:function:: int fcntl(int fd, int cmd, ...);

``unistd.h``
------------

.. c:function:: int     close(int fd);
.. c:function:: int     dup(int fd);
.. c:function:: int     dup2(int fd1, int fd2);
.. c:function:: off_t   lseek(int fd, off_t offset, int whence);
.. c:function:: ssize_t pread(int fd, void *buf, size_t nbytes, off_t offset);
.. c:function:: ssize_t pwrite(int fd, const void *buf, size_t nbytes, off_t offset);
.. c:function:: ssize_t read(int fd, void *buf, size_t nbytes);
.. c:function:: int     unlink(const char *path);
.. c:function:: ssize_t write(int fd, const void *buf, size_t nbytes);

``sys/ioctl.h``
---------------

.. c:function:: int ioctl(int fd, int req, ...)

``poll.h``
----------

.. c:function:: int poll(struct pollfd *fds, nfds_t nfds, int timeout)

  Waits for one of a set of file descriptors
  to become ready to perform I/O. If none of the events requested (and no
  error) has occurred for any of the file descriptors, then ``poll()``
  blocks until one of the events occurs.

  **Configuration Settings**. In order to use the select with TCP/IP
  sockets test, you must have the following things selected in your NuttX
  configuration file:

    -  ``CONFIG_NET`` Defined for general network support
    -  ``CONFIG_NET_TCP`` Defined for TCP/IP support

  In order to for select to work with incoming connections, you must also
  select:

    -  ``CONFIG_NET_TCPBACKLOG`` Incoming connections pend in a backlog
       until ``accept()`` is called. The size of the backlog is selected
       when ``listen()`` is called.

  :param fds: List of structures describing file descriptors to be
    monitored.
  :param nfds: The number of entries in the list.
  :param timeout: Specifies an upper limit on the time for which
    ``poll()`` will block in milliseconds. A negative value of
    ``timeout`` means an infinite timeout.

  :return:
    On success, the number of structures that have nonzero ``revents``
    fields. A value of 0 indicates that the call timed out and no file
    descriptors were ready. On error, -1 is returned, and ``errno`` is set
    appropriately:

    -  ``EBADF``. An invalid file descriptor was given in one of the sets.
    -  ``EFAULT``. The fds address is invalid
    -  ``EINTR``. A signal occurred before any requested event.
    -  ``EINVAL``. The nfds value exceeds a system limit.
    -  ``ENOMEM``. There was no space to allocate internal data structures.
    -  ``ENOSYS``. One or more of the drivers supporting the file descriptor
       does not support the poll method.

``sys/select.h``
----------------

.. c:function:: int select(int nfds, FAR fd_set *readfds, FAR fd_set *writefds, \
           FAR fd_set *exceptfds, FAR struct timeval *timeout)

  Allows a program to monitor multiple file
  descriptors, waiting until one or more of the file descriptors become
  "ready" for some class of I/O operation (e.g., input possible). A file
  descriptor is considered ready if it is possible to perform the
  corresponding I/O operation (e.g., read(2)) without blocking.

  **NOTE:** ```poll()`` <#poll>`__ is the fundamental API for performing
  such monitoring operation under NuttX. ``select()`` is provided for
  compatibility and is simply a layer of added logic on top of ``poll()``.
  As such, ``select()`` is more wasteful of resources and
  ```poll()`` <#poll>`__ is the recommended API to be used.

  :param nfds: the maximum file descriptor number (+1) of any descriptor
     in any of the three sets.
  :param readfds: the set of descriptions to monitor for read-ready events
  :param writefds: the set of descriptions to monitor for write-ready
     events
  :param exceptfds: the set of descriptions to monitor for error events
  :param timeout: Return at this time if none of these events of interest
     occur.

  :return:
    -  ``0:`` Timer expired
    -  ``>0:`` The number of bits set in the three sets of descriptors
    -  ``-1:`` An error occurred (``errno`` will be set appropriately, see
       ```poll()`` <#poll>`__).

Directory Operations (``dirent.h``)
-----------------------------------

.. c:function:: int        closedir(DIR *dirp);

.. c:function:: FAR DIR   *opendir(const char *path);

.. c:function:: FAR struct dirent *readdir(FAR DIR *dirp);

.. c:function:: int        readdir_r(FAR DIR *dirp, FAR struct dirent *entry, FAR struct dirent **result);

.. c:function:: void       rewinddir(FAR DIR *dirp);

.. c:function:: void       seekdir(FAR DIR *dirp, int loc);

.. c:function:: int        telldir(FAR DIR *dirp);

UNIX Standard Operations (``unistd.h``)
---------------------------------------

.. code-block:: c

  #include <unistd.h>

  /* Task Control Interfaces */

  pid_t   vfork(void);
  pid_t   getpid(void);
  void    _exit(int status) noreturn_function;
  unsigned int sleep(unsigned int seconds);
  void    usleep(unsigned long usec);
  int     pause(void);

  /* File descriptor operations */

  int     close(int fd);
  int     dup(int fd);
  int     dup2(int fd1, int fd2);
  int     fsync(int fd);
  off_t   lseek(int fd, off_t offset, int whence);
  ssize_t read(int fd, FAR void *buf, size_t nbytes);
  ssize_t write(int fd, FAR const void *buf, size_t nbytes);
  ssize_t pread(int fd, FAR void *buf, size_t nbytes, off_t offset);
  ssize_t pwrite(int fd, FAR const void *buf, size_t nbytes, off_t offset);

  /* Check if a file descriptor corresponds to a terminal I/O file */

  int     isatty(int fd);

  /* Memory management */

  #if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC) && \
      defined(CONFIG_ARCH_USE_MMU)
  FAR void *sbrk(intptr_t incr);
  #endif

  /* Special devices */

  int     pipe(int fd[2]);

  /* Working directory operations */

  int     chdir(FAR const char *path);
  FAR char *getcwd(FAR char *buf, size_t size);

  /* File path operations */

  int     access(FAR const char *path, int amode);
  int     rmdir(FAR const char *pathname);
  int     unlink(FAR const char *pathname);

  #ifdef CONFIG_PSEUDOFS_SOFTLINKS
  int     link(FAR const char *path1, FAR const char *path2);
  ssize_t readlink(FAR const char *path, FAR char *buf, size_t bufsize);
  #endif

  /* Execution of programs from files */

  #ifdef CONFIG_LIBC_EXECFUNCS
  int     execl(FAR const char *path, ...);
  int     execv(FAR const char *path, FAR char * const argv[]);
  #endif

  /* Networking */

  #ifdef CONFIG_NET
  int     gethostname(FAR char *name, size_t size);
  int     sethostname(FAR const char *name, size_t size);
  #endif

  /* Other */

  int     getopt(int argc, FAR char * const argv[], FAR const char *optstring);

Standard I/O
------------

.. code-block:: c

  #include <stdio.h>

  /* Operations on streams (FILE) */

  void   clearerr(FAR FILE *stream);
  int    fclose(FAR FILE *stream);
  int    fflush(FAR FILE *stream);
  int    feof(FAR FILE *stream);
  int    ferror(FAR FILE *stream);
  int    fileno(FAR FILE *stream);
  int    fgetc(FAR FILE *stream);
  int    fgetpos(FAR FILE *stream, FAR fpos_t *pos);
  FAR char *fgets(FAR char *s, int n, FAR FILE *stream);
  FAR FILE *fopen(FAR const char *path, FAR const char *type);
  int    fprintf(FAR FILE *stream, FAR const char *format, ...);
  int    fputc(int c, FAR FILE *stream);
  int    fputs(FAR const char *s, FAR FILE *stream);
  size_t fread(FAR void *ptr, size_t size, size_t n_items, FAR FILE *stream);
  FAR FILE *freopen(FAR const char *path, FAR const char *mode,
           FAR FILE *stream);
  int    fseek(FAR FILE *stream, long int offset, int whence);
  int    fsetpos(FAR FILE *stream, FAR fpos_t *pos);
  long   ftell(FAR FILE *stream);
  size_t fwrite(FAR const void *ptr, size_t size, size_t n_items, FAR FILE *stream);
  FAR char *gets(FAR char *s);
  FAR char *gets_s(FAR char *s, rsize_t n);
  void   setbuf(FAR FILE *stream, FAR char *buf);
  int    setvbuf(FAR FILE *stream, FAR char *buffer, int mode, size_t size);
  int    ungetc(int c, FAR FILE *stream);

  /* Operations on the stdout stream, buffers, paths, and the whole printf-family *    /

  int    printf(FAR const char *format, ...);
  int    puts(FAR const char *s);
  int    rename(FAR const char *source, FAR const char *target);
  int    sprintf(FAR char *dest, FAR const char *format, ...);
  int    asprintf(FAR char **ptr, FAR const char *fmt, ...);
  int    snprintf(FAR char *buf, size_t size, FAR const char *format, ...);
  int    sscanf(FAR const char *buf, FAR const char *fmt, ...);
  void   perror(FAR const char *s);

  int    vprintf(FAR const char *s, va_list ap);
  int    vfprintf(FAR FILE *stream, FAR const char *s, va_list ap);
  int    vsprintf(FAR char *buf, FAR const char *s, va_list ap);
  int    vasprintf(FAR char **ptr, FAR const char *fmt, va_list ap);
  int    vsnprintf(FAR char *buf, size_t size, FAR const char *format, va_list ap);
  int    vsscanf(FAR char *buf, FAR const char *s, va_list ap);

  /* Operations on file descriptors including:
   *
   * POSIX-like File System Interfaces (fdopen), and
   * Extensions from the Open Group Technical Standard, 2006, Extended API Set
   *   Part 1 (dprintf and vdprintf)
   */

  FAR FILE *fdopen(int fd, FAR const char *type);
  int    dprintf(int fd, FAR const char *fmt, ...);
  int    vdprintf(int fd, FAR const char *fmt, va_list ap);

  /* Operations on paths */

  FAR char *tmpnam(FAR char *s);
  FAR char *tempnam(FAR const char *dir, FAR const char *pfx);
  int       remove(FAR const char *path);

  #include <sys/stat.h>

  int mkdir(FAR const char *pathname, mode_t mode);
  int mkfifo(FAR const char *pathname, mode_t mode);
  int stat(FAR const char *path, FAR struct stat *buf);
  int fstat(int fd, FAR struct stat *buf);

  #include <sys/statfs.h>

  int statfs(FAR const char *path, FAR struct statfs *buf);
  int fstatfs(int fd, FAR struct statfs *buf);

Standard Library (``stdlib.h``)
-------------------------------

Generally addresses other operating system interfaces.
However, the following may also be considered as file system interfaces:

.. c:function:: int mktemp(FAR char *template);
.. c:function:: int mkstemp(FAR char *template);

Asynchronous I/O
----------------

.. code-block:: c

  #include <aio.h>

  int aio_cancel(int, FAR struct aiocb *aiocbp);
  int aio_error(FAR const struct aiocb *aiocbp);
  int aio_fsync(int, FAR struct aiocb *aiocbp);
  int aio_read(FAR struct aiocb *aiocbp);
  ssize_t aio_return(FAR struct aiocb *aiocbp);
  int aio_suspend(FAR const struct aiocb * const list[], int nent,
                  FAR const struct timespec *timeout);
  int aio_write(FAR struct aiocb *aiocbp);
  int lio_listio(int mode, FAR struct aiocb * const list[], int nent,
                 FAR struct sigevent *sig);

Standard String Operations
--------------------------

.. code-block:: c

  #include <string.h>

  char  *strchr(const char *s, int c);
  FAR char *strdup(const char *s);
  const char *strerror(int);
  size_t strlen(const char *);
  size_t strnlen(const char *, size_t);
  char  *strcat(char *, const char *);
  char  *strncat(char *, const char *, size_t);
  int    strcmp(const char *, const char *);
  int    strncmp(const char *, const char *, size_t);
  int    strcasecmp(const char *, const char *);
  int    strncasecmp(const char *, const char *, size_t);
  char  *strcpy(char *dest, const char *src);
  char  *strncpy(char *, const char *, size_t);
  char  *strpbrk(const char *, const char *);
  char  *strchr(const char *, int);
  char  *strrchr(const char *, int);
  size_t strspn(const char *, const char *);
  size_t strcspn(const char *, const char *);
  char  *strstr(const char *, const char *);
  char  *strtok(char *, const char *);
  char  *strtok_r(char *, const char *, char **);

  void  *memset(void *s, int c, size_t n);
  void  *memcpy(void *dest, const void *src, size_t n);
  int    memcmp(const void *s1, const void *s2, size_t n);
  void  *memmove(void *dest, const void *src, size_t count);

  #include <strings.h>

  #define bcmp(b1,b2,len)  memcmp(b1,b2,(size_t)len)
  #define bcopy(b1,b2,len) memmove(b2,b1,len)
  #define bzero(s,n)       memset(s,0,n)
  #define index(s,c)       strchr(s,c)
  #define rindex(s,c)      strrchr(s,c)

  int    ffs(int j);
  int    strcasecmp(const char *, const char *);
  int    strncasecmp(const char *, const char *, size_t);

Pipes and FIFOs
---------------

.. c:function:: int pipe(int fd[2])

  Creates a pair of file descriptors, pointing to a pipe inode,
  and places them in the array pointed to by ``fd``.

  :param fd: The user provided array in which to catch the pipe file
    descriptors. ``fd[0]`` is for reading, ``fd[1]`` is for writing.

  :return: 0 is returned on success; otherwise, -1 is returned with errno set appropriately.

.. c:function:: int mkfifo(FAR const char *pathname, mode_t mode);

  mkfifo() makes a FIFO device driver file with name pathname. Unlike Linux,
  a NuttX FIFO is not a special file type but simply a device driver instance.
  mode specifies the FIFO's permissions (but is ignored in the current implementation).

  Once the FIFO has been created by mkfifo(), any thread can open it for reading
  or writing, in the same way as an ordinary file. However, it must have been
  opened from both reading and writing before input or output can be performed.
  This FIFO implementation will block all attempts to open a FIFO read-only
  until at least one thread has opened the FIFO for writing.

  If all threads that write to the FIFO have closed, subsequent calls to
  read() on the FIFO will return 0 (end-of-file).

  :param pathname: The full path to the FIFO instance to attach to or to
    create (if not already created).
  :param mode: Ignored for now

  :return: 0 is returned on success; otherwise, -1 is returned with errno set appropriately.

``mmap()`` and eXecute In Place (XIP)
-------------------------------------

NuttX operates in a flat open address space and is focused on MCUs that
do support Memory Management Units (MMUs). Therefore, NuttX generally
does not require ``mmap()`` functionality and the MCUs generally cannot
support true memory-mapped files.

However, memory mapping of files is the mechanism used by NXFLAT, the
NuttX tiny binary format, to get files into memory in order to execute
them. ``mmap()`` support is therefore required to support NXFLAT. There
are two conditions where ``mmap()`` can be supported:

1. ``mmap()`` can be used to support *eXecute In Place* (XIP) on random
   access media under the following very restrictive conditions:

   a. The file-system supports the ``FIOC_MMAP`` ioctl command. Any file
      system that maps files contiguously on the media should support
      this ``ioctl`` command. By comparison, most file system scatter
      files over the media in non-contiguous sectors. As of this
      writing, ROMFS is the only file system that meets this
      requirement.

   b. The underlying block driver supports the ``BIOC_XIPBASE``
      ``ioctl`` command that maps the underlying media to a randomly
      accessible address. At present, only the RAM/ROM disk driver does
      this.

   Some limitations of this approach are as follows:

   a. Since no real mapping occurs, all of the file contents are
      "mapped" into memory.

   b. All mapped files are read-only.

   c. There are no access privileges.

2. If ``CONFIG_FS_RAMMAP`` is defined in the configuration, then
   ``mmap()`` will support simulation of memory mapped files by copying
   files whole into RAM. These copied files have some of the properties
   of standard memory mapped files. There are many, many exceptions
   exceptions, however. Some of these include:

   a. The goal is to have a single region of memory that represents a
      single file and can be shared by many threads. That is, given a
      filename a thread should be able to open the file, get a file
      descriptor, and call ``mmap()`` to get a memory region. Different
      file descriptors opened with the same file path should get the
      same memory region when mapped.

      The limitation in the current design is that there is insufficient
      knowledge to know that these different file descriptors correspond
      to the same file. So, for the time being, a new memory region is
      created each time that ``rammmap()`` is called. Not very useful!

   b. The entire mapped portion of the file must be present in memory.
      Since it is assumed that the MCU does not have an MMU,
      on-demanding paging in of file blocks cannot be supported. Since
      the while mapped portion of the file must be present in memory,
      there are limitations in the size of files that may be memory
      mapped (especially on MCUs with no significant RAM resources).

   c. All mapped files are read-only. You can write to the in-memory
      image, but the file contents will not change.

   d. There are no access privileges.

   e. Since there are no processes in NuttX, all ``mmap()`` and
      ``munmap()`` operations have immediate, global effects. Under
      Linux, for example, ``munmap()`` would eliminate only the mapping
      with a process; the mappings to the same file in other processes
      would not be effected.

   f. Like true mapped file, the region will persist after closing the
      file descriptor. However, at present, these ram copied file
      regions are *not* automatically "unmapped" (i.e., freed) when a
      thread is terminated. This is primarily because it is not possible
      to know how many users of the mapped region there are and,
      therefore, when would be the appropriate time to free the region
      (other than when munmap is called).

      NOTE: Note, if the design limitation of a) were solved, then it
      would be easy to solve exception d) as well.

.. c:function:: FAR void *mmap(FAR void *start, size_t length, int prot, int flags, int fd, off_t offset);

  Provides minimal mmap() as needed to support eXecute In Place (XIP) operation (as described above).

  :param start: A hint at where to map the memory -- ignored. The address
    of the underlying media is fixed and cannot be re-mapped without MMU
    support.
  :param length: The length of the mapping -- ignored. The entire
    underlying media is always accessible.
  :param prot: See the ``PROT_*`` definitions in ``sys/mman.h``.

     -  ``PROT_NONE`` - Will cause an error.
     -  ``PROT_READ`` - ``PROT_WRITE`` and ``PROT_EXEC`` also assumed.
     -  ``PROT_WRITE`` - ``PROT_READ`` and ``PROT_EXEC`` also assumed.
     -  ``PROT_EXEC`` - ``PROT_READ`` and ``PROT_WRITE`` also assumed.

  :param flags: See the ``MAP_*`` definitions in ``sys/mman.h``.

     -  ``MAP_SHARED`` - Required
     -  ``MAP_PRIVATE`` - Will cause an error
     -  ``MAP_FIXED`` - Will cause an error
     -  ``MAP_FILE`` - Ignored
     -  ``MAP_ANONYMOUS`` - Will cause an error
     -  ``MAP_ANON`` - Will cause an error
     -  ``MAP_GROWSDOWN`` - Ignored
     -  ``MAP_DENYWRITE`` - Will cause an error
     -  ``MAP_EXECUTABLE`` - Ignored
     -  ``MAP_LOCKED`` - Ignored
     -  ``MAP_NORESERVE`` - Ignored
     -  ``MAP_POPULATE`` - Ignored
     -  ``AP_NONBLOCK`` - Ignored

  :param fd: file descriptor of the backing file -- required.
  :param offset: The offset into the file to map.

  :return:

    On success, ``mmap()`` returns a pointer to the mapped area. On error,
    the value ``MAP_FAILED`` is returned, and ``errno`` is set
    appropriately.

    -  ``ENOSYS`` - Returned if any of the unsupported ``mmap()`` features
       are attempted.
    -  ``EBADF`` - ``fd`` is not a valid file descriptor.
    -  ``EINVAL`` - Length is 0. flags contained neither ``MAP_PRIVATE`` or
       ``MAP_SHARED``, or contained both of these values.
    -  ``ENODEV`` - The underlying file-system of the specified file does
       not support memory mapping.

