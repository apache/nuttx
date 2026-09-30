=======
Inotify
=======
Inotify is a kernel subsystem designed for monitoring filesystem events. It enables applications to monitor changes to files and directories in real-time, such as creation, deletion, modification, renaming, and more. inotify offers an efficient way to detect changes in the filesystem without the need for polling, thereby conserving system resources.

CONFIG
------
.. code-block:: c

    COFNIG_FS_NOTIFY=y

User Space API
--------------

All inotify user interfaces are declared in the file ``include/sys/inotify.h``.
And the usage is consistent with the Linux version.

.. c:function:: int inotify_init(void)

  Initializes a new inotify instance and returns a file descriptor
  associated with a new inotify event queue.

.. c:function:: int inotify_init1(int flags)

  inotify_init1 is an extended version of inotify_init, offering additional
  options for initializing an inotify instance. Unlike inotify_init,
  inotify_init1 allows you to specify certain flags to control the behavior of
  the inotify instance.

.. c:function:: int inotify_add_watch(int fd, FAR const char *pathname, uint32_t mask)

  Add a new watch, or modifies an existing watch, for the file whose
  location is specified in pathname; the caller must have read permission
  for this file. The fd argument is a file descriptor referring to the
  inotify instance whose watch list is to be modified. The events to be
  monitored for pathname are specified in the mask bit-mask argument.

.. c:function:: int inotify_rm_watch(int fd, uint32_t wd)

  Removes the watch associated with the watch descriptor wd from the
  inotify instance associated with the file descriptor fd.

Reading events from an inotify file descriptor
----------------------------------------------

To  determine  what  events have occurred, an application read from
the inotify file descriptor.  If no events have so far occurred,  then,
assuming  a blocking file descriptor, read will block until at least
one event occurs

Each  successful read returns a buffer containing one or more of the
following structures:

.. code-block:: c

  struct inotify_event {
    int      wd;       /* Watch descriptor */
    uint32_t mask;     /* Mask describing event */
    uint32_t cookie;   /* Unique cookie associating related
                         events (for rename(2)) */
    uint32_t len;      /* Size of name field */
    char     name[];   /* Optional null-terminated name */
  };

**wd** identifies the watch for which this event occurs.  It is one of  the
watch descriptors returned by a previous call to inotify_add_watch.

**mask** contains bits that describe the event that occurred

**cookie** is a unique integer that connects related events.  Currently,
this is used only for rename events, and allows the resulting  pair  of
IN_MOVED_FROM  and  IN_MOVED_TO  events to be connected by the application
For all other event types, cookie is set to 0.

The **name** field is present only when an event is returned for a file
inside a watched directory; it identifies the filename within the watched
directory.  This filename is null-terminated, and may  include  further
null  bytes  ('\0')  to  align  subsequent  reads to a suitable address boundary.

The **len** field counts all of the  bytes in name, including the null
bytes; the length of each inotify_event structure is thus sizeof(struct
inotify_event)+len.

inotify events
--------------
The **inotify_add_watch** mask argument and the mask field of the inotify_event
structure returned when reading an inotify file  descriptor are both bit masks
identifying inotify events.  The following bits can be specified in mask when
calling inotify_add_watch and  may  be returned in the mask field returned by read.

  **IN_ACCESS** :File was accessed

  **IN_MODIFY** :File was modified (``write()`` or ``truncate()``)

  **IN_ATTRIB** :Metadata changed

  **IN_OPEN** :File was opened

  **IN_CLOSE_WRITE** :File opened for writing was closed

  **IN_CLOSE_NOWRITE** : File not opened for writing was closed

  **IN_MOVED_FROM** :File was moved from X

  **IN_MOVED_TO** :File was moved to Y

  **IN_CREATE** :Subfile was created

  **IN_DELETE** :Subfile was deleted

  **IN_DELETE_SELF** :Self was deleted

  **IN_MOVE_SELF** :Self was moved

Examples
--------
Suppose  an  application  is  watching  the  directory ``dir`` and the file
``dir/myfile`` for all events.  The examples below show  some  events  that
will be generated for these two objects.

  fd = open("dir/myfile", O_RDWR);
    Generates **IN_OPEN** events for both ``dir`` and ``dir/myfile``.

  read(fd, buf, count);
    Generates **IN_ACCESS** events for both ``dir`` and ``dir/myfile``.

  write(fd, buf, count);
    Generates **IN_MODIFY** events for both ``dir`` and ``dir/myfile``.

  fchmod(fd, mode);
    Generates **IN_ATTRIB** events for both ``dir`` and ``dir/myfile``.

NOTE
----
Inotify file descriptors can be monitored using select, poll, and
epoll.  When an event is available, the file descriptor indicates as
readable.
