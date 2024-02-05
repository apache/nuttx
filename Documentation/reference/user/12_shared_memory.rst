========================
Shared Memory Interfaces
========================

Shared memory interfaces are only available with the NuttX kernel build
(``CONFIG_BUILD_KERNEL=y``). These interfaces support user memory
regions that can be shared between multiple user processes. Shared
memory interfaces:

  - :c:func:`shmget`
  - :c:func:`shmat`
  - :c:func:`shmctl`
  - :c:func:`shmdt`

Functions
=========

.. c:function:: int shmget(key_t key, size_t size, int shmflg)

  Returns the shared memory identifier associated with ``key``.

  A shared memory identifier, associated data structure, and shared memory
  segment of at least size bytes is created for ``key`` if one of the
  following is true:

  -  The argument ``key`` is equal to ``IPC_PRIVATE``.

  -  The argument ``key`` does not already have a shared memory identifier
     associated with it and ``(shmflg & IPC_CREAT)`` is non-zero.

  Upon creation, the data structure associated with the new shared memory
  identifier will be initialized as follows:

  -  The low-order nine bits of ``shm_perm.mode`` are set equal to the
     low-order nine bits of ``shmflg``.

  -  The value of ``shm_segsz`` is set equal to the value of size.

  -  The values of ``shm_lpid``, ``shm_nattch``, ``shm_atime``, and
     ``shm_dtime`` are set equal to 0.

  -  The value of ``shm_ctime`` is set equal to the current time.

  When the shared memory segment is created, it will be initialized with
  all zero values.

  :param key: The key that is used to access the unique shared memory
     identifier.
  :param size: The shared memory region that is created will be at least
     this size in bytes.
  :param shmflg: See ``IPC_*`` definitions in ``sys/ipc.h``. Only the
     values ``IPC_PRIVATE`` or ``IPC_CREAT`` are supported.

  :return: Upon successful completion, ``shmget()`` will return
    a non-negative integer, namely a shared memory identifier; otherwise, it
    will return -1 and set ``errno`` to indicate the error.

    - ``EACCES``. A shared memory identifier exists for key but operation
      permission as specified by the low-order nine bits of ``shmflg``
      would not be granted.
    - ``EEXIST``. A shared memory identifier exists for the argument key
      but ``(shmflg & IPC_CREAT) && (shmflg & IPC_EXCL)`` are non-zero.
    - ``EINVAL``. A shared memory segment is to be created and the value of
      size is less than the system-imposed minimum or greater than the
      system-imposed maximum.
    - ``EINVAL``. No shared memory segment is to be created and a shared
      memory segment exists for key but the size of the segment associated
      with it is less than size and size is not 0.
    - ``ENOENT``. A shared memory identifier does not exist for the
      argument key and ``(shmflg & IPC_CREAT)`` is 0.
    - ``ENOMEM``. A shared memory identifier and associated shared memory
      segment will be created, but the amount of available physical memory
      is not sufficient to fill the request.
    - ``ENOSPC``. A shared memory identifier is to be created, but the
      system-imposed limit on the maximum number of allowed shared memory
      identifiers system-wide would be exceeded.

  **POSIX Deviations**

  -  The values of ``shm_perm.cuid``, ``shm_perm.uid``, ``shm_perm.cgid``,
     and ``shm_perm.gid`` should be set equal to the effective user ID and
     effective group ID, respectively, of the calling process. The NuttX
     ``ipc_perm`` structure, however, does not support these fields
     because user and group IDs are not yet supported by NuttX.

.. c:function:: void *shmat(int shmid, FAR const void *shmaddr, int shmflg)

  Attaches the shared memory
  segment associated with the shared memory identifier specified by
  ``shmid`` to the address space of the calling process. The segment is
  attached at the address specified by one of the following criteria:

  -  If ``shmaddr`` is a null pointer, the segment is attached at the
     first available address as selected by the system.

  -  If ``shmaddr`` is not a null pointer and ``(shmflg & SHM_RND)`` is
     non-zero, the segment is attached at the address given by
     ``(shmaddr - ((uintptr_t)shmaddr % SHMLBA))``.

  -  If ``shmaddr`` is not a null pointer and ``(shmflg & SHM_RND)`` is 0,
     the segment is attached at the address given by ``shmaddr``.

  -  The segment is attached for reading if ``(shmflg & SHM_RDONLY)`` is
     non-zero and the calling process has read permission; otherwise, if
     it is 0 and the calling process has read and write permission, the
     segment is attached for reading and writing.

  :param shmid: Shared memory identifier
  :param smaddr: Determines mapping of the shared memory region
  :param shmflg: See ``SHM_*`` definitions in ``include/sys/shm.h``. Only
     ``SHM_RDONLY`` and ``SHM_RND`` are supported.

  :return: Upon successful completion, ``shmat()`` will
    increment the value of ``shm_nattch`` in the data structure associated
    with the shared memory ID of the attached shared memory segment and
    return the segment's start address. Otherwise, the shared memory segment
    will not be attached, ``shmat()`` will return -1, and ``errno`` will be
    set to indicate the error.

    -  ``EACCES``. Operation permission is denied to the calling process
    -  ``EINVAL``. The value of ``shmid`` is not a valid shared memory
       identifier, the ``shmaddr`` is not a null pointer, and the value of
       ``(shmaddr -((uintptr_t)shmaddr % SHMLBA))`` is an illegal address
       for attaching shared memory; or the ``shmaddr`` is not a null
       pointer, ``(shmflg & SHM_RND)`` is 0, and the value of ``shmaddr`` is
       an illegal address for attaching shared memory.
    -  ``EMFILE``. The number of shared memory segments attached to the
       calling process would exceed the system-imposed limit.
    -  ``ENOMEM``. The available data space is not large enough to
       accommodate the shared memory segment.

.. c:function:: int shmctl(int shmid, int cmd, FAR struct shmid_ds *buf)

  Provides a variety of shared
  memory control operations as specified by ``cmd``. The following values
  for ``cmd`` are available:

  -  ``IPC_STAT``. Place the current value of each member of the
     ``shmid_ds`` data structure associated with ``shmid`` into the
     structure pointed to by ``buf``.

  -  ``IPC_SET``. Set the value of the ``shm_perm.mode`` member of the
     ``shmid_ds`` data structure associated with ``shmid`` to the
     corresponding value found in the structure pointed to by ``buf``.

  -  ``IPC_RMID``. Remove the shared memory identifier specified by
     ``shmid`` from the system and destroy the shared memory segment and
     ``shmid_ds`` data structure associated with it.

  :param shmid: Shared memory identifier
  :param cmd: ``shmctl()`` command
  :param buf: Data associated with the ``shmctl()`` command

  :return: Upon successful completion, ``shmctl()`` will return
    0; otherwise, it will return -1 and set ``errno`` to indicate the error.

    -  ``EACCES``. The argument ``cmd`` is equal to ``IPC_STAT`` and the
       calling process does not have read permission.
    -  ``EINVAL``. The value of ``shmid`` is not a valid shared memory
       identifier, or the value of ``cmd``\ is not a valid command.
    -  ``EPERM``. The argument ``cmd`` is equal to ``IPC_RMID`` or
       ``IPC_SET`` and the effective user ID of the calling process is not
       equal to that of a process with appropriate privileges and it is not
       equal to the value of ``shm_perm.cuid`` or ``shm_perm.uid`` in the
       data structure associated with ``shmid``.
    -  ``EOVERFLOW``. The ``cmd`` argument is ``IPC_STAT`` and the ``gid``
       or ``uid`` value is too large to be stored in the structure pointed
       to by the ``buf`` argument.

  **POSIX Deviations**

  -  ``IPC_SET``. Does not set the ``shm_perm.uid`` or
     ``shm_perm.gid``\ members of the ``shmid_ds`` data structure
     associated with ``shmid`` because user and group IDs are not yet
     supported by NuttX
  -  ``IPC_SET``. Does not restrict the operation to processes with
     appropriate privileges or matching user IDs in ``shmid_ds`` data
     structure associated with ``shmid``. Again because user IDs and
     user/group privileges are are not yet supported by NuttX
  -  ``IPC_RMID``. Does not restrict the operation to processes with
     appropriate privileges or matching user IDs in ``shmid_ds`` data
     structure associated with ``shmid``. Again because user IDs and
     user/group privileges are are not yet supported by NuttX

.. c:function:: int shmdt(FAR const void *shmaddr)

  Detaches the shared memory
  segment located at the address specified by ``shmaddr`` from the address
  space of the calling process.

  :param shmid: Shared memory identifier

  :return: Upon successful completion, ``shmdt()`` will
    decrement the value of ``shm_nattch`` in the data structure associated
    with the shared memory ID of the attached shared memory segment and
    return 0.

    Otherwise, the shared memory segment will not be detached, ``shmdt()``
    will return -1, and ``errno`` will be set to indicate the error.

    -  ``EINVAL``. The value of ``shmaddr`` is not the data segment start
       address of a shared memory segment.

