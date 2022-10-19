=======================
Task Control Interfaces
=======================

**Tasks**. NuttX is a flat address OS. As such it does not support
*processes* in the way that, say, Linux does. NuttX only supports simple
threads running within the same address space. However, the programming
model makes a distinction between *tasks* and *pthreads*:

  - *tasks* are threads which have a degree of independence
  - `pthreads <#Pthread>`__ share some resources.

**File Descriptors and Streams**. This applies, in particular, in the
area of opened file descriptors and streams. When a task is started
using the interfaces in this section, it will be created with at most
three open files.

If ``CONFIG_DEV_CONSOLE`` is defined, the first three file descriptors
(corresponding to stdin, stdout, stderr) will be duplicated for the new
task. Since these file descriptors are duplicated, the child task can
free close them or manipulate them in any way without effecting the
parent task. File-related operations (open, close, etc.) within a task
will have no effect on other tasks. Since the three file descriptors are
duplicated, it is also possible to perform some level of redirection.

pthreads, on the other hand, will always share file descriptors with the
parent thread. In this case, file operations will have effect only all
pthreads the were started from the same parent thread.

**Executing Programs within a File System**. NuttX also provides
internal interfaces for the execution of separately built programs that
reside in a file system. These internal interfaces are, however,
non-standard and are documented with the NuttX binary
loader and NXFLAT documentation.

**Task Control Interfaces**. The following task control interfaces are
provided by NuttX:

.. note::
  Maybe this can be converted into a table, or could otherwise
  be replaced by the index if these are sectioned in this way.

Non-standard task control interfaces inspired by VxWorks interfaces:

  - :c:func:`task_create`
  - :c:func:`task_delete`
  - :c:func:`task_restart`

Non-standard extensions to VxWorks-like interfaces to support POSIX
`Cancellation
Points <https://cwiki.apache.org/confluence/display/NUTTX/Cancellation+Points>`__.

  - :c:func:`task_setcancelstate`
  - :c:func:`task_setcanceltype`
  - :c:func:`task_testcancel`

Standard interfaces

  - :c:func:`exit`
  - :c:func:`getpid`

Standard ``vfork`` and ``exec[v|l]`` interfaces:

  - :c:func:`vfork`
  - :c:func:`exec`
  - :c:func:`execv`
  - :c:func:`execl`

Standard ``posix_spawn`` interfaces:

  - :c:func:`posix_spawn` and :c:func:`posix_spawnp`
  - :c:func:`posix_spawn_file_actions_init`
  - :c:func:`posix_spawn_file_actions_destroy`
  - :c:func:`posix_spawn_file_actions_addclose`
  - :c:func:`posix_spawn_file_actions_adddup2`
  - :c:func:`posix_spawn_file_actions_addopen`
  - :c:func:`posix_spawnattr_init`
  - :c:func:`posix_spawnattr_getflags`
  - :c:func:`posix_spawnattr_getschedparam`
  - :c:func:`posix_spawnattr_getschedpolicy`
  - :c:func:`posix_spawnattr_getsigmask`
  - :c:func:`posix_spawnattr_setflags`
  - :c:func:`posix_spawnattr_setschedparam`
  - :c:func:`posix_spawnattr_setschedpolicy`
  - :c:func:`posix_spawnattr_setsigmask`

Non-standard task control interfaces inspired by ``posix_spawn``:

  - :c:func:`task_spawn`
  - :c:func:`posix_spawnattr_getstacksize`
  - :c:func:`posix_spawnattr_setstacksize`

Functions
---------

.. c:function:: int task_create(char *name, int priority, int stack_size, main_t entry, char * const argv[])

  This function creates and activates a new task with a
  specified priority and returns its system-assigned ID.

  The entry address entry is the address of the "main" function of the
  task. This function will be called once the C environment has been set
  up. The specified function will be called with four arguments. Should
  the specified routine return, a call to :c:func:`exit` will automatically be
  made.

  Note that an arbitrary number of arguments may be passed to the spawned
  functions.

  The arguments are copied (via ``strdup``) so that the life of the passed
  strings is not dependent on the life of the caller to :c:func:`task_create`.

  The newly created task does not inherit scheduler characteristics from
  the parent task: The new task is started at the default system priority
  and with the ``SCHED_FIFO`` scheduling policy. These characteristics may be
  modified after the new task has been started.

  The newly created task does inherit the first three file descriptors
  (corresponding to stdin, stdout, and stderr) and redirection of standard
  I/O is supported.

  :param name: Name of the new task
  :param priority: Priority of the new task
  :param stack_size: size (in bytes) of the stack needed
  :param entry: Entry point of a new task
  :param argv: A pointer to an array of input parameters. The array should
               be terminated with a NULL argv[] value. If no parameters are
               required, argv may be NULL.

  :return: the non-zero task ID of the new task or ERROR if memory is
           insufficient or the task cannot be created
           (```errno`` <#ErrnoAccess>`__ is not set).

  **Defined in:** ``sched.h``

  **POSIX Compatibility:** This is a NON-POSIX interface. VxWorks provides
  the following similar interface:

  .. code-block:: c

    int taskSpawn(char *name, int priority, int options, int stackSize, FUNCPTR entryPt,
                  int arg1, int arg2, int arg3, int arg4, int arg5,
                  int arg6, int arg7, int arg8, int arg9, int arg10);

  The NuttX :c:func:`task_create` differs from VxWorks' :c:func:`taskSpawn` in the
  following ways:

    - Interface name
    - Various differences in types of arguments
    - There is no options argument.
    - A variable number of parameters can be passed to a task (VxWorks
      supports ten).

.. c:function:: int task_delete(pid_t pid)

  This function causes a specified task to cease to
  exist. Its stack and TCB will be deallocated. This function is the
  companion to ``task_create()``. This is the version of the function
  exposed to the user; it is simply a wrapper around the internal,
  ``nxtask_terminate()`` function.

  The logic in this function only deletes non-running tasks. If the
  ``pid`` parameter refers to the currently running task, then processing
  is redirected to ``exit()``. This can only happen if a task calls
  ``task_delete()`` in order to delete itself.

  This function obeys the semantics of pthread cancellation: task deletion
  is deferred if cancellation is disabled or if deferred cancellation is
  supported (with `Cancellation
  Points <https://cwiki.apache.org/confluence/display/NUTTX/Cancellation+Points>`__
  enabled).

  :param pid: The task ID of the task to delete. An ID of zero signifies
     the calling task. Any attempt by the calling task will be
     automatically re-directed to ``exit()``.

  :return: ``OK``, or ``ERROR`` if the task cannot be deleted. The
     ```errno`` <#ErrnoAccess>`__ is set to indicate the nature of the
     failure. This function can fail, for example, if the provided pid
     does not correspond to a currently executing task.

  **Assumptions/Limitations:**

  ``task_delete()`` must be used with caution: If the task holds resources
  (for example, allocated memory or semaphores needed by other tasks),
  then ``task_delete()`` can strand those resources.

  **POSIX Compatibility:** This is a NON-POSIX interface. VxWorks provides
  the following similar interface:

  The NuttX task_delete() differs from VxWorks' taskDelete() in the
  following ways:

  - No support is provided for calling the tasks deletion routines
    (because the VxWorks ``taskDeleteHookAdd()`` is not supported).
    However, if ``atexit()`` or ``on_exit`` support is enabled, those
    will be called when the task deleted.
  - Deletion of self is supported, but only because ``task_delete()``
    will re-direct processing to ``exit()``.

.. :c:function:: int task_restart(pid_t pid)

  This function *restarts* a task. The task is first
  terminated and then reinitialized with same ID, priority, original entry
  point, stack size, and parameters it had when it was first started.

  **NOTES:**

    #. The normal task exit clean up is not performed. For example, file
       descriptors are not closed; any files opened prior to the restart
       will remain opened after the task is restarted.
    #. Memory allocated by the task before it was restart is not freed. A
       task that is subject to being restart must be designed in such a way
       as to avoid memory leaks.
    #. Initialized data is not reset. All global or static data is left in
       the same state as when the task was terminated. This *feature* may be
       used by restart task to detect that it has been restarted, for
       example.

  :param pid: The task ID of the task to delete. An ID of zero would
    signify the calling task (However, support for a task to restart
     itself has not been implemented).

  :return: ``OK``, or ``ERROR`` if the task ID is invalid or the task could not be
     restarted. This function can fail if: (1) A pid of zero or the pid of
     the calling task is provided (functionality not implemented) (2) The
     pid is not associated with any task known to the system.

  **POSIX Compatibility:** This is a NON-POSIX interface. VxWorks provides
  the following similar interface:

  .. code-block:: c

    STATUS taskRestart (int tid);

  The NuttX :c:func:`task_restart` differs from VxWorks' :c:func:`taskRestart` in the
  following ways:

  -  Restart of the currently running task is not supported by NuttX.
  -  The VxWorks description says that the ID, priority, etc. take the
     value that they had when the task was *terminated*.

.. c:function:: int task_setcancelstate(int state, int *oldstate)

  This function atomically sets
  both the calling task's cancellability state to the indicated state and
  returns the previous cancellability state at the location referenced by
  oldstate. Legal values for state are TASK_CANCEL_ENABLE and
  TASK_CANCEL_DISABLE.

  Any pending thread cancellation may occur at the time that the
  cancellation state is set to TASK_CANCEL_ENABLE.

  The cancellability state and type of any newly created tasks are
  TASK_CANCEL_ENABLE and TASK_CANCEL_DEFERRED respectively.

  :param state: New cancellation state. One of PTHREAD_CANCEL_ENABLE or
   PTHREAD_CANCEL_DISABLE.
  :param oldstate: Location to return the previous cancellation state.

  :return: Zero (``OK``) on success; ``ERROR`` is returned on any failure
    with the ``errno`` value set appropriately:

      - ``ESRCH``. No thread could be found corresponding to that specified
        by the given thread ID.

  **POSIX Compatibility:** This is a non-standard interface. It extends
  the functionality of ``pthread_setcancelstate()`` to tasks and supports
  use of ``task_delete()``.

.. c:function:: int task_setcanceltype(int type, FAR int *oldtype);

  This function atomically both
  sets the calling task's cancellability type to the indicated type and
  returns the previous cancellability type at the location referenced by
  ``oldtype``. Legal values for type are ``TASK_CANCEL_DEFERRED`` and
  ``TASK_CANCEL_ASYNCHRONOUS``.

  The cancellability state and type of any newly created tasks are
  ``TASK_CANCEL_ENABLE`` and ``TASK_CANCEL_DEFERRED`` respectively.

  :param type: New cancellation state. One of ``PTHREAD_CANCEL_DEFERRED``
     or ``PTHREAD_CANCEL_ASYNCHRONOUS``.
  :param oldtype: Location to return the previous cancellation type.

  :return: Zero (``OK``) on success; ``ERROR`` is returned on any failure with the
    ``errno`` value set appropriately:

    - ``ESRCH``. No thread could be found corresponding to that specified
      by the given thread ID.

  **POSIX Compatibility:** This is a non-standard interface. It extends
  the functionality of ``pthread_setcanceltype()`` to tasks and supports
  use of ``task_delete()``.

.. c:function:: void task_testcancel(void)

  Creates a `Cancellation
  Point <https://cwiki.apache.org/confluence/display/NUTTX/Cancellation+Points>`__
  in the calling task. The ``task_testcancel()`` function has no effect if
  cancellability is disabled.

  **POSIX Compatibility:** This is a non-standard interface. It extends
  the functionality of ``pthread_testcancel()`` to tasks and supports use
  of ``task_delete()``.

.. c:function:: void exit(int code)
.. c:function:: void _exit(int code)

  ..  #include <sched.h>
  ..  #include <nuttx/unistd.h>

  Causes the calling task to cease to exist
  -- its stack and TCB will be deallocated. exit differs from \_exit in
  that it flushes streams, closes file descriptors and will execute any
  function registered with ``atexit()`` or ``on_exit()``.

  :param code: (ignored)

  **POSIX Compatibility:** This is equivalent to the ANSI interface:

  ::

         void exit(int code);

  And the UNIX interface:

  ::

         void _exit(int code);

  The NuttX exit() differs from ANSI exit() in the following ways:

    -  The ``code`` parameter is ignored.

.. c:function:: pid_t getpid(void)

.. #include <unistd.h>

  Returns the task ID of the calling task.
  The task ID will be invalid if called at the interrupt level.

  :return: The task ID of the calling task.

  **POSIX Compatibility:** Compatible with the POSIX interface of the same
  name.

.. c:function:: pid_t vfork(void)

  The ``vfork()`` function has the same effect as
  ``fork()``, except that the behavior is undefined if the process created
  by ``vfork()`` either modifies any data other than a variable of type
  ``pid_t`` used to store the return value from ``vfork()``, or returns
  from the function in which ``vfork()`` was called, or calls any other
  function before successfully calling ``_exit()`` or one of the ``exec``
  family of functions.

     NOTE: ``vfork()`` is not an independent NuttX feature, but is
     implemented in architecture-specific logic (using only helper
     functions from the NuttX core logic). As a result, ``vfork()`` may
     not be available on all architectures.

  :return: Upon successful completion, ``vfork()`` returns 0 to
    the child process and returns the process ID of the child process to the
    parent process. Otherwise, -1 is returned to the parent, no child
    process is created, and ``errno`` is set to indicate the error.

  **POSIX Compatibility:** Compatible with the BSD/Linux interface of the
  same name. POSIX marks this interface as Obsolete.

.. c:function:: int exec(FAR const char *filename, FAR char * const *argv, FAR const struct symtab_s *exports, int nexports)

  This non-standard, NuttX function is similar to
  ``execv()`` and ``posix_spawn()`` but differs in the following ways;

  -  Unlike ``execv()`` and ``posix_spawn()`` this function accepts symbol
     table information as input parameters. This means that the symbol
     table used to link the application prior to execution is provided by
     the caller, not by the system.
  -  Unlike ``execv()``, this function always returns.

  This non-standard interface is included as a official NuttX API only
  because it is needed in certain build modes: ``exec()`` is probably the
  only want to load programs in the PROTECTED mode. Other file execution
  APIs rely on a symbol table provided by the OS. In the PROTECTED build
  mode, the OS cannot provide any meaningful symbolic information for
  execution of code in the user-space blob so that is the ``exec()``
  function is really needed in that build case

  The interface is available in the FLAT build mode although it is not
  really necessary in that case. It is currently used by some example code
  under the ``apps/`` that that generate their own symbol tables for
  linking test programs. So although it is not necessary, it can still be
  useful.

  The interface would be completely useless and will not be supported in
  the KERNEL build mode where the contrary is true: An application process
  cannot provide any meaning symbolic information for use in linking a
  different process.

  **NOTE**: This function is flawed and useless without
  ``CONFIG_SCHED_ONEXIT`` and ``CONFIG_SCHED_HAVE_PARENT`` because without
  those features there is then no mechanism to unload the module once it
  exits.

  :param filename: The path to the program to be executed. If
     ``CONFIG_LIBC_ENVPATH`` is defined in the configuration, then this may
     be a relative path from the current working directory. Otherwise,
     ``path`` must be the absolute path to the program.
  :param argv: A pointer to an array of string arguments. The end of the
     array is indicated with a NULL entry.
  :param exports: The address of the start of the caller-provided symbol
     table. This symbol table contains the addresses of symbols exported
     by the caller and made available for linking the module into the
     system.
  :param nexports: The number of symbols in the ``exports`` table.

  :return: Zero (OK) is returned on success; On any failure, ``exec()``
    will return -1 (``ERROR``) and will set the ``errno`` value
    appropriately.

  **POSIX Compatibility:** This is a non-standard interface unique to
  NuttX. Motivation for inclusion of this non-standard interface in
  certain build modes is discussed above.

.. c:function:: int execv(FAR const char *path, FAR char * const argv[])

  The standard ``exec`` family of functions will replace
  the current process image with a new process image. The new image will
  be constructed from a regular, executable file called the new process
  image file. There will be no return from a successful ``exec``, because
  the calling process image is overlaid by the new process image.

  Simplified ``execl()`` and ``execv()`` functions are provided by NuttX
  for compatibility. NuttX is a tiny embedded RTOS that does not support
  processes and hence the concept of overlaying a tasks process image with
  a new process image does not make any sense. In NuttX, these functions
  are wrapper functions that:

    #. Call the non-standard ``binfmt`` function ``exec()``, and then
    #. ``exit(0)``.

  Note the inefficiency when ``execv()`` or ``execl()`` is called in the
  normal, two-step process: (1) first call ``vfork()`` to create a new
  thread, then (2) call ``execv()`` or ``execl()`` to replace the new
  thread with a program from the file system. Since the new thread will be
  terminated by the ``execv()`` or ``execl()`` call, it really served no
  purpose other than to support POSIX compatibility.

  The non-standard binfmt function ``exec()`` needs to have (1) a symbol
  table that provides the list of symbols exported by the base code, and
  (2) the number of symbols in that table. This information is currently
  provided to ``exec()`` from ``execv()`` or ``execl()`` via NuttX
  configuration settings:

    -  ``CONFIG_LIBC_EXECFUNCS``: Enable ``execv()`` and ``execl()`` support
    -  ``CONFIG_EXECFUNCS_SYMTAB_ARRAY``: Name of the symbol table used by
       ``execv()`` or ``execl()``.
    -  ``CONFIG_EXECFUNCS_NSYMBOLS_VAR``: Name of the ``int`` variable
       holding the number of symbols in the symbol table

  As a result of the above, the current implementations of ``execl()`` and
  ``execv()`` suffer from some incompatibilities that may or may not be
  addressed in a future version of NuttX. Other than just being an
  inefficient use of MCU resource, the most serious of these is that the
  ``exec``'ed task will not have the same task ID as the ``vfork``'ed
  function. So the parent function cannot know the ID of the ``exec``'ed
  task.

  :param path: The path to the program to be executed. If
     ``CONFIG_LIBC_ENVPATH`` is defined in the configuration, then this may
     be a relative path from the current working directory. Otherwise,
  :param path: must be the absolute path to the program.

  :return: This function does not return on success. On
    failure, it will return -1 (``ERROR``) and will set the ``errno`` value
    appropriately.

  **POSIX Compatibility:** Similar with the POSIX interface of the same
  name. There are, however, several compatibility issues as detailed in
  the description above.

.. c:function:: int execl(FAR const char *path, ...)

  ``execl()`` is functionally equivalent to
  `execv() <#execv>`__, differing only in the form of its input
  parameters. See the description of `execv() <#execv>`__ for additional
  information.

  :param path: The path to the program to be executed. If
     ``CONFIG_LIBC_ENVPATH`` is defined in the configuration, then this may
     be a relative path from the current working directory. Otherwise,
  :param path: must be the absolute path to the program.

  :return: This function does not return on success. On
    failure, it will return -1 (``ERROR``) and will set the ``errno`` value
    appropriately.

  **POSIX Compatibility:** Similar with the POSIX interface of the same
  name. There are, however, several compatibility issues as detailed in
  the description of `execv() <#execv>`__.

.. c:function:: int posix_spawn(FAR pid_t *pid, FAR const char *path, \
    FAR const posix_spawn_file_actions_t *file_actions, \
    FAR const posix_spawnattr_t *attr, \
    FAR char * const argv[], FAR char * const envp[])

.. c:function:: int posix_spawnp(FAR pid_t *pid, FAR const char *file, \
    FAR const posix_spawn_file_actions_t *file_actions, \
    FAR const posix_spawnattr_t *attr, \
    FAR char * const argv[], FAR char * const envp[]);

  The ``posix_spawn()`` and ``posix_spawnp()`` functions
  will create a new, child task, constructed from a regular executable
  file.

  :param pid: Upon successful completion, ``posix_spawn()`` and
    ``posix_spawnp()`` will return the task ID of the child task to the
    parent task, in the variable pointed to by a non-NULL ``pid``
    argument. If the ``pid`` argument is a null pointer, the process ID
    of the child is not returned to the caller.

  :param path: The ``path`` argument to ``posix_spawn()`` is
    the absolute path that identifies the file to execute. The ``file``
    argument to ``posix_spawnp()`` may also be a relative path and will
    be used to construct a pathname that identifies the file to execute.
    In the case of a relative path, the path prefix for the file will be
    obtained by a search of the directories passed as the environment
    variable PATH.

    NOTE: NuttX provides only one implementation: If
    ``CONFIG_LIBC_ENVPATH`` is defined, then only ``posix_spawnp()``
    behavior is supported; otherwise, only ``posix_spawn`` behavior is
    supported.

  :param file_actions: If ``file_actions`` is a null pointer, then file
    descriptors open in the calling process will remain open in the child
    process (unless ``CONFIG_FDCLONE_STDIO`` is defined). If
    ``file_actions`` is not NULL, then the file descriptors open in the
    child process will be those open in the calling process as modified
    by the spawn file actions object pointed to by ``file_actions``.

  :param attr: If the value of the ``attr`` parameter is ``NULL``, the all
    default values for the POSIX spawn attributes will be used.
    Otherwise, the attributes will be set according to the spawn flags.
    The ``posix_spawnattr_t`` spawn attributes object type is defined in
    ``spawn.h``. It will contains these attributes, not all of which are
    supported by NuttX:

    -  ``POSIX_SPAWN_SETPGROUP``: Setting of the new task's process group
       is not supported. NuttX does not support process groups.
    -  ``POSIX_SPAWN_SETSCHEDPARAM``: Set new tasks priority to the
       ``sched_param`` value.
    -  ``POSIX_SPAWN_SETSCHEDULER``: Set the new task's scheduler policy
       to the ``sched_policy`` value.
    -  ``POSIX_SPAWN_RESETIDS`` Resetting of the effective user ID of the
       child process is not supported. NuttX does not support effective
       user IDs.
    -  ``POSIX_SPAWN_SETSIGMASK``: Set the new task's signal mask.
    -  ``POSIX_SPAWN_SETSIGDEF``: Resetting signal default actions is not
       supported. NuttX does not support default signal actions.

  :param argv: ``argv[]`` is the argument list for the new task.
    ``argv[]`` is an array of pointers to null-terminated strings. The
    list is terminated with a null pointer.

  :param envp: The ``envp[]`` argument is not used by NuttX and may be
    ``NULL``. In standard implementations, ``envp[]`` is an array of
    character pointers to null-terminated strings that provide the
    environment for the new process image. The environment array is
    terminated by a null pointer. In NuttX, the ``envp[]`` argument is
    ignored and the new task will inherit the environment of the parent
    task unconditionally.

  :return: Zero on success. Otherwise, an error number will be returned as the
    function return value to indicate the error:

    -  ``EINVAL``: The value specified by ``file_actions`` or ``attr`` is
       invalid.
    -  Any errors that might have been return if ``vfork()`` and
       ``exec[l|v]()`` had been called.

  **Assumptions/Limitations:**

  -  NuttX provides only ``posix_spawn()`` or ``posix_spawnp()`` behavior
     depending upon the setting of ``CONFIG_LIBC_ENVPATH``: If
     ``CONFIG_LIBC_ENVPATH`` is defined, then only ``posix_spawnp()``
     behavior is supported; otherwise, only ``posix_spawn()`` behavior is
     supported.
  -  The ``envp`` argument is not used and the ``environ`` variable is not
     altered (NuttX does not support the ``environ`` variable).
  -  Process groups are not supported (See ``POSIX_SPAWN_SETPGROUP``
     above).
  -  Effective user IDs are not supported (See ``POSIX_SPAWN_RESETIDS``
     above).
  -  Signal default actions cannot be modified in the newly task executed
     because NuttX does not support default signal actions (See
     ``POSIX_SPAWN_SETSIGDEF``).

  **POSIX Compatibility:** The value of the ``argv[0]`` received by the
  child task is assigned by NuttX. For the caller of ``posix_spawn()``,
  the provided argv[0] will correspond to ``argv[1]`` received by the new
  task.

.. c:function:: int posix_spawn_file_actions_init(FAR posix_spawn_file_actions_t *file_actions)

  Initializes the object referenced by ``file_actions`` to an empty set of
  file actions for subsequent use in a call to ``posix_spawn()`` or
  ``posix_spawnp()``.

  **Input Parameters:**

  -  ``file_actions``: The address of the ``posix_spawn_file_actions_t``
     to be initialized.

  **Returned Value:** On success, this function returns 0; on failure it
  will return an error number from ``<errno.h>``.

.. c:function:: int posix_spawn_file_actions_destroy(FAR posix_spawn_file_actions_t *file_actions)

  Destroys the object referenced by ``file_actions`` which was previously
  initialized by ``posix_spawn_file_actions_init()``, returning any
  resources obtained at the time of initialization to the system for
  subsequent reuse. A ``posix_spawn_file_actions_t`` may be reinitialized
  after having been destroyed, but must not be reused after destruction,
  unless it has been reinitialized.

  :param file_actions: The address of the ``posix_spawn_file_actions_t``
    to be destroyed.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawn_file_actions_addclose(FAR posix_spawn_file_actions_t *file_actions, int fd)

  Adds a *close* operation to the list of operations associated with the
  object referenced by ``file_actions``, for subsequent use in a call to
  ``posix_spawn()`` or ``posix_spawnp()``. The descriptor referred to by
  ``fd`` is closed as if ``close()`` had been called on it prior to the
  new child process starting execution.

  :param file_actions: The address of the ``posix_spawn_file_actions_t``
     object to which the *close* operation will be appended.
  :param fd: The file descriptor to be closed.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawn_file_actions_adddup2(FAR posix_spawn_file_actions_t *file_actions, int fd1, int fd2)

  Adds a *dup2* operation to the list of operations associated with the
  object referenced by ``file_actions``, for subsequent use in a call to
  ``posix_spawn()`` or ``posix_spawnp()``. The descriptor referred to by
  ``fd2`` is created as if ``dup2()`` had been called on ``fd1`` prior to
  the new child process starting execution.

  :param file_actions: The address of the ``posix_spawn_file_actions_t``
     object to which the *dup2* operation will be appended.
  :param fd1: The file descriptor to be be duplicated. The first file
     descriptor to be argument to ``dup2()``.
  :param fd2: The file descriptor to be be created. The second file
     descriptor to be argument to ``dup2()``.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawn_file_actions_addopen(FAR posix_spawn_file_actions_t *file_actions, \
    int fd, FAR const char *path, int oflags, mode_t mode);

  Adds an *open* operation to the list of operations associated with the
  object referenced by ``file_actions``, for subsequent use in a call to
  ``posix_spawn()`` or ``posix_spawnp()``. The descriptor referred to by
  ``fd`` is opened using the ``path``, ``oflag``, and ``mode`` arguments
  as if ``open()`` had been called on it prior to the new child process
  starting execution. The string path is copied by the
  ``posix_spawn_file_actions_addopen()`` function during this process, so
  storage need not be persistent in the caller.

  :param file_actions: The address of the ``posix_spawn_file_actions_t``
     object to which the *open* operation will be appended.
  :param fd: The file descriptor to be opened.
  :param path: The path to be opened.
  :param oflags: Open flags.
  :param mode: File creation mode/
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_init(FAR posix_spawnattr_t *attr)

  The ``posix_spawnattr_init()`` function initializes the
  object referenced by ``attr``, to an empty set of spawn attributes for
  subsequent use in a call to ``posix_spawn()`` or ``posix_spawnp()``.

  Then the spawn attributes are no longer needed, they should be destroyed
  by calling ``posix_spawnattr_destroyed()``. In NuttX, however,
  ``posix_spawnattr_destroyed()`` is just stub:

  For portability, the convention of calling
  ``posix_spawnattr_destroyed()`` when the attributes are not longer
  needed should still be followed.

  :param attr: The address of the spawn attributes to be initialized.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_getflags(FAR const posix_spawnattr_t *attr, FAR short *flags)

  The ``posix_spawnattr_getflags()`` function will obtain
  the value of the *spawn-flags* attribute from the attributes object
  referenced by ``attr``.

  :param attr: The address spawn attributes to be queried.
  :param flags: The location to return the spawn flags
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_getschedparam(FAR const posix_spawnattr_t *attr, FAR struct sched_param *param)

  The ``posix_spawnattr_getschedparam()`` function will
  obtain the value of the *spawn-schedparam* attribute from the attributes
  object referenced by ``attr``.

  :param attr: The address spawn attributes to be queried.
  :param param: The location to return the *spawn-schedparam* value.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_getschedpolicy(FAR const posix_spawnattr_t *attr, FAR int *policy)

  The ``posix_spawnattr_getschedpolicy()`` function will
  obtain the value of the *spawn-schedpolicy* attribute from the
  attributes object referenced by ``attr``.

  :param attr: The address spawn attributes to be queried.
  :param policy: The location to return the *spawn-schedpolicy* value.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_getsigmask(FAR const posix_spawnattr_t *attr, FAR sigset_t *sigmask)

  ``posix_spawnattr_getsigdefault()`` function will
  obtain the value of the *spawn-sigmask* attribute from the attributes
  object referenced by ``attr``.

  :param attr: The address spawn attributes to be queried.
  :param sigmask: The location to return the *spawn-sigmask* value.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_setflags(FAR posix_spawnattr_t *attr, short flags)

  The ``posix_spawnattr_setflags()`` function will set
  the *spawn-flags* attribute in an initialized attributes object
  referenced by ``attr``.

  :param attr: The address spawn attributes to be used.
  :param flags: The new value of the *spawn-flags* attribute.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_setschedparam(FAR posix_spawnattr_t *attr, FAR const struct sched_param *param)

  The ``posix_spawnattr_setschedparam()`` function will
  set the *spawn-schedparam* attribute in an initialized attributes object
  referenced by ``attr``.

  :param attr: The address spawn attributes to be used.
  :param param: The new value of the *spawn-schedparam* attribute.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_setschedpolicy(FAR posix_spawnattr_t *attr, int policy)

  The ``posix_spawnattr_setschedpolicy()`` function will
  set the *spawn-schedpolicy* attribute in an initialized attributes
  object referenced by ``attr``.

  :param attr: The address spawn attributes to be used.
  :param policy: The new value of the *spawn-schedpolicy* attribute.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_setsigmask(FAR posix_spawnattr_t *attr, FAR const sigset_t *sigmask)

  The ``posix_spawnattr_setsigmask()`` function will set
  the *spawn-sigmask* attribute in an initialized attributes object
  referenced by ``attr``.

  :param attr: The address spawn attributes to be used.
  :param sigmask: The new value of the *spawn-sigmask* attribute.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int task_spawn(FAR const char *name, main_t entry, \
      FAR const posix_spawn_file_actions_t *file_actions, \
      FAR const posix_spawnattr_t *attr, \
      FAR char * const argv[], FAR char * const envp[])

  The ``task_spawn()`` function will create a new, child
  task, where the entry point to the task is an address in memory.

  :param name: The name to assign to the child task.

  :param entry: The child task's entry point (an address in memory).

  :param file_actions: If ``file_actions`` is a null pointer, then file
     descriptors open in the calling process will remain open in the child
     process (unless ``CONFIG_FDCLONE_STDIO`` is defined). If
     ``file_actions`` is not NULL, then the file descriptors open in the
     child process will be those open in the calling process as modified
     by the spawn file actions object pointed to by ``file_actions``.

  :param attr: If the value of the ``attr`` parameter is ``NULL``, the all
     default values for the POSIX spawn attributes will be used.
     Otherwise, the attributes will be set according to the spawn flags.
     The ``posix_spawnattr_t`` spawn attributes object type is defined in
     ``spawn.h``. It will contains these attributes, not all of which are
     supported by NuttX:

     -  ``POSIX_SPAWN_SETPGROUP``: Setting of the new task's process group
        is not supported. NuttX does not support process groups.
     -  ``POSIX_SPAWN_SETSCHEDPARAM``: Set new tasks priority to the
        ``sched_param`` value.
     -  ``POSIX_SPAWN_SETSCHEDULER``: Set the new task's scheduler policy
        to the ``sched_policy`` value.
     -  ``POSIX_SPAWN_RESETIDS`` Resetting of the effective user ID of the
        child process is not supported. NuttX does not support effective
        user IDs.
     -  ``POSIX_SPAWN_SETSIGMASK``: Set the new task's signal mask.
     -  ``POSIX_SPAWN_SETSIGDEF``: Resetting signal default actions is not
        supported. NuttX does not support default signal actions.

     And the non-standard:

     -  ``TASK_SPAWN_SETSTACKSIZE``: Set the stack size for the new task.

  :param argv: ``argv[]`` is the argument list for the new task.
     ``argv[]`` is an array of pointers to null-terminated strings. The
     list is terminated with a null pointer.

  :param envp: The ``envp[]`` argument is not used by NuttX and may be
     ``NULL``.

  :return: ``task_spawn()`` will return process ID of new task on success.
    Otherwise, a negative number will be returned as the function return
    value to indicate the error:

  **POSIX Compatibility:** This is a non-standard interface inspired by
  ``posix_spawn()``.

.. c:function:: int posix_spawnattr_getstacksize(FAR const posix_spawnattr_t *attr, FAR size_t *stacksize)

  The ``posix_spawnattr_getstacksize()`` function will
  obtain the value of the *spawn-stacksize* attribute from the attributes
  object referenced by ``attr``.

  :param attr: The address spawn attributes to be queried.
  :param policy: The location to return the *spawn-stacksize* value.

  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``

.. c:function:: int posix_spawnattr_setstacksize(FAR posix_spawnattr_t *attr, size_t stacksize)

  The ``posix_spawnattr_setstacksize()`` function will set
  the *spawn-stacksize* attribute in an initialized attributes object
  referenced by ``attr``.

  :param attr: The address spawn attributes to be used.
  :param policy: The new value of the *spawn-stacksize* attribute.
  :return: On success, this function returns 0; on failure it
    will return an error number from ``<errno.h>``
