.. _file-descriptors:

================
File Descriptors
================

Drivers within Drivers
======================

Have you ever wanted to write a driver the uses another driver?
For example, suppose you want to develop a character driver for a module
that provides a serial interface to the host. Wouldn't you like to be able to
open and use the serial driver within your module driver?

Or suppose the you have a device with analog inputs such as joystick
and you would like to use the ADC character driver to sample the joystick?
Would you not want to contain the ADC character driver within the joystick
driver?


Drivers File Descriptors
========================

One obstacle to containing a driver instance within another driver
is the nature of the file descriptor. When you open a file or a driver from
any thread in a task, you receive a file descriptor that you then may use
in the task to access the device.

In the NuttX implementation, that file descriptor is really an index into
a pre-allocated array of type struct file.
It is really the underlying struct file instance referred to by the
file descriptor that use used to access the device.
The file descriptor/index is simply a portable way to reference the struct
file instance.

.. note:: In NuttX, threads are organized into task groups, see
          :ref:`Tasks vs. Threads <tasks-vs-threads>` for more details.

Each task group has its own array of struct file. All of the threads within
the same task group share the same array allocation. Therefore, the same file
descriptor index is valid for all threads within the task group.

But, on the other hand, threads running within a different task group
use a different array of struct file and file descriptors opened
in this other task group have no relationship.
In short, file descriptors cannot be used by different tasks.

A driver must not be associated with any specific task group's resources.
That would make the driver dependent upon that task group. Any task should be
able top open and use the driver. Therefore, the driver cannot use
file descriptors to access the contained driver instance.

So, if you cannot use file descriptors within a device drivers, how can
one driver contain another driver instance? The answer is by detaching
the file descriptor from the open file instance.


Detaching Open Files
====================

There is an internal OS interface called ``file_detach()`` that is prototyped
and described in the header file ``include/nuttx/fs/fs.h``:

.. code-block:: c

  /****************************************************************************
   * Name: file_detach
   *
   * Description:
   *   This function is used to device drivers to create a task-independent
   *   handle to an entity in the file system.  file_detach() duplicates the
   *   'struct file' that underlies the file descriptor, then closes the file
   *   descriptor.
   *
   *   This function will fail if fd is not a valid file descriptor.  In
   *   particular, it will fail if fd is a socket descriptor.
   *
   * Input Parameters:
   *   fd    - The file descriptor to be detached.  This descriptor will be
   *           closed and invalid if the file was successfully detached.
   *   filep - A pointer to a user provided memory location in which to
   *           received the duplicated, detached file structure.
   *
   * Returned Value:
   *   Zero (OK) is returned on success; A negated errno value is returned on
   *   any failure to indicate the nature of the failure.
   *
   ****************************************************************************/
  
  #if CONFIG_NFILE_DESCRIPTORS > 0
  int file_detach(int fd, FAR struct file *filep);
  #endif

In order to use ``file_detach()`` you would do the following:

* Allocate an instance of struct file either dynamically or statically.
  This instance will represent the contained driver.
* Open the file or driver to get a file descriptor
* Call ``file_detach()`` in order to clone the underlying struct file to your
  allocated instance. Detaching the file descriptor has the side-effect
  of closing the original instance.
* You can then use the driver methods of the contained driver directly from
  your driver. These driver methods are described in ``include/nuttx/fs/fs.h``.

There is also a ``file_close_detached()`` internal OS function that you can use
to recover resources if you no longer need the contained driver instance:

.. code-block:: c

  /****************************************************************************
   * Name: file_close_detached
   *
   * Description:
   *   Close a file that was previously detached with file_detach().
   *
   * Input Parameters:
   *   filep - A pointer to a user provided memory location containing the
   *           open file data returned by file_detach().
   *
   * Returned Value:
   *   Zero (OK) is returned on success; A negated errno value is returned on
   *   any failure to indicate the nature of the failure.
   *
   ****************************************************************************/
  
  int file_close_detached(FAR struct file *filep);

This technique is not limited to character drivers but can also be used
with files in a file system or even block drivers.


Detached File Helpers
=====================

Once the file structure has been detached from its file descriptor,
you can no longer use the standard VFS functions ``read()``, ``write()``,
``ioctl()``, etc. Fortunately, there are a parallel set of interfaces
that can be used with detached files. These are decribed in detail
in ``include/nuttx/fs/fs.h`` and only listed here below:

.. code-block:: c

    ssize_t file_read(FAR struct file *filep, FAR void *buf, size_t nbytes);
    ssize_t file_write(FAR struct file *filep, FAR const void *buf, size_t nbytes);
    ssize_t file_pread(FAR struct file *filep, FAR void *buf, size_t nbytes,
                       off_t offset);
    ssize_t file_pwrite(FAR struct file *filep, FAR const void *buf,
                        size_t nbytes, off_t offset);
    off_t file_seek(FAR struct file *filep, off_t offset, int whence);
    int file_ioctl(FAR struct file *filep, int req, unsigned long arg);
    int file_fsync(FAR struct file *filep);
    int file_dup2(FAR struct file *filep1, FAR struct file *filep2);
    int file_vfcntl(FAR struct file *filep, int cmd, va_list ap);


The SYLOG Device: A Case Study
==============================

This technique is used for the SYSLOG device. Originally, NuttX used
file descriptor ``1`` for SYSLOG output by default. For most task groups,
file descriptor ``1`` (``stdout``) mapped to ``/dev/console`` and this
solution worked well in most cases.

But using file descriptor ``1`` caused some very strange results when
``stdout`` was redirected such as when a USB or a Telnet console was used.
In those cases, SYSLOG output would go to different places depending upon
how stdout was re-directed within a given task group.

This was fixed using ``file_detach()``. In the default case, the SYSLOG
initialization logic now opens ``/dev/console`` then calls ``file_detach()``
to disassociate the open file instance from any task group.
Now, SYSLOG output consistently goes to ``/dev/console`` regardless of how
file descriptor ``1`` may be re-directed when SYSLOG output is generated.


Other Examples
==============

There are some other examples in analog joystick lower half drivers
that use the ADC character driver to read joystick positions::

  boards/arm/stm32/nucleo-f4x1re/src/stm32_ajoystick.c:
    ret = file_detach(fd, &g_adcfile);

  boards/arm/stm32l4/nucleo-l476rg/src/stm32_ajoystick.c:
    ret = file_detach(fd, &g_adcfile);

  boards/arm/sama5/sama5d3-xplained/src/sam_ajoystick.c:
    ret = file_detach(fd, &g_adcfile);


Socket Descriptors
==================

There is a similar story for socket descriptors but this is probably
not the place for that whole story. Instead, we will just summarize
that story here. First, let's compare file and socket descriptors here.
Socket descriptors are similar to file descriptors in that:

* They are a numeric index into a task-private array of structures.
  For the case of socket descriptors, this is an array of type struct socket.
* Like the file descriptor, the socket descriptor is simply a portable way
  to reference the underlying socket structure.
* And also like the file descriptor, the socket descriptor is meaningful
  only in the context of task group where the socket descriptor was created.

As in the case for file descriptors, there are also a set of socket interfaces
that accept a reference to struct socket as an input parameter.
For each standard socket interface that accepts a socket descriptor,
there is a corresponding non-standard, OS-internal interface that accepts
a reference to struct socket as as an input parameters. These low-level socket
interfaces all have the prefix ``psock_`` and are all prototyped in the file
``include/nuttx/net/net.h``. They are not discussed further here.

These low-level socket interfaces are also independent of task groups
and may be used without regard to the thread that created the socket.
The primary usage difference is in how the struct socket instances are created:

* In the case of struct file, there is no special ``file_open()`` interface
  to create the detached file reference. Instead, a two step procedure is used:
  First the file is opened the standard ``open()`` function to obtain the
  file descriptor, then the struct file is detached from the file descriptor
  using ``file_detach()``.
* The struct socket instance, on the other hand, is created in the detached
  state directly using ``psock_socket()``.

You can find a good example of a character driver that contains
a task-independent socket interface in the Telnet character driver at
``driver/net/telnet.c``. That driver is used to encapsulate a Telnet
session and to provide Telnet ``stdin``, ``stdout``, and ``stderr``
for remote NSH sessions.
