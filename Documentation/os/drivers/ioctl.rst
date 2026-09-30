.. _ioctl:

=====
ioctl
=====
 
Not a Typewriter
================

  "In computing, **Not a typewriter** or ``ENOTTY`` is an error code defined
  in the ``errno.h`` found on many Unix systems.
  This code is now used to indicate that
  an invalid ``ioctl`` (input/output control) number was specified
  in an ``ioctl`` system call."

  --Source: https://en.wikipedia.org/wiki/Not_a_typewriter.


NuttX
=====

In ``ioctl()`` implementations in NuttX, ``-ENOTTY`` is always returned
if the ``ioctl()`` command is not recognized.  You will often see driver
``ioctl()`` implement ions with a general structure similar to the following:

.. code:: c

   int driver_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
   {
   int ret;

   switch (cmd)
     {
       ...

       default:
         ret = -ENOTTY;
         break;
       }
     }

   return ret;
   }

Note that ``-ENOTTY`` is returned internally in NuttX.
This will subsequently be used to set the errno value to ``ENOTTY``
and to ``return -1`` to indicate the error condition.

ERRORS
------

These are the return values from a Linux ``ioctl()`` call:

* ``EBADF`` fd is not a valid file descriptor.
* ``EFAULT`` argp references an inaccessible memory area.
* ``EINVAL`` request or argp is not valid.
* ``ENOTTY`` fd is not associated with a character special device.
* ``ENOTTY`` The specified request does not apply to the kind of object
  that the file descriptor fd references.

Reference: https://www.man7.org/linux/man-pages/man2/ioctl.2.html.


Linux Explanation
=================

On Jun 27 Linus Torvalds wrote:

  "The correct error code for "I don't understand this ioctl" is ENOTTY.
  The naming may be odd, but you should think of that error value as a
  "unrecognized ioctl number, you're feeding me random numbers that I
  don't understand and I assume for historical reasons that you tried to
  do some tty operation on me".
  ...
  The EINVAL thing goes way back, and is a disaster. It predates Linux
  itself, as far as I can tell. You'll find lots of man-pages that have
  this line in it:

    EINVAL Request or argp is not valid.

  and it shows up in POSIX etc. And sadly, it generally shows up
  _before_ the line that says

    ENOTTY The specified request does not apply to the kind of object
    that the descriptor d references.

  so a lot of people get to the EINVAL, and never even notice the ENOTTY.

  (..)

  At least glibc (and hopefully other C libraries) use a _string_ that
  makes much more sense: strerror(ENOTTY) is "Inappropriate ioctl for
  device"."

  --Source: https://lore.kernel.org/patchwork/patch/258361.


How is this useful?

Knowing that no error occurred but the ``ioctl()`` command was not recognized
is a useful piece of information.
Suppose, for example, I have nfds open character drivers in an array ``fd[]``.
Then I could do something like this:

.. code:: c

   int do_command(FAR int *fd, int nfds, int cmd)
   {
     int ret;
     int i;

     /* Try all file descriptors */

     for (i = 0; i < nfds; i++)
       {
         ret = ioctl(fd[i], cmd, 0ul);  /* No argument in this example */
         if (ret < 0)
           {
             int errcode = errno;

             /* Try the next file descriptor if this one return ENOTTY */

             if (errcode != ENOTTY)
               {
                 /* Other errors, including EINVAL, are fatal */

                 return -errcode
               }
           }
         else if (ret >= 0)
           {
             return OK;  /* Success! */
           }
       }

     return -ENOENT;
   }
