=======================================================
Why can't I put my special stuff in NuttX header files?
=======================================================

.. warning::
    Migrated from: https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629466

The Problem
===========

I am very picky about what goes into NuttX header files. I don't accept 
non-standardized changes to go into them just to permit external code to 
compile; nor do I accept changes that imply something is implemented in 
NuttX when it is not (although there are cases like that in the header 
files now).

* But I want to use `newlib` logic that depends on certain definitions on the 
  Nuttx header files!
* But I am trying to compile an application that depends on non-standard 
  declarations in header files! Or prototypes for functions that are not 
  provided by NuttX! Or types that are not used by NuttX!

You will find that I am very stubborn on this subject and you will be wasting 
your time and energy if you try to get kruft included into NuttX header files 
for your personal purposes.

A Work-Around
=============

But there is a work-around for my pickiness and stubborn-ness (at least for 
compilers like GCC that support the GNU extensions). Let's suppose you wanted 
to add this definition:

.. code-block:: c

    #define I_AM_A_NERD true

to the standard ``time.h`` header file. You submitted a patch to do this and 
I refused it. Now what?

While I refuse to put non-standard or useless stuff in NuttX header files, 
there are ways to work around this. Suppose that you create a directory 
called ``myincludes/`` and in your ``myincludes/`` directory is a header called 
``time.h``. This ``time.h`` header file consists of:

.. code-block:: C

    #define I_AM_A_NERD true
    #include_next <time.h>

Then in your ``CFLAGS``, you use an ``-isystem`` setting to include header 
files from ``myincludes/`` before any header files from the NuttX ``include/`` 
directory. Then when your application includes ``time.h``, the version of 
``time.h`` in ``myincludes/`` is the one that will be included. That version 
will define ``I_AM_A_NERD`` as you want and then include the next file named 
``time.h`` in the compiler's include path. That file will be the standard 
``time.h`` header file that is provided in the NuttX ``include/`` directory

In this way you an append or modify any of the NuttX header files to suit 
your own purposes without my having to accept changes that I do not want 
into the NuttX repository.

When Does It Make Sense?
========================

When does it make sense to add new definitions, types, and function prototypes 
to the NuttX header files? Only under the following conditions:

* The changes are standard and specified in OpenGroup.org
* The changes are provided by a patch that includes the full, verified 
  implementation of the feature that uses the types and implements the 
  functions.