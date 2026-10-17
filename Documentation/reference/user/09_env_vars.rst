=====================
Environment Variables
=====================

NuttX supports environment variables that can be used to
control the behavior of programs. In the spirit of NuttX the environment
variable behavior attempts to emulate the behavior of environment
variables in the multi-processing OS:

-  **Task environments**. When a new task is created using
   `task_create <#taskcreate>`__, the environment of the child task is
   an inherited, exact copy of the environment of the parent. However,
   after child task has been created, subsequent operations by the child
   task on its environment does not alter the environment of the parent.
   Nor do operations by the parent effect the child's environment. The
   environments start identical but are independent and may diverge.
-  **Thread environments**. When a pthread is created using
   `pthread_create <#pthreadcreate>`__, the child thread also inherits
   that environment of the parent. However, the child does not receive a
   copy of the environment but, rather, shares the same environment.
   Changes to the environment are visible to all threads with the same
   parentage.

Programming Interfaces
======================

The following environment variable
programming interfaces are provided by NuttX and are described in detail
in the following paragraphs.

  - :c:func:`getenv`
  - :c:func:`putenv`
  - :c:func:`clearenv`
  - :c:func:`setenv`
  - :c:func:`unsetenv`

Disabling Environment Variable Support
======================================

All support for environment
variables can be disabled by setting ``CONFIG_DISABLE_ENVIRON`` in the
board configuration file.

The ``environ`` Variable
========================

``environ`` points to the environment of the calling task group: an array
of ``name=value`` strings that ends with a ``NULL`` pointer. Each task
group has its own environment, so ``<stdlib.h>`` defines ``environ`` as a
macro, ``(*get_environ_ptr())``. NuttX has no ``environ`` symbol: include
``<stdlib.h>`` to use ``environ``. A program can read the array, change an
entry in place, or assign its own array to ``environ``. The functions
below then work on that array. With ``CONFIG_DISABLE_ENVIRON``,
``environ`` is ``NULL``.

An ELF program can also get the environment as the third argument of
``main()``:

.. code-block:: c

  int main(int argc, FAR char *argv[], FAR char *envp[]);

The ``crt0`` of NuttX passes ``environ`` there on arm, arm64, risc-v,
x86_64 and xtensa (``NULL`` with ``CONFIG_DISABLE_ENVIRON``). A program
that is not started through ``crt0``, for example a builtin application,
gets only ``argc`` and ``argv``.

The environment functions are in the C library, not system calls. Each
environment is in the heap that holds the task information of its task
group: the user heap for a user task, the kernel heap for a kernel thread.

The C library frees no environment string that it allocated until the
task group exits. So a pointer that :c:func:`getenv` returned stays valid
after :c:func:`setenv`, :c:func:`putenv`, :c:func:`unsetenv` or
:c:func:`clearenv` changes the environment. A program that sets one
variable many times keeps the old values until it exits.

Functions
=========

.. c:function:: FAR char *getenv(const char *name)

  Searches the environment list for a string that matches
  the string pointed to by ``name``.

  :param name: The name of the variable to find.

  :return:
    The value of the variable (read-only) or NULL on failure.

.. c:function:: int putenv(char *string)

  Adds or changes the value of environment variables. The
  argument string is of the form ``name=value``. If name does
  not already exist in the environment, then string is added
  to the environment. If name does exist, then the value of name in the
  environment is changed to value.

  :param string: ``name=value`` string describing the environment setting to
     add/modify.

  :return: Zero on success.

.. c:function:: int clearenv(void)

  Clears the environment of all name-value pairs and sets
  the value of the external variable environ to NULL.

  :return: Zero on success.

.. c:function:: int setenv(const char *name, const char *value, int overwrite)

  Adds the variable ``name`` to the environment with the specified
  ``value`` if the variable ``name`` does not exist. If the ``name``
  does exist in the environment, then its value is changed to ``value``
  if ``overwrite`` is non-zero; if
  ``overwrite`` is zero, then the value of ``name`` is unaltered.

  :param name: The name of the variable to change.
  :param value: The new value of the variable.
  :param value: Replace any existing value if non-zero.
  :return: Zero on success.

.. c:function:: int unsetenv(const char *name)

  Deletes the variable ``name`` from the environment.

  :param name: The name of the variable to delete.
  :return: Zero on success.
