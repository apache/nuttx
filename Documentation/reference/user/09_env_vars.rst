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
   No do operations by the parent effect the child's environment. The
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
