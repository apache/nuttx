==============================
Linux Processes vs NuttX Tasks
==============================

You may be used to running programs that are stored in files on Linux or Windows.
If you transition to using NuttX tasks on an MCU with limited resources, you will
encounter some behavioral differences. This Wiki page will summarize a few of
those differences.

NuttX Build Types
=================

NuttX can be built in several different ways:

* **Kernel Build** The kernal build, selected with ``CONFIG_BUILD_KERNEL``, uses
  the MCU's Memory Management Unit (MMU) to implement processes very similar to
  Linux processes. There is no interesting discussion here; NuttX behaves very
  much like Linux.

* **Flat Build** Most resource-limited MCUs have no MMU and the code is built as
  a blob that runs in an unprotected, flat address space out of on-chip FLASH
  memory. This build mode is selected with ``CONFIG_BUILD_FLAT`` and is, by far, the
  most common way that people build NuttX. This is the interesting case to which
  this Wiki page is directed.

* **Protected Build** Another build option is the protected build. This is essentially
  the same as the flat build, but uses the MCU's Memory Protection Unit (MPU) to
  separate unproctect user address ranges from protected system address ranges.
  The comments of this Wiki page also apply in this case.

Initialization of Global Variables
==================================

Linux Behavior
--------------

If you are used to writing programs for Linux, then one thing you will notice is that
global variables are initialized only once when the system powers up. For example.
Consider this tiny program:

.. code-block:: C
                
   bool test = true;
 
   int main(int argc, char **argv)
   {
     printf("test: %i\n", test);
     test = false;
     printf("test: %i\n", test);
     return 0;
   }

If you build and run this program under Linux, you will always see this output::

  test: 1
  test: 0

In this case, the global variables are re-initialized each time that you load the
file into memory and run it.

NuttX Flat-Build Behavior
-------------------------

But if you build this program into on-chip FLASH and start it as a task (via, say,
``task_start()``) you will see this the first time that you run the program::

  test: 1
  test: 0

But after that, you will always see::

  test: 0
  test: 0

The test variable was initialized to true (1) only once at power up, but reset to
false (0) each time that the program runs.

If you want the same behavior when the program is built into the common FLASH blob,
then you will have modify the code so that global variables are explicitly reset
each time the program runs like:

.. code-block:: C

   bool test;
   
   int main(int argc, char **argv)
   {
     test = true;
     printf("test: %i\n", test);
     test = false;
     printf("test: %i\n", test);
     return 0;
   }

NuttX Load-able Programs
------------------------

If you load programs from an file into RAM and execute them, as Linux does, then
NuttX will again behave like Linux. Because the flat build NuttX works the same way:
When you execute a NuttX ELF or NxFLAT module in a file, the file is copied into RAM
and the global variables are initialized before the program runs.

But code that is built into FLASH works differently. There is only one set of global
variables: All of the global variables for the blob that is the monolithic FLASH image.
They are all initialized  once at power-up reset.

This is one of the things that makes porting Linux applications into the FLASH blob
more complex. You have to manually initialize each global variable in the ``main()``
each time your start the task.

Global Variables and Multiple Task Copies
=========================================

It is better to avoid the use of global variables in the flat build context whenever
possible because that usage adds another limitation:  No more that one copy of the
program can run at any given time.  That is because the global variables are shared
by each instance (unlike, again, running a program from a file where there is a private
copy of each global variable).

One way to support multiple copies of an in-FLASH program is to move all global variables
into a structure. If the amount of memory need for global variables is small, then each
``main()`` could simply allocate a copy of that structure on the stack. In the simple
example above, this might be:

.. code-block:: C

   struct my_globals_s
   {
     bool test;
   };
 
   int main(int argc, char **argv)
   {
     struct my_globals_s my_globals = { true };
 
     printf("test: %i\n", my_globals.test);
     my_globals.test = false;
     printf("test: %i\n", my_globals.test);
     return EXIT_SUCCESS;
   }

A pointer to the structure containing the allocated global variables would then have
to passed as a parameter to every internal function that needs access to the global
variables. So you would change a internal function like:

.. code-block:: C

   static void print_value(void)
   {
     printf("test: %i\n", test);
   }

to:

.. code-block:: C

   static void print_value(FAR struct my_globals_s *globals)
   {
     printf("test: %i\n", globals->test);
   }

Then pass a reference to the allocated global data structure each time that the
function is called like:

.. code-block:: C

   print_value(&my_globals);

If the size of the global variable structure is large, then allocating the instance
on the stack might not be such a good idea. In that case, it might be better to
allocate the global variable structure using ``malloc()``. But don't forget to ``free()``
the allocated variable structure before exiting! (See the following Memory Clean-Up
discussion).

.. code-block:: C

   struct my_globals_s
   {
     bool test;
   };
 
   int main(int argc, char **argv)
   {
     FAR struct my_globals_s *my_globals;
 
     my_globals = (FAR struct my_globals_s *)malloc(sizeof(struct my_globals_s));
     if (my_globals = NULL)
       {
         fprintf(stderr, "ERROR: Failed to allocate state structure\n");
         return EXIT_FAILURE;
       }
 
     my_globals=>test = true;
     printf("test: %i\n", my_globals->test);
     my_globals=>test = false;
     printf("test: %i\n", my_globals->test);
 
     free(my_globals);
     return EXIT_SUCCESS;
   }

Memory Clean-Up
===============

Linux Process Exit
------------------

Another, unrelated thing that makes porting Linux programs into the FLASH blob
is the memory clean-up.  When a Linux process exits, its entire address environment
is destroyed including all of allocated memory. This tiny program will not leak
memory if implemented as a Linux process:

.. code-block:: C

   int main(int argc, char **argv)
   {
     char *buffer = malloc(1024);
     ... do stuff with buffer ...
     return 0;
   }

That same program, if ported into the FLASH blob will now have memory leaks because
there is no automatic clean-up of allocated memory when the task exits. Instead, you
must explicitly clean up all allocated memory by freeing it:

.. code-block:: C

   int main(int argc, char **argv)
   {
     char *buffer = malloc(1024);
     ... do stuff with buffer ...
     free(buffer);
     return 0;
   }

The memory clean-up with the Linux process exits is a consequent of the teardown of
the process address environment when the process terminates. Each process contains
its own heap; when the process address environment is torndown, that process heap
is returned to the OS page allocator. So the memory clean-up basically comes for free.

NuttX Task Exit
---------------

But when you run a task in the monolithic, on-chip FLASH blob, you share the same
heap with all other tasks. There is no magic clean-up that can find and free your
tasks's allocations within the common heap (see "Ways to Free Memory on Task Exit").

NuttX Process Exit
------------------

NOTE that when you run processes on NuttX (with ``CONFIG_BUILD_KERNEL``), NuttX also
behaves the same way as Linux:  The address environment is destroyed with the task
exits and all of the memory is reclaimed.  But all other cases will leak memory.

Ways to Free Memory on Task Exit
--------------------------------

There are ways that you could associate allocated memory with a task so that it could
cleaned up when the task exits. That approach has been rejected, however, because (1)
it could not be done reliably, and (2) it would add a memory allocation overhead that
would not be acceptable in context where memory is constrained.

Related issue can be found on `Github <https://github.com/apache/nuttx/issues/3345>`_.
