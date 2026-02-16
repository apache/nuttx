=================================
``unity`` Unity testing framework
=================================

Unity is a unit testing framework for C developed by ThrowTheSwitch.org:

http://www.throwtheswitch.org/unity

.. note::

   There is more detailed documentation than what can be found on the website on
   the GitHub page. It can be found in `this collection of markdown
   documents <https://github.com/ThrowTheSwitch/Unity/tree/master/docs>`_

It is very minimalist and is designed to also run on embedded systems. Unity is
highly configurable through the use of different macros. These are described in
the `configuration guide
<https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityConfigurationGuide.md>`_.
Some of these configuration options are exposed in NuttX via the Kconfig
selection menu for the Unity test framework, but not all of them are
implemented.

.. note::

   If some configuration options you'd like to use for Unity are missing from
   NuttX, please open a pull request following the :doc:`contribution guidelines
   </contributing/index>`.


Usage
=====

In order to test your application using the Unity framework in NuttX, you can
create a test application. Documentation for creating a custom NuttX application
can be found :doc:`here </guides/customapps>`.

Just mark your application to depend on any Unity options you use:

.. code-block:: kconfig

   config MY_TESTS
       tristate "Test cases"
       depends on TESTING_UNITY
       depends on TESTING_UNITY_EXCLUDE_SETJMP
       depends on TESTING_UNITY_PRINT_FORMATTED
       depends on MY_APPLICATION_CODE

In your application's main function, you can include the testing library and
write tests!

.. code-block:: c

   #include <testing/unity.h>

   void setUp(void) { /* Test setup code for Unity */ }
   void tearDown(void) { /* Test tear-down code for Unity */ }

   /* Short test case */
   
   static void test_mything__passes(void)
   {
     int mynum = 0;
     mynum++;
     TEST_ASSERT_EQUAL(1, mynum);
   }

   int main(void)
   {
      UNITY_BEGIN();
      RUN_TEST(test_mything__passes);
      return UNITY_END();
   }

Now when you build your code, your test application will be visible in NSH to
run. If you run it, you will see a test report!

.. code-block:: console

   nsh> mytests
   mytests_main.c:18:test_mything__passes:PASS

   -----------------------
   1 Tests 0 Failures 0 Ignored
