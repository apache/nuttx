============
Rules Checks
============

.. note:: this should be revised

NuttX uses Kconfig and its main checking for rules and dependencies between
features to guarantee that the system will compile correctly.

Another resource used to enforce that none pre-required feature was left behind
is the "#ifdef" inside the source code. Although effective to detect configuration
issue, this technic should be avoid in favor of more well though rules inside
Kconfig.

However, it is not possible to enforce many rules inside the Kconfig, otherwise
the configuration becomes very inflexible and will not allow some use cases.

For those case where Kconfig rule cannot fix everything, we need to return to
#ifdef approach inside the code. But there is an issue: imagine we put a rule
inside the file_feature_A.c:

.. code-block:: c

  #if !defined(CONFIG_FEATURE_B)
  #error "You need FEATURE_B to normal use case"
  #endif

It will give a hint to users to select FEATURE_B, but imagine we have a smart
user that knows how to use FEATURE_A without FEATURE_B, or want to it with
FEATURE_C instead of FEATURE_B.

This special user for instance knows all the IP addresses from all domains under
the sun and decided that he can use the network without DNS support. How we can
support this user and yet keep to rules in place for "ordinaries" users?

For this we have an option to disable those #ifdef common checking, as explained
in the next section.

Ignoring Rules Checks
=====================

A way to ignore the forcing of those common usage #ifdef is using an option to
disable rules check: CONFIG_IGNORE_RULES_CHECK.

So, for the above example the new rules will include a test to confirm that the
CONFIG_IGNORE_RULES_CHECK is not enabled:

.. code-block:: c

  #if !defined(CONFIG_IGNORE_RULES_CHECK) && !defined(CONFIG_FEATURE_B)
  #error "You need FEATURE_B to normal use case or enable CONFIG_IGNORE_RULES_CHECK"
  #endif

Note: In order to enable CONFIG_IGNORE_RULES_CHECK you need first enable CONFIG_EXPERIMENTAL.
