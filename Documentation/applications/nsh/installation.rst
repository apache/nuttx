==============================
Customizing NSH Initialization
==============================

**Ways to Customize NSH Initialization**. There are three ways to
customize the NSH start-up behavior. Here they are presented in order of
increasing difficulty:

  #. You can extend the initialization logic in
     ``boards/arm/stm32/stm3240g-eval/src/stm32_appinit.c``. The logic
     there is called each time that NSH is started and is good place in
     particular for any device-related initialization.

  #. You replace the sample code at ``apps/examples/nsh/nsh_main.c`` with
     whatever start-up logic that you want. NSH is a library at
     ``apps/nshlib``. ``apps.examples/nsh`` is just a tiny, example
     start-up function (``CONFIG_INIT_ENTRYPOINT``\ ()) that runs
     immediately and illustrates how to start NSH If you want something
     else to run immediately then you can write your write your own custom
     ``CONFIG_INIT_ENTRYPOINT``\ () function and then start other tasks
     from your custom ``CONFIG_INIT_ENTRYPOINT``\ ().

  #. NSH also supports a start-up script that executed when NSH first
     runs. This mechanism has the advantage that the start-up script can
     contain any NSH commands and so can do a lot of work with very little
     coding. The disadvantage is that is is considerably more complex to
     create the start-up script. It is sufficiently complex that is
     deserves its own paragraph

