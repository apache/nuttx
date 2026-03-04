===================
``ide_exporter.py``
===================

This Python script will help to create NuttX project in the IAR and
uVision IDEs.  These are few simple the steps to export the IDE
workspaces.

1) Start the NuttX build from the Cygwin command line before trying to
   create your project by running::

       make V=1 |& tee build_log

   This is necessary to certain auto-generated files and directories that
   will be needed.   This will provide the build log to construct the IDE
   project also.

2) Export the IDE project base on that make log. The script usage:

   usage: ide_exporter.py [-h] [-v] [-o OUT_DIR] [-d] build_log {iar,uvision_armcc,uvision_gcc} template_dir

   positional arguments::

       build_log             Log file from make V=1
       {iar,uvision_armcc,uvision_gcc}
                             The target IDE: iar, uvision_gcc, (uvision_armcc is experimental)
       template_dir          Directory that contains IDEs template projects

   optional arguments::

       -h, --help            show this help message and exit
       -v, --version         show program's version number and exit
       -o OUT_DIR, --output OUT_DIR
                             Output directory
       -d, --dump            Dump project structure tree

   Example::

        cd nuttx
        make V=1 |& tee build_log

        ./tools/ide_exporter.py makelog_f2nsh_c  iar ./boards/<arch>/<chip>/<board>/ide/template/iar -o ./boards/<arch>/<chip>/<board>/ide/nsh/iar

   or::

        ./tools/ide_exporter.py makelog_f2nsh_c uvision_gcc ./boards/<arch>/<chip>/<board>/ide/template/uvision_gcc/ -o ./boards/<arch>/<chip>/<board>/ide/nsh/uvision

3) Limitations:

     - IAR supports C only. Iar C++ does not compatible with g++ so disable
       C++ if you want to use IAR.
     - uvision_armcc : nuttx asm (inline and .asm) can't be compiled with
       armcc so do not use this option.
     - uvision_gcc : uvision project that uses gcc. Need to specify path to
       gnu toolchain.
       In uVison menu, select::

         Project/Manage/Project Items.../FolderExtension/Use GCC compiler/ PreFix, Folder

4) Template projects' constrains:

     - mcu, core, link script shall be configured in template project
     - Templates' name are fixed:

        - template_nuttx.eww  : IAR nuttx workspace template
        - template_nuttx_lib.ewp : IAR nuttx library project template
        - template_nuttx_main.ewp : IAR nuttx main project template
        - template_nuttx.uvmpw : uVision workspace
        - template_nuttx_lib.uvproj : uVision library project
        - template_nuttx_main.uvproj : uVision main project
     - iar:

        - Library option shall be set to 'None' so that IAR could use nuttx
           libc
        - __ASSEMBLY__ symbol shall be defined in assembler

     - uVision_gcc:

        - There should be one fake .S file in projects that has been defined
          __ASSEMBLY__ in assembler.
        - In Option/CC tab : disable warning
        - In Option/CC tab : select Compile thump code (or Misc control =
          -mthumb)
        - template_nuttx_lib.uvproj shall add 'Post build action' to copy .a
          file to .\lib
        - template_nuttx_main.uvproj Linker:

          - Select 'Do not use Standard System Startup Files' and 'Do not
            use Standard System Libraries'
          - Do not select 'Use Math libraries'
          - Misc control = --entry=__start

5) How to create template for other configurations:

        1) uVision with gcc toolchain:

            - Copy 3 uVision project files
            - Select the MCU for main and lib project
            - Correct the path to ld script if needed

        2) iar:

            - Check if the arch supports IAR (only armv7-m is support IAR
              now)
            - Select the MCU for main and lib project
            - Add new ld script file for IAR

.. note::

   Due to bit rot, the template files for the stm3220g-eval and for the
   stm32f429-disco have been removed from the NuttX repository. For reference,
   they can be found in the Obsoleted repository at
   Obsoleted/stm32f429i_disco/ltcd/template and at
   Obsoleted/stm3220g-eval/template.
