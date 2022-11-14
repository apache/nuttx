@echo off

rem tools/configure.bat
rem
rem Licensed to the Apache Software Foundation (ASF) under one or more
rem  contributor license agreements.  See the NOTICE file distributed with
rem  this work for additional information regarding copyright ownership.  The
rem  ASF licenses this file to you under the Apache License, Version 2.0 (the
rem  "License"); you may not use this file except in compliance with the
rem  License.  You may obtain a copy of the License at
rem 
rem    http://www.apache.org/licenses/LICENSE-2.0
rem 
rem  Unless required by applicable law or agreed to in writing, software
rem  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
rem  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
rem License for the specific language governing permissions and limitations
rem under the License.
rem

if exist tools goto :GoToolDir
if exist ..\tools goto :SetToolDir

echo Cannot find tools\ directory
goto End

:GoToolDir
cd tools

:SetTooldir
set tooldir=%CD%

rem Parse command line arguments

set debug=
set enforce_distclean=
set distclean=
set fmt=-b
set posix=
set help=
set appdir=
set config=
set hostopt=

:ArgLoop
if "%1"=="" goto :NoConfig
if "%1"=="-h" goto :ShowUsage
if "%1"=="-d" goto :SetDebug
if "%1"=="-E" goto :SetEnforceDistclean
if "%1"=="-e" goto :SetDistclean
if "%1"=="-f" goto :SetFormat
if "%1"=="-b" goto :SetFormat
if "%1"=="-l" goto :SetHostOption
if "%1"=="-m" goto :SetHostOption
if "%1"=="-c" goto :SetHostOption
if "%1"=="-n" goto :SetHostOption
if "%1"=="-B" goto :SetHostOption
if "%1"=="-L" goto :SetList
if "%1"=="-a" goto :SetAppDir

set config=%1
goto EndOfLoop

:SetDebug
set debug=%1
goto :NextArg

:SetEnforceDistclean
set enforce_distclean=%1
goto :NextArg

:SetDistclean
set distclean=%1
goto :NextArg

:SetFormat
set fmt=%1
goto :NextArg

:SetHostOption
set hostopt=%1
goto :NextArg

:SetList
set list=%1
goto :EndOfLoop

:SetAppDir
shift
set appdir=-a %1

:NextArg
shift
goto :ArgLoop

:EndOfLoop

rem Check if we have to build configure.exe

if exist configure.exe goto :HaveConfigureExe

set cc=mingw32-gcc.exe
set cflags=-Wall -Wstrict-prototypes -Wshadow -g -pipe -I. -DCONFIG_WINDOWS_NATIVE=y
echo %cc% %cflags% -o configure.exe configure.c cfgparser.c
%cc% %cflags% -o configure.exe configure.c cfgparser.c
if errorlevel 1 (
  echo ERROR: %cc% failed
  echo Is mingw32-gcc.exe installed?  Is it in the PATH variable?
  goto End
)

:HaveConfigureExe
cd ..
tools\configure.exe %debug% %enforce_distclean% %distclean% %fmt% %hostopt% %appdir% %config% %list%
if errorlevel 1 echo configure.exe failed
goto End

:NoConfig
echo Missing ^<board-name^>:^<config-name^> argument

:ShowUsage
echo USAGE: %0 [-d] [-E] [-e] [-b|f] [-a ^<app-dir^>] ^<board-name^>:^<config-name^>
echo        %0 [-h]
echo\nWhere:
echo  -d:
echo    Enables debug output
echo  -E:
echo    Enforces distclean if already configured.
echo  -e:
echo    Performs distclean if configuration changed.
echo  -b:
echo    Informs the tool that it should use Windows style paths like C:\\Program Files
echo    instead of POSIX style paths are used like /usr/local/bin.  Windows
echo    style paths are used by default.
echo  -f:
echo    Informs the tool that it should use POSIX style paths like /usr/local/bin.
echo    By default, Windows style paths like C:\\Program Files are used.
echo  -l selects the Linux (l) host environment.  The [-c^|n] options
echo    select one of the Windows environments.  Default:  Use host setup
echo    in the defconfig file
echo  [-c^|n] selects the Windows host and a Windows environment:
echo    Cygwin (c), or Windows native (n). Default Cygwin
echo  -L:
echo    List all available configurations.
echo  -a ^<app-dir^>:
echo    Informs the configuration tool where the application build
echo    directory.  This is a relative path from the top-level NuttX
echo    build directory.  But default, this tool will look in the usual
echo    places to try to locate the application directory:  ../apps or
echo    ../apps-xx.yy where xx.yy is the NuttX version number.
echo  ^<board-name^>:
echo    Identifies the board.  This must correspond to a board directory
echo    under nuttx/boards/.
echo  ^<config-name^>:
echo    Identifies the specific configuration for the selected ^<board-name^>.
echo    This must correspond to a sub-directory under the board directory at
echo    under nuttx/boards/^<board-name^>/configs/.
echo  -h:
echo    Prints this message and exits.

:End
