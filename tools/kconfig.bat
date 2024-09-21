@echo off

rem tools/kconfig.bat
rem
rem SPDX-License-Identifier: Apache-2.0
rem
rem Licensed to the Apache Software Foundation (ASF) under one or more
rem contributor license agreements.  See the NOTICE file distributed with
rem this work for additional information regarding copyright ownership.  The
rem ASF licenses this file to you under the Apache License, Version 2.0 (the
rem "License"); you may not use this file except in compliance with the
rem License.  You may obtain a copy of the License at
rem
rem   http://www.apache.org/licenses/LICENSE-2.0
rem
rem Unless required by applicable law or agreed to in writing, software
rem distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
rem WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
rem License for the specific language governing permissions and limitations
rem under the License.
rem

rem Remember the state of the PATH variable on entry

set oldpath=%PATH%

rem Handle command line options

set action=%1
shift
if "%action%"=="" goto :MissingArgument

set appsdir=..\apps
set cygwindir=C:\Cygwin

:ArgLoop

if "%1"=="" goto :CheckArguments

if "%1"=="-a" (
  shift
  set appsdir=%1
  goto :NextArg
)

if "%1"=="-c" (
  shift
  set cygwindir=%1
  goto :NextArg
)

echo ERROR: Unrecognized option: %1
goto :ShowUsage

:NextArg
shift
goto :ArgLoop

rem Verify that all of the paths are valid

:CheckArguments
if exist "%appsdir%" goto :CheckCygwinDir

echo ERROR: %appsdir% does not exist
goto :ShowUsage

:CheckCygwinDir

if exist "%cygwindir%" goto :SetPath

echo ERROR: %cygwindir% does not exist
goto :ShowUsage

rem Setup some required environment variables and PATH settings

:SetPath
set PATH=%cygwindir%\usr\local\bin;%cygwindir%\usr\bin;%cygwindir%\bin;%PATH%
set APPSDIR=%appsdir%

rem Execute the requested action

if "%action%"=="config" goto :DoConfig
if "%action%"=="oldconfig" goto :DoOldConfig
if "%action%"=="menuconfig" goto :DoMenuConfig

echo ERROR: Unrecognized action: %action%
goto :ShowUsage

:DoConfig
kconfig-conf Kconfig
goto End

:DoOldConfig
kconfig-conf --oldconfig Kconfig
goto End

:DoMenuConfig
kconfig-mconf Kconfig
goto End

:MissingArgument

echo ERROR: Missing required argument

:ShowUsage
echo USAGE: %0 ^<action^> [-a ^<appsdir^>] [-c ^<cygwindir^>]
echo Where:
echo  ^<action^> is one of config, oldconf, or menuconfig
echo  ^<appsdir^> is the relative path to the apps\ directory.
echo    This defaults to ..\apps
echo  ^<cygwindir^> is the relative path to the Cygwin installation
echo    directory.  This defaults to C:\Cygwin

rem Restore the original PATH settings

:End
set PATH=%oldpath%
