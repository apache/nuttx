@echo off

rem tools/copydir.bat
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

rem
rem NuttX uses symbolic links to configure platform-specific directories into
rem the build system.  This works great except for when a Windows native
rem toolchain is used in a Cygwin environment.  In that case, symbolic
rem links do not work correctly when accessed from the Windows native toolchain;
rem rather, just look link files with the extension .lnk
rem
rem In this environment, the build system will work around this using this script
rem as a replacement for the 'ln' command.  This scrpt will simply copy the
rem directory into the expected positiion.
rem

set src=%1
set dest=%2

rem Verify that arguments were provided

if "%src%"=="" goto :MissingSrc
if "%dest%"=="" goto :MissingDest
goto CheckSrc

:MissingSrc

echo Missing ^<src^> and ^<dest^> arguments
goto :ShowUsage

:MissingDest

echo Missing ^<dest^> arguments
goto :ShowUsage

rem Verify that a directory exists at the source path

:CheckSrc

if exist %src% goto :CheckDest

echo No directory at %src%
goto :ShowUsage

:CheckDest

rem If something already exists at the destination path, remove it

if not exist %dest% goto :CopyDir

rmdir /q /s %dest%
if errorlevel 1 (
  echo Failed to remove existing object at %dest%
  goto :ShowUsage
)

rem Copy the directory

:CopyDir

xcopy %src% %dest% /c /q /s /e /y /i
echo FAKELNK >  %dest%\.fakelnk
goto :End

:ShowUsage
echo USAGE: %0 ^<src^> ^<dest^>
echo Where:
echo  ^<src^> is the source directory to be copied
echo  ^<dest^> is the destination directory to be created

:End
