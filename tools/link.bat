@echo off

rem tools/link.bat
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

set usemklink=
if "%1"=="-m" (
  set usemklink="y"
shift
)

set src=%1
set link=%2

rem Verify that arguments were provided

if "%src%"=="" goto :MissingSrc
if "%link%"=="" goto :MissingLink
goto CheckSrc

:MissingSrc

echo Missing ^<src^> and ^<link^> arguments
goto :ShowUsage

:MissingLink

echo Missing ^<link^> arguments
goto :ShowUsage

rem Verify that a directory exists at the source path

:CheckSrc

if exist %src% goto :CheckLink

echo No directory at %src%
goto :ShowUsage

:CheckLink

rem If something already exists at the destination path, remove it

if not exist %link% goto :MkLink

rmdir /q /s %link%
if errorlevel 1 (
  echo Failed to remove existing object at %link%
  goto :ShowUsage
)

rem Copy the directory

:MkLink

if "%usemklink%"=="y" (
  /user:administrator mklink /d %src% %link%
goto :End
)

rem %src% may include forward slashes.  That upsets xcopy, but not GNUWin32 cp
rem xcopy %src% %link% /c /q /s /e /y /i
cp -dR %src% %link%
echo FAKELNK > %link%\.fakelnk
goto :End

:ShowUsage
echo USAGE: %0 ^<src^> ^<link^>
echo Where:
echo  ^<src^> is the source directory to be linked
echo  ^<link^> is the link to be created

:End
