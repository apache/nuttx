@echo off

rem tools/define.bat
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

rem Handle command line options
rem [-h] <compiler-path> <def1>[=val1] [<def2>[=val2] [<def3>[=val3] ...]]
rem [-w] [-d] ignored for compatibility with define.sh

set progname=%0

:ArgLoop
if "%1"=="-d" goto :NextArg
if "%1"=="-w" goto :NextArg
if "%1"=="-h" goto :ShowUsage

goto :CheckCompilerPath

:NextArg
shift
goto :ArgLoop

:CheckCompilerPath

if "%1"=="" (
  echo Missing compiler path
  goto :ShowUsage
)

set ccpath=%1
shift

set compiler=
for /F %%i in ("%ccpath%") do set compiler=%%~ni

if "%1"=="" (
  echo Missing definition list
  goto :ShowUsage
)

rem Check for some well known, non-GCC Windows native tools that require
rem a special output format as well as special paths

:GetFormat
set fmt=std
if "%compiler%"=="ez8cc" goto :SetZdsFormt
if "%compiler%"=="zneocc" goto :SetZdsFormt
if "%compiler%"=="ez80cc" goto :SetZdsFormt
goto :ProcessDefinitions

:SetZdsFormt
set fmt=zds

rem Now process each directory in the directory list

:ProcessDefinitions
set response=

:DefinitionLoop
if "%1"=="" goto :Done

set varname=%1
shift

rem Handle the output depending on if there is a value for the variable or not

if not "%1"=="" goto :GetValue

rem Handle the output using the selected format

:NoValue
if "%fmt%"=="zds" goto :NoValueZDS

:NoValueStandard
rem Treat the first definition differently

if "%response%"=="" (
  set response=-D%varname%
  goto :DefinitionLoop
)

set response=%response% -D%varname%
goto :DefinitionLoop

:NoValueZDS
rem Treat the first definition differently

if "%response%"=="" (
  set response=-define:%varname%
  goto :DefinitionLoop
)

set response=%response% -define:%varname%
goto :DefinitionLoop

rem Get value following the variable name

:GetValue
set varvalue=%1
shift

rem Handle the output using the selected format

if "%fmt%"=="zds" goto :ValueZDS

:ValueStandard
rem Treat the first definition differently

if "%response%"=="" (
  set response=-D%varname%=%varvalue%
  goto :DefinitionLoop
)

set response=%response% -D%varname%=%varvalue%
goto :DefinitionLoop

:ValueZds
rem Treat the first definition differently

if "%response%"=="" (
  set response=-define:%varname%=%varvalue%
  goto :DefinitionLoop
)

set response=%response% -define:%varname%=%varvalue%
goto :DefinitionLoop

:Done
echo %response%
goto :End

:ShowUsage
echo %progname% is a tool for flexible generation of command line pre-processor
echo definitions arguments for a variety of diffent ccpaths in a variety of
echo compilation environments"
echo USAGE:%progname% [-h] ^<compiler-path^> ^<def1^>[=^<val1^>] [^<def2^>[=^<val2^>] [^<def3^>[=^<val3^>] ...]]
echo Where:"
echo  ^<compiler-path^>
echo    The full path to your ccpath
echo  ^<def1^> ^<def2^> ^<def3^> ...
echo    A list of pre-preprocesser variable names to be defined.
echo  [=^<val1^>] [=^<val2^>] [=^<val3^>] ...
echo    optional values to be assigned to each pre-processor variable.
echo    If not supplied, the variable will be defined with no explicit value.
echo  -h
echo    Show this text and exit

:End
