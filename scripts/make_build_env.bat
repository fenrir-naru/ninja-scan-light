@ECHO OFF
REM -- Automates creation of ninja-scan-light build environment
REM -- @see https://github.com/rtwolf/cygwin-auto-install
REM -- @see https://gist.github.com/mikepruett3/8f3d7bc527776551fb1629f934f68bbe
 
SETLOCAL ENABLEEXTENSIONS

REM -- Configure our paths
REM -- The packages shown here are added to the default packages
SET SITE=https://ftp.iij.ad.jp/pub/cygwin/
if 1 equ 0 (
  REM -- Install batch file directory
  SET "ROOT_DIR=%~dp0"
  CALL SET ROOT_DIR=%%ROOT_DIR:~0,-1%%
) else (
  REM -- Install current directory
  SET "ROOT_DIR=%CD%"
)
SET "ROOT_DIR=%ROOT_DIR%/cygwin"
SET "LOCAL_PACKAGE_DIR=%ROOT_DIR%/download"
SET PACKAGES=mintty,wget,diffutils,git,make,patch,p7zip,ruby
SET SDCC_VER=3.3.0

REM -- to remove double quote from variable => ex) %ROOT_DIR:"=%

:: Check if we are interactive
:: https://steve-jansen.github.io/guides/windows-batch-scripting/part-10-advanced-tricks.html
SET INTERACTIVE=0
ECHO %CMDCMDLINE% | FINDSTR /L %COMSPEC% >NUL 2>&1
IF %ERRORLEVEL% == 0 SET INTERACTIVE=1

:: Borrowed Code from - https://stackoverflow.com/questions/7985755/how-to-detect-if-cmd-is-running-as-administrator-has-elevated-privileges
SET ADMIN_OPT=
NET SESSION > NUL 2>&1
IF %ERRORLEVEL% NEQ 0 SET ADMIN_OPT=--no-admin

IF NOT EXIST "%ROOT_DIR%" MKDIR "%ROOT_DIR%"

REM -- Download Cygwin installer
IF NOT EXIST "%ROOT_DIR%/cygwin-setup.exe" (
  ECHO cygwin-setup.exe NOT found! Downloading installer...
  bitsadmin /transfer cygwinDownloadJob /download /priority foreground https://cygwin.com/setup-x86_64.exe "%ROOT_DIR%/cygwin-setup.exe"
) ELSE (
  ECHO cygwin-setup.exe found! Skipping installer download...
)
 
REM -- More info on command line options at: https://cygwin.com/faq/faq.html#faq.setup.cli
START "Installing Cygwin" ^
/WAIT ^
"%ROOT_DIR%/cygwin-setup.exe" --quiet-mode %ADMIN_OPT% --no-desktop --download --local-install --no-verify -s %SITE% -l "%LOCAL_PACKAGE_DIR%" -R "%ROOT_DIR%" --packages %PACKAGES%
ECHO Cygwin installed.

REM -- Download SDCC installer
IF NOT EXIST "%ROOT_DIR%/usr/local/sdcc-%SDCC_VER%-setup.exe" (
  ECHO sdcc-%SDCC_VER%-setup.exe NOT found! Downloading installer...
  bitsadmin /transfer sdccDownloadJob /download /priority foreground https://downloads.sourceforge.net/project/sdcc/sdcc-win32/%SDCC_VER%/sdcc-%SDCC_VER%-setup.exe "%ROOT_DIR%/usr/local/sdcc-%SDCC_VER%-setup.exe"
) ELSE (
  ECHO sdcc-%SDCC_VER%-setup.exe found! Skipping installer download...
)

START "INSTALLING SDCC" ^
/WAIT ^
"%ROOT_DIR%/bin/bash.exe" --login -i -c "if ^! which sdcc ^> /dev/null 2^>^&1; then 7z x /usr/local/sdcc-%SDCC_VER%-setup.exe -o/usr/local/sdcc-%SDCC_VER% ^&^& chmod 755 /usr/local/sdcc-%SDCC_VER%/bin/* ^&^& echo 'PATH=$PATH:/usr/local/sdcc-%SDCC_VER%/bin' ^>^> ~/.bashrc; fi"
ECHO sdcc-%SDCC_VER% installed.

:: Pause exiting script if we are run interactivly
IF "%INTERACTIVE%"=="0" PAUSE
EXIT /B 0
