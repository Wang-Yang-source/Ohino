; Ohino Studio — Windows installer (NSIS 3.x)
; Built with makensis on any host (macOS/Linux/Windows).
; Usage: makensis -DVERSION=0.2.0 -DSTAGE_DIR=<dir> -DOUT_FILE=<file.exe> ohino.nsi

!ifndef VERSION
  !define VERSION "0.2.0"
!endif
!ifndef STAGE_DIR
  !define STAGE_DIR "..\..\dist\Ohino-windows-x86_64"
!endif
!ifndef OUT_FILE
  !define OUT_FILE "..\..\dist\Ohino-v${VERSION}-windows-x86_64.exe"
!endif

!define APP_NAME "Ohino Studio"
!define APP_PUBLISHER "Ohino Project"
!define APP_URL "https://github.com/Wang-Yang-source/Ohino"
!define APP_EXE "opentl_cli.exe"

Name "${APP_NAME} ${VERSION}"
OutFile "${OUT_FILE}"
InstallDir "$PROGRAMFILES64\Ohino"
InstallDirRegKey HKLM "Software\Ohino" "InstallDir"
RequestExecutionLevel admin
SetCompressor /SOLID lzma
Unicode true
!include "WinMessages.nsh"
!include "StrFunc.nsh"
${Using:StrFunc} StrStr

VIProductVersion "${VERSION}.0"
VIAddVersionKey "ProductName" "${APP_NAME}"
VIAddVersionKey "CompanyName" "${APP_PUBLISHER}"
VIAddVersionKey "FileDescription" "${APP_NAME} installer"
VIAddVersionKey "FileVersion" "${VERSION}.0"
VIAddVersionKey "ProductVersion" "${VERSION}.0"
VIAddVersionKey "LegalCopyright" "MIT License"

Page directory
Page instfiles
UninstPage uninstConfirm
UninstPage instfiles

Section "Ohino Studio (required)" SEC_CORE
  SectionIn RO
  SetOutPath "$INSTDIR"
  File /r "${STAGE_DIR}\*.*"

  WriteRegStr HKLM "Software\Ohino" "InstallDir" "$INSTDIR"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "DisplayName" "${APP_NAME}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "DisplayVersion" "${VERSION}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "Publisher" "${APP_PUBLISHER}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "URLInfoAbout" "${APP_URL}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "UninstallString" "$INSTDIR\uninstall.exe"
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "NoModify" 1
  WriteRegDWORD HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino" "NoRepair" 1
  WriteUninstaller "$INSTDIR\uninstall.exe"

  CreateDirectory "$SMPROGRAMS\Ohino"
  CreateShortcut "$SMPROGRAMS\Ohino\Ohino Studio.lnk" "$INSTDIR\bin\${APP_EXE}" "--gui" "$INSTDIR\bin\${APP_EXE}"
  CreateShortcut "$SMPROGRAMS\Ohino\Ohino Studio (headless render).lnk" "$INSTDIR\bin\${APP_EXE}" "--render-gui" "$INSTDIR\bin\${APP_EXE}"
  CreateShortcut "$SMPROGRAMS\Ohino\Uninstall Ohino.lnk" "$INSTDIR\uninstall.exe"
SectionEnd

Section "Add Ohino to PATH" SEC_PATH
  ; Stock NSIS only: append to the per-user PATH unless it is already there.
  ReadRegStr $0 HKCU "Environment" "Path"
  StrCpy $1 "$INSTDIR\bin"
  ${StrStr} $2 "$0" "$1"
  StrCmp $2 "" 0 +2
    WriteRegExpandStr HKCU "Environment" "Path" "$0;$1"
  SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000
SectionEnd

Section "Uninstall"
  Delete "$SMPROGRAMS\Ohino\Ohino Studio.lnk"
  Delete "$SMPROGRAMS\Ohino\Ohino Studio (headless render).lnk"
  Delete "$SMPROGRAMS\Ohino\Uninstall Ohino.lnk"
  RMDir "$SMPROGRAMS\Ohino"

  ReadRegStr $0 HKCU "Environment" "Path"
  StrCpy $2 "$0"
  ; remove our entry (naive but sufficient: exact ";$INSTDIR\bin" suffix)
  StrLen $3 ";$INSTDIR\bin"
  StrCpy $4 $2 -$3
  StrCmp $4 "" 0 +2
    WriteRegExpandStr HKCU "Environment" "Path" "$4"

  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\Ohino"
  DeleteRegKey HKLM "Software\Ohino"
  RMDir /r "$INSTDIR"
SectionEnd
