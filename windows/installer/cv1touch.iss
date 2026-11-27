; cv1touch for Windows: the installer (Inno Setup 6).
;
; Installs for the person running it, with no administrator rights: the program under
; %LOCALAPPDATA%\Programs\cv1touch, a Start-menu shortcut for the tray app, and Run entries
; for the daemon (cv1touchd-quiet) and tray. After the files are in place it gives the
; sensors Windows' own WinUSB driver (cv1touch sensors install, which asks for permission once),
; registers the SteamVR driver (cv1touch steamvr install) and starts the daemon. Interactive
; installs also start the tray, with any unfinished steps explained in a message box. The tray
; app's Settings can retry them. The uninstaller stops the tray and daemon,
; unregisters the driver and takes the sensors' driver off again;
; settings and calibration under %LOCALAPPDATA%\cv1touch stay, so that installing again finds them.
;
; Build: ISCC /DVersion=0.6.0 /DBundle=..\..\target\bundle cv1touch.iss, with a bundle from
; windows\bundle.ps1; the installer lands in target\installer (CI does both:
; .github\workflows\windows.yml).

#ifndef Version
  #error Pass the version: ISCC /DVersion=0.6.0 ...
#endif
#ifndef Bundle
  #define Bundle "..\..\target\bundle"
#endif

[Setup]
AppId={{75E2A0C5-05F8-4B5B-85E4-3F134B2B1F51}
AppName=cv1touch
AppVersion={#Version}
AppVerName=cv1touch {#Version}
AppPublisher=michaelkrauty
AppPublisherURL=https://github.com/michaelkrauty/steamvr-oculus-touch
AppSupportURL=https://github.com/michaelkrauty/steamvr-oculus-touch/issues
AppUpdatesURL=https://github.com/michaelkrauty/steamvr-oculus-touch/releases
DefaultDirName={autopf}\cv1touch
DefaultGroupName=cv1touch
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0.22000
SetupLogging=yes
UninstallLogging=yes
LicenseFile={#Bundle}\LICENSE
OutputDir={#SourcePath}\..\..\target\installer
OutputBaseFilename=cv1touch-setup-{#Version}
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\bin\cv1touch-gui.exe
UninstallDisplayName=cv1touch

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Messages]
; The last page of the uninstaller ('%' is doubled: it introduces an argument in a message).
UninstalledAll=cv1touch was removed. Your settings and calibration were kept in %%LOCALAPPDATA%%\cv1touch, in case you install it again; delete that folder to remove them too.

[Files]
Source: "{#Bundle}\bin\*"; DestDir: "{app}\bin"; Flags: ignoreversion
Source: "{#Bundle}\lib\cv1touch\*"; DestDir: "{app}\lib\cv1touch"; Flags: ignoreversion
Source: "{#Bundle}\LICENSE"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#Bundle}\NOTICE"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#Bundle}\THIRD_PARTY_NOTICES.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#Bundle}\third_party\*"; DestDir: "{app}\third_party"; Flags: ignoreversion recursesubdirs

[InstallDelete]
Type: files; Name: "{group}\Set up cv1touch.lnk"
Type: files; Name: "{group}\Line up the controllers with your headset.lnk"
Type: files; Name: "{group}\Check cv1touch.lnk"
Type: files; Name: "{group}\Finish setting up cv1touch.lnk"
Type: files; Name: "{group}\Send a cv1touch report.lnk"
Type: files; Name: "{group}\Start cv1touch.lnk"
Type: files; Name: "{group}\Stop cv1touch.lnk"

[Icons]
Name: "{group}\cv1touch"; Filename: "{app}\bin\cv1touch-gui.exe"; Parameters: "--open"; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch-gui.exe"; Comment: "Open cv1touch"

[Registry]
; Both start at logon, without opening windows.
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "cv1touch"; ValueData: """{app}\bin\cv1touchd-quiet.exe"""; Flags: uninsdeletevalue
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "cv1touch-gui"; ValueData: """{app}\bin\cv1touch-gui.exe"""; Flags: uninsdeletevalue

[Run]
Filename: "{app}\bin\cv1touch-gui.exe"; Parameters: "--setup"; WorkingDir: "{app}\bin"; Description: "Set up cv1touch now (wake both controllers and start SteamVR first)"; Flags: postinstall nowait skipifsilent; Check: ReadyForCalibration
Filename: "{app}\bin\cv1touch-gui.exe"; Parameters: "--window"; WorkingDir: "{app}\bin"; Description: "Open cv1touch to finish setup"; Flags: postinstall nowait skipifsilent; Check: not ReadyForCalibration

[Code]
var
  SetupReady: Boolean;

function ReadyForCalibration: Boolean;
begin
  Result := SetupReady;
end;

function NoProcessNamed(Name: String): Boolean;
var
  Locator: Variant;
  Services: Variant;
  Processes: Variant;
begin
  // WMI is inbox on Windows. Failure is not proof that a process is stopped.
  Result := False;
  try
    Locator := CreateOleObject('WbemScripting.SWbemLocator');
    Services := Locator.ConnectServer('', 'root\CIMV2');
    Processes := Services.ExecQuery('SELECT ProcessId FROM Win32_Process WHERE Name = ''' + Name + '''');
    Result := Processes.Count = 0;
  except
    Log('Could not check running processes: ' + GetExceptionMessage);
  end;
end;

function StopGui: Boolean;
var
  Gui: String;
  Code, Attempt: Integer;
begin
  Result := True;
  Gui := ExpandConstant('{app}\bin\cv1touch-gui.exe');
  if not FileExists(Gui) then
    exit;
  if not Exec(Gui, '--quit', '', SW_HIDE, ewWaitUntilTerminated, Code) then
    Code := -1;
  Log(Format('cv1touch-gui --quit: exit code %d', [Code]));
  // The tray and windows release their rendezvous handles just before exiting.
  for Attempt := 1 to 10 do
  begin
    Result := NoProcessNamed('cv1touch-gui.exe');
    if Result then
      exit;
    Sleep(100);
  end;
end;

// Do not kill every process named cv1touchd: another Windows user's tracking session is not
// ours. Refuse an upgrade/removal that cannot stop cleanly rather than removing live files.
function StopDaemon: Boolean;
var
  Tool: String;
  Code: Integer;
begin
  Result := True;
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  if not FileExists(Tool) then
    exit;
  if not ExecAndLogOutput(Tool, 'stop', '', SW_SHOWNORMAL, ewWaitUntilTerminated, Code, nil) then
    Code := -1;
  Log(Format('cv1touch stop: exit code %d', [Code]));
  // A broken old installation may not even load (for example, v0.3.0 without the VC runtime).
  // Permit repairing it only if the independent process check proves nothing is running.
  Result := NoProcessNamed('cv1touchd.exe');
end;

function SteamVRStopped: Boolean;
begin
  Result := NoProcessNamed('vrserver.exe');
end;

// Before files are replaced.
function PrepareToInstall(var NeedsRestart: Boolean): String;
begin
  Result := '';
  if not SteamVRStopped then
    Result := 'Quit SteamVR, then click Next to try again. If it is already closed, restart Windows.'
  else if not StopGui or not StopDaemon then
    Result := 'cv1touch could not stop safely. Restart Windows, then run this installer again.';
end;

function InitializeUninstall: Boolean;
begin
  Result := SteamVRStopped;
  if not Result then
    SuppressibleMsgBox('Quit SteamVR before uninstalling, then try again.', mbError, MB_OK, IDOK);
end;

// This event is after the user's confirmation and before files are removed. An exception
// here aborts removal; InitializeUninstall must remain read-only so Cancel leaves both running.
procedure InitializeUninstallProgressForm;
begin
  if SteamVRStopped and StopGui and StopDaemon then
    exit;
  SuppressibleMsgBox('cv1touch could not stop safely. Restart Windows, then uninstall again.', mbError, MB_OK, IDOK);
  Abort;
end;

// Before files are removed: the daemon, the SteamVR registration, the sensors' driver (which asks
// for permission once). Their exit codes do not matter here: what is not there cannot be removed.
procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
var
  Tool: String;
  Code: Integer;
begin
  if CurUninstallStep <> usUninstall then
    exit;
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  ExecAndLogOutput(Tool, 'steamvr uninstall', '', SW_SHOWNORMAL, ewWaitUntilTerminated, Code, nil);
  if Code <> 0 then
    SuppressibleMsgBox('cv1touch could not remove itself from SteamVR. Your calibration is kept. To retry, install cv1touch again, quit SteamVR, then uninstall again. What went wrong is in ' + ExpandConstant('{localappdata}') + '\cv1touch\state\logs\uninstaller.log.', mbInformation, MB_OK, IDOK);
  ExecAndLogOutput(Tool, 'sensors remove', '', SW_SHOWNORMAL, ewWaitUntilTerminated, Code, nil);
  if Code <> 0 then
    SuppressibleMsgBox('Windows did not finish removing the sensor driver. Restart Windows if requested. To retry, reinstall cv1touch, plug in all sensors, then uninstall again and allow the permission prompt.', mbInformation, MB_OK, IDOK);
end;

var
  Reasons: String;
  InReasons: Boolean;

// finish-setup ends, when a step did not finish, with the line 'Setup is incomplete:' and one
// '- ' line per such step saying what to do (src/bin/cv1touch/finish_setup.rs): keep those. With
// a callback, ExecAndLogOutput no longer logs the output itself.
procedure KeepReason(const S: String; const Error, FirstLine: Boolean);
begin
  Log(S);
  if S = 'Setup is incomplete:' then
    InReasons := True
  else if InReasons and (Copy(S, 1, 2) = '- ') then
    Reasons := Reasons + S + #13#10;
end;

// After the files are in place: the sensors' driver, the SteamVR registration, and the daemon.
// The [Run] section would not look at exit codes.
procedure CurStepChanged(CurStep: TSetupStep);
var
  Code: Integer;
begin
  if CurStep <> ssPostInstall then
    exit;
  WizardForm.StatusLabel.Caption := 'Preparing the sensors and SteamVR (Windows may ask for permission)...';
  if not ExecAndLogOutput(ExpandConstant('{app}\bin\cv1touch.exe'), 'finish-setup', '', SW_SHOWNORMAL,
    ewWaitUntilTerminated, Code, @KeepReason) then
    Code := -1;
  Log(Format('cv1touch finish-setup: exit code %d', [Code]));
  SetupReady := Code = 0;
  if not SetupReady and (Reasons = '') then
    Reasons := '- cv1touch could not run its setup. Restart Windows, then try again.' + #13#10;
  // Start the tray after finish-setup and before the optional window on the final page.
  if not WizardSilent then
    if not Exec(ExpandConstant('{app}\bin\cv1touch-gui.exe'), '', ExpandConstant('{app}\bin'),
      SW_SHOWNORMAL, ewNoWait, Code) then
      Log(Format('Could not start the cv1touch tray (error %d)', [Code]));
  if not SetupReady and not WizardSilent then
    MsgBox('cv1touch is installed, but not everything is set up yet:' + #13#10 + #13#10 + Reasons + #13#10 +
      'When that is done, open "cv1touch" from the Start menu and choose "Finish driver & SteamVR setup" in Settings.',
      mbInformation, MB_OK);
end;

procedure PreserveLog(Name: String);
var
  Directory, Source: String;
begin
  Source := ExpandConstant('{log}');
  Directory := ExpandConstant('{localappdata}\cv1touch\state\logs');
  if (Source <> '') and ForceDirectories(Directory) then
    FileCopy(Source, Directory + '\' + Name, False);
end;

procedure DeinitializeSetup;
begin
  PreserveLog('installer.log');
end;

procedure DeinitializeUninstall;
begin
  PreserveLog('uninstaller.log');
end;
