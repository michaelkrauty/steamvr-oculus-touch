; cv1touch for Windows: the installer (Inno Setup 6).
;
; Installs for the person running it, with no administrator rights: the program under
; %LOCALAPPDATA%\Programs\cv1touch, a Start-menu folder, and a Run entry so that the daemon
; starts at logon without a window (cv1touchd-quiet). After the files are in place it gives the
; sensors Windows' own WinUSB driver (cv1touch sensors install, which asks for permission once),
; registers the SteamVR driver (cv1touch steamvr install) and starts the daemon; a step that does
; not finish is said in a message box, in finish-setup's own words, and the
; Start menu's "Finish setting up cv1touch" runs both again. The uninstaller stops the daemon,
; unregisters the driver and takes the sensors' driver off again;
; settings and calibration under %LOCALAPPDATA%\cv1touch stay, so that installing again finds them.
;
; Build: ISCC /DVersion=0.5.1 /DBundle=..\..\target\bundle cv1touch.iss, with a bundle from
; windows\bundle.ps1; the installer lands in target\installer (CI does both:
; .github\workflows\windows.yml).

#ifndef Version
  #error Pass the version: ISCC /DVersion=0.5.1 ...
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
UninstallDisplayIcon={app}\bin\cv1touch.exe
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

[Icons]
; Each opens a Command Prompt that stays open (cmd /k), so that what the command says can be read.
; The command line is quoted twice over: cmd strips the outer quotes and keeps the path's own.
Name: "{group}\Set up cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" setup"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Calibrate: where the sensors stand, and how the controllers are held"
Name: "{group}\Line up the controllers with your headset"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" setup --redo headset --quick"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Redo the last step of setup in about 15 seconds: for another headset, or when the controllers appear away from your hands"
Name: "{group}\Check cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" doctor"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Whether everything cv1touch needs is in place"
Name: "{group}\Finish setting up cv1touch"; Filename: "{cmd}"; Parameters: "/d /k """"{app}\bin\cv1touch.exe"" finish-setup"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "The sensors' driver and the SteamVR registration, for when the installer could not do them"
Name: "{group}\Send a cv1touch report"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" report"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "One file with the logs and settings, for someone helping you"
Name: "{group}\Start cv1touch"; Filename: "{cmd}"; Parameters: "/d /k ""start """" /wait ""{app}\bin\cv1touchd-quiet.exe"" && echo cv1touch is running. It also starts by itself when you log in."""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Starts the daemon, which normally starts at logon"
Name: "{group}\Stop cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" stop"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Stops the daemon until the next logon or Start cv1touch"

[Registry]
; The daemon starts at logon, without a window.
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "cv1touch"; ValueData: """{app}\bin\cv1touchd-quiet.exe"""; Flags: uninsdeletevalue

[Run]
Filename: "{cmd}"; Parameters: "/d /k """"{app}\bin\cv1touch.exe"" setup"""; WorkingDir: "{app}\bin"; Description: "Set up cv1touch now (wake both controllers and start SteamVR first)"; Flags: postinstall nowait skipifsilent; Check: ReadyForCalibration

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
  else if not StopDaemon then
    Result := 'cv1touch could not stop safely. Restart Windows, then run this installer again.';
end;

function InitializeUninstall: Boolean;
begin
  Result := SteamVRStopped;
  if not Result then
    SuppressibleMsgBox('Quit SteamVR before uninstalling, then try again.', mbError, MB_OK, IDOK);
end;

// This event is after the user's confirmation and before files are removed. An exception
// here aborts removal; InitializeUninstall must remain read-only so Cancel keeps tracking.
procedure InitializeUninstallProgressForm;
begin
  if not SteamVRStopped or not StopDaemon then
  begin
    SuppressibleMsgBox('cv1touch could not stop safely. Restart Windows, then uninstall again.', mbError, MB_OK, IDOK);
    Abort;
  end;
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
  if not SetupReady and not WizardSilent then
    MsgBox('cv1touch is installed, but not everything is set up yet:' + #13#10 + #13#10 + Reasons + #13#10 +
      'When that is done, choose "Finish setting up cv1touch" in the Start menu: it runs these steps again and says what it finds.',
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
