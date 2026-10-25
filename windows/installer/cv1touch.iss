; cv1touch for Windows: the installer (Inno Setup 6).
;
; Installs for the person running it, with no administrator rights: the program under
; %LOCALAPPDATA%\Programs\cv1touch, a Start-menu folder, and a Run entry so that the daemon
; starts at logon without a window (cv1touchd-quiet). After the files are in place it gives the
; sensors Windows' own WinUSB driver (cv1touch sensors install, which asks for permission once),
; registers the SteamVR driver (cv1touch steamvr install) and starts the daemon; a step that does
; not finish (no sensor plugged in, SteamVR running or missing) is said in a message box, and the
; Start menu's "Finish setting up cv1touch" runs both again. The uninstaller stops the daemon,
; unregisters the driver and takes the sensors' driver off again;
; settings and calibration under %LOCALAPPDATA%\cv1touch stay, so that installing again finds them.
;
; Build: ISCC /DVersion=0.3.0 /DBundle=..\..\target\bundle cv1touch.iss, with a bundle from
; windows\bundle.ps1; the installer lands in target\installer (CI does both:
; .github\workflows\windows.yml).

#ifndef Version
  #error Pass the version: ISCC /DVersion=0.3.0 ...
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
Name: "{group}\Check cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" doctor"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Whether everything cv1touch needs is in place"
Name: "{group}\Finish setting up cv1touch"; Filename: "{cmd}"; Parameters: "/d /k """"{app}\bin\cv1touch.exe"" finish-setup"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "The sensors' driver and the SteamVR registration, for when the installer could not do them"
Name: "{group}\Send a cv1touch report"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" report"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "One file with the logs and settings, for someone helping you"
Name: "{group}\Start cv1touch"; Filename: "{app}\bin\cv1touchd-quiet.exe"; Comment: "Starts the daemon, which normally starts at logon"
Name: "{group}\Stop cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" stop"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Stops the daemon until the next logon or Start cv1touch"

[Registry]
; The daemon starts at logon, without a window.
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "cv1touch"; ValueData: """{app}\bin\cv1touchd-quiet.exe"""; Flags: uninsdeletevalue

[Run]
Filename: "{cmd}"; Parameters: "/d /k """"{app}\bin\cv1touch.exe"" setup"""; WorkingDir: "{app}\bin"; Description: "Set up cv1touch now (plug in the sensors and the headset first)"; Flags: postinstall nowait skipifsilent; Check: ReadyForCalibration

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
  if Result then
    Result := StopDaemon;
  if not Result then
    SuppressibleMsgBox('Quit SteamVR and stop cv1touch before uninstalling. If they will not stop, restart Windows and try again.', mbError, MB_OK, IDOK);
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
    SuppressibleMsgBox('SteamVR registration could not be removed. Your files and calibration are safe. Send a cv1touch report before removing its saved settings if you need help.', mbInformation, MB_OK, IDOK);
  ExecAndLogOutput(Tool, 'sensors remove', '', SW_SHOWNORMAL, ewWaitUntilTerminated, Code, nil);
  if Code <> 0 then
    SuppressibleMsgBox('Windows did not finish removing the sensor driver. Restart Windows if requested. To retry, reinstall cv1touch, plug in all sensors, then uninstall again and allow the permission prompt.', mbInformation, MB_OK, IDOK);
end;

// Runs the tool with Arguments, Doing shown on the progress page meanwhile, and says, as a line
// for the message box, what to do when it did not finish; '' when it did. The [Run] section would
// not look at exit codes.
function Step(Tool, Arguments, Doing, Advice: String): String;
var
  Code: Integer;
begin
  Result := '';
  WizardForm.StatusLabel.Caption := Doing;
  if not ExecAndLogOutput(Tool, Arguments, '', SW_SHOWNORMAL, ewWaitUntilTerminated, Code, nil) then
    Code := -1;
  Log(Format('cv1touch %s: exit code %d', [Arguments, Code]));
  if Code = 3 then
    Result := '- Restart Windows to finish giving the sensors their driver.' + #13#10
  else if Code <> 0 then
    Result := '- ' + Advice + #13#10;
end;

// After the files are in place: the sensors' driver, the SteamVR registration, and the daemon.
procedure CurStepChanged(CurStep: TSetupStep);
var
  Tool, Left: String;
begin
  if CurStep <> ssPostInstall then
    exit;
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  Left := Step(Tool, 'finish-setup', 'Preparing the sensors and SteamVR (Windows may ask for permission)...',
    'Driver setup did not finish. Plug in the sensors, allow the permission prompt, and ensure SteamVR is installed and has been started once, then closed.');
  SetupReady := Left = '';
  if (Left <> '') and not WizardSilent then
    MsgBox('cv1touch is installed, but not everything is set up yet:' + #13#10 + #13#10 + Left + #13#10 +
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
