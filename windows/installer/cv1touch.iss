; cv1touch for Windows: the installer (Inno Setup 6).
;
; Installs for the person running it, with no administrator rights: the program under
; %LOCALAPPDATA%\Programs\cv1touch, a Start-menu folder, and a Run entry so that the daemon
; starts at logon without a window (cv1touchd-quiet). After the files are in place it gives the
; sensors Windows' own WinUSB driver (cv1touch sensors install, which asks for permission once),
; registers the SteamVR driver (cv1touch steamvr install) and starts the daemon; a step that does
; not finish (no sensor plugged in, SteamVR running or missing) is said in a message box, and the
; Start menu's "Finish setting up cv1touch" runs both again. The uninstaller stops the daemon (by
; force if it will not end), unregisters the driver and takes the sensors' driver off again;
; settings and calibration under %LOCALAPPDATA%\cv1touch stay, so that installing again finds them.
;
; Build: ISCC /DVersion=0.2.0 /DBundle=..\..\target\bundle cv1touch.iss, with a bundle from
; windows\bundle.ps1; the installer lands in target\installer (CI does both:
; .github\workflows\windows.yml).

#ifndef Version
  #error Pass the version: ISCC /DVersion=0.2.0 ...
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
Name: "{group}\Finish setting up cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" sensors install & ""{app}\bin\cv1touch.exe"" steamvr install"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "The sensors' driver and the SteamVR registration, for when the installer could not do them"
Name: "{group}\Send a cv1touch report"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" report"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "One file with the logs and settings, for someone helping you"
Name: "{group}\Start cv1touch"; Filename: "{app}\bin\cv1touchd-quiet.exe"; Comment: "Starts the daemon, which normally starts at logon"
Name: "{group}\Stop cv1touch"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" stop"""; WorkingDir: "{app}\bin"; IconFilename: "{app}\bin\cv1touch.exe"; Comment: "Stops the daemon until the next logon or Start cv1touch"

[Registry]
; The daemon starts at logon, without a window.
Root: HKCU; Subkey: "Software\Microsoft\Windows\CurrentVersion\Run"; ValueType: string; ValueName: "cv1touch"; ValueData: """{app}\bin\cv1touchd-quiet.exe"""; Flags: uninsdeletevalue

[Run]
Filename: "{cmd}"; Parameters: "/k """"{app}\bin\cv1touch.exe"" setup"""; WorkingDir: "{app}\bin"; Description: "Set up cv1touch now (plug in the sensors and the headset first)"; Flags: postinstall nowait skipifsilent

[Code]
// A running daemon holds its files: cv1touch stop asks it to stop and waits until it, and its
// supervisor, have ended (exit code 0; 1 when none runs). One that still runs after that (it
// hangs, say: exit code 2) is ended by force, since the files must be replaced or removed either
// way; Windows needs a moment after taskkill to let go of them.
procedure StopDaemon;
var
  Tool: String;
  Code: Integer;
begin
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  if not FileExists(Tool) then
    exit;
  if not Exec(Tool, 'stop', '', SW_HIDE, ewWaitUntilTerminated, Code) then
    Code := -1;
  Log(Format('cv1touch stop: exit code %d', [Code]));
  if Code = 2 then
  begin
    Log('cv1touchd still runs: ending it by force');
    Exec(ExpandConstant('{sys}\taskkill.exe'), '/F /IM cv1touchd.exe', '', SW_HIDE, ewWaitUntilTerminated, Code);
    Sleep(500);
  end;
end;

// Before files are replaced.
function PrepareToInstall(var NeedsRestart: Boolean): String;
begin
  Result := '';
  StopDaemon;
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
  StopDaemon;
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  Exec(Tool, 'steamvr uninstall', '', SW_HIDE, ewWaitUntilTerminated, Code);
  Exec(Tool, 'sensors remove', '', SW_HIDE, ewWaitUntilTerminated, Code);
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
  if not Exec(Tool, Arguments, '', SW_HIDE, ewWaitUntilTerminated, Code) then
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
  Code: Integer;
begin
  if CurStep <> ssPostInstall then
    exit;
  Tool := ExpandConstant('{app}\bin\cv1touch.exe');
  Left := Step(Tool, 'sensors install', 'Giving the sensors their driver (Windows asks for permission once)...',
    'The sensors are not ready: plug in every sensor, and allow the permission prompt.');
  Left := Left + Step(Tool, 'steamvr install', 'Registering the SteamVR driver...',
    'cv1touch is not registered with SteamVR: install SteamVR, and quit it while cv1touch is set up.');
  WizardForm.StatusLabel.Caption := 'Starting cv1touch...';
  Exec(ExpandConstant('{app}\bin\cv1touchd-quiet.exe'), '', '', SW_HIDE, ewNoWait, Code);
  if (Left <> '') and not WizardSilent then
    MsgBox('cv1touch is installed, but not everything is set up yet:' + #13#10 + #13#10 + Left + #13#10 +
      'When that is done, choose "Finish setting up cv1touch" in the Start menu: it runs these steps again and says what it finds.',
      mbInformation, MB_OK);
end;
