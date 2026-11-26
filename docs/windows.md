# cv1touch on Windows

One installer sets up everything. No command line, Python or developer tools are needed.

## What you need

- Windows 11 x64 with [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) and a headset SteamVR already tracks. The Rift CV1's display is not used: the CV1 headset is only the controllers' USB radio receiver, and its HDMI cable can stay unplugged.
- The CV1 headset's original Touch controllers, already paired to it, with charged batteries.
- Two or more CV1 sensors with overlapping views, on USB 3 ports where possible and not all on one hub.
- Permission to approve one Windows administrator prompt, for the sensor driver.
- No Oculus or Meta PC software running: it competes for the same USB devices. cv1touch gives the sensors Windows' WinUSB driver, replacing any other driver they had.

## Install

1. Start SteamVR once with your headset, then quit it. Plug in the CV1 headset's USB cable and every sensor.
2. Download `cv1touch-setup-<version>.exe` from the [releases](https://github.com/michaelkrauty/steamvr-oculus-touch/releases) and run it. It installs for your account only, into `%LOCALAPPDATA%\Programs\cv1touch`.
3. The installer is unsigned. If SmartScreen warns, choose **More info**, then **Run anyway**. Do not turn off antivirus, SmartScreen, driver signature enforcement or Secure Boot.
4. Windows asks once for permission to change the system. This is `cv1touch sensors install`, giving each sensor WinUSB, the USB driver Microsoft ships with Windows. No other driver or certificate is installed.
5. The installer registers the driver with SteamVR and starts the daemon. An interactive install also starts the tray icon. Both start when you sign in.

If a step could not be done, the installer says what to do, such as plugging in a sensor or approving the prompt. It offers **Open cv1touch to finish setup**. You can also open **cv1touch** from the Start menu and choose **Finish driver & SteamVR setup** in **Settings**. Restart Windows if it asks.

## Set up

Wake both controllers and start SteamVR with your headset tracking. Choose **Set up cv1touch now** at the end of installation, or open **cv1touch** from the Start menu and choose **Set up…**. The setup window runs the same guide described in [guided setup](setup.md#guided-setup). Allow 10 to 20 minutes and room to move. Keep the sensors fixed afterwards.

Until setup is complete, the status window shows that setup is still needed. That is expected.

SteamVR shows the controllers as **Oculus Touch (cv1touch)** and uses each game's Oculus Touch bindings. To use the right Oculus button, [bind it](setup.md#bind-the-right-oculus-button).

## Status window and tray

Open **cv1touch** from the Start menu, or click the tray icon, to see the status window. The single Start menu entry replaces the old shortcuts on upgrade. Closing the window leaves the tray icon and daemon running.

| Control | What it does |
| --- | --- |
| **Set up…** | Runs the setup guide and lists the stages you can redo. |
| **Align controllers to headset (15 s)…** in **Settings** | Redoes headset alignment for a new headset or when the controllers are away from your hands. |
| **Overview** and **Stats** | See what needs attention in Overview and all diagnostic checks in Stats. |
| **Finish driver & SteamVR setup** in **Settings** | Installs the sensor driver and registers with SteamVR again. Use it if the installer could not finish, or if the input profile is out of date after a SteamVR update. |
| **Save report** | Saves one zip file with logs, settings and calibration and shows where it went. Nothing is uploaded. |
| **Start daemon**, **Restart daemon**, **Stop daemon** | Manage the daemon by hand. The window and tray menu show the actions that apply. |

A report contains your room's calibration, and its logs can contain device serial numbers, your user folder name and network addresses. Review it before posting it publicly.

The command-line tools are in `%LOCALAPPDATA%\Programs\cv1touch\bin`. `cv1touch sensors` shows whether each sensor has WinUSB; `cv1touch sensors remove` takes it off. Settings and calibration are in `%LOCALAPPDATA%\cv1touch`, and the daemon's log is `%LOCALAPPDATA%\cv1touch\state\logs\cv1touchd.log`.

## Uninstall

Quit SteamVR, keep every sensor plugged in, and uninstall cv1touch from Windows' installed apps. Approve the prompt to remove the sensors' WinUSB driver. Uninstalling closes the tray app, stops the daemon, unregisters the driver from SteamVR and removes its base-station setting.

A sensor that was unplugged keeps WinUSB. To remove it, reinstall cv1touch with that sensor plugged in, then uninstall again.

Settings and calibration stay in `%LOCALAPPDATA%\cv1touch`. Delete that folder to remove them.
