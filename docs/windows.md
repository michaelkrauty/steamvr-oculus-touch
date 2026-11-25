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
5. The installer registers the driver with SteamVR and starts cv1touch. From then on it starts at logon, without a window.

If a step could not be done, the installer says what to do, such as plugging in a sensor or approving the prompt. Do that, then choose **Finish setting up cv1touch** in the Start menu. Restart Windows if it asks.

## Set up

Wake both controllers, start SteamVR with your headset tracking, and choose **Set up cv1touch** in the Start menu (the installer offers to start it). Follow the text window; it is the same guide as on Linux, described in [guided setup](setup.md#guided-setup). Allow 10 to 20 minutes and room to move. Keep the sensors fixed afterwards.

Until setup is complete, **Check cv1touch** shows a `todo setup` line. That is expected.

SteamVR shows the controllers as **Oculus Touch (cv1touch)** and uses each game's Oculus Touch bindings. To use the right Oculus button, [bind it](setup.md#bind-the-right-oculus-button).

## Start menu

| Entry | What it does |
| --- | --- |
| **Set up cv1touch** | Runs the setup guide. When setup is complete, lists the stages to redo. |
| **Line up the controllers with your headset** | The 15-second headset alignment, for a new headset or when the controllers are away from your hands. |
| **Check cv1touch** | `cv1touch doctor`: checks SteamVR, the sensors and their driver, and the daemon, and says what to fix. |
| **Finish setting up cv1touch** | Installs the sensor driver and registers with SteamVR again. Use it when the installer could not, or after a SteamVR update when Check cv1touch says the input profile is out of date. |
| **Send a cv1touch report** | Saves one zip file on your desktop with logs, settings and calibration. Nothing is uploaded. |
| **Start cv1touch**, **Stop cv1touch** | Start or stop the daemon by hand. |

A report contains your room's calibration, and its logs can contain device serial numbers, your user folder name and network addresses. Review it before posting it publicly.

The command-line tools are in `%LOCALAPPDATA%\Programs\cv1touch\bin`. `cv1touch sensors` shows whether each sensor has WinUSB; `cv1touch sensors remove` takes it off. Settings and calibration are in `%LOCALAPPDATA%\cv1touch`, and the daemon's log is `%LOCALAPPDATA%\cv1touch\state\logs\cv1touchd.log`.

## Uninstall

Quit SteamVR, keep every sensor plugged in, and uninstall cv1touch from Windows' installed apps. Approve the prompt to remove the sensors' WinUSB driver. Uninstalling stops the daemon, unregisters the driver from SteamVR and removes its base-station setting.

A sensor that was unplugged keeps WinUSB. To remove it, reinstall cv1touch with that sensor plugged in, then uninstall again.

Settings and calibration stay in `%LOCALAPPDATA%\cv1touch`. Delete that folder to remove them.
