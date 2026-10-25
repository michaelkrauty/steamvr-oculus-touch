# cv1touch on Windows

cv1touch adds original Oculus Rift CV1 Touch controllers to an existing SteamVR setup. There is one installer; no command-line knowledge, Python, developer tools, or separate Visual C++ runtime installation is needed.

**Status:** new. The Windows build passes its tests and the installer installs and uninstalls cleanly on a test machine, but no one has yet played with it: you may be the first. When something is off, send a report (below); that is what makes it work for the next person.

## What you need

- An independently tracked headset already working in SteamVR. **This does not make the Rift CV1 display work.** The CV1 headset is only a USB radio receiver for the controllers; its HDMI cable is not needed.
- A Rift CV1 headset and its original left/right Touch controllers, already paired to that headset, with charged batteries. Pairing is not implemented here. Rift S and Quest controllers are not supported.
- At least two CV1 sensors with overlapping views of your play area, plugged directly into USB 3 ports where possible. One sensor is not recommended. Avoid putting every sensor on one hub.
- Windows 11 (64-bit) with [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) installed.
- No active Oculus/Meta PC software: it can compete for the same USB devices. If you still need that software for your main headset, do not uninstall it blindly; discuss that setup before testing cv1touch.
- Permission to approve Windows' administrator prompt for the sensor driver, or someone who can enter an administrator password. Keep the installer and SteamVR running as your normal user; only sensor-driver installation needs elevation.

## Install

1. Install SteamVR, start it once with your usual headset, and confirm that headset works. Then quit SteamVR. Plug in the CV1 headset's USB cable and every sensor.
2. Download `cv1touch-setup-<version>.exe` from the [releases](https://github.com/michaelkrauty/steamvr-oculus-touch/releases) and run it. Windows may warn that it comes from an unknown publisher: cv1touch is not signed. It installs for you alone.
3. Once during the install, Windows asks whether to allow a change (and, if your account is not an administrator's, for an administrator's password): that is `cv1touch sensors install`, which gives your sensors Windows' own USB driver ("WinUsb Device", which Windows ships and Microsoft signed). cv1touch reads the sensors through it. Nothing else is installed on your system, and the uninstaller takes it off again.
4. The installer registers cv1touch with SteamVR and starts it. From then on it starts whenever you log in, without a window. Installation pauses if SteamVR is running. If a sensor was missing, permission was declined, or SteamVR could not be configured, the installer says setup is incomplete and does not send you straight into calibration. Fix the message, then choose **Finish setting up cv1touch** in the Start menu. It stops tracking safely, retries the drivers and registration, and starts tracking again without resetting calibration. Restart Windows if requested.

The installer is unsigned. Windows may show **More info → Run anyway** for a file downloaded from this repository. Do not disable antivirus, SmartScreen, driver-signature enforcement, or Secure Boot. If Windows does not offer a normal way to approve the download, stop and send a screenshot of the warning.

## Set up

Wake both Touch controllers by pressing a face button, start SteamVR with your usual headset tracking, then choose **Set up cv1touch** in the Start menu (the installer offers this when driver setup succeeds). A text window guides you; you do not need to type commands. Read each instruction, press Enter when ready, and leave the window open. Allow about 10–20 minutes for the first setup, with room to move safely. Your sensors must remain fixed after calibration.

The guide measures sensor positions, floor level, controller models, buttons/sticks, and headset alignment. During headset alignment you hold the controllers firmly against your usual headset and follow vibration cues. Do not attach them where they cover tracking sensors or cameras. If a step fails, it explains why and offers a retry; finished steps are kept. Missing calibration in **Check cv1touch** is normal before this guide is complete. [Setting up](setup.md) explains each step.

Then start SteamVR. The controllers appear as soon as they are awake. SteamVR shows them as Oculus Touch (cv1touch), which uses each game's Oculus Touch bindings. To use the right controller's Oculus button in a game, bind it in SteamVR, with the game running: [Bind the right Oculus button](setup.md#bind-the-right-oculus-button).

## When something is wrong

- **Check cv1touch** in the Start menu lists what cv1touch needs and whether it is in place: SteamVR, the sensors and their driver, the daemon.
- **Send a cv1touch report** saves one zip file on your desktop and opens the folder. Send that file to the person helping you, together with what you were doing and a screenshot of the error. It includes installer/repair logs, daemon and SteamVR logs, and settings/calibration. **Review it before posting publicly:** calibration describes your room layout; logs can contain device serial numbers, your user-folder name and network addresses. Share it privately with your tester contact if preferred. Creating a report uploads nothing.
- **Finish setting up cv1touch** gives the sensors their driver and registers with SteamVR again, for when the installer could not, or when **Check cv1touch** says the input profile is out of date after a SteamVR update.
- **Stop cv1touch** and **Start cv1touch** stop and start the daemon by hand. It otherwise starts at logon.

The command line is there too, in `%LOCALAPPDATA%\Programs\cv1touch\bin`: `cv1touch doctor`, `cv1touch sensors`, `cv1touch status`, `cv1touch report`, `cv1touch stop`. Your settings and calibration live in `%LOCALAPPDATA%\cv1touch`, the daemon's log in `%LOCALAPPDATA%\cv1touch\state\logs\cv1touchd.log`.

## Uninstall

Uninstall cv1touch from Windows' installed apps. It stops the daemon, unregisters it from SteamVR and takes the sensors' driver off again (Windows asks once more). Your settings and calibration stay in `%LOCALAPPDATA%\cv1touch`, in case you install it again; delete that folder to remove them.
