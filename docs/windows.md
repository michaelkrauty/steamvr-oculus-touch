# cv1touch on Windows

cv1touch runs Oculus Rift CV1 Touch controllers in SteamVR without any Oculus software. On Windows it is one installer, and it needs nothing else but SteamVR.

**Status:** new. The Windows build passes its tests and the installer installs and uninstalls cleanly on a test machine, but no one has yet played with it: you may be the first. When something is off, send a report (below); that is what makes it work for the next person.

## What you need

- A Rift CV1 headset and its Touch controllers, paired to it as they came.
- One or more Rift sensors, plugged in to USB 3 ports where possible.
- Windows 11 (64-bit) with [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) installed.
- No Oculus software. If the Oculus app was ever installed, uninstall it first: its sensor driver would keep the sensors from cv1touch.

## Install

1. Plug in the sensors, and quit SteamVR if it is running.
2. Download `cv1touch-setup-<version>.exe` from the [releases](https://github.com/michaelkrauty/steamvr-oculus-touch/releases) and run it. Windows may warn that it comes from an unknown publisher: cv1touch is not signed. It installs for you alone.
3. Once during the install, Windows asks whether to allow a change (and, if your account is not an administrator's, for an administrator's password): that is `cv1touch sensors install`, which gives your sensors Windows' own USB driver ("WinUsb Device", which Windows ships and Microsoft signed). cv1touch reads the sensors through it. Nothing else is installed on your system, and the uninstaller takes it off again.
4. The installer registers cv1touch with SteamVR and starts it. From then on it starts whenever you log in, without a window. If a sensor was not plugged in, or SteamVR was running or missing, the installer says so at the end; fix that, then choose **Finish setting up cv1touch** in the Start menu, which does those two steps again and says what it finds.

## Set up

Plug in the sensors and the headset, then choose **Set up cv1touch** in the Start menu (the installer offers to run it at once). It calibrates where your sensors stand and how the controllers are held; it says what to do at each step, and the whole thing takes a few minutes. [Setting up](setup.md) explains each step.

Then start SteamVR. The controllers appear as soon as they are awake.

## When something is wrong

- **Check cv1touch** in the Start menu lists what cv1touch needs and whether it is in place: SteamVR, the sensors and their driver, the daemon.
- **Send a cv1touch report** saves one zip file on your desktop with the logs and settings someone helping you needs, and opens the folder. It says what it holds: the calibration describes where your sensors stand in your room, and SteamVR's log names your other VR devices, your user folder and your network addresses. Attach it to an [issue](https://github.com/michaelkrauty/steamvr-oculus-touch/issues).
- **Finish setting up cv1touch** gives the sensors their driver and registers with SteamVR again, for when the installer could not.
- **Stop cv1touch** and **Start cv1touch** stop and start the daemon by hand. It otherwise starts at logon.

The command line is there too, in `%LOCALAPPDATA%\Programs\cv1touch\bin`: `cv1touch doctor`, `cv1touch sensors`, `cv1touch status`, `cv1touch report`, `cv1touch stop`. Your settings and calibration live in `%LOCALAPPDATA%\cv1touch`, the daemon's log in `%LOCALAPPDATA%\cv1touch\state\logs\cv1touchd.log`.

## Uninstall

Uninstall cv1touch from Windows' installed apps. It stops the daemon, unregisters it from SteamVR and takes the sensors' driver off again (Windows asks once more). Your settings and calibration stay in `%LOCALAPPDATA%\cv1touch`, in case you install it again; delete that folder to remove them.
