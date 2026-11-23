# First Windows hardware test

This checklist is for an invited tester. Windows USB streaming and gameplay have not yet been validated. Do not mistake passing automated tests for a guarantee that your devices will work.

## Before volunteering

You need Windows 11 x64, a headset already working in SteamVR, a CV1 headset connected by USB as a radio receiver, its already-paired left/right CV1 Touch controllers, and preferably at least two fixed CV1 sensors with overlapping views. The CV1 display, new controller pairing, Rift S and Quest controllers are not supported. If your main headset needs the Meta/Oculus PC application, keep it; do not remove software you need just to try this.

Allow time for initial physical calibration. You will approve a Windows administrator prompt for the sensor driver, follow a guided text window, and move the controllers as instructed. No terminal commands, Device Manager edits, certificates, or security-setting changes should be needed. If any of those become necessary, stop and report it rather than improvising.

## Install and set up

1. Download `cv1touch-setup-<version>.exe` from the [latest release](https://github.com/michaelkrauty/steamvr-oculus-touch/releases/latest).
2. Follow [the Windows guide](windows.md). If installation says setup is incomplete, fix the stated prerequisite and use **Finish setting up cv1touch**. Do not repeatedly run the installer or delete calibration to try to fix an unexplained problem.
3. Wake both Touch controllers, start SteamVR with your usual headset tracking, and open **Set up cv1touch**. Complete the guided steps. If a step fails twice without a clear reason, stop and send a report instead of forcing it through.

## Check ordinary use

Start in SteamVR, with clear space around you. Only continue while the controllers look correctly aligned with your real hands.

- Check both controller models, left/right identity, position and rotation around the play area.
- Check each trigger, grip, thumbstick and face button. Confirm vibration in both hands. The guide includes a vibration check.
- Briefly cover a controller's ring, then uncover it. Tracking should recover without needing setup again.
- Let the controllers sleep, then wake both with a face button. They should return without reinstalling or recalibrating.
- Quit and restart SteamVR, then restart Windows once. The controllers should work again after waking; the daemon should start automatically.
- Try one familiar game for 15–30 minutes. Tell us which game and whether you changed bindings. The right Oculus button is an optional per-game binding, not a requirement for basic tracking.

Do not move the mounted sensors during this first check. Once ordinary use works, the maintainer may ask for a separate USB replug or sleep/resume check; that is not a reason to risk your existing room setup.

## What to send back

Whether it worked or not, leave cv1touch installed and choose **Send a cv1touch report** in the Start menu. It saves a ZIP on your desktop and opens the folder. Send that file privately to your tester contact, with a few words of your own: the game you tried, and what did not work, for example:

> I was trying to …
>
> I expected …, but … happened.
>
> It affects the left hand / right hand / both / a sensor.
>
> It started immediately / after sleep / after a restart / after about … minutes.

The ZIP includes settings, room calibration, device identities, installer/repair logs and runtime logs. It does not upload itself. Review it before sharing publicly. If the installer or report tool cannot start, send a screenshot of the complete Windows error and the installer filename; do not disable Windows security or download DLLs from another site.

To stop testing, quit SteamVR, keep the sensors connected, and uninstall cv1touch through Windows' installed apps. Approve sensor-driver cleanup. Your settings are retained in case you try a later build.
