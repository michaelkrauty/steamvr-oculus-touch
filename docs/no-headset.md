# Without a Rift headset

cv1touch needs a radio to reach the Touch controllers. Without a CV1 headset, a small USB board running [FreeTouch](https://github.com/Juspertinry/freetouch-dongle) firmware 3.1 or later does that job. You still need the CV1 sensors and a SteamVR headset of any kind. A CV1 headset plugged in is always used as the radio, and a dongle beside it is left alone.

Untested with a FreeTouch dongle, but it should work. The [Adafruit nRF52840 USB Key (product 5199)](https://www.adafruit.com/product/5199) is recommended; other nRF52840 boards with Adafruit's UF2 bootloader on S140 6.1.1 (application at `0x26000`) are untested, but should work.

## What to buy

- The Adafruit nRF52840 USB Key, product 5199 specifically: generic Raytac MDBT50Q-RX sticks from other sellers lack Adafruit's UF2 bootloader.
- A short USB 2.0 extension cable, to keep the dongle clear of rear USB 3.0 ports and the sensors' cables.

Boards with the same bootloader layout include the nice!nano v2 (FreeTouch's own board) and April Brother nRF52840 dongles made after 2025-01-17. Do not copy the published file onto a Seeed XIAO nRF52840, a Makerdiary nRF52840 MDK dongle or Nordic's nRF52840 Dongle (PCA10059): their bootloaders place the application elsewhere. The computer's own Bluetooth cannot reach the controllers.

## Install FreeTouch

Use FreeTouch [release v3.1](https://github.com/Juspertinry/freetouch-dongle/releases/tag/v3.1) and follow its [installing instructions](https://github.com/Juspertinry/freetouch-dongle/blob/db8599b/docs/INSTALL.md). On the 5199, hold its button while plugging it in; when the drive `MDBT50QBOOT` appears, copy `freetouch-dongle-3.1.uf2` onto it. Its LED stays dark while FreeTouch runs.

On Linux, install the [udev rule](setup.md#device-access) again so that the dongle's serial port is yours, then plug the dongle in again.

## Pair the controllers

Pairing a controller to the dongle unpairs it from its headset, and pairing it back to a headset needs the Oculus app on Windows. cv1touch does not pair controllers itself.

Stop the daemon (`cv1touch stop`) so that the dongle's port is free, follow FreeTouch's [pairing instructions](https://github.com/Juspertinry/freetouch-dongle/blob/db8599b/docs/PAIRING.md) for each controller, then start the daemon again.

## What works

Buttons, sticks, triggers, capacitive touch, battery level, vibration, orientation and position. With no headset, cv1touch times the sensors' camera exposures from the dongle's clock; position starts a few seconds after the sensors' video does. Each controller's factory calibration is read through the dongle. `cv1touch status`, `cv1touch doctor` and `cv1touch setup` say which radio is in use.

## Back to a headset

Pair each controller to the headset again with the Oculus app on Windows, unplug the dongle and plug the headset in.
