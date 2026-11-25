# cv1touch GUI

The optional `cv1touch-gui` program provides a tray icon and an on-demand status window. It requires an existing cv1touch installation. Closing the window or tray does not stop the daemon.

Build it separately from the headless programs with Rust 1.95 or newer:

```sh
cd gui
cargo build --locked --release
```

On Windows, place `cv1touch-gui.exe` beside the installed cv1touch programs. Run `cv1touch-gui` to start the tray icon; running it again opens the status window. Click the icon to show or hide the window. The tray menu starts, restarts or stops the daemon, runs setup and saves a diagnostic report. A report can contain private information; review it before sharing it.

The window's Overview tab shows the controllers, sensors and calibration at a glance, with advice when something needs attention. The Stats tab lists the daemon's tracking counters, each sensor's video and agreement figures, the saved calibration's fit quality and every diagnostic check; hover over any of them for what it means.

The Cameras window (`cv1touch-gui --cameras`, the status window's Cameras button or the tray menu) shows each sensor's live infrared image with the blobs it detects: blue for the left controller, green for the right, violet for the headset, grey for unmatched ones. Click an image to see that sensor alone at a higher resolution; Esc goes back. Images flow only while the window is open.

On Linux, the desktop needs a StatusNotifierItem host. GNOME can display the icon with an AppIndicator extension.

On Linux, `cv1touch-gui --install-desktop` adds a menu entry and icons under `~/.local/share` (or `$XDG_DATA_HOME`). Wayland desktops need them to show the window's icon. Run it again after moving the program; `cv1touch-gui --uninstall-desktop` removes them.
