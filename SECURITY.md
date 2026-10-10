# Security

The daemon and SteamVR driver run as the desktop user and access USB/hidraw devices. They do not expose a network service. Do not run them as root or expose the control socket to other users. Only use recordings and configuration files from sources you trust; parsing robustness does not make an arbitrary device or file safe.

Use GitHub's private vulnerability reporting where available for a security-sensitive issue. If it is unavailable, open an issue requesting a private reporting channel without publishing exploit details or personal data. Include the affected commit, reproducible conditions, impact, and a proposed mitigation when possible.

There are no long-term-support releases yet. Security fixes target the current development branch; update from a reviewed revision and retain configuration backups.
