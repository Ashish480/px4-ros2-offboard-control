#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time
import os

# Locations of PX4-Autopilot and QGroundControl (override with environment variables)
PX4_DIR = os.environ.get("PX4_DIR", "~/PX4-Autopilot")
QGC_APPIMAGE = os.environ.get("QGC_APPIMAGE", "~/qgc_temp/QGroundControl.AppImage")

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    f"cd {PX4_DIR} && make px4_sitl gz_x500",

    # Run QGroundControl
    f"{QGC_APPIMAGE}",
]


def main():
    # Loop through each command in the list
    for command in commands:
        # Each command is run in a new tab of the gnome-terminal
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

        # Pause between each command
        time.sleep(1)


if __name__ == '__main__':
    main()
