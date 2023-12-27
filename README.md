# Gym Franka Server
The is the server-side code intended to run on a realtime kernel with ros-noetic and franka-ros installed.

## Getting started
1. Setup franka-ros for ros-noetic as guided in https://frankaemika.github.io/docs/installation_linux.html.
2. Clone this repository.
3. Run ```bash ./setup.sh``` from the project's directory.
4. Start a new terminal so the workspace is properly sourced.

## Additional setups
1. To provent the CPU from entering ```Powersave``` mode and causing communication delays, install indicator-cpufreq by running ```sudo apt-get install indicator-cpufreq``` and change to ```Performance``` mode.
2. Adjust the IPs of this PC and the robot according to your networking setup.
