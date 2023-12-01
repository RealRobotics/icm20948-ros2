# ROS 2 driver for ICM20948 IMU

This ROS 2 driver of the ICM20948 implements a ROS2 wrapper around these Pimoroni Python library.

* <https://github.com/pimoroni/icm20948-python/tree/master>

My development notes can be found [here](development.md).

The datasheets directory has the PDFs for the the BNO055 and the breakout board we are using.

## Run on the Raspberry Pi

To install on a Raspberry Pi, install ROS2 Humble on RPi.  Then on the RPi, run:

```bash
cd ~
git clone https://github.com/pipebots/pipebot-scripts.git
mkdir ws
cd ws
cp ~/pipebot-scripts/setup/pipebot_repo.bash .
cp ~/pipebot-scripts/vars.bash .
./pipebot_repo.bash imu-bno055
colcon build
ros2 launch imu_bno055 bno055.launch.py
```

## Known issues

Work in progress so expect nothing to work!

Please raise a GitHub issue on this repo if you find a problem.

## Acknowledgements

&copy; 2023 University of Leeds.

The author, A. Blight, asserts his moral rights.
