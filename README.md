# CNS Flight Stack: Takeoff and Land ROS1 Package (to_landing_detector)

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

## License
This software is made available to the public to use (_source-available_),
licensed under the terms of the BSD-2-Clause-License with no commercial use
allowed, the full terms of which are made available in the `LICENSE` file.
No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{cns_flight_stack22,
   author   = {Martin Scheiber and Alessandro Fornasier and Roland Jung and Christoph Boehm and Rohit Dhakate
               and Christian Stewart and Jan Steinbrener and Stephan Weiss and Christian Brommer},
   journal  = {IEEE Robotics and Automation Letters},
   title    = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
   year     = {2022},
   doi      = {10.1109/LRA.2022.3196117}
}
```

## Getting Started

### Prerequesites
This package is part of the [CNS Flight Stack] and thus depends on the other packages of the flight stack:
- [CNS Flight Stack: Autonomy Engine]


Further the following libraries are required
- Eigen
- ROS noetic



### Build

As this is a ROS package, please build it within the catkin environment with

```bash
catkin build to_landing_detector
```

## Usage

The intended usage is together with the [CNS Flight Stack: Autonomy Engine], which will interact with the takeoff and landing detector. Use the provided launchfile to start the ROS1 node

```bash
roslaunch to_landing_detector toland.launch
```

The following parameters can be set to modify the detector's behavior:

### ROS Parameters

| ROS parameter | description | default value |
|---------------|-------------|---------------|
| `sensor_readings_window`  | time window used for buffering messages to calculate median/mean when checking takeoff/landing condition | `1.0` |
| `angle_threshold`         | threshold in [deg] acceptable as flat (gravity-aligned check) | `10.0` |
| `distance_threshold`      | threshold in [m] acceptable as on the ground (height check) | `0.1` |
| `takeoff_theshold`        | threshold in [m] upon which a successful takeoff is registered | `0.5` |
| `R_IP`                    | rotation calibration between IMU and "body" frame | Identity Matrix |
| `R_LP`                    | rotation calibration between "body" frame and range sensor | Identity Matrix |
| `t_LP`                    | translation calibration between "body" frame and range sensor | Zero Vector |
| `lrf_use_median`          | automatic state changes in sequencer (if used without Autonomy Engine) | `false` |
| `imu_topic`               | ROS topic name for the IMU measurements | `/imu` |
| `lrf_topic`               | ROS topic name for the range measurements | `/lrf` |

### Default Launchfile Parameters

The following parameters are part of the default launchfile, and can be set via the command line:
```bash
roslaunch toland_flight toland.launch <PARAM1_NAME>:=<VALUE> <PARAM2_NAME>:=<VALUE> ...
```

| Launch parameter | description | default value |
|---------------|-------------|---------------|
| `imu_topic`               | ROS topic name for the IMU measurements  | `/imu` |
| `lrf_topic`               | ROS topic name for the range measurements  | `/lrf` |
| `R_IP`                    | rotation calibration between IMU and "body" frame | Identity Matrix |
| `R_PL`                    | rotation calibration between "body" frame and range sensor | Identity Matrix |
| `t_PL`                    | translation calibration between "body" frame and range sensor | Zero Vector |
| `sensor_readings_window`  | time window used for buffering messages to calculate median/mean when checking takeoff/landing condition | `1.0` |
| `angle_threshold`         | threshold in [deg] acceptable as flat (gravity-aligned check) | `10.0` |
| `distance_threshold`      | threshold in [m] acceptable as on the ground (height check) | `0.15` |
| `takeoff_theshold`        | threshold in [m] upon which a successful takeoff is registered | `0.5` |
| `lrf_use_median`          | use the median rather than the mean for LRF-based caluclation | `false` |

### Usage without Autonomy Engine

If required the detector can be used without the [CNS FlightStack: Autonomy Engine]. You can interact with the detector using the provided ROS service

```bash
rosservice call /toland/service/takeoff "{}"
# Feedback:
success: False|True
message: "number of measurements -- IMU: x -- LRF: x -- BARO: x"
```

If this service was called once, you will receive a message on the `/toland/is_landed` topic, if a landing was detected. In order for this to happen, the UAV must have taken off successfully (`dist>takeoff_theshold`). Please note that for this feature a **range sensor must be used**.

## Architecture

Please refer to the academic paper for further insights of the Takeoff and Landing Detector.


## Known Issues

#### Landing Detection Message only transmitted once
The current implementation only transmits a landing detection message once. There it does not check for flatness (as it should not). However, when you then request via the provided service if the vehicle is on the ground, it might return `false` as the flatness (gravity-aligned) condition might not be met.

We are working on this issue, which requires modification also on the [CNS Flight Stack: Autonomy Engine] - see also issue #2.

#### Barometer triggers landing detection early
This is due to inaccurate barometric measurements (especially indoors) before flight. The current initialization routine sets the reference pressure at startup using the mean of the first 100 measurements. If between startup external influences (even start of the motors) would change the reference pressure, technically a restart of the node is currently required.

This issue is also WIP, but can be circumvent by using a more accurate AGL sensor.

## Package Layout

```console
/path/to/to_landing_detector$ tree -L 3 --noreport --charset unicode
.
|-- CHANGELOG.md
|-- CMakeLists.txt
|-- CONTRIBUTORS.md
|-- include
|   |-- toland_flight
|   |   `-- FlightDetector.hpp
|   `-- utils
|       |-- mathematics.h
|       |-- physics.h
|       `-- sensors.h
|-- launch
|   `-- toland.launch
|-- LICENSE
|-- package.xml
|-- README.md
`-- src
    |-- FlightDetector.cpp
    `-- FlightDetector_Node.cpp
```

---

Copyright (C) 2021-2022 Martin Scheiber and Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=toland_flight%20package), [alessandro.fornasier@aau.at](mailto:alessandro.fornasier@aau.at?subject=toland_flight%20package).


<!-- LINKS: -->
[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[CNS Flight Stack: Autonomy Engine]: https://github.com/aau-cns/autonomy_engine
