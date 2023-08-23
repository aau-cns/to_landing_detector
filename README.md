# CNS Flight Stack: Takeoff and Land ROS1 Package (to_landing_detector)

 [![Release](https://img.shields.io/github/v/release/aau-cns/to_landing_detector?include_prereleases&logo=github)](https://github.com/aau-cns/to_landing_detector/releases) [![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE) [![Paper](https://img.shields.io/badge/IEEEXplore-10.1109/LRA.2022.3196117-00629B.svg?logo=ieee)](https://doi.org/10.1109/LRA.2022.3196117)

Maintainer: [Martin Scheiber](mailto:martin.scheiber@aau.at) and [Alessandro Fornasier](mailto:alessandro.fornasier@aau.at)

## Credit
This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt, Klagenfurt, Austria.

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

## Getting Started

### Prerequisites
This package is part of the [CNS Flight Stack] and thus depends on the other packages of the flight stack:
- [CNS Flight Stack: Autonomy Engine]


Further, the following libraries are required
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
| `default_sensor`          | default AGL sensor to use | `lrf` |
| `sensor_readings_window`  | time window used for buffering messages to calculate median/mean when checking takeoff/landing condition | `1.0` |
| `angle_threshold`         | threshold in [deg] acceptable as flat (gravity-aligned check) | `10.0` |
| `distance_threshold`      | threshold in [m] acceptable as on the ground (height check) | `0.1` |
| `takeoff_theshold`        | threshold in [m] upon which a successful takeoff is registered | `0.5` |
| `R_IP`                    | rotation calibration between IMU and "body" frame | Identity Matrix |
| `R_LP`                    | rotation calibration between "body" frame and range sensor | Identity Matrix |
| `t_LP`                    | translation calibration between "body" frame and range sensor | Zero Vector |
| `use_median`              | use the median rather than the mean for LRF-based caluclation | `false` |
| `require_srv_call`        | require a service call before any takeoff is detected | `false` |
| `imu_topic`               | ROS topic name for the IMU measurements | `~imu` |
| `lrf_topic`               | ROS topic name for the range measurements | `~lrf` |
| `baro_topic`              | ROS topic name for the barometric measurements | `~baro` |

### Default Launchfile Parameters

The following parameters are part of the default launchfile, and can be set via the command line:
```bash
roslaunch toland_flight toland.launch <PARAM1_NAME>:=<VALUE> <PARAM2_NAME>:=<VALUE> ...
```

| Launch parameter | description | default value |
|---------------|-------------|---------------|
| `default_sensor`          | default AGL sensor to use (`lrf`, `baro`, or `disabled`) | `lrf` |
| `imu_topic`               | ROS topic name for the IMU measurements  | `~imu` |
| `lrf_topic`               | ROS topic name for the range measurements  | `~lrf` |
| `baro_topic`              | ROS topic name for the barometric measurements  | `~baro` |
| `R_IP`                    | rotation calibration between IMU and "body" frame | Identity Matrix |
| `R_PL`                    | rotation calibration between "body" frame and range sensor | Identity Matrix |
| `t_PL`                    | translation calibration between "body" frame and range sensor | Zero Vector |
| `sensor_readings_window`  | time window used for buffering messages to calculate median/mean when checking takeoff/landing condition | `1.0` |
| `angle_threshold`         | threshold in [deg] acceptable as flat (gravity-aligned check) | `10.0` |
| `distance_threshold`      | threshold in [m] acceptable as on the ground (height check) | `0.15` |
| `takeoff_theshold`        | threshold in [m] upon which a successful takeoff is registered | `0.5` |
| `use_median`              | use the median rather than the mean for LRF-based caluclation | `false` |
| `require_srv_call`        | require a service call before any takeoff is detected | `true` |

### Usage without Autonomy Engine

If required the detector can be used without the [CNS FlightStack: Autonomy Engine]. You can interact with the detector using the provided ROS service

```bash
rosservice call /toland/service/takeoff "{}"
# Feedback:
success: False|True
message: "number of measurements -- IMU: x -- LRF: x -- BARO: x"
```

If this service was called once, you will receive a message on the `/toland/is_landed` topic, if a landing was detected. In order for this to happen, the UAV must have taken off successfully (`dist>takeoff_theshold`). Please note that for this feature a **range sensor must be used**.

In case you do not want to execute the service call, you can deactivate it by launching
```bash
roslaunch toland_flight toland.launch require_srv_call:=False
```

## Architecture

Please refer to the academic paper for further insights into the Takeoff and Landing Detector.


## Known Issues

#### Landing Detection Message only transmitted once
The current implementation only transmits a landing detection message once. There it does not check for flatness (as it should not). However, when you then request via the provided service if the vehicle is on the ground, it might return `false` as the flatness (gravity-aligned) condition might not be met.

We are working on this issue, which also requires modification in the [CNS Flight Stack: Autonomy Engine].

#### Barometer triggers landing detection early
This is due to inaccurate barometric measurements (especially indoors) before flight. The current initialization routine sets the reference pressure at startup using the mean of the first 100 measurements. If between startup external influences (even the start of the motors) would change the reference pressure, technically a restart of the node is currently required.

This issue is also WIP but can be circumvented by using a more accurate AGL sensor.

## Package Layout

```console
/path/to/to_landing_detector$ tree -L 3 --noreport --charset unicode
.
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

Copyright (C) 2021-2023 Alessandro Fornasier and Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [alessandro.fornasier@aau.at](mailto:alessandro.fornasier@aau.at?subject=[CNS%20Flight%20Stack]%20to_landing_detector%20package), [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=[CNS%20Flight%20Stack]%20to_landing_detector%20package).


<!-- LINKS: -->
[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[CNS Flight Stack: Autonomy Engine]: https://github.com/aau-cns/autonomy_engine
