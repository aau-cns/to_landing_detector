# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [UNRELEASED]
**[FEATURE 4]**: `bugfix/dh4_fixes`
### Added
- Median calculation for LRF measurements (over the complete buffer)
- Takeoff hysteresis and successfull takeoff distance (default 0.5 m)

### Fixed
- Measurement adding and rejection based on measurement time


**[FEATURE 3]**: `feat/landing_interface`
### Added
- Interface to topic which provides msg on `/toland/is_landed` with boolean set to true if is_flat().
- `CONTRIBUTORS.md` file.

**[FEATURE 2]**: `feat/landing_detection`
### Added
- Added LRF callback and structures
- Added saftey mechanism to return failure if no measurement was received for the duration of the window size
(parameter: `sensor_readings_window_s_`)
- Added default launchfile.

### Changed
- Created interface sturcture for sensor data, which specific sensor data readings inherit.
- Renamed `main.cpp` to `FlightDetector_Node.cpp`.

**[FEATURE 1]**: `feat/takeoff_interface`
### Added
- Added ROS package structure
- Added IMU flatness detection from AMAZE Autonomy.
- Added service for takeoff request using `std_srvs/Trigger.srv`

### Changed
- Added namespaces for flatness detection (utils) from AMAZE autonomy.
- Updated license headers in utils module.

[Feature 4]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/tree/0f58d100
[Feature 3]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/tree/6d04815b
[Feature 2]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/tree/68994fe4
[Feature 1]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/tree/07ee025c
[Unreleased]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/compare/develop...main
