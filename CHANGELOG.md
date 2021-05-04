# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [UNRELEASED]
**FEATURE 2**: `feat/landing_detection`
### Added
- Added LRF callback and structures

### Changed
- Created interface sturcture for sensor data, which specific sensor data readings inherit.

**[FEATURE 1]**: `feat/takeoff_interface`
### Added
- Added ROS package structure
- Added IMU flatness detection from AMAZE Autonomy.
- Added service for takeoff request using `std_srvs/Trigger.srv`

### Changed
- Added namespaces for flatness detection (utils) from AMAZE autonomy.
- Updated license headers in utils module.

[Feature 1]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/tree/07ee025c
[Unreleased]: https://gitlab.aau.at/aau-cns/ros_pkgs/toland_flight/-/compare/develop...main
