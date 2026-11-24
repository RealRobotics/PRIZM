# PRIZM

[![CI](https://github.com/RealRobotics/PRIZM/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/RealRobotics/PRIZM/actions/workflows/ci.yml)

A collection of ROS2 (Jazzy) packages being developed for the PRIZM project.

There's an accompanying docker image which can be found [in this repo](https://github.com/RealRobotics/PRIZM_docker).

## Content Summary

* [hitech_bindings](hitech_bindings) ROS2 driver for Hitec CAN servos/motors, talking to the [CAN feather firmware](can_feather_firmware) over serial.
* [prizm_interfaces](prizm_interfaces) Custom messages and services used by the drivers.
* [prizm_slam](prizm_slam) Ties up an LIO and SLAM packages to carry out some basic slam.
* [can_feather_firmware](can_feather_firmware) CircuitPython firmware for the CAN feather board (not a ROS package).

## Testing and linting

Unit tests and linters both run through `colcon test`. The linters are registered as tests via
`ament_lint_auto`: clang-format, cppcheck, flake8, pep257, lint_cmake and xmllint. Lint failures
are reported alongside gtest failures:

```bash
cd ~/prizm/ros2_ws
colcon build --packages-up-to hitech_bindings prizm_slam
colcon test --packages-select prizm_interfaces hitech_bindings prizm_slam
colcon test-result --verbose
```

C++ style is defined in [.clang-format](.clang-format). To catch formatting problems before you push:

```bash
pip install pre-commit
pre-commit install            # runs on every commit
pre-commit run --all-files    # run once over the whole repo
```

## CI

[.github/workflows/ci.yml](.github/workflows/ci.yml) runs on every pull request and push to `main`:

* **build-test**: in a `ros:jazzy` container, it installs dependencies with rosdep, then runs `colcon build` and
  `colcon test`. Test and lint results are uploaded and posted to the PR as a test report.
* **pre-commit**: runs the same hooks as above.

## Author

Vittorio Francescon @ University of Leeds
