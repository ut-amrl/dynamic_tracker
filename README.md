[![Build Status](https://travis-ci.com/ut-amrl/dynamic_tracker.svg?branch=master)](https://travis-ci.com/ut-amrl/dynamic_tracker)

# Dynamic Tracker

Multi-sensor, multi-platform dynamic tracker.

## Code checkout
git clone --recurse-submodules

## Dependencies

```bash
sudo apt install libgflags-dev libgoogle-glog-dev libeigen3-dev  libgtest-dev liblua5.1-dev libceres-dev
```

## Compiling
1. Add the working directory to the `ROS_PACKAGE_PATH` environment variable with:

   ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```
1. Run `make`.

## Development
1. This project follows the [Google C++ Style guide](https://google.github.io/styleguide/cppguide.html)
1. A helpful linter: [cpplint](https://github.com/cpplint/cpplint)
1. Please make sure to add your name to the copyright notice for any files you edit.
