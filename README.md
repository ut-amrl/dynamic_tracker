# Dynamic Tracker

Multi-sensor, multi-platform dynamic tracker.

## Code checkout
git clone --recurse-submodules

## Dependencies

```bash
sudo apt install libgflags-dev libgoogle-glog-dev libeigen3-dev  libgtest-dev liblua5.1-dev
```

## Compiling
1. Add the working directory to the `ROS_PACKAGE_PATH` environment variable with:

   ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```
1. Run `make`.
