ethzasl_msf_noros
=====================

## Description
Transplant version of [ethasl_msf](https://github.com/ethz-asl/ethzasl_msf]). It is a time delay compensated single and multi sensor fusion framework based on an EKF.
Please see the wiki for more information: https://github.com/ethz-asl/ethzasl_msf/wiki

## Changes
- A simple config reader dep on YAML-CPP as substitution of ROS param server.
- Message definitions look like ROS in `msf_struct`.
- `msf_IMUHandler_NOROS.h` simple implemented sensor message collector.
- `msf_updates/pose_sensor_handler/*` and `msf_updates/position_sensor_handler/*` get rid of ROS dependency.
- an example named `pose_position_msf` including two test datasets. `vicon.txt` and `imu.txt` are extract from original dataset.bag

## Notice
Without ROS support, the filter is degenerated to `single-thread model`. If concurrent or RPC process are required, you must implement these feature by yourself, with callback function in `sensormanager`.

## Build dependency
You should install and assign dep libs by your self. Includes:

    find_package(Eigen3 REQUIRED)
    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)
    find_package(Glog REQUIRED)
    find_package(yaml-cpp REQUIRED)
    find_package(PCL REQUIRED)
    
## License
The source code is released under the Apache 2.0 license
