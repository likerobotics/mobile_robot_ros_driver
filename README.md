# Mobile robot ROS driver

This ROS package provides a control interface via a serial port. The code is implemented in Python with the minimum number of dependencies.

> [!WARNING]
> There are parameters to be specified in `config/omnicar.yaml`: serial port and the names of wheels. 

Launch via command:
```
roslaunch mobile_robot_ros_driver bringup.launch
```

serialdrive.py - the main script, provides the connection to the serial port, parses the odometry from hardware, and also tracks the topic/cmd and sends proper commands to serial.

example.py - simple controller that constantly provides height-level commands to the driver (just for testing).


Related compatible hardware project [here](https://github.com/industrial-robotics-lab/OmniCar-arduino).

> [!NOTE]
> /cmd -  topic for commands
> /odom - Odometry topic (implemented on hardware side)
> /state/pos/* - odometry of each wheel (could be useful for some purposes)