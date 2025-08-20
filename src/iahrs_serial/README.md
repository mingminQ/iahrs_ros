# iahrs_serial
iAHRS RB-SDA-v1 serial communication package

## serial_bridge
Publishes linear acceleration /  angualar velocity / orientation / magnetic field data with 100Hz publication rate.  
It uses data received asynchronously from sensors. 

### How to launch
Please enter one of the two commands to execute.  

``` bash
$ ros2 launch iahrs_serial serial_bridge.launch.py
```
``` bash
$ ros2 launch iahrs_serial serial_bridge.launch.xml
```

### Unit
It does not mean the unit received from the sensor, but the unit converted by **serial_bridge** and finally published.  
| Data                | Unit           |
| ------------------- | -------------- |
| Linear Acceleration | **m/s^2**      |
| Angular Velocity    | **rad/s**      |
| Orientation         | **Quaternion** |
| Magnetic Field      | **T**          |

### Topic / Service Names
| Interface | Entitiy   | Type                              | Name                                  | Description                                                                                           |
| --------- | --------- | --------------------------------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Topic     | Publisher | **sensor_msgs/msg/Imu**           | **/iahrs/imu**(changeable)            | IMU sensor data includes linear acceleration, angular velocity, orientation.                          |
| Topic     | Publisher | **sensor_msgs/msg/MagneticField** | **/iahrs/magnetic_field**(changeable) | Magnetic field data, If magnetic field calibration is required, it must be set at the firmware level. |

### QoS
| QoS Policy  | QoS Policy Key  |
| ----------- | --------------- |
| History     | **Keep Last**   |
| Depth       | **1**           |
| Reliability | **Best Effort** |
| Durability  | **Volatile**    |

### Parameters
| Parameter Name                    | Unit     | Description                                                                                 |
| --------------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| frame_id                          | -        | Frame ID of IMU data and magnetic field data.                                               |
| imu_topic                         | -        | IMU data publication topic name.                                                            |
| magnetic_field_topic              | -        | Magnetic field publication topic name.                                                      |
| port_path                         | -        | Serial port path. ( e.g. /dev/ttyUSB0 )                                                     |
| baud_rate                         | -        | Serial communication speed. only 115200 or 9600 can be selected. But 115200 is recommended. |
| remove_gravitational_acceleration | **bool** | If True, publishes data with gravitational acceleration removed from linear acceleration.   |

## interactive_bridge
...
