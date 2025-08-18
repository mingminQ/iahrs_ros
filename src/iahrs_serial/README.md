# iahrs_serial
iAHRS RB-SDA-v1 serial communication package

## serial_bridge
Publishes linear acceleration /  angualar velocity / orientation / magnetic field data.  
Please enter one of the two commands to execute.  

``` console
$ ros2 launch iahrs_serial serial_bridge.launch.py
$ ros2 launch iahrs_serial serial_bridge.launch.xml
```

### Unit
| Data | Unit |
|---|---|
| Linear Acceleration | **$m/s^2$** |
| Angular Velocity| **rad/s** |
| Orientation | **Quaternion** |
| Magnetic Field | **T** |

### Topic / Service Names
| Communication Interface | Communication Entities | Interface Name | Description |
|---|---|---|---|
| [Topic] **sensor_msgs/msg/Imu** | Publisher | **/iahrs/imu** | IMU sensor data includes linear acceleration, angular velocity, orientation |
|  [Topic] **sensor_msgs/msg/MagneticField** | Publisher | **/iahrs/magnetic_field** | Magnetic field data, If magnetic field calibration is required, it must be set at the firmware level. |

### QoS
| QoS Policy | QoS Policy Key |
|---|---|
| History | **Keep Last** |
| Depth | **1** |
| Reliability | **Best Effort** |
| Durability | **Volatile** |

### Parameters
| Parameter Name | Unit | Description |
|---|---|---|
| frame_id | - |  |
| imu_topic | - |  |
| magnetic_field_topic | - |  |
