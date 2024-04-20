# ROS2 Sensor Data Processing System

## Overview
This ROS2-based system is designed to simulate, process, and handle high-frequency data from 3-DOF sensors. It includes the provided sensor simulator, a filtering service server, and a client that publishes the filtered data. The system is built to handle high-frequency operations efficiently.

## System Components

### Sensor Simulator (`sensor.py`)
- **Functionality**: Simulates a 3-DOF force-torque sensor by generating random data. It operates over TCP/IP, accepting connections and responding with data based on the requested sample size.

### ROS2 Service Server (`server.py`)
- **Functionality**: Connects to the sensor simulator to retrieve data, applies a moving average filter to smooth the data and serves this data through a ROS2 service.
- **Implementation:**:
  - `SensorFilter`: Maintains a rolling window of sensor data and computes the moving average. As the signal is random and goal of the project is the data pipeline rather than signal processing moving average filter with relaive window size is used, however in real applications, there are alternative real-time filters with better impulse reponse. We also are ignoring the signal quality effects of the irregular effective sampling of the pipeline and down-sampling. With a real signal proper decimation needs to be applied to prevent aliasing and other undesired effects.
  - `SensorServer`: A ROS2 node that handles requests for filtered sensor data, manages network communications, and integrates the data filtering process. The sampling frequency of the sensor is 2000Hz. At the topic we want to have a 500Hz frequency for both sensors. That's an efffective down-sampling ratio of 8. However due to sensor read overhead and other system variables, in choosing the sample read size in each cycle there is a tradeof between how much we load the system and how fresh our data is. In a real application it has to be tuned or even an adaptive strategy taken. Here we have chosen 500Hz for each server node and it can be modified in the launch file. In this frequency relative sample-request-size, filter-window-size equal 10 and 20 correspondingly. The processed data buffer size is also equivalent to one second of signal.
- **ROS2 Concepts**: Utilizes custom services (`GetSensorData.srv`), parameters, and intra-process communication. The reponse basically follows the standard StampedWrench response format.

### ROS2 Service Client (`client.py`)
- **Functionality**: Subscribes to the ROS2 service provided by `SensorServer`, retrieves filtered sensor data, and publishes it to a single ROS2 topic at 500 Hz. Both sensors are published to the same topic, distinguised by their header.frame-id.
- **Implementation**: Manages timing to maintain a consistent publishing rate, handles potential service unavailability, and queues data for publishing to ensure real-time performance. A buffer seperates data collection and publication making the constant 500Hz topic frequency possible. The frequency of reading from each server is 250Hz. If server falls behind the client, the current strategy is to republish the latest buffered data. All the timestamps are also updated in the client as the client has it's own constant clock. A more sophisticated approach if we are allowed to modify the signal, is to quantize and interpolte the data received from the server. Even some predication algorithm (e.g. ARIMA)can be added to improve the tolerance to data loss, however architecturally it makes more sense for those to be out-sourced to a separate node (e.g. a topic subscriber) focusing more on higher level signal processing.

### Custom Service Definition (`sensor_interfaces/srv/GetSensorData.srv`)
- **Structure**:
--- std_msgs/Header header
geometry_msgs/Wrench data
- **Purpose**: Defines the service interface for transmitting filtered sensor data, including timestamp and frame metadata.

### Launch File (`launch/sensor_service_launch.py`)
- **Purpose**: Configures and initiates multiple instances of the sensor servers and the client node, setting parameters such as IP addresses, port numbers, and operating frequencies.
- **Launch Parameters**: Allows dynamic configuration of sensor nodes and clients, facilitating easy adjustments to deployment environments and operational parameters.

### Docker Configuration (`Dockerfile`)
- **Environment Setup**: Provides a reproducible environment for running the ROS2 system, including all necessary dependencies.

## Setup and Running Instructions

### Prerequisites
- ROS2 Humble Hawksbill installed on your system or a compatible Docker environment.
- Python 3.8 or higher.

### Local Setup
Open a new terminal at the root of the repo.
```
colcon build
```

### Run Sensor Simulator File
Open a new terminal at the root of the repo.
Execute the following command to launch the system:
```
python3 src/sensor.py
```

### Launch ROS2 Network
Open a new terminal at the root of the repo.
Execute the following command to launch the system:
```
source install/setup.bash
ros2 launch sensor_service launch.py
```

### Monitor ROS2 Network
Open a new terminal at the root of the repo.
Execute the following command to launch the system:
```
source install/setup.bash
ros2 topic echo sensor_service/sensor_data
```
or
```
source install/setup.bash
ros2 topic hz sensor_service/sensor_data
```

### Development and Maintenance
- **Logging**: The system is equipped with detailed logging for debugging and maintenance purposes.
- **Performance Tuning**: Parameters such as ip and port address, server frequency and sizes may need adjustment/tuning based on the specific deployment environment and performance requirements.

