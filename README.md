# ros2_autoware_lgsvl

## Ros2 + Autoware.Auto + LGSVL

Currently integrated:

* Ubuntu 20.04 Focal Rosa
* Ros2 Foxy
* Autoware.Auto
* lgsvl_bridge
* lgsvl_msgs - the package delivering LGSVL message types for ROS
* package from this repo, which converts boundaries estimated by Autoware to the coordinates which can be properly rendered by LGSVL groundtruth viualizer.
* Running the node chain to estimate object boundaries with Autoware euclidian cluster implemenation


![Example how to render autoware bounding boxes to LGSVL](https://i.imgur.com/ecWMZiZ.png)

## How to run this demo

1. Set up the following sensors in your vehicle
```json
[
  {
    "type": "3D Ground Truth Visualizer",
    "name": "3D Ground Truth Visualizer",
    "params": {
      "Topic": "/ar4development/lgsvl_detections"
    },
    "transform": {
      "x": 0,
      "y": 1.975314,
      "z": -0.3679201,
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    }
  },
  {
    "type": "Lidar",
    "name": "Lidar",
    "params": {
      "LaserCount": 32,
      "MinDistance": 0.5,
      "MaxDistance": 100,
      "RotationFrequency": 10,
      "FieldOfView": 41.33,
      "CenterAngle": 10,
      "Compensated": true,
      "PointColor": "#ff000000",
      "Topic": "/ar4development/points_raw",
      "Frame": "velodyne",
      "MeasurementsPerRotation": 360
    },
    "transform": {
      "x": 0,
      "y": 2.312,
      "z": -0.3679201,
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    }
  },
  {
    "type": "GPS Odometry",
    "name": "GPS Odometry",
    "params": {
      "Frequency": 12.5,
      "Topic": "/ar4development/gps_odometry",
      "Frame": "gps",
      "IgnoreMapOrigin": true
    },
    "transform": {
      "x": 0,
      "y": 0,
      "z": 0,
      "pitch": 0,
      "yaw": 0,
      "roll": 0
    }
  }
]
```

2. Clone this repo and build image from `Dockerfile`
3. Execute the image with port `9090` exposed to `9090`
4. Run simulation using `BorregasAve` map and `127.0.0.1:9090` as ROS connector
5. In simulator expan sensors panel and click "eye" icon against Ground Truth Visualizer sensor
