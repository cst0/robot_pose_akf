# robot_pose_akf
Address some of the robot_pose_ekf TODO's by making Another Kalman Filtering package for robot pose estimation

Still in alpha, but aims to provide the following features already covered by the robot_pose_ekf package:
- Sensor fusion of odometry/IMU
- (Extended) Kalman filtering of robot pose/velocity data
- Robot pose estimation via the above.

However, the robot_pose_ekf package outlines a handful of areas for improvement which this package attempts to address:
- Ability to take in arbitrary number of sensors
- Leveraging usage of the control inputs and state model for more effective filtering

Additionally, the robot_pose_ekf package assumes a model which is two dimensional (plus rotation). This is often an
acceptable simplification, as many of the robots performing localization are operating on flat ground. However,
many robots may operate three-dimensionally. Additionally, even robots which are constrained to the ground may go
up hills, stairs, etc. For this reason, a 7-DoF model (position + quaternion) may be prefereable, and this is what
I aim to implement.
