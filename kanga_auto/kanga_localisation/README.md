# Odometry and Localisation stack

### Handles
- Localisation of the rover from initial position
- Various options available
    - [Point Lio](https://github.com/dfloreaa/point_lio_ros2/tree/main/launch) for Unitree lidar l2  (CPU Based)
    - Onboard Zed2i VSLAM  (GPU based)
    - Possible sensor fusion however likely unneccesary for amount of addition processing power required
    - May not be required at all but can compliment RTABMAP as input odometry data