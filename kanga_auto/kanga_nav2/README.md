# kanga_nav2

Basic Nav2 bringup for Kanga simulation using:
- Gazebo odometry on `/odom`
- Identity static transform `map -> odom`
- A static occupancy map with built-in keep-out zones

## Run

1. Start Gazebo and spawn robot (your existing flow).
2. Launch Nav2:

```bash
ros2 launch kanga_nav2 nav2_sim.launch.py
```

3. Send waypoint route:

```bash
ros2 run kanga_nav2 send_waypoints
```

## Notes

- No localization node is used in this setup.
- Edit keep-out regions by modifying `maps/kanga_static_map.pgm`.
- If your robot footprint is larger/smaller, tune `robot_radius` in `config/nav2_params.yaml`.
