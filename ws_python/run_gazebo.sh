ros2 launch jackal_gazebo jackal_world.launch.py &
ros2 launch fast_lio mapping.launch.py config:=mid360.yaml &
# python3 static_odom_to_world.py &
python3 static_camera_init_to_world.py &
ros2 launch jackal_viz view_model.launch.py