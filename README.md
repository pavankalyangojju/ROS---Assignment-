# overview
Create a Gazebo world (or reuse a TurtleBot3 world) and a 2D map file (image + YAML) that represent the environment. Put a few known points (x,y,yaw) in it (we call them waypoints: Station A/B/C/D, Docking station, Home).
Launch Gazebo with TurtleBot3, and start the standard Nav2 stack (map server, AMCL, Nav2 nav2_bringup components) so the robot has localization and planning. RViz is started for visualization.
Start a custom ROS 2 node waypoint_manager_node:
It knows the coordinates of the named waypoints (from YAML).
It publishes visualization markers in RViz for those waypoints.
It listens to commands from the GUI and sends goals to Nav2 using the NavigateToPose action.
It monitors feedback/result and handles failures/timeouts, and at the end returns robot to Home.
Start a small GUI (Tkinter):
Shows buttons for the named waypoints.
Allows selecting one or more waypoints (buttons toggle).
When user clicks “Go”, GUI publishes the chosen waypoint sequence to the waypoint manager node.
GUI displays status messages (Navigating…, Reached, Failed) by subscribing to a status topic.
Has a “Stop/Cancel” button to cancel current navigation.
All components (Gazebo, Nav2, RViz, waypoint_manager_node, GUI) are launched together via one ROS 2 launch file.

# File list
your_package/
├─ launch/
│  └─ bringup_launch.py                # main launch file (starts gazebo, nav2 bringup, rviz, nodes)
├─ config/
│  ├─ waypoints.yaml                   # named waypoint coordinates
│  ├─ nav2_params.yaml                 # minimal Nav2 params (or include nav2 defaults)
│  └─ map.yaml + map.pgm               # your map for map_server (create with gmapping/Cartographer/handmade)
├─ rviz/
│  └─ nav2_waypoints.rviz
├─ src/
│  ├─ waypoint_manager_node.py         # ROS2 python node (action client + markers + statuspublisher)
│  └─ gui_tkinter.py                   # Tkinter GUI that talks to waypoint_manager_node
└─ package.xml / setup.py / CMakeLists.txt

# waypoint YAML
# config/waypoints.yaml
home:   {x: 0.0,  y: 0.0,  yaw: 0.0}
station_a: {x: 1.5,  y: 0.5,  yaw: 0.0}
station_b: {x: -1.2, y: 0.8,  yaw: 1.57}
station_c: {x: -0.5, y: -1.4, yaw: -1.57}
station_d: {x: 2.2,  y: -0.7, yaw: 3.14}
docking_station: {x: 0.0, y: 2.0, yaw: 0.0}

# waypoint_manager_node.py — action client + markers + status publisher
Save as src/waypoint_manager_node.py. This node:
Loads config/waypoints.yaml
Publishes RViz visualization markers for each waypoint
Subscribes to /waypoint_sequence (std_msgs/String) where the GUI publishes names separated with commas (e.g. "station_a,station_b")
Subscribes to /cancel_nav (std_msgs/Bool) to cancel current goal
Publishes /waypoint_status (std_msgs/String) for GUI status updates

code :- https://github.com/pavankalyangojju/ROS---Assignment-/blob/main/waypoint_manager_node.py

