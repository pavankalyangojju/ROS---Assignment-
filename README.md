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
│  └─ bringup_launch.py
├─ config/
│  ├─ waypoints.yaml                   
│  ├─ nav2_params.yaml                 
│  └─ map.yaml + map.pgm               
├─ rviz/
│  └─ nav2_waypoints.rviz
├─ src/
│  ├─ waypoint_manager_node.py         
│  └─ gui_tkinter.py                   
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

# Simple GUI (Tkinter)
This Tkinter app:
Shows toggle buttons for all waypoint names (hardcoded to same names as YAML — or you can load the YAML similarly),
When user clicks Go, it sends the list as a comma-separated String on waypoint_sequence.
Subscribes to waypoint_status to update an on-screen label.
Stop publishes True on cancel_nav.

Code :- https://github.com/pavankalyangojju/ROS---Assignment-/blob/main/gui_tkinter.py

# Launch file
A single launch that includes:
--> Gazebo world (TurtleBot3) — you can reuse turtlebot3_gazebo launch,
--> Nav2 bringup (map server, AMCL, nav2 bringup) — include existing nav2 launch files or minimal ones,
--> Our two nodes (waypoint_manager_node and GUI),
--> RViz.
Below is a simplified example that assumes the user has installed standard TurtleBot3 + Nav2 packages. You will likely need to adapt paths to your installation and the map file. This launch includes our package nodes.

Code :- 
