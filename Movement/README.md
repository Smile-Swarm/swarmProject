First create a controller package: "catkin_create_pkg controller rospy" .

Then place python scripts in controller's "src" folder.

Change permissions: "chmod +x script.py" .

Then run "rosrun controller script.py" in shell.


PID: Pre-define a coordinate to move to in python script. Robot's movement is controlled by the PID.

simplemove: Pre-define a coordinate to move to in python script. Robot stops and rotates before moving forward towards goal.

AStar: Uses a Pre-defined map (empty_wall.launch in ROSDS) that must be changed for different environments. Finds the shortest path. (does not actually move the robot)

move_astar: Uses a Pre-defined map (empty_wall.launch in ROSDS) that must be changed for different environments. Implements astar algorithm with movement.

DAstar: Uses a Pre-defined map (empty_wall.launch in ROSDS) that must be changed for different environments. First calculates shortest path, then moves the robot (movement is a combo of simplemove and PID). Collision causes rerouting of new shortest path.
