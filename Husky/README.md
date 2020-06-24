Clone above packages into your workspace:

"cd ~/catkin_ws/src"

"git clone https://github.com/husky/husky.git"

"git clone https://github.com/bsb808/nre_simmultihusky.git"

"git clone https://github.com/clearpathrobotics/cpr_multimaster_tools.git"

"cd ~/catkin_ws"

"source devel/setup.bash"

Fix husky.urdf.xacro file:
In IDE find catkin_ws/src/husky/husky_description/urdf/husky.urdf.xacro and delete the last optional argument line

For one husky: "roslaunch husky_gazebo husky_playpen.launch"

For multiple huskies: "roslaunch husky_gazebo multi_husky_playpen.launch"
