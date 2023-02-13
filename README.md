# Joint-Space-PID-Control
PID controller for joint space tracking of the 3-DOF revolute-revolute-prismatic (RRP) robot manipulator in Gazebo

Video for gazebo simulation result: https://youtu.be/MB3ovGn2Q-A

Documentation: https://github.com/RaviTeja163/Joint-Space-PID-Control/blob/main/Report.pdf

The objective of this project is to develop a joint space position control for a 3-DOF revolute-revolute-prismatic (RRP) robot manipulator. The task is to implement a PID controller for joint space tracking of the robot in Gazebo. The control approach is considered a position-based control method, that is, the desired set-points provided to the controller are (xd, yd, zd) coordinates of the end-effector. The program first calculates the corresponding desired joint configurations using the inverse kinematics of the robot, and then use a PID controller to move each joint to the desired configuration and hence reaching the desired position in the task space. The robot joints are controlled using an independent joint control framework, that is, each joint of the robot is controlled via a separate PID controller.

There are 2 nodes:
1. An inverse kinematics node as a service server that takes a (desired) pose of the end effector and returns joint configurations as the response.
2. A position controller node

The robot stops for 1 second at each desired configuration, and then moves to the next point. The robot stops at the final position after completing the task, under 1 minute, with a smooth with no overshoot or oscillations.

There are 
• Two python scripts: rrp_ik_server.py and rrp_pid.py.
• A service file named rrpIK.srv which contains the end-effector position as the request fields, and the three joint angles as the response fileds.
• A launch file named rrp.launch which first launches the inverse kinematics node, and then loads the position controller node.
