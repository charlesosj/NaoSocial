# NaoSocial
Installation
For compatibility purposes we recommend installing Ubuntu version 14.04 to ensure everything runs smoothly.
ROS and Nao SDK`s
1.	Install ROS Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu). Follow steps 1.0 to 1.7
2.	Configure your ROS Environment by creating a catkin workspace (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Staring from tutorial number 3 select ‘catkin’ and follow the steps to make your work space
3.	Follow the guide to installing the NAOQI SDK and setting up your python bindings from 1.2 to 1.3 (http://wiki.ros.org/nao/Tutorials/Installation#ROS). Make sure you change the name, version and path of your downloaded SDK to match all commands that need to be executed. You will need to download both the python and c++ sdk

Dependencies
1.	Open the src folder of your catkin workspace you created in the first part in a terminal using cd ~/catkin_ws/src
2.	Download the required dependencies into your src
•	Naoqi Bridge git clone https://github.com/ros-naoqi/naoqi_bridge.git
•	Driver git clone https://github.com/ros-naoqi/naoqi_driver.git
•	Rviz Meshes sudo apt-get install ros-indigo-nao-meshes
