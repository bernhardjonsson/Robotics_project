# Robotics_project

This is a project repository for the final project option 2 in the Robotics course at DTU.
The Matlab files hold the theoretical calculations (RoboticsAssignment), whereas the python files show the physical 
manipulation of the provided robot (main.py).

## Installation and setup

The project uses the Dynamixel SDK library for robot control and OpenCV for image recognition.

###### Dynamixel SDK
A detailed instruction of the setup can be found [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/python_windows/).
The library can be downloaded from git:

```bash
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

and setup by running the following command

```bash
python .\DynamixelSDK\python\setup.py install
```

###### OpenCV
OpenCV can be setup up using following
```bash
pip install opencv-python
```


