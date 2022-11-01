import os
import numpy as np

import os
import sys
import inspect

import dynamixel_sdk as dxl

class MyRobot():
    def __init__(self, speed, USB_port, Baudrate, Protocol = 1.0, torque_lim = 1):
        # Definition of communication settings
        self.PROTOCOL_VERSION            = Protocol         # See which protocol version is used in the Dynamixel
        self.BAUDRATE                    = Baudrate      # Baudrate for Motors
        self.DEVICENAME                  = USB_port       # Check which port is being used on your controller

        # Definition of the AX12-A parameters
        self.ADDR_MX_TORQUE_ENABLE       = 24           # Control table address for enabling torque mode
        self.ADDR_MX_GOAL_POSITION       = 30           # Control table address for reading goal position
        self.ADDR_MX_PRESENT_POSITION    = 36           # Control table address for reading current position
        self.TORQUE_ENABLE               = 1            # Value for enabling the torque
        self.TORQUE_DISABLE              = 0            # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 10           # Dynamixel moving status threshold
        self.COMM_SUCCESS                = 0            # Communication Success result value
        self.COMM_TX_FAIL                = -1001        # Communication Tx Failed

        # Setup of the robot
        self.motor_ids = [1, 2, 3, 4]                      # Motor IDs chronologically (see Dynamixel Wizard for more info)
        self.dh = [[0,-np.pi/2,	0.0955, 0],              # Denavit Hartenberg Parameters for Robot (a, alpha, d, theta)
            [0.116,	0,0,0],
            [0.096,0,0,0],
            [0.064,0,0,0]]
        self.forward_transform = np.zeros((4,4))             # Forward transformation Matrix
        self.joint_angles = [0, 0, 0, 0]                   # Internal joint angles in degree
        self.joint_pos = np.zeros((4,4))                     # Internal joint positions calculated with each move_j

        self.use_smooth_speed_flag = 0                  # Flag for using smooth speed
        self.rbt = 0                                    # RigidBodyTree
        self.joint_limits = [0, 300, 30, 270, 10, 270, 10, 270] #Joint Limits in degree
        self.ik = 0                                     # Inverse Kinematics Object
        self.ik_weights = [0.25, 0.25, 0.25, 1, 1, 1]        # Weights for inverse kinematics
        self.joint_offsets = [0, 0, 0, 0]     # Joint offsets to send to motor
        self.joint_angle_error = [0, 0, 0, 0]              # Internal joint angle error between read out of joint angles and input joint angles
        self.init_status = 0                            # Initialization succesfull flag
        self.movement_history = []                      # List to record movement history
        self.motor_speed = 0                            # List for motor speed
        self.motor_torque = 0                           # List for motor torque
        self.pitch = 0                                  # Pitch Angle for motor 3

        # Establish connection to robot
        self.portHandler = dxl.PortHandler(self.DEVICENAME)
        self.packetHandler = dxl.PacketHandler(self.PROTOCOL_VERSION)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            raise Execption("Failed to open the port")

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            raise Execption("Failed to change the baudrate")

        # Move to initial position
        self.set_speed([speed,speed,speed,speed],True)
        self.set_torque_limit([torque_lim,torque_lim,torque_lim,torque_lim])
        self.move_j(150,150,150,150)
        self.init_status = 1


    def smooth_speed(self,joint_angles):
        #smooth_speed function for the MyRobot Class.
        #   Dynamically changes the speed of each joint to create
        #   smoother motion. It assures all joint movements finish at
        #   the same time
        #
        #Inputs:
        #   joint_angles : a vector representing joint angles [deg]
        #Outputs:
        #   None
        max_angle = max(abs(joint_angles))
        speed_per_deg = max_angle/100
        if speed_per_deg!=0:
            new_speeds = abs(joint_angles/speed_per_deg)*0.01
            for i in range(0,len(self.motor_speed)-1):
                if new_speeds[i]==0:
                    new_speeds[i]=self.motor_speed[i]
                else:
                    new_speeds[i]=new_speeds[i]*self.motor_speed[i]
            self.set_speed(new_speeds,false)


    def set_speed(self, speeds, overwrite_speeds):
        #set_speed function for the MyRobot Class.
        #   Sets individual motor speeds between 0# and 100#
        #
        #Inputs:
        #   speeds : a vector representing motor speeds for each motor
        #   ID between 0 and 1
        #   overwrite_speeds: boolean, if true class internal motor
        #   speeds are overwritten to motor speeds of function call
        #Outputs:
        #   None
        if overwrite_speeds:
            self.motor_speed = speeds

        for i in range(0,len(self.motor_ids)-1):
            if speeds[i] > 0 and speeds[i] <= 1:
                speed = int(speeds[i]*100)
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[i], 32, speed)

                if dxl_comm_result != self.COMM_SUCCESS:
                    print(self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Speeds successfully changed for joint " + str(self.motor_id[i]))
            else:
                print("\nMovement speed out of range, enter value between ]0,1]")



    def set_torque_limit(self, torques):
        #set_torque_limit function for the MyRobot Class.
        #   Sets individual motor torques between 0# and 100#
        #
        #Inputs:
        #   speeds : a vector representing motor torque for each motor
        #   ID between 0 and 1
        #Outputs:
        #   None

        self.motor_torque = torques
        for i in range(0,len(self.motor_ids)-1):
            if torques[i] > 0 and torques[i] <= 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[i], 34, torques[i]*1023)

                if dxl_comm_result != self.COMM_SUCCESS:
                    print(self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Motor torque succesfully changed for joint " + str(self.motor_id[i]))
            else:
               print("\nTorque limit out of range, enter value between ]0,1]")


    def check_limits(self,deg, motor_id):
        #check_limits function for the MyRobot Class.
        #   Checks if joint angle is within motor limits, depending on
        #   the motor, see https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
        #
        #Inputs:
        #   deg : value for joint angle [deg]
        #   motor_id : int of motors ID
        #Outputs:
        #   deg : returns input value if checks pass [deg]
        if motor_id==self.motor_ids[0]:
            error_str = "Angle Limits for first Axis Reached, got " + str(deg)
            assert deg > 0 or deg < 300, error_str
        elif motor_id==self.motor_ids[1]:
            error_str = "Angle Limits for second Axis Reached, got " + str(deg)
            assert deg > 30 or deg < 270, error_str
        else:
            error_str = "Angle Limits Reached, got " + str(deg)
            assert deg > 10 or deg < 290 , error_str
        return deg


    def  deg_to_rot(self,deg):
        #deg_to_rot function for the MyRobot Class.
        #   Converts degree to units per rotation of motors
        #
        #Inputs:
        #   deg : value [deg]
        #Outputs:
        #   rot : value in units per rotation of motor
        rot = deg*1/0.29
        return int(rot)

    def  rot_to_deg(self,rot):
        #rot_to_deg function for the MyRobot Class.
        #   Convers units per rotation of motors to degree
        #
        #Inputs:
        #   rot : value in units per rotation of motor
        #Outputs:
        #   deg : value [deg]
        deg = rot*0.29
        return deg

    def move_j(self,j1,j2,j3,j4):
        #move_j function for the MyRobot Class.
        #   Moves the robot arm to the desired joint angles, checks
        #   joint limits, updates internal robot state and waits until
        #   the joint angle error between desired and mesured joint
        #   angle is below 2°
        #
        #Inputs:
        #   j1 : value for joint one [deg]
        #   j2 : value for joint two [deg]
        #   j3 : value for joint three [deg]
        #   j4 : value for joint four [deg]

        #Outputs:
        #   None
        print("Goal Posistion:\n1: " + str(j1) + ", 2:" + str(j2) + ", 3:" + str(j3) + ", 4:" + str(j4))

        j1 = self.check_limits(j1, self.motor_ids[0])
        j2 = self.check_limits(j2, self.motor_ids[0])
        j3 = self.check_limits(j3, self.motor_ids[0])
        j4 = self.check_limits(j4, self.motor_ids[0])

        if self.use_smooth_speed_flag:
            self.smooth_speed([j1, j2, j3, j4]-self.joint_angles)

        self.joint_angles = [j1, j2, j3, j4]
        #self.forward([j1 j2 j3 j4])
        #if self.draw_robot_flag
        #    self.draw_robot()

        j1 = j1 + self.joint_offsets[0]
        j2 = j2 + self.joint_offsets[1]
        j3 = j3 + self.joint_offsets[2]
        j4 = j4 + self.joint_offsets[3]

        self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[0], self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j1))
        self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[1], self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j2))
        self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[2], self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j3))
        self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[3], self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j4))

        while True:
            self.read_joint_angles()
            if sum(self.joint_angle_error)<2:
                break
            else:
                print(self.joint_angles)
        print("At Goal")

    def  read_joint_angles(self):
        #read_joint_angles function for the MyRobot Class.
        #   reads joint angles of all motor IDs
        #
        #Inputs:
        #   None
        #Outputs:
        #   j_a : a vector containing joint angles [deg]
        j_a = [0,0,0,0]
        for i in range(0,len(self.motor_ids)-1):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_ids[i], 36)

            if dxl_comm_result != self.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
            elif dxl_error != 0:
                print(self.packetHandler.getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
            else:
                j_a[i] = self.rot_to_deg(dxl_present_position) - self.joint_offsets[i]
                self.joint_angle_error[i] = j_a[i]-self.joint_angles[i]

        return j_a

if __name__ == "__main__":
    robot = MyRobot(0.5, 'COM4', 1000000)
