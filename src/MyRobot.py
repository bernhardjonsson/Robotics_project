import os
import numpy as np

import os
import sys
import inspect
import time

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
        self.joint_pos = [0, 0, 0, 0]

        self.use_smooth_speed_flag = 0                  # Flag for using smooth speed
        self.rbt = 0                                    # RigidBodyTree
        self.joint_limits = [0, 300, 30, 270, 10, 270, 135, 270] #Joint Limits in degree
        self.ik = 0                                     # Inverse Kinematics Object
        self.ik_weights = [0.25, 0.25, 0.25, 1, 1, 1]        # Weights for inverse kinematics
        self.joint_offsets = [150, 60, 150, 240]     # Joint offsets to send to motor
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
        self.move_j(0,0,0,0)
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

        for i in range(0,len(self.motor_ids)):
            if speeds[i] > 0 and speeds[i] <= 1:
                speed = int(speeds[i]*100)
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[i], 32, speed)

                if dxl_comm_result != self.COMM_SUCCESS:
                    print(self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Speeds successfully changed for joint " + str(self.motor_ids[i]))
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
        for i in range(0,len(self.motor_ids)):
            if torques[i] > 0 and torques[i] <= 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_ids[i], 34, torques[i]*1023)

                if dxl_comm_result != self.COMM_SUCCESS:
                    print(self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(self.packetHandler.getRxPacketError(dxl_error))
                else:
                    print("Motor torque succesfully changed for joint " + str(self.motor_ids[i]))
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
        elif motor_id==self.motor_ids[2]:
            error_str = "Angle Limits for second Axis Reached, got " + str(deg)
            assert deg > 30 or deg < 270, error_str
        else:
            error_str = "Angle Limits Reached, got " + str(deg)
            assert deg > 125 or deg < 300 , error_str
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
            if max(self.joint_angle_error) < 2.5:
                break
            #print(self.joint_pos)
        print("At Goal!\nCurrent pos; 1: " + str(self.joint_pos[0]) + ", 2: " + str(self.joint_pos[1]) + ", 3: " + str(self.joint_pos[2]) + ", 4: " + str(self.joint_pos[3]))

    def  read_joint_angles(self):
        #read_joint_angles function for the MyRobot Class.
        #   reads joint angles of all motor IDs
        #
        #Inputs:
        #   None
        #Outputs:
        #   j_a : a vector containing joint angles [deg]
        j_a = [0,0,0,0]
        for i in range(0,len(self.motor_ids)):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_ids[i], 36)

            if dxl_comm_result != self.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
            elif dxl_error != 0:
                raise Exception(self.packetHandler.getRxPacketError(dxl_error))
            else:
                j_a[i] = self.rot_to_deg(dxl_present_position) - self.joint_offsets[i]
                self.joint_pos[i] = j_a[i]
                self.joint_angle_error[i] = abs(j_a[i]-self.joint_angles[i])

        return j_a

if __name__ == "__main__":
    robot = MyRobot(0.2, 'COM4', 1000000)
    time.sleep(1)
    q1 = [0.785398163397448,	0.782859700178104,	0.775245229424790,
    	0.762563057934510,	0.744845701771392,	0.722178281396460,
    	0.694738276196703,	0.662844822335062,	0.627012769781415,
        	0.588002603547568,	0.546853258599602,	0.504883106683238,	0.463647609000806,
        	0.424851402989201,	0.390225480757561,	0.361390800374622,	0.339732374460581,
        	0.326301283451130,	0.321750554396642,	0.326301283451130,	0.339732374460581,
            	0.361390800374622,	0.390225480757561,	0.424851402989201,	0.463647609000806,
                	0.504883106683238,	0.546853258599602,	0.588002603547567,	0.627012769781414,	0.662844822335062,	0.694738276196703,	0.722178281396460,	0.744845701771392,	0.762563057934510,	0.775245229424790,	0.782859700178104,	0.785398163397448]
    q2 = [0.827932523851085,	-0.914383143187339,	0.182783856970528,
    	1.03315441902074,	1.71960564774657,	-0.782746439039840,
        -0.0280185209855236,	1.06652809089144,	-0.355140794249573,	-0.870729768768192,	-0.355140794249573,	1.06652809089142,	-0.0280185209855236,	-0.782746439039840,	1.71960564774657,	1.03315441902074,	0.182783856970528,	-0.914383143187339,	0.827932523851085,	-0.903540506333484,	0.168809594507393,	0.921397222236792,	1.39347273833791,	1.64123903488269,	-1.41014658166026,	-1.40876553348348,	-1.43582917870912,	-1.44979096017814,	-1.43582917870912,	-1.40876553348348,	-1.41014658166026,1.64123903488269,	1.39347273833791,	0.921397222236792,
        	0.168809594507393,	-0.903540506333484,	0.827932523851085]
    q3 =[63.5604229014964,60.0129070558653,55.8735482356189,51.2509522949370,46.3091903468590,41.2803460930672,36.4887999985645,32.3824758136985,29.5319907453210,28.4984617912664,29.5319907453210,32.3824758136984,36.4887999985645,41.2803460930672,46.3091903468590,51.2509522949370,55.8735482356189,60.0129070558653,63.5604229014964,66.4581611915136,68.6970707451787,70.3143317146558,71.3872658159858,72.0227685196403,72.3427287729831,72.4670605264257,72.4966377695115,72.4986417657988,72.4966377695115,72.4670605264257,72.3427287729831,72.0227685196403,71.3872658159858,70.3143317146558,68.6970707451787,66.4581611915136,63.5604229014964]
    q4 = [-62.8175590985526,-57.5277275858830,-54.4855357657945,-50.7133103871628,-46.4579996678107,-38.9268033272324,-34.8899851507841,-31.8782075777950,-27.6060536242765,-26.0569356957033,-27.6060536242765,-31.8782075777949,-34.8899851507841,-38.9268033272324,-46.4579996678107,-50.7133103871628,-54.4855357657945,-57.5277275858830,-62.8175590985526,-63.9838243583852,-67.2950840128912,-69.6649326100977,-71.2099422275288,-72.0932112277281,-69.3617858645279,-69.4874986661473,-69.4900122640075,-69.4780544788257,-69.4900122640075,-69.4874986661473,-69.3617858645279,-72.0932112277281,-71.2099422275288,-69.6649326100977,-67.2950840128912,-63.9838243583852,-62.8175590985526]
    for i in range(len(q1)):
        robot.move_j(q1[i],q2[i],q3[i],q4[i])
