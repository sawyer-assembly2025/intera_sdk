
"""
Created on Wed Nov 19 17:23:10 2025

@author: Hector Quijada

Class Sawyer robot that integrates functionality for gripper handling and linear movements in space given a 
cartesian position or angle position


"""

import rospy
import numpy as np
import os
from intera_interface import (
    Gripper,
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff,
    Limb,
    RobotParams,
)
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SawyerRobot():
    
    #Initialize gripper and limb
    """
    #ToDo
    #Check tip_name = "right_gripper_tip" as we need to move in the current EOAT TCP
    #How to make "right_gripper_tip" be recognized? does it have to do with the URDF loaded?
    """
    def __init__(self, max_linear_speed = 0.01, max_linear_accel = 0.01, max_rotational_speed = 1.57, max_rotational_accel = 1.57, tip_name = "right_hand", timeout=None):
        
        try:
            #Obtain limb reference
            self._limb = Limb()
            
        except:
            rospy.logerr("Limb reference not found, check connection to robot")
                    
        #Set movement parameters
        self.max_linear_speed = max_linear_speed
        self.max_linear_accel = max_linear_accel
        self.max_rotational_speed = max_rotational_speed
        self.max_rotational_accel = max_rotational_accel
        self.tip_name = tip_name
        
        #Positions saved for later movement
        self.positions = self._read_positions()
        
        #Trajectory configuration
        self._traj_options = TrajectoryOptions()
        self._traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        self.traj = MotionTrajectory(trajectory_options = self._traj_options, limb = self._limb)
        
        #Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout
        self.timeout = timeout

        wpt_opts = MotionWaypointOptions(max_linear_speed = self.max_linear_speed,
                                         max_linear_accel = self.max_linear_accel,
                                         max_rotational_speed = self.max_rotational_speed,
                                         max_rotational_accel = self.max_rotational_accel,
                                         max_joint_speed_ratio=1.0)
        
        self.waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self._limb)
        
        #Get joint names
        self.joint_names = self._limb.joint_names()
        
        #EOAT configuration
        
        #Important Notes:
        #Gripper must be first configured in Intera Studio with robot in manufacturing mode, then pass to Intera SDK mode
        #Mass, CoM, and Endpoint position are very important
        #To connecto to Intera Studio use Robot IP address in a browser while connected to the robot
        #While in Intera SDK mode, check for topic /robot/io_end_effector/config/"here goes gripper ID config file"
        
        try:
            rp = RobotParams()
            valid_limbs = rp.get_limb_names()
            
            #Get current EOAT reference
            self._cuff = Cuff(limb=valid_limbs[0])
            self._gripper = get_current_gripper_interface()
            
            #Flag for ClickSmartGripper 
            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)
            
            #If ClickSmartGripper initialize if needed
            if self._is_clicksmart:
                if self._gripper.needs_init():
                    self._gripper.initialize()
                msg = "Smart Click Connection found and initialized, ready for use"
                rospy.logwarn(msg)
            else:
                #If electric gripper not calibrated, raise exception, and calibrate, then create instance of SawyerRobot
                if not (self._gripper.is_calibrated() or self._gripper.calibrate() == True):
                    raise
        except:
            
            try:
                self._gripper = Gripper()
                self._is_clicksmart = False
                msg = "Smart Click Connection not found, connecting to class Gripper(), for electric gripper"
                rospy.logwarn(msg)
            except:
                self._gripper = None
                self._is_clicksmart = False
                msg = "No gripper connection found, check EOAT if gripper connected but not detected"
                rospy.logwarn(msg)
            
    #Positions for movement convenience (CARTESIAN):
    def _read_positions(self, positions_path="sawyer_pos.csv"):
        
        if os.path.exists(positions_path):
        
            pos = np.zeros(6)
            pos_aux = np.zeros(6)
            
            #Extract all Inputs from train file as a single numpy array for ARTa and ARTb
            with open(positions_path, 'r') as file:
                
                # Read all lines and process each one
                lines = file.readlines()
                for line in lines:
                    pos_str = line.strip().split(',') #Extract EOL and split in a list with separator ","
                    
                    #Split into two vectors and convert to floar
                    for i in range(len(pos_str)):
                        pos_aux[i] = float(pos_str[i])
                    
                    #Stack current line vector into a matrix
                    pos = np.vstack((pos, pos_aux))
                    #Delete first row as it was auxiliary
                
                pos = np.delete(pos, 0, axis=0)
                
        else:
            msg = "No file named sawyer_pos.csv found, create file with new positions"
            rospy.logwarn(msg)
            pos = None
                
        return pos
    
    #Save Position in a current
    def save_position(self):
        
        position, euler_orientation = self.current_endpoint_pose()
        
        save_pose = np.zeros((1,6))
        
        save_pose[0][0] = position[0]
        save_pose[0][1] = position[1]
        save_pose[0][2] = position[2]
        save_pose[0][3] = euler_orientation[0]
        save_pose[0][4] = euler_orientation[1]
        save_pose[0][5] = euler_orientation[2]
        
        if len(self.positions) > 0:
            self.positions = np.vstack((self.positions, save_pose))
        else:
            self.positions = save_pose
        
        positions_path = "sawyer_pos.csv"
        np.savetxt(positions_path, self.positions, delimiter=',', fmt='%f')
        rospy.loginfo('New position saved')
        

    #Give current endpoint pose with orientation in euler angles
    def current_endpoint_pose(self, quaternion=False):
        
        """
        #To Do
        Test endpoint pose corresponds to current gripper setup done in Intera Studio
        else use endpoint_state = limb.tip_state(self.tip_name) which returns PoseStamped msg
        """
        #Returns a dictionary
        #pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        endpoint_pose = self._limb.endpoint_pose()
        
        #(x,y,z) Cartesian Position 
        endpoint_position = endpoint_pose['position']
        
        endpoint_position = list(endpoint_position)
        
        #(x,y,z,w) Cuaternion Orientation 
        endpoint_orientation_quaternion = endpoint_pose['orientation']
        
        #Transform to roll pitch yaw for movement
        roll, pitch, yaw = euler_from_quaternion(endpoint_orientation_quaternion, 'sxyz')
        
        endpoint_orientation_euler = [roll, pitch, yaw]
        
        if not quaternion:
            #Return as list so values can be edited with euler angles
            return endpoint_position, endpoint_orientation_euler
        else:
            #Return as list so values can be edited with quaternions orientation
            endpoint_orientation_quaternion = list(endpoint_orientation_quaternion)
            return endpoint_position, endpoint_orientation_quaternion
    
    #Open EOAT Gripper
    def open_gripper(self):
        if self._gripper.is_ready():
            rospy.loginfo("Gripper open triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
            else:
                self._gripper.open()
    
    #Close EOAT Gripper
    def close_gripper(self):
        if self._gripper.is_ready():
            rospy.loginfo("Gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', True)
            else:
                self._gripper.close()
    
    #Manual movement of the joint positions in radians as a list
    def move_to_joint_positions(self, positions, rotational_speed, rotational_accel):
        
        #Reinitialize trajectory so that only one waypoint is executed
        self.traj = MotionTrajectory(trajectory_options = self._traj_options, limb = self._limb)

        #Set waypoint
        self.waypoint.set_joint_angles(positions, self.tip_name, self.joint_names)
        
        #Send movement
        self.traj.append_waypoint(self.waypoint.to_msg())

        #Check trajectory result
        result = self.traj.send_trajectory(timeout=self.timeout)
        
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    
    #Moves to desired absolute position using linear interpolation (Orientation in Euler angles ori = [roll, pitch, yaw])
    #Pos no = 0, moves to the position at position and orientation arguments, else to the saved position
    def move_to_cartesian_absolute(self, position = [0.4, 0.0, 0.28], orientation = [-3.14, 0.0, -3.14], pos_no=0):
        
        #Reinitialize trajectory so that only one waypoint is executed
        self.traj = MotionTrajectory(trajectory_options = self._traj_options, limb = self._limb)

        #Position number 0 means to move to a desired position and orientation else move to position saved
        if pos_no == 0:
            move_pose = PoseStamped()
            #Get position from argument
            move_pose.pose.position.x = position[0]
            move_pose.pose.position.y = position[1]
            move_pose.pose.position.z = position[2]
            roll = orientation[0]
            pitch = orientation[1]
            yaw = orientation[2]
            rospy.loginfo("Moving to predetermined position: %s, %s",position,orientation)
        else:
            try:
                move_pose = PoseStamped()
                
                move_pose.pose.position.x = self.positions[pos_no-1][0]
                move_pose.pose.position.y = self.positions[pos_no-1][1]
                move_pose.pose.position.z = self.positions[pos_no-1][2]
                roll = self.positions[pos_no-1][3]
                pitch = self.positions[pos_no-1][4]
                yaw = self.positions[pos_no-1][5]
                rospy.loginfo("Moving to position index: %s", pos_no)
                
            except:
                rospy.logerr("Position index not found in positions list")
        
        #Transform orientation back to quaternion as movement command need this orientation

        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]

        quaternion = quaternion_from_euler(roll, pitch, yaw,'sxyz')
        move_pose.pose.orientation.x = float(quaternion[0])
        move_pose.pose.orientation.y = float(quaternion[1])
        move_pose.pose.orientation.z = float(quaternion[2])
        move_pose.pose.orientation.w = float(quaternion[3])
        
        #Move to desired position
        joint_angles = self._limb.joint_ordered_angles()
        self.waypoint.set_cartesian_pose(move_pose, self.tip_name, joint_angles)
        
        rospy.loginfo('Sending waypoint: \n%s', self.waypoint.to_string())

        self.traj.append_waypoint(self.waypoint.to_msg())

        #Check trajectory result
        result = self.traj.send_trajectory(timeout=self.timeout)
        
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
            
    #Moves to desired relative position using linear interpolation (Orientation in Euler angles ori = [roll, pitch, yaw])
    def move_to_cartesian_relative(self, position, orientation):
        
        #Reinitialize trajectory so that only one waypoint is executed
        self.traj = MotionTrajectory(trajectory_options = self._traj_options, limb = self._limb)

        #Get current endpoint pose
        current_position, current_orientation = self.current_endpoint_pose()
        
        move_pose = PoseStamped()
        
        #Add movement to current position
        move_pose.pose.position.x = current_position[0] + position[0]
        move_pose.pose.position.y = current_position[1] + position[1]
        move_pose.pose.position.z = current_position[2] + position[2]
        
        #Add orientation movement to current orientation in euler angles and transform to quaternion
        
        move_euler = [0,0,0]
        move_euler[0] = current_orientation[0] + orientation[0]
        move_euler[1] = current_orientation[1] + orientation[1] 
        move_euler[2] = current_orientation[2] + orientation[2] 
        
        roll = move_euler[0]
        pitch = move_euler[1]
        yaw = move_euler[2]

        move_quaternion = quaternion_from_euler(roll, pitch, yaw,'sxyz')
        
        move_pose.pose.orientation.x = float(move_quaternion[0])
        move_pose.pose.orientation.y = float(move_quaternion[1])
        move_pose.pose.orientation.z = float(move_quaternion[2])
        move_pose.pose.orientation.w = float(move_quaternion[3])
        
        #Move to desired position
        joint_angles = self._limb.joint_ordered_angles()
        self.waypoint.set_cartesian_pose(move_pose, self.tip_name, joint_angles)
        
        rospy.loginfo('Sending waypoint: \n%s', self.waypoint.to_string())

        self.traj.append_waypoint(self.waypoint.to_msg())

        #Check trajectory result
        result = self.traj.send_trajectory(timeout=self.timeout)
        
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
        
        
if __name__ == '__main__':
    
    sawyer = SawyerRobot()
    
    print(sawyer.current_endpoint_pose())
    
        
        
        
        
        
        
        
    