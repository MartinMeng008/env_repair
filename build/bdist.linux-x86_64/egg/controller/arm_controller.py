#!/usr/bin/env python

import rospy
import numpy as np
import hello_helpers.hello_misc as hm
import threading
from sensor_msgs.msg import JointState

from env_repair.scripts.tools import (print_debug, are_array_close)

DEBUG = False

class ArmController(hm.HelloNode):
    """Given a goal position [x, y, z], this class will move the end effector of the arm to the goal."""
    def __init__(self):
        rospy.loginfo("Initializing Arm Controller")

        # Parameters
        self.frequency = 5.0
        self.gripper_moving_time = 3
        self.arm_moving_time = 7
        # self.stretch_base_width = 0.36
        self.wrist_extension_offset = 0.35
        self.lift_height_offset = 0
        self.min_wrist_extension = 0.01
        self.max_wrist_extension = 0.51
        self.min_lift_height = 0.1
        self.max_lift_height = 1.2
        self.default_joint_state = [0.01, 1.05, 0] # [wrist_extension, lift_position, wrist_yaw]

        # Set up the node
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.lift_position = None
        self.wrist_position = None
        self.wrist_yaw_position = None
        
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_arm_controller', wait_for_first_pointcloud=False)
        self.rate = rospy.Rate(self.frequency)
        self.move_lock = threading.Lock()
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback, queue_size=1)

    ## ==== Threading ==== ##
    def set_events(self, ready_event: threading.Event, shutdown_event: threading.Event, path_condition: threading.Condition) -> None:
        self.ready_event = ready_event
        self.shutdown_event = shutdown_event
        self.path_condition = path_condition

    def shutdown(self):
        """Shut down the control loop"""
        self.shutdown_events(self.ready_event, self.shutdown_event, self.path_condition)

    def shutdown_events(self, ready_event: threading.Event, shutdown_event: threading.Event, path_condition: threading.Condition):
        """Shut down the control loop"""
        shutdown_event.set()
        with path_condition:
            path_condition.notify_all()
        ready_event.clear()
        with path_condition:
            path_condition.notify_all()
        while self.is_event_alive(shutdown_event):
            self.rate.sleep()
    
    def is_event_alive(self, shutdown_event: threading.Event) -> bool:
        """Returns whether the controller is alive"""
        return not shutdown_event.is_set()

    ## ==== Callbacks ==== ##
    def joint_states_callback(self, joint_states: JointState) -> None:
        with self.joint_states_lock:
            self.joint_states = joint_states
        self.wrist_position, _, _ = hm.get_wrist_state(self.joint_states)
        self.lift_position, _, _ = hm.get_lift_state(self.joint_states)
        self.wrist_yaw_position, _, _ = self.get_wrist_yaw_state(self.joint_states)
        self.joint_state = np.array([self.wrist_position, self.lift_position, self.wrist_yaw_position]) # [wrist_extension, lift_position, wrist_yaw]
        if DEBUG:
            rospy.loginfo('=====================')
            rospy.loginfo(rospy.get_caller_id() + " Wrist position: " + str(self.wrist_position))
            rospy.loginfo(rospy.get_caller_id() + " Lift position: " + str(self.lift_position))
            rospy.loginfo(rospy.get_caller_id() + " Wrist yaw position: " + str(self.wrist_yaw_position))

    ## ==== Arm skills ==== ##
    def pickup(self, curr_pose: np.ndarray, target: np.ndarray) -> None:
        """Given the robot curr pose and a point [x y z] in glbal frame, reach the point and pick up the object.
        
        Args:
            curr_pose: 1-by-3 np.ndarray of pose values [x, y, theta] relative to the global frame
            target: 1-by-3 np.ndarray of point values [x, y, z] relative to the global frame
        """
        rospy.loginfo(rospy.get_caller_id() + " Picking up object at point: " + str(target))
        # if self.shutdown_event.is_set():
        #     return None
        # self.reach_default()
        if self.shutdown_event.is_set():
            return None
        self.open_gripper()
        if self.shutdown_event.is_set():
            return None
        self.reach_point(curr_pose, target)
        if self.shutdown_event.is_set():
            return None
        self.close_gripper()
        if self.shutdown_event.is_set():
            return None
        self.reach_default()
        # self.shutdown_event.set()
    
    def pick(self, curr_pose: np.ndarray, target: np.ndarray) -> None:
        """Given the robot curr pose and a point [x y z] in glbal frame, reach the point and pick up the object.
        
        Args:
            curr_pose: 1-by-3 np.ndarray of pose values [x, y, theta] relative to the global frame
            target: 1-by-3 np.ndarray of point values [x, y, z] relative to the global frame
        """
        rospy.loginfo(rospy.get_caller_id() + " Picking up object at point: " + str(target))
        if self.shutdown_event.is_set():
            return None
        self.open_gripper()
        if self.shutdown_event.is_set():
            return None
        self.reach_point(curr_pose, target)
        if self.shutdown_event.is_set():
            return None
        self.close_gripper()

    def dropoff(self, curr_pose: np.ndarray, target: np.ndarray) -> None:
        """Given the robot curr pose and a point [x y z] in glbal frame, reach the point and place the object.
        
        Args:
            curr_pose: 1-by-3 np.ndarray of pose values [x, y, theta] relative to the global frame
            target: 1-by-3 np.ndarray of point values [x, y, z] relative to the global frame
        """
        rospy.loginfo(rospy.get_caller_id() + " Placing object at point: " + str(target))
        self.reach_point(curr_pose, target)
        self.open_gripper()
        self.reach_default()

    def reach_default(self) -> None:
        """Reach the default point."""
        rospy.loginfo(rospy.get_caller_id() + " Reaching default point")
        if DEBUG:
            print(are_array_close(self.joint_state, np.array(self.default_joint_state)))
        self.reach_joint_state(self.default_joint_state)

    def reach_point(self, curr_pose: np.ndarray, target: np.ndarray) -> None:
        """Given the robot curr pose and a point [x y z] in glbal frame, reach the point.
        
        Args:
            curr_pose: 1-by-3 np.ndarray of pose values [x, y, theta] relative to the global frame
            target: 1-by-3 np.ndarray of point values [x, y, z] relative to the global frame
        """
        rospy.loginfo(rospy.get_caller_id() + " Reaching point: " + str(target))
        joint_state = self.globalpoint2jointstate(curr_pose, target)
        self.reach_joint_state(joint_state)

    def reach_point_robot(self, point: np.ndarray) -> None:
        """Given a point [x y z] in base frame, reach the point.
        
        Args:
            point: 1-by-3 np.ndarray of point values [x, y, z] relative to the base frame
        """
        rospy.loginfo(rospy.get_caller_id() + " Reaching point: " + str(point))
        joint_state = self.localpoint2jointstate(point)
        self.reach_joint_state(joint_state)

    

    def globalpoint2jointstate(self, curr_pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Given the robot curr pose and a point [x y z] in glbal frame, 
        return the joint state so that the end-effector of the arm is at the point.
        
        Args:
            curr_pose: 1-by-3 np.ndarray of pose values [x, y, theta] relative to the global frame
            target: 1-by-3 np.ndarray of point values [x, y, z] relative to the global frame
        Returns:
            joint_state: 1-by-3 np.ndarray of joint values [wrist_extension, lift_position, wrist_yaw]
        """
        joint_state = np.zeros(3)

        wrist_extension = np.linalg.norm(curr_pose[:2] - target[:2])
        joint_state[0] = max(wrist_extension - self.wrist_extension_offset, self.min_wrist_extension)

        lift_position = target[2] - self.lift_height_offset
        joint_state[1] = max(lift_position, self.min_lift_height)

        # wrist_yaw = 0

        return joint_state

    def localpoint2jointstate(self, point: np.ndarray) -> np.ndarray:
        """Given a pose, return the joint state so that the end-effector of the arm is at the pose.
        
        Args:
            pose: 1-by-3 np.ndarray of pose values [x, y, z] relative to the base frame
        Returns:
            joint_state: 1-by-3 np.ndarray of joint values [wrist_extension, lift_position, wrist_yaw]
        """
        joint_state = np.zeros(3)

        # wrist_extension = np.linalg.norm(pose[:2])
        print(np.linalg.norm(point[:2]))
        joint_state[0] = max(np.linalg.norm(point[:2]) - self.wrist_extension_offset, 0)
        
        # lift_position = z - offset
        joint_state[1] = max(point[2] - self.lift_height_offset, 0)

        # wrist_yaw = 0
        return joint_state
    
    def jointstate2localpoint(self, joint_state: np.ndarray) -> np.ndarray:
        """Given a joint state, return the pose of the end-effector relative to the base frame."""
        return np.array([0, -(joint_state[0] + self.wrist_extension_offset), joint_state[1] + self.lift_height_offset])


    def reach_joint_state(self, joint_state: np.ndarray) -> None:
        """Given a joint state, reach the joint state.
        
        Args:
            joint_state: 1-by-3 np.ndarray of joint values [wrist_extension, lift_position, wrist_yaw]
        Returns:
            None
        """
        joint_state = self.clip_joint_state(joint_state)
        rospy.loginfo(rospy.get_caller_id() + " Reaching joint state [extension, height, wrist_yaw]: " + str(joint_state))
        # self.moveArm(np.array(joint_state))
        with self.move_lock: 
            pose = dict()
            pose['wrist_extension'] = joint_state[0]
            pose['joint_lift'] = joint_state[1]
            pose['joint_wrist_yaw'] = joint_state[2]
            self.move_to_pose(pose)
            while not are_array_close(self.joint_state, joint_state, 1):
                print('Current joint state: ', self.joint_state)
                print('Target joint state: ', joint_state)
                print('Difference: ', are_array_close(self.joint_state, joint_state, 1))
                self.rate.sleep()
            # rospy.sleep(5)
            # rospy.sleep(self.arm_moving_time)

    def clip_joint_state(self, joint_state: np.ndarray) -> np.ndarray:
        """Clip the joint state to the valid range."""
        joint_state[0] = max(joint_state[0], self.min_wrist_extension)
        joint_state[0] = min(joint_state[0], self.max_wrist_extension)
        joint_state[1] = max(joint_state[1], self.min_lift_height)
        joint_state[1] = min(joint_state[1], self.max_lift_height)
        joint_state[2] = 0.0
        return joint_state

    def open_gripper(self):
        rospy.loginfo(rospy.get_caller_id() + ": Opening gripper")
        with self.move_lock:
            pose = {
                'gripper_aperture': 0.057,
                # 'gripper_aperture': 0.06,
            }
            self.move_to_pose(pose)
            rospy.sleep(self.gripper_moving_time)
    
    def close_gripper(self):
        rospy.loginfo(rospy.get_caller_id() + ": Closing gripper")
        with self.move_lock:
            pose = {
                'gripper_aperture': -0.05
            }
            self.move_to_pose(pose)
            rospy.sleep(self.gripper_moving_time)

    ## ==== Getter ==== ##
    def get_wrist_yaw_state(self, joint_states: JointState) -> list:
        joint_name = 'joint_wrist_yaw'
        i = joint_states.name.index(joint_name)
        wrist_yaw_position = joint_states.position[i]
        wrist_yaw_velocity = joint_states.velocity[i]
        wrist_yaw_effort = joint_states.effort[i]
        return [wrist_yaw_position, wrist_yaw_velocity, wrist_yaw_effort]
    
    def get_joint_state(self):
        """Get the current joint values of the arm."""
        while self.wrist_position is None or self.lift_position is None or self.wrist_yaw_position is None:
            rospy.loginfo(rospy.get_caller_id() + " Waiting for joint states")
            self.rate.sleep()
        return np.array([self.wrist_position, self.lift_position, self.wrist_yaw_position])
    
    def get_eeR_pose(self):
        """Get the pose of the end-effector relative to the robot frame.
        
        Returns:
            eeR_pose: 1-by-3 np.ndarray of pose values [x, y, z] relative to the robot frame
        """
        joint_state = self.get_joint_state()
        if DEBUG:
            print('Joint state: ', joint_state)
        return self.jointstate2localpoint(joint_state)
    

def test_open_gripper(arm_controller):
    try:
        arm_controller.open_gripper()
    except rospy.ROSInterruptException:
        pass

def test_close_gripper(arm_controller):
    try:
        arm_controller.close_gripper()
    except rospy.ROSInterruptException:
        pass

def test_move_arm(arm_controller):
    try:
        arm_controller.reach_joint_state(np.array([0.1, 0.2, 0]))
    except rospy.ROSInterruptException:
        pass

def test_reach_point(arm_controller):
    try:
        point = np.array([0.0, -0.5, 0.8])
        arm_controller.reach_point(point)
    except rospy.ROSInterruptException:
        pass

def test_get_eeR_pose(arm_controller):
    try:
        arm_controller.reach_default()
        print(arm_controller.get_eeR_pose())
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    arm_controller = ArmController()
    # test_open_gripper(arm_controller)
    # test_get_eeR_pose(arm_controller)
    # test_move_arm(arm_controller)
    # test_reach_point(arm_controller)
    arm_controller.reach_default()
    # test_close_gripper(arm_controller)
    
    
    

