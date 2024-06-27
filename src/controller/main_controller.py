#! /usr/bin/env python
from env_relax_repair.src.stretch_controller.mobile_controller import MobileController
from arm_controller import ArmController
import numpy as np
import rospy
import threading
import copy
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import (
    PoseStamped, 
    TransformStamped,
    Transform,
)

DEBUG = False

class MainController:
    """This class is the main controller for the Stretch RE1 robot.
    Given a goal position [x, y, z] in the global frame, this class will move the robot base close to [x,y] and move the end-effector of the arm to [x,y,z].
    """
    def __init__(self, objects_list: list = ['cone', 'block', 'plate', 'cup']) -> None:

        ## ==== Setup the threads related ==== ##
        self.shutdown_event = threading.Event() # <- fire event when the controller should shutdown
        self.ready_event = threading.Event() # <- fire event when the controller is ready
        self.path_condition = threading.Condition() # <- condition variable for the path
        self.shutdown_event.set()

        ## ==== Setup controllers ==== ## 
        self.arm_controller = ArmController()
        self.arm_controller.set_events(self.ready_event, self.shutdown_event, self.path_condition)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)
        self.max_lookup_iterations = 50
        self.base_controller = MobileController(transform_listener=self.tf_listener, base_only=False)
        # self.close_to_dist = 0.4
        self.close_to_dist = 0.51 # <- just for robot learning
        self.arm_extension_max = 0.51
        self.theta_base_offset = 0.1
        self.dropoff_offset = 0.02
        self.global_frame = 'global'
        self.ee_frame = 'link_gripper_fingertip_left'
        
        ## ==== Setup objects ==== ##
        self.objects_list = objects_list
        self.objects_setup(self.objects_list)
        self.primitive_skills_setup()

        self.__control_thread = None

        # ## ==== Arm reach default ==== ##
        # self.arm_controller.reach_default()

    def primitive_skills_setup(self) -> None:
        """Setup the primitive skills for the robot."""
        self.primitive_skills = {
            'movebase': self.movebase,
            'pickup': self.pickup,
            'dropoff': self.dropoff,
            'pickdrop': self.pickdrop,
            'default': self.default,
            'moveobj': self.pickdrop,
            'pick': self.pick,
            'move_rl': self.move_rl,
            'reach_joint_state': self.reach_joint_state,
        }

    def start_primitive(self, primitive: str, target: np.ndarray) -> None:
        """Start the primitive skill."""
        self.clear_shutdown()
        self.__control_thread = threading.Thread(target=self.primitive_skills[primitive], args=(target,))
        self.__control_thread.start()

    def start_arm_primitive(self, primitive: str, target: np.ndarray) -> None:
        """Start the primitive skill for arm in Robot Learning."""
        self.arm_shutdown_event.wait()
        self.arm_shutdown_event.clear()
        self.__control_thread = threading.Thread(target=self.primitive_skills[primitive], args=(target,))
        self.__control_thread.start()
        
    # ==== Primitive Skills ==== #
    def movebase(self, path: np.ndarray = None) -> None:
        """Move the base to the target position.
        
        Args:
            curr_pose: [x, y, theta] in the global frame
            path_region: [[x_min, x_max],[y_min, y_max]], ...] in the global frame
        """
        # if curr_pose is None:
        #     curr_pose = self.get_robot_pose()
        assert path is not None, "Target path is not provided."

        # Move the base to the target
        # self.clear_shutdown()
        self.base_controller.set_path(path)
        self.base_controller.control_loop(self.ready_event, self.shutdown_event, self.path_condition)
        self.shutdown_base()

    def pickup(self, target: np.ndarray = None) -> None:
        """Pick up the object at the target position.
        
        Args:
            curr_pose: [x, y, theta] in the global frame
            target: [x, y, z] in the global frame
        """
        # self.clear_shutdown()
        # Move the base to align to the target
        if DEBUG: breakpoint()
        self.align_base(target=target)
        print("Main controller shutdown: ", self.shutdown_event.is_set())
        curr_pose = self.get_robot_pose()
        assert target is not None, "Target position is not provided."

        self.arm_controller.pickup(curr_pose, target)
        self.shutdown_base()

    def pick(self, target: np.ndarray = None) -> None:
        """Pick up the object at the target position.
        
        Args:
            curr_pose: [x, y, theta] in the global frame
            target: [x, y, z] in the global frame
        """
        # self.clear_shutdown()
        # Move the base to align to the target
        if DEBUG: breakpoint()
        self.align_base(target=target)
        print("Main controller shutdown: ", self.shutdown_event.is_set())
        curr_pose = self.get_robot_pose()
        assert target is not None, "Target position is not provided."

        self.arm_controller.pick(curr_pose, target)
        self.shutdown_base()

    def move_rl(self, model) -> None: 
        """Move the robot according to the rl model"""
        cup_pose = self.get_cup_pose()
        self.arm_shutdown_event = threading.Event()
        self.arm_shutdown_event.set()

        for idx in range(len(model)):
            
            # action, _ = model.predict(cup_pose)
            action = model[idx]
            action = (action - 1) * 0.2
            
            # Arm target
            joint_state = self.arm_controller.get_joint_state()
            target_joint_state = joint_state.copy()
            target_joint_state[0] += action[1]
            target_joint_state[1] += action[2]
            # self.arm_controller.reach_joint_state(target_joint_state)

            # Base target
            base_pose = self.get_robot_pose()
            assert base_pose is not None, "Base pose is not provided."
            base_target = base_pose[:2].copy()
            base_target[1] -= action[0]

            # Start the thread to move the arm and the base together
            
            self.start_primitive('movebase', target = [base_target])
            self.start_arm_primitive('reach_joint_state', target = target_joint_state)
            self.arm_shutdown_event.wait()
            self.shutdown_event.wait()
            
            cup_pose = self.get_cup_pose()

    def reach_joint_state(self, target_joint_state: np.ndarray) -> None:
        """Reach the target joint state
        
        Args:
            target_joint_state: 1-by-7 np.ndarray of the target joint state
        """
        self.arm_controller.reach_joint_state(target_joint_state)
        self.arm_shutdown_event.set()
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            self.rate.sleep()

    def dropoff(self, target: np.ndarray = None) -> None:
        """Drop off the object at the target position.
        
        Args:
            curr_pose: [x, y, theta] in the global frame
            target: [x, y, z] in the global frame
        """
        # self.clear_shutdown()
        target[2] += self.dropoff_offset
        self.align_base(target=target)
        curr_pose = self.get_robot_pose()
        assert target is not None, "Target position is not provided."

        self.arm_controller.dropoff(curr_pose, target)
        self.shutdown_base()

    def pickdrop(self, target) -> None:
        """Pick up the object at the pickup_target position and drop it off at the dropoff_target position
        
        Args:
            pickup_target: [x, y, z] in the global frame
            dropoff_target: [x, y, z] in the global frame
        """
        pickup_target, dropoff_target = target[0], target[1]
        self.align_base(target=pickup_target)
        curr_pose = self.get_robot_pose()
        assert pickup_target is not None, "Pickup target position is not provided."
        target = pickup_target
        rospy.loginfo(rospy.get_caller_id() + " Picking up object at point: " + str(target))
        # if self.shutdown_event.is_set():
        #     return None
        # self.reach_default()
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.open_gripper()
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.reach_point(curr_pose, target)
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.close_gripper()
        if self.shutdown_event.is_set():
            return None
        
        if DEBUG: breakpoint()
        self.align_base(target=dropoff_target, arm_extension_max = self.arm_extension_max * 2)
        curr_pose = self.get_robot_pose()
        assert dropoff_target is not None, "Dropoff target position is not provided."
        target = dropoff_target
        rospy.loginfo(rospy.get_caller_id() + " Dropping off object at point: " + str(target))
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.reach_point(curr_pose, target)
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.open_gripper()
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.reach_default()
        if self.shutdown_event.is_set():
            return None    
        self.shutdown()

    def default(self, target=None) -> None:
        self.arm_controller.open_gripper()
        if self.shutdown_event.is_set():
            return None
        self.arm_controller.reach_default()
        if self.shutdown_event.is_set():
            return None    
        self.shutdown()
    # ================ #

    # ==== Demo ==== #
    def pickup_can_test_dropoff_depot(self) -> None:
        """Pick up the can_test and drop it off at the depot."""
        while self.depot_pose is None or self.can_test_pose is None:
            self.rate.sleep()
            print("Waiting for depot/can_test pose...")

        # Pick up the can_test
        self.reach_point(self.can_test_pose, 'pickup')
        # Drop off the can_test
        self.reach_point(self.depot_pose, 'dropoff')

    ## ==== Helper functions for primitive skills ==== ##
    def align_base(self, curr_pose: np.ndarray = None, target: np.ndarray = None, arm_extension_max: float = None) -> None:
        """Align the base to the target position.
        
        Args:
            curr_pose: [x, y, theta] in the global frame
            target: [x, y, z] in the global frame
        """
        if curr_pose is None:
            curr_pose = self.get_robot_pose()
        assert target is not None, "Target position is not provided."
        if DEBUG:
            print("Current pose: ", curr_pose)
            print("Target pose: ", target)
        # Find the target_base on the line between curr_pose and target
        if arm_extension_max is not None and np.linalg.norm(target[:2] - curr_pose[:2]) <= arm_extension_max:
            target_base = curr_pose[:2]
        else:
            target_base = self.find_close_to_point(curr_pose, target, self.close_to_dist)
        theta_base = self.find_theta(curr_pose, target) + self.theta_base_offset
        
        # Move the base to the target_base
        if DEBUG:
            print("Target base: ", target_base)
            print("Target theta: ", theta_base)
        self.base_controller.set_path([np.hstack([target_base, theta_base])])
        self.base_controller.control_loop(self.ready_event, self.shutdown_event, self.path_condition)
        

    def reach_point(self, target: np.ndarray, mode: str = 'reach') -> None:
        """Given a point [x y z] in the global frame, reach the point by the end-effector.
        
        Args:
            target: 1-by-3 np.ndarray of point values [x, y, z] in the global frame
        """
        # Find the target [x, y, theta] for the base
        while self.base_controller.get_curr_pose() is None:
            continue
        curr_pose = self.base_controller.get_curr_pose()

        # Align the base to the target for pickup
        self.align_base(curr_pose, target)

        # Find the target [x, y, z] in robot frame
        curr_pose = self.base_controller.get_curr_pose()
        # target_robot = self.global2robot(curr_pose, target)
        # print("Target in robot frame:", target_robot)
        # print("Distance: ", np.linalg.norm(target_robot[:2] - curr_pose[:2]))
        if mode == 'reach':
            self.arm_controller.reach_point(curr_pose, target)
        elif mode == 'pickup':
            self.pickup(curr_pose, target)
        elif mode == 'dropoff':
            self.dropoff(curr_pose, target)
        else:
            raise ValueError("Invalid mode: " + mode)
    
    def robot2global(self, curr_pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Convert the target point from the robot frame to the global frame
        
        Args: 
            curr_pose: 1-by-3 np.ndarray of the current pose of the robot [x, y, theta]
            target: 1-by-3 np.ndarray of the target point [x, y, z] in the robot frame
        Returns:
            target_global: 1-by-3 np.ndarray of the target point [x, y, z] in the global frame
        """
        theta = curr_pose[2]
        R_IB = np.array([[np.cos(theta), -np.sin(theta), curr_pose[0]],
                         [np.sin(theta), np.cos(theta), curr_pose[1]],
                         [0, 0, 1]])
        target_global = np.array([target[0], target[1], 1])
        # print('target_global', target_global)
        # print('R_IB', R_IB)
        target_global = np.dot(R_IB, target_global)
        target_global[2] = target[2]
        return target_global

    def find_close_to_point(self, curr_pose: np.ndarray, target: np.ndarray, close_to_dist: float) -> np.ndarray:
        """Find the point on the line between the robot and the target that is close_to_dist away from the target
        
        Args:
            curr_pose: 1-by-3 np.ndarray of the current pose of the robot [x, y, theta]
            target: 1-by-3 np.ndarray of the target point [x, y, z] in the global frame
            close_to_dist: float of the distance away from the target that the point should be
        Returns:
            target_base: 1-by-2 np.ndarray [x, y] of the point on the line between the robot and the target that is close_to_dist away from the target
        """
        theta = np.arctan2(curr_pose[1] - target[1], curr_pose[0] - target[0])
        unit_vector = np.array([np.cos(theta), np.sin(theta)])
        target_base = target[:2] + unit_vector * close_to_dist
        return target_base
    
    def find_theta(self, curr_pose: np.ndarray, target: np.ndarray) -> float:
        """Find the angle that the robot should turn to be parallel the target
        
        Args:
            curr_pose: 1-by-3 np.ndarray of the current pose of the robot [x, y, theta]
            target: 1-by-3 np.ndarray of the target point [x, y, z] in the global frame
        Returns:
            theta: float of the angle that the robot should turn to face the target
        """
        theta = np.arctan2(target[1] - curr_pose[1], target[0] - curr_pose[0])
        return theta + np.pi/2
    
    def find_pose(self, frame: str) -> Transform:
        """Find the pose of the robot in the given frame
        
        Args:
            frame: frame name of the object to find the pose of
        Returns:
            pose: Transform of the pose of the object in the global frame
        """
        for cnt in range(self.max_lookup_iterations):
            try:
                trans_stamped = self.tf_listener.buffer.lookup_transform(self.global_frame, frame, rospy.Time(), rospy.Duration(1.0))
                return trans_stamped.transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                if cnt % 2 == 0:
                    rospy.loginfo(f"Can't find transform between {self.global_frame} and {frame}")
                self.rate.sleep()
                continue
    
    def state_in_region(self, state: np.ndarray, region: list) -> bool:
        for dim in range(len(state)):
            if state[dim] < region[dim][0] or state[dim] > region[dim][1]:
                return False
        return True

    ## ==== Setup object callback functions ==== ##

    def objects_setup(self, objects: list) -> None:
        """Setup the objects in the scene
        
        Args:
            objects: list of objects to setup in the scene

        Q: How to add a new object?
        A: 1. add a new elif statement in the object_setup function
           2. add new setup and callback functions
           3. add a getter function for the object
           4. add the object to the get_poses function
        """
        for obj in objects:
            self.object_setup(obj)
    
    def object_setup(self, obj: str) -> None:
        """Setup the object in the scene
        
        Args:
            obj: name of the object to setup in the scene
        """
        if obj == 'depot':
            self.depot_setup()
        elif obj == 'can_test':
            self.can_test_setup()
        elif obj == 'plate':
            self.plate_setup()
        elif obj == 'cup':
            self.cup_setup()
        elif obj == 'cone':
            self.cone_setup()
        elif obj == 'block':\
            self.block_setup()
        else:
            raise ValueError("Invalid object: " + obj)
    
    def depot_setup(self) -> None:
        """Setup the depot subscriber"""
        self.depot_pose = None
        self.depot_sub = rospy.Subscriber('/mocap_node/depot/pose', PoseStamped, self.depot_callback, queue_size=1)
    
    def depot_callback(self, msg: PoseStamped) -> None:
        """Callback for the depot subscriber
        
        Args:
            msg: PoseStamped message of the depot
        Returns:
            depot_pose: 1-by-3 np.ndarray of the position of the depot in the global frame
        """
        depot_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = depot_pose_stamped_transformed.pose.position
        self.depot_pose = np.array([pos.x, pos.y, pos.z])

    def can_test_setup(self) -> None:
        """Setup the can_test subscriber"""
        self.can_test_pose = None
        self.can_test_sub = rospy.Subscriber('/mocap_node/can_test/pose', PoseStamped, self.can_test_callback, queue_size=1)

    def can_test_callback(self, msg: PoseStamped) -> None:
        """Callback for the can_test subscriber
        
        Args:
            msg: PoseStamped message of the can_test
        Returns:
            can_test_pose: 1-by-3 np.ndarray of the position of the can_test in the global frame
        """
        can_test_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = can_test_pose_stamped_transformed.pose.position
        self.can_test_pose = np.array([pos.x, pos.y, pos.z])

    def plate_setup(self) -> None:
        """Setup the plate subscriber"""
        self.plate_pose = None
        self.plate_sub = rospy.Subscriber('/mocap_node/plate/pose', PoseStamped, self.plate_callback, queue_size=1)

    def plate_callback(self, msg: PoseStamped) -> None:
        """Callback for the plate subscriber
        
        Args:
            msg: PoseStamped message of the plate
        Returns:
            plate_pose: 1-by-3 np.ndarray of the position of the plate in the global frame
        """
        plate_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = plate_pose_stamped_transformed.pose.position
        self.plate_pose = np.array([pos.x, pos.y, pos.z])
    
    def cup_setup(self) -> None:
        """Setup the cup subscriber"""
        self.cup_pose = None
        self.cup_sub = rospy.Subscriber('/mocap_node/cup/pose', PoseStamped, self.cup_callback, queue_size=1)
    
    def cup_callback(self, msg: PoseStamped) -> None:
        """Callback for the cup subscriber
        
        Args:
            msg: PoseStamped message of the cup
        Returns:
            cup_pose: 1-by-3 np.ndarray of the position of the cup in the global frame
        """
        cup_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = cup_pose_stamped_transformed.pose.position
        self.cup_pose = np.array([pos.x, pos.y, pos.z])

    def cone_setup(self) -> None:
        """Setup the cone subscriber"""
        self.cone_pose = None
        self.cone_sub = rospy.Subscriber('/mocap_node/cone2/pose', PoseStamped, self.cone_callback, queue_size=1)

    def cone_callback(self, msg: PoseStamped) -> None:
        """Callback for the cone subscriber
        
        Args:
            msg: PoseStamped message of the cone
        Returns:
            cone_pose: 1-by-3 np.ndarray of the position of the cone in the global frame
        """
        cone_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = cone_pose_stamped_transformed.pose.position
        self.cone_pose = np.array([pos.x, pos.y, pos.z]) 

    def block_setup(self) -> None:
        """Setup the block1 subscriber"""
        self.block_pose = None
        self.block_sub = rospy.Subscriber('/mocap_node/block1/pose', PoseStamped, self.block_callback, queue_size=1)

    def block_callback(self, msg: PoseStamped) -> None:
        """Callback for the block1 subscriber
        
        Args:
            msg: PoseStamped message of the block1
        Returns:
            block1_pose: 1-by-3 np.ndarray of the position of the block1 in the global frame
        """
        block_pose_stamped_transformed = self.transform_poststamped(msg, self.global_frame)
        pos = block_pose_stamped_transformed.pose.position
        self.block_pose = np.array([pos.x, pos.y, pos.z])

    # ======== Helper functions ======== #
    def get_transform(self, source_frame: str, target_frame: str, stamp = rospy.Time(), duration = rospy.Duration(1.0)) -> TransformStamped:
        """
        GET TRANSFORM: Get the transformation between two frames
        Args:
            source_frame: source frame
            target_frame: target frame
            stamp: timestamp
            duration: duration to wait for the transform
        Returns:
            trans: TransformStamped
        """
        # rospy.loginfo("In get_transform")
        try:
            trans = self.tf_listener.buffer.lookup_transform(source_frame, target_frame, stamp, duration)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(rospy.get_caller_id() + f"Cannot find transformation from {source_frame} to {target_frame}")
            return None
        return trans
    
    def transform_poststamped(self, pose_stamped: PoseStamped, target_frame: str) -> PoseStamped:
        """
        TRANSFORM POSTSTAMPED: Transform a PoseStamped from its current frame to a target frame
        Args:
            pose_stamped: PoseStamped
            target_frame: target frame
        Returns:
            post_stamped in target frame
        """
        source_frame = pose_stamped.header.frame_id
        trans = self.get_transform(source_frame, target_frame)
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() + " Transformation: %s" % trans)
        pose_stamped_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
        if DEBUG: 
            rospy.loginfo(rospy.get_caller_id() + " Transformed pose: %s" % pose_stamped_transformed)
        return pose_stamped_transformed
    
    # ==== Getter ==== #
    def get_robot_pose(self) -> np.ndarray:
        """Get the pose of the robot in the global frame"""
        while self.base_controller.get_curr_pose() is None and not rospy.is_shutdown():
            print("Waiting for robot pose")
            rospy.sleep(0.1)
            continue
        return self.base_controller.get_curr_pose()

    def get_can_pose(self) -> np.ndarray:
        """Get the position of the can in the global frame"""
        while self.can_test_pose is None and not rospy.is_shutdown():
            print("Waiting for can pose")
            rospy.sleep(0.1)
            continue
        return self.can_test_pose
    
    def get_depot_pose(self) -> np.ndarray:
        """Get the position of the depot in the global frame"""
        while self.depot_pose is None and not rospy.is_shutdown():
            print("Waiting for depot pose")
            rospy.sleep(0.1)
            continue
        return self.depot_pose
    
    def get_plate_pose(self) -> np.ndarray:
        """Get the position of the plate in the global frame"""
        while self.plate_pose is None and not rospy.is_shutdown():
            print("Waiting for plate pose")
            rospy.sleep(0.1)
            continue
        return self.plate_pose
    
    def get_cup_pose(self) -> np.ndarray:
        """Get the position of the cup in the global frame"""
        while self.cup_pose is None and not rospy.is_shutdown():
            print("Waiting for cup pose")
            rospy.sleep(0.1)
            continue
        return self.cup_pose
    
    def get_ee_pose(self) -> np.ndarray:
        """Get the position of the end effector in the global frame"""
        # Method 1: use tf transform
        # ee = self.find_pose(self.ee_frame)
        # return utils.transform_to_pq(ee)[0]

        # Method 2: use robot2global
        eeR = self.arm_controller.get_eeR_pose()
        if DEBUG:
            print('eeR: ', eeR)
        base_pose = self.base_controller.get_curr_pose()
        if DEBUG:
            print('base_pose: ', base_pose)
            print('robot2global: ', self.robot2global(base_pose, eeR))
        return self.robot2global(base_pose, eeR)
    
    def get_cone_pose(self) -> np.ndarray:
        """Get the position of the cone in the global frame"""
        while self.cone_pose is None and not rospy.is_shutdown():
            print("Waiting for cone pose")
            rospy.sleep(0.1)
            continue
        return self.cone_pose
    
    def get_block_pose(self) -> np.ndarray:
        """Get the position of the block1 in the global frame"""
        while self.block_pose is None and not rospy.is_shutdown():
            print("Waiting for block1 pose")
            rospy.sleep(0.1)
            continue
        return self.block_pose
    
    def get_poses(self) -> dict:
        poses = dict()
        for obj in self.objects_list:
            poses[obj] = getattr(self, 'get_' + obj + '_pose')()
        poses['base'] = self.get_robot_pose()
        poses['ee'] = self.get_ee_pose()
        if DEBUG:
            print('EE pose: ', poses['ee'])
        return poses
        # return {'base': self.get_robot_pose(),
        #         'ee': self.get_ee_pose(), 
        #         # self.get_can_pose(), 
        #         # self.get_depot_pose(), 
        #         'plate': self.get_plate_pose(), 
        #         'cup': self.get_cup_pose(),
        #         'cone': self.get_cone_pose(), 
        #         'block1': self.get_block1_pose()}
    
    def get_3Dposes(self) -> dict:
        poses = copy.deepcopy(self.get_poses())
        if DEBUG:
            print('EE pose: ', poses['ee'])
            print('cup pose: ', poses['cup'])
        poses['base'][2] = 0.0 # z is 0 because the robot is on the ground, originally base[2] is theta
        if 'cone' in self.objects_list:
            poses['cone'][2] = 0.0 # z is 0 because the cone is on the ground
        return poses
        
    def get_primitive_skills(self) -> dict:
        return self.primitive_skills
    
    ## ==== Threads related function ==== ##
    def clear_shutdown(self) -> None:
        """Clear the shutdown event"""
        self.shutdown_event.wait()
        self.shutdown_event.clear()
        self.base_controller.shutdown_event.clear()

    def shutdown(self) -> None:
        """Set the shutdown event"""
        self.shutdown_base()

    def is_alive(self) -> bool:
        """Check if the controller is alive"""
        return self.base_controller.is_event_alive(self.shutdown_event)

    def shutdown_base(self) -> None:
        self.base_controller.shutdown_events(ready_event=self.ready_event, shutdown_event=self.shutdown_event, path_condition=self.path_condition)

    def wait_for_shutdown(self) -> None:
        """Wait for the controller to shutdown"""
        if self.__control_thread:
            self.__control_thread.join()
            print("Main controller fully shutdown!")

def test_pickup(controller):
    point = [-1.5, 2, 0.7]
    target = np.array(point)
    controller.reach_point(target, mode='pickup')
    print('Done!')

def test_dropoff(controller):
    point = [0, 0, 0.2]
    target = np.array(point)
    controller.reach_point(target, mode='dropoff')
    print('Done!')

def test_find_pose(controller):
    frame = 'depot'
    pose = controller.find_pose(frame)
    print(pose)

def test_find_ee(controller):
    while not rospy.is_shutdown():
        ee_pose = controller.get_ee_pose()
        print("EE pose: ")
        print(ee_pose)
        print("cup pose: ")
        print(controller.get_cup_pose())
        controller.rate.sleep()

def test_objects(controller):
    while controller.depot_pose is None or controller.can_test_pose is None:
        controller.rate.sleep()
        print("Waiting for depot/can_test pose...")
    print("Depot position: " + np.array2string(controller.depot_pose))
    print("Can_test position: " + np.array2string(controller.can_test_pose))

if __name__ == '__main__':
    controller = MainController(objects_list=['cone', 'cup'])
    # test_pickup(controller)
    # test_dropoff(controller)
    # test_find_pose(controller)
    # test_objects(controller)
    # controller.arm_controller.reach_default()
    # controller.pickup_can_test_dropoff_depot()
    test_find_ee(controller)
    
        