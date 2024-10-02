#!/usr/bin/env python

import rospy
import tf2_ros, tf2_geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import (
    PoseStamped, 
    TransformStamped
)
from env_repair.scripts.tools import (
    pose_to_particle
)
import time
import threading

DEBUG = False
IS_SIM = False
IS_REAL = not IS_SIM
# BASE_ONLY = False

class BaseController:

    def __init__(self, x: float = None, y: float = None, t: float = None, tf_prefix="", transform_listener=None, base_only = True):
        ## ==== Setup events ==== ##
        self.shutdown_event = threading.Event()
        self.ready_event = threading.Event()
        self.path_condition = threading.Condition()
        
        ## ==== Setup parameters ==== ##
        self.IS_SIM = IS_SIM
        self.IS_REAL = not self.IS_SIM
        self.path = None
        self.path_region = None
        self.progress_threshold = 0.2
        self.progress_time_threshold = 60.0
        self.reset_state()
        self.pose_lock = threading.Lock()
        
        self.wheel2center = 0.15875

        self.kv = 1
        self.kw = 0.3
        # if t is None:
        #     self.set_path([[x, y]])
        # else:
        #     self.set_path([[x, y, t]])
        self.tf_prefix = tf_prefix
        self.base_only = base_only

        

        ## ==== Create publisher and subscriber ==== ##
        if self.IS_SIM or self.base_only:
            rospy.init_node('mobile_control', anonymous=True)
        if self.IS_SIM:
            self.max_vel = 0.5
            self.finish_threshold = 0.1
            self.finish_threshold_theta = 0.1
            self.frequency = 10
            self.pub = rospy.Publisher(
                '/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=10)
            self.sub = rospy.Subscriber('/ground_truth', Odometry, self.callback_sim)
        elif self.IS_REAL:
            self.max_vel = 0.1
            self.finish_threshold = 0.1
            self.finish_threshold_theta = 0.05
            self.frequency = 10
            self.pub = rospy.Publisher(
                '/stretch/cmd_vel', Twist, queue_size=10)
            self.sub = rospy.Subscriber('/mocap_node/stretch/pose', PoseStamped, self.callback_real, queue_size=1)
        else:
            raise ValueError("Must be either IS_SIM or IS_REAL")
        self.rate = rospy.Rate(self.frequency)
        if transform_listener:
            self.tf = transform_listener
        else:
            self.tf = tf2_ros.TransformListener(tf2_ros.Buffer())

    ######################    
    #### Control loop ####
    ######################

    def start(self):
        """Start the control loop"""
        # print("Path: ", self.path)
        self.clear_shutdown()
        self.__control_thread = threading.Thread(target=self._control_loop)
        self.__control_thread.start()

    def shutdown(self):
        """Shut down the control loop"""
        self.shutdown_events(self.ready_event, self.shutdown_event, self.path_condition)

    def shutdown_events(self, ready_event: threading.Event, shutdown_event: threading.Event, path_condition: threading.Condition):
        """Shut down the control loop"""
        shutdown_event.set()
        with path_condition:
            self._publish_control(np.array([0, 0]))
            path_condition.notify_all()
        ready_event.clear()
        with path_condition:
            self._publish_control(np.array([0, 0]))
            path_condition.notify_all()
        while self.is_event_alive(shutdown_event):
            self.rate.sleep()
        

    def clear_shutdown(self):
        """Clear the shutdown event"""
        self.shutdown_event.clear()

    def clear_shutdown_event(self, shutdown_event: threading.Event):
        """Clear the shutdown event"""
        shutdown_event.clear()

    def control_loop(self, ready_event: threading.Event, shutdown_event: threading.Event, path_condition: threading.Condition):
        """Implement the control loop, without shuttign down the controller after the function is completed"""
        ready_event.set()

        rospy.loginfo(rospy.get_caller_id() + " Start base control loop")
        if DEBUG: breakpoint()
        self.wrap_path_radians(self.path)
        gotopt = 0
        while not rospy.is_shutdown() and not shutdown_event.is_set():
            # Wait for path to be set
            with path_condition:
                while self.path is None and not shutdown_event.is_set() and not rospy.is_shutdown():
                    path_condition.wait()
                if shutdown_event.is_set() or rospy.is_shutdown():
                    break

                if self.curr_pose is None:
                    self.rate.sleep()
                    continue
                if self._no_progress():
                    rospy.logerr(rospy.get_caller_id(
                    ) + " Error: No progress after %ss -- Current pose: %s" % (self.progress_time_threshold, np.array2string(self.curr_pose)))
                    break
                gotopt = self.get_reference_index(self.curr_pose, self.path, gotopt)
                
                if self._path_complete(self.path, gotopt):
                    rospy.loginfo(rospy.get_caller_id() + " Reach the end of path.")

                    rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Path: %s" %
                                (np.array2string(self.curr_pose), np.array2string(self.path)))
                    return None
                self.goal_pose = np.array(self.path[gotopt])
                self.error = self._get_error(self.curr_pose, self.goal_pose)
                self.next_control = self._get_control(
                    self.curr_pose, self.goal_pose, self.error)
                if DEBUG:
                    rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
                            np.array2string(self.next_control))
                self._publish_control(self.next_control)
                self.rate.sleep()
        # self.shutdown_events(ready_event, shutdown_event, path_condition)

    def _control_loop(self):
        """Implement the control loop with threading"""
        self.ready_event.set()

        rospy.loginfo(rospy.get_caller_id() + " Start control loop")
        self.wrap_path_radians(self.path)
        gotopt = 0
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            # Wait for path to be set
            with self.path_condition:
                while self.path is None and not self.shutdown_event.is_set() and not rospy.is_shutdown():
                    self.path_condition.wait()
                if self.shutdown_event.is_set() or rospy.is_shutdown():
                    break

                if self.curr_pose is None:
                    self.rate.sleep()
                    continue
                if self._no_progress():
                    rospy.logerr(rospy.get_caller_id(
                    ) + " Error: No progress after %ss -- Current pose: %s" % (self.progress_time_threshold, np.array2string(self.curr_pose)))
                    break
                gotopt = self.get_reference_index(self.curr_pose, self.path, gotopt)
                
                if self._path_complete(self.path, gotopt):
                    rospy.loginfo(rospy.get_caller_id() + " Reach the end of path.")

                    rospy.loginfo(rospy.get_caller_id() + " -- Path completed -- Current pose: %s -- Path: %s" %
                                (np.array2string(self.curr_pose), np.array2string(self.path)))
                    break
                self.goal_pose = np.array(self.path[gotopt])
                self.error = self._get_error(self.curr_pose, self.goal_pose)
                self.next_control = self._get_control(
                    self.curr_pose, self.goal_pose, self.error)
                if DEBUG:
                    rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
                            np.array2string(self.next_control))
                self._publish_control(self.next_control)
                self.rate.sleep()
        self.shutdown()

    def control_loop_theta(self, target: np.ndarray, gotopt: int) -> None:
        """After reaching x, y, rotate theta to the target location"""
        rospy.loginfo(rospy.get_caller_id() + " Start theta control loop")
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            pose = self.curr_pose
            if self._no_progress():
                rospy.logerr(rospy.get_caller_id(
                ) + " Error: No progress after 20s -- Current pose: %s" % np.array2string(pose))
                break
            self.error = self._get_error_theta(pose, target)
            if self._path_complete_theta(self.error):
                rospy.loginfo(rospy.get_caller_id() + " -- Rotation completed for waypoint %s -- Current pose: %s -- Goal pose: %s" %
                              (gotopt, np.array2string(pose), np.array2string(target)))
                break
            self.next_control = self._get_control_theta(self.error)
            if DEBUG:
                rospy.loginfo(rospy.get_caller_id() + " Control: %s" %
                          np.array2string(self.next_control))
            self._publish_control(self.next_control)
            self.rate.sleep()

    def _get_control(self, pose: np.ndarray, target: np.ndarray, error: np.ndarray) -> np.ndarray:
        """Compute the PD control

        Args:
            pose: np.ndarray: current pose of the car [x, y, heading]
            target: np.ndarray: target pose [x, y, theta, v]
            error: np.ndarray error [e_x, e_y, e_heading]

        Returns:
            control: np.ndarray: linear velocity and steering angle [v_x, w_z]
        """
        raise NotImplementedError

    #### Helper functions ####

    def _get_error(self, pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Get the absolute difference between the current pose and the target pose"""
        raise NotImplementedError
    
    def get_curr_pose(self) -> np.ndarray:
        """Get the current pose of the car"""
        while self.curr_pose is None and not rospy.is_shutdown():
            rospy.loginfo(rospy.get_caller_id() + " Waiting for current pose")
            self.rate.sleep()
        with self.pose_lock:
            if DEBUG:
                print("Current base pose in get_curr_pose: ", self.curr_pose)
            return self.curr_pose

    def get_car_pose_sim(self, odometry: Odometry):
        """Return the current car pose

        Args:
            odometry: Odometry from nav_msgs.ms

        Returns:
            pose: np array [x, y, heading]
        """
        return np.array(pose_to_particle(odometry.pose.pose))
    
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
            trans = self.tf.buffer.lookup_transform(source_frame, target_frame, stamp, duration)
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

    def get_car_pose_real(self, pose_stamped: PoseStamped):
        """Return the current car pose

        Args:
            pose_stamped: PoseStamped from geometry_msgs.msg

        Returns:
            pose: np array [x, y, heading]
        """
        pose_stamped_transformed = self.transform_poststamped(pose_stamped, 'global')
        return np.array(pose_to_particle(pose_stamped_transformed.pose))
    
    def _get_error_theta(self, pose: np.ndarray, target: np.ndarray) -> float:
        return self._wrap_radians_angle(pose[2] - target[2])

    def _path_complete_theta(self, error: float) -> bool:
        return np.linalg.norm(error) < self.finish_threshold_theta

    def _get_control_theta(self, error: float) -> np.ndarray:
        return np.array([0, -self.kw * error])

    def callback_sim(self, odometry: Odometry):
        self.curr_pose = self.get_car_pose_sim(odometry)
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() + " Current pose: %s" %
                      np.array2string(self.curr_pose))
            
    def callback_real(self, pose_stamped: PoseStamped):
         with self.pose_lock:
            self.curr_pose = self.get_car_pose_real(pose_stamped)
            if DEBUG:
                rospy.loginfo(rospy.get_caller_id() + " Current pose: %s" %
                        np.array2string(self.curr_pose))

    def get_reference_index(self, pose: np.ndarray, path: list, gotopt: int):
        """Return the index to the next control waypoint on the path

        Args:
            pose: np.ndarray: current pose of the car [x, y, heading]
            path: list[np.ndarray] of control waypoints

        Returns:
            index to the next control waypoint on the reference path
        """
        if gotopt >= len(path):
            print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
            print("gotopt out of range")
            print("gotopt: ", gotopt)
            print("path: ", path)
            print("path region: ", self.path_region)
            print("\n\n\n\n\n\n\n")
            raise SystemExit(0)
            exit(0)
        target = np.array(path[gotopt])
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() + " Difference Norm: %s" % np.linalg.norm(pose[:2] - target[:2]))
            print("Target: ", target)
        waypoint_reached = np.linalg.norm(pose[:2] - target[:2]) < self.finish_threshold or (self.path_region is not None and self.point_in_region(pose[:2], self.path_region[gotopt]))
        if waypoint_reached:
            rospy.loginfo(rospy.get_caller_id() + f": Reach waypoint {gotopt}")
            if self.is_xyt(target):
                self.control_loop_theta(target, gotopt)
            return gotopt + 1
        else:
            return gotopt
        
    def point_in_region(self, point: np.ndarray, region: np.ndarray) -> bool:
        """Returns true if the point is in the region
        
        Args:
            point: np.ndarray [x, y]
            region: np.ndarray [[x_min, x_max], [y_min, y_max]]   
        """
        return region[0][0] <= point[0] <= region[0][1] and region[1][0] <= point[1] <= region[1][1]

    def _no_progress(self) -> bool:
        """Returns true if there is no progress after 10 seconds"""
        with self.pose_lock:
            if self.prev_pose is None:
                self.prev_pose = self.curr_pose
                self.prev_pose_stamp = time.time()
            else:
                progress = np.linalg.norm(self.curr_pose - self.prev_pose)
                if progress > self.progress_threshold:
                    self.prev_pose = self.curr_pose
                    self.prev_pose_stamp = time.time()
                elif time.time() - self.prev_pose_stamp > self.progress_time_threshold:
                    return True
            return False

    def _path_complete(self, path: list, gotopt: int) -> bool:
        """Returns whether the reference path has been completed

        Args:
            path: list of pose [x, y, heading]
            error: current error [e_x, e_y, e_theta]

        Returns:
            True if the path has been completed
        """
        # return (self.get_reference_index(pose, self.path) == (len(self.path) - 1)) and (np.linalg.norm(error) < self.finish_threshold)
        return gotopt == len(path)

    def _publish_control(self, control: np.ndarray) -> None:
        """Publish a control input to stretch

        Args:
            control: np.ndarray: [v_x, w_z]

        Returns:
            publishes a Twist message 

        """
        # control_published = control
        control_published = self.limit_control(control, self.max_vel, self.wheel2center)
        if DEBUG:
            rospy.loginfo(rospy.get_caller_id() + " Published Control: %s" % control_published)
        control_cmd = Twist()
        control_cmd.linear.x = control_published[0]
        control_cmd.angular.z = control_published[1]
        self.pub.publish(control_cmd)

    def limit_control(self, control: np.ndarray, max_vel: float, wheel2center: float) -> np.ndarray:
        """Limit the control input to the maximum velocity"""
        v_x = control[0]
        w_z = control[1]
        vl = v_x - wheel2center * w_z
        vr = v_x + wheel2center * w_z
        if np.abs(vl) > max_vel:
            factor = np.abs(vl / max_vel)
            vl /= factor
            vr /= factor
        if np.abs(vr) > max_vel:
            factor = np.abs(vr / max_vel)
            vl /= factor
            vr /= factor
        v_x = (vr + vl) / 2
        w_z = (vr - vl) / (2 * wheel2center)
        return np.array([v_x, w_z])


    def wrap_path_radians(self, path):
        for target in path:
            if len(target) >= 3:
                target[2] = self._wrap_radians_angle(target[2])

    def _wrap_radians_angle(self, angle):
        """Wraps the given angle to the range [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    # def set_target(self, x: float = None, y: float = None, t: float = None):
    #     self.is_xyt_controller = t is not None
    #     if self.is_xyt_controller:
    #         self.path = [np.array([x, y, t])]  # pose = [x, y, theta]
    #     else:
    #         self.path = [np.array([x, y])]

    def set_path(self, path: list) -> None:
        """Set the reference path

        Args:
            path: list of pose [x, y, heading]
        """
        with self.path_condition:
            self.reset_state()
            assert len(path) > 0, "Path should not be empty"
            # print(path)
            self.path = np.array(path)
            # print(self.path)
            self.path_condition.notify_all()
            rospy.loginfo(rospy.get_caller_id() + " Set Path: %s" % self.path)

    def set_path_region(self, regions: list) -> None:
        """Set the reference path

        Args:
            regions: list of regions [[x1, x2], [y1, y2]]
        """
        with self.path_condition:
            self.reset_state()
            assert len(regions) > 0, "Regions should not be empty"
            assert len(regions[0]) == 2, "Regions should be 2D"
            dim = 2
            # TODO: set path and regions
            self.path = np.array([self.select_middle_point_in_region(region, dim) for region in regions])
            self.path_region = np.array(regions)
            self.path_condition.notify_all()
            rospy.loginfo(rospy.get_caller_id() + " Set Path: %s" % self.path)
            rospy.loginfo(rospy.get_caller_id() + " Set Path Region: %s" % self.path_region)

    def select_middle_point_in_region(self, region: list, dims) -> np.array:
        """Select a point in the bounds
        
        Args:
            region: list of bounds [[x1, x2], [y1, y2]]
            dims: number of dimensions = 2
        """
        target_location = np.zeros(dims)
        for dim in range(dims):
            # target_location[dim] = np.random.uniform(bounds[dim][0], bounds[dim][1])
            target_location[dim] = (region[dim][0] + region[dim][1]) / 2
        return target_location

    def is_xyt(self, point: np.ndarray) -> bool:
        rospy.loginfo(rospy.get_caller_id() + " is_xyt Point: %s" % point)
        return len(point) == 3

    def reset_state(self):
        """Reset the controller state"""
        self.prev_pose = None
        self.prev_pose_stamp = None
        self.curr_pose = None
        self.path = None
        self.path_region = None

    def is_ready(self) -> bool:
        """Returns whether the controller is ready to start following the path"""
        return self.ready_event.is_set()
    
    def is_alive(self) -> bool:
        """Returns whether the controller is alive"""
        return self.is_event_alive(self.shutdown_event)
    
    def is_event_alive(self, shutdown_event: threading.Event) -> bool:
        """Returns whether the controller is alive"""
        return not shutdown_event.is_set()

if __name__ == '__main__':
    raise NotImplementedError(
        "Should not call the base controller. Should initialize the subclass controller and call the control loop.")
