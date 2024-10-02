#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import sys
from controller.base_controller import BaseController


sys.path.insert(0, '/home/qian/catkin_ws/src/stretch_controller/scripts')

from tools import print_debug

DEBUG = False

class MobileController(BaseController):
    def __init__(self, x: np.float = None, y: np.float = None, t = None, tf_prefix="", transform_listener=None, base_only = True):
        # print(t)
        # return
        super().__init__(x, y, t, tf_prefix, transform_listener, base_only)
        if self.IS_SIM:
            self.epsilon = 0.15
            # self.set_path(np.array([[0,2], [2,2,5/4*np.pi], [0,0], [2,0], [2,2], [0,0,0], [-2, -3, -1]]))
        else:
            self.epsilon = 0.1
            # self.set_path(np.array([[-2,0]]))
            # if x is not None and y is not None:
            #     if t is not None and t != 0.0:
            #         rospy.loginfo(rospy.get_caller_id() + " t: %s" % t)
            #         self.set_path(np.array([[x, y, t]]))
            #     else:
            #         self.set_path(np.array([[x, y]]))

    def _get_control(self, pose: np.ndarray, target: np.ndarray, error: np.ndarray) -> np.ndarray:
        """
        Args:
            pose: [x, y, theta] in global frame
            target: [x, y, theta] in global frame
            error: [e_x, e_y] in global frame
        
        Returns:
            control: [v, w]
        """
        theta = pose[2]
        R_BI = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        V_I =  error
        V_B = np.matmul(R_BI, V_I)
        control = np.matmul(np.array([[1, 0], [0, 1/self.epsilon]]), V_B)
        return control

    def _get_error(self, pose: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Returns errors in x, y
        Args:
            pose: [x, y ,theta]
            target: [x, y, theta]

        Returns:
            error: [x, y, theta]
        """
        # return pose - target
        return target[:2] - pose[:2]

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            prog='StretchPointController', description='Control the stretch robot to move to a user-provided target pose [x, y, theta]')
        parser.add_argument('-x', action='store', dest='x', default='-3')
        parser.add_argument('-y', action='store', dest='y', default='0')
        parser.add_argument('-t', action='store', dest='t', default='0')
        args = parser.parse_args()
        controller = MobileController(
            np.float(args.x), np.float(args.y), np.float(args.t))
        controller.set_path([[np.float(args.x), np.float(args.y), -np.float(args.t)]])
        # controller.set_path([[2, 1, 1.57], [-2, 1, 3], [6, -2, -1.57]])
        # controller.set_path_region([[[0, 1], [0, 1]], [[0, 1], [1, 2]], [[0, 1], [2, 3]], [[1, 2], [2, 3]]])
        # rospy.sleep(1)
        print(controller.path)
        controller.start()
        # rospy.sleep(5)
        # if controller.is_alive():
        #     controller.shutdown()
    except rospy.ROSInterruptException:
        pass
