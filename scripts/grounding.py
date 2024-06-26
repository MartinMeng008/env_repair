#! /usr/bin/env python

import argparse

import threading
import rospy
import copy
import numpy as np
import threading

from .tools import (json_load_wrapper, 
                   symbol_to_object_location, 
                   dump_json,
                   create_symbol_dict,
                   list2dict_bool,
                   dict_bool2list,
                   find_controllable_symbols,
                   find_uncontrollable_symbols,
                   find_controllable_mobile_symbols,
                   find_controllable_manipulation_symbols,
                   assert_exit,)

from env_relax_repair.src.stretch_controller.main_controller import MainController

DEBUG = False
CREATE_SYMBOLS_ONLY = False

class Groundings:
    def __init__(self, files_json, objects_data: dict, locations_data: dict, user_inputs_data: dict, controller: MainController = None):
        """Grounding class for the cupplate example
           Given the locations, objects, and the location's grounding, 
           create the controllable variables and 
           provide the true value of the variables
        
        Args:
            files_json: str: json file name containing all json files
            objects_data: dict: dictionary of objects
            locations_data: dict: dictionary of locations
            user_inputs_data: dict: dictionary of user inputs
            controller: MainController: the controller of the robot
        """
        ## ==== Setup the inputs ==== ##
        self.controller = controller

        # We don't need to monitor uncontrollable variables separately because all variables have grounding
        # self.uncontrollable_variables_list = uncontrollable_variables
        # self.exists_uncontrollable_variables = len(self.uncontrollable_variables) > 0
        # if self.exists_uncontrollable_variables:
        #     self.start_monitoring_uncontrollable_variable_thread()

        self.objects_data = objects_data
        self.locations_data = locations_data
        self.user_inputs_data = user_inputs_data
        self.files = json_load_wrapper(files_json)
        self.inputs = self.create_inputs()

        ## ==== Setup the threads related ==== ##
        self.ready_event = threading.Event()
        self.user_input_ready_event = threading.Event() 
        self.shutdown_event = threading.Event()
        self.change_event = threading.Event() # <- fire event when the controllable inputs change
        self.state_lock = threading.RLock()
        self.user_input_lock = threading.RLock()
        self.frequency = 1
        self.rate = rospy.Rate(self.frequency)

        ## ==== Setup the constants ==== ##
        self.ee_epsilon = 0.17  
        self.base_epsilon = 0.4
        self.height_default = 1.0
        self.default_user_input_status = True
        self.user_input_status = self.default_user_input_status

        self.start()
    
    def create_inputs(self) -> list:
        """Create symbols from the json files

        Args:
            self.objects_data: dict: dictionary of objects
            self.locations_data: dict: dictionary of locations
            self.user_inputs_data: dict: dictionary of user inputs
        Returns:
            self.inputs_data: dict: dictionary of inputs
            self.controllable_inputs: list[str]: list of controllable inputs
            self.uncontrollable_inputs: list[str]: list of uncontrollable inputs
            self.controllable_mobile_inputs: list[str]: list of controllable mobile inputs
            self.controllable_manipulation_inputs: list[str]: list of controllable manipulation inputs
            inputs: list[str]: list of symbols
            self.user_input: str: the user input or None if no user input
        """
        self.inputs_data = self.create_inputs_from_objects_and_locations(
            self.objects_data, 
            self.locations_data)
        
        self.controllable_inputs = find_controllable_symbols(self.inputs_data, objects_data=self.objects_data)
        self.controllable_mobile_inputs = find_controllable_mobile_symbols(self.inputs_data, objects_data=self.objects_data)
        self.controllable_manipulation_inputs = find_controllable_manipulation_symbols(self.inputs_data, objects_data=self.objects_data)
        self.uncontrollable_inputs = find_uncontrollable_symbols(self.inputs_data, objects_data=self.objects_data) + list(self.user_inputs_data.keys())

        self.inputs_data.update(self.user_inputs_data)       

        dump_json(self.files['inputs'], self.inputs_data)
        dump_json(self.files['controllable_inputs'], {"controllable_inputs": self.controllable_inputs})
        dump_json(self.files['uncontrollable_inputs'], {"uncontrollable_inputs": self.uncontrollable_inputs})
        dump_json(self.files['controllable_mobile_inputs'], {"controllable_mobile_inputs": self.controllable_mobile_inputs})
        dump_json(self.files['controllable_manipulation_inputs'], {"controllable_manipulation_inputs": self.controllable_manipulation_inputs})

        # Test symbol generation
        if CREATE_SYMBOLS_ONLY:
            print('')
            for key, val in self.inputs_data.items():
                print(f'{key}: {val}')
            print('')
            exit(0)

        # self.state = list2dict_bool(self.symbols)
        # print(self.controllable_inputs_state)
        if DEBUG:
            for key, val in self.state.items():
                print(f'"{key}": false,')
            exit(0)

        user_inputs = list(self.user_inputs_data.keys())
        assert len(user_inputs) <= 1, f'Only one user input is allowed, but {len(user_inputs)} are found'
        self.user_input = user_inputs[0] if len(user_inputs) == 1 else None
        # self.full_locations_name = workspace['locations'] + workspace['robot_locations']
        # self.full_objects_name = workspace['objects'] + workspace['robot_objects']
        return list(self.inputs_data.keys())
    
    def create_inputs_from_objects_and_locations(self, objects_data: dict, locations_data: dict) -> dict:
        """Create inputs from the objects and locations
        Args:
            objects_data: dict: dictionary of objects
            locations_data: dict: dictionary of locations
        Returns:
            inputs_data: dict: dictionary of inputs
        """
        inputs_data = {}
        idx = 0
    
        ## objects X locations
        for obj, obj_dict in objects_data.items():
            for loc, loc_dict in locations_data.items():
                if obj_dict['type'] == loc_dict['type']:
                    sym_name = f'p_{obj}_{loc}'
                    inputs_data[sym_name] = create_symbol_dict(sym_name, obj, loc, idx, locations_data)
                    idx += 1
        
        return inputs_data
        
    def reset_states(self) -> None:
        """Reset the input state"""
        self.prev_state = None
        self.state = list2dict_bool(self.inputs)
        
        self.monitor_state()
        if DEBUG:
            print(self.state)
            exit(0)
        with open(self.files['log_grounding'], 'w') as f:
            f.write(f"{dict_bool2list(self.state)}\n")
        self.shutdown_event.clear()
        self.ready_event.clear()
        self.change_event.clear()
    
    def start(self):
        """Start the monitoring"""
        self.reset_states()
        
        if self.user_input is not None:
            self.user_input_thread = threading.Thread(target=self._signal_user_input)
            self.user_input_thread.start()
            self.user_input_ready_event.wait()

        self.__monitor_thread = threading.Thread(target=self._monitor_loop)
        self.__monitor_thread.start()
        self.ready_event.wait()
        print('Grounding is ready')

    def shutdown(self):
        """Shut down the monitoring"""
        self.shutdown_event.set()
        self.ready_event.clear()

    def clear_change_event(self):
        """Clear the change event"""
        self.change_event.clear()

    def is_ready(self) -> bool:
        """Check if the grounding is ready"""
        return self.ready_event.is_set()

    def is_shutdown(self) -> bool:
        """Check if the grounding is alive"""
        return self.shutdown_event.is_set()

    def get_inputs(self) -> dict:
        """Get the inputs
        
        Returns:
            inputs: dict[str, bool]: representing the truth value of input props
        """
        self.monitor_state()
        with self.state_lock:
            return copy.deepcopy(self.state)
    
    def update_controllable_inputs(self, inputs: dict) -> None:
        """Update the controllable inputs in place
        
        Args:
            inputs: dict[str, bool]: representing the truth value of input props
        """
        self.monitor_state()
        with self.state_lock:
            for input in self.controllable_inputs:
                inputs[input] = self.state[input]
        return None

    def _signal_user_input(self) -> None:
        """"A thread to receive the user input"""
        with self.user_input_lock:
            self.user_input_status = self.default_user_input_status
            self.user_input_ready_event.set()
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            status = input('Please enter the user input (y/n): ')
            print("user_input: ", status)
            assert_exit(status in ['y', 'n'], f'Invalid user input: {status}')
            with self.user_input_lock:
                self.user_input_status = status == 'y'
                self.default_user_input_status = self.user_input_status



    def _monitor_loop(self) -> None:
        """Continuously monitor the inputs, stop the monitor if the change_event is set"""
        # self.ready_event.set() # <- Don't need this because we set it in the loop
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            self.monitor_state()
            if DEBUG:
                print(self.state)
            with self.state_lock:
                if self.prev_state is not None and self.prev_state != self.state:
                    print('Input state changed!')
                    print(f'Previous state: {dict_bool2list(self.prev_state)}')
                    print(f'Current state: {dict_bool2list(self.state)}')
                    with open(self.files['log_grounding'], 'a') as f:
                        f.write(f"{dict_bool2list(self.state)}\n")
                    self.change_event.set()
                self.prev_state = copy.deepcopy(self.state)
            if not self.ready_event.is_set():
                self.ready_event.set()
            self.rate.sleep() 
    
    def monitor_state(self) -> None:
        """Monitor the state of the inputs"""
        self.monitor_grounding_inputs()
        self.monitor_user_inputs()
        return None
    
    def monitor_user_inputs(self) -> None:
        """Update the user input"""
        if self.user_input is None:
            with self.user_input_lock:
                self.user_input_status = self.default_user_input_status
        with self.state_lock:
                with self.user_input_lock:
                    self.state[self.user_input] = self.user_input_status

    def monitor_grounding_inputs(self) -> None:
        """Update the controllable_inputs_state based on controller information
        
        Given:
            self.controllable_inputs_state: dict[str, bool]: dictionary of controllable inputs
            self.locations: dict[str, dict]: groundings of locations
            poses of objects from self.controller
        Find:
            an updated self.controllable_inputs_state
        """
        with self.state_lock:
            poses = self.controller.get_3Dposes() # dict[str, np.ndarray]
            if DEBUG:
                print("cup pose", poses['cup'])
            for input in self.state.keys():
                if not self.inputs_data[input]:
                    # SKip monitoring the user input
                    continue 
                o, l = symbol_to_object_location(input)
                pose = poses[o]
                if l in self.locations_data.keys():
                    location_dict = self.locations_data[l]
                    if l == 'ee':
                        poses = self.controller.get_3Dposes()
                        location_dict = self.create_box_dict_from_pose_and_height(poses['base'], self.base_epsilon, self.ee_epsilon, self.height_default)
                else:
                    raise ValueError(f'Location {l} not found in locations_data')
                self.state[input] = self.check_pose_in_location(pose, location_dict)
            return None

    def check_pose_in_location(self, pose: np.ndarray, location_dict: dict) -> bool:
        """Check if the pose is in the location"""
        for dim in location_dict['dims']:
            bounds = location_dict['bounds'][dim]
            if pose[dim] < bounds[0] or pose[dim] >= bounds[1]:
                return False
        return True

    def create_box_dict_from_pose(self, pose: np.ndarray, epsilon: float) -> dict:
        """Create a box dictionary from the pose and epsilon"""
        box_dict = {}
        box_dict['dims'] = [0, 1, 2]
        box_dict['bounds'] = [[pose[0] - epsilon, pose[0] + epsilon], 
                              [pose[1] - epsilon, pose[1] + epsilon], 
                              [pose[2] - epsilon, pose[2] + epsilon]]
        return box_dict
    
    def create_box_dict_from_pose_and_height(self, base: np.ndarray, base_epsilon: float, height_epsilon, height: float) -> dict:
        """Create a box dictionary from the pose and epsilon and height"""
        base = copy.deepcopy(base)
        base[2] = height
        box_dict = {}
        box_dict['dims'] = [0, 1, 2]
        box_dict['bounds'] = [[base[0] - base_epsilon, base[0] + base_epsilon], 
                              [base[1] - base_epsilon, base[1] + base_epsilon], 
                              [base[2] - height_epsilon, base[2] + height_epsilon]]
        return box_dict
        

def test_groundings(groundings: Groundings) -> None:
    """Test the grounding class"""
    print('Testing Groundings')
    print('Testing controllable inputs')
    rate = rospy.Rate(1)
    prev_state = None
    unique_states = []
    log_file_name = "log_grounding.txt"
    with open(log_file_name, 'w') as f:
        f.write('')
    while not rospy.is_shutdown():
        # groundings.monitor_state()
        # print(groundings.get_state())
        state = groundings.get_inputs()
        input_state = [symbol for symbol, val in state.items() if val]
        if prev_state is None or prev_state != input_state:
            unique_states.append(input_state)
            prev_state = input_state
            with open(log_file_name, 'a') as f:
                # f.write(f'{input_state}\n')
                f.write('{\n')
                for input in input_state:
                    if input not in groundings.uncontrollable_inputs:
                        f.write(f'"{input}": true,\n')
                for input in groundings.controllable_inputs:
                    if input not in input_state:
                        f.write(f'"{input}": false,\n')
                f.write('}\n')
        print('=====================')
        for input in input_state:
            print(input)
        print("cup: ", groundings.controller.get_cup_pose())
        print("base: ", groundings.controller.get_robot_pose())
        # exit(0)
        rate.sleep()
    print('Unique states: ')
    for state in unique_states:
        print(state)
    

def test_pickup_dropoff_cup(main_controller, groundings: Groundings) -> None:
    # Setup
    main_controller = MainController()
    primitive_skills = main_controller.get_primitive_skills()
    cup_target = np.array([1.77, 0.4, 0.85])
    cup_target_base = cup_target.copy()
    cup_target_base[0] -= 0.5

    # Monitor controllable inputs
    grounding_thread = threading.Thread(target=test_groundings, args=(groundings,))
    groundings.reset_states()
    grounding_thread.start()

    cup_pose = main_controller.get_cup_pose()
    cup_pose_base = cup_pose.copy()
    cup_pose_base[0] += 0.5
    primitive_skills["movebase"](target = cup_pose_base[:2])
    primitive_skills["pickup"](target = cup_pose)

    primitive_skills["movebase"](target = cup_target_base[:2])
    primitive_skills["dropoff"](target = cup_target)

def test_pickup_dropoff_plate_cup(main_controller, groundings) -> None:
    # Setup
    main_controller = MainController()
    # primitive_skills = main_controller.get_primitive_skills()
    plate_target = np.array([1.7, 0, 0.8])
    cup_target = np.array([1.77, 0.4, 0.85])
    
    # Monitor controllable inputs
    grounding_thread = threading.Thread(target=test_groundings, args=(groundings,))
    grounding_thread.start()

    # Pick up and drop off cup
    
    main_controller.align_base(target = main_controller.get_cup_pose())
    main_controller.pickup(target = main_controller.get_cup_pose())
    main_controller.align_base(target = cup_target)
    main_controller.dropoff(target = cup_target)

    # Pick up and drop off plate
    main_controller.align_base(target = main_controller.get_plate_pose())
    main_controller.pickup(target = main_controller.get_plate_pose())
    main_controller.align_base(target = plate_target)
    main_controller.dropoff(target = plate_target)

def test_pickup_block1_thread(main_controller, groundings) -> None:
    rospy.loginfo('Reaching default pose')
    main_controller.arm_controller.reach_default()
    poses = main_controller.get_poses()
    x4_midpt = [-0.945, 0]

    # groundings_thread = threading.Thread(target=test_groundings, args=(groundings,))
    # groundings_thread.start()
    rospy.loginfo('Align to block1')
    main_controller.start_primitive('movebase', target = [x4_midpt])
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    rospy.loginfo('Picking up block1')
    main_controller.start_primitive('pickup', target = main_controller.get_block1_pose())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished picking up block1")

    block1_target = np.array([-1.60817322, 0.46190252, 0.81105231])
    rospy.loginfo('Dropoff block1')
    main_controller.start_primitive('dropoff', target = block1_target)
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished dropping off block1")

    rospy.loginfo('Picking up the cup')
    main_controller.start_primitive('pickup', target = main_controller.get_cup_pose())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished picking up the cup")

def test_pickdrop_block_thread(main_controller: MainController, groundings: Groundings):
    rospy.loginfo('Reaching default pose')
    main_controller.arm_controller.reach_default()
    poses = main_controller.get_poses()
    main_controller.start_primitive('pickdrop', target = [main_controller.get_block_pose(), np.array([-1.51674434, -0.11124739, 0.81568371])])
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished pickdrop")

def test_pickup_dropoff_cup_controller_thread(main_controller, groundings: Groundings) -> None:
    # Setup
    # main_controller = MainController(objects_list = ['plate', 'cup', 'cone', 'block1'])
    rospy.loginfo('Reaching default pose')
    main_controller.arm_controller.reach_default()
    poses = main_controller.get_poses()
    x4_midpt = [-0.945, 0]
    x0_midpt = [1.12, 0.18]
    cup_target = np.array([1.79, 0, 0.84])

    # Monitor controllable inputs
    grounding_thread = threading.Thread(target=test_groundings, args=(groundings,))
    groundings.reset_states()
    grounding_thread.start()

    # Pick up and drop off cup
    rospy.loginfo('Aligning to the cup')
    main_controller.start_primitive('movebase', target = [x4_midpt])
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")
    
    # rospy.loginfo('Picking up the cup')
    main_controller.start_primitive('pickup', target = main_controller.get_cup_pose())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished picking up the cup")

    # rospy.loginfo('Aligning to the cup dropoff location')
    main_controller.start_primitive('movebase', target = [x0_midpt])
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    # rospy.loginfo('Dropping off the cup')
    main_controller.start_primitive('dropoff', target = cup_target)
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished dropping off the cup")

def test_poses(controller: MainController):
    while not rospy.is_shutdown():
        poses = controller.get_poses()
        print('Poses: ')
        for key, val in poses.items():
            print(f'{key}: {val}')
        print('===========')
        rospy.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files_json', type=str, default='inputs/empty_cup/files.json', help='json file containing all json files')
    parser.add_argument('--uncontrollable_variables', nargs='*', default=[], help='list of uncontrollable variables')
    args = parser.parse_args()
    files_json = args.files_json
    uncontrollable_variables = args.uncontrollable_variables

    # rospy.init_node('groundings')
    CREATE_SYMBOLS_ONLY = True
    if not CREATE_SYMBOLS_ONLY:
        controller = MainController(objects_list = ['cup', 'cone', 'block', 'plate'])
    else:
        controller = None
    # test_pickup_dropoff_plate_cup(files_json)
    files = json_load_wrapper(files_json)
    objects_data = json_load_wrapper(files['objects'])
    locations_data = json_load_wrapper(files['locations'])
    user_inputs_data = json_load_wrapper(files['user_inputs'])
    groundings = Groundings(files_json, objects_data, locations_data, user_inputs_data, controller)
    # groundings.start()
    # rospy.sleep(3)
    # rospy.spin()
    # test_pickup_dropoff_plate_cup(controller, groundings)
    # test_pickup_dropoff_cup_controller_thread(controller, groundings)
    test_groundings(groundings)
    # test_poses(controller)
    # test_pickdrop_block_thread(controller, groundings)


    
