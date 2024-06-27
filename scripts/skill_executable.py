#! /usr/bin/env python

import argparse
import numpy as np
import rospy
from grounding import Groundings
from roadmap import RoadmapMobile, RoadmapManipulation

from tools import (
    json_load_wrapper,
    select_midpoint_in_bounds,
    MobileTreeNode, 
    ManipulationTreeNodeNoEE,
    print_mobile_tree,
    print_manipulation_tree_no_ee,
    dict_bool2list,
    find_true_robot_mobile_symbols,
    find_true_robot_manipulation_symbols,
    symbol_to_location,
    find_loc_from_true_manipulation_symbols_no_ee,
    find_changing_object)

from env_repair.src.controller.main_controller import MainController

DEBUG = False

class IntermediateStateExecutable:
    """Executable for an intermediate state"""
    def __init__(self, pre_dict: dict, post_dict_list: list, controller: MainController, skill_data: dict, symbols_data: dict, n_dims: int):
        """Initialize the executable for an intermediate state
        
        Args:
            pre_dict: dict: pre state
            post_dict_list: list: list of post states
            controller: MainController: the controller providing the truth pose of the robot
            skill_data: dict: data of the skill
            symbols_data: dict: data of the symbols
            n_dims: int: number of dimensions
        """
        self.pre_dict = pre_dict
        self.post_dict_list = post_dict_list
        self.controller = controller
        self.skill_data = skill_data
        # self.num_original_skills = len(self.skill_data.key)
        self.n_dims = n_dims
        self.primitive_skill_name = self.skill_data["primitive_skill"] if "primitive_skill" in self.skill_data.keys() else None
        self.symbols_data = symbols_data
        # self.set_executable()

    def set_executable(self) -> None:
        """Set the executable for an intermediate state, 
           which is a main controller call to the primitive skill
        """
        self.primitive_skill = self.controller.get_primitive_skills()[self.primitive_skill_name]
        target_pose = self.get_target_pose()
        self.executable = lambda: self.primitive_skill(target = target_pose)
    
    def get_target_pose(self) -> np.ndarray:
        """Get the target pose of the primitive skill"""
        post_dict = self.select_post_dict(self.post_dict_list)
        if self.skill_data["goal_type"] == "region":
            bounds = self.get_bounds(post_dict)
            target_pose = select_midpoint_in_bounds(bounds, self.n_dims)
            if self.skill_data["type"] == "mobile":
                target_pose = target_pose[:2]
        elif self.skill_data["goal_type"] == "object":
            for symbol, value in post_dict.items():
                if value:
                    symbol_info = self.symbols_data[symbol]
                    if symbol_info["location"] == "ee":
                        return self.controller.get_3Dposes()[symbol_info["object"]]
        else:
            raise Exception(f"Unknown goal type {self.skill_data['goal_type']} for skill {self.skill_data['name']}")
        return target_pose
    
    # Getter functions
    def get_pre_dict(self) -> dict:
        return self.pre_dict
    
    def get_post_dict_list(self) -> list:
        return self.post_dict_list
    
    def get_executable(self) -> callable:
        return self.executable


    def select_post_dict(self, post_dict_list: list) -> dict:
        # TODO: add support for multiple post-conditions
        return post_dict_list[0]
    
    def get_bounds(self, post_dict: dict) -> np.ndarray:
        """Get the bounds of the region specified in the post state"""
        bounds = []
        for _ in range(self.n_dims):
            bounds.append([-np.inf, np.inf])
        for symbol, value in post_dict.items():
            if value:
                symbol_info = self.symbols_data[symbol]
                print(symbol_info)
                for dim in symbol_info["dims"]:
                    bounds[dim][0] = max(bounds[dim][0], symbol_info["bounds"][dim][0])
                    bounds[dim][1] = min(bounds[dim][1], symbol_info["bounds"][dim][1])
        return bounds
        

class SkillExecutable:
    """Executable for a skill"""
    def __init__(self, skill_data: dict, controller: MainController, objects_data: dict, locations_data: dict, symbols_data: dict, skills_data: dict, n_dims: int, roadmap_mobile: RoadmapMobile, roadmap_manipulation: RoadmapManipulation, is_original_skill: bool):
        """Initialize the executable for a skill
        
        Args:
            skill_data: dict: data of the skill
            controller: MainController: the controller providing the truth pose of the robot
            symbols_data: dict: data of the symbols
            n_dims: int: number of dimensions
        """
        self.skill_data = skill_data
        self.controller = controller
        self.objects_data = objects_data
        self.locations_data = locations_data
        self.symbols_data = symbols_data
        self.skills_data = skills_data
        self.n_dims = n_dims
        self.roadmap_mobile = roadmap_mobile
        self.roadmap_manipulation = roadmap_manipulation
        self.is_original_skill = is_original_skill

        ## ==== Set the intermediate states executable ==== ##
        self.intermediate_states_executable: list[IntermediateStateExecutable] = []
        for pre_dict, post_dict_list in self.skill_data['intermediate_states']:
            self.intermediate_states_executable.append(
                IntermediateStateExecutable(pre_dict, 
                                            post_dict_list, 
                                            self.controller, 
                                            self.skill_data, 
                                            self.symbols_data,
                                            self.n_dims))
        
        ## ==== Set the executable for the skill, if the skill is not an original skill ==== ##
        self.set_path()

    def set_original_skill_path(self) -> None:
        """Set the path for an original skill"""
        ## ==== Get target points ==== ##
        if 'object' in self.skill_data['goal_type'] and 'region' in self.skill_data['goal_type']:
            object_target = target = self.controller.get_3Dposes()[self.symbols_data[self.skill_data["goal"]]["object"]]
            region_target_bounds = self.symbols_data[self.skill_data["goal"]]["bounds"]
            region_target = select_midpoint_in_bounds(region_target_bounds, self.n_dims)
            target = [object_target, region_target]
        elif self.skill_data['goal_type'] == 'region':
            target_bounds = self.symbols_data[self.skill_data["goal"]]["bounds"]
            # target_point = select_midpoint_in_bounds(target_bounds, self.n_dims)
            target = select_midpoint_in_bounds(target_bounds, self.n_dims)
            if self.skill_data['type'] == 'mobile':
                target= [target[:2]]
        elif self.skill_data['goal_type'] == 'object':
            target = self.controller.get_3Dposes()[self.symbols_data[self.skill_data["goal"]]["object"]]
        else:
            raise Exception(f"Unknown goal type {self.skill_data['goal_type']} for skill {self.skill_data['name']}")
        self.path = target
        return None

    def set_path(self) -> None:
        """Set the path for a new skill"""
        if self.skill_data["type"] == "mobile":
            self.set_mobile_skill_path()
        elif self.skill_data["type"] == "manipulation":
            self.set_manipulation_skill_path()
        else:
            raise Exception(f"Unknown skill type {self.skill_data['type']} for skill {self.skill_data['name']}")
        return None

    def set_mobile_skill_path(self) -> None:
        """Set the path for a new mobile skill"""
        roots: list[MobileTreeNode] = self._build_mobile_tree()
        if True:
            for root in roots:
                print_mobile_tree(root)
        self.paths = {}
        for root in roots:
            self.paths[root.location] = self._build_mobile_path_from_mobile_tree(root)
        if True:
            print("paths: ", self.paths)

    def _build_mobile_tree(self) -> list:
        """Given the intermediate states of the new skills,
           build a tree connecting the intermediate states
        """
        ## ==== Build the tree ==== ##
        roots = []
        for initial_pre in self.skill_data["initial_preconditions"]:
            roots.append(self._build_mobile_tree_helper(initial_pre))
        return roots
    
    def _build_mobile_tree_helper(self, pre_dict: dict) -> MobileTreeNode:
        """Helper function for building the tree"""
        node = MobileTreeNode(pre_dict=pre_dict, symbols_data=self.symbols_data, objects_data=self.objects_data, locations_data=self.locations_data)
        for intermediate_state_executable in self.intermediate_states_executable:
            if pre_dict == intermediate_state_executable.get_pre_dict():
                post_dict_list = intermediate_state_executable.get_post_dict_list()
                for post_dict in post_dict_list:
                    node.add_child(self._build_mobile_tree_helper(post_dict))
        return node
    
    def _build_mobile_path_from_mobile_tree(self, root: MobileTreeNode) -> list:
        """Given the tree, build the path"""
        ## ==== Build the path ==== ##
        ## Base case: leaf node
        if root.children == []:
            return [self.roadmap_mobile.get_node_pos(root.location)]

        ## Inductive case: internal node
        for child in root.children:
            child_path = self._build_mobile_path_from_mobile_tree(child)
            if child_path is not None: 
                edge_to_child_attr = self.roadmap_mobile.get_edge_attributes(root.location, child.location)
                if edge_to_child_attr is not None:
                    return [self.roadmap_mobile.get_node_pos(root.location), edge_to_child_attr['midpt']] + child_path
        
        # No children can be connected
        return None
        
    def set_manipulation_skill_path(self) -> None:
        """Set the path for a new manipulation skill
        Types of manipulation skill:
            1. pickup: align base, open gripper, reach point, close gripper, reach default
            2. dropoff: align base, reach point, open gripper, reach default
        """
        # self.object: str = symbol_to_object(self.skill_data["goal"])
        self.object : str = find_changing_object(self.skill_data["intermediate_states"], self.objects_data)
        roots: list[ManipulationTreeNodeNoEE] = self._build_manipulation_tree()
        if True:
            for root in roots:
                print_manipulation_tree_no_ee(root)
        self.paths = {}
        for root in roots:
            self.paths[root.symbol] = self._build_manipulation_path_from_manipulation_tree(root)
        if True:
            print("paths: ", self.paths)

    def _build_manipulation_tree(self) -> list:
        """Given the intermediate states of the new skills,
           build a tree connecting the intermediate states
        """
        ## ==== Build the tree ==== ##
        roots = []
        for initial_pre in self.skill_data["initial_preconditions"]:
            roots.append(self._build_manipulation_tree_helper(initial_pre))
        return roots
    
    def _build_manipulation_tree_helper(self, pre_dict: dict) -> ManipulationTreeNodeNoEE:
        """Helper function for building the tree"""
        node = ManipulationTreeNodeNoEE(pre_dict=pre_dict, symbols_data=self.symbols_data, objects_data=self.objects_data, locations_data=self.locations_data, object=self.object)
        for intermediate_state_executable in self.intermediate_states_executable:
            if pre_dict == intermediate_state_executable.get_pre_dict():
                post_dict_list = intermediate_state_executable.get_post_dict_list()
                for post_dict in post_dict_list:
                    node.add_child(self._build_manipulation_tree_helper(post_dict))
        return node
    
    def _build_manipulation_path_from_manipulation_tree(self, root: ManipulationTreeNodeNoEE) -> list:
        """Given the manipulation tree, build the path"""
        ## Base case: leaf node:
        if root.children == []:
            return [root.symbol]
        
        ## Inductive case: internal node
        for child in root.children:
            if DEBUG: breakpoint()
            child_path = self._build_manipulation_path_from_manipulation_tree(child)
            if child_path is not None:
                return [root.symbol] + child_path
        
        ## No children can be connected
        return None

    def get_intermediate_state_executable(self, X: dict) -> IntermediateStateExecutable:
        """Select the executable for an intermediate state based on the inputs X"""
        for intermediate_state_executable in self.intermediate_states_executable:
            pre_dict = intermediate_state_executable.get_pre_dict()
            if all([X[input] == value for input, value in pre_dict.items()]):
                return intermediate_state_executable
        raise Exception("No intermediate state executable found for the inputs X: {}".format(X))
    
    def execute(self, X: dict) -> None:
        """Execute the skill based on the inputs X
        
        Args:
            X: dict[str, bool]: state of inputs
        """
        ## ==== Execute the original skills ==== ##
        rospy.loginfo(f"Executing skill {self.skill_data['name']}")
        if DEBUG:
            print(self.skill_data)

        # Set the path based on X
        if self.skill_data["type"] == "mobile":
            x = self.get_mobile_path_key(X)
            if x is not None: 
                if x not in self.paths.keys():
                    raise Exception(f"Path {x} not found for skill {self.skill_data['name']}")
                self.path = self.paths[x]

        elif self.skill_data["type"] == "manipulation":
            if DEBUG: breakpoint()
            x = self.get_manipulation_path_key(X)
            if x is not None:
                if x not in self.paths.keys():
                    raise Exception(f"Path {x} not found for skill {self.skill_data['name']}")
                symbolic_path = self.paths[x]
            else:
                raise Exception(f"Path {x} not found for skill {self.skill_data['name']}")
            
            assert(len(symbolic_path) >= 2)
            if DEBUG: breakpoint()
            if len(symbolic_path) == 2:
                if symbol_to_location(symbolic_path[0]) == 'ee' and symbol_to_location(symbolic_path[1]) != 'ee':
                    self.skill_data['primitive_skill'] = 'dropoff'
                    loc = symbol_to_location(symbolic_path[1])
                    self.path = select_midpoint_in_bounds(bounds=self.locations_data[loc]["bounds"], n_dims=3)
                elif symbol_to_location(symbolic_path[0]) != 'ee' and symbol_to_location(symbolic_path[1]) == 'ee':
                    self.skill_data['primitive_skill'] = 'pickup'
                    self.path = self.controller.get_3Dposes()[self.object]
            
            if len(symbolic_path) >= 2:
                if all([symbol_to_location(symbol) != 'ee' for symbol in symbolic_path]):
                    self.skill_data['primitive_skill'] = 'moveobj'
                    self.path = [self.controller.get_3Dposes()[self.object]]
                    for symbol in symbolic_path[1:]:
                        loc = symbol_to_location(symbol)
                        self.path.append(select_midpoint_in_bounds(bounds=self.locations_data[loc]["bounds"], n_dims=3))

        else:
            raise Exception(f"Unknown skill type {self.skill_data['type']} for skill {self.skill_data['name']}")
        
        if True:
            print("Skill: ", self.skill_data['name'])
            print("Path: ", self.path)
        self.controller.start_primitive(primitive=self.skill_data["primitive_skill"], target=self.path)

    def get_mobile_path_key(self, X: dict) -> str:
        for initial_pre in self.skill_data["initial_preconditions"]:
            if all([X[input] == value for input, value in initial_pre.items()]):
                if self.skill_data['type'] == 'mobile':
                    return self.symbols_data[find_true_robot_mobile_symbols(X=X, 
                                                        symbols_data=self.symbols_data, 
                                                        objects_data=self.objects_data, 
                                                        locations_data=self.locations_data)[0]]['location']
        raise Exception("No relevant true symbol found for the inputs X: {}".format(dict_bool2list(X)))
    
    def get_manipulation_path_key(self, X: dict) -> list:
        """Get the root of the path"""
        for initial_pre in self.skill_data["initial_preconditions"]:
            if all([X[input] == value for input, value in initial_pre.items()]):
                if self.skill_data['type'] == 'manipulation':
                    return find_true_robot_manipulation_symbols(X=X, 
                                                        symbols_data=self.symbols_data, 
                                                        objects_data=self.objects_data, 
                                                        locations_data=self.locations_data,
                                                        object=self.object)[0]
        raise Exception("No relevant true symbol found for the inputs X: {}".format(dict_bool2list(X)))
        
class SkillsExecutable:
    """Executable for skills"""
    def __init__(self, files_json: str, skills_data: dict, symbols_data: dict, objects_data: dict, locations_data: dict, opts: dict, controller: MainController):
        """Initialize the executable for skills
        
        Args:
            files_json: str: json file containing all json files
            controller: MainController: the controller providing the truth pose of the robot
            uncontrollable_variables: list: list of uncontrollable variables
        """
        self.files_data = json_load_wrapper(files_json)
        self.skills_data = skills_data
        # self.original_skills_names = [skill_name for skill_name in self.skills_data.keys()]
        self.symbols_data = symbols_data
        self.objects_data = objects_data
        self.locations_data = locations_data
        # self.locations_data = json_load_wrapper(self.files_data['locations'])
        self.roadmap_mobile = RoadmapMobile(files_json_name=files_json)
        self.roadmap_manipulation = RoadmapManipulation(files_json_name=files_json)
        self.opts = opts
        self.n_dims = self.opts['n_dims']
        self.controller = controller

        ## ==== Set the executables for the skills ==== ##
        self.skills_executable_dict: dict[str, SkillExecutable] = {}
        self.skills_status = dict()
        for skill_name, skill_data in self.skills_data.items():
            self.skills_executable_dict[skill_name] = SkillExecutable(skill_data=skill_data, 
                                                                      controller=self.controller, 
                                                                      objects_data=self.objects_data,
                                                                      locations_data=self.locations_data,
                                                                      symbols_data=self.symbols_data,
                                                                      skills_data=self.skills_data,
                                                                      n_dims=self.n_dims, 
                                                                      roadmap_mobile=self.roadmap_mobile,
                                                                      roadmap_manipulation=self.roadmap_manipulation,
                                                                      is_original_skill=True,)
            self.skills_status[skill_name] = False
    
    def reset_skills_status(self) -> None:
        """Reset the status of skills"""
        for skill_name in self.skills_status.keys():
            self.skills_status[skill_name] = False

    # def is_original_skill(self, skill_name: str) -> bool:
    #     """Check if the skill is an original skill"""
    #     return skill_name in self.original_skills_names

    def turn_on_skill(self, skill_name: str) -> None:
        """Turn on a skill"""
        self.reset_skills_status()
        self.skills_status[skill_name] = True

    def get_executable_dict(self) -> dict:
        """Get the executable for the skills"""
        return self.skills_executable_dict
    
    def execute_skill(self, skill_name: str, X: dict) -> None:
        """Execute a skill based on the inputs X
        
        Args:
            skill_name: str: name of the skill
            X: dict[str, bool]: state of inputs
        """
        if skill_name is None:
            print("No skill to execute")
            self.controller.shutdown()
            self.reset_skills_status()
            # self.controller.clear_shutdown()
            print("Controller is alive: ", self.controller.is_alive())
            return None
        print(f'Executing {skill_name}')
        if True:
            print(f"Is controller shutdown: {self.controller.shutdown_event.is_set()}")
        assert skill_name in self.skills_executable_dict.keys(), f"Skill {skill_name} not found"
        if self.skills_status[skill_name]:
            print(f"Skill {skill_name} is already on")
            return None
        skill_executable = self.skills_executable_dict[skill_name]
        skill_executable.execute(X)
        self.turn_on_skill(skill_name)

    def add_skills_executable(self, new_skills: dict) -> None:
        """Add skills executable to the skills executable dict
        
        Args:
            new_skills: dict[str, dict]: dict of new skills
        """
        for skill_name, skill in new_skills.items():
            if skill_name not in self.skills_executable_dict.keys():
                if True:
                    print(f"Adding skill {skill_name}")
                    print(f"Skill data: {skill}")
                self.skills_executable_dict[skill_name] = SkillExecutable(skill_data=skill, 
                                                                          controller=self.controller, 
                                                                          objects_data=self.objects_data,
                                                                          locations_data=self.locations_data,
                                                                          symbols_data=self.symbols_data,
                                                                          skills_data=self.skills_data,
                                                                          n_dims=self.n_dims,
                                                                          roadmap_mobile=self.roadmap_mobile, 
                                                                          roadmap_manipulation=self.roadmap_manipulation,
                                                                          is_original_skill=False)
                self.skills_status[skill_name] = False
        self.reset_skills_status()
        return None
    
    def check_physical_implementability(self, new_skills: dict) -> list:
        """Check if the new skills are physically implementable
        
        Args:
            new_skills: dict[str, dict]: dict of new skills
        
        Returns:
            bad_transitions: list: list of intermediate transitions that are not physically implementable
            bad_skills: list: list of skills that are not physically implementable
        """
        if DEBUG: 
            print("Checking physical implementability")
            breakpoint()
        bad_transitions = []
        bad_skills = set()
        for skill_name, skill in new_skills.items():
            if skill["type"] == 'mobile':
                for pre_dict, post_dict_list in skill["intermediate_states"]:
                    for post_dict in post_dict_list:
                        if not self.check_mobile_intermediate_transition_physical_implementability(pre_dict, post_dict):
                            bad_transitions.append((pre_dict, post_dict))
                            bad_skills.add(skill_name)
            elif skill["type"] == 'manipulation':
                # return []
                for pre_dict, post_dict_list in skill["intermediate_states"]:
                    for post_dict in post_dict_list:
                        if not self.check_manipulation_intermediate_transition_physical_implementability(pre_dict, post_dict, object=find_changing_object(skill["intermediate_states"], self.objects_data)):
                            bad_transitions.append((pre_dict, post_dict))
                            bad_skills.add(skill_name)
            else:
                raise Exception(f"Unknown skill type {skill['type']} for skill {skill['name']}")
        return bad_transitions, list(bad_skills)
    
    def check_mobile_intermediate_transition_physical_implementability(self, pre_dict: dict, post_dict: dict) -> bool:
        """Check if the mobile intermediate transition is physically implementable"""
        pre_location = self.symbols_data[find_true_robot_mobile_symbols(X=pre_dict, 
                                                      symbols_data=self.symbols_data, 
                                                      objects_data=self.objects_data, 
                                                      locations_data=self.locations_data)[0]]['location']
        post_location = self.symbols_data[find_true_robot_mobile_symbols(X=post_dict, 
                                                      symbols_data=self.symbols_data, 
                                                      objects_data=self.objects_data, 
                                                      locations_data=self.locations_data)[0]]['location']
        if pre_location == post_location:
            return True
        return self.roadmap_mobile.get_edge_attributes(pre_location, post_location) is not None

    def check_manipulation_intermediate_transition_physical_implementability(self, pre_dict: dict, post_dict: dict, object: str) -> bool:
        """Check if the manipulation intermediate transition is physically implementable"""
        pre_location = find_loc_from_true_manipulation_symbols_no_ee(obj=object, true_symbols=find_true_robot_manipulation_symbols(X=pre_dict, 
                                                                                                                                   symbols_data=self.symbols_data, 
                                                                                                                                   objects_data=self.objects_data, 
                                                                                                                                   locations_data=self.locations_data,
                                                                                                                                   object=object))
        post_location = find_loc_from_true_manipulation_symbols_no_ee(obj=object, true_symbols=find_true_robot_manipulation_symbols(X=post_dict, 
                                                                                                                                   symbols_data=self.symbols_data, 
                                                                                                                                   objects_data=self.objects_data, 
                                                                                                                                   locations_data=self.locations_data,
                                                                                                                                   object=object))
        if pre_location == post_location or pre_location == 'ee' or post_location == 'ee':
            return True
        return self.roadmap_manipulation.get_edge_attributes(pre_location, post_location) is not None

def test_skills_executable(skills_executable: SkillsExecutable):
    """Test the skills executable"""
    print("Testing skills executable")
    groundings = Groundings(files_json, controller)
    # X = {
    #     "p_base_x0": False,
    #     "p_base_x1": True,
    #     "p_base_x2": False,
    #     "p_base_x3": False,
    #     "p_base_x4": False,
    #     "p_base_x5": False,
    # }
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill0", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill0", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill3", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill1", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill1", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill5", X)
    X = groundings.get_state()
    skills_executable.execute_skill("skill0", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill0", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill4", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill2", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill2", X)
    groundings.monitor_state()
    X = groundings.get_state()
    skills_executable.execute_skill("skill6", X)

def test_skill_executable_new(skills_executable: SkillsExecutable, groundings: Groundings, main_controller: MainController):
    # Setup
    # main_controller = MainController(objects_list = ['plate', 'cup', 'cone', 'block1'])
    rospy.loginfo('Reaching default pose')
    main_controller.arm_controller.reach_default()
    poses = main_controller.get_poses()
    groundings.start()

    # Test skill0
    rospy.loginfo('Testing skill0')
    skills_executable.execute_skill("skill0", groundings.get_inputs())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    # Test skill2
    rospy.loginfo('Testing skill2')
    skills_executable.execute_skill("skill2", groundings.get_inputs())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    # Test skill1
    rospy.loginfo('Testing skill1')
    skills_executable.execute_skill("skill1", groundings.get_inputs())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    # Test skill3
    rospy.loginfo('Testing skill3')
    skills_executable.execute_skill("skill3", groundings.get_inputs())
    while not rospy.is_shutdown() and not main_controller.shutdown_event.is_set():
        main_controller.rate.sleep()
    rospy.loginfo("Finished moving the base")

    groundings.shutdown()

def test_add_new_skill(files_json: str, skills_data: dict, objects_data, locations_data, symbols_data: dict, opts, controller: MainController):
    skills_executable = SkillsExecutable(files_json, skills_data, symbols_data, objects_data, locations_data, opts, controller)
    new_skills = {"skill8": {"name": "skill8", "primitive_skill": "movebase",
        "type": "mobile",
        "goal_type": "region",
        "intermediate_states": [[{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}, 
[
{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': False, 'p_base_x4': False, 'p_base_x0': True, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}, 
{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': True, 'p_base_x4': False, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}]], 

[{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': False, 'p_base_x4': False, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': True, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}, 
[
{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': True, 'p_base_x4': False, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}]],

[{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': True, 'p_base_x4': False, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}, 
[
{'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': False, 'p_base_x4': False, 'p_base_x0': True, 'p_base_x2': False, 'p_base_x3': False, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}]]]
,
        "initial_preconditions": [
            {
                "p_base_x4": True,
                "p_ee_else": True,
                "p_cup_ee": True,
                "p_cup_t": False,
                "p_cup_k": False,
                "p_cup_x0": False,
                "p_cup_x1": False,
                "p_cup_x2": False,
                "p_cup_x3": False,
                "p_cup_x4": False,
                "p_base_x0": False,
                "p_base_x1": False,
                "p_base_x2": False,
                "p_base_x3": False,
                "p_ee_t": False,
                "p_ee_k": False,
                "p_ee_x0": False,
                "p_ee_x1": False,
                "p_ee_x2": False,
                "p_ee_x3": False,
                "p_ee_x4": False
                },
                {'p_cup_x0': False, 'p_cup_x1': False, 'p_cup_x2': False, 'p_cup_x3': False, 'p_cup_x4': False, 'p_cup_k': False, 'p_cup_t': False, 'p_cup_ee': True, 'p_base_x1': False, 'p_base_x4': False, 'p_base_x0': False, 'p_base_x2': False, 'p_base_x3': True, 'p_ee_else': True, 'p_ee_t': False, 'p_ee_x1': False, 'p_ee_k': False, 'p_ee_x0': False, 'p_ee_x2': False, 'p_ee_x3': False, 'p_ee_x4': False}
        ],
        "final_postconditions": [
            {
                "p_base_x0": True,
                "p_ee_else": True,
                "p_cup_ee": True,
                "p_cup_t": False,
                "p_cup_k": False,
                "p_cup_x0": False,
                "p_cup_x1": False,
                "p_cup_x2": False,
                "p_cup_x3": False,
                "p_cup_x4": False,
                "p_base_x1": False,
                "p_base_x2": False,
                "p_base_x3": False,
                "p_base_x4": False,
                "p_ee_t": False,
                "p_ee_k": False,
                "p_ee_x0": False,
                "p_ee_x1": False,
                "p_ee_x2": False,
                "p_ee_x3": False,
                "p_ee_x4": False
                }
        ]}}
    skills_executable.add_skills_executable(new_skills)

def test_add_new_skills_manipulation(files_json: str, skills_data: dict, objects_data, locations_data, symbols_data: dict, opts, controller: MainController):
    skills_executable = SkillsExecutable(files_json, skills_data, symbols_data, objects_data, locations_data, opts, controller)
    new_skills = {
        "skill3": {'name': 'skill3', 'intermediate_states': [[{'p_block_k0': True, 'p_block_k1': False, 'p_block_k3': False, 'p_ee_k3': False, 'p_ee_k0': True, 'p_cup_k3': False, 'p_cup_k0': False, 'p_block_k2': False, 'p_block_ee': True, 'p_ee_k1': False, 'p_ee_else': False, 'p_ee_k2': False, 'p_cup_ee': False, 'p_cup_k2': True, 'p_cup_k1': False}, [{'p_block_k0': True, 'p_block_k1': False, 'p_block_k3': False, 'p_ee_k3': False, 'p_ee_k0': False, 'p_cup_k3': False, 'p_cup_k0': False, 'p_block_k2': False, 'p_block_ee': False, 'p_ee_k1': False, 'p_ee_else': True, 'p_ee_k2': False, 'p_cup_ee': False, 'p_cup_k2': True, 'p_cup_k1': False}]]], 'initial_preconditions': [{'p_block_k0': True, 'p_block_k1': False, 'p_block_k3': False, 'p_ee_k3': False, 'p_ee_k0': True, 'p_cup_k3': False, 'p_cup_k0': False, 'p_block_k2': False, 'p_block_ee': True, 'p_ee_k1': False, 'p_ee_else': False, 'p_ee_k2': False, 'p_cup_ee': False, 'p_cup_k2': True, 'p_cup_k1': False}], 'final_postconditions': [{'p_block_k0': True, 'p_block_k1': False, 'p_block_k3': False, 'p_ee_k3': False, 'p_ee_k0': False, 'p_cup_k3': False, 'p_cup_k0': False, 'p_block_k2': False, 'p_block_ee': False, 'p_ee_k1': False, 'p_ee_else': True, 'p_ee_k2': False, 'p_cup_ee': False, 'p_cup_k2': True, 'p_cup_k1': False}], 'original_skill': 'skill1', 'primitive_skill': 'dropoff', 'type': 'manipulation', 'goal_type': 'region', 'goal': 'p_block_k3'}
    }
    skills_executable.add_skills_executable(new_skills)

def test_check_manipulation_implementability(files_json: str, skills_data: dict, objects_data, locations_data, symbols_data: dict, opts, controller: MainController): 
    skills_executable = SkillsExecutable(files_json, skills_data, symbols_data, objects_data, locations_data, opts, controller)
    new_skills = {
        "skill12": {'name': 'skill12', 'type': 'manipulation', 'new_skill': {'skill0': False, 'skill1': False, 'skill1backup': False, 'skill5': False, 'skill5backup': False, 'skill4backup': False, 'skill4': False, 'skill2': True, 'skill2backup': False, 'skill3': False, 'skill3backup': False, 'skill0backup': False}, 'original_skill': {'skill0': False, 'skill1': False, 'skill1backup': False, 'skill5': False, 'skill5backup': False, 'skill4backup': False, 'skill4': False, 'skill2': True, 'skill2backup': False, 'skill3': False, 'skill3backup': False, 'skill0backup': False}, 'intermediate_states_all_pres': [], 'intermediate_states': [[{'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': True, 'p_cup_t': False, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}, [{'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': False, 'p_cup_t': True, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}]]], 'final_postconditions': [{'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': False, 'p_cup_t': True, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}], 'initial_preconditions': [{'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': True, 'p_cup_t': False, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}], 'unique_states': [{'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': False, 'p_cup_t': True, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}, {'p_base_x3': False, 'p_base_x0': False, 'p_base_x1': False, 'p_base_x4': True, 'p_base_x2': False, 'p_cup_ee': False, 'p_block_ee': False, 'p_block_k1': True, 'p_cup_k3': False, 'p_block_k3': False, 'p_cup_k1': False, 'p_cup_k2': True, 'p_cup_t': False, 'p_cup_k0': False, 'p_block_k0': False, 'p_block_t': False, 'p_block_k2': False}], 'swapped': [], 'avoid_states': [], 'folder_train': '', 'folder_val': '', 'suggestion': True}}
    res = skills_executable.check_physical_implementability(new_skills)
    print("infeasible transitions: ", res)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files_json', type=str, default='inputs/pickup_dropoff_cup/files.json', help='json file containing all json files')
    parser.add_argument('--uncontrollable_variables', nargs='*', default=[], help='list of uncontrollable variables')
    args = parser.parse_args()
    files_json = args.files_json
    files = json_load_wrapper(files_json)
    skills_data = json_load_wrapper(files['skills'])
    symbols_data = json_load_wrapper(files['symbols'])
    objects_data = json_load_wrapper(files['objects'])
    locations_data = json_load_wrapper(files['locations'])
    opts = json_load_wrapper(files['opts'])
    # controller = MainController(objects_list = ['plate', 'cup', 'cone'])
    
    controller = MainController(objects_list = ['cup', 'block', 'plate', 'cone'])
    # test_pickup_dropoff_plate_cup(files_json)
    # groundings = Groundings(files_json=files_json, objects_data=objects_data, locations_data=locations_data, controller=controller)
    # skills_executable = SkillsExecutable(files_json, skills_data, symbols_data, opts, controller)
    # test_skills_executable(skills_executable)
    # test_skill_executable_new(skills_executable, groundings, controller)
    # test_add_new_skill(files_json, skills_data, objects_data, locations_data, symbols_data, opts, controller)
    # test_add_new_skills_manipulation(files_json, skills_data, objects_data, locations_data, symbols_data, opts, controller)
    test_check_manipulation_implementability(files_json, skills_data, objects_data, locations_data, symbols_data, opts, controller)