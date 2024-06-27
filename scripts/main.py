#!/usr/bin/env python
from skill_executable import SkillsExecutable
from grounding import Groundings
from spec_writer import SpecWriter
from monitor_compiler import Compiler
from repair import Repair

import sys
import os
import argparse
import rospy
import copy
import json
from importlib import reload

curr_dir = os.getcwd()

from .tools import (
    json_parser, 
    json_load_wrapper,
    make_build_dir,
    get_nonrobot_objects,
    list2dict_bool,
    dict_bool2list,
    wait_for_either_events,
    inputs_have_changed,
    controllable_inputs_have_changed,
    is_final_post,
    run_slugs,
    filter_new_skills,
    is_uncontrol_mani_obj_changed,
    create_build_monitor,
)

create_build_monitor()

from env_repair.src.controller.main_controller import MainController

import build.generated_monitor
from build.generated_monitor import Monitor

DEBUG = False
SYNTHESIS_ONLY = False

def print_debug(*text: str):
    """print_debug"""
    if DEBUG:
        print(text)

class Main:
    """
    The main function to run the framework
    """
    ## ==== Initialization ==== ##
    def __init__(self, files_json: str) -> None:
        make_build_dir(curr_dir)
        self.setup_files(files_json)
        self.initialize_submodules()
        self.inputs_data = json_load_wrapper(self.files['inputs'])
        self.relax_cnt = self.repair_cnt = 0
        self.epsilon_time = 0.5  # time to wait
        self.SYNTHESIS_ONLY = SYNTHESIS_ONLY

    def setup_files(self, files_json: str) -> None:
        """Setup all the files needed for the env relax & repair process"""
        self.files_json_name = files_json
        self.files = json_load_wrapper(files_json)
        self.objects_data = json_load_wrapper(self.files['objects'])
        self.locations_data = json_load_wrapper(self.files['locations'])
        self.user_inputs_data = json_load_wrapper(self.files['user_inputs'])
        self.skills_data = json_load_wrapper(self.files['skills'])
        self.input_file_structuredslugsplus = self.files['structuredslugsplus']
        self.repair_filename_structuredslugsplus = ""
        self.filename = os.path.splitext(self.input_file_structuredslugsplus)[
            0].split('/')[-1]
        self.slugsin = self.files['slugsin']
        self.opts = json_load_wrapper(self.files['opts'])

    def initialize_submodules(self) -> None:
        """Initialize all the submodules needed for the env relax & repair process"""
        self.controller = MainController(
            objects_list=get_nonrobot_objects(self.objects_data))
        if DEBUG:
            print("Initialized the controller")
        self.grounding = Groundings(
            self.files_json_name, self.objects_data, self.locations_data, self.user_inputs_data, self.controller)
        self.controller.arm_controller.reach_default()
        self.inputs_data = json_load_wrapper(self.files['inputs'])
        self.skills_executable = SkillsExecutable(files_json=self.files_json_name,
                                                  skills_data=self.skills_data,
                                                  symbols_data=self.inputs_data,
                                                  objects_data=self.objects_data,
                                                  locations_data=self.locations_data,
                                                  opts=self.opts,
                                                  controller=self.controller)
        self.spec_writer = SpecWriter(self.files_json_name, self.grounding)
        self.compiler = Compiler(input_file=self.input_file_structuredslugsplus,
                                 skills_data=self.skills_data,
                                 symbols_data=self.inputs_data,
                                 objects_data=self.objects_data,
                                 controllabe_variables=self.grounding.controllable_inputs,
                                 uncontrollable_variables=self.grounding.uncontrollable_inputs)

    ## ==== Main function ==== ##
    def main(self):
        """Main function to run the env relax & repair process"""
        self.X, self.Y, self.X_p = self.initialize_props()

        while not rospy.is_shutdown():
            self.reload_monitor()
            self.compiler.generate_slugsin(self.slugsin)
            self.strategy = self.produce_strategy()
            print("Strategy: ")
            print(self.strategy)
            if self.strategy is None:
                # spec unrealizable, repair should be finished
                # self.grounding.shutdown()
                # raise NotImplementedError
                self.X, self.Y, self.X_p = self.initialize_props()
                self.skills_executable.add_skills_executable(
                    filter_new_skills(self.skills_data, self.new_skills))
                continue
            self.execute_strategy(self.strategy)
        return None

    ## ==== Synthesize and run strategy ==== ##
    def initialize_props(self):
        # vars = self.compiler.get_vars()
        inputs = list2dict_bool(list(self.inputs_data.keys()))
        outputs = list2dict_bool(list(self.skills_data.keys()))
        if DEBUG:
            print("outputs: ", outputs.keys())
        return inputs, outputs, copy.deepcopy(inputs)

    def reload_monitor(self):
        self.compiler.generate_z3_monitor(
            f'{curr_dir}/build/generated_monitor.py')
        reload(build.generated_monitor)
        from build.generated_monitor import Monitor
        self.monitor = Monitor()

    def produce_strategy(self) -> dict:
        """Given a specification file, return the strategy produced by slug

        Args: None, use the slugsin file name specified in the constructor

        Returns:
            d: a dictionary representing the strategy produced by slug
        """
        result = run_slugs(self.slugsin)
        if "unrealizable" in result.stderr:
            print(result.stderr)
            if not self.SYNTHESIS_ONLY:
                print("Start the repair process...")
                self.new_skills = self.run_repair()
            else:
                print("Unrealizable spec")
                sys.exit(0)
            return None
        # print_debug("result.stdout: ", result.stdout)
        result_stdout = json_parser(result.stdout)
        print('Strategy generated')
        with open("build/strategy.json", 'w') as outfile:
            json.dump(result_stdout, outfile)
        return result_stdout

    def execute_strategy(self, strategy: dict) -> None:
        """Execute the strategy produced by slugs on robots

        Args:
            strategy: a dictionary representing the strategy produced by slug

        Returns:
            None

        Procedure:
            1. Initialization:
                1.1. Set the current node and the prev_skill
                1.2. Start the grounding monitor, wait for it to be ready, and get the initial state
            2. Iterate:
                2.1. Check if the current node in the strategy matches the current environment; if not resynthesize
                2.2. Get the current skill in the current node and execute it
                2.3. Wait for either the controller shutdown or the input state changes
                2.4. Clear the change event from the grounding, wait a little bit for the controller to shutdown 
                    2.4.1. If the the current stae is the final postcondition of the current skill, wait for the controller to shutdown, record the state

                2.5. If the controller is shutdown, then a skill is finished: wait for the controller thread to fully shutdown and record the next input state
                2.6. If the controller is not shutdown, then the controllable input state changes, only update the controllable inputs in the input state
                2.7. Use the monitor function to check if the assumption is violated; if so, call the assumption relaxation and exit the loop
                2.8. Update the current node and current input state
            3. Termination:
                3.1. Stop the grounding monitor
        """
        print("Executing strategy")
        # 1. Initialization
        # 1.1. Setup
        self.curr_node = '0'
        self.prev_skill = None

        # 1.2. Start the grounding monitor
        self.grounding.start()
        self.X = self.grounding.get_inputs()

        # 2. Iteration
        while not rospy.is_shutdown():
            # 2.1. Check if curr_node matches the input state X
            if not self.is_env_eq(self.curr_node, strategy['nodes'], self.X):
                self.process_env_unmatch()
                break

            # 2.2. Get the current skill in the current node and execute it
            print('Starting to execute node %s' % self.curr_node)
            skill_to_execute = self.get_skill_to_execute(
                strategy, self.curr_node, self.X, self.Y)
            self.skills_executable.execute_skill(skill_to_execute, self.X)

            # 2.3. Wait for either the controller shutdown or the controllable input state changes
            wait_for_either_events(
                [self.controller.shutdown_event, self.grounding.change_event])
            print("Controller shutdown: ", self.controller.shutdown_event.is_set(
            ), ", Grounding monitor event change: ", self.grounding.change_event.is_set())

            # 2.4. Clear the change event from the grounding, wait a little bit for the controller to shutdown
            self.grounding.clear_change_event()
            # self.grounding.start()
            if not self.controller.shutdown_event.is_set():
                if True:
                    print(
                        "Controller is not shutdown yet, wait for the controller to shutdown")
                # Wait for the controller to shut down, since the rate of the controller is slower than the grounding monitor
                rospy.sleep(self.epsilon_time)
                # 2.4.1. If the the current stae is the final postcondition of the current skill, wait for the controller to shutdown
                if is_final_post(self.grounding.get_inputs(), skill_to_execute, self.skills_data):
                    print(
                        f"Current state is the final postcondition of {skill_to_execute}, wait for the controller to shutdown")
                    self.controller.wait_for_shutdown()
                    rospy.loginfo(
                        "Controller shutdown in main, a skill is completed")

            # 2.5. If the controller is shutdown, then a skill is finished: wait for the controller thread to fully shutdown and record the next input state
            if self.controller.shutdown_event.is_set():
                # The controller shutdown means that the skill is completed, so we update uncontrollable inputs as well
                self.controller.wait_for_shutdown()
                rospy.loginfo(
                    "Controller shutdown in main function, a skill is completed")
                rospy.sleep(self.epsilon_time)
                self.X_p = self.grounding.get_inputs()
                self.flag_hard_assumptions_only = False

            else:
                # 2.6. If the controller is not shutdown, then the input state changes, update X_p
                rospy.sleep(self.epsilon_time)
                self.X_p_full = self.grounding.get_inputs()
                if inputs_have_changed(self.X, self.X_p_full):
                    rospy.loginfo(
                        "inputs have changed in main function")
                    print("X: ", dict_bool2list(self.X))
                    print("X_p: ", dict_bool2list(self.grounding.get_inputs()))
                    self.X_p = self.X_p_full
                    if controllable_inputs_have_changed(self.X, self.X_p, self.grounding.controllable_inputs):
                        self.flag_hard_assumptions_only = False
                    else:
                        self.flag_hard_assumptions_only = True
                else:
                    continue

            # 2.7. Use the monitor function to check if the assumption is violated; if so, call the assumption relaxation and exit the loop
            if self.monitor_assumption_has_violation(self.X, self.Y, self.X_p):
                self.controller.shutdown()
                self.process_violation(self.curr_node)
                break

            # 2.8. Update the current node and current input state
            self.curr_node = self.select_next_node(
                strategy, self.curr_node, self.X_p)
            self.X = copy.deepcopy(self.X_p)

        # 3. Termination
        self.grounding.shutdown()

    def is_env_eq(self, node: str, nodes: dict, X: dict) -> bool:
        """Return true if the environment of node equals to X"""
        state = nodes[node]['state']
        for i, value in enumerate(X.values()):
            if state[i] != value:
                return False
        return True

    def process_env_unmatch(self) -> None:
        """Process the case where the current environment does not match the environment of the current node"""
        print("Current environment state: ", self.X)
        print("Current pose: ", self.controller.get_3Dposes())
        print("Current environment does not match the environment of the current node, Resynthesis")
        # Change the env_init of AST to be the current state
        self.X = self.grounding.get_inputs()
        self.compiler.change_env_init(self.X)

    def get_skill_to_execute(self, strategy: dict, curr_node: str, inputs: dict, outputs: dict) -> str:
        """Returns the name of the skills in outputs to execute
           Set the executed skill in outputs (self.Y) to be true
        """
        skill_to_execute = None
        if DEBUG:
            print(f"skills_data: {self.skills_data.keys()}")
            print(f"inputs: {inputs.keys()}")
            print(f"state: {strategy['nodes'][curr_node]['state']}")
        for i in range(len(outputs.keys())):
            if strategy['nodes'][curr_node]['state'][len(inputs.keys()) + i] == 1:
                skill_to_execute = list(outputs.keys())[i]
                outputs[skill_to_execute] = True
            else:
                outputs[list(outputs.keys())[i]] = False
        if DEBUG:
            print(strategy['nodes'][curr_node]['state'][6])
            print('outputs: ', outputs)
            print('skill_to_execute: ', skill_to_execute)
        return skill_to_execute

    def select_next_node(self, strategy: dict, curr_node: str, X: dict) -> str:
        """Returns the next node in the strategy

        Args: 
            strategy: dict 
            curr_node: str
            X: dict: current environment after executing the skill

        Returns:
            next_node: str

        Procedure:
            go through all the next nodes from current node
            check whether their environment state is equivalent to the current environment
        """
        nodes = strategy['nodes']
        next_nodes = self.get_next_nodes(curr_node, strategy)  # list[str]
        print("curr_node: ", curr_node)
        print("next_nodes: ", next_nodes)
        for next_node in next_nodes:
            print("Processing node %s" % next_node)
            if True:
                print("X: ", dict_bool2list(X))
                print("state: ", nodes[next_node]['state'])
            if self.is_env_eq(next_node, nodes, X):
                return next_node
        self.grounding.shutdown()
        raise Exception(
            "No next node's environment matches the current environment")
        # return str(strategy['nodes'][curr_node]['trans'][0])

    def get_next_nodes(self, curr_node: str, strategy: dict) -> list:
        return list(map(str, strategy['nodes'][curr_node]['trans']))

    ## ==== Monitor and Relax ==== ##
    def monitor_assumption_has_violation(self, X: dict, Y: dict, X_prime: dict):
        if self.flag_hard_assumptions_only:
            return self.monitor_hard_assumption_has_violation_XYXPrime(X, Y, X_prime)
        else:
            return self.monitor_assumption_has_violation_XYXprime(X, Y, X_prime)
            # return self.monitor_assumption_has_violation_XYXprime_and_Xprime(X, Y, X_prime)

    def monitor_assumption_has_violation_XYXprime(self, X: dict, Y: dict, X_prime: dict):
        """Monitor the assumption and return True if there is a violation. 
           Check phi \land X \land Y \land next X' only"""
        results_XYXprime = self.monitor.monitor_postconditions(X, Y, X_prime)
        if self.monitor.no_violation(results_XYXprime):
            return False
        else:
            self.monitor.violated_assumptions_XYXprime = self.monitor.filter_violation(
                results_XYXprime)
            self.monitor.violated_assumptions_Xprime = []
            return True
    
    def monitor_hard_assumption_has_violation_XYXPrime(self, X: dict, Y: dict, X_prime: dict):
        """Monitor the hard assumption and return True if there is a violation.
              Check phi \land X \land Y \land next X'
        """
        results_XYXprime = self.monitor.monitor_postconditions(X, Y, X_prime)
        if self.monitor.no_violation_hard(results_XYXprime, len(self.compiler.get_env_trans_mutable_asts())):
            return False
        else:
            self.monitor.violated_assumptions_XYXprime = self.monitor.filter_violation_hard(
                results_XYXprime, 
                len(self.compiler.get_env_trans_mutable_asts()))
            self.monitor.violated_assumptions_Xprime = []
            return True

    def monitor_assumption_has_violation_XYXprime_and_Xprime(self, X: dict, Y: dict, X_prime: dict):
        """Monitor the assumption and return True if there is a violation
           Check phi \land X \land Y \land next X', and phi \land X'"""
        results_XYXprime = self.monitor.monitor_postconditions(X, Y, X_prime)
        results_Xprime = self.monitor.monitor_curr_input_state(X_prime)
        XYXprime_good = self.monitor.no_violation(results_XYXprime)
        Xprime_good = self.monitor.no_violation(results_Xprime)
        if XYXprime_good and Xprime_good:
            return False
        else:
            if not XYXprime_good:
                self.monitor.violated_assumptions_XYXprime = self.monitor.filter_violation(
                    results_XYXprime)
            else:
                self.monitor.violated_assumptions_XYXprime = []
            if not Xprime_good:
                self.monitor.violated_assumptions_Xprime = self.monitor.filter_violation(
                    results_Xprime)
            else:
                self.monitor.violated_assumptions_Xprime = []
            return True

    def process_violation(self, curr_node: str):
        print("Environment assumption is violated")
        # raise NotImplementedError
        # exit(0)
        # print("Expected state: ", self.strategy['nodes'][curr_node]['state'])
        print("X: ", dict_bool2list(self.X))
        print("Y: ", dict_bool2list(self.Y))
        print("X': ", dict_bool2list(self.X_p))
        print("Current poses: ", self.controller.get_3Dposes())
        print("Violated assumptions indices XYXprime: ",
              self.monitor.violated_assumptions_XYXprime)
        print("Violated assumptions indices Xprime: ",
              self.monitor.violated_assumptions_Xprime)
        if is_uncontrol_mani_obj_changed(self.X, self.X_p, self.objects_data, self.inputs_data):
            self.opts['mobile_repair'] = False
        else:
            self.opts['mobile_repair'] = True
        if True:
            print("========")
            print("Mobile repair?: ", self.opts['mobile_repair'])
            print("========")
        if DEBUG:
            print("Violated XYXprime assumptions: ")
            for index in self.monitor.violated_assumptions_XYXprime:
                print("==== Violated XYXprime Assumption index: ", index, " ====")
                if DEBUG:
                    print(self.monitor.formulas[index])
                print("========\n")
            for index in self.monitor.violated_assumptions_Xprime:
                print("==== Violated Xprime Assumption index: ", index, " ====")
                if DEBUG:
                    print(self.monitor.formulas[index])
                print("========\n")

        # Relax assumption
        self.compiler.relax_assumption(self.monitor.violated_assumptions_XYXprime,
                                       self.monitor.violated_assumptions_Xprime,
                                       copy.deepcopy(self.X), copy.deepcopy(self.Y), copy.deepcopy(self.X_p))

        while self.controller.is_alive():
            rospy.sleep(0.1)
        self.X = self.grounding.get_inputs()
        self.compiler.change_env_init(self.X)
        # exit(0)

        # Reorder the liveness goal
        self.compiler.reorder_liveness_goals(
            self.strategy['nodes'][curr_node]['rank'])

        # Generate files
        self.input_file_structuredslugsplus = f"build/relaxed_{self.filename}_{self.relax_cnt}.structuredslugsplus"
        self.relax_cnt += 1
        self.compiler.generate_structuredslugsplus(
            self.input_file_structuredslugsplus)

    ## ==== Repair ==== ##
    def run_repair(self) -> dict:
        """Run the repair module

        Returns:
            new_skills: dict: new skills generated by the repair module

        Procedure:
            1. Add backup skills to the compiler
            2. Update compiler's env_init to be the current state
            3. Generate the relaxed structuredslugsplus based on current ASTs
            4. Initialize and run the repair module and get the new skills
            5. Remove backup skills from the compiler
            6. Reduce the size of the new skills and add them to the compiler
            7. Edit the input_file and generate the structuredslugsplus and slugsin files
        """
        if DEBUG:
            breakpoint()
        # 0. Record the original spec file to be repaired
        self.X = self.grounding.get_inputs()
        self.compiler.change_env_init(self.X)
        self.compiler.change_not_allowed_repair_curr_inp(self.X, self.opts['mobile_repair'], self.objects_data, self.inputs_data)
        self.repair_filename_structuredslugsplus = f"repaired_{self.filename}_{self.relax_cnt-1}_{self.repair_cnt}.structuredslugsplus"
        input_file = f"build/original_{self.repair_filename_structuredslugsplus}"
        self.compiler.generate_structuredslugsplus(input_file)

        # 1. Add backup skills to the compiler
        self.compiler.add_backup_skills()
        input_file = f"build/added_backup_{self.repair_filename_structuredslugsplus}"
        if True:
            self.compiler.generate_structuredslugsplus(input_file)

        # 4. Initialize and run the repair to get new skills
        input_file = f"build/{self.repair_filename_structuredslugsplus}"
        self.repair = Repair(self.compiler, input_file, self.opts, self.files)
        new_skills = self.repair.run_symbolic_repair()
        if True:
            add_skilll_file = f'build/repaired_skill_added_{self.repair_filename_structuredslugsplus}'
            self.compiler.generate_structuredslugsplus(add_skilll_file)

        # 5. Remove backup skills from the compiler
        self.compiler.remove_backup_skills()
        if True:
            input_file = f"build/removed_backup_{self.repair_filename_structuredslugsplus}"
            self.compiler.generate_structuredslugsplus(input_file)

        # 6. Reduce the size of the new skills and add them to the compiler
        self.add_skills_with_reduced_size(new_skills)

        # 7. Check the physical implementability of the skills
        bad_intermediate_transitions, bad_skills = self.skills_executable.check_physical_implementability(
            filter_new_skills(self.skills_data, new_skills))

        # 8. If there are bad intermediate transitions, then add bad transitions to compiler, remove the previously added skill, and repair again
        if len(bad_intermediate_transitions) > 0:
            print("There are bad intermediate transitions")
            self.compiler.add_bad_intermediate_transitions(
                bad_intermediate_transitions)
            self.compiler.remove_skills()

            if DEBUG: breakpoint()
            good_skills = {k: v for k, v in new_skills.items() if k not in bad_skills}
            if len(good_skills) > 0:
                self.add_skills_with_reduced_size(good_skills)
                self.compiler.reset_after_successful_repair()
            # new_skills = self.repair.run_symbolic_repair()
            self.repair_cnt += 1
            new_skills = self.run_repair()

        if len(new_skills) == 0:
            print("No new skills generated by the repair module")
            # sys.exit(0)

        # 9. Reset the compiler after successful repair and edit the input_file and generate the structuredslugsplus and slugsin files
        # <- No bad intermediate transitions implies this call is the leaf of the recursion tree
        if len(bad_intermediate_transitions) == 0:
            self.compiler.reset_after_successful_repair()
            self.input_file_structuredslugsplus = f"build/{self.repair_filename_structuredslugsplus}"
            self.compiler.generate_structuredslugsplus(
                self.input_file_structuredslugsplus)
            self.compiler.generate_slugsin(self.slugsin)
            self.repair_cnt += 1

        return new_skills

    def add_skills_with_reduced_size(self, new_skills: dict) -> None:
        """Add skills with reduced size to the compiler
           The goal of this function is to replace self.compiler.add_skills(new_skills)

            Args:
                new_skills: dict: skills to be added to the compiler
            Returns:
                None, modify new_skills in-place
        """
        if len(new_skills) >= 2:
            for skill_name in list(new_skills.keys()):
                removed_skill = new_skills.pop(skill_name)
                self.compiler.add_skills(new_skills)
                print("Skills to check: ", self.compiler.get_skills())
                is_realizable = self.compiler.check_realizability()
                print("is_realizable: ", is_realizable)
                if not is_realizable:
                    new_skills[skill_name] = removed_skill
        self.rename_new_skills(new_skills)
        self.compiler.add_skills(new_skills)

    def rename_new_skills(self, skills: dict) -> None:
        """Rename the new skills to avoid name conflicts"""
        self.compiler.remove_skills()
        cnt = len(self.compiler.get_skills())
        for old_name in list(skills.keys()):
            skill = skills.pop(old_name)
            name = f"skill{cnt}"
            assert name not in self.compiler.get_skills()
            skill.name = name
            skill.info['name'] = name
            skills[name] = skill
            cnt += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files_json', type=str,
                        default='inputs/empty_cup/files.json', help='json file containing all json files')
    args = parser.parse_args()
    main = Main(args.files_json)
    main.main()
