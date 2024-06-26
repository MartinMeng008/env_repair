#! /usr/bin/env python
import os
import sys
import copy
curr_dir = os.getcwd()
from tools import (
    dump_json,
)

class SkillGenerator:
    def __init__(self, skill_name: str, start_state: dict, transition: str, primitive_skill: str, type: str, goal_type: str):
        self.skill_name = skill_name
        self.start_state = start_state
        self.primitive_skill = primitive_skill
        self.type = type
        self.goal_type = goal_type
        self.transitions = [tran.strip() for tran in transition.split("->")]
        # print(self.transitions)
        self.skill_data = self.generate()


    def generate(self) -> dict:
        """Generate a skill dictionary from start_state and transition"""
        skill_data = {}
        skill_data["name"] = self.skill_name
        skill_data["primitive_skill"] = self.primitive_skill
        skill_data["type"] = self.type
        skill_data["goal_type"] = self.goal_type
        skill_data["initial_preconditions"] = [copy.deepcopy(self.start_state)]
        skill_data["intermediate_states"] = self.generate_intermediate_states()
        skill_data["final_postconditions"] = copy.deepcopy(skill_data["intermediate_states"][-1][-1])
        return skill_data
    
    def generate_intermediate_states(self) -> list:
        """Generate intermediate states from transitions"""
        intermediate_states = []
        curr_state = copy.deepcopy(self.start_state)
        for i in range(len(self.transitions) - 1):
            assert curr_state[self.transitions[i]] == True
            next_state = copy.deepcopy(curr_state)
            next_state[self.transitions[i]] = False
            next_state[self.transitions[i+1]] = True
            intermediate_states.append([copy.deepcopy(curr_state), [copy.deepcopy(next_state)]])
            curr_state = copy.deepcopy(next_state)
        return intermediate_states

if __name__ == "__main__":
    start_state = {
                "p_base_x0": False,
                "p_base_x1": False,
                "p_base_x2": False,
                "p_base_x3": False,
                "p_base_x4": True,
                "p_cup_k0": False,
                "p_cup_k1": False,
                "p_cup_k2": False,
                "p_cup_k3": False,
                "p_cup_t": False,
                "p_cup_ee": True,
                # "p_block_k0": False,
                # "p_block_k1": False,
                # "p_block_k2": False,
                # "p_block_k3": False,
                # "p_block_t": False,
                # "p_block_ee": True,
                }
    transition = "p_base_x4 -> p_base_x2 -> p_base_x0"
    skill_name = "skill1"
    primitive_skill = "movebase"
    type = "mobile"
    goal_type = "region"
    skill_generator = SkillGenerator(skill_name, start_state, transition, primitive_skill, type, goal_type)
    skill_data = skill_generator.generate()
    # skill_generator.print_skill_data(skill_data)
    dump_json(f"{curr_dir}/build/{skill_name}.json", {skill_name: skill_data})