#! /usr/bin/env python

import argparse
from grounding import Groundings
import sys

from monitor_compiler import Compiler
from tools import (
    json_load_wrapper, 
    clear_file, 
    write_symbols, 
    pre_posts_to_env_formula,
    find_symbols_by_object,
    find_symbols_by_object_wo_ee,
    find_controllable_manipulation_symbols_by_not_object,
    find_symbols_by_location,
    find_symbols_by_location_wo_ee,
    find_symbols_by_objects_type_category,
    find_controllable_manipulation_symbols,
    find_controllable_mobile_symbols,
    find_controllable_symbols,
    find_controllable_objects,
    dict_to_formula,
    write_section,
    list2dict_bool,
    find_manipulation_locations,
    find_controllable_manipulation_objects_wo_ee,
    load_list_keys_from_json,
    load_list_values_from_json,
    get_skill_type,
    list_minus,
    is_manipulation_object,
    is_mobile_object,
)   

DEBUG = False

class SpecWriter:
    """Given a spec json file, and symbols and skills data json files, 
       write a structuredslugsplus spec
    """
    def __init__(self, files_json: str, groundings: Groundings = None, self_loop: bool = False):
        """Initialize the spec writer"""
        self.files_data = json_load_wrapper(files_json)
        self.groundings = groundings
        self.self_loop = self_loop
        if groundings:
            self.groundings.ready_event.wait()
        self.write_spec()

    def write_env_init(self, target_filename: str, groundings: Groundings, inputs: list) -> None:
        """Write the environment init part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write('[ENV_INIT]\n\n')
        if groundings:
            inputs_dict = groundings.get_inputs()
        else:
            inputs_dict = list2dict_bool(inputs)
        for key, value in inputs_dict.items():
            if value:
                fid.write(f'{key}\n')
            else:
                fid.write(f'!{key}\n')
        fid.write('\n')
        fid.close()

    def write_sys_init(self, target_filename: str, skills_names: list, sys_init_true: list) -> None:
        """Write the system init part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write('[SYS_INIT]\n\n')
        for skill_name in skills_names:
            if skill_name in sys_init_true:
                fid.write(f'{skill_name}\n')
            else:
                fid.write(f'!{skill_name}\n')
        fid.write('\n')
        fid.close()

    def write_env_trans(self, target_filename: str, skills_data: dict) -> None:
        """Write the environment transition part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write('[ENV_TRANS]\n\n')

        fid.write('# Post-condition\n')
        for skill_name, skill_data in skills_data.items():
            for pre_dict, post_dict_list in skill_data['intermediate_states']:
                tmp_formula = pre_posts_to_env_formula(skill_name, pre_dict, post_dict_list)
                fid.write(f'{tmp_formula}\n')

        fid.write('\n')
        fid.close()

    def write_env_trans_hard(self, target_filename: str, symbols_data: dict, skills_data: dict, locations_data: dict, objects_data, obstacle_constraints: list, user_env_assumptions: list) -> None:
        """Write the environment transition hard part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write('[ENV_TRANS_HARD]\n\n')

        if obstacle_constraints:
            fid.write('\n# Obstacle constraints\n')
            for obstacle_constraint in obstacle_constraints:
                fid.write(f'{obstacle_constraint}\n')
        if user_env_assumptions:
            fid.write('\n# User environment assumptions\n')
            for assumption in user_env_assumptions:
                fid.write(f'{assumption}\n')

        self.write_inputs_mutual_exclusion(fid, symbols_data, objects_data
                                           )
        # This should go to sys_trans_hard
        loc = "ee"
        symbols = find_symbols_by_location(loc, symbols_data, locations_data, objects_data)
        if symbols:
            fid.write('\n# Location mutual exclusion for loc == ee\n')
            self.write_mutual_exclusion(fid, symbols)

        # Only of type of objects (mobile or manipulation) can change at a time
        controllable_mobile_symbols = find_controllable_mobile_symbols(symbols_data, objects_data)
        if DEBUG: breakpoint()
        controllable_manipulation_symbols = find_controllable_manipulation_symbols(symbols_data, objects_data)
        if controllable_mobile_symbols and controllable_manipulation_symbols:
            fid.write('\n# Only of type of objects (mobile or manipulation) can change at a time\n')

            # If controllable mobile objects change, then controllable manipulation objects must not changed
            fid.write('\n# If controllable mobile objects change, then controllable manipulation objects must not change\n')
            self.write_change_implies_stay(controllable_mobile_symbols, controllable_manipulation_symbols, fid)
                
            # If controllable manipulation objects change, then controllable mobile objects must not change
            fid.write('\n# If controllable manipulation objects change, then controllable mobile objects must not change\n')
            self.write_change_implies_stay(controllable_manipulation_symbols, controllable_mobile_symbols, fid)

        if 'ee' not in list(objects_data.keys()):
            # if any controllable manipulation object changes, then every other object must not change
            self.write_control_man_obj_change_implies_other_obj_stay(fid, objects_data, symbols_data)

        self.write_manipulation_object_ee_relation(fid, objects_data, locations_data, symbols_data)
        self.write_manipulation_object_inactivity_wo_ee(fid, objects_data, locations_data, symbols_data)
        
        fid.write('\n# Inactivity without skills\n')
        # Inactivity without each type of skills
        for type in ["mobile", "manipulation"]:
            skills = [skill for skill, skill_data in skills_data.items() if get_skill_type(skill_data, objects_data) == type]
            symbols = find_symbols_by_objects_type_category(type, symbols_data, objects_data)
            # print("mobile_only: ", self.MOBILE_ONLY)
            # print("manipulation_only: ", self.MANIPULATION_ONLY)
            if skills and not self.MOBILE_ONLY and not self.MANIPULATION_ONLY:
                self.write_inactivity_without_skills(fid, skills, symbols)
        
        # Inactivity without any skills
        # self.write_inactivity_without_skills(fid, list(skills_data.keys()), self.groundings.controllable_symbols)
        if True:
            self.write_inactivity_without_skills(fid, list(skills_data.keys()), find_controllable_symbols(symbols_data, objects_data))


        fid.write('\n# Skills mutual exclusion\n')
        self.write_skills_mutual_exclusion(fid, skills_data)

        fid.write('\n')
        fid.close()
    
    def write_inputs_mutual_exclusion(self, fid, symbols_data, objects_data):
        """Write inputs mutual exclusion"""
        fid.write('\n# Inputs mutual exclusion\n\n')
        fid.write('# Objects mutual exclusion\n')
        for obj in list(objects_data.keys()):
            if objects_data[obj]["type"] == "mobile": # object is mobile type
                symbols = symbols_with_ee = find_symbols_by_object(obj, symbols_data)
            elif objects_data[obj]["type"] == "manipulation" or objects_data[obj]["type"] == "any":
                # without p_XX_ee as an object can be both at a table and at ee"""
                symbols, symbols_with_ee = find_symbols_by_object_wo_ee(obj, symbols_data)
            else:
                raise Exception(f"Invalid object type {objects_data[obj]['type']}")
            if 'ee' in list(objects_data.keys()):
                self.write_mutual_exclusion(fid, symbols)
            else:
                self.write_mutual_exclusion(fid, symbols_with_ee)
            self.write_must_exist(fid, symbols_with_ee)

    
    def write_mutual_exclusion(self, fid, symbols):
        """Write mutual exclusion (exactly one symbol in the set of symbols is and will be true)"""
        for i in range(len(symbols)):
            for j in range(i+1, len(symbols)):
                fid.write(f'!({symbols[i]} & {symbols[j]})\n')
                fid.write(f'!({symbols[i]}\' & {symbols[j]}\')\n')

    def write_must_exist(self, fid, symbols):
        fid.write('(' + ' | '.join(symbols) + ')\n')
        fid.write('(' + '\' | '.join(symbols) + '\')\n')
        

    def write_inactivity_without_skills(self, fid, skills, symbols):
        """Write inactivity without skills (if no skill in the set of skills is true, then no symbol in the set of symbols is true)"""
        not_action = "!" + " & !".join(skills)
        sym_stay_list = []
        for sym in symbols:
            sym_stay_list.append("({} <-> {}')".format(sym, sym))
        sym_stay = " & ".join(sym_stay_list)
        fid.write('({}) -> ({})\n'.format(not_action, sym_stay))

    def write_skills_mutual_exclusion(self, fid, skills_data, isPrime = False):
        """Write skills mutual exclusion (at most one skill in the set of skills is true)"""
        skills = list(skills_data.keys())
        if isPrime:
            skills = [skill + "'" for skill in skills]
        for i in range(len(skills)):
            s_write = ''
            for j in range(len(skills)):
                if i == j:
                    s_write = s_write + " & " + skills[j]
                else:
                    s_write = s_write + " & !" + skills[j]
            s_out = '(' + s_write[3:] + ')'
            fid.write(s_out + ' | ')
        # no actions
        self.no_actions = "!" + " & !".join(skills)
        fid.write(f"({self.no_actions})\n")

    def write_change_implies_stay(self, change_symbols, stay_symbols, fid):
        """Write change implies stay
        If a symbol in the change_symbols changed value, then all symbols in the stay_symbols must not change
        """
        if change_symbols and stay_symbols:
            sym_change_list = []
            for sym in change_symbols:
                sym_change_list.append("({} <-> !{}')".format(sym, sym))
            sym_change = " | ".join(sym_change_list)
            sym_stay_list = []
            for sym in stay_symbols:
                sym_stay_list.append("({} <-> {}')".format(sym, sym))
            sym_stay = " & ".join(sym_stay_list)
            fid.write('({}) -> ({})\n'.format(sym_change, sym_stay))

    def write_stay(self, symbols, fid):
        """Write stay (symbol value does not change)"""
        if symbols:
            sym_stay_list = []
            for sym in symbols:
                sym_stay_list.append("({} <-> {}')".format(sym, sym))
            sym_stay = " & ".join(sym_stay_list)
            fid.write('({})\n'.format(sym_stay))

    def write_control_man_obj_change_implies_other_obj_stay(self, fid, objects_data, symbols_data):
        """Write control manipulation object change implies other objects stay"""
        controllable_objects = find_controllable_objects(objects_data)
        if controllable_objects:
            fid.write('\n# Control manipulation object change implies other objects stay\n')
            for obj in controllable_objects:
                if objects_data[obj]["type"] == "manipulation" or objects_data[obj]["type"] == "any":
                    change_symbols = find_symbols_by_object(obj, symbols_data)
                    stay_symbols = []
                    for obj2 in find_controllable_objects(objects_data):
                        if obj2 != obj:
                            stay_symbols += find_symbols_by_object(obj2, symbols_data)
                    self.write_change_implies_stay(change_symbols, stay_symbols, fid)
        return None


    def write_sys_trans(self, target_filename: str, skills_data: dict) -> None:
        """Write the system transition part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write(f"[SYS_TRANS]\n\n")
        
        # What states need to be true for a skill to be performed
        allowable_skill_list = []
        skill_continue_list = []
        for skill_name, skill_data in skills_data.items():
            init_pres = skill_data['initial_preconditions']
            # final_post = skill_data['final_postconditions']
            intermediate_states = skill_data['intermediate_states']
            allowable_skill = '!('
            all_pres = [pre for pre, _ in intermediate_states]
            if DEBUG:
                print(f"all_pres: {(all_pres)}")
            for pre_dict, post_dict_list in intermediate_states:
                for post_dict in post_dict_list:
                    if post_dict not in all_pres or pre_dict == post_dict:
                        # print("skip")
                        continue
                    allowable_skill += "({} & {} & {}) | ".format(dict_to_formula(pre_dict), skill_name, dict_to_formula(post_dict, prime=True))
                    skill_continue = "({} & {} & {}) -> {}'".format(dict_to_formula(pre_dict), skill_name, dict_to_formula(post_dict, prime=True), skill_name)
                    skill_continue_list.append(skill_continue)

            p_sym_list = []
            for pre_dict in init_pres:
                p_sym_list.append("(" + dict_to_formula(pre_dict, prime=True) + ")")
            allowable_skill += "{}) -> !{}\'".format(" | ".join(p_sym_list), skill_name)
            allowable_skill_list.append(allowable_skill)

        fid.write("{}\n".format("\n".join(allowable_skill_list)))
        fid.write("{}\n".format("\n".join(skill_continue_list)))

        fid.write("\n")
        fid.close()

    def write_sys_trans_hard(self, target_filename: str, skills_data: dict, locations_data: dict, objects_data: dict, symbols_data: dict, obstacles_sys: list, user_sys_constraints: list) -> None:
        """Write the system transition hard part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write("[SYS_TRANS_HARD]\n\n")

        if obstacles_sys:
            fid.write('# Obstacle constraint system\n')
            for obstacle_constraint in obstacles_sys:
                fid.write(f'{obstacle_constraint}\n')
        
        if user_sys_constraints:
            fid.write('\n# User system safety constraints\n')
            for constraint in user_sys_constraints:
                fid.write(f'{constraint}\n')

        fid.write('\n# Location mutual exclusion\n')
        for loc in list(locations_data.keys()):
            if locations_data[loc]['type'] == 'mobile' or locations_data[loc]['type'] == 'robot' or loc == "ee":
                # For mobile, robot locations, objects are mutual exclusive
                symbols = find_symbols_by_location(loc, symbols_data, locations_data, objects_data)
                
            elif locations_data[loc]["type"] == "manipulation":
                symbols = find_symbols_by_location_wo_ee(loc, symbols_data, locations_data, objects_data)
            else:
                raise Exception(f"Invalid location type {locations_data[loc]['type']}")
            self.write_mutual_exclusion(fid, symbols)

        fid.write('\n# Skills mutual exclusion\n')
        self.write_skills_mutual_exclusion(fid, skills_data, isPrime=True)

        fid.write('\n')
        fid.close()
    
    def write_manipulation_object_ee_relation(self, fid, objects_data, locations_data, symbols_data):
        if 'ee' in list(objects_data.keys()):
            fid.write('\n# Object-ee relation implies ee location\n')
            manipulation_locations: dict = find_manipulation_locations(locations_data)
            manipulation_locations_wo_ee = {k: v for k, v in manipulation_locations.items() if k != "ee"}
            manipulation_objects = find_controllable_manipulation_objects_wo_ee(objects_data)
            for object in list(manipulation_objects.keys()):
                for loc in list(manipulation_locations_wo_ee.keys()):
                    fid.write(f"((p_{object}_{loc} & p_{object}_ee) -> p_ee_{loc})\n")
                    fid.write(f"((p_{object}_{loc}' & p_{object}_ee') -> p_ee_{loc}')\n")
            
            fid.write('\n# Object-ee relation implies object location\n')
            for object in list(manipulation_objects.keys()):
                for loc in list(manipulation_locations_wo_ee.keys()):
                    fid.write(f"((p_ee_{loc} & p_{object}_ee) -> p_{object}_{loc})\n")
                    fid.write(f"((p_ee_{loc}' & p_{object}_ee') -> p_{object}_{loc}')\n")

            fid.write('\n# Object ee locations imply object-ee relation\n')
            for object in list(manipulation_objects.keys()):
                for loc in list(manipulation_locations_wo_ee.keys()):
                    fid.write(f"((p_ee_{loc} & p_{object}_{loc}) -> p_{object}_ee)\n")
                    fid.write(f"((p_ee_{loc}' & p_{object}_{loc}') -> p_{object}_ee')\n")
        return None
    
    def write_manipulation_object_inactivity_wo_ee(self, fid, objects_data, locations_data, symbols_data):
        if 'ee' in list(objects_data.keys()):
            fid.write('\n# Manipulation_object_inactivity_wo_ee\n')
            manipulation_objects = find_controllable_manipulation_objects_wo_ee(objects_data)
            for obj in list(manipulation_objects.keys()):
                fid.write(f"!p_{obj}_ee -> ")
                locs, _ = find_symbols_by_object_wo_ee(obj, symbols_data)
                self.write_stay(locs, fid)
        else:
            manipulation_objects = find_controllable_manipulation_objects_wo_ee(objects_data)
            if len(manipulation_objects.keys()) > 0:
                fid.write('\n# Manipulation_object_inactivity_wo_ee\n')
                for obj in list(manipulation_objects.keys()):
                    stay_symbols = find_controllable_manipulation_symbols_by_not_object(obj, symbols_data, objects_data)
                    if len(stay_symbols) > 0:
                        fid.write(f"p_{obj}_ee -> ")
                        self.write_stay(stay_symbols, fid)
        return None
    
    def write_ee_continuity(self, fid, objects_data, locations_data, symbols_data):
        if 'ee' in list(objects_data.keys()):
            fid.write('\n# ee_continuity\n')        
            manipulation_objects = find_controllable_manipulation_objects_wo_ee(objects_data)
            for obj in list(manipulation_objects.keys()):
                symbols_wo_ee, _ = find_symbols_by_object_wo_ee(obj, symbols_data)
                some_change = self.create_some_change(symbols_wo_ee)
                some_change_or_ee = f"({some_change} | p_ee_else)"
                fid.write(f"(p_{obj}_ee & {some_change_or_ee}) -> p_{obj}_ee'\n")
        return None

    def write_object_ee_relation_disappear(self, fid, objects_data, locations_data, symbols_data):
        if 'ee' in list(objects_data.keys()):
            fid.write('\n# Object-ee relation disappear implies ee in else\n')
            manipulation_objects = find_controllable_manipulation_objects_wo_ee(objects_data)
            ee_disappear = ""
            if len(manipulation_objects.keys()) > 0:
                for obj in list(manipulation_objects.keys()):
                    ee_disappear += f" | (p_{obj}_ee & !p_{obj}_ee')"
                fid.write(f"({ee_disappear[3:]}) -> p_ee_else'\n")
        return None


    def write_change_constraints(self, target_filename: str, symbols_data: dict, locations_data: dict, objects_data: dict, opts: dict, uncontrollable_inputs: list, user_inputs: list = []) -> None:
        """Write the change constraints part of the structuredslugsplus file"""
        fid = open(target_filename, 'a')
        fid.write('[CHANGE_CONS]\n')

        # Only of type of objects (mobile or manipulation) can change at a time 
        # # <- We don't need this because of the mobile/manipulation symbols must not change constraint
        # fid.write('\n# Only of type of objects (mobile or manipulation) can change at a time\n')
        controllable_mobile_symbols = find_controllable_mobile_symbols(symbols_data, objects_data)
        controllable_manipulation_symbols = find_controllable_manipulation_symbols(symbols_data, objects_data)

        if "mobile_repair" in opts.keys():
            new_skill_manipulation = not opts["mobile_repair"]
        else:
            new_skill_manipulation = True

        if new_skill_manipulation:
            # Mobile symbols must not change
            fid.write('\n# Mobile symbols must not change\n')
            self.write_stay(controllable_mobile_symbols, fid)
        else:
            # Manipulation symbols must not change
            fid.write('\n# Manipulation symbols must not change\n')
            self.write_stay(controllable_manipulation_symbols, fid)

        # Uncontrollable symbols must not change
        fid.write('\n# Uncontrollable symbols must not change\n')
        # if self.groundings:
        #     self.write_stay(self.groundings.uncontrollable_symbols, fid)
        self.write_stay(list_minus(uncontrollable_inputs, user_inputs), fid)

        self.write_inputs_mutual_exclusion(fid, symbols_data, objects_data)
        
        fid.write('\n# Some controllable inputs must change\n')
        fid.write(f"!({self.create_no_change(find_controllable_symbols(symbols_data, objects_data))})\n")

        fid.write('\n\n')
        
        fid.write('\n')
        fid.close()
    
    def write_not_allowed_repair(self, target_filename: str, symbols_data: dict, locations_data: dict, objects_data: dict, not_allowed_repair_data: str, opts: dict, uncontrollable_inputs: list, user_inputs: list = []): 
        """Write the not allowed repair part of the structuredslugsplus file"""

        fid = open(target_filename, 'a')
        fid.write('[NOT_ALLOWED_REPAIR]\n\n')
        
        # Only of type of objects (mobile or manipulation) can change at a time 
        # # <- We don't need this because of the mobile/manipulation symbols must not change constraint
        # fid.write('\n# Only of type of objects (mobile or manipulation) can change at a time\n')
        controllable_mobile_symbols = find_controllable_mobile_symbols(symbols_data, objects_data)
        controllable_manipulation_symbols = find_controllable_manipulation_symbols(symbols_data, objects_data)

        if "mobile_repair" in opts.keys():
            new_skill_manipulation = not opts["mobile_repair"]
        else:
            new_skill_manipulation = True

        if new_skill_manipulation:
            # Mobile symbols must not change
            fid.write('\n# Mobile symbols must not change\n')
            self.write_stay(controllable_mobile_symbols, fid)
        else:
            # Manipulation symbols must not change
            fid.write('\n# Manipulation symbols must not change\n')
            self.write_stay(controllable_manipulation_symbols, fid)

        # Uncontrollable symbols must not change
        fid.write('\n# Uncontrollable symbols must not change\n')
        # if self.groundings:
        #     self.write_stay(self.groundings.uncontrollable_symbols, fid)
        print("uncontrollable_inputs", uncontrollable_inputs)
        print("user_inputs", user_inputs)
        self.write_stay(list_minus(uncontrollable_inputs, user_inputs), fid)

        # If controllable mobile objects change, then controllable manipulation objects must not changed
        fid.write('\n# If controllable mobile objects change, then controllable manipulation objects must not change\n')
        self.write_change_implies_stay(controllable_mobile_symbols, controllable_manipulation_symbols, fid)
                
        # If controllable manipulation objects change, then controllable mobile objects must not change
        fid.write('\n# If controllable manipulation objects change, then controllable mobile objects must not change\n')
        self.write_change_implies_stay(controllable_manipulation_symbols, controllable_mobile_symbols, fid)

        if 'ee' not in list(objects_data.keys()):
            # if any controllable manipulation object changes, then every other object must not change
            self.write_control_man_obj_change_implies_other_obj_stay(fid, objects_data, symbols_data)


        fid.write('\n# Location mutual exclusion\n')
        for loc in list(locations_data.keys()):
            if locations_data[loc]['type'] == 'mobile' or locations_data[loc]['type'] == 'robot' or loc == "ee":
                # For mobile, robot locations, objects are mutual exclusive
                symbols = find_symbols_by_location(loc, symbols_data, locations_data, objects_data)
                
            elif locations_data[loc]["type"] == "manipulation":
                symbols = find_symbols_by_location_wo_ee(loc, symbols_data, locations_data, objects_data)
            else:
                raise Exception(f"Invalid location type {locations_data[loc]['type']}")
            self.write_mutual_exclusion(fid, symbols)

        self.write_manipulation_object_ee_relation(fid, objects_data, locations_data, symbols_data)
        self.write_manipulation_object_inactivity_wo_ee(fid, objects_data, locations_data, symbols_data)
        self.write_ee_continuity(fid, objects_data, locations_data, symbols_data)
        self.write_object_ee_relation_disappear(fid, objects_data, locations_data, symbols_data)

        fid.write("\n")
        fid.write(not_allowed_repair_data)
        fid.write("\n")
        fid.close()

    def write_either_some_change_or_no(self, fid, symbols: list):
        """Write either some change or no change"""
        s_write = ''
        for sym in symbols:
            s_write = s_write + " | " + f"({sym} <-> !{sym}')"
        s_out = '(' + s_write[3:] + ')'
        s_out = s_out + " | " + f"({self.create_no_change(symbols)})"
        fid.write(s_out + '\n\n')
    
    def create_no_change(self, symbols: list) -> str:
        """Create no change in symbols"""
        s_write = ''
        for sym in symbols:
            s_write = s_write + " & " + f"({sym} <-> {sym}')"
        s_out = '(' + s_write[3:] + ')'
        return s_out
    
    def create_some_change(self, symbols: list) -> str:
        """Create some change in symbols"""
        s_write = ''
        for sym in symbols:
            s_write = s_write + " | " + f"({sym} <-> !{sym}')"
        s_out = '(' + s_write[3:] + ')'
        return s_out


    def write_spec(self):
        """Write the structuredslugsplus file"""
        spec_data = json_load_wrapper(self.files_data['spec_json'])
        opts = json_load_wrapper(self.files_data['opts'])

        target_filename = self.files_data['structuredslugsplus']
        inputs_data = json_load_wrapper(self.files_data['inputs'])
        inputs = list(inputs_data.keys())
        uncontrollable_inputs = load_list_values_from_json(self.files_data['uncontrollable_inputs'])
        if self.self_loop:
            skills_data = json_load_wrapper(self.files_data['skills_selfloop'])
        else:
            skills_data = json_load_wrapper(self.files_data['skills'])
        skills_names = list(skills_data.keys())
        locations_data = json_load_wrapper(self.files_data['locations'])
        objects_data = json_load_wrapper(self.files_data['objects'])
        user_inputs = load_list_keys_from_json(self.files_data['user_inputs'])
        
        if 'obstacle_constraints' in spec_data.keys():
            obstacles = spec_data['obstacle_constraints']
        else:
            obstacles = []
        if 'obstacle_constraints_sys' in spec_data.keys():
            obstacles_sys = spec_data['obstacle_constraints_sys']
        else:
            obstacles_sys = []
        if 'user_sys' in spec_data.keys():
            user_sys_constraints = spec_data['user_sys']
        else:
            user_sys_constraints = []
        if 'user_env' in spec_data.keys():
            user_env_assumptions = spec_data['user_env']
        else:
            user_env_assumptions = []

        self.MANIPULATION_ONLY = all(is_manipulation_object(obj, objects_data) for obj in objects_data.keys())
        self.MOBILE_ONLY = all(is_mobile_object(obj, objects_data) for obj in objects_data.keys())
        
        clear_file(target_filename)

        # INPUT
        write_symbols(target_filename, 'INPUT', inputs)

        # OUTPUT
        write_symbols(target_filename, 'OUTPUT', skills_names)

        # ENV_INIT
        self.write_env_init(target_filename, self.groundings, inputs)

        # SYS_INIT
        self.write_sys_init(target_filename, skills_names, spec_data['sys_init_true'])

        # ENV_TRANS
        self.write_env_trans(target_filename, skills_data)

        # ENV_TRANS_HARD
        self.write_env_trans_hard(target_filename, inputs_data, skills_data, 
                                  locations_data, objects_data, obstacles, user_env_assumptions)

        # SYS_TRANS
        self.write_sys_trans(target_filename, skills_data)

        # SYS_TRANS_HARD
        self.write_sys_trans_hard(target_filename, skills_data, locations_data, objects_data, inputs_data, obstacles_sys, user_sys_constraints)

        # ENV_LIVENESS
        if spec_data['env_live']:
            write_section(target_filename, "ENV_LIVENESS", spec_data['env_live'])

        # SYS_LIVENESS
        write_section(target_filename, "SYS_LIVENESS", spec_data['sys_live'])
        print(f"user inputs: {user_inputs}")
        # CHANGE CONSTRAINTS
        self.write_change_constraints(target_filename, inputs_data, locations_data, objects_data, opts, uncontrollable_inputs, user_inputs)

        # NOT ALLOWED REPAIR
        if 'not_allowed_repair' in spec_data.keys():
            self.write_not_allowed_repair(target_filename, inputs_data, locations_data, objects_data, spec_data['not_allowed_repair'], opts, uncontrollable_inputs, user_inputs)
        # write_section(target_filename, "NOT_ALLOWED_REPAIR", spec_data['not_allowed_repair'])

def test_spec_realizability(files_json):
    files_data = json_load_wrapper(files_json)
    compiler = Compiler(files_data["structuredslugsplus"], files_data["skills"])
    # start_time = rospy.get_time()
    compiler.check_realizability()
    # end_time = rospy.get_time()
    # elapsed_time = end_time - start_time
    # print(f"Time to check realizability: {elapsed_time}")

def test_spec_writer(files_json, self_loop: bool = False):
    # controller = MainController(objects_list = ['cup', 'cone', 'block'])
    # groundings = Groundings(files_json, controller)
    # rospy.sleep(1)
    # while not rospy.is_shutdown() and not groundings.is_ready():
    #     rospy.sleep(1)
    # start_time = rospy.get_time()
    spec_writer = SpecWriter(files_json, None, self_loop)
    # end_time = rospy.get_time()
    # elapsed_time = end_time - start_time
    # print(f"Time to write spec: {elapsed_time}")

    test_spec_realizability(files_json)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files_json', type=str, default='inputs/empty_cup/files.json', 
                        help='json file containing all json files')
    parser.add_argument('-s', '--self_loop', action='store_true', dest='self_loop', required=False, default=False,) # Store true if the flag is presented
    args = parser.parse_args()
    files_json = args.files_json
    # set recursion limit
    sys.setrecursionlimit(10**6)
    test_spec_writer(files_json, args.self_loop)
    # test_spec_realizability(files_json)
    
    # test_spec_writer(spec_writer)