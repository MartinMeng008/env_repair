# !/usr/bin/env python

import json
from copy import deepcopy
import os
import threading
import sys
import subprocess
import numpy as np
import copy
from tf_conversions import transformations
from geometry_msgs.msg import (
    Pose,
    Quaternion,
)

from skills import Skill

DEBUG = False

## ==== Abstraction related functions ==== ##
def create_symbols_from_objects_and_locations(objects_data: dict, locations_data: dict) -> list:
    """Create symbols from objects and locations.
    
    Args:
        objects_data: Dict of objects data.
        location_data: Dict of grounding of locations.
    Returns:
        List of symbols data.
    """
    symbols = {}
    index_cnt = 0
    
    ## All objects X locations
    for obj, obj_dict in objects_data.items():
        for loc, loc_dict in locations_data.items():
            if ((obj_dict['type'] == loc_dict['type'] or obj_dict['type'] == 'any') and 
                not loc_dict["is_robot_location"]):
                
                sym_name = f'p_{obj}_{loc}'
                if DEBUG:
                    print(sym_name)
                symbols[sym_name] = create_symbol_dict(sym_name, obj, loc, index_cnt, locations_data)
                index_cnt += 1
        if obj == 'ee':
            sym_name = 'p_ee_else'
            symbols[sym_name] = create_symbol_dict(sym_name, obj, 'else', index_cnt)
            index_cnt += 1

    ## Controllable objects X robot locations
    for obj, obj_dict in objects_data.items():
        if (obj_dict['category'] == 'controllable'):
            for loc, loc_dict in locations_data.items():
                if ((obj_dict['type'] == loc_dict['type'] or obj_dict['type'] == 'any') and 
                    loc_dict["is_robot_location"]):

                    sym_name = f'p_{obj}_{loc}'
                    if DEBUG:
                        print(sym_name)
                    symbols[sym_name] = create_symbol_dict(sym_name, obj, loc, index_cnt)
                    index_cnt += 1
    return symbols

def create_symbol_dict(sym_name, obj, loc, index_cnt, locations_data = None):
    """Create a symbol dictionary.
    
    Args:
        sym_name: Symbol name.
        obj: Object name.
        loc: Location name.
        index_cnt: Index count.
        obj_idx: Object index.
        location_data: Dict of grounding of locations. If none, then location is not grounded (ee)
    Returns:
        Symbol dictionary.
    """
    sym_dict = {}
    sym_dict['index'] = index_cnt
    if DEBUG:
        print(index_cnt)
    sym_dict['name'] = sym_name
    if locations_data and not locations_data[loc]['is_robot_location']:
        sym_dict['dims'] = locations_data[loc]['dims']
        sym_dict['bounds'] = locations_data[loc]['bounds']
    # sym_dict['factor'] = obj_idx
    sym_dict['object'] = obj
    sym_dict['location'] = loc
    return sym_dict

def object_location_to_symbol(object_name, location_name):
    """Given object, location, create symbol"""
    return f'p_{object_name}_{location_name}'

def symbol_to_object_location(symbol):
    """Given symbol, return object and location"""
    return symbol.split('_')[1], symbol.split('_')[2]

def symbol_to_object(symbol):
    """symbol -> object"""
    return symbol_to_object_location(symbol)[0]

def symbol_to_location(symbol):
    """symbol -> location"""
    return symbol_to_object_location(symbol)[1]

def printList(L):
    for i in range(len(L)):
        print(L[i])

def find_symbols_by_object(obj: str, symbols_data: dict) -> list:
    """Return list of symbols with object name == obj"""
    return [s for s, s_data in symbols_data.items() if s_data and 'object' in s_data and s_data['object'] == obj]

def find_symbols_by_object_wo_ee(obj: str, symbols_data: dict) -> list:
    """Return list of symbols with object name == obj, without p_object_ee"""
    symbols_with_ee = find_symbols_by_object(obj, symbols_data)
    return [s for s in symbols_with_ee if s != f'p_{obj}_ee'], symbols_with_ee

def find_symbols_by_location(loc: str, symbols_data: dict, locations_data, objects_data) -> list:
    """Return list of symbols with location name == loc and object has the same type as location"""
    return [s for s, s_data in symbols_data.items() if 
            s_data and 'location' in s_data and 'object' in s_data and
            (s_data['location'] == loc and (objects_data[s_data['object']]['type'] == locations_data[loc]['type'] or 
                (locations_data[loc]['type']  == 'manipulation' and objects_data[s_data['object']]['type'] == 'any')))]

def find_symbols_by_location_wo_ee(loc: str, symbols_data: dict, locations_data, objects_data) -> list:
    """Return list of symbols with location name == loc and object has the same type as location"""
    return [s for s in find_symbols_by_location(loc, symbols_data, locations_data, objects_data) if s != f'p_ee_{loc}']

def find_symbols_by_objects_type_category(type, symbols_data, objects_data):
    """Return list of symbols with object type == type"""
    return [s for s, s_data in symbols_data.items() 
            if s_data and (
            objects_data[s_data['object']]['category'] != "uncontrollable" and
            (objects_data[s_data['object']]['type'] == type 
            or type == "manipulation" 
            and objects_data[s_data['object']]['type'] == "any"))]

def find_controllable_symbols(symbols_data, objects_data):
    """Return list of symbols with object category == controllable"""
    return [s for s, s_data in symbols_data.items() 
            if s_data and objects_data[s_data['object']]['category'] != "uncontrollable"]

def find_controllable_objects(objects_data):
    """Return list of symbols with object category == controllable"""
    return [obj for obj, obj_data in objects_data.items() 
            if obj_data['category'] != "uncontrollable"]

def find_uncontrollable_symbols(symbols_data, objects_data):
    """Return list of symbols with object category == uncontrollable"""
    return [s for s, s_data in symbols_data.items() 
            if objects_data[s_data['object']]['category'] == "uncontrollable"]

def load_list_keys_from_json(json_file: str) -> list:
    """Load inputs list from json file"""
    inputs_data = json_load_wrapper(json_file)
    return list(inputs_data.keys())[0] if inputs_data else []

def load_list_values_from_json(json_file: str) -> list:
    """Load inputs list from json file"""
    inputs_data = json_load_wrapper(json_file)
    return list(inputs_data.values())[0]

def find_controllable_mobile_symbols(symbols_data, objects_data):
    """Return list of symbols with object category == controllable and type == mobile"""
    return [s for s, s_data in symbols_data.items() 
            if s_data and 
            (objects_data[s_data['object']]['category'] != "uncontrollable" and
            objects_data[s_data['object']]['type'] == "mobile")]

def find_controllable_manipulation_symbols(symbols_data, objects_data):
    """Return list of symbols with object category == controllable and type == manipulation"""
    if DEBUG:
        breakpoint()
        for obj, obj_data in objects_data.items():
            if obj_data['category'] != "uncontrollable" and\
                (obj_data['type'] == "any" or obj_data['type']  == "manipulation"):
                print("controllable_manipulation_obj: ", obj)
            
    
    return [s for s, s_data in symbols_data.items()
            if s_data and 
            (objects_data[s_data['object']]['category'] != "uncontrollable" and
            (objects_data[s_data['object']]['type'] == "any" or objects_data[s_data['object']]['type'] == "manipulation"))]

def find_controllable_manipulation_symbols_by_not_object(obj: str, symbols_data: dict, objects_data: dict) -> list:
    """Find controllable manipulation symbols whose object is not the given object"""
    return [s for s in find_controllable_manipulation_symbols(symbols_data=symbols_data, objects_data=objects_data) if symbol_to_object(s) != obj]

def find_mobile_locations(locations_data: dict) -> dict:
    """Return dicts of locations with type == mobile"""
    return {loc: loc_data for loc, loc_data in locations_data.items() if loc_data['type'] == 'mobile'}

def find_manipulation_locations(locations_data: dict) -> dict:
    """Return dicts of locations with type == manipulation"""
    return {loc: loc_data for loc, loc_data in locations_data.items() if loc_data['type'] == 'manipulation'}

def find_manipulation_locations_wo_ee(locations_data: dict) -> dict:
    """Return dicts of locations with type == manipulation"""
    return {loc: loc_data for loc, loc_data in find_manipulation_locations(locations_data).items() if loc != 'ee'}

def find_manipulation_objects(objects_data: dict) -> dict:
    """Return dicts of objects with type == manipulation"""
    return {obj: obj_data for obj, obj_data in objects_data.items() if obj_data['type'] == 'manipulation'}

def find_manipulation_objects_wo_ee(objects_data: dict) -> dict:
    """Return dicts of objects with type == manipulation"""
    return {obj: obj_data for obj, obj_data in objects_data.items() if obj_data['type'] == 'manipulation' and obj != 'ee'}

def find_controllable_manipulation_objects_wo_ee(objects_data: dict) -> dict:
    """Return dicts of objects with type == manipulation"""
    return {obj: obj_data for obj, obj_data in objects_data.items() if obj_data['type'] == 'manipulation' and obj != 'ee' and obj_data['category'] != 'uncontrollable'}

def is_manipulation_object(obj: str, objects_data: dict) -> bool:
    """Check if the object is a manipulation object"""
    return objects_data[obj]['type'] == 'manipulation'

def is_mobile_object(obj: str, objects_data: dict) -> bool:
    """Check if the object is a mobile object"""
    return objects_data[obj]['type'] == 'mobile'

def find_changing_object(intermediate_states: list, objects_data: dict) -> str:
    """Find the object that has changed in the intermediate states"""
    prev_obj = None
    for pre_cond, post_cond_list in intermediate_states:
        for post_cond in post_cond_list:
            obj = find_changing_object_in_pre_post(pre_cond, post_cond, objects_data)
            if prev_obj is None or prev_obj == obj:
                prev_obj = obj
            else:
                raise Exception(f"More than one object has changed in the intermediate states: {prev_obj} and {obj}")
    return obj

def find_changing_object_in_pre_post(pre_cond: dict, post_cond: dict, objects_data: dict) -> str:
    """Find the object that has changed in the pre and post conditions"""
    prev_obj = None
    for sym, val in pre_cond.items():
        if val != post_cond[sym]:
            obj = symbol_to_object(sym)
            if prev_obj is None or prev_obj == obj:
                prev_obj = obj
            else:
                raise Exception(f"More than one object has changed in the pre and post conditions: {prev_obj} and {obj}")
    return obj
            
def is_uncontrol_mani_obj_changed(X: dict, X_p: dict, objects_data: dict, inputs_data: dict) -> bool:
    """Check if the uncontrollable manipulation object has changed"""
    changed_input = find_changing_input(X, X_p)
    if changed_input and inputs_data[changed_input] and \
        objects_data[inputs_data[changed_input]['object']]['category'] == 'uncontrollable' and \
        objects_data[inputs_data[changed_input]['object']]['type'] == 'manipulation':
        return True
    return False
    

## ==== Slugs related functions ==== ##
def run_slugs(filename_slugsin: str) -> dict:
    """Given a slugsin file, return the strategy produced by slugs"""
    print("Generate strategy")
    result = subprocess.run(['slugs', '--explicitStrategy', '--jsonOutput', '--cooperativeGR1Strategy',
                            filename_slugsin], capture_output=True, text=True)
    if DEBUG:
        result = subprocess.run(['slugs', '--counterStrategy',
                            filename_slugsin], capture_output=True, text=True)
    if DEBUG:
        result = subprocess.run(['slugs', '--explicitStrategy', '--cooperativeGR1Strategy',
                            filename_slugsin], capture_output=True, text=True)
    return result

def check_slugsin_realizability(filename_slugsin: str) -> bool:
    """Given a slugsin file, check if the specification is realizable"""
    print("Checking realizability")
    result = run_slugs(filename_slugsin)
    if True:
        print(result.stdout)
    print(result.stderr)
    return not "unrealizable" in result.stderr

## ==== Json related functions ==== ##
def remove_invalid_json_entries(d):
    for k, v in d.items():
        if v == 'none':
            d[k] = None


def json_load_wrapper(arg_file: str) -> dict:
    """Load a json file into a dictionary """
    fid = open(arg_file, "r")
    d = json.load(fid)
    remove_invalid_json_entries(d)
    return d


def json_parser(text: str) -> dict:
    """Parse a string of json file into a dictionary of json file

    Args:
        text: str: some JSON '{"name" : "John", "age" : "30"}'

    Returns:
        a dictionary representing the JSON in Python
    """
    d = json.loads(text)
    remove_invalid_json_entries(d)
    return d

def dump_json(file: str, d: dict) -> None:
    """Dump a dictionary into a json file

    Args:
        file: str: file name
        d: dict: dictionary to be dumped
    """
    with open(file, 'w') as fp:
        json.dump(d, fp)

def load_skills_from_json(json_file: str) -> dict:
    """Convert json file to skills dictionary"""
    skills_data = json_load_wrapper(json_file)
    # print(skills_data)
    skills = {}
    for skill_name, skill_data in skills_data.items():
        skills[skill_name] = Skill(skill_data)
    return skills

def find_true_symbols(X: dict) -> list:
    """Find the true symbols in the environment state"""
    return [sym for sym, val in X.items() if val]


## ==== Event related functions ==== ##

def wait_for_either_events(events: list) -> bool:
    """Wait for either of the events to be triggered"""
    wait_event = threading.Event()
    def wait_for_event(event: threading.Event) -> None:
        """Wait for the event to be triggered"""
        event.wait()
        wait_event.set()
    
    threads = []
    for event in events:
        thread = threading.Thread(target=wait_for_event, args=(event,))
        threads.append(thread)
        thread.start()
    wait_event.wait()
    # for thread in threads:
    #     thread.join()
    return True

def controllable_inputs_have_changed(X: dict, X_p: dict, controllable_inputs) -> bool:
    """Check if the controllable inputs have changed"""
    for input in controllable_inputs:
        if X[input] != X_p[input]:
            return True
    return False

def inputs_have_changed(X: dict, X_p: dict) -> bool:
    """Check if the inputs have changed"""
    for input in list(X.keys()):
        if X[input] != X_p[input]:
            return True
    return False

def update_controllable_inputs(X, X_p, controllable_symbols): 
    """Update the controllable inputs"""
    X_copy = copy.deepcopy(X)
    for symbol in controllable_symbols:
        X_copy[symbol] = X_p[symbol]
    return X_copy



## ==== Debug, helper functions ==== ##

def print_debug(text: str):
    if DEBUG:
        print(text)

def clear_file(f: str):
    """Clear the file"""
    fid = open(f, "w")
    fid.write('')
    fid.close()

def list2dict_bool(ls: list) -> dict:
    dic = {}
    for name in ls:
        dic[name] = False
    return dic

def state2dict_bool(inputs: list, inp_state: list) -> dict:
    dic = list2dict_bool(inputs)
    for inp in inp_state:
        dic[inp] = True
    return dic

def dict_bool2list(dic: dict) -> list:
    ls = []
    for name, val in dic.items():
        if val:
            ls.append(name)
    return ls

def list2dict_dict(ls: list) -> list:
    dic = {}
    for name in ls:
        dic[name] = {}
    return dic

def make_build_dir(curr_dir: str) -> None:
    os.system('rm -rf %s/build' % curr_dir)
    os.system('mkdir %s/build' % curr_dir)
    os.system('touch %s/build/__init__.py' % curr_dir)

def get_nonrobot_objects(objects_data: dict) -> list:
    """Return list of non-robot objects"""
    return [obj for obj, obj_data in objects_data.items() if obj_data['category'] != 'robot' and obj != 'base' and obj != 'ee']

def varlist2prime(varlist: list) -> list:
    """Return list of primed variables"""
    return [var + "'" for var in varlist]

def varlist2doubleprime(varlist: list) -> list:
    """Return list of double primed variables"""
    return [var + "''" for var in varlist]

## ==== Spec related functions ==== ##

def write_symbols(file_spec: str, heading: str, symbols: list, uncontrollable_symbols: list = []):
    if heading != 'INPUT' and heading != 'OUTPUT':
        raise Exception("Heading should be INPUT or OUTPUT")
    fid = open(file_spec, 'a')
    fid.write(f'[{heading}]\n\n')
    for s in symbols + uncontrollable_symbols:
        fid.write(f'{s}\n')
    fid.write('\n')
    fid.close()

def write_init(file_spec: str, heading: str, symbols_true: list, symbols_false: list) -> None:
    if heading != 'ENV_INIT' and heading != 'SYS_INIT':
        raise Exception("Heading should be either ENV_INIT or SYS_INIT")
    fid = open(file_spec, 'a')
    fid.write(f'[{heading}]\n\n')
    for s in symbols_true:
        fid.write(f'{s}\n')
    for s in symbols_false:
        fid.write(f'!{s}\n')
    fid.write('\n')
    fid.close()

def classify_inputs_by_factors(inputs: dict, num_factors: int) -> list:
    """Classify the inputs by factors
    
    Returns:
        list[list[str]] of size num_factors x num_inputs_in_each_factor   
    """
    result = [[] for _ in range(num_factors)]
    for symbol, info in inputs.items():
        result[info['factor']].append(symbol)
    return result

def write_env_trans(file_spec: str, postconditions: list, inputs: dict, skills: dict, opts: dict) -> None:
    fid = open(file_spec, 'a')
    fid.write('[ENV_TRANS]\n\n')

    fid.write('# Post-condition\n')
    for postcondition in postconditions:
        fid.write(f'{postcondition}\n')
    
    fid.write('\n# Inputs mutual exclusion\n')
    factors = classify_inputs_by_factors(inputs, opts['num_factors'])
    for factor in factors:
        for i in range(len(factor)-1):
            for j in range(i+1, len(factor)):
                fid.write(f'!({factor[i]}\' & {factor[j]}\')')
                fid.write(f'!({factor[i]} & {factor[j]})')
        fid.write('(' + ' | '.join(factor) + ')\n')
        fid.write('(' + '\' | '.join(factor) + '\')\n')

    fid.write('\n# Inactivity without skills')


    fid.close()

def dict_to_formula(sym_dict, prime=False, include_false=True):
    str_list = []
    for sym, val in sym_dict.items():
        if val:
            str_list.append(sym)
        elif include_false:
            str_list.append("!" + sym)
    if prime:
        out = "' & ".join(str_list) + "'"
    else:
        out = " & ".join(str_list)

    return out

def pre_posts_to_env_formula(skill_name, pre_syms_dict, post_syms_dict_list):
    pre = dict_to_formula(pre_syms_dict)
    post_list = []
    for one_post in post_syms_dict_list:
        post_list.append("(" + dict_to_formula(one_post, prime=True) + ")")
    post = "(" + " | ".join(post_list) + ")"

    return "{} & {} -> {}".format(skill_name, pre, post)

def write_section(file_spec, section_name, section_spec):
    fid = open(file_spec, 'a')
    fid.write('[{}]\n\n'.format(section_name))
    fid.write(section_spec)
    fid.write("\n")
    fid.close()

def get_skill_type(skill_data: dict, objects_data) -> str:
    """Given a skill data dictionary, return the skill type in {mobile, manipulation, both}
        Assume that the skill 
    """
    int_states = skill_data['intermediate_states']
    assert len(int_states) > 0, "Intermediate states should not be empty"
    for pre, post_list in int_states:
        for post in post_list:
            changed_type = get_input_type(find_changing_input(pre, post), objects_data)
            return changed_type

def find_changing_input(pre: dict, post: dict) -> str:
    """Find the input that has changed"""
    for sym, val in pre.items():
        if val != post[sym]:
            return sym
    try: 
        raise Exception("Exception: No input has changed in pre and post")
    except Exception as e:
        print(e)
        return None

def get_input_type(input: str, objects_data: dict) -> str:
    """Given an input, return the input type in {mobile, manipulation, both}"""
    if input is not None:
        return objects_data[symbol_to_object(input)]['type']
    else:
        return "mobile" # <- default type is mobile
        # return "manipulation" # <- default type is manipulation

## ==== Skill execution ==== ##
def select_midpoint_in_bounds(bounds: list, n_dims: int) -> np.array:
    """Select a point in the bounds"""
    target_location = np.zeros(n_dims)
    for dim in range(n_dims):
        # target_location[dim] = np.random.uniform(bounds[dim][0], bounds[dim][1])
        target_location[dim] = (bounds[dim][0] + bounds[dim][1]) / 2
    return target_location

def is_final_post(input_state, skill_name, skills_data):
    final_posts_list_of_dicts = skills_data[skill_name]['final_postconditions']
    for final_post_dict in final_posts_list_of_dicts:
        if all(input_state[sym] == val for sym, val in final_post_dict.items()):
            return True
    return False

def bounds2vertices(bounds: list) -> list:
    """Convert bounds to vertices
    
    Args:
        bounds: [[x_min, x_max], [y_min, y_max]]]
    
    Returns:
        [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]]
    """
    return [[bounds[0][0], bounds[1][0]], [bounds[0][0], bounds[1][1]], 
            [bounds[0][1], bounds[1][1]], [bounds[0][1], bounds[1][0]]]

def find_true_robot_mobile_symbols(X: dict, symbols_data: dict, objects_data: dict, locations_data: dict) -> list:
    """Get the true mobile symbols"""
    true_symbols = find_true_symbols(X)
    true_mobile_symbols = []
    for sym in true_symbols:
        if symbols_data[sym] and (
            objects_data[symbols_data[sym]['object']]['type'] == 'mobile' and \
            (objects_data[symbols_data[sym]['object']]['category'] == 'robot' or symbols_data[sym]['object'] == 'base') and \
            locations_data[symbols_data[sym]['location']]['type'] == 'mobile'):
            true_mobile_symbols.append(sym)
    if len(true_mobile_symbols) == 0:
        raise Exception(f"Cannot find true mobile symbols in X = {X}")
    return true_mobile_symbols

def find_true_robot_manipulation_symbols(X: dict, symbols_data: dict, objects_data: dict, locations_data: dict, object: str) -> list:
    """Get the true manipulation objects"""
    true_symbols = find_true_symbols(X)
    true_manipulation_symbols = []
    for sym in true_symbols:
        if symbols_data[sym] and (
            symbols_data[sym]['object'] == object or \
            symbols_data[sym]['object'] == 'ee'):
            true_manipulation_symbols.append(sym)
    return true_manipulation_symbols

def find_manipulation_object_location(symbols: list, symbols_data: dict, object: dict) -> str:
    """Find the location of the manipulation object"""
    for sym in symbols:
        if symbols_data[sym]['object'] == object:
            return symbols_data[sym]['location']
    return None

def find_loc_from_true_manipulation_symbols(obj: str, true_symbols: list) -> str:
    """Find the location from the true manipulation symbols"""
    for sym in true_symbols:
        sym_obj, sym_loc = symbol_to_object_location(sym)

        # If ee is an object
        if sym_obj == obj and sym_loc != 'ee':
            return sym_loc

        # If ee is a location only, not an object
        # if sym_obj == obj:
        #     return sym_loc
    raise Exception(f"Cannot find location of the manipulation object in the true symbols with object {obj} and true symbols {true_symbols}")

def find_loc_from_true_manipulation_symbols_no_ee(obj: str, true_symbols: list) -> str:
    """Find the location from the true manipulation symbols"""
    for sym in true_symbols:
        sym_obj, sym_loc = symbol_to_object_location(sym)

        # If ee is an object
        # if sym_obj == obj and sym_loc != 'ee':
        #     return sym_loc

        # If ee is a location only, not an object
        if sym_obj == obj:
            return sym_loc
    raise Exception(f"Cannot find location of the manipulation object in the true symbols with object {obj} and true symbols {true_symbols}")


class MobileTreeNode:
    def __init__(self, pre_dict: dict, symbols_data: dict, objects_data: dict, locations_data: dict) -> None:
        self.pre_dict = pre_dict
        self.symbol: str = find_true_robot_mobile_symbols(X=pre_dict, 
                                                     symbols_data=symbols_data, 
                                                     objects_data=objects_data, 
                                                     locations_data=locations_data)[0]
        self.location: str = symbols_data[self.symbol]['location']
        self.children: list[MobileTreeNode] = []
    
    def add_child(self, child) -> None:
        self.children.append(child)

class ManipulationTreeNode:
    def __init__(self, object: dict, pre_dict: dict, symbols_data: dict, objects_data: dict, locations_data: dict) -> None:
        self.object = object
        self.pre_dict = pre_dict
        self.symbols: list = find_true_robot_manipulation_symbols(X=pre_dict, 
                                                     symbols_data=symbols_data, 
                                                     objects_data=objects_data, 
                                                     locations_data=locations_data,
                                                     object=object)
        self.location: str = find_manipulation_object_location(symbols=self.symbols, symbols_data=symbols_data, object=self.object)
        self.children: list[MobileTreeNode] = []
    
    def add_child(self, child) -> None:
        self.children.append(child)
    
    def get_height(self):
        if len(self.children) == 0:
            return 1
        else:
            return 1 + max([child.get_height() for child in self.children])

class ManipulationTreeNodeNoEE:
    def __init__(self, object: dict, pre_dict: dict, symbols_data: dict, objects_data: dict, locations_data: dict) -> None:
        self.object = object
        self.pre_dict = pre_dict
        self.symbol: list = find_true_robot_manipulation_symbols(X=pre_dict, 
                                                     symbols_data=symbols_data, 
                                                     objects_data=objects_data, 
                                                     locations_data=locations_data,
                                                     object=object)[0]
        self.location: str = find_manipulation_object_location(symbols=[self.symbol], symbols_data=symbols_data, object=self.object)
        self.children: list[MobileTreeNode] = []
    
    def add_child(self, child) -> None:
        self.children.append(child)
    
    def get_height(self):
        if len(self.children) == 0:
            return 1
        else:
            return 1 + max([child.get_height() for child in self.children])


def print_mobile_tree(root: MobileTreeNode, tab = 0) -> None:
    print("  " * tab, root.location)
    for child in root.children:
        print_mobile_tree(child, tab+1)
    return None

def print_manipulation_tree(root: ManipulationTreeNode, tab = 0) -> None:
    print("  " * tab, root.symbols)
    for child in root.children:
        print_manipulation_tree(child, tab+1)
    return None

def print_manipulation_tree_no_ee(root: ManipulationTreeNodeNoEE, tab = 0) -> None:
    print("  " * tab, root.symbol)
    for child in root.children:
        print_manipulation_tree_no_ee(child, tab+1)
    return None

def filter_new_skills(skills: dict, new_skills: dict) -> dict:
    """Filter the new skills"""
    filtered_skills = {}
    for skill_name, skill in skills.items():
        if skill_name in new_skills:
            filtered_skills[skill_name] = skill
    return filtered_skills

def are_array_close(array1: np.array, array2: np.array, decimal_place = 2) -> bool:
    """Check if two arrays are close"""
    return np.all(np.round(array1, decimal_place) == np.round(array2, decimal_place))

def list_minus(list1: list, list2: list) -> list:
    """Return list1 - list2"""
    return [item for item in list1 if item not in list2]

def assert_exit(condition, err_message):
    try:
        assert condition
    except AssertionError:
        sys.exit(err_message)

def print_set_of_sets(set_of_sets: set) -> None:
    for s in set_of_sets:
        print(s)
    return None

## ==== Tests ==== ##
def test_create_symbols():
    workspace = json_load_wrapper('examples/cupplate/inputs/workspace.json')
    # print(workspace)
    symbols = create_symbols_from_objects_and_locations(workspace['objects'], workspace['locations'], workspace['robot_objects'], workspace['robot_locations'])
    printList(symbols)

def test_find_symbols_by_object_wo_ee():
    objects_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/objects.json')
    locations_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/locations.json')
    # print(workspace)
    symbols = create_symbols_from_objects_and_locations(
            objects_data, 
            locations_data)
    printList(find_symbols_by_object_wo_ee('cup', symbols))

def test_find_symbols_by_objects_type_category():
    objects_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/objects.json')
    locations_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/locations.json')
    # print(workspace)
    symbols = create_symbols_from_objects_and_locations(
            objects_data, 
            locations_data)
    printList(find_symbols_by_objects_type_category('mobile', symbols, objects_data))

def test_find_controllable_symbols():
    objects_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/objects.json')
    locations_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/locations.json')
    symbols = create_symbols_from_objects_and_locations(
            objects_data,
            locations_data)
    printList(find_controllable_symbols(symbols, objects_data))

def test_is_final_post(symbols):
    # Create input state dict
    input_state = ['p_base_x4', 'p_ee_else', 'p_cone_x2', 'p_cup_ee']
    input_state_dict = dict()
    for sym in symbols:
        input_state_dict[sym] = sym in input_state

    # Test function
    print(is_final_post(input_state_dict, 'skill2', skills_data))

def test_find_true_robot_mobile_symbol(symbols, objects, locations):
    # Create input state dict
    input_state = ['p_base_x4', 'p_ee_else', 'p_cone_x2', 'p_cup_ee']
    input_state_dict = dict()
    for sym in symbols:
        input_state_dict[sym] = sym in input_state

    # Test function
    print(find_true_robot_mobile_symbols(input_state_dict, symbols, objects, locations))
    
def test_slugin_realizability(filename):
    # run_slugs(filename)
    check_slugsin_realizability(filename)
    sys.exit(0)

def create_build_monitor():
    """Create build directory"""
    curr_dir = os.getcwd()
    make_build_dir(curr_dir)
    os.system('touch %s/build/generated_monitor.py' % curr_dir)
    with open('%s/build/generated_monitor.py' % curr_dir, 'w') as f:
        f.write('''\
class Monitor:
    def __init__(self) -> None:
        pass
''')
    return None

def print_skill_data(skills_data: dict):
    """Print skill data in a readable format"""
    for _, skill_data in skills_data.items():
        print(f"\nSkill Name: {skill_data['name']}")
        print(f"Primitive Skill: {skill_data['primitive_skill']}")
        print(f"Type: {skill_data['type']}")
        print(f"Goal Type: {skill_data['goal_type']}")
        
        print("\nInitial Preconditions:")
        for precondition in skill_data["initial_preconditions"]:
            print_precondition(precondition)
        
        print("\nIntermediate States:")
        for state in skill_data["intermediate_states"]:
            print_precondition(state[0], indent=4)
            print("[")
            for postcondition in state[1]:
                print_precondition(postcondition, indent=8)
            print("]")
        
        print("\nFinal Postconditions:")
        for postcondition in skill_data["final_postconditions"]:
            print_precondition(postcondition)
        
        print("-" * 40)

def print_precondition(precondition: dict, indent: int = 0):
    """Print a single precondition or postcondition"""
    indent_str = " " * indent
    for key, value in precondition.items():
        print(f"{indent_str}{key}: {value}")

def quaternion_to_angle(q: Quaternion) -> float:
    """Convert a C{geometry_msgs/Quaternion} into a yaw angle.

    Args:
      q: ROS message to be converted

    Returns:
      The equivalent yaw angle (radians)
    """
    _, _, yaw = transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    return yaw

def pose_to_particle(msg: Pose) -> list:
    """Convert a C{geometry_msgs/Pose} into a particle.

    Args:
      msg: ROS message to be converted.

    Returns:
      A particle [x, y, theta]
    """
    x = msg.position.x
    y = msg.position.y
    theta = quaternion_to_angle(msg.orientation)
    return [x, y, theta]

if __name__ == '__main__':
    filename = 'examples/approach2/build/tmp.slugsin'
    test_slugin_realizability(filename)
    objects_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/objects.json')
    locations_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/locations.json')
    # print(workspace)
    symbols = create_symbols_from_objects_and_locations(
            objects_data, 
            locations_data)
    skills_data = json_load_wrapper('examples/cupplate/inputs/pickup_dropoff_cup/abstraction/skills.json')
    # test_find_controllable_symbols()
    # test_create_symbols()
    # test_is_final_post(symbols)
    test_find_true_robot_mobile_symbol(symbols, objects_data, locations_data)