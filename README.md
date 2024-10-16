# env_repair

## Overview

This repository provides the code associated with the paper [Automated Robot Recovery from Assumption Violations of High-level Specifications](http://arxiv.org/abs/2407.00562).

This project provides the first framework that enables robots to automatically recover from environment assumption violations of high-level specifications during task execution. We create a monitor to detect assumption violations during the task execution, relax the violated assumptions to admit observed environment behaviors, and obtain new robot skills for task completion. 

We demonstrate the framework in a factory-like setting: [video](https://youtu.be/OTUEyqQfQQs)

This work is presented at [CASE 2024](https://2024.ieeecase.org/): [video](https://youtu.be/O7PITKfLQ_k)


## Setup

This code requires python > 3 and has been tested on python 3.8.10 and Ubuntu 20.04

### Required repositories

- This [fork](https://github.com/MartinMeng008/synthesis_based_repair) of synthesis_based_repair and its dependencies should be installed and added to the PATH:
  - [BDD package](https://github.com/tulip-control/dd).

  ```shell
  git clone -b env_relax_repair https://github.com/MartinMeng008/synthesis_based_repair.git
  export PATH=[parent_dir_of_synthesis_based_repair]/synthesis_based_repair/synthesis_based_repair:$PATH
  git clone https://github.com/tulip-control/dd.git
  cd dd
  pip install cython
  python setup.py install --fetch --cudd
  ```

- This [fork](https://github.com/MartinMeng008/slugs) of slugs. The path to the src and parser directory should be added to PATH.

  ```shell
  git clone https://github.com/MartinMeng008/slugs.git
  export PATH=[parent_dir_of_slugs]/slugs/src:$PATH
  export PYTHONPATH=[parent_dir_of_slugs]/slugs/tools/StructuredSlugsParser:$PYTHONPATH
  ```
- ROS. This code has been tested on ROS Noetic. Follow the installation [instructions](http://wiki.ros.org/ROS/Installation).

- Our demonstration runs on a Hello Robot Stretch. Install their ROS [package](https://github.com/hello-robot/stretch_ros):

   ```shell
  cd [parent_dir_of_catkin_ws]/catkin_ws/src
  git clone https://github.com/hello-robot/stretch_ros.git
   ```

### Required python packages
- argparse
- matplotlib
- networkx
- numpy
- shapley
- z3-solver

```shell
pip install argparse matplotlib networkx numpy shapley z3-solver
```

### Installation
Clone the repository to the catkin_ws directory and build the workspace:

```shell
cd [parent_dir_of_catkin_ws]/catkin_ws/src
git clone https://github.com/MartinMeng008/env_repair.git
cd ..
catkin build
source devel/setup.bash
```

## Usage

### Inputs

#### Workspace
The user should provide a `JSON` file that defines each region in the workspace as a rectangular box, e.g.:

```json
{
    "t": {
        "dims": [0, 1, 2],
        "name": "t",
        "index": 0,
        "bounds": [[1.6, 1.98], [-0.81, 0.7], [0.7, 0.87]],
        "type": "manipulation",
        "is_robot_location": false
    }
}
```

where `dims` defines the dimensions, `bounds` defines the low and up bounds on each dimension, `type` defines whether the region is a manipulation region (e.g. on a table) or mobile region (e.g. on the floor), and `is_robot_location` determines whether the region is a part of the robot, such as the robot's end effector.

In the demonstration, we define the regions in [`locations.json`](scripts/inputs/three_violations/abstraction/locations.json).

#### Objects
The user should provide a `JSON` file that defines the objects in the workspace, e.g.:

```json
{
    "cup": {
        "index": 0,
        "name": "cup",
        "type": "manipulation",
        "category": "controllable"
    }
}
```
where `type` indicates whether the object is located in the mobile regions only, such as the robot base, or can also operate in the manipulation regions.

In the demonstration, we define the objects in [`objects.json`](scripts/inputs/three_violations/abstraction/objects.json).

#### Propositions
Our framework creates atomic propositions by taking a Cartesian product of the locations and the objects. Each proposition `p_obj_loc` indicates that the object `obj` is in the region `loc`, e.g. `p_base_x0` indicates that the robot base is in the region `x0`. 

We also allow the user to provide a set of user inputs that the user controls via the keyboard through a `JSON` file. In the demonstration, the user controls whether the cup is empty or full, as defined in [`user_inputs.json`](scripts/inputs/three_violations/abstraction/user_inputs.json).

To generate the input propositions, from `[parent_dir_of_env_repair]/env_repair/scripts`, run:

```shell
python grounding.py -f inputs/three_violations/files.json
```

#### Skills
The user should provide an abstraction of robot skills in a `JSON` file, such as 

```json
{
    "skill0": {
        "name": "skill0", 
        "primitive_skill": "movebase", 
        "type": "mobile", 
        "goal_type": "region", 
        "initial_preconditions": [{"p_base_x0": true, "p_base_x1": false, "p_base_x2": false, "p_base_x3": false, "p_base_x4": false, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}], 
        "intermediate_states": [[{"p_base_x0": true, "p_base_x1": false, "p_base_x2": false, "p_base_x3": false, "p_base_x4": false, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}, [{"p_base_x0": false, "p_base_x1": false, "p_base_x2": true, "p_base_x3": false, "p_base_x4": false, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}]], [{"p_base_x0": false, "p_base_x1": false, "p_base_x2": true, "p_base_x3": false, "p_base_x4": false, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}, [{"p_base_x0": false, "p_base_x1": false, "p_base_x2": false, "p_base_x3": false, "p_base_x4": true, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}]]], 
        "final_postconditions": [{"p_base_x0": false, "p_base_x1": false, "p_base_x2": false, "p_base_x3": false, "p_base_x4": true, "p_cup_k0": false, "p_cup_k1": false, "p_cup_k2": false, "p_cup_k3": false, "p_cup_t": false, "p_cup_ee": true}]
    }
}
```

where `initial_preconditions` is a list of input states indicating where the skill can initiate, `final_postconditions` is a list of input states indicating where the skill results, `intermediate_states` is a list of state-list pairs in which the state is the precondition of the intermediate transition and the list of states is the postconditions of the transition. Other fields, `primitive_skill`, `type`, and `goal_type`, indicate which low-level controller implements the skill.

In the demonstration, we define the skill abstraction in [`skills.json`](scripts/inputs/three_violations/abstraction/skills.json). 

We also provide a helper function [skill_generation.py](scripts/skill_generation.py) that takes in a state and a set of transitions, and generates the corresponding skill abstraction.

We provide low-level [controllers](src/controller/main_controller.py) that implement the skills for our demonstration, but the user can use different low-level controllers, such as a learned policy.

#### Tasks 
The user should describe the robot task in a `JSON` file. In the demonstration, we define the task in [`spec.json`](scripts/inputs/three_violations/spec.json) as shown below:

```json
{
    "sys_init_true": [],
    "env_live": "",
    "sys_live": "empty -> p_cup_k2\n!empty -> p_cup_t\n",
    "user_sys": ["p_cup_ee & empty & !p_base_x0 -> !p_base_x0'", 
                "p_cup_ee & !empty & !p_base_x4-> !p_base_x4'",
                "p_block_k1 & p_cup_k2-> !p_cup_ee'"],
    "user_env": ["empty & !empty' -> p_cup_k2",
                "!empty & empty' -> p_cup_t",
                 "p_cone_x0 -> p_cone_x0'",
                 "p_cone_x1 -> p_cone_x1'",
                 "p_cone_x2 -> p_cone_x2'",
                 "p_cone_x3 -> p_cone_x3'",
                 "p_cone_x4 -> p_cone_x4'",
                 "p_plate_k0 -> p_plate_k0'",
                 "p_plate_k1 -> p_plate_k1'",
                 "p_plate_k2 -> p_plate_k2'",
                 "p_plate_k3 -> p_plate_k3'"]
}
```
where 
- `sys_init_true` states the robot skills that operate initially
- `env_live` states the environment liveness assumptions
- `sys_live` specifies the system goals that should be satisfied repeatedly
- `user_sys` specifies user-defined safety constraints that the robot should obey
- `user_env` states the user-defined environment safety assumptions

To generate the specification, after [generating input propositions](https://github.com/MartinMeng008/env_repair/main/README.md#propositions), from `[parent_dir_of_env_repair]/env_repair/scripts`, run:

```shell
python spec_writer.py -f inputs/three_violations/files.json
```  

#### Options and Files
The user should provide options for [synthesis_based_repair](https://github.com/apacheck/synthesis_based_repair) in a `JSON` file, such as [`opts.json`](scripts/inputs/three_violations/opts.json) in our demonstration.

The user should also provide a `JSON` file consisting of all the informations of the input files, such as [`files.json`](scripts/inputs/three_violations/files.json) in our demonstration. 

### Demonstration
To run our demonstration, from `[parent_dir_of_env_repair]/env_repair/scripts`, run:

```shell
python main.py -f inputs/three_violations/files.json
```

### New Example
To create a new example, first create a new input directory:

```shell
mkdir [parent_dir_of_env_repair]/env_repair/scripts/inputs/[new_example]
```

Then in the new directory `[new_example]`, create the following `JSON` files as described [above](https://github.com/MartinMeng008/env_repair/blob/main/README.md#inputs):
- `abstraction/locations.json`
- `abstraction/objects.json`
- `abstraction/skills.json`
- `abstraction/user_inputs.json`
- `spec.json`
- `opts.json`
- `files.json`

To run the new example, from `[parent_dir_of_env_repair]/env_repair/scripts`, run:

```shell
python main.py -f inputs/[new_example]/files.json
```

## Reference
If you use this repository or find this project interesting, please kindly cite:

```bib
@article{meng2024automated,
  title={Automated Robot Recovery from Assumption Violations of High-Level Specifications},
  author={Meng, Qian and Kress-Gazit, Hadas},
  journal={arXiv preprint arXiv:2407.00562},
  year={2024}
}
```
