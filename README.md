# env_relax_repair

## Overview

This repository provides the code associated with the paper [Automated Robot Recovery from Assumption Violations of High-level Specifications]()

## Setup

This code requires python > 3 and has been tested on python 3.8.10 and Ubuntu 20.04

### Required repositories

- This [fork](https://github.com/MartinMeng008/synthesis_based_repair) of synthesis_based_repair and its dependencies:
  - dd package to handle BDDs (https://github.com/tulip-control/dd)
- This [fork](https://github.com/MartinMeng008/slugs) of slugs. The path to the src directory should be added to PATH.
  ```
  git clone https://github.com/MartinMeng008/slugs.git
  export PATH=[parent_dir_of_slugs]/slugs/src:$PATH
  ```
- This [repo](https://github.com/MartinMeng008/cs4750) containing helper functions should be installed in the catkin_ws directory of ROS:
  ```
  cd [parent_dir_of_catkin]/catkin_ws/src
  git clone https://github.com/MartinMeng008/cs4750.git
  cd ..
  catkin build
  ``` 

### Other installations

- shapley
- z3

## Usage
Todo

## Reference
Todo
